//! The public runtime surface: `Session::builder() … build()`,
//! `session.start_episode(task)` (blocks through reset), `episode.gate(...)`.
//!
//! Threading (the design invariant made code): the session owns named
//! core threads — reducer, verb dispatch, bypass pump, media intake,
//! tripwire evaluator — and the control-plane client thread. Nothing
//! executes on the caller's thread except `Episode::gate()`.

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::mpsc::Sender;
use std::thread::JoinHandle;

use waddle_controlplane::{ClientConfig, ControlPlaneClient, ControlTransport};
use waddle_fsm::{Phase, SessionConfig, SessionEvent, WindowSpec};
use waddle_gate::gate::{Gate, GateOutput, GateShared};
use waddle_gate::jitter::TimedAction;
use waddle_gate::plan::GatePlan;
use waddle_gate::record::GateRecord;
use waddle_ingest::{LatestSlot, SessionClock};
use waddle_media::MediaPlane;
use waddle_tripwire::{
    Evaluator, ObsSnapshot, ShutdownToken, Tripwire, TripwireFire, TripwireSink,
};
use waddle_types::pb::v0 as pb;
use waddle_types::time::Clock;
use waddle_types::{
    ActorKind, CellId, EpisodeId, HandoffPolicy, LeaseEnforcement, MonoNs, ResetVerificationMode,
    RobotDescription, RobotId, SessionId, TerminalOutcome,
};

use crate::RuntimeError;
use crate::ack::{ACKS_FLAG, Injected};
use crate::media_uplink::{self, CameraUplink, FrameData};
use crate::mirror::Mirror;
use crate::pumps;
use crate::reducer::Reducer;
use crate::verbs::{ControlRegistry, VerbDispatch, VerbOutcome};

/// How resets run until the closed reset planner is wired: a callable
/// returning (ok, verified). The default reports ok+verified — honest only
/// for scenes reset by hand between episodes; integrations override it.
pub type ResetHook = Arc<dyn Fn(&str) -> (bool, Option<bool>) + Send + Sync>;

/// How a single reset phase (pre or post) is driven. One or the other, never
/// both, per phase — configured on the [`SessionBuilder`] (session default)
/// and optionally overridden per episode via [`EpisodeOptions`].
#[derive(Clone)]
pub enum ResetSpec {
    /// Run a [`ResetHook`]: for pre-reset, inline on `start_episode`'s own
    /// caller thread (except reducer-opened retake successors, which the
    /// reset pump services); for post-reset, always on the reset pump's
    /// thread (`waddle-reset-hooks`), since nothing blocks on it. Hooks must
    /// therefore be `Send + Sync` (the type already requires it) and must
    /// return — session shutdown joins the pump thread.
    Hook(ResetHook),
    /// A plane-directed remote actor performs this reset through a window
    /// (flag `waddle.v0.reset.remote`, FSM.md rows E19–E22): the actor
    /// expected to engage it, the prompt shown to them, and the window's
    /// deadline. The runtime injects no hook result for this phase at all —
    /// the FSM's window machinery owns the whole reset, including its
    /// timeout.
    Remote {
        actor: ActorKind,
        prompt: String,
        timeout_ns: i64,
    },
}

impl ResetSpec {
    /// The remote window this spec declares, if any (`None` for `Hook` —
    /// that phase has no window; the hook runs inline instead).
    pub(crate) fn window(&self) -> Option<WindowSpec> {
        match self {
            Self::Hook(_) => None,
            Self::Remote {
                actor,
                prompt,
                timeout_ns,
            } => Some(WindowSpec {
                expected: *actor,
                prompt: prompt.clone(),
                timeout_ns: *timeout_ns,
            }),
        }
    }
}

impl std::fmt::Debug for ResetSpec {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Hook(_) => f.debug_tuple("Hook").field(&"<fn>").finish(),
            Self::Remote {
                actor,
                prompt,
                timeout_ns,
            } => f
                .debug_struct("Remote")
                .field("actor", actor)
                .field("prompt", prompt)
                .field("timeout_ns", timeout_ns)
                .finish(),
        }
    }
}

/// Per-episode override of the session's declared reset specs
/// ([`SessionBuilder::pre_reset`]/[`SessionBuilder::post_reset`]), passed to
/// [`Session::start_episode_with`]. For each field: outer `None` inherits the
/// session default; inner `None` disables that phase for this episode only
/// (the trivial "already reset by hand" default applies: `(true,
/// Some(true))`, exactly as if the session declared no spec at all).
///
/// Overrides can only narrow what the session already declared, never widen
/// it: the `waddle.v0.reset.remote` feature flag is negotiated once, at
/// build time, from the session-level config alone (see
/// `SessionBuilder::build`). A per-episode `ResetSpec::Remote` override
/// requires the session to have already declared a `Remote` spec for some
/// phase at build time — otherwise the plane was never told to expect a
/// remote-reset negotiation for this session at all.
///
/// An override belongs to the episode it was passed for only: a retake
/// successor (reducer-opened, never routed through this call) always
/// inherits the SESSION-level specs, never the predecessor's override.
#[derive(Clone, Debug, Default)]
pub struct EpisodeOptions {
    pub pre_reset: Option<Option<ResetSpec>>,
    pub post_reset: Option<Option<ResetSpec>>,
}

/// Hand-off slot for the gate-record consumer: born on the caller thread
/// (`Gate::new` inside `start_episode`), consumed by the reducer thread,
/// which drains it onto the episode's MCAP. A slot rather than a channel:
/// `SessionEvent` is pure `waddle-fsm` and cannot carry an `rtrb::Consumer`,
/// and mpsc has no select to multiplex a second channel into the reducer
/// loop.
pub(crate) type RecordSlot = Arc<parking_lot::Mutex<Option<rtrb::Consumer<GateRecord>>>>;

/// The current task, written by `start_episode` before the open event so the
/// reducer stamps it into the episode's records.
pub(crate) type TaskSlot = Arc<parking_lot::Mutex<String>>;

/// The episode `start_episode_with` is currently running the pre-reset phase
/// for, inline on the caller thread — recorded before `EpisodeOpen` is
/// injected, cleared once the call returns (success or failure). Set for
/// every inline pre-reset path (`Hook` and the no-spec default), never for
/// `Remote` (nothing runs inline there). Consulted by the reset pump so it
/// never double-services an episode that call is already driving.
pub(crate) type ResetOwnerSlot = Arc<parking_lot::Mutex<Option<EpisodeId>>>;

/// The resolved (inherit/disable applied) reset specs of the episode most
/// recently opened by `start_episode_with`, written before its `EpisodeOpen`
/// is injected — so by the time the mirror can show that id at all, the
/// reset pump can resolve the *effective* spec for it (per-episode overrides
/// included) instead of guessing from session defaults. Reducer-opened
/// retake successors never get an entry here; the pump falls back to the
/// session-level config for them.
#[derive(Clone)]
pub(crate) struct EpisodeResetSpecs {
    pub id: EpisodeId,
    pub pre: Option<ResetSpec>,
    pub post: Option<ResetSpec>,
}

pub(crate) type ResetSpecSlot = Arc<parking_lot::Mutex<Option<EpisodeResetSpecs>>>;

/// The intervention stream's single write end, Mutex-shared rather than
/// owned outright: `rtrb` is strictly SPSC, but two producers need it — the
/// media intake thread (teleop poses) and the plane pump's
/// `forward_server_msg` (reset-window agent chunks, flag
/// `waddle.v0.reset.remote`). Never touched from the caller thread or the
/// gate fast path; both writers already take other locks.
pub(crate) type StreamProducer = Arc<parking_lot::Mutex<rtrb::Producer<TimedAction>>>;

/// The gate record stream's latest observation: published by the
/// reducer on every ring-drained record that carries one (regardless of
/// whether local MCAP recording is even on), read by the tripwire
/// evaluator's `ObsSource`. Wait-free (`LatestSlot`) — the write lives on
/// the reducer thread, never `Gate::gate()`'s fast path.
pub(crate) type ObsSlot = Arc<LatestSlot<ObsSnapshot>>;

/// A frame-tagged end-effector pose (see [`ProprioReport::ee_pose`]).
/// `descriptors.proto`'s `Pose` is always frame-tagged ("an empty frame_id
/// is a validation error, never a default: untagged geometry is how
/// misaligned data corrupts a corpus silently") — so, unlike a bare
/// `[f64; 7]`, this can only be constructed with a non-empty frame
/// ([`Self::new`]). This widens the brief's literal `Option<[f64; 7]>`
/// shape by one required argument for exactly this reason.
#[derive(Clone, Debug, PartialEq)]
pub struct EePose {
    /// xyz position, expressed in `frame_id`.
    pub position: [f64; 3],
    /// wxyz unit quaternion (w first — the protocol's pinned convention).
    pub orientation: [f64; 4],
    pub frame_id: waddle_types::FrameId,
}

impl EePose {
    /// `frame_id` must be non-empty (see the struct rustdoc); an empty one
    /// is [`waddle_types::TypesError::EmptyFrame`], the same error
    /// `waddle-types` already raises for every other untagged `Pose` in
    /// this workspace.
    pub fn new(
        position: [f64; 3],
        orientation: [f64; 4],
        frame_id: impl AsRef<str>,
    ) -> Result<Self, waddle_types::TypesError> {
        Ok(Self {
            position,
            orientation,
            frame_id: waddle_types::FrameId::new(frame_id)?,
        })
    }

    pub(crate) fn to_pb(&self) -> pb::Pose {
        pb::Pose {
            position: Some(pb::Vec3 {
                x: self.position[0],
                y: self.position[1],
                z: self.position[2],
            }),
            rotation: Some(pb::Quat {
                w: self.orientation[0],
                x: self.orientation[1],
                y: self.orientation[2],
                z: self.orientation[3],
            }),
            frame_id: self.frame_id.as_str().to_owned(),
        }
    }
}

/// One reported proprioceptive sample: [`Session::report_proprio`]'s
/// payload, merged with the reducer's own `joint_pos` (from the caller's
/// `gate(obs=...)` stream) into a richer `ProprioSample` than the bare
/// `joint_pos` every gate tick already records. Every field PATCHES the
/// reducer's latest known sample — `None` leaves the previously reported
/// value in place (there is no way to clear a previously-reported field in
/// v0), so a caller can e.g. report `gripper` on every tick without
/// re-supplying `ee_pose` each time.
#[derive(Clone, Debug, Default)]
pub struct ProprioReport {
    pub joint_vel: Option<Vec<f64>>,
    pub ee_pose: Option<EePose>,
    pub gripper: Option<f64>,
}

pub struct SessionBuilder {
    project: String,
    robot: Option<pb::RobotDescription>,
    control: ControlRegistry,
    recording_dir: Option<PathBuf>,
    transport: Option<Arc<dyn ControlTransport>>,
    media: Option<Arc<dyn MediaPlane>>,
    tripwires: Vec<Tripwire>,
    handoff: HandoffPolicy,
    enforcement: LeaseEnforcement,
    pre_reset: Option<ResetSpec>,
    post_reset: Option<ResetSpec>,
    verification_mode: ResetVerificationMode,
    clutch_actor: ActorKind,
    clutch_source: String,
}

impl std::fmt::Debug for SessionBuilder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SessionBuilder")
            .field("project", &self.project)
            .finish_non_exhaustive()
    }
}

impl SessionBuilder {
    #[must_use]
    pub fn new(project: impl Into<String>) -> Self {
        Self {
            project: project.into(),
            robot: None,
            control: ControlRegistry::default(),
            recording_dir: None,
            transport: None,
            media: None,
            tripwires: Vec::new(),
            handoff: HandoffPolicy::HoldFirst,
            enforcement: LeaseEnforcement::Advisory,
            pre_reset: None,
            post_reset: None,
            verification_mode: ResetVerificationMode::Blocking,
            // The runtime's honest default (N17): a clutch edge on the media
            // plane is our teleoperators' takeover path. waddle-fsm's own
            // default stays SiteOperator/"custom" for fixture stability —
            // this is the layer that owns the real-world identity.
            clutch_actor: ActorKind::Teleoperator,
            clutch_source: "teleop-clutch".to_owned(),
        }
    }

    #[must_use]
    pub fn robot(mut self, robot: pb::RobotDescription) -> Self {
        self.robot = Some(robot);
        self
    }

    #[must_use]
    pub fn control(mut self, control: ControlRegistry) -> Self {
        self.control = control;
        self
    }

    /// Local recording mode: sidecars + MCAP episodes under this directory.
    #[must_use]
    pub fn recording_dir(mut self, dir: impl Into<PathBuf>) -> Self {
        self.recording_dir = Some(dir.into());
        self
    }

    #[must_use]
    pub fn transport(mut self, transport: Arc<dyn ControlTransport>) -> Self {
        self.transport = Some(transport);
        self
    }

    #[must_use]
    pub fn media(mut self, media: Arc<dyn MediaPlane>) -> Self {
        self.media = Some(media);
        self
    }

    #[must_use]
    pub fn tripwires(mut self, wires: Vec<Tripwire>) -> Self {
        self.tripwires = wires;
        self
    }

    #[must_use]
    pub fn handoff(mut self, handoff: HandoffPolicy) -> Self {
        self.handoff = handoff;
        self
    }

    #[must_use]
    pub fn lease_enforcement(mut self, enforcement: LeaseEnforcement) -> Self {
        self.enforcement = enforcement;
        self
    }

    /// Configure the pre-reset phase: runs before RESETTING → READY, before
    /// the caller's `start_episode`/`start_episode_with` call returns.
    #[must_use]
    pub fn pre_reset(mut self, spec: ResetSpec) -> Self {
        self.pre_reset = Some(spec);
        self
    }

    /// Configure the post-reset phase (flag `waddle.v0.reset.phases`): runs
    /// INSIDE the finishing episode, after the terminal outcome is pinned
    /// and before TERMINAL (FSM.md row E14). Declaring this at all — with
    /// either variant of [`ResetSpec`] — is what makes an episode detour
    /// through `Phase::PostReset` on completion instead of terminating
    /// directly.
    #[must_use]
    pub fn post_reset(mut self, spec: ResetSpec) -> Self {
        self.post_reset = Some(spec);
        self
    }

    /// How reset verification gates entry to READY (N12): `Blocking` (the
    /// default) holds RESETTING until verification passes; `OptimisticAsync`
    /// enters READY immediately and permanently flags a late verification
    /// failure instead of blocking on it.
    #[must_use]
    pub fn verification_mode(mut self, mode: ResetVerificationMode) -> Self {
        self.verification_mode = mode;
        self
    }

    /// Deprecated alias for `pre_reset(ResetSpec::Hook(hook))`.
    #[deprecated(note = "use `pre_reset(ResetSpec::Hook(hook))` instead")]
    #[must_use]
    pub fn reset_hook(self, hook: ResetHook) -> Self {
        self.pre_reset(ResetSpec::Hook(hook))
    }

    /// Override the actor/source recorded for clutch-initiated
    /// (self-initiated) claims — the leader-arm/console-clutch takeover
    /// path. Defaults to `ActorKind::Teleoperator` / "teleop-clutch".
    #[must_use]
    pub fn clutch_identity(mut self, actor: ActorKind, source: impl Into<String>) -> Self {
        self.clutch_actor = actor;
        self.clutch_source = source.into();
        self
    }

    /// Build the session, or a build-time error.
    ///
    /// Validates the verb registry against the configuration that can
    /// actually dispatch each verb, so a missing callable fails loudly here
    /// instead of silently the first time something requests it:
    ///
    /// - `hold`: required whenever the *effective* handoff policy is
    ///   [`HandoffPolicy::HoldFirst`] **and** the session has a live engage
    ///   path — every engage issues `Verb::Hold` before the intervenor's
    ///   first action lands. A live engage path is a wired media plane (the
    ///   teleoperator's clutch) **or** `hold`/`send` registered in
    ///   `Control` directly: [`grant_and_engage`] is a real, exported,
    ///   always-live function with zero dependency on `self.media` — a
    ///   caller that registers `hold`/`send` and drives engage through it
    ///   directly (its own doc comment names this "local intervention
    ///   sources", plural) has exactly the same exposure as one wired to a
    ///   media plane, so this check cannot key on `self.media` alone.
    ///   "Effective" matters too: `waddle_fsm::begin_engage` silently
    ///   degrades a declared [`HandoffPolicy::Immediate`] to `HoldFirst` on
    ///   the very first engage whenever the robot's action space contains a
    ///   delta component (FSM.md §5 — delta spaces refuse mid-chunk splice
    ///   entry). This check mirrors that same degrade so a declared-IMMEDIATE
    ///   session over a delta space cannot build clean and then stall at the
    ///   first engage exactly like the undegraded case. Only a session that
    ///   wires no media plane **and** registers neither `hold` nor `send`
    ///   (the fully descriptors-only / minimal-local case, including the
    ///   PyO3 shim's all-None-verbs `create_session`) has no build-time
    ///   visible engage path, so it alone stays buildable without `hold`
    ///   even under the default `HoldFirst` policy — see the safety note on
    ///   [`grant_and_engage`] for why that residual shape is still the
    ///   caller's own responsibility if they invoke it directly.
    /// - `send`: required under that same live-engage-path condition,
    ///   independent of handoff policy — the bypass pump can drive
    ///   `Verb::Send` directly once a claimed loop stalls (see
    ///   `pumps::spawn_bypass_pump`), and reaching "claimed" needs nothing
    ///   more than the same `grant_and_engage` call.
    /// - `estop`: never build-fatal (an integrator legitimately without
    ///   hardware estop must still be able to build a session) but the
    ///   degradation is recorded on [`crate::Status::estop_unregistered`]
    ///   so it stays observable rather than surfacing only as a
    ///   `VerbError::NotRegistered` the first time something requests one.
    pub fn build(self) -> Result<Session, RuntimeError> {
        let robot_pb = self.robot.ok_or(RuntimeError::MissingRobot)?;
        let robot = RobotDescription::try_from(&robot_pb)?;

        // Cameras: `declared_cameras` backs `publish_frame`'s
        // unknown-camera + declared-resolution checks regardless of whether
        // a media plane is wired at all; `camera_uplinks` (one per declared
        // camera, only when a media plane IS wired) is what actually
        // publishes — an unwired declaration is a cheap no-op, never an
        // error. Resolving each camera's uplink policy here (not lazily on
        // first frame) means an unsupported encoding fails loudly at build
        // time instead of silently dropping every frame later.
        let mut declared_cameras: HashMap<String, (u32, u32)> = HashMap::new();
        for cam in &robot_pb.cameras {
            declared_cameras.insert(cam.name.clone(), (cam.width, cam.height));
        }
        let media_for_cameras = self.media.clone();
        let mut camera_uplinks: HashMap<String, Arc<CameraUplink>> = HashMap::new();
        if media_for_cameras.is_some() {
            for cam in &robot_pb.cameras {
                let uplink = media_uplink::build_camera_uplink(cam)?;
                camera_uplinks.insert(cam.name.clone(), Arc::new(uplink));
            }
        }

        // A live engage path: a wired media plane, or `hold`/`send`
        // registered directly. `grant_and_engage` doesn't consult
        // `self.media` at all, so registering either verb without media is
        // just as live a path into the same HOLD_FIRST engage handshake as
        // wiring media is (see the `build` rustdoc above and the safety
        // note on `grant_and_engage`).
        let intervention_wired =
            self.media.is_some() || self.control.hold.is_some() || self.control.send.is_some();
        if intervention_wired {
            // Same degrade `begin_engage` applies at engage time (FSM.md
            // §5): under a delta action space, a declared IMMEDIATE becomes
            // HOLD_FIRST for the first engage. Gate the `hold` requirement on
            // that effective policy, not the raw declared enum variant.
            let effective_handoff = match self.handoff {
                HandoffPolicy::Immediate { .. } if robot.action_space.contains_delta() => {
                    HandoffPolicy::HoldFirst
                }
                other => other,
            };
            if matches!(effective_handoff, HandoffPolicy::HoldFirst) && self.control.hold.is_none()
            {
                return Err(RuntimeError::MissingVerb {
                    verb: "hold",
                    required_by: "handoff HOLD_FIRST",
                    remedy: "choose a different handoff policy",
                });
            }
            if self.control.send.is_none() {
                return Err(RuntimeError::MissingVerb {
                    verb: "send",
                    required_by: "a wired media plane or a registered `hold` verb \
                                  (bypass/intervention dispatch)",
                    remedy: "remove that wiring",
                });
            }
        }
        let estop_unregistered = self.control.estop.is_none();

        let clock = SessionClock::capture();
        let session_id = SessionId::new(uuid::Uuid::new_v4().to_string());

        let mut cfg = SessionConfig::minimal("customer-loop", self.handoff, self.enforcement);
        cfg.session_id = session_id.clone();
        cfg.robot_id = if robot.robot_id.is_empty() {
            RobotId::new(&robot.name)
        } else {
            robot.robot_id.clone()
        };
        cfg.cell_id = if robot.cell_id.is_empty() {
            CellId::new("cell")
        } else {
            robot.cell_id.clone()
        };
        cfg.grants = robot.grants.clone();
        cfg.space_contains_delta = robot.action_space.contains_delta();
        cfg.clutch_actor = self.clutch_actor;
        cfg.clutch_source = self.clutch_source.clone();
        let dims = robot.action_space.dims();
        let gripper_spec = robot.action_space.gripper.clone();

        let (gate_shared, stream_tx) = GateShared::new(
            GatePlan::passthrough(MonoNs(0)),
            1024,
            20_000_000,
            robot.action_space.chunking.replan,
        );
        // Shared (see `StreamProducer`): media intake and the plane pump's
        // reset-window agent-chunk arm both write into it.
        let stream_tx: StreamProducer = Arc::new(parking_lot::Mutex::new(stream_tx));

        let (outcome_tx, outcome_rx) = std::sync::mpsc::channel::<VerbOutcome>();
        let verbs = Arc::new(VerbDispatch::spawn(self.control, clock.clone(), outcome_tx));

        // Feature-flag declaration (VERSIONING.md registry): `waddle.v0.core`
        // is the always-required baseline; `waddle.v0.reset` always rides
        // alongside it since every episode runs some reset pipeline (even
        // the trivial "already reset by hand" default). `.phases`/`.remote`
        // are declared from the session-level config only — per-episode
        // overrides can narrow (disable, or swap a Remote default for a
        // Hook) but a `ResetSpec::Remote` override requires the session to
        // have already declared a Remote spec for that phase (or the other
        // one) at build time, since flags are negotiated once, at Register,
        // before any episode opens.
        // Directive acks (`waddle.v0.plane.acks`) are always declared when a
        // transport is configured — safe unconditionally, since emission
        // additionally requires the plane to accept the flag AND the
        // directive to carry a `directive_id` (see `crate::ack`).
        let mut feature_flags = vec![
            "waddle.v0.core".to_owned(),
            "waddle.v0.reset".to_owned(),
            ACKS_FLAG.to_owned(),
        ];
        if self.post_reset.is_some() {
            feature_flags.push("waddle.v0.reset.phases".to_owned());
        }
        if matches!(self.pre_reset, Some(ResetSpec::Remote { .. }))
            || matches!(self.post_reset, Some(ResetSpec::Remote { .. }))
        {
            feature_flags.push("waddle.v0.reset.remote".to_owned());
        }

        let plane = self.transport.map(|t| {
            let register = pb::RegisterRequest {
                project: self.project.clone(),
                client: Some(pb::ClientInfo {
                    implementation: "waddle-core".into(),
                    version: env!("CARGO_PKG_VERSION").into(),
                    platform: std::env::consts::OS.into(),
                }),
                robot: Some(robot_pb.clone()),
                clock_anchor: Some(clock.anchor().to_pb()),
                feature_flags,
                session_nonce: session_id.to_string(),
            };
            Arc::new(ControlPlaneClient::spawn(t, ClientConfig::new(register)))
        });

        let mirror = Mirror::new();
        if estop_unregistered {
            // estop unregistered: dispatch degrades to NotRegistered at
            // estop time (never build-fatal — see the `build` rustdoc) —
            // recorded here, before any thread starts, so it is observable
            // from the first `session.status()` read onward.
            mirror.update(|s| s.estop_unregistered = true);
        }
        let (inject_tx, inject_rx) = std::sync::mpsc::channel::<Injected>();
        // `Session::report_proprio`'s side channel into the
        // reducer — deliberately NOT the `Injected`/`SessionEvent` funnel
        // (this carries no FSM guard, so it never touches `step()`; see
        // `Reducer::drain_proprio_reports`).
        let (proprio_tx, proprio_rx) = std::sync::mpsc::channel::<ProprioReport>();
        let record_slot: RecordSlot = Arc::new(parking_lot::Mutex::new(None));
        let task_slot: TaskSlot = Arc::new(parking_lot::Mutex::new(String::new()));
        // Tripwire ObsSource wiring: published by the reducer from
        // the gate record stream, read by the tripwire evaluator below.
        let obs_slot: ObsSlot = Arc::new(LatestSlot::new());

        // The reducer thread. `self.post_reset` is the session-level default
        // a reducer-opened retake successor inherits (`Effect::OpenSuccessor`
        // has no per-episode override slot to consult — see its rustdoc).
        let reducer = Reducer::new(
            cfg,
            clock.clone(),
            gate_shared.clone(),
            verbs.clone(),
            mirror.clone(),
            plane.clone(),
            self.recording_dir,
            self.project.clone(),
            robot_digest(&robot_pb),
            robot.action_space.clone(),
            record_slot.clone(),
            task_slot.clone(),
            self.post_reset.clone(),
            obs_slot.clone(),
            proprio_rx,
        );
        let reducer_tx = inject_tx.clone();
        let reducer_thread = std::thread::Builder::new()
            .name("waddle-reducer".into())
            .spawn(move || reducer.run(&inject_rx, &reducer_tx))
            .expect("spawn reducer");

        let mut threads = vec![reducer_thread];
        let tripwire_shutdown = ShutdownToken::new();

        // Verb outcomes → FSM events.
        threads.push(pumps::spawn_outcome_pump(
            outcome_rx,
            inject_tx.clone(),
            clock.clone(),
        ));

        // Bypass pump: stall detection + direct sends during bypass.
        threads.push(pumps::spawn_bypass_pump(
            gate_shared.clone(),
            mirror.clone(),
            verbs.clone(),
            inject_tx.clone(),
            clock.clone(),
            dims.unwrap_or(0),
        ));

        // Reset pump: the single scripted-hook invocation site (mirror-watch,
        // like the bypass pump). Services RESETTING episodes nobody runs
        // inline (reducer-opened retake successors) and every declared POST
        // hook; remote windows are the FSM's, not the pump's.
        let inline_reset_owner: ResetOwnerSlot = Arc::new(parking_lot::Mutex::new(None));
        let episode_reset_specs: ResetSpecSlot = Arc::new(parking_lot::Mutex::new(None));
        threads.push(pumps::spawn_reset_pump(
            mirror.clone(),
            inject_tx.clone(),
            clock.clone(),
            task_slot.clone(),
            inline_reset_owner.clone(),
            episode_reset_specs.clone(),
            self.pre_reset.clone(),
            self.post_reset.clone(),
        ));

        // Media intake: teleop stream → gate ring; clutch → FSM.
        if let Some(media) = self.media {
            threads.push(pumps::spawn_media_intake(
                media,
                stream_tx.clone(),
                inject_tx.clone(),
                clock.clone(),
                mirror.clone(),
                dims,
                gripper_spec,
            )?);
        }

        // Camera uplink: one dedicated pump servicing every
        // declared camera that has a media plane to publish into;
        // `Session::publish_frame` feeds it through the per-camera bounded
        // queues built above.
        if !camera_uplinks.is_empty() {
            let media = media_for_cameras
                .expect("camera_uplinks is only ever populated when media is wired");
            threads.push(media_uplink::spawn_media_uplink(
                media,
                camera_uplinks.values().cloned().collect(),
                mirror.clone(),
            ));
        }

        // Plane directives → FSM events (claims, episode directives, reset
        // windows, reset-window agent chunks).
        if let Some(plane) = plane.clone() {
            threads.push(pumps::spawn_plane_pump(
                plane,
                inject_tx.clone(),
                clock.clone(),
                mirror.clone(),
                stream_tx.clone(),
                Arc::new(robot.action_space.clone()),
            ));
        }

        // Tripwires: fires REQUEST verbs through dispatch (never an
        // envelope). The observation source is the gate record
        // stream: `obs_slot` above, published by the reducer from every
        // `gate(obs=...)` call the customer's loop makes.
        if !self.tripwires.is_empty() {
            struct Sink {
                verbs: Arc<VerbDispatch>,
            }
            impl TripwireSink for Sink {
                fn request(&self, fire: TripwireFire) {
                    let req = match fire.requested_verb {
                        waddle_types::Verb::Estop => waddle_types::VerbRequest::Estop,
                        waddle_types::Verb::Resume => waddle_types::VerbRequest::Resume,
                        waddle_types::Verb::Home => waddle_types::VerbRequest::Home,
                        _ => waddle_types::VerbRequest::Hold,
                    };
                    self.verbs.request(req);
                }
            }
            /// Reads the latest gate-record obs: the customer's
            /// flat `gate(obs=...)` vector maps onto `ObsSnapshot::joint_pos`
            /// verbatim; `ee_pos`/`force_n` stay `None` (this seam carries a
            /// flat vector, not semantically-tagged fields), so
            /// `JointLimitMargin`/`Staleness` tripwires fire from it and
            /// `WorkspaceAabb`/`ForceThreshold` never do — those need a
            /// capture integration publishing structured obs, out of scope
            /// here.
            struct RecordObsSource {
                slot: ObsSlot,
            }
            impl waddle_tripwire::ObsSource for RecordObsSource {
                fn latest(&self) -> Option<waddle_tripwire::ObsSnapshot> {
                    self.slot.latest().map(|snap| (*snap).clone())
                }
            }
            threads.push(waddle_tripwire::spawn_evaluator(
                Evaluator::new(self.tripwires),
                clock.clone(),
                Arc::new(RecordObsSource {
                    slot: obs_slot.clone(),
                }),
                Arc::new(Sink {
                    verbs: verbs.clone(),
                }),
                std::time::Duration::from_millis(10),
                tripwire_shutdown.clone(),
            ));
        }

        Ok(Session {
            inner: Arc::new(SessionInner {
                clock,
                gate_shared,
                mirror,
                inject_tx,
                proprio_tx,
                record_slot,
                task_slot,
                pre_reset: self.pre_reset,
                post_reset: self.post_reset,
                inline_reset_owner,
                episode_reset_specs,
                verification_mode: self.verification_mode,
                threads: parking_lot::Mutex::new(threads),
                tripwire_shutdown,
                declared_cameras,
                camera_uplinks,
                _verbs: verbs,
                _plane: plane,
            }),
        })
    }
}

fn robot_digest(robot: &pb::RobotDescription) -> String {
    // A stable content digest of the declaration (FNV-1a over the encoded
    // bytes — collision resistance is not a goal here; identity is).
    let bytes = prost::Message::encode_to_vec(robot);
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        hash ^= u64::from(b);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("fnv1a:{hash:016x}")
}

struct SessionInner {
    clock: SessionClock,
    gate_shared: Arc<GateShared>,
    mirror: Arc<Mirror>,
    inject_tx: Sender<Injected>,
    /// See [`Session::report_proprio`].
    proprio_tx: Sender<ProprioReport>,
    record_slot: RecordSlot,
    task_slot: TaskSlot,
    pre_reset: Option<ResetSpec>,
    post_reset: Option<ResetSpec>,
    /// See [`ResetOwnerSlot`]; shared with the reset pump.
    inline_reset_owner: ResetOwnerSlot,
    /// See [`ResetSpecSlot`]; shared with the reset pump.
    episode_reset_specs: ResetSpecSlot,
    verification_mode: ResetVerificationMode,
    threads: parking_lot::Mutex<Vec<JoinHandle<()>>>,
    tripwire_shutdown: ShutdownToken,
    /// Every camera the robot declared (name → (width, height)), regardless
    /// of whether a media plane is wired — backs `publish_frame`'s
    /// unknown-camera and declared-resolution checks.
    declared_cameras: HashMap<String, (u32, u32)>,
    /// One entry per declared camera, present only when a media plane is
    /// wired: `publish_frame` enqueues into these; absent means "declared,
    /// but nothing to publish into" (a cheap no-op, never an error).
    camera_uplinks: HashMap<String, Arc<CameraUplink>>,
    _verbs: Arc<VerbDispatch>,
    _plane: Option<Arc<ControlPlaneClient>>,
}

/// One live supervision session.
#[derive(Clone)]
pub struct Session {
    inner: Arc<SessionInner>,
}

impl std::fmt::Debug for Session {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Session").finish_non_exhaustive()
    }
}

impl Session {
    #[must_use]
    pub fn builder(project: impl Into<String>) -> SessionBuilder {
        SessionBuilder::new(project)
    }

    /// Advanced/testing surface: inject a session event directly (the same
    /// funnel the pumps use). Local intervention sources and tests drive
    /// claims through this. Never carries directive-ack correlation — acks
    /// answer plane directives only (`crate::ack`).
    pub fn inject(&self, event: SessionEvent) {
        let _ = self.inner.inject_tx.send(event.into());
    }

    /// [`Self::start_episode_with`] using the session's declared reset specs
    /// unchanged (no per-episode override).
    pub fn start_episode(&self, task: &str) -> Result<Episode, RuntimeError> {
        self.start_episode_with(task, EpisodeOptions::default())
    }

    /// Open an episode and block through the reset pipeline (the design
    /// contract: `rollout()` does not yield until the scene is valid), with
    /// a per-episode override (`opts`) of the session's declared pre/post
    /// reset specs. See [`EpisodeOptions`] for the inherit/disable rules.
    ///
    /// One active episode per session (N18): opening while another episode
    /// is live and has not yet entered `Phase::PostReset` returns
    /// [`RuntimeError::EpisodeActive`] instead of injecting an event the FSM
    /// would reject (a guard, not a synchronization primitive — concurrent
    /// callers race to the FSM, which stays the authority). A predecessor
    /// that HAS entered `Phase::PostReset` is instead waited out to Terminal
    /// and then opened over: POST_RESET self-resolves (its own cleanup, past
    /// the pinned outcome already), so this serializes back-to-back rollouts
    /// started without an explicit `terminate` + wait in between, rather than
    /// erroring on a predecessor that is already on its way out.
    pub fn start_episode_with(
        &self,
        task: &str,
        opts: EpisodeOptions,
    ) -> Result<Episode, RuntimeError> {
        loop {
            let s = self.inner.mirror.read();
            match s.episode_state {
                None | Some(Phase::Terminal(_)) => break,
                Some(Phase::PostReset) => {
                    let status = self
                        .inner
                        .mirror
                        .wait_until(|s| !matches!(s.episode_state, Some(Phase::PostReset)));
                    if status.shutdown {
                        return Err(RuntimeError::ShuttingDown);
                    }
                    // POST_RESET can only leave to Terminal; loop back around
                    // to re-check rather than assume it.
                }
                Some(_) => return Err(RuntimeError::EpisodeActive),
            }
        }

        let id = EpisodeId::new(format!("ep-{}", uuid::Uuid::new_v4().simple()));
        // Written before the open event so the reducer stamps this task into
        // the episode's sidecar (retake successors inherit it).
        *self.inner.task_slot.lock() = task.to_owned();
        // The fresh record ring goes into the hand-off slot BEFORE the open
        // event: the reducer adopts it (discarding any stale predecessor
        // ring) no later than the wake that opens this episode's recording,
        // so a stale Episode handle can never write into this episode's
        // MCAP. The ring stays empty until this call returns.
        let (gate, records_rx) = Gate::new(
            self.inner.gate_shared.clone(),
            self.inner.clock.clone(),
            8192,
        );
        *self.inner.record_slot.lock() = Some(records_rx);

        // Resolve effective specs: outer None inherits the session default;
        // inner None disables that phase for this episode only.
        let pre = opts
            .pre_reset
            .unwrap_or_else(|| self.inner.pre_reset.clone());
        let post = opts
            .post_reset
            .unwrap_or_else(|| self.inner.post_reset.clone());
        let pre_window = pre.as_ref().and_then(ResetSpec::window);
        let post_window = post.as_ref().and_then(ResetSpec::window);
        // `post_reset` (bool) is true for either variant — a hook still
        // makes this episode detour through Phase::PostReset (E14); only
        // `post_window` distinguishes a *remote* post-reset from a hook one.
        let post_reset_declared = post.is_some();

        // Publish this episode's resolved specs BEFORE EpisodeOpen: by the
        // time the mirror can show this id at all, the reset pump can
        // resolve the effective spec for it (per-episode overrides
        // included) instead of falling back to session defaults.
        *self.inner.episode_reset_specs.lock() = Some(EpisodeResetSpecs {
            id: id.clone(),
            pre: pre.clone(),
            post: post.clone(),
        });

        // The inline pre-reset path covers everything but a declared Remote:
        // the configured Hook, and the no-spec default (both run on this
        // thread and inject ResetResult directly). Record the id before
        // EpisodeOpen so the reset pump never also services it; clear it
        // once this call returns, success or failure.
        let inline_pre = !matches!(pre, Some(ResetSpec::Remote { .. }));
        if inline_pre {
            *self.inner.inline_reset_owner.lock() = Some(id.clone());
        }

        let now = self.inner.clock.stamp_now().mono_ns();
        self.inject(SessionEvent::EpisodeOpen {
            id: id.clone(),
            verification: self.inner.verification_mode,
            born_claimed: false,
            parent: None,
            post_reset: post_reset_declared,
            pre_window,
            post_window,
            agent_invite: None,
            at: now,
        });

        match &pre {
            Some(ResetSpec::Hook(hook)) => {
                let (ok, verified) = hook(task);
                self.inject(SessionEvent::ResetResult {
                    ok,
                    verified,
                    at: self.inner.clock.stamp_now().mono_ns(),
                });
            }
            None => {
                // No spec configured (session default absent, or disabled
                // for this episode): the placeholder default — a scene
                // reset by hand between episodes.
                self.inject(SessionEvent::ResetResult {
                    ok: true,
                    verified: Some(true),
                    at: self.inner.clock.stamp_now().mono_ns(),
                });
            }
            Some(ResetSpec::Remote { .. }) => {
                // Skip the hook and the ResetResult injection entirely: the
                // window machinery (FSM) drives RESETTING to READY or
                // Terminal on its own. No runtime-side timeout is added here
                // — the FSM's reset-window timer owns that.
            }
        }

        let status = self.inner.mirror.wait_until(|s| {
            s.episode_id.as_ref() == Some(&id)
                && matches!(
                    s.episode_state,
                    Some(Phase::Ready | Phase::Terminal(_)) | None
                )
        });
        // Only clear the guard once the reducer has actually observed the
        // episode leaving RESETTING (confirmed via the mirror, not merely
        // "we finished sending the events") — `inject` is fire-and-forget,
        // so clearing this any earlier (e.g. right after the ResetResult
        // send above) would reopen the window the guard exists to close: a
        // pump thread could see `Phase::Resetting` with the guard already
        // cleared and double-service an episode this call is still driving.
        if inline_pre {
            *self.inner.inline_reset_owner.lock() = None;
        }
        if status.shutdown {
            return Err(RuntimeError::ShuttingDown);
        }
        match status.episode_state {
            Some(Phase::Ready) => {}
            other => {
                return Err(RuntimeError::ResetFailed(format!(
                    "episode entered {other:?} instead of READY"
                )));
            }
        }

        Ok(Episode {
            id,
            session: self.clone(),
            gate,
            started: false,
        })
    }

    /// True once `id` is no longer the live, still-rolling episode — because
    /// it terminated, entered `Phase::PostReset`, a successor replaced it,
    /// or the session shut down.
    ///
    /// POST_RESET counts as done: the terminal outcome is pinned at entry
    /// (FSM.md E14), so the rollout is over from the caller's view — only
    /// the scene cleanup is still running, and it self-resolves (the reset
    /// pump, a remote window, or its timeout). This also makes
    /// [`Self::terminate_episode`] a no-op during POST_RESET: a caller's
    /// teardown path (e.g. a context-manager exit racing a plane directive)
    /// must never inject a second Terminate against a pinned outcome.
    #[must_use]
    pub fn episode_done(&self, id: &EpisodeId) -> bool {
        let s = self.inner.mirror.read();
        s.shutdown
            || s.episode_id.as_ref() != Some(id)
            || matches!(s.episode_state, Some(Phase::Terminal(_) | Phase::PostReset))
    }

    /// Terminate episode `id` and block until the core confirms the
    /// terminal state. A no-op when `id` is not the live episode — a stale
    /// handle must never terminate a successor or a later episode — and
    /// (via [`Self::episode_done`]) when the episode is already in
    /// `Phase::PostReset`: its outcome is pinned and the cleanup
    /// self-resolves to Terminal. When the terminate itself detours through
    /// POST_RESET (a declared post-reset, FSM.md E14), this still blocks
    /// until Terminal — through the cleanup — per the design contract.
    pub fn terminate_episode(&self, id: &EpisodeId, outcome: TerminalOutcome, reason: &str) {
        if self.episode_done(id) {
            return;
        }
        let at = self.inner.clock.stamp_now().mono_ns();
        self.inject(SessionEvent::Terminate {
            outcome,
            reason: reason.to_owned(),
            at,
        });
        let id = id.clone();
        self.inner.mirror.wait_until(|s| {
            s.shutdown
                || s.episode_id.as_ref() != Some(&id)
                || matches!(s.episode_state, Some(Phase::Terminal(_)))
        });
    }

    #[must_use]
    pub fn status(&self) -> crate::mirror::Status {
        self.inner.mirror.read()
    }

    /// Publish one raw RGB8 video frame for a declared camera.
    /// Cheap on the caller's thread — validates `camera` against the
    /// robot's declared `cameras` and `frame`'s dimensions against that
    /// camera's declaration, applies the declared uplink fps throttle (a
    /// wait-free timestamp check — a too-soon frame is silently dropped,
    /// never an error, never counted in [`Self::camera_frames_dropped`]),
    /// and otherwise only enqueues the frame onto a small per-camera bounded
    /// queue. The (lazy, once-per-camera) `publish_track` call and the
    /// actual encode/`push_frame` run off this thread, on the dedicated
    /// `waddle-media-uplink` pump.
    ///
    /// - Unknown camera name (not in `RobotDescription.cameras`):
    ///   [`RuntimeError::UnknownCamera`].
    /// - `frame`'s (width, height) doesn't match the camera's declaration:
    ///   [`RuntimeError::Media`] (`MediaError::BadFrame`).
    /// - Declared camera, but no media plane wired at all: `Ok(())`,
    ///   nothing published — Local mode records no video in v0.
    ///
    /// The camera's declared `StreamPolicy.uplink.encoding` is
    /// bandwidth-intent for the video track, not a literal wire format:
    /// `RGB8`/`BGR8`/`JPEG` (and unspecified) all publish this same raw RGB8
    /// frame through to the track, which the wired transport converts as it
    /// needs (a real `LiveKit`-backed session encodes the track itself; a
    /// still-image byte stream is never produced on this path). `H264` is
    /// the one unsupported encoding — declaring it against a wired media
    /// plane is a build-time [`crate::SessionBuilder::build`] error, never a
    /// silent per-frame failure discovered later. See
    /// `crate::media_uplink`'s module docs for the full mapping.
    pub fn publish_frame(&self, camera: &str, frame: FrameData) -> Result<(), RuntimeError> {
        let Some(&(width, height)) = self.inner.declared_cameras.get(camera) else {
            return Err(RuntimeError::UnknownCamera(camera.to_owned()));
        };
        let Some(uplink) = self.inner.camera_uplinks.get(camera) else {
            // Declared, but no media plane wired: nothing to publish into.
            return Ok(());
        };
        let expected_len = (width as usize) * (height as usize) * 3;
        if frame.width() != width || frame.height() != height || frame.byte_len() != expected_len {
            return Err(RuntimeError::Media(waddle_media::MediaError::BadFrame {
                got: frame.byte_len(),
                expected: expected_len,
                layout: "RGB8 at the camera's declared resolution",
            }));
        }
        let now_ns = self.inner.clock.stamp_now().mono_ns().0;
        media_uplink::admit_and_enqueue(uplink, now_ns, frame);
        Ok(())
    }

    /// Report a richer proprioceptive sample than the bare `joint_pos`
    /// every `gate(obs=...)` call already records: the reducer
    /// merges `report` with its latest known `joint_pos` into every
    /// subsequent gate-tick's recorded `/waddle/observations` `ProprioSample`
    /// (Local mode — see [`ProprioReport`]'s rustdoc for the patch
    /// semantics) and into the periodic `StreamObservations` uplink,
    /// whenever a transport is configured. Cheap on the caller's thread: an
    /// unbounded fire-and-forget enqueue (occasional-call traffic, not the
    /// gate fast path — unlike `publish_frame`'s per-frame throttle, there
    /// is no declared rate to enforce here). Merged fields are stamped with
    /// whichever event actually lands them (the owning gate tick, or the
    /// uplink pump's own `SessionClock` read) — v0 accepts no
    /// caller-supplied timestamp on the report itself (the two-clock
    /// discipline).
    pub fn report_proprio(&self, report: ProprioReport) {
        let _ = self.inner.proprio_tx.send(report);
    }

    /// Frames dropped for `camera` because the uplink pump fell behind (the
    /// bounded per-camera queue overflowed and the oldest queued frame was
    /// discarded to admit the newest) — or because `publish_track`/encode/
    /// `push_frame` itself failed. `0` for an unknown camera or one with no
    /// media plane wired. Never counts fps-throttled frames: those are the
    /// declared policy working as intended, not data loss.
    #[must_use]
    pub fn camera_frames_dropped(&self, camera: &str) -> u64 {
        self.inner
            .camera_uplinks
            .get(camera)
            .map_or(0, |u| u.dropped())
    }

    /// Join all core threads and flush recorders. Ordered teardown: signal
    /// everything first (mirror flag, tripwires, verb dispatch — stopping
    /// dispatch drops the outcome sender so the outcome pump's blocking recv
    /// ends), then join.
    pub fn shutdown(self) {
        self.inner.mirror.update(|s| s.shutdown = true);
        self.inner.tripwire_shutdown.shutdown();
        self.inner._verbs.stop();
        let mut threads = self.inner.threads.lock();
        for t in threads.drain(..) {
            let _ = t.join();
        }
    }
}

/// One rollout attempt. Owned by the caller's loop thread; `gate()` is the
/// only core code that runs there.
pub struct Episode {
    id: EpisodeId,
    session: Session,
    gate: Gate<SessionClock>,
    started: bool,
}

impl std::fmt::Debug for Episode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Episode").field("id", &self.id).finish()
    }
}

impl Episode {
    #[must_use]
    pub fn id(&self) -> &EpisodeId {
        &self.id
    }

    /// The synchronous fast path: log + tripwires + claim consultation.
    /// `obs` is the observation the caller computed this tick's action from;
    /// it rides into the gate record so Pass records are training pairs and
    /// Substitute/Blend records are pre-labeled DAgger pairs.
    pub fn gate(
        &mut self,
        values: &[f64],
        gripper: Option<f64>,
        obs: Option<&[f64]>,
    ) -> GateOutput {
        if !self.started {
            self.started = true;
            let at = self.session.inner.clock.stamp_now().mono_ns();
            self.session.inject(SessionEvent::GateTick { at });
        }
        self.gate.gate(values, gripper, obs)
    }

    /// Flips when a judge, a directive, a timeout, `terminate`, or session
    /// shutdown ends the episode — including at `Phase::PostReset` entry,
    /// where the terminal outcome is already pinned and only the scene
    /// cleanup (which self-resolves) is still running. See
    /// [`Session::episode_done`].
    #[must_use]
    pub fn done(&self) -> bool {
        self.session.episode_done(&self.id)
    }

    /// The episode's outcome: the terminal outcome once `Phase::Terminal`,
    /// or the outcome pinned at POST_RESET entry while the cleanup is still
    /// running (they are the same value — E15–E17 carry the pinned outcome
    /// to Terminal unchanged). `None` while the rollout is still live.
    #[must_use]
    pub fn outcome(&self) -> Option<TerminalOutcome> {
        let s = self.session.inner.mirror.read();
        s.outcome.or(s.pinned_outcome)
    }

    /// Gate records dropped because the ring filled (the recording fell
    /// behind the caller's loop). Nonzero means training-data loss.
    #[must_use]
    pub fn records_dropped(&self) -> u64 {
        self.gate.records_dropped()
    }

    /// The session this episode runs on.
    #[must_use]
    pub fn session(&self) -> &Session {
        &self.session
    }

    pub fn terminate(&self, outcome: TerminalOutcome, reason: &str) {
        self.session.terminate_episode(&self.id, outcome, reason);
    }
}

/// Convenience: engage a local claim (used by tests and local intervention
/// sources; production claims arrive as plane directives).
///
/// Safety note: this injects `ClaimGranted`/`Engage` directly onto the
/// session's event funnel — it has zero dependency on `self.media` or
/// `self.control` and completely bypasses whatever engage path
/// `SessionBuilder::build` validated against. `build()` requires a
/// registered `hold` whenever the effective handoff is `HoldFirst` **and**
/// the session wires a media plane or registers `hold`/`send` directly; a
/// session built with none of those (the fully descriptors-only shape, see
/// the `build` rustdoc) passes that check even under the default
/// `HOLD_FIRST` policy. Calling `grant_and_engage` on such a session
/// reproduces the exact "clutch press, nothing happens" stall the check
/// exists to catch: `Verb::Hold` dispatch fails `NotRegistered` and engage
/// fails closed until the 10s `EngageTimeout`, with no diagnosable error at
/// the call site. Callers that drive engage this way — rather than through
/// a wired media plane — must register `hold` (and, for the same reason,
/// `send`) themselves; the build-time check cannot see through this call
/// site to know it will be used.
pub fn grant_and_engage(session: &Session, claim_id: &str, source: &str, actor: ActorKind) {
    let clock_now = |s: &Session| s.inner.clock.stamp_now().mono_ns();
    session.inject(SessionEvent::ClaimGranted {
        id: waddle_types::ClaimId::new(claim_id),
        source: source.to_owned(),
        actor,
        self_initiated: false,
        at: clock_now(session),
    });
    session.inject(SessionEvent::Engage {
        claim: waddle_types::ClaimId::new(claim_id),
        at: clock_now(session),
    });
}

/// Convenience: release a local claim (the counterpart of
/// [`grant_and_engage`]; production releases arrive as plane directives).
pub fn release_claim(session: &Session, claim_id: &str) {
    session.inject(SessionEvent::Release {
        claim: waddle_types::ClaimId::new(claim_id),
        at: session.inner.clock.stamp_now().mono_ns(),
    });
}

/// Convenience: engage an already-open reset window — the runtime-side
/// half of a plane ENGAGE directive (`pumps::forward_server_msg`'s
/// `ResetWindow::Engage` arm produces this exact two-event sequence:
/// `ClaimGranted` then `ResetWindowEngage`), so tests and the `waddle-sdk`
/// shim's testing hooks can drive a remote reset window without a
/// control-plane transport. `source` is recorded on the claim exactly as
/// [`grant_and_engage`] does; `actor` must satisfy C6 (match the window's
/// expected actor) or the FSM rejects the `ClaimGranted`.
pub fn reset_window_engage(session: &Session, claim_id: &str, source: &str, actor: ActorKind) {
    let clock_now = |s: &Session| s.inner.clock.stamp_now().mono_ns();
    let claim = waddle_types::ClaimId::new(claim_id);
    session.inject(SessionEvent::ClaimGranted {
        id: claim.clone(),
        source: source.to_owned(),
        actor,
        self_initiated: false,
        at: clock_now(session),
    });
    session.inject(SessionEvent::ResetWindowEngage {
        claim,
        at: clock_now(session),
    });
}

/// Convenience: complete an engaged reset window (the runtime-side half of
/// a plane COMPLETE directive) — injects `ResetWindowComplete{claim, ok,
/// verified}`.
pub fn reset_window_complete(session: &Session, claim_id: &str, ok: bool, verified: Option<bool>) {
    session.inject(SessionEvent::ResetWindowComplete {
        claim: waddle_types::ClaimId::new(claim_id),
        ok,
        verified,
        at: session.inner.clock.stamp_now().mono_ns(),
    });
}
