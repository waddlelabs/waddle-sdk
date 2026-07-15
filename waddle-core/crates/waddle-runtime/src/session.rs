//! The public runtime surface: `Session::builder() … build()`,
//! `session.start_episode(task)` (blocks through reset), `episode.gate(...)`.
//!
//! Threading (the design invariant made code): the session owns named
//! core threads — reducer, verb dispatch, bypass pump, media intake,
//! tripwire evaluator — and the control-plane client thread. Nothing
//! executes on the caller's thread except `Episode::gate()`.

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::mpsc::Sender;
use std::thread::JoinHandle;

use waddle_controlplane::{ClientConfig, ControlPlaneClient, ControlTransport};
use waddle_fsm::{Phase, SessionConfig, SessionEvent};
use waddle_gate::gate::{Gate, GateOutput, GateShared};
use waddle_gate::plan::GatePlan;
use waddle_gate::record::GateRecord;
use waddle_ingest::SessionClock;
use waddle_media::MediaPlane;
use waddle_tripwire::{Evaluator, ShutdownToken, Tripwire, TripwireFire, TripwireSink};
use waddle_types::pb::v0 as pb;
use waddle_types::time::Clock;
use waddle_types::{
    ActorKind, CellId, EpisodeId, HandoffPolicy, LeaseEnforcement, MonoNs, ResetVerificationMode,
    RobotDescription, RobotId, SessionId, TerminalOutcome,
};

use crate::RuntimeError;
use crate::mirror::Mirror;
use crate::pumps;
use crate::reducer::Reducer;
use crate::verbs::{ControlRegistry, VerbDispatch, VerbOutcome};

/// How resets run until the closed reset planner is wired: a callable
/// returning (ok, verified). The default reports ok+verified — honest only
/// for scenes reset by hand between episodes; integrations override it.
pub type ResetHook = Arc<dyn Fn(&str) -> (bool, Option<bool>) + Send + Sync>;

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
    reset_hook: Option<ResetHook>,
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
            reset_hook: None,
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

    #[must_use]
    pub fn reset_hook(mut self, hook: ResetHook) -> Self {
        self.reset_hook = Some(hook);
        self
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

        let (gate_shared, stream_tx) =
            GateShared::new(GatePlan::passthrough(MonoNs(0)), 1024, 20_000_000);

        let (outcome_tx, outcome_rx) = std::sync::mpsc::channel::<VerbOutcome>();
        let verbs = Arc::new(VerbDispatch::spawn(self.control, clock.clone(), outcome_tx));

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
                feature_flags: vec!["waddle.v0.core".into()],
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
        let (inject_tx, inject_rx) = std::sync::mpsc::channel::<SessionEvent>();
        let record_slot: RecordSlot = Arc::new(parking_lot::Mutex::new(None));
        let task_slot: TaskSlot = Arc::new(parking_lot::Mutex::new(String::new()));

        // The reducer thread.
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

        // Media intake: teleop stream → gate ring; clutch → FSM.
        if let Some(media) = self.media {
            threads.push(pumps::spawn_media_intake(
                media,
                stream_tx,
                inject_tx.clone(),
                clock.clone(),
                mirror.clone(),
                dims,
                gripper_spec,
            )?);
        }

        // Plane directives → FSM events.
        if let Some(plane) = plane.clone() {
            threads.push(pumps::spawn_plane_pump(
                plane,
                inject_tx.clone(),
                clock.clone(),
                mirror.clone(),
            ));
        }

        // Tripwires: fires REQUEST verbs through dispatch (never an
        // envelope). The observation source is wired by capture
        // integrations; until one registers, the evaluator idles.
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
            struct EmptySource;
            impl waddle_tripwire::ObsSource for EmptySource {
                fn latest(&self) -> Option<waddle_tripwire::ObsSnapshot> {
                    None
                }
            }
            threads.push(waddle_tripwire::spawn_evaluator(
                Evaluator::new(self.tripwires),
                clock.clone(),
                Arc::new(EmptySource),
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
                record_slot,
                task_slot,
                reset_hook: self.reset_hook,
                verification_mode: self.verification_mode,
                threads: parking_lot::Mutex::new(threads),
                tripwire_shutdown,
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
    inject_tx: Sender<SessionEvent>,
    record_slot: RecordSlot,
    task_slot: TaskSlot,
    reset_hook: Option<ResetHook>,
    verification_mode: ResetVerificationMode,
    threads: parking_lot::Mutex<Vec<JoinHandle<()>>>,
    tripwire_shutdown: ShutdownToken,
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
    /// claims through this.
    pub fn inject(&self, event: SessionEvent) {
        let _ = self.inner.inject_tx.send(event);
    }

    /// Open an episode and block through the reset pipeline (the design
    /// contract: `rollout()` does not yield until the scene is valid).
    ///
    /// One active episode per session (N18): opening while another episode
    /// is live returns [`RuntimeError::EpisodeActive`] instead of injecting
    /// an event the FSM would reject (a guard, not a synchronization
    /// primitive — concurrent callers race to the FSM, which stays the
    /// authority).
    pub fn start_episode(&self, task: &str) -> Result<Episode, RuntimeError> {
        {
            let s = self.inner.mirror.read();
            if s.episode_id.is_some() && !matches!(s.episode_state, Some(Phase::Terminal(_))) {
                return Err(RuntimeError::EpisodeActive);
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
        let now = self.inner.clock.stamp_now().mono_ns();
        self.inject(SessionEvent::EpisodeOpen {
            id: id.clone(),
            verification: self.inner.verification_mode,
            born_claimed: false,
            parent: None,
            // Post-reset declaration / remote windows are wired by the runtime
            // reset seams (a later task); undeclared for now.
            post_reset: false,
            pre_window: None,
            post_window: None,
            at: now,
        });

        // Run the reset hook (placeholder for the closed reset planner).
        let (ok, verified) = match &self.inner.reset_hook {
            Some(hook) => hook(task),
            None => (true, Some(true)),
        };
        self.inject(SessionEvent::ResetResult {
            ok,
            verified,
            at: self.inner.clock.stamp_now().mono_ns(),
        });

        let status = self.inner.mirror.wait_until(|s| {
            s.episode_id.as_ref() == Some(&id)
                && matches!(
                    s.episode_state,
                    Some(Phase::Ready | Phase::Terminal(_)) | None
                )
        });
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

    /// True once `id` is no longer the live, non-terminal episode — because
    /// it terminated, a successor replaced it, or the session shut down.
    #[must_use]
    pub fn episode_done(&self, id: &EpisodeId) -> bool {
        let s = self.inner.mirror.read();
        s.shutdown
            || s.episode_id.as_ref() != Some(id)
            || matches!(s.episode_state, Some(Phase::Terminal(_)))
    }

    /// Terminate episode `id` and block until the core confirms the
    /// terminal state. A no-op when `id` is not the live episode — a stale
    /// handle must never terminate a successor or a later episode.
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
    /// shutdown ends the episode.
    #[must_use]
    pub fn done(&self) -> bool {
        self.session.episode_done(&self.id)
    }

    #[must_use]
    pub fn outcome(&self) -> Option<TerminalOutcome> {
        self.session.inner.mirror.read().outcome
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
