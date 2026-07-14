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

    pub fn build(self) -> Result<Session, RuntimeError> {
        let robot_pb = self.robot.ok_or(RuntimeError::MissingRobot)?;
        let robot = RobotDescription::try_from(&robot_pb)?;

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
        let dims = robot.action_space.dims();

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
    pub fn start_episode(&self, task: &str) -> Result<Episode, RuntimeError> {
        let id = EpisodeId::new(format!("ep-{}", uuid::Uuid::new_v4().simple()));
        // Written before the open event so the reducer stamps this task into
        // the episode's sidecar (retake successors inherit it).
        *self.inner.task_slot.lock() = task.to_owned();
        let now = self.inner.clock.stamp_now().mono_ns();
        self.inject(SessionEvent::EpisodeOpen {
            id: id.clone(),
            verification: self.inner.verification_mode,
            born_claimed: false,
            parent: None,
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

        let (gate, records_rx) = Gate::new(
            self.inner.gate_shared.clone(),
            self.inner.clock.clone(),
            8192,
        );
        // Hand the consumer end to the reducer, which drains it onto the
        // episode's MCAP (the reducer owns all recording).
        *self.inner.record_slot.lock() = Some(records_rx);
        Ok(Episode {
            id,
            session: self.clone(),
            gate,
            started: false,
        })
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

    /// Flips when a judge, a directive, a timeout, or `terminate` ends the
    /// episode.
    #[must_use]
    pub fn done(&self) -> bool {
        let s = self.session.inner.mirror.read();
        s.episode_id.as_ref() != Some(&self.id)
            || matches!(s.episode_state, Some(Phase::Terminal(_)))
    }

    #[must_use]
    pub fn outcome(&self) -> Option<TerminalOutcome> {
        self.session.inner.mirror.read().outcome
    }

    pub fn terminate(&self, outcome: TerminalOutcome, reason: &str) {
        let at = self.session.inner.clock.stamp_now().mono_ns();
        self.session.inject(SessionEvent::Terminate {
            outcome,
            reason: reason.to_owned(),
            at,
        });
        let id = self.id.clone();
        self.session.inner.mirror.wait_until(|s| {
            s.episode_id.as_ref() != Some(&id)
                || matches!(s.episode_state, Some(Phase::Terminal(_)))
        });
    }
}

/// Convenience: engage a local claim (used by tests and local intervention
/// sources; production claims arrive as plane directives).
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
