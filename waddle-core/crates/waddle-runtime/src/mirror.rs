//! The status mirror: a lock-guarded snapshot of the reducer's state that
//! the caller-facing handles (Session/Episode) and the pump threads read.
//! The reducer is the only writer; a condvar wakes blockers (e.g.
//! `start_episode` waiting through reset).

use std::sync::Arc;

use parking_lot::{Condvar, Mutex};
use waddle_types::pb::v0 as pb;
use waddle_types::{EpisodeId, GateMode, ProvenanceTag, TerminalOutcome};

/// Pipeline progress within a plane-EXECUTED reset (the `RequestReset`/
/// `ResetProgress` RPCs, `waddle.v0.reset`) — distinct from an SDK-executed
/// remote reset WINDOW (flag `waddle.v0.reset.remote`, `ResetWindowEvent`).
/// Mirrors `services.proto`'s `ResetPhase` permissively: an unrecognized
/// wire value maps to `Unspecified` rather than erroring, since this is
/// observational status only, never consulted by the FSM.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ResetProgressPhase {
    #[default]
    Unspecified,
    Planning,
    Executing,
    Verifying,
    Done,
}

impl ResetProgressPhase {
    #[must_use]
    pub fn from_pb(value: i32) -> Self {
        match pb::ResetPhase::try_from(value) {
            Ok(pb::ResetPhase::Planning) => Self::Planning,
            Ok(pb::ResetPhase::Executing) => Self::Executing,
            Ok(pb::ResetPhase::Verifying) => Self::Verifying,
            Ok(pb::ResetPhase::Done) => Self::Done,
            _ => Self::Unspecified,
        }
    }
}

/// The plane's most recent `ResetProgress` message. Observational
/// only: nothing in the FSM reads this, and `episode.proto` doesn't model
/// `ResetProgress` as an `EpisodeEvent` (services-message, not sidecar/wire
/// history) — so this mirror field is the only surface for it. `None` until
/// the plane sends its first `ResetProgress` for the session.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ResetProgressStatus {
    pub phase: ResetProgressPhase,
    pub strategy: String,
    pub detail: String,
}

#[derive(Debug, Clone, Default)]
pub struct Status {
    pub episode_id: Option<EpisodeId>,
    pub episode_state: Option<waddle_fsm::Phase>,
    pub gate_mode: Option<GateMode>,
    pub claim_active: bool,
    /// Provenance tag of the active claim (bypass pump stamps sends with it).
    pub provenance: Option<ProvenanceTag>,
    pub outcome: Option<TerminalOutcome>,
    /// The terminal outcome pinned at POST_RESET entry (FSM.md E14), before
    /// the episode actually reaches `Phase::Terminal`. `None` until pinned;
    /// for post-reset-declared episodes it equals `outcome` once terminal
    /// (E15–E17 carry it unchanged). This is what makes the episode "done"
    /// from the caller's view at `Phase::PostReset`.
    pub pinned_outcome: Option<TerminalOutcome>,
    /// PERMANENT once set (FSM.md E16/E17): the post-reset cleanup failed
    /// or was estopped. NEVER alters the (pinned) outcome.
    pub post_reset_failed: bool,
    /// The plane's most recent plane-executed reset progress; see
    /// [`ResetProgressStatus`].
    pub reset_progress: Option<ResetProgressStatus>,
    pub plane_connected: bool,
    pub shutdown: bool,
    /// Set once, at build time, when the session's `ControlRegistry` has no
    /// `estop` callable. Missing `estop` never fails the build (unlike
    /// `hold`/`send` — see `SessionBuilder::build`), but the degradation
    /// must stay observable rather than surfacing only as a
    /// `VerbError::NotRegistered` the first time something actually
    /// requests an estop.
    pub estop_unregistered: bool,
}

#[derive(Debug, Default)]
pub struct Mirror {
    state: Mutex<Status>,
    changed: Condvar,
}

impl Mirror {
    #[must_use]
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    pub fn update(&self, f: impl FnOnce(&mut Status)) {
        let mut s = self.state.lock();
        f(&mut s);
        self.changed.notify_all();
    }

    #[must_use]
    pub fn read(&self) -> Status {
        self.state.lock().clone()
    }

    /// Block until the predicate holds (or shutdown). Returns the snapshot.
    pub fn wait_until(&self, mut pred: impl FnMut(&Status) -> bool) -> Status {
        let mut s = self.state.lock();
        while !pred(&s) && !s.shutdown {
            self.changed.wait(&mut s);
        }
        s.clone()
    }
}
