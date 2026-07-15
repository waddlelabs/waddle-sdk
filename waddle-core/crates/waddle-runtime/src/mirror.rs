//! The status mirror: a lock-guarded snapshot of the reducer's state that
//! the caller-facing handles (Session/Episode) and the pump threads read.
//! The reducer is the only writer; a condvar wakes blockers (e.g.
//! `start_episode` waiting through reset).

use std::sync::Arc;

use parking_lot::{Condvar, Mutex};
use waddle_types::{EpisodeId, GateMode, ProvenanceTag, TerminalOutcome};

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
