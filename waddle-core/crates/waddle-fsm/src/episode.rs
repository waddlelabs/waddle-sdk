//! Episode state (FSM.md §1).

use waddle_types::{
    ActorKind, EpisodeId, EpisodeStateKind, InterventionPhase, ResetKind, ResetVerificationMode,
    TerminalOutcome,
};

use crate::event::WindowSpec;

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub enum Phase {
    Resetting,
    Ready,
    Running,
    Intervention(InterventionPhase),
    Terminal(TerminalOutcome),
    /// Post-run scene cleanup INSIDE the finishing episode (flag
    /// `waddle.v0.reset.phases`). The terminal outcome is pinned before entry
    /// and never changed here (FSM.md §1.3, rows E14–E18).
    PostReset,
}

impl Phase {
    #[must_use]
    pub fn kind(&self) -> EpisodeStateKind {
        match self {
            Self::Resetting => EpisodeStateKind::Resetting,
            Self::Ready => EpisodeStateKind::Ready,
            Self::Running => EpisodeStateKind::Running,
            Self::Intervention(_) => EpisodeStateKind::Intervention,
            Self::Terminal(_) => EpisodeStateKind::Terminal,
            Self::PostReset => EpisodeStateKind::PostReset,
        }
    }

    #[must_use]
    pub fn is_terminal(&self) -> bool {
        matches!(self, Self::Terminal(_))
    }
}

/// A remote reset window currently open on the episode (flag
/// `waddle.v0.reset.remote`, rows E19–E22). Present only while a window is
/// OPEN or ENGAGED; cleared when the window closes (complete/timeout/estop).
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct ResetWindowState {
    /// PRE (in RESETTING) or POST (in POST_RESET).
    pub kind: ResetKind,
    /// The actor the plane expects to perform the reset (C6 admission).
    pub expected: ActorKind,
    /// The reset claimant has engaged (lease → claimant, gate → RESET).
    pub engaged: bool,
    pub prompt: String,
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct EpisodeState {
    pub id: EpisodeId,
    pub phase: Phase,
    /// Retake successor opened under a still-held claim (N18).
    pub born_claimed: bool,
    pub parent: Option<EpisodeId>,
    /// PERMANENT once set (N2/N12).
    pub reset_unverified: bool,
    pub verification: ResetVerificationMode,
    /// The reset pipeline reported ok (a reset ran to completion).
    pub reset_ok: bool,
    /// A passing verification has been observed.
    pub verified: bool,
    /// The episode left RESETTING optimistically (eligible for late
    /// invalidation).
    pub optimistic_entry: bool,
    /// A post-reset pipeline (hook or remote window) runs before TERMINAL
    /// (flag `waddle.v0.reset.phases`). Undeclared episodes leave this false
    /// and behave exactly per E1–E13.
    pub post_reset_declared: bool,
    /// The terminal outcome fixed at POST_RESET entry (E14); the →TERMINAL
    /// transition after cleanup carries it unchanged (E15–E17).
    pub pinned_outcome: Option<TerminalOutcome>,
    /// PERMANENT once set (E16/E17): post-reset cleanup failed or was
    /// estopped. NEVER alters the pinned outcome.
    pub post_reset_failed: bool,
    /// The remote reset window currently open (E19–E22), if any.
    pub reset_window: Option<ResetWindowState>,
    /// The declared POST reset remote window, stashed at open so E14 can open
    /// it. `None` means the post-reset pipeline (if declared) is a hook.
    pub post_window: Option<WindowSpec>,
    /// The episode was opened agent-invited (flag `waddle.v0.agent`, E23):
    /// C8 claim admission and the E24 caller-tick Noop plan apply. Otherwise
    /// this is a NORMAL episode (FSM.md §1.5).
    pub agent_invited: bool,
    /// LATCHED at the first agent ENGAGE (E7 on an agent-invited episode):
    /// true from then on, never reset within the episode — a
    /// release/re-engage cycle does not re-arm the invite timer.
    pub agent_engaged: bool,
    /// LATCHED when the invite machinery itself closes the run: E25 (the
    /// invite deadline elapsed) or E26 (a plane DENIED while the invite was
    /// open). Never set by any other close — an E5 reset failure, E10
    /// terminate, or E11 estop on an agent-invited episode leaves this
    /// false, so an embedder can tell "the ask was declined / unanswered"
    /// (a legitimate agent outcome) apart from "the episode broke for
    /// reasons unrelated to the invite" (an error to surface).
    pub invite_aborted: bool,
}

impl EpisodeState {
    #[must_use]
    pub fn open(
        id: EpisodeId,
        verification: ResetVerificationMode,
        born_claimed: bool,
        parent: Option<EpisodeId>,
    ) -> Self {
        Self {
            id,
            phase: Phase::Resetting,
            born_claimed,
            parent,
            reset_unverified: false,
            verification,
            reset_ok: false,
            verified: false,
            optimistic_entry: false,
            post_reset_declared: false,
            pinned_outcome: None,
            post_reset_failed: false,
            reset_window: None,
            post_window: None,
            agent_invited: false,
            agent_engaged: false,
            invite_aborted: false,
        }
    }

    /// The invite is open (FSM.md §1.5): from E23 until the first agent
    /// ENGAGE (E7) or any exit from {RESETTING, READY, RUNNING}. E25/E26
    /// transition only while open; a stale `AgentInviteTimeout` expiry after
    /// close is discarded, and a late DENIED is E26b's recorded-only
    /// rejection.
    #[must_use]
    pub fn invite_open(&self) -> bool {
        self.agent_invited
            && !self.agent_engaged
            && matches!(self.phase, Phase::Resetting | Phase::Ready | Phase::Running)
    }
}
