//! Episode state (FSM.md §1).

use waddle_types::{
    EpisodeId, EpisodeStateKind, InterventionPhase, ResetVerificationMode, TerminalOutcome,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub enum Phase {
    Resetting,
    Ready,
    Running,
    Intervention(InterventionPhase),
    Terminal(TerminalOutcome),
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
        }
    }

    #[must_use]
    pub fn is_terminal(&self) -> bool {
        matches!(self, Self::Terminal(_))
    }
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
        }
    }
}
