//! Episode-level enums shared across the workspace.

use crate::error::TypesError;
use crate::pb::v0 as pb;

/// Terminal outcomes (N2). `AbortedRetake` is never silently folded into
/// success-rate denominators.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TerminalOutcome {
    Success,
    Failure,
    Abort,
    AbortedRetake,
}

impl TerminalOutcome {
    pub fn from_pb(value: i32) -> Result<Self, TypesError> {
        match pb::TerminalOutcome::try_from(value) {
            Ok(pb::TerminalOutcome::Success) => Ok(Self::Success),
            Ok(pb::TerminalOutcome::Failure) => Ok(Self::Failure),
            Ok(pb::TerminalOutcome::Abort) => Ok(Self::Abort),
            Ok(pb::TerminalOutcome::AbortedRetake) => Ok(Self::AbortedRetake),
            Ok(pb::TerminalOutcome::Unspecified) | Err(_) => Err(TypesError::InvalidEnum {
                field: "TerminalOutcome",
                value,
            }),
        }
    }

    #[must_use]
    pub fn to_pb(self) -> pb::TerminalOutcome {
        match self {
            Self::Success => pb::TerminalOutcome::Success,
            Self::Failure => pb::TerminalOutcome::Failure,
            Self::Abort => pb::TerminalOutcome::Abort,
            Self::AbortedRetake => pb::TerminalOutcome::AbortedRetake,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum EpisodeStateKind {
    Resetting,
    Ready,
    Running,
    Intervention,
    Terminal,
}

impl EpisodeStateKind {
    #[must_use]
    pub fn to_pb(self) -> pb::EpisodeState {
        match self {
            Self::Resetting => pb::EpisodeState::Resetting,
            Self::Ready => pb::EpisodeState::Ready,
            Self::Running => pb::EpisodeState::Running,
            Self::Intervention => pb::EpisodeState::Intervention,
            Self::Terminal => pb::EpisodeState::Terminal,
        }
    }
}

/// The intervention lifecycle: engage → settle → release | retake.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum InterventionPhase {
    Engage,
    Settle,
    Release,
    Retake,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GateMode {
    Passthrough,
    Intervention,
    Bypass,
}

/// Who waits for reset verification (N12).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ResetVerificationMode {
    /// The episode may not leave RESETTING until verification passes
    /// (autonomous resets).
    Blocking,
    /// Enter optimistically; a late verification failure permanently flags
    /// `reset_unverified` (teleoperator retakes — operator flow is sacred,
    /// the flag keeps optimism honest).
    OptimisticAsync,
}

impl ResetVerificationMode {
    pub fn from_pb(value: i32) -> Result<Self, TypesError> {
        match pb::ResetVerificationMode::try_from(value) {
            Ok(pb::ResetVerificationMode::Blocking) => Ok(Self::Blocking),
            Ok(pb::ResetVerificationMode::OptimisticAsync) => Ok(Self::OptimisticAsync),
            Ok(pb::ResetVerificationMode::Unspecified) | Err(_) => Err(TypesError::InvalidEnum {
                field: "ResetVerificationMode",
                value,
            }),
        }
    }
}
