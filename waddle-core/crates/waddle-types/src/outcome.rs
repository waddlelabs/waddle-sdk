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
    /// Post-run scene cleanup INSIDE the finishing episode (flag
    /// `waddle.v0.reset.phases`). The terminal outcome is decided before
    /// entry and never changed here.
    PostReset,
}

impl EpisodeStateKind {
    pub fn from_pb(value: i32) -> Result<Self, TypesError> {
        match pb::EpisodeState::try_from(value) {
            Ok(pb::EpisodeState::Resetting) => Ok(Self::Resetting),
            Ok(pb::EpisodeState::Ready) => Ok(Self::Ready),
            Ok(pb::EpisodeState::Running) => Ok(Self::Running),
            Ok(pb::EpisodeState::Intervention) => Ok(Self::Intervention),
            Ok(pb::EpisodeState::Terminal) => Ok(Self::Terminal),
            Ok(pb::EpisodeState::PostReset) => Ok(Self::PostReset),
            Ok(pb::EpisodeState::Unspecified) | Err(_) => Err(TypesError::InvalidEnum {
                field: "EpisodeState",
                value,
            }),
        }
    }

    #[must_use]
    pub fn to_pb(self) -> pb::EpisodeState {
        match self {
            Self::Resetting => pb::EpisodeState::Resetting,
            Self::Ready => pb::EpisodeState::Ready,
            Self::Running => pb::EpisodeState::Running,
            Self::Intervention => pb::EpisodeState::Intervention,
            Self::Terminal => pb::EpisodeState::Terminal,
            Self::PostReset => pb::EpisodeState::PostReset,
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
    /// A remote actor is performing a scene reset through the SDK (flag
    /// `waddle.v0.reset.remote`): the reset claimant holds the lease and the
    /// SDK drives `send` from its own thread; any caller tick gets a
    /// `NoopMarker`.
    Reset,
}

impl GateMode {
    pub fn from_pb(value: i32) -> Result<Self, TypesError> {
        match pb::GateMode::try_from(value) {
            Ok(pb::GateMode::Passthrough) => Ok(Self::Passthrough),
            Ok(pb::GateMode::Intervention) => Ok(Self::Intervention),
            Ok(pb::GateMode::Bypass) => Ok(Self::Bypass),
            Ok(pb::GateMode::Reset) => Ok(Self::Reset),
            Ok(pb::GateMode::Unspecified) | Err(_) => Err(TypesError::InvalidEnum {
                field: "GateMode",
                value,
            }),
        }
    }

    #[must_use]
    pub fn to_pb(self) -> pb::GateMode {
        match self {
            Self::Passthrough => pb::GateMode::Passthrough,
            Self::Intervention => pb::GateMode::Intervention,
            Self::Bypass => pb::GateMode::Bypass,
            Self::Reset => pb::GateMode::Reset,
        }
    }
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

/// Which reset a record refers to (flag `waddle.v0.reset.phases` /
/// `waddle.v0.reset.remote`); distinct from services.proto's `ResetPhase`
/// (pipeline progress within one reset).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ResetKind {
    Pre,
    Post,
}

impl ResetKind {
    pub fn from_pb(value: i32) -> Result<Self, TypesError> {
        match pb::ResetKind::try_from(value) {
            Ok(pb::ResetKind::Pre) => Ok(Self::Pre),
            Ok(pb::ResetKind::Post) => Ok(Self::Post),
            Ok(pb::ResetKind::Unspecified) | Err(_) => Err(TypesError::InvalidEnum {
                field: "ResetKind",
                value,
            }),
        }
    }

    #[must_use]
    pub fn to_pb(self) -> pb::ResetKind {
        match self {
            Self::Pre => pb::ResetKind::Pre,
            Self::Post => pb::ResetKind::Post,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // reset-phases: these mirrors are inert in this task (no FSM/gate/runtime
    // behavior reads them yet); the round trip is what's testable here.

    #[test]
    fn episode_state_post_reset_pb_round_trip() {
        assert_eq!(
            EpisodeStateKind::PostReset.to_pb(),
            pb::EpisodeState::PostReset
        );
        assert_eq!(
            EpisodeStateKind::from_pb(pb::EpisodeState::PostReset as i32),
            Ok(EpisodeStateKind::PostReset)
        );
    }

    #[test]
    fn gate_mode_reset_pb_round_trip() {
        assert_eq!(GateMode::Reset.to_pb(), pb::GateMode::Reset);
        assert_eq!(
            GateMode::from_pb(pb::GateMode::Reset as i32),
            Ok(GateMode::Reset)
        );
    }

    #[test]
    fn reset_kind_pb_round_trip() {
        for (kind, pb_kind) in [
            (ResetKind::Pre, pb::ResetKind::Pre),
            (ResetKind::Post, pb::ResetKind::Post),
        ] {
            assert_eq!(kind.to_pb(), pb_kind);
            assert_eq!(ResetKind::from_pb(pb_kind as i32), Ok(kind));
        }
    }

    #[test]
    fn reset_kind_unspecified_is_error() {
        assert_eq!(
            ResetKind::from_pb(pb::ResetKind::Unspecified as i32),
            Err(TypesError::InvalidEnum {
                field: "ResetKind",
                value: 0
            })
        );
    }
}
