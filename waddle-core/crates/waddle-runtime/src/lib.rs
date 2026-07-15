//! waddle-runtime — the composition root: the Session object that wires the
//! FSM reducer, gate, verb dispatch, tripwires, sidecar recording, media
//! intake, and the control-plane client onto core-owned named threads.
//!
//! Nothing in the core executes on a caller's thread except the synchronous
//! `Episode::gate()` fast path. There is deliberately no async runtime:
//! dedicated threads + channels until the tonic/LiveKit integrations land.

pub mod mirror;
pub mod pumps;
pub mod reducer;
pub mod session;
pub mod verbs;

pub use mirror::Status;
pub use pumps::STALL_THRESHOLD_NS;
pub use session::{
    Episode, EpisodeOptions, ResetHook, ResetSpec, Session, SessionBuilder, grant_and_engage,
    release_claim, reset_window_complete, reset_window_engage,
};
pub use verbs::{ControlRegistry, EstopDecl, SendVerb, UnitVerb, VerbDispatch, VerbError};

#[derive(Debug, thiserror::Error)]
pub enum RuntimeError {
    #[error("a RobotDescription is required")]
    MissingRobot,
    #[error("invalid declaration: {0}")]
    Types(#[from] waddle_types::TypesError),
    #[error("reset failed: {0}")]
    ResetFailed(String),
    #[error("an episode is already active (one active episode per session)")]
    EpisodeActive,
    #[error("media plane: {0}")]
    Media(#[from] waddle_media::MediaError),
    #[error("the session is shutting down")]
    ShuttingDown,
    /// A build-time verb-registration check: the session's configuration
    /// (its handoff policy, or a wired feature) requires a verb whose
    /// callable the integrator never registered on the `ControlRegistry`.
    /// Caught here instead of failing silently at first dispatch — for
    /// `hold` under `HandoffPolicy::HoldFirst`, that failure mode is a 10s
    /// engage timeout with nothing to diagnose it: the teleoperator presses
    /// the clutch and nothing happens.
    #[error(
        "{required_by} requires a registered `{verb}` verb — register one in your Control, or {remedy}"
    )]
    MissingVerb {
        verb: &'static str,
        required_by: &'static str,
        remedy: &'static str,
    },
}
