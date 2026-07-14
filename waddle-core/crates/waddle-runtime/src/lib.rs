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
pub use session::{Episode, ResetHook, Session, SessionBuilder, grant_and_engage, release_claim};
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
}
