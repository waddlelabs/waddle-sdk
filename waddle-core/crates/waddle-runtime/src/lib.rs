//! waddle-runtime — the composition root: the Session object that wires the
//! FSM reducer, gate, verb dispatch, tripwires, sidecar recording, media
//! intake, and the control-plane client onto core-owned named threads.
//!
//! Nothing in the core executes on a caller's thread except the synchronous
//! `Episode::gate()` fast path. Everything runs on dedicated named threads +
//! channels; tokio exists only inside the transport features (`grpc` →
//! waddle-controlplane's tonic worker, `livekit` → waddle-media's worker),
//! each confined to its own thread's private current-thread runtime.

mod ack;
mod chat;
mod jog;
mod media_uplink;
pub mod mirror;
mod plane_events;
pub mod pumps;
pub mod reducer;
pub mod session;
pub mod verbs;

pub use jog::{JogAxis, JogRefusal, JogRequest};
pub use media_uplink::{FrameData, FramePixels};
pub use mirror::{AgentTaskKind, AgentTaskStatus, ResetProgressPhase, ResetProgressStatus, Status};
pub use pumps::STALL_THRESHOLD_NS;
pub use session::{
    AgentOutcome, EePose, Episode, EpisodeOptions, ProprioReport, ResetHook, ResetSpec, Session,
    SessionBuilder, SessionStamp, grant_and_engage, push_intervention_chunk, release_claim,
    reset_window_complete, reset_window_engage,
};
// The invite payload `EpisodeOptions::agent_invite` carries is waddle-fsm's
// own type (the FSM is the authority on what an invite is — hollow
// frontend); re-exported so runtime callers need not name the fsm crate.
pub use verbs::{ControlRegistry, EstopDecl, SendVerb, UnitVerb, VerbDispatch, VerbError};
pub use waddle_fsm::AgentInvite;

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
    #[error("invalid task metadata: {0}")]
    InvalidTaskMetadata(String),
    #[error("invalid chat request: {0}")]
    InvalidChat(String),
    #[error("chat unavailable: {0}")]
    ChatUnavailable(String),
    #[error("invalid optional plane request: {0}")]
    InvalidPlaneRequest(String),
    #[error("optional plane service unavailable: {0}")]
    PlaneServiceUnavailable(String),
    #[error("media plane: {0}")]
    Media(#[from] waddle_media::MediaError),
    #[error("the session is shutting down")]
    ShuttingDown,
    /// `Session::publish_frame` named a camera the robot never declared in
    /// `RobotDescription.cameras`.
    #[error("unknown camera {0:?} (not declared in the robot's cameras)")]
    UnknownCamera(String),
    /// A build-time check (`SessionBuilder::build`): a declared camera's
    /// `StreamPolicy.uplink.encoding` names a codec no encoder implements
    /// yet (currently: H.264 — see `waddle_media::VideoEncoding::H264`).
    /// Caught here, for cameras a wired media plane will actually publish,
    /// instead of failing every frame silently at runtime.
    #[error(
        "camera {camera:?} declares uplink encoding {encoding} but that codec integration is \
         not implemented yet — declare rgb8 (raw passthrough) or jpeg instead"
    )]
    UnsupportedCameraEncoding {
        camera: String,
        encoding: &'static str,
    },
    /// A build-time check: a camera declares an uplink policy at all (`
    /// StreamPolicy.uplink` is present) but its `fps` is not positive. A
    /// non-positive fps is meaningful only as "no policy declared" (the
    /// unthrottled default for a camera with none) — a *present* policy
    /// with `fps <= 0` is always a misconfiguration, never a request to
    /// suppress every frame or run unthrottled.
    #[error("camera {camera:?} declares an uplink policy with fps {fps} (must be > 0)")]
    InvalidCameraUplinkFps { camera: String, fps: f64 },
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
    /// A build-time check: `SessionBuilder::recording_dir` names a path this
    /// session cannot keep an archive in. A directory that does not exist yet
    /// is CREATED rather than refused — a caller who asks for local recording
    /// means it — so what reaches here is a path no directory can be made at,
    /// or one nothing may write into: an existing file, an unwritable parent,
    /// a read-only mount.
    ///
    /// Caught at build time because the alternative is the failure this
    /// variant exists to prevent: every writer downstream opens files INSIDE
    /// that directory, so the session opens clean, runs for as long as it is
    /// asked to, and leaves nothing on disk. The local recorder holds the
    /// full-rate archive; it may not silently hold nothing.
    #[error(
        "recording_dir {path:?} cannot hold this session's archive: {source} — \
         point it at a writable directory (a missing one is created)"
    )]
    RecordingDirUnusable {
        path: std::path::PathBuf,
        source: std::io::Error,
    },
}
