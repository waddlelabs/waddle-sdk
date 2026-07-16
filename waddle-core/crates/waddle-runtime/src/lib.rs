//! waddle-runtime — the composition root: the Session object that wires the
//! FSM reducer, gate, verb dispatch, tripwires, sidecar recording, media
//! intake, and the control-plane client onto core-owned named threads.
//!
//! Nothing in the core executes on a caller's thread except the synchronous
//! `Episode::gate()` fast path. Everything runs on dedicated named threads +
//! channels; tokio exists only inside the transport features (`grpc` →
//! waddle-controlplane's tonic worker, `livekit` → waddle-media's worker),
//! each confined to its own thread's private current-thread runtime.

mod media_uplink;
pub mod mirror;
pub mod pumps;
pub mod reducer;
pub mod session;
pub mod verbs;

pub use media_uplink::{FrameData, FramePixels};
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
}
