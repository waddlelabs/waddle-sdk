//! Error type for the sidecar crate.

/// Everything that can go wrong producing, serializing, or persisting a
/// sidecar record.
#[derive(Debug, thiserror::Error)]
pub enum SidecarError {
    #[error("json: {0}")]
    Json(#[from] serde_json::Error),

    #[error("message type {0:?} not found in the embedded descriptor set")]
    MissingDescriptor(String),

    #[error("proto transcode: {0}")]
    ProtoDecode(#[from] prost::DecodeError),

    #[error("io: {0}")]
    Io(#[from] std::io::Error),

    #[error("mcap: {0}")]
    Mcap(#[from] mcap::McapError),

    #[error("invalid sidecar record: {0}")]
    Invalid(String),

    #[error("archive ref {ref_id:?} cannot be resolved: {reason}")]
    Unresolvable { ref_id: String, reason: String },
}
