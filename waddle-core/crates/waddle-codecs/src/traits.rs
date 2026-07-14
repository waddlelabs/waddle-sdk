//! The [`Codec`] trait: the seam between an upstream wire format and the
//! Waddle protocol types.
//!
//! # Stability
//!
//! This trait is **unstable** until at least two *external* dialects exist
//! against it (mirroring the N5 rule that an abstraction is proven by its
//! second independent implementation, not its first). The two in-tree
//! dialects exercise the shape; they do not freeze it. Downstream codec
//! authors should expect breaking trait changes across `waddle-codecs`
//! minor versions until this notice is removed.

use waddle_types::pb::v0 as pb;

use crate::descriptor::CodecDescriptor;

/// What a codec can honestly do with the dialect it speaks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CodecCaps {
    /// Full bidirectional understanding of observations AND actions.
    /// `Total` is what unlocks chunk substitution: Waddle may synthesize
    /// action chunks (intervention, hold, bypass) and encode them back onto
    /// the integrator's wire.
    Total,
    /// The codec understands framing (message boundaries, timestamps) but
    /// not payload semantics. A `FramingOnly` codec degrades the session to
    /// observe-only: Waddle can watch and record, but never substitutes
    /// chunks it cannot faithfully encode.
    FramingOnly,
}

/// A decoded observation frame, dialect-independent.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ObsFrame {
    /// Session-monotonic nanoseconds.
    pub t_ns: i64,
    /// Proprioceptive state vector, in the dialect's declared layout.
    pub state: Vec<f64>,
    /// Named encoded images (camera name → encoded bytes).
    pub images: Vec<(String, bytes::Bytes)>,
    /// Natural-language task string, when the dialect carries one.
    pub task: String,
}

/// Errors from codec encode/decode.
#[derive(Debug, thiserror::Error)]
pub enum CodecError {
    /// The wire payload does not parse as this dialect.
    #[error("malformed {dialect} wire payload: {reason}")]
    Malformed {
        dialect: &'static str,
        reason: String,
    },
    /// The value is valid Waddle-side but has no representation on this
    /// dialect's wire (e.g. a non-joint-space action on a joints-only wire).
    #[error("not representable on the {dialect} wire: {reason}")]
    NotRepresentable {
        dialect: &'static str,
        reason: String,
    },
    /// The operation is outside this codec's capabilities (see
    /// [`CodecCaps::FramingOnly`]).
    #[error("operation unsupported by this codec: {0}")]
    Unsupported(&'static str),
}

/// A dialect codec. Works on `waddle_types::pb` wire messages, NOT the
/// validated domain layer: codecs sit on the wire side of the boundary, and
/// validation against the declared action space happens exactly once, after
/// decode, in `waddle-types`.
///
/// See the module docs for the stability declaration: this trait is unstable
/// until two external dialects exist (N5).
pub trait Codec: Send + Sync {
    /// Identity, versions, and signature material for this codec.
    fn descriptor(&self) -> &CodecDescriptor;

    /// What this codec can honestly do (see [`CodecCaps`]).
    fn caps(&self) -> CodecCaps;

    /// Decode one observation frame from the dialect wire.
    fn decode_obs(&self, wire: &[u8]) -> Result<ObsFrame, CodecError>;

    /// Encode one observation frame onto the dialect wire.
    fn encode_obs(&self, obs: &ObsFrame) -> Result<Vec<u8>, CodecError>;

    /// Decode one action chunk from the dialect wire.
    fn decode_action(&self, wire: &[u8]) -> Result<pb::ActionChunk, CodecError>;

    /// Encode one action chunk onto the dialect wire.
    fn encode_action(&self, chunk: &pb::ActionChunk) -> Result<Vec<u8>, CodecError>;
}
