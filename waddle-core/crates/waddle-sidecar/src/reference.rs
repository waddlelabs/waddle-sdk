//! Reference-mode helpers (N8): the customer's recorder keeps the bulk
//! bytes; the sidecar carries [`pb::ArchiveRef`]s resolved at read time.

use sha2::{Digest, Sha256};
use waddle_types::pb::v0 as pb;

use crate::error::SidecarError;

/// Resolves an [`pb::ArchiveRef`] to its bytes at read time. Implemented by
/// the integrator against their storage (S3, local MCAP shelf, ...); the
/// `resolver` field of the ref names which registered resolver is
/// authoritative — `uri_hint` is advisory only.
pub trait RefResolver: Send + Sync {
    fn resolve(&self, r: &pb::ArchiveRef) -> Result<bytes::Bytes, SidecarError>;
}

/// Builds [`pb::ArchiveRef`]s for one stream, hashing the referenced bytes
/// so any later resolution can be verified against the record.
#[derive(Debug, Clone)]
pub struct StreamRefBuilder {
    stream_id: String,
    resolver: String,
    media_type: String,
}

impl StreamRefBuilder {
    #[must_use]
    pub fn new(stream_id: impl Into<String>, resolver: impl Into<String>) -> Self {
        Self {
            stream_id: stream_id.into(),
            resolver: resolver.into(),
            media_type: String::new(),
        }
    }

    #[must_use]
    pub fn media_type(mut self, media_type: impl Into<String>) -> Self {
        self.media_type = media_type.into();
        self
    }

    /// Build a ref over `payload` for the half-open session-time range
    /// `[t_start_ns, t_end_ns)`. `content_hash` is `sha256:<hex>` of the
    /// payload bytes.
    #[must_use]
    pub fn build_ref(
        &self,
        ref_id: impl Into<String>,
        t_start_ns: i64,
        t_end_ns: i64,
        payload: &[u8],
        uri_hint: impl Into<String>,
    ) -> pb::ArchiveRef {
        pb::ArchiveRef {
            ref_id: ref_id.into(),
            stream_id: self.stream_id.clone(),
            t_start_ns,
            t_end_ns,
            content_hash: format!("sha256:{}", sha256_hex(payload)),
            resolver: self.resolver.clone(),
            uri_hint: uri_hint.into(),
            media_type: self.media_type.clone(),
        }
    }
}

/// Verify resolved bytes against a ref's `content_hash`. Returns an
/// [`SidecarError::Unresolvable`] on mismatch so resolver implementations
/// can call this before handing bytes out.
pub fn verify_ref(r: &pb::ArchiveRef, payload: &[u8]) -> Result<(), SidecarError> {
    let expected = r
        .content_hash
        .strip_prefix("sha256:")
        .unwrap_or(&r.content_hash)
        .to_ascii_lowercase();
    let actual = sha256_hex(payload);
    if expected == actual {
        Ok(())
    } else {
        Err(SidecarError::Unresolvable {
            ref_id: r.ref_id.clone(),
            reason: format!("content hash mismatch: record pins {expected}, bytes are {actual}"),
        })
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut out = String::with_capacity(64);
    for b in digest {
        use std::fmt::Write;
        // Writing to a String cannot fail.
        let _ = write!(out, "{b:02x}");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn built_refs_carry_verifiable_hashes() {
        let builder =
            StreamRefBuilder::new("camera/overhead", "customer-s3").media_type("video/h264");
        let payload = b"pretend video bytes";
        let r = builder.build_ref("ref-1", 100, 200, payload, "s3://bucket/overhead.h264.mcap");
        assert_eq!(r.stream_id, "camera/overhead");
        assert_eq!(r.resolver, "customer-s3");
        assert_eq!(r.media_type, "video/h264");
        assert!(r.content_hash.starts_with("sha256:"));
        verify_ref(&r, payload).unwrap();
        assert!(matches!(
            verify_ref(&r, b"tampered"),
            Err(SidecarError::Unresolvable { .. })
        ));
    }
}
