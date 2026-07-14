//! Codec artifact verification.
//!
//! The registry never has an opinion about *how* codecs are signed — the
//! deployment provides a [`SignatureVerifier`]. Two implementations ship
//! here: [`Sha256ContentPin`] (real content pinning, no key material) and
//! [`InsecureAcceptAll`] (a development stand-in that is loudly NOT
//! cryptography).

use sha2::{Digest, Sha256};

use crate::descriptor::CodecDescriptor;

/// Errors from signature/content verification.
#[derive(Debug, thiserror::Error)]
pub enum SignatureError {
    /// The descriptor carries no `content_hash` to pin against.
    #[error("codec descriptor {name:?} carries no content_hash")]
    MissingContentHash { name: String },
    /// The artifact's sha256 does not match the descriptor's pin.
    #[error("codec artifact content mismatch: descriptor pins {expected}, artifact is {actual}")]
    ContentMismatch { expected: String, actual: String },
    /// A cryptographic check failed (bad signature, unknown key, ...).
    #[error("signature verification failed: {0}")]
    Rejected(String),
}

/// Verifies that a codec artifact is the one its descriptor claims.
pub trait SignatureVerifier: Send + Sync {
    /// Verify `artifact` against descriptor `d`. `Ok(())` means "this
    /// artifact may be loaded"; any error means it must not be.
    fn verify(&self, d: &CodecDescriptor, artifact: &[u8]) -> Result<(), SignatureError>;
}

/// **NOT REAL CRYPTOGRAPHY. Development and tests only.**
///
/// Accepts every artifact unconditionally: no hash check, no signature
/// check, no key material. Deployments MUST replace this with a verifier
/// backed by their key infrastructure before loading codecs from anywhere
/// but their own build tree. It exists so integration tests and local
/// development do not need a signing pipeline — nothing more.
#[derive(Debug, Clone, Copy, Default)]
pub struct InsecureAcceptAll;

impl SignatureVerifier for InsecureAcceptAll {
    fn verify(&self, _d: &CodecDescriptor, _artifact: &[u8]) -> Result<(), SignatureError> {
        Ok(())
    }
}

/// Content pinning: verifies `descriptor.content_hash` equals the sha256 of
/// the artifact bytes. This is a real integrity check (the artifact is
/// exactly the bytes the descriptor names) but NOT an authenticity check —
/// it proves nothing about who produced the descriptor. The hash may be
/// written with or without a `"sha256:"` prefix; comparison is
/// case-insensitive hex.
#[derive(Debug, Clone, Copy, Default)]
pub struct Sha256ContentPin;

impl SignatureVerifier for Sha256ContentPin {
    fn verify(&self, d: &CodecDescriptor, artifact: &[u8]) -> Result<(), SignatureError> {
        if d.content_hash.is_empty() {
            return Err(SignatureError::MissingContentHash {
                name: d.name.clone(),
            });
        }
        let expected = d
            .content_hash
            .strip_prefix("sha256:")
            .unwrap_or(&d.content_hash)
            .to_ascii_lowercase();
        let actual = sha256_hex(artifact);
        if expected == actual {
            Ok(())
        } else {
            Err(SignatureError::ContentMismatch {
                expected: d.content_hash.clone(),
                actual,
            })
        }
    }
}

/// Lowercase hex sha256 of `bytes`.
#[must_use]
pub fn sha256_hex(bytes: &[u8]) -> String {
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

    fn descriptor(content_hash: &str) -> CodecDescriptor {
        CodecDescriptor {
            name: "test".into(),
            dialect: "test".into(),
            version: semver::Version::new(1, 0, 0),
            upstream_version: String::new(),
            content_hash: content_hash.into(),
            signature: Vec::new(),
            signer_key_id: String::new(),
        }
    }

    #[test]
    fn content_pin_accepts_matching_artifact() {
        let artifact = b"codec bytes";
        let d = descriptor(&format!("sha256:{}", sha256_hex(artifact)));
        Sha256ContentPin.verify(&d, artifact).unwrap();
        // Unprefixed hash is accepted too.
        let d = descriptor(&sha256_hex(artifact));
        Sha256ContentPin.verify(&d, artifact).unwrap();
    }

    #[test]
    fn content_pin_rejects_mismatch_and_missing() {
        let d = descriptor(&format!("sha256:{}", sha256_hex(b"other bytes")));
        assert!(matches!(
            Sha256ContentPin.verify(&d, b"codec bytes"),
            Err(SignatureError::ContentMismatch { .. })
        ));
        assert!(matches!(
            Sha256ContentPin.verify(&descriptor(""), b"codec bytes"),
            Err(SignatureError::MissingContentHash { .. })
        ));
    }

    #[test]
    fn accept_all_accepts_anything() {
        InsecureAcceptAll
            .verify(&descriptor(""), b"whatever")
            .unwrap();
    }
}
