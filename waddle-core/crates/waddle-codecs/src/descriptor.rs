//! The codec identity record: who a codec is, what dialect it speaks, and
//! the material a [`crate::SignatureVerifier`] checks before the registry
//! will touch it.

use waddle_types::pb::v0 as pb;

/// Errors converting a wire `CodecDescriptor` into the validated form.
#[derive(Debug, thiserror::Error)]
pub enum DescriptorError {
    /// The wire `version` field is not a valid semver version.
    #[error("codec version {value:?} is not valid semver: {source}")]
    InvalidVersion {
        value: String,
        #[source]
        source: semver::Error,
    },
}

/// Validated codec identity. The wire twin is
/// [`waddle_types::pb::v0::CodecDescriptor`]; the difference is that
/// `version` is a parsed [`semver::Version`] here, so registry resolution
/// can never operate on an unparseable version string.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CodecDescriptor {
    /// Human-readable codec name, e.g. `"lerobot-async-json"`.
    pub name: String,
    /// The dialect this codec speaks, e.g. `"lerobot-async"`, `"openpi"`.
    /// Registry lookup keys on this string.
    pub dialect: String,
    /// The codec's own semver — independent of the workspace version (N4).
    pub version: semver::Version,
    /// The upstream wire version this codec speaks, in the upstream
    /// ecosystem's own versioning idiom (opaque to Waddle).
    pub upstream_version: String,
    /// sha256 of the codec artifact, optionally prefixed `"sha256:"`.
    pub content_hash: String,
    /// Detached signature over the artifact; scheme is deployment-defined.
    pub signature: Vec<u8>,
    /// Key id the deployment's verifier resolves to a public key.
    pub signer_key_id: String,
}

impl CodecDescriptor {
    #[must_use]
    pub fn to_pb(&self) -> pb::CodecDescriptor {
        pb::CodecDescriptor {
            name: self.name.clone(),
            dialect: self.dialect.clone(),
            version: self.version.to_string(),
            upstream_version: self.upstream_version.clone(),
            content_hash: self.content_hash.clone(),
            signature: self.signature.clone(),
            signer_key_id: self.signer_key_id.clone(),
        }
    }
}

impl TryFrom<&pb::CodecDescriptor> for CodecDescriptor {
    type Error = DescriptorError;

    fn try_from(d: &pb::CodecDescriptor) -> Result<Self, Self::Error> {
        let version = semver::Version::parse(&d.version).map_err(|source| {
            DescriptorError::InvalidVersion {
                value: d.version.clone(),
                source,
            }
        })?;
        Ok(Self {
            name: d.name.clone(),
            dialect: d.dialect.clone(),
            version,
            upstream_version: d.upstream_version.clone(),
            content_hash: d.content_hash.clone(),
            signature: d.signature.clone(),
            signer_key_id: d.signer_key_id.clone(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn descriptor() -> CodecDescriptor {
        CodecDescriptor {
            name: "lerobot-async-json".into(),
            dialect: "lerobot-async".into(),
            version: semver::Version::new(0, 1, 0),
            upstream_version: "lerobot-0.4".into(),
            content_hash: "sha256:00".into(),
            signature: vec![1, 2, 3],
            signer_key_id: "dev-key".into(),
        }
    }

    #[test]
    fn pb_round_trip() {
        let d = descriptor();
        let back = CodecDescriptor::try_from(&d.to_pb()).unwrap();
        assert_eq!(d, back);
    }

    #[test]
    fn invalid_semver_is_rejected() {
        let mut wire = descriptor().to_pb();
        wire.version = "latest".into();
        assert!(matches!(
            CodecDescriptor::try_from(&wire),
            Err(DescriptorError::InvalidVersion { value, .. }) if value == "latest"
        ));
    }
}
