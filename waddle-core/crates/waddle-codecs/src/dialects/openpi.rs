//! The `openpi` dialect: openpi naming over a compact JSON wire.
//!
//! Observations:
//! `{"t": <ns>, "observation/state": [f64...], "prompt": str,
//!   "observation/images": {name: base64}}`
//!
//! Action chunks:
//! `{"actions": [[f64,...],...], "horizon_ns": ..., "t_obs_ns": ..., "seq": ...}`
//!
//! Upstream openpi frames its wire as msgpack; this crate avoids the extra
//! dependency and ships the same information as compact JSON with openpi's
//! key idiom (`observation/state`, `prompt`). The msgpack-exact wire ships
//! as a codec update on its own release cadence (N4) — swapping the framing
//! is a codec version bump, not a Waddle release.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use waddle_types::pb::v0 as pb;

use crate::descriptor::CodecDescriptor;
use crate::dialects::{b64, chunk_from_rows, rows_from_chunk};
use crate::traits::{Codec, CodecCaps, CodecError, ObsFrame};

const DIALECT: &str = "openpi";

#[derive(Serialize, Deserialize)]
struct WireObs {
    t: i64,
    #[serde(rename = "observation/state")]
    state: Vec<f64>,
    prompt: String,
    #[serde(
        rename = "observation/images",
        default,
        skip_serializing_if = "BTreeMap::is_empty"
    )]
    images: BTreeMap<String, String>,
}

#[derive(Serialize, Deserialize)]
struct WireChunk {
    actions: Vec<Vec<f64>>,
    horizon_ns: i64,
    t_obs_ns: i64,
    seq: u64,
}

/// The in-tree `openpi` codec. Same image-ordering note as
/// [`crate::LerobotAsyncCodec`]: decode yields images name-sorted.
#[derive(Debug, Clone)]
pub struct OpenPiCodec {
    descriptor: CodecDescriptor,
}

impl OpenPiCodec {
    #[must_use]
    pub fn new() -> Self {
        Self {
            descriptor: CodecDescriptor {
                name: "openpi-json".into(),
                dialect: DIALECT.into(),
                version: semver::Version::new(0, 1, 0),
                upstream_version: "openpi-0.2".into(),
                content_hash: String::new(),
                signature: Vec::new(),
                signer_key_id: String::new(),
            },
        }
    }
}

impl Default for OpenPiCodec {
    fn default() -> Self {
        Self::new()
    }
}

impl Codec for OpenPiCodec {
    fn descriptor(&self) -> &CodecDescriptor {
        &self.descriptor
    }

    fn caps(&self) -> CodecCaps {
        CodecCaps::Total
    }

    fn decode_obs(&self, wire: &[u8]) -> Result<ObsFrame, CodecError> {
        let w: WireObs = serde_json::from_slice(wire).map_err(|e| malformed(e.to_string()))?;
        let mut images = Vec::with_capacity(w.images.len());
        for (name, encoded) in w.images {
            let data =
                b64::decode(&encoded).map_err(|e| malformed(format!("image {name:?}: {e}")))?;
            images.push((name, bytes::Bytes::from(data)));
        }
        Ok(ObsFrame {
            t_ns: w.t,
            state: w.state,
            images,
            task: w.prompt,
        })
    }

    fn encode_obs(&self, obs: &ObsFrame) -> Result<Vec<u8>, CodecError> {
        let w = WireObs {
            t: obs.t_ns,
            state: obs.state.clone(),
            prompt: obs.task.clone(),
            images: obs
                .images
                .iter()
                .map(|(name, data)| (name.clone(), b64::encode(data)))
                .collect(),
        };
        serde_json::to_vec(&w).map_err(|e| malformed(e.to_string()))
    }

    fn decode_action(&self, wire: &[u8]) -> Result<pb::ActionChunk, CodecError> {
        let w: WireChunk = serde_json::from_slice(wire).map_err(|e| malformed(e.to_string()))?;
        Ok(chunk_from_rows(w.actions, w.horizon_ns, w.t_obs_ns, w.seq))
    }

    fn encode_action(&self, chunk: &pb::ActionChunk) -> Result<Vec<u8>, CodecError> {
        let w = WireChunk {
            actions: rows_from_chunk(DIALECT, chunk)?,
            horizon_ns: chunk.horizon_ns,
            t_obs_ns: chunk.t_obs_ns,
            seq: chunk.seq,
        };
        serde_json::to_vec(&w).map_err(|e| malformed(e.to_string()))
    }
}

fn malformed(reason: String) -> CodecError {
    CodecError::Malformed {
        dialect: DIALECT,
        reason,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn obs_round_trips_with_openpi_key_idiom() {
        let codec = OpenPiCodec::new();
        let obs = ObsFrame {
            t_ns: 9_000,
            state: vec![1.5, -0.5],
            images: vec![("base_0_rgb".into(), bytes::Bytes::from_static(b"\x10\x20"))],
            task: "pick up the mug".into(),
        };
        let wire = codec.encode_obs(&obs).unwrap();
        let text = std::str::from_utf8(&wire).unwrap();
        assert!(text.contains("\"observation/state\""));
        assert!(text.contains("\"prompt\""));
        assert!(text.contains("\"observation/images\""));
        assert_eq!(codec.decode_obs(&wire).unwrap(), obs);
    }

    #[test]
    fn action_round_trips() {
        let codec = OpenPiCodec::new();
        let chunk = crate::dialects::chunk_from_rows(
            vec![vec![0.0; 7], vec![0.1; 7], vec![0.2; 7]],
            300_000_000,
            123,
            42,
        );
        let wire = codec.encode_action(&chunk).unwrap();
        assert_eq!(codec.decode_action(&wire).unwrap(), chunk);
    }
}
