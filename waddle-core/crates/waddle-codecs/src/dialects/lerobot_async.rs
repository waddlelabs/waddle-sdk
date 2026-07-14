//! The `lerobot-async` dialect: JSON wire, lerobot naming.
//!
//! Observations:
//! `{"t": <ns int>, "state": [f64...], "task": str, "images": {name: base64}}`
//!
//! Action chunks:
//! `{"actions": [[f64,...],...], "horizon_ns": ..., "t_obs_ns": ..., "seq": ...}`
//! mapping onto `pb::ActionChunk` with `joint_position` targets and
//! `t_offset_ns` spaced by `horizon_ns / len`.
//!
//! Real-but-minimal (N4): this carries the dialect's full information
//! content over readable JSON; the upstream-exact async transport ships as
//! a codec update on its own release cadence.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use waddle_types::pb::v0 as pb;

use crate::descriptor::CodecDescriptor;
use crate::dialects::{b64, chunk_from_rows, rows_from_chunk};
use crate::traits::{Codec, CodecCaps, CodecError, ObsFrame};

const DIALECT: &str = "lerobot-async";

#[derive(Serialize, Deserialize)]
struct WireObs {
    t: i64,
    state: Vec<f64>,
    task: String,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    images: BTreeMap<String, String>,
}

#[derive(Serialize, Deserialize)]
struct WireChunk {
    actions: Vec<Vec<f64>>,
    horizon_ns: i64,
    t_obs_ns: i64,
    seq: u64,
}

/// The in-tree `lerobot-async` codec.
///
/// Image ordering note: the wire carries images as a JSON object keyed by
/// camera name, so decode yields them name-sorted; frames whose `images`
/// are name-sorted round-trip exactly.
#[derive(Debug, Clone)]
pub struct LerobotAsyncCodec {
    descriptor: CodecDescriptor,
}

impl LerobotAsyncCodec {
    #[must_use]
    pub fn new() -> Self {
        Self {
            descriptor: CodecDescriptor {
                name: "lerobot-async-json".into(),
                dialect: DIALECT.into(),
                version: semver::Version::new(0, 1, 0),
                upstream_version: "lerobot-0.4".into(),
                content_hash: String::new(),
                signature: Vec::new(),
                signer_key_id: String::new(),
            },
        }
    }
}

impl Default for LerobotAsyncCodec {
    fn default() -> Self {
        Self::new()
    }
}

impl Codec for LerobotAsyncCodec {
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
            task: w.task,
        })
    }

    fn encode_obs(&self, obs: &ObsFrame) -> Result<Vec<u8>, CodecError> {
        let w = WireObs {
            t: obs.t_ns,
            state: obs.state.clone(),
            task: obs.task.clone(),
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
    fn obs_round_trips_including_images() {
        let codec = LerobotAsyncCodec::new();
        let obs = ObsFrame {
            t_ns: 1_000_000,
            state: vec![0.1, -0.2, 0.3],
            // Name-sorted, as the wire yields them.
            images: vec![
                (
                    "cam_high".into(),
                    bytes::Bytes::from_static(b"\x00\x01\xff"),
                ),
                ("cam_wrist".into(), bytes::Bytes::from_static(b"jpegish")),
            ],
            task: "fold_towel_half".into(),
        };
        let wire = codec.encode_obs(&obs).unwrap();
        assert_eq!(codec.decode_obs(&wire).unwrap(), obs);
    }

    #[test]
    fn obs_wire_shape_is_the_documented_json() {
        let codec = LerobotAsyncCodec::new();
        let wire = br#"{"t": 42, "state": [1.0, 2.0], "task": "stack", "images": {"cam": "AAE="}}"#;
        let obs = codec.decode_obs(wire).unwrap();
        assert_eq!(obs.t_ns, 42);
        assert_eq!(obs.state, vec![1.0, 2.0]);
        assert_eq!(obs.task, "stack");
        assert_eq!(
            obs.images,
            vec![("cam".into(), bytes::Bytes::from_static(b"\x00\x01"))]
        );
    }

    #[test]
    fn action_round_trips_with_offsets_spaced_by_horizon() {
        let codec = LerobotAsyncCodec::new();
        let wire = br#"{"actions": [[0.1, 0.2], [0.3, 0.4]], "horizon_ns": 200000000, "t_obs_ns": 5000, "seq": 7}"#;
        let chunk = codec.decode_action(wire).unwrap();
        assert_eq!(chunk.actions.len(), 2);
        assert_eq!(chunk.actions[0].t_offset_ns, 0);
        assert_eq!(chunk.actions[1].t_offset_ns, 100_000_000);
        assert_eq!(chunk.horizon_ns, 200_000_000);
        assert_eq!(chunk.t_obs_ns, 5_000);
        assert_eq!(chunk.seq, 7);
        let back = codec
            .decode_action(&codec.encode_action(&chunk).unwrap())
            .unwrap();
        assert_eq!(back, chunk);
    }

    #[test]
    fn non_joint_chunks_are_not_representable() {
        let codec = LerobotAsyncCodec::new();
        let chunk = pb::ActionChunk {
            actions: vec![pb::Action {
                target: Some(pb::action::Target::Opaque(vec![1, 2, 3])),
                ..Default::default()
            }],
            ..Default::default()
        };
        assert!(matches!(
            codec.encode_action(&chunk),
            Err(CodecError::NotRepresentable { .. })
        ));
    }
}
