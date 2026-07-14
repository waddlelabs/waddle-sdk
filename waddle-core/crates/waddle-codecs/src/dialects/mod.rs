//! In-tree dialects.
//!
//! These are real-but-minimal: they carry the full information content of
//! the dialect (state, task, images, joint-space chunks) over a JSON wire
//! that is easy to certify and to read in a debugger. The upstream-exact
//! wire formats (lerobot's async transport, openpi's msgpack framing) ship
//! as codec updates on their own release cadence (N4) — that is the whole
//! point of independently versioned codecs.

pub mod lerobot_async;
pub mod openpi;

pub(crate) mod b64;

use waddle_types::pb::v0 as pb;

use crate::traits::CodecError;

/// Build a `pb::ActionChunk` from joint-space rows. Step offsets are implied
/// by the wire (`t_offset_ns = i * horizon_ns / len`); everything the wire
/// does not carry (`t_emitted_ns`, `source_id`, `provenance`) is left at its
/// proto3 default.
pub(crate) fn chunk_from_rows(
    rows: Vec<Vec<f64>>,
    horizon_ns: i64,
    t_obs_ns: i64,
    seq: u64,
) -> pb::ActionChunk {
    let step = if rows.is_empty() {
        0
    } else {
        horizon_ns / rows.len() as i64
    };
    let actions = rows
        .into_iter()
        .enumerate()
        .map(|(i, values)| pb::Action {
            gripper: None,
            t_offset_ns: i as i64 * step,
            part: String::new(),
            target: Some(pb::action::Target::JointPosition(pb::JointVector {
                values,
            })),
        })
        .collect();
    pb::ActionChunk {
        actions,
        horizon_ns,
        t_emitted_ns: 0,
        t_obs_ns,
        seq,
        source_id: String::new(),
        provenance: None,
    }
}

/// Extract joint-space rows from a `pb::ActionChunk`, rejecting anything the
/// joints-only wire cannot represent. Fields the wire does not carry
/// (`t_offset_ns` spacing, `t_emitted_ns`, `source_id`, `provenance`) are
/// dropped silently — certification fixtures are what pin the representable
/// subset, and a fixture outside it fails certification loudly.
pub(crate) fn rows_from_chunk(
    dialect: &'static str,
    chunk: &pb::ActionChunk,
) -> Result<Vec<Vec<f64>>, CodecError> {
    chunk
        .actions
        .iter()
        .map(|a| match &a.target {
            Some(pb::action::Target::JointPosition(v)) => Ok(v.values.clone()),
            other => Err(CodecError::NotRepresentable {
                dialect,
                reason: format!(
                    "only joint_position actions exist on this wire, got {:?}",
                    other.as_ref().map(std::mem::discriminant)
                ),
            }),
        })
        .collect()
}
