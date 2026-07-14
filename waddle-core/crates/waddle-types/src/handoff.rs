//! Chunk-handoff policy (design doc §2.4; sequences in FSM.md §5).

use crate::error::TypesError;
use crate::pb::v0 as pb;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum HandoffPolicy {
    /// Drop the remaining chunk; cross-fade over `blend_ns` using the
    /// space's declared interpolation rule.
    Immediate { blend_ns: i64 },
    /// Finish the executing chunk (capped at `max_wait_ns`; 0 = the full
    /// remaining horizon), then switch.
    ChunkBoundary { max_wait_ns: i64 },
    /// Hold first; the intervenor starts from rest. The conservative default
    /// for advisory-lease integrations.
    HoldFirst,
}

impl HandoffPolicy {
    pub fn from_pb(p: &pb::HandoffPolicy) -> Result<Self, TypesError> {
        match p.policy.as_ref() {
            Some(pb::handoff_policy::Policy::Immediate(i)) => Ok(Self::Immediate {
                blend_ns: i.blend_ns,
            }),
            Some(pb::handoff_policy::Policy::ChunkBoundary(c)) => Ok(Self::ChunkBoundary {
                max_wait_ns: c.max_wait_ns,
            }),
            Some(pb::handoff_policy::Policy::HoldFirst(_)) => Ok(Self::HoldFirst),
            None => Err(TypesError::MissingField("HandoffPolicy.policy")),
        }
    }
}
