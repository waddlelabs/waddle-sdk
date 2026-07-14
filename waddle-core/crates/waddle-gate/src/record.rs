//! Gate records: what happened at each tick, for the sidecar's provenance
//! spans and the local recorder. Pushed onto a wait-free SPSC ring; a full
//! ring drops (and counts) rather than ever blocking the caller's loop.

use waddle_types::ProvenanceTag;
use waddle_types::time::Stamp;

use crate::gate::OwnedAction;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateDecision {
    Pass,
    Substitute,
    Blend,
    Noop,
    Hold,
}

#[derive(Debug, Clone)]
pub struct GateRecord {
    pub stamp: Stamp,
    pub seq: u64,
    pub decision: GateDecision,
    pub provenance: ProvenanceTag,
    /// The action that left the gate (what the robot was asked to do), when
    /// one did.
    pub action: Option<OwnedAction>,
}
