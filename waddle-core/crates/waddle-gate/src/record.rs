//! Gate records: what happened at each tick, for the sidecar's provenance
//! spans and the local recorder. Pushed onto a wait-free SPSC ring; a full
//! ring drops (and counts) rather than ever blocking the caller's loop.

use waddle_types::time::Stamp;
use waddle_types::{ObsValues, ProvenanceTag};

use crate::gate::OwnedAction;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateDecision {
    Pass,
    Substitute,
    Blend,
    Noop,
    Hold,
    /// A remote actor is performing a scene reset (`PlanMode::Reset`); shaped
    /// like `Noop` (dispatch nothing) but distinct so the reducer's marker
    /// translation can render `NoopReason::RESET_ACTIVE` instead of
    /// `BYPASS_ACTIVE` (D7 edge 3).
    ResetActive,
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
    /// The observation the caller computed this tick's action from, when the
    /// caller supplied one. Recorded on every decision arm: Pass records are
    /// the training pairs; Substitute/Blend records are the pre-labeled
    /// DAgger pairs (obs + intervenor action).
    pub obs: Option<ObsValues>,
}
