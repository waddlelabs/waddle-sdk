//! waddle-types — protocol types for the Waddle reference implementation.
//!
//! Two layers, deliberately separate:
//!
//! - [`pb`] — the prost-generated wire types, compiled at build time from
//!   `waddle-protocol/proto` via protox (no system protoc). Public for wire,
//!   fixture, and FFI-config use ONLY.
//! - The domain layer (everything else) — validated types constructed from
//!   `pb` exactly once at boundaries via `TryFrom`. Units, frames, wxyz
//!   quaternions, and the two-clock discipline become unrepresentable-if-wrong
//!   here instead of conventions.
//!
//! No I/O, no clocks, no threads, no randomness in this crate.

pub mod action;
pub mod error;
pub mod grants;
pub mod handoff;
pub mod ids;
pub mod outcome;
pub mod provenance;
pub mod space;
pub mod time;
pub mod verb;

/// The prost-generated wire types. Wire, fixtures, and FFI config only —
/// domain code consumes the validated layer instead.
pub mod pb {
    #[allow(clippy::all, clippy::pedantic, missing_debug_implementations)]
    pub mod v0 {
        include!(concat!(env!("OUT_DIR"), "/waddle.v0.rs"));
    }
}

/// The compiled `FileDescriptorSet` for all six schema files. Consumers:
/// the MCAP recorder (protobuf channel schemas) and descriptor-driven config.
pub const FILE_DESCRIPTOR_SET: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/descriptor_set.bin"));

pub use action::{ActionChunk, ActionValues, ObsValues, Step};
pub use error::TypesError;
pub use grants::{Grant, GrantStatus, LeaseEnforcement, Verb};
pub use handoff::HandoffPolicy;
pub use ids::{
    CellId, ClaimId, ClientId, EpisodeId, FrameId, LeaseId, RobotId, SessionId, SourceId,
};
pub use outcome::{
    EpisodeStateKind, GateMode, InterventionPhase, ResetVerificationMode, TerminalOutcome,
};
pub use provenance::{ActorKind, ActorRef, Provenance, ProvenanceTag};
pub use space::{
    ActionSpace, Chunking, DeltaFrame, GripperKind, Interp, JointDescriptor, ReplanPolicy,
    RobotDescription, RotationEncoding, SpaceKind, SpaceSpec,
};
pub use time::{Clock, ClockAnchor, EpochNs, MonoNs, Stamp};
pub use verb::VerbRequest;

#[cfg(test)]
mod tests {
    #[test]
    fn descriptor_set_is_embedded() {
        assert!(!super::FILE_DESCRIPTOR_SET.is_empty());
    }
}
