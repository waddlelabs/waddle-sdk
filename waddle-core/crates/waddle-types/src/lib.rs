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

pub use action::{
    ActionChunk, ActionValues, FlattenedChunk, ObsValues, PartPolicy, Step, flatten_action,
    unflatten_action,
};
pub use error::TypesError;
pub use grants::{Grant, GrantStatus, LeaseEnforcement, Verb};
pub use handoff::HandoffPolicy;
pub use ids::{
    CellId, ClaimId, ClientId, EpisodeId, FrameId, LeaseId, RobotId, SessionId, SourceId,
};
pub use outcome::{
    EpisodeStateKind, GateMode, InterventionPhase, ResetKind, ResetVerificationMode,
    TerminalOutcome,
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
    use prost::Message as _;

    use super::pb::v0 as pb;

    #[test]
    fn descriptor_set_is_embedded() {
        assert!(!super::FILE_DESCRIPTOR_SET.is_empty());
    }

    // -----------------------------------------------------------------------
    // waddle.v0.agent / waddle.v0.obs.stills wire surface.
    //
    // Per the reset-window precedent (EpisodeEvent arm 17, GateServerMessage
    // arm 6), event and service messages stay pb-only — no native mirrors
    // exist for ResetWindowEvent/ResetWindowDirective/ProprioSample, so none
    // are added for AgentInviteEvent/AgentTaskUpdate/FrameStill either
    // (`ActorKind::Agent` was already mirrored; NoopReason and StreamPolicy
    // have no native mirror to extend). These tests pin the regenerated
    // arms, their append-only wire numbers, and both-ways prost conversion.
    // -----------------------------------------------------------------------

    /// Field number of the first key in an encoding. Every protobuf encoding
    /// starts with a `(field << 3) | wire_type` varint; when all earlier
    /// fields are at their defaults (skipped), the first key is the field
    /// under test — pinning it pins the append-only wire number.
    fn leading_field_number(bytes: &[u8]) -> u32 {
        let mut key = 0u32;
        let mut shift = 0;
        for &b in bytes {
            key |= u32::from(b & 0x7f) << shift;
            if b & 0x80 == 0 {
                break;
            }
            shift += 7;
        }
        key >> 3
    }

    #[test]
    fn episode_event_agent_invite_round_trips_on_arm_18() {
        let ev = pb::EpisodeEvent {
            t_ns: 0,
            episode_id: String::new(),
            event: Some(pb::episode_event::Event::AgentInvite(
                pb::AgentInviteEvent {
                    prompt: "stack the cups".into(),
                    timeout_ns: 30_000_000_000,
                },
            )),
        };
        let bytes = ev.encode_to_vec();
        assert_eq!(leading_field_number(&bytes), 18);
        assert_eq!(pb::EpisodeEvent::decode(bytes.as_slice()).unwrap(), ev);
    }

    #[test]
    fn gate_server_message_agent_update_round_trips_on_arm_7() {
        let msg = pb::GateServerMessage {
            msg: Some(pb::gate_server_message::Msg::AgentUpdate(
                pb::AgentTaskUpdate {
                    episode_id: "ep-7".into(),
                    kind: pb::AgentTaskUpdateKind::Completed as i32,
                    detail: "task finished".into(),
                    recording_ref: "rec/2026/07/31/abc".into(),
                    directive_id: None,
                },
            )),
        };
        let bytes = msg.encode_to_vec();
        assert_eq!(leading_field_number(&bytes), 7);
        let decoded = pb::GateServerMessage::decode(bytes.as_slice()).unwrap();
        assert_eq!(decoded, msg);
        let Some(pb::gate_server_message::Msg::AgentUpdate(update)) = decoded.msg else {
            panic!("expected agent_update arm");
        };
        assert_eq!(
            pb::AgentTaskUpdateKind::try_from(update.kind),
            Ok(pb::AgentTaskUpdateKind::Completed)
        );
    }

    #[test]
    fn observation_update_still_round_trips_on_arm_5() {
        let update = pb::ObservationUpdate {
            t_ns: 0,
            payload: Some(pb::observation_update::Payload::Still(pb::FrameStill {
                camera: "wrist".into(),
                frame_seq: 42,
                encoding: pb::CameraEncoding::Jpeg as i32,
                width: 640,
                height: 480,
                data: vec![0xff, 0xd8, 0xff],
            })),
        };
        let bytes = update.encode_to_vec();
        assert_eq!(leading_field_number(&bytes), 5);
        assert_eq!(
            pb::ObservationUpdate::decode(bytes.as_slice()).unwrap(),
            update
        );
    }

    #[test]
    fn stream_policy_still_fps_round_trips_on_field_3() {
        let policy = pb::StreamPolicy {
            local_full_rate: false,
            uplink: None,
            still_fps: Some(2.0),
        };
        let bytes = policy.encode_to_vec();
        assert_eq!(leading_field_number(&bytes), 3);
        assert_eq!(pb::StreamPolicy::decode(bytes.as_slice()).unwrap(), policy);
        // Passthrough semantics: absent means absent (0/absent = no stills),
        // never a synthesized default.
        assert_eq!(pb::StreamPolicy::default().still_fps, None);
    }

    #[test]
    fn noop_reason_agent_episode_is_value_4() {
        assert_eq!(pb::NoopReason::AgentEpisode as i32, 4);
        assert_eq!(
            pb::NoopReason::try_from(4),
            Ok(pb::NoopReason::AgentEpisode)
        );
        assert_eq!(
            pb::NoopReason::AgentEpisode.as_str_name(),
            "NOOP_REASON_AGENT_EPISODE"
        );
        assert_eq!(
            pb::NoopReason::from_str_name("NOOP_REASON_AGENT_EPISODE"),
            Some(pb::NoopReason::AgentEpisode)
        );
    }

    #[test]
    fn agent_task_update_kind_values_are_pinned() {
        for (kind, value, name) in [
            (
                pb::AgentTaskUpdateKind::Unspecified,
                0,
                "AGENT_TASK_UPDATE_KIND_UNSPECIFIED",
            ),
            (
                pb::AgentTaskUpdateKind::Queued,
                1,
                "AGENT_TASK_UPDATE_KIND_QUEUED",
            ),
            (
                pb::AgentTaskUpdateKind::Denied,
                2,
                "AGENT_TASK_UPDATE_KIND_DENIED",
            ),
            (
                pb::AgentTaskUpdateKind::Completed,
                3,
                "AGENT_TASK_UPDATE_KIND_COMPLETED",
            ),
        ] {
            assert_eq!(kind as i32, value);
            assert_eq!(pb::AgentTaskUpdateKind::try_from(value), Ok(kind));
            assert_eq!(kind.as_str_name(), name);
            assert_eq!(pb::AgentTaskUpdateKind::from_str_name(name), Some(kind));
        }
    }
}
