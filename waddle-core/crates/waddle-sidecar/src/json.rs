//! Canonical proto3 JSON for sidecar records.
//!
//! The JSON on disk is exactly the fixture dialect
//! (`waddle-protocol/fixtures/README.md`): lowerCamelCase field names,
//! `int64`/`uint64` as decimal strings, enums as full prefixed names,
//! defaults omitted. This is achieved by transcoding the prost message
//! through a `prost_reflect::DynamicMessage` built from the descriptor set
//! embedded in `waddle-types` — one schema, one JSON dialect, no parallel
//! serde model to drift.
//!
//! Reading is tolerant of unknown fields (sidecar evolution is append-only;
//! an older reader must not choke on a newer writer's fields). Note this is
//! deliberately laxer than the fixture suite's own conformance rule — a
//! fixture with an unknown field fails conformance, but a *reader* of
//! production sidecars must keep working.

use std::sync::OnceLock;

use prost_reflect::{DescriptorPool, DeserializeOptions, DynamicMessage, MessageDescriptor};
use waddle_types::pb::v0 as pb;

use crate::error::SidecarError;

/// The process-wide descriptor pool, decoded once from the descriptor set
/// compiled into `waddle-types` at build time.
fn pool() -> &'static DescriptorPool {
    static POOL: OnceLock<DescriptorPool> = OnceLock::new();
    POOL.get_or_init(|| {
        DescriptorPool::decode(waddle_types::FILE_DESCRIPTOR_SET)
            .expect("waddle-types embeds a valid FILE_DESCRIPTOR_SET (built by its build.rs)")
    })
}

fn descriptor(type_name: &str) -> Result<MessageDescriptor, SidecarError> {
    pool()
        .get_message_by_name(type_name)
        .ok_or_else(|| SidecarError::MissingDescriptor(type_name.to_owned()))
}

/// Serialize any `waddle.v0` message to canonical proto3 JSON.
pub fn message_to_json<M: prost::Message>(
    type_name: &str,
    msg: &M,
) -> Result<String, SidecarError> {
    let desc = descriptor(type_name)?;
    let mut dynamic = DynamicMessage::new(desc);
    dynamic.transcode_from(msg)?;
    // prost-reflect's default `SerializeOptions` ARE the canonical proto3
    // JSON mapping: stringified 64-bit ints, enum names, lowerCamelCase,
    // defaults skipped.
    Ok(serde_json::to_string(&dynamic)?)
}

/// Parse any `waddle.v0` message from canonical proto3 JSON. Unknown fields
/// are ignored (append-only schema evolution; see module docs).
pub fn message_from_json<M: prost::Message + Default>(
    type_name: &str,
    json: &str,
) -> Result<M, SidecarError> {
    let desc = descriptor(type_name)?;
    let mut de = serde_json::Deserializer::from_str(json);
    let options = DeserializeOptions::new().deny_unknown_fields(false);
    let dynamic = DynamicMessage::deserialize_with_options(desc, &mut de, &options)?;
    de.end()?;
    Ok(dynamic.transcode_to::<M>()?)
}

/// Canonical proto3 JSON for a sidecar record.
pub fn sidecar_to_json(s: &pb::Sidecar) -> Result<String, SidecarError> {
    message_to_json("waddle.v0.Sidecar", s)
}

/// Parse a sidecar record from canonical proto3 JSON.
pub fn sidecar_from_json(json: &str) -> Result<pb::Sidecar, SidecarError> {
    message_from_json("waddle.v0.Sidecar", json)
}

/// Canonical proto3 JSON for one episode event.
pub fn event_to_json(e: &pb::EpisodeEvent) -> Result<String, SidecarError> {
    message_to_json("waddle.v0.EpisodeEvent", e)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_json_uses_camel_case_string_int64_and_enum_names() {
        let s = pb::Sidecar {
            sidecar_version: 1,
            episode_id: "ep-1".into(),
            t_start_unix_ns: 1_784_000_000_000_000_000,
            outcome: pb::TerminalOutcome::Success as i32,
            ..Default::default()
        };
        let json = sidecar_to_json(&s).unwrap();
        let v: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(v["episodeId"], "ep-1");
        assert_eq!(v["tStartUnixNs"], "1784000000000000000");
        assert_eq!(v["outcome"], "TERMINAL_OUTCOME_SUCCESS");
        // Defaults are omitted.
        assert!(v.get("project").is_none());
        assert!(v.get("bornClaimed").is_none());
    }

    #[test]
    fn unknown_fields_are_tolerated_on_read() {
        let json = r#"{"sidecarVersion": 1, "episodeId": "ep-2", "aFieldFromTheFuture": true}"#;
        let s = sidecar_from_json(json).unwrap();
        assert_eq!(s.episode_id, "ep-2");
    }

    #[test]
    fn event_oneof_serializes_as_single_arm() {
        let e = pb::EpisodeEvent {
            t_ns: 42,
            episode_id: "ep-3".into(),
            event: Some(pb::episode_event::Event::State(pb::StateTransition {
                from: pb::EpisodeState::Running as i32,
                to: pb::EpisodeState::Terminal as i32,
                reason: "done".into(),
                outcome: pb::TerminalOutcome::Success as i32,
            })),
        };
        let v: serde_json::Value = serde_json::from_str(&event_to_json(&e).unwrap()).unwrap();
        assert_eq!(v["tNs"], "42");
        assert_eq!(v["state"]["to"], "EPISODE_STATE_TERMINAL");
        assert!(v.get("claim").is_none());
    }
}
