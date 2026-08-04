//! The emission log: everything the target emitted, in order, as JSON values
//! matchable by `expect_emission` / `expect_send`.
//!
//! Entries are `{"event": <canonical proto3 JSON of waddle.v0.EpisodeEvent>}`
//! or `{"effect": {<snake_case effect name>: {fields…}}}` per
//! scenario-format.md's effects table.

use prost::Message;
use prost_reflect::{DescriptorPool, DynamicMessage, SerializeOptions};
use serde_json::{Value, json};
use waddle_fsm::TimerId;
use waddle_fsm::effect::{Effect, LeaseOpKind};
use waddle_types::{
    GateMode, GrantStatus, InterventionPhase, ResetVerificationMode, Verb, pb::v0 as pb,
};

use crate::ConformanceError;

/// One emitted event or effect, stamped with the virtual time it was
/// produced at.
#[derive(Debug, Clone)]
pub struct EmissionEntry {
    pub at_ns: i64,
    pub value: Value,
}

/// prost-reflect bridge: canonical proto3 JSON in and out, driven by the
/// descriptor set embedded in `waddle-types`.
#[derive(Debug, Clone)]
pub struct Codec {
    pool: DescriptorPool,
}

impl Codec {
    pub fn new() -> Result<Self, ConformanceError> {
        let pool = DescriptorPool::decode(waddle_types::FILE_DESCRIPTOR_SET)
            .map_err(|e| crate::scenario_err(format!("descriptor set: {e}")))?;
        Ok(Self { pool })
    }

    fn descriptor(
        &self,
        full_name: &str,
    ) -> Result<prost_reflect::MessageDescriptor, ConformanceError> {
        self.pool
            .get_message_by_name(full_name)
            .ok_or_else(|| crate::scenario_err(format!("unknown message type {full_name}")))
    }

    /// Parse canonical proto3 JSON into a generated prost type.
    pub fn parse<T: Message + Default>(
        &self,
        full_name: &str,
        value: &Value,
    ) -> Result<T, ConformanceError> {
        let desc = self.descriptor(full_name)?;
        let dynamic = DynamicMessage::deserialize(desc, value.clone())
            .map_err(|e| crate::scenario_err(format!("{full_name}: {e}")))?;
        dynamic
            .transcode_to::<T>()
            .map_err(|e| crate::scenario_err(format!("{full_name} transcode: {e}")))
    }

    /// Parse canonical proto3 JSON against a descriptor named at RUNTIME,
    /// for a payload whose type is a string rather than a Rust type — a wire
    /// fixture's `type` envelope field. Strict: an unknown field or a value
    /// the descriptor cannot take is an error, which is the whole point of
    /// running a fixture through this.
    pub fn parse_dynamic(
        &self,
        full_name: &str,
        value: &Value,
    ) -> Result<DynamicMessage, ConformanceError> {
        let desc = self.descriptor(full_name)?;
        DynamicMessage::deserialize(desc, value.clone())
            .map_err(|e| crate::scenario_err(format!("{full_name}: {e}")))
    }

    /// Canonical proto3 JSON of an already-parsed dynamic message (the
    /// serializing half of [`Codec::parse_dynamic`]).
    pub fn dynamic_to_value(&self, msg: &DynamicMessage) -> Result<Value, ConformanceError> {
        let opts = SerializeOptions::new().skip_default_fields(false);
        let mut buf = Vec::new();
        let mut ser = serde_json::Serializer::new(&mut buf);
        msg.serialize_with_options(&mut ser, &opts)
            .map_err(|e| crate::scenario_err(format!("serialize: {e}")))?;
        Ok(serde_json::from_slice(&buf)?)
    }

    /// Serialize a generated prost type to canonical proto3 JSON
    /// (lowerCamelCase fields, int64 as strings, enums as full names),
    /// with default-valued fields present so partial matches like
    /// `{"verified": false}` resolve.
    pub fn to_value<T: Message>(
        &self,
        full_name: &str,
        msg: &T,
    ) -> Result<Value, ConformanceError> {
        let desc = self.descriptor(full_name)?;
        let dynamic = DynamicMessage::decode(desc, msg.encode_to_vec().as_slice())
            .map_err(|e| crate::scenario_err(format!("{full_name} decode: {e}")))?;
        let opts = SerializeOptions::new().skip_default_fields(false);
        let mut buf = Vec::new();
        let mut ser = serde_json::Serializer::new(&mut buf);
        dynamic
            .serialize_with_options(&mut ser, &opts)
            .map_err(|e| crate::scenario_err(format!("{full_name} serialize: {e}")))?;
        Ok(serde_json::from_slice(&buf)?)
    }

    pub fn event_to_value(&self, ev: &pb::EpisodeEvent) -> Result<Value, ConformanceError> {
        Ok(json!({ "event": self.to_value("waddle.v0.EpisodeEvent", ev)? }))
    }

    pub fn provenance_to_value(&self, tag: &pb::ProvenanceTag) -> Result<Value, ConformanceError> {
        self.to_value("waddle.v0.ProvenanceTag", tag)
    }
}

pub fn timer_name(id: TimerId) -> &'static str {
    match id {
        TimerId::EngageTimeout => "engage_timeout",
        TimerId::ChunkBoundaryCap => "chunk_boundary_cap",
        TimerId::HeartbeatStale => "heartbeat_stale",
        TimerId::ResetWindowTimeout => "reset_window_timeout",
        TimerId::AgentInviteTimeout => "agent_invite_timeout",
    }
}

pub fn gate_mode_name(mode: GateMode) -> &'static str {
    mode.to_pb().as_str_name()
}

pub fn verb_name(verb: Verb) -> &'static str {
    verb.to_pb().as_str_name()
}

pub fn grant_status_name(status: GrantStatus) -> &'static str {
    match status {
        GrantStatus::Active => pb::GrantStatus::Active.as_str_name(),
        GrantStatus::Demoted => pb::GrantStatus::Demoted.as_str_name(),
        GrantStatus::Revoked => pb::GrantStatus::Revoked.as_str_name(),
    }
}

pub fn verification_mode_name(mode: ResetVerificationMode) -> &'static str {
    match mode {
        ResetVerificationMode::Blocking => pb::ResetVerificationMode::Blocking.as_str_name(),
        ResetVerificationMode::OptimisticAsync => {
            pb::ResetVerificationMode::OptimisticAsync.as_str_name()
        }
    }
}

pub fn intervention_phase_name(phase: InterventionPhase) -> &'static str {
    match phase {
        InterventionPhase::Engage => pb::InterventionPhase::Engage.as_str_name(),
        InterventionPhase::Settle => pb::InterventionPhase::Settle.as_str_name(),
        InterventionPhase::Release => pb::InterventionPhase::Release.as_str_name(),
        InterventionPhase::Retake => pb::InterventionPhase::Retake.as_str_name(),
    }
}

/// Render a non-`Emit` effect as its scenario-format JSON form. `Emit` is
/// handled by [`Codec::event_to_value`]; returns `None` for it.
pub fn effect_to_value(effect: &Effect) -> Option<Value> {
    let body = match effect {
        Effect::Emit(_) => return None,
        Effect::SetGateMode(mode) => json!({ "set_gate_mode": { "mode": gate_mode_name(*mode) } }),
        Effect::RequestVerb(verb) => json!({ "request_verb": { "verb": verb_name(*verb) } }),
        Effect::ArmTimer { id, deadline } => json!({ "arm_timer": {
            "timer_id": timer_name(*id),
            "deadline_ns": deadline.0.to_string(),
        } }),
        Effect::CancelTimer { id } => {
            json!({ "cancel_timer": { "timer_id": timer_name(*id) } })
        }
        Effect::OpenSuccessor {
            predecessor,
            successor,
            claim,
            born_claimed,
            mode,
        } => json!({ "open_successor": {
            "predecessor_episode_id": predecessor.as_str(),
            "successor_episode_id": successor.as_str(),
            "claim_id": claim.as_str(),
            "born_claimed": born_claimed,
            "verification_mode": verification_mode_name(*mode),
        } }),
        Effect::MintLeaseToken(pending) => {
            let to = match &pending.op {
                LeaseOpKind::Acquire { client } => client.as_str().to_owned(),
                LeaseOpKind::Handoff { to, .. } => to.as_str().to_owned(),
            };
            json!({ "mint_lease_token": { "to_client_id": to } })
        }
        Effect::ReprimePolicy => json!({ "reprime_policy": {} }),
        Effect::SetResetUnverified { .. } => {
            json!({ "set_flag": { "flag": "reset_unverified" } })
        }
        Effect::SetPostResetFailed { .. } => {
            json!({ "set_flag": { "flag": "post_reset_failed" } })
        }
        // The post-reset hook trigger is a runtime seam, not a scenario-visible
        // effect (the reset pump answers it with a PostResetResult inject).
        Effect::RunPostReset { .. } => return None,
    };
    Some(json!({ "effect": body }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use waddle_types::{EpisodeId, EpisodeStateKind, MonoNs};

    /// The matcher relies on two properties of the canonical JSON we emit:
    /// default-valued scalar fields are PRESENT (so `{"verified": false}`
    /// matches), and unset oneof arms are ABSENT (so `{"state": {}}` never
    /// matches a fault event).
    #[test]
    fn canonical_json_keeps_defaults_and_omits_unset_oneof_arms() {
        let codec = Codec::new().unwrap();
        let ep = EpisodeId::new("ep-1");
        let ev = waddle_fsm::emit::reset_verification(
            MonoNs(5),
            &ep,
            waddle_types::ResetVerificationMode::Blocking,
            false,
            false,
        );
        let v = codec.event_to_value(&ev).unwrap();
        let rv = &v["event"]["resetVerification"];
        assert_eq!(rv["verified"], Value::Bool(false), "{v}");
        assert!(v["event"].get("state").is_none(), "unset oneof arm: {v}");
        assert!(v["event"].get("fault").is_none(), "unset oneof arm: {v}");

        let ev = waddle_fsm::emit::state_transition(
            MonoNs(7),
            &ep,
            Some(EpisodeStateKind::Resetting),
            EpisodeStateKind::Ready,
            "reset verified",
            None,
        );
        let v = codec.event_to_value(&ev).unwrap();
        assert_eq!(v["event"]["state"]["to"], "EPISODE_STATE_READY");
        assert_eq!(
            v["event"]["state"]["outcome"],
            "TERMINAL_OUTCOME_UNSPECIFIED"
        );
        assert_eq!(v["event"]["tNs"], "7");
    }
}
