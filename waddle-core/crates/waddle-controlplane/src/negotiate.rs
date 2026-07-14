//! Grant negotiation (N6/N7): builds the NegotiateRequest from domain
//! declarations and folds the response into initial grant states.

use waddle_types::pb::v0 as pb;
use waddle_types::{Grant, GrantStatus, HandoffPolicy, LeaseEnforcement};

#[derive(Debug, Clone)]
pub struct NegotiationInputs {
    pub session_id: String,
    pub grants: Vec<Grant>,
    pub enforcement: LeaseEnforcement,
    pub handoff: HandoffPolicy,
    pub recording_mode: pb::recording_mode_declaration::Mode,
    /// N13: random retention quota in basis points; 0 = declined (judge
    /// metrics permanently marked unaudited).
    pub audit_quota_bp: u32,
    pub codecs: Vec<pb::CodecDescriptor>,
    pub feature_flags: Vec<String>,
}

fn grant_to_pb(g: &Grant) -> pb::Grant {
    pb::Grant {
        verb: g.verb.to_pb() as i32,
        send_interfaces: g.send_interfaces.iter().map(|s| s.to_pb() as i32).collect(),
        declared_latency_bound_ns: g.declared_latency_bound_ns,
        hardware: g.hardware,
    }
}

fn handoff_to_pb(h: HandoffPolicy) -> pb::HandoffPolicy {
    pb::HandoffPolicy {
        policy: Some(match h {
            HandoffPolicy::Immediate { blend_ns } => {
                pb::handoff_policy::Policy::Immediate(pb::handoff_policy::Immediate { blend_ns })
            }
            HandoffPolicy::ChunkBoundary { max_wait_ns } => {
                pb::handoff_policy::Policy::ChunkBoundary(pb::handoff_policy::ChunkBoundary {
                    max_wait_ns,
                })
            }
            HandoffPolicy::HoldFirst => {
                pb::handoff_policy::Policy::HoldFirst(pb::handoff_policy::HoldFirst {})
            }
        }),
    }
}

#[must_use]
pub fn build_request(inputs: &NegotiationInputs) -> pb::NegotiateRequest {
    pb::NegotiateRequest {
        session_id: inputs.session_id.clone(),
        declared_grants: inputs.grants.iter().map(grant_to_pb).collect(),
        lease_enforcement: match inputs.enforcement {
            LeaseEnforcement::Enforced => pb::LeaseEnforcement::Enforced as i32,
            LeaseEnforcement::Advisory => pb::LeaseEnforcement::Advisory as i32,
        },
        handoff: Some(handoff_to_pb(inputs.handoff)),
        recording: Some(pb::RecordingModeDeclaration {
            mode: inputs.recording_mode as i32,
            audit_quota_bp: inputs.audit_quota_bp,
        }),
        codecs: inputs.codecs.clone(),
        feature_flags: inputs.feature_flags.clone(),
    }
}

/// Initial per-verb grant status from the negotiate response.
#[must_use]
pub fn fold_response(resp: &pb::NegotiateResponse) -> Vec<(waddle_types::Verb, GrantStatus)> {
    resp.grants
        .iter()
        .filter_map(|gs| {
            let verb = waddle_types::Verb::from_pb(gs.grant.as_ref()?.verb).ok()?;
            let status = match pb::GrantStatus::try_from(gs.status) {
                Ok(pb::GrantStatus::Demoted) => GrantStatus::Demoted,
                Ok(pb::GrantStatus::Revoked) => GrantStatus::Revoked,
                _ => GrantStatus::Active,
            };
            Some((verb, status))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use waddle_types::{SpaceKind, Verb};

    #[test]
    fn request_carries_enforcement_audit_quota_and_codecs() {
        let req = build_request(&NegotiationInputs {
            session_id: "s1".into(),
            grants: vec![Grant {
                verb: Verb::Send,
                send_interfaces: vec![SpaceKind::JointPosition, SpaceKind::EePoseDelta],
                declared_latency_bound_ns: None,
                hardware: false,
            }],
            enforcement: LeaseEnforcement::Advisory,
            handoff: HandoffPolicy::HoldFirst,
            recording_mode: pb::recording_mode_declaration::Mode::Local,
            audit_quota_bp: 500,
            codecs: vec![],
            feature_flags: vec!["waddle.v0.core".into()],
        });
        assert_eq!(req.lease_enforcement, pb::LeaseEnforcement::Advisory as i32);
        assert_eq!(req.recording.as_ref().unwrap().audit_quota_bp, 500);
        assert_eq!(req.declared_grants[0].send_interfaces.len(), 2);
    }
}
