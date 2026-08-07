//! Builders for the `pb::EpisodeEvent`s the session machine emits. Emissions
//! are real wire messages so every consumer (sidecar, recorder, control
//! plane, conformance runner) sees exactly one vocabulary.

use std::collections::BTreeMap;

use waddle_types::{
    ActorKind, ClaimId, ClientId, EpisodeId, EpisodeStateKind, GateMode, GrantStatus,
    InterventionPhase, LeaseEnforcement, LeaseId, MonoNs, ResetKind, TerminalOutcome, Verb,
    pb::v0 as pb,
};

use crate::claim::ActiveClaim;

fn base(at: MonoNs, episode: Option<&EpisodeId>) -> pb::EpisodeEvent {
    pb::EpisodeEvent {
        t_ns: at.0,
        episode_id: episode.map(|e| e.as_str().to_owned()).unwrap_or_default(),
        event: None,
    }
}

fn phase_pb(phase: InterventionPhase) -> pb::InterventionPhase {
    match phase {
        InterventionPhase::Engage => pb::InterventionPhase::Engage,
        InterventionPhase::Settle => pb::InterventionPhase::Settle,
        InterventionPhase::Release => pb::InterventionPhase::Release,
        InterventionPhase::Retake => pb::InterventionPhase::Retake,
    }
}

fn grant_status_pb(s: GrantStatus) -> pb::GrantStatus {
    match s {
        GrantStatus::Active => pb::GrantStatus::Active,
        GrantStatus::Demoted => pb::GrantStatus::Demoted,
        GrantStatus::Revoked => pb::GrantStatus::Revoked,
    }
}

fn state_pb(k: Option<EpisodeStateKind>) -> pb::EpisodeState {
    k.map_or(pb::EpisodeState::Unspecified, EpisodeStateKind::to_pb)
}

pub fn state_transition(
    at: MonoNs,
    episode: &EpisodeId,
    from: Option<EpisodeStateKind>,
    to: EpisodeStateKind,
    reason: &str,
    outcome: Option<TerminalOutcome>,
) -> pb::EpisodeEvent {
    let mut ev = base(at, Some(episode));
    ev.event = Some(pb::episode_event::Event::State(pb::StateTransition {
        from: state_pb(from) as i32,
        to: to.to_pb() as i32,
        reason: reason.to_owned(),
        outcome: outcome.map_or(pb::TerminalOutcome::Unspecified as i32, |o| {
            o.to_pb() as i32
        }),
    }));
    ev
}

pub fn claim_event(
    at: MonoNs,
    episode: &EpisodeId,
    kind: pb::ClaimEventKind,
    claim: &ActiveClaim,
    detail: &str,
) -> pb::EpisodeEvent {
    let mut ev = base(at, Some(episode));
    ev.event = Some(pb::episode_event::Event::Claim(pb::ClaimEvent {
        kind: kind as i32,
        claim: Some(pb::Claim {
            claim_id: claim.id.as_str().to_owned(),
            episode_id: episode.as_str().to_owned(),
            // WHO is the point of a claim event. Dropping the actor here
            // left every downstream consumer — journal, sidecar spans,
            // judges — with a claim it could not attribute (a claim
            // "source_name" names the STREAM, not the actor).
            actor: Some(claim.actor.to_pb()),
            source_name: claim.source.clone(),
            self_initiated: claim.self_initiated,
            ..Default::default()
        }),
        detail: detail.to_owned(),
    }));
    ev
}

pub fn lease_event(
    at: MonoNs,
    episode: Option<&EpisodeId>,
    kind: pb::LeaseEventKind,
    lease: Option<(&LeaseId, &ClientId)>,
    enforcement: LeaseEnforcement,
    detail: &str,
) -> pb::EpisodeEvent {
    let mut ev = base(at, episode);
    ev.event = Some(pb::episode_event::Event::Lease(pb::LeaseEvent {
        kind: kind as i32,
        lease: lease.map(|(id, client)| pb::Lease {
            lease_id: id.as_str().to_owned(),
            holder_client_id: client.as_str().to_owned(),
            enforcement: match enforcement {
                LeaseEnforcement::Enforced => pb::LeaseEnforcement::Enforced as i32,
                LeaseEnforcement::Advisory => pb::LeaseEnforcement::Advisory as i32,
            },
            t_acquired_ns: at.0,
        }),
        detail: detail.to_owned(),
    }));
    ev
}

pub fn intervention(
    at: MonoNs,
    episode: &EpisodeId,
    phase: InterventionPhase,
    claim: &ClaimId,
) -> pb::EpisodeEvent {
    let mut ev = base(at, Some(episode));
    ev.event = Some(pb::episode_event::Event::Intervention(
        pb::InterventionEvent {
            phase: phase_pb(phase) as i32,
            claim_id: claim.as_str().to_owned(),
        },
    ));
    ev
}

pub fn tripwire(
    at: MonoNs,
    episode: Option<&EpisodeId>,
    name: &str,
    requested_verb: Verb,
    detail: &str,
) -> pb::EpisodeEvent {
    let mut ev = base(at, episode);
    ev.event = Some(pb::episode_event::Event::Tripwire(pb::TripwireEvent {
        name: name.to_owned(),
        detail: detail.to_owned(),
        requested_verb: requested_verb.to_pb() as i32,
        evidence: Default::default(),
    }));
    ev
}

pub fn fault(
    at: MonoNs,
    episode: Option<&EpisodeId>,
    kind: pb::FaultKind,
    source: &str,
    detail: &str,
) -> pb::EpisodeEvent {
    let mut ev = base(at, episode);
    ev.event = Some(pb::episode_event::Event::Fault(pb::Fault {
        kind: kind as i32,
        source: source.to_owned(),
        detail: detail.to_owned(),
        t_ns: at.0,
        cause: String::new(),
        evidence: Default::default(),
    }));
    ev
}

pub fn gate_mode_change(
    at: MonoNs,
    episode: &EpisodeId,
    from: GateMode,
    to: GateMode,
    reason: &str,
) -> pb::EpisodeEvent {
    let mut ev = base(at, Some(episode));
    ev.event = Some(pb::episode_event::Event::Gate(pb::GateModeChange {
        from: from.to_pb() as i32,
        to: to.to_pb() as i32,
        reason: reason.to_owned(),
    }));
    ev
}

pub fn dual_write(
    at: MonoNs,
    episode: &EpisodeId,
    divergence_metric: f64,
    window_ns: i64,
    trace_ref: &str,
    action_taken: Verb,
) -> pb::EpisodeEvent {
    let mut ev = base(at, Some(episode));
    ev.event = Some(pb::episode_event::Event::DualWrite(pb::DualWriteDetected {
        detail: "commanded-vs-proprioception divergence during advisory-lease bypass".to_owned(),
        divergence_metric,
        window_ns,
        trace_ref: trace_ref.to_owned(),
        action_taken: action_taken.to_pb() as i32,
    }));
    ev
}

pub fn grant_change(
    at: MonoNs,
    episode: Option<&EpisodeId>,
    verb: Verb,
    from: GrantStatus,
    to: GrantStatus,
    reason: &str,
    effective_t_ns: MonoNs,
) -> pb::EpisodeEvent {
    let mut ev = base(at, episode);
    ev.event = Some(pb::episode_event::Event::Grant(pb::GrantStatusChange {
        verb: verb.to_pb() as i32,
        send_interface: pb::SpaceKind::Unspecified as i32,
        from: grant_status_pb(from) as i32,
        to: grant_status_pb(to) as i32,
        reason: reason.to_owned(),
        effective_t_ns: effective_t_ns.0,
    }));
    ev
}

pub fn reset_verification(
    at: MonoNs,
    episode: &EpisodeId,
    mode: waddle_types::ResetVerificationMode,
    verified: bool,
    invalidated_async: bool,
) -> pb::EpisodeEvent {
    let mut ev = base(at, Some(episode));
    ev.event = Some(pb::episode_event::Event::ResetVerification(
        pb::ResetVerification {
            mode: match mode {
                waddle_types::ResetVerificationMode::Blocking => {
                    pb::ResetVerificationMode::Blocking as i32
                }
                waddle_types::ResetVerificationMode::OptimisticAsync => {
                    pb::ResetVerificationMode::OptimisticAsync as i32
                }
            },
            verified,
            invalidated_async,
            judge_id: String::new(),
            judge_version: String::new(),
            score: None,
            t_ns: at.0,
        },
    ));
    ev
}

pub fn judgment(
    at: MonoNs,
    episode: &EpisodeId,
    judge_id: &str,
    passed: Option<bool>,
) -> pb::EpisodeEvent {
    let mut ev = base(at, Some(episode));
    ev.event = Some(pb::episode_event::Event::Judgment(pb::JudgmentAttached {
        judgment: Some(pb::Judgment {
            judge_id: judge_id.to_owned(),
            passed,
            t_ns: at.0,
            ..Default::default()
        }),
    }));
    ev
}

/// The post-reset pipeline result (E15/E16). `result.ok` reflects the
/// pipeline; `pinned_outcome` is the terminal outcome fixed at E14.
pub fn post_reset(
    at: MonoNs,
    episode: &EpisodeId,
    ok: bool,
    detail: &str,
    pinned: TerminalOutcome,
) -> pb::EpisodeEvent {
    let mut ev = base(at, Some(episode));
    ev.event = Some(pb::episode_event::Event::PostReset(pb::PostResetResult {
        result: Some(pb::ResetResult {
            ok,
            detail: detail.to_owned(),
            strategy: String::new(),
            verification: None,
            fault: None,
        }),
        pinned_outcome: pinned.to_pb() as i32,
    }));
    ev
}

/// A remote reset window lifecycle event (E19–E22). `claim_id` is set from
/// ENGAGED onward; `result` is set on COMPLETED / TIMED_OUT.
#[allow(clippy::too_many_arguments)]
pub fn reset_window(
    at: MonoNs,
    episode: &EpisodeId,
    kind: pb::ResetWindowEventKind,
    reset: ResetKind,
    prompt: &str,
    expected: ActorKind,
    claim_id: &str,
    result: Option<pb::ResetResult>,
) -> pb::EpisodeEvent {
    let mut ev = base(at, Some(episode));
    ev.event = Some(pb::episode_event::Event::ResetWindow(
        pb::ResetWindowEvent {
            kind: kind as i32,
            reset: reset.to_pb() as i32,
            prompt: prompt.to_owned(),
            expected_actor: expected.to_pb() as i32,
            claim_id: claim_id.to_owned(),
            result,
        },
    ));
    ev
}

/// The agent-invite emission (flag `waddle.v0.agent`, E23): the open asked a
/// Waddle-hosted agent to drive this episode. Carries the ask, never
/// authority — the agent claims via the ordinary machinery (C8).
pub fn agent_invite(
    at: MonoNs,
    episode: &EpisodeId,
    prompt: &str,
    timeout_ns: i64,
    task_metadata: &BTreeMap<String, String>,
) -> pb::EpisodeEvent {
    let mut ev = base(at, Some(episode));
    ev.event = Some(pb::episode_event::Event::AgentInvite(
        pb::AgentInviteEvent {
            prompt: prompt.to_owned(),
            timeout_ns,
            task_metadata: task_metadata.clone().into_iter().collect(),
        },
    ));
    ev
}

pub fn mark(at: MonoNs, episode: &EpisodeId, kind: pb::MarkKind) -> pb::EpisodeEvent {
    let mut ev = base(at, Some(episode));
    ev.event = Some(pb::episode_event::Event::Mark(pb::MarkEvent {
        kind: kind as i32,
        actor: None,
        note: String::new(),
    }));
    ev
}
