//! Builders for the `pb::EpisodeEvent`s the session machine emits. Emissions
//! are real wire messages so every consumer (sidecar, recorder, control
//! plane, conformance runner) sees exactly one vocabulary.

use waddle_types::{
    ClaimId, ClientId, EpisodeId, EpisodeStateKind, GateMode, GrantStatus, InterventionPhase,
    LeaseEnforcement, LeaseId, MonoNs, TerminalOutcome, Verb, pb::v0 as pb,
};

use crate::claim::ActiveClaim;

fn base(at: MonoNs, episode: Option<&EpisodeId>) -> pb::EpisodeEvent {
    pb::EpisodeEvent {
        t_ns: at.0,
        episode_id: episode.map(|e| e.as_str().to_owned()).unwrap_or_default(),
        event: None,
    }
}

fn gate_mode_pb(mode: GateMode) -> pb::GateMode {
    match mode {
        GateMode::Passthrough => pb::GateMode::Passthrough,
        GateMode::Intervention => pb::GateMode::Intervention,
        GateMode::Bypass => pb::GateMode::Bypass,
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
        from: gate_mode_pb(from) as i32,
        to: gate_mode_pb(to) as i32,
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

pub fn mark(at: MonoNs, episode: &EpisodeId, kind: pb::MarkKind) -> pb::EpisodeEvent {
    let mut ev = base(at, Some(episode));
    ev.event = Some(pb::episode_event::Event::Mark(pb::MarkEvent {
        kind: kind as i32,
        actor: None,
        note: String::new(),
    }));
    ev
}
