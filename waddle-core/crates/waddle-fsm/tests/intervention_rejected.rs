//! The intake-validation contract: `SessionEvent::InterventionRejected`
//! must land a `Fault{FAULT_KIND_VALIDATION_ERROR}` on the episode event
//! stream — naming the producer AND the reason — without touching
//! claim/gate/phase state. It is a diagnostic emission, not a state
//! transition.

use waddle_fsm::{RejectReason, SessionConfig, SessionEvent, SessionFsm, step};
use waddle_types::pb::v0 as pb;
use waddle_types::{EpisodeId, HandoffPolicy, LeaseEnforcement, MonoNs, ResetVerificationMode};

fn opened_fsm() -> (SessionConfig, SessionFsm) {
    let cfg = SessionConfig::minimal(
        "loop-client",
        HandoffPolicy::HoldFirst,
        LeaseEnforcement::Advisory,
    );
    let state = SessionFsm::new(&cfg);
    let stepped = step(
        &cfg,
        &state,
        &SessionEvent::EpisodeOpen {
            id: EpisodeId::new("ep-1"),
            verification: ResetVerificationMode::Blocking,
            born_claimed: false,
            parent: None,
            post_reset: false,
            pre_window: None,
            post_window: None,
            agent_invite: None,
            at: MonoNs(0),
        },
    )
    .expect("episode_open is legal from a fresh session");
    (cfg, stepped.next)
}

/// Every `Fault` the step emitted, in order.
fn faults(stepped: &waddle_fsm::Step) -> Vec<pb::Fault> {
    stepped
        .effects
        .iter()
        .filter_map(|effect| match effect {
            waddle_fsm::Effect::Emit(ev) => match &ev.event {
                Some(pb::episode_event::Event::Fault(f)) => Some(f.clone()),
                _ => None,
            },
            _ => None,
        })
        .collect()
}

#[test]
fn intervention_rejected_emits_exactly_one_validation_fault() {
    let (cfg, state) = opened_fsm();
    let before = state.clone();

    let stepped = step(
        &cfg,
        &state,
        &SessionEvent::InterventionRejected {
            source: "media-intake",
            reason: RejectReason::Dims { got: 7, want: 6 },
            at: MonoNs(1_000),
        },
    )
    .expect("intervention_rejected is legal with an active episode");

    let faults = faults(&stepped);
    assert_eq!(
        faults.len(),
        1,
        "expected exactly one fault, got {faults:?}"
    );
    assert_eq!(faults[0].kind, pb::FaultKind::ValidationError as i32);

    // A diagnostic emission only: no claim/gate/phase side effects.
    assert_eq!(stepped.next.claim, before.claim);
    assert_eq!(stepped.next.gate_mode, before.gate_mode);
    assert_eq!(
        stepped.next.episode.as_ref().map(|e| e.phase),
        before.episode.as_ref().map(|e| e.phase),
    );
}

/// The fault's `source` must reflect the ACTUAL rejecting
/// producer, not a hardcoded "media-intake" — an agent-chunk mismatch
/// (`forward_server_msg`'s `InterventionChunk` arm) must never be
/// misreported as a teleop one.
#[test]
fn intervention_rejected_carries_the_producers_source_into_the_fault() {
    let (cfg, state) = opened_fsm();

    let stepped = step(
        &cfg,
        &state,
        &SessionEvent::InterventionRejected {
            source: "agent-chunk",
            reason: RejectReason::Dims { got: 3, want: 6 },
            at: MonoNs(1_000),
        },
    )
    .expect("intervention_rejected is legal with an active episode");

    let faults = faults(&stepped);
    assert_eq!(faults.len(), 1);
    assert_eq!(faults[0].source, "agent-chunk");
    assert!(faults[0].detail.contains("3 dims"));
    assert!(faults[0].detail.contains("6"));
}

/// Each reason reports itself in its own words. A refusal described in the
/// wrong shape — a skipped inert step reported as a dims mismatch — sends
/// whoever reads the recording looking at the wrong thing, so the fault
/// text has to name what actually happened.
#[test]
fn every_reject_reason_reports_what_actually_happened() {
    let (cfg, state) = opened_fsm();

    let stepped = step(
        &cfg,
        &state,
        &SessionEvent::InterventionRejected {
            source: "agent-chunk",
            reason: RejectReason::InertStepsSkipped { skipped: 1, of: 4 },
            at: MonoNs(1_000),
        },
    )
    .expect("intervention_rejected is legal with an active episode");
    let skip = faults(&stepped);
    assert_eq!(skip.len(), 1);
    assert_eq!(skip[0].kind, pb::FaultKind::ValidationError as i32);
    assert_eq!(skip[0].source, "agent-chunk");
    assert!(
        skip[0].detail.contains("skipped 1 of 4"),
        "the skip must say how much of the chunk it was: {:?}",
        skip[0].detail
    );
    assert!(
        !skip[0].detail.contains("dims"),
        "an inert step is not a dims mismatch: {:?}",
        skip[0].detail
    );

    let stepped = step(
        &cfg,
        &state,
        &SessionEvent::InterventionRejected {
            source: "agent-chunk",
            reason: RejectReason::NotExecutable(
                "target arm does not match the declared action space".to_owned(),
            ),
            at: MonoNs(2_000),
        },
    )
    .expect("intervention_rejected is legal with an active episode");
    let refusal = faults(&stepped);
    assert_eq!(refusal.len(), 1);
    assert!(
        refusal[0].detail.contains("target arm"),
        "the refusal must carry the producer's own reason: {:?}",
        refusal[0].detail
    );
}

#[test]
fn intervention_rejected_without_an_active_episode_is_rejected() {
    let cfg = SessionConfig::minimal(
        "loop-client",
        HandoffPolicy::HoldFirst,
        LeaseEnforcement::Advisory,
    );
    let state = SessionFsm::new(&cfg);
    let result = step(
        &cfg,
        &state,
        &SessionEvent::InterventionRejected {
            source: "media-intake",
            reason: RejectReason::Dims { got: 7, want: 6 },
            at: MonoNs(0),
        },
    );
    assert!(result.is_err());
}
