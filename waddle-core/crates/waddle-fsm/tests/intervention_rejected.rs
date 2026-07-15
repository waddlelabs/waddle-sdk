//! Bug 2 (action-space validation): `SessionEvent::InterventionRejected`
//! must land a `Fault{FAULT_KIND_VALIDATION_ERROR}` on the episode event
//! stream without touching claim/gate/phase state — it is a diagnostic
//! emission, not a state transition.

use waddle_fsm::{SessionConfig, SessionEvent, SessionFsm, step};
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
            at: MonoNs(0),
        },
    )
    .expect("episode_open is legal from a fresh session");
    (cfg, stepped.next)
}

#[test]
fn intervention_rejected_emits_exactly_one_validation_fault() {
    let (cfg, state) = opened_fsm();
    let before = state.clone();

    let stepped = step(
        &cfg,
        &state,
        &SessionEvent::InterventionRejected {
            dims_got: 7,
            dims_want: 6,
            at: MonoNs(1_000),
        },
    )
    .expect("intervention_rejected is legal with an active episode");

    let faults: Vec<pb::Fault> = stepped
        .effects
        .iter()
        .filter_map(|effect| match effect {
            waddle_fsm::Effect::Emit(ev) => match &ev.event {
                Some(pb::episode_event::Event::Fault(f)) => Some(f.clone()),
                _ => None,
            },
            _ => None,
        })
        .collect();
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
            dims_got: 7,
            dims_want: 6,
            at: MonoNs(0),
        },
    );
    assert!(result.is_err());
}
