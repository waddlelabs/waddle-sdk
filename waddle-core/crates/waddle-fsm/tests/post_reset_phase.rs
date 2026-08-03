//! FSM.md §1.3 — the POST_RESET phase (flag `waddle.v0.reset.phases`), rows
//! E14–E18 and E14b. An episode that declares a post-reset runs a cleanup
//! pipeline INSIDE the finishing episode; the terminal outcome is pinned at
//! entry and never changes.

use waddle_fsm::{Effect, LeaseState, Phase, SessionConfig, SessionEvent, SessionFsm, step};
use waddle_types::{
    ActorKind, ActorRef, ClaimId, EpisodeId, HandoffPolicy, LeaseEnforcement, LeaseId, MonoNs,
    ResetVerificationMode, TerminalOutcome, Verb, pb::v0 as pb,
};

/// Drives events, auto-answering `MintLeaseToken`, and accumulates every
/// effect emitted across a run for assertions.
struct Driver {
    cfg: SessionConfig,
    state: SessionFsm,
    effects: Vec<Effect>,
    lease_seq: u32,
    clock: i64,
}

impl Driver {
    fn new() -> Self {
        let cfg = SessionConfig::minimal(
            "loop-client",
            HandoffPolicy::HoldFirst,
            LeaseEnforcement::Advisory,
        );
        let state = SessionFsm::new(&cfg);
        Self {
            cfg,
            state,
            effects: Vec::new(),
            lease_seq: 0,
            clock: 0,
        }
    }

    fn tick(&mut self) -> MonoNs {
        self.clock += 1_000_000;
        MonoNs(self.clock)
    }

    /// Apply an event that must succeed, running lease follow-ups.
    fn ok(&mut self, ev: SessionEvent) {
        self.try_apply(ev).expect("scripted event must be legal");
    }

    fn try_apply(&mut self, ev: SessionEvent) -> Result<(), String> {
        let stepped = step(&self.cfg, &self.state, &ev).map_err(|e| e.to_string())?;
        self.state = stepped.next;
        let mut follow_ups = Vec::new();
        for effect in &stepped.effects {
            if let Effect::MintLeaseToken(_) = effect {
                self.lease_seq += 1;
                let at = self.tick();
                follow_ups.push(SessionEvent::LeaseTokenMinted {
                    minted: LeaseId::new(format!("L{}", self.lease_seq)),
                    at,
                });
            }
        }
        self.effects.extend(stepped.effects);
        for f in follow_ups {
            self.ok(f);
        }
        Ok(())
    }

    fn phase(&self) -> Phase {
        self.state.episode.as_ref().expect("episode").phase
    }

    fn transitions(&self) -> Vec<(i32, i32)> {
        self.effects
            .iter()
            .filter_map(|e| match e {
                Effect::Emit(ev) => match &ev.event {
                    Some(pb::episode_event::Event::State(s)) => Some((s.to, s.outcome)),
                    _ => None,
                },
                _ => None,
            })
            .collect()
    }

    fn post_reset_events(&self) -> Vec<pb::PostResetResult> {
        self.effects
            .iter()
            .filter_map(|e| match e {
                Effect::Emit(ev) => match &ev.event {
                    Some(pb::episode_event::Event::PostReset(p)) => Some(p.clone()),
                    _ => None,
                },
                _ => None,
            })
            .collect()
    }

    fn faults(&self) -> Vec<i32> {
        self.effects
            .iter()
            .filter_map(|e| match e {
                Effect::Emit(ev) => match &ev.event {
                    Some(pb::episode_event::Event::Fault(f)) => Some(f.kind),
                    _ => None,
                },
                _ => None,
            })
            .collect()
    }

    fn has_run_post_reset(&self) -> bool {
        self.effects
            .iter()
            .any(|e| matches!(e, Effect::RunPostReset { .. }))
    }

    fn has_set_post_reset_failed(&self) -> bool {
        self.effects
            .iter()
            .any(|e| matches!(e, Effect::SetPostResetFailed { .. }))
    }

    fn marks(&self) -> usize {
        self.effects
            .iter()
            .filter(|e| {
                matches!(
                    e,
                    Effect::Emit(ev) if matches!(&ev.event, Some(pb::episode_event::Event::Mark(_)))
                )
            })
            .count()
    }
}

fn open_declared(d: &mut Driver) {
    let at = d.tick();
    d.ok(SessionEvent::EpisodeOpen {
        id: EpisodeId::new("ep-pr"),
        verification: ResetVerificationMode::Blocking,
        born_claimed: false,
        parent: None,
        post_reset: true,
        pre_window: None,
        post_window: None,
        agent_invite: None,
        at,
    });
    let at = d.tick();
    d.ok(SessionEvent::ResetResult {
        ok: true,
        verified: Some(true),
        at,
    });
    let at = d.tick();
    d.ok(SessionEvent::Start { at });
}

fn engage(d: &mut Driver) {
    let at = d.tick();
    d.ok(SessionEvent::ClaimGranted {
        id: ClaimId::new("claim-pr"),
        source: "teleop".to_owned(),
        actor: ActorRef::of_kind(ActorKind::Teleoperator),
        self_initiated: false,
        at,
    });
    let at = d.tick();
    d.ok(SessionEvent::Engage {
        claim: ClaimId::new("claim-pr"),
        at,
    });
    let at = d.tick();
    d.ok(SessionEvent::VerbResult {
        verb: Verb::Hold,
        ok: true,
        fault: None,
        at,
    });
}

fn terminate(d: &mut Driver, outcome: TerminalOutcome) {
    let at = d.tick();
    d.ok(SessionEvent::Terminate {
        outcome,
        reason: "done".to_owned(),
        at,
    });
}

const STATE_POST_RESET: i32 = pb::EpisodeState::PostReset as i32;
const STATE_TERMINAL: i32 = pb::EpisodeState::Terminal as i32;
const OUTCOME_SUCCESS: i32 = pb::TerminalOutcome::Success as i32;

// E14 --------------------------------------------------------------------

#[test]
fn e14_terminate_from_running_enters_post_reset_pinned() {
    let mut d = Driver::new();
    open_declared(&mut d);
    terminate(&mut d, TerminalOutcome::Success);

    assert_eq!(d.phase(), Phase::PostReset);
    let ep = d.state.episode.as_ref().unwrap();
    assert_eq!(ep.pinned_outcome, Some(TerminalOutcome::Success));
    assert!(ep.post_reset_declared);
    // The →POST_RESET transition carries the pinned outcome.
    assert!(
        d.transitions()
            .contains(&(STATE_POST_RESET, OUTCOME_SUCCESS))
    );
    // Hook pipeline engaged; no terminal transition yet.
    assert!(d.has_run_post_reset());
    assert!(!d.transitions().iter().any(|(to, _)| *to == STATE_TERMINAL));
}

#[test]
fn e14_terminate_from_intervention_releases_claim_then_post_reset() {
    let mut d = Driver::new();
    open_declared(&mut d);
    engage(&mut d); // now in SETTLE with a claim + lease held
    assert!(matches!(d.phase(), Phase::Intervention(_)));
    terminate(&mut d, TerminalOutcome::Success);

    assert_eq!(d.phase(), Phase::PostReset);
    assert!(d.state.claim.is_none(), "claim released at E14");
    assert_eq!(d.state.gate_mode, waddle_types::GateMode::Passthrough);
    assert!(
        d.transitions()
            .contains(&(STATE_POST_RESET, OUTCOME_SUCCESS))
    );
}

// E15 --------------------------------------------------------------------

#[test]
fn e15_post_reset_ok_reaches_terminal_pinned() {
    let mut d = Driver::new();
    open_declared(&mut d);
    terminate(&mut d, TerminalOutcome::Success);
    let at = d.tick();
    d.ok(SessionEvent::PostResetResult {
        ok: true,
        detail: "clean".to_owned(),
        at,
    });

    assert_eq!(d.phase(), Phase::Terminal(TerminalOutcome::Success));
    let pr = d.post_reset_events();
    assert_eq!(pr.len(), 1);
    assert!(pr[0].result.as_ref().unwrap().ok);
    assert_eq!(pr[0].pinned_outcome, OUTCOME_SUCCESS);
    assert!(d.transitions().contains(&(STATE_TERMINAL, OUTCOME_SUCCESS)));
    assert!(!d.has_set_post_reset_failed());
}

// E16 --------------------------------------------------------------------

#[test]
fn e16_post_reset_failure_flags_but_keeps_pinned_outcome() {
    let mut d = Driver::new();
    open_declared(&mut d);
    terminate(&mut d, TerminalOutcome::Success);
    let at = d.tick();
    d.ok(SessionEvent::PostResetResult {
        ok: false,
        detail: "strategies exhausted".to_owned(),
        at,
    });

    assert_eq!(d.phase(), Phase::Terminal(TerminalOutcome::Success));
    assert!(d.state.episode.as_ref().unwrap().post_reset_failed);
    assert!(d.has_set_post_reset_failed());
    let pr = d.post_reset_events();
    assert_eq!(pr.len(), 1);
    assert!(!pr[0].result.as_ref().unwrap().ok);
    assert_eq!(pr[0].pinned_outcome, OUTCOME_SUCCESS);
    assert!(!d.faults().is_empty(), "E16 emits a Fault");
    assert!(d.transitions().contains(&(STATE_TERMINAL, OUTCOME_SUCCESS)));
}

// E17 --------------------------------------------------------------------

#[test]
fn e17_estop_in_post_reset_keeps_pinned_and_flags_failed() {
    let mut d = Driver::new();
    open_declared(&mut d);
    terminate(&mut d, TerminalOutcome::Success);
    let at = d.tick();
    d.ok(SessionEvent::Estop { at });

    assert_eq!(d.phase(), Phase::Terminal(TerminalOutcome::Success));
    assert_eq!(d.state.lease, LeaseState::Vacant, "estop revokes the lease");
    assert!(d.state.episode.as_ref().unwrap().post_reset_failed);
    assert!(d.has_set_post_reset_failed());
    assert!(
        d.faults().contains(&(pb::FaultKind::Estop as i32)),
        "E17 emits Fault{{ESTOP}}"
    );
    assert!(d.transitions().contains(&(STATE_TERMINAL, OUTCOME_SUCCESS)));
}

// E14b -------------------------------------------------------------------

#[test]
fn e14b_terminate_in_post_reset_is_rejected() {
    let mut d = Driver::new();
    open_declared(&mut d);
    terminate(&mut d, TerminalOutcome::Success);
    let at = d.tick();
    let res = d.try_apply(SessionEvent::Terminate {
        outcome: TerminalOutcome::Failure,
        reason: "late".to_owned(),
        at,
    });
    assert!(res.is_err(), "terminate in POST_RESET rejected (E14b)");
    assert_eq!(d.phase(), Phase::PostReset);
    assert_eq!(
        d.state.episode.as_ref().unwrap().pinned_outcome,
        Some(TerminalOutcome::Success)
    );
}

#[test]
fn e14b_mark_in_post_reset_records_event_without_transition() {
    let mut d = Driver::new();
    open_declared(&mut d);
    terminate(&mut d, TerminalOutcome::Success);
    let before_marks = d.marks();
    let at = d.tick();
    d.ok(SessionEvent::Mark {
        kind: waddle_fsm::MarkKind::EndFailure,
        at,
    });
    assert_eq!(d.marks(), before_marks + 1, "the late mark is recorded");
    assert_eq!(d.phase(), Phase::PostReset, "but never transitions (E14b)");
    assert!(!d.transitions().iter().any(|(to, _)| *to == STATE_TERMINAL));
}

// E18 --------------------------------------------------------------------

#[test]
fn e18_retake_bypasses_post_reset() {
    let mut d = Driver::new();
    open_declared(&mut d);
    engage(&mut d); // SETTLE
    let at = d.tick();
    d.ok(SessionEvent::Retake {
        claim: ClaimId::new("claim-pr"),
        initiator: ActorKind::Teleoperator,
        successor: EpisodeId::new("ep-pr2"),
        at,
    });

    // Predecessor goes straight to TERMINAL{ABORTED_RETAKE}, never POST_RESET.
    assert!(
        !d.transitions()
            .iter()
            .any(|(to, _)| *to == STATE_POST_RESET)
    );
    assert!(
        d.transitions()
            .iter()
            .any(|(to, outcome)| *to == STATE_TERMINAL
                && *outcome == pb::TerminalOutcome::AbortedRetake as i32)
    );
    assert!(
        d.effects
            .iter()
            .any(|e| matches!(e, Effect::OpenSuccessor { .. }))
    );
}
