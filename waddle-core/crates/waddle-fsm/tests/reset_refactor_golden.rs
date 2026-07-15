//! Refactor protection for the `enter_terminal` → `close_run` /
//! `request_terminal` extraction (Task 5). These scripts drive an
//! **undeclared** episode (no post-reset) through the run-closing paths and
//! capture the full ordered effect trace. The extraction must keep every one
//! of these traces byte-identical — an undeclared episode behaves exactly per
//! FSM.md E1–E13 (the additive guarantee).
//!
//! Captured against the pre-refactor implementation; if the refactor changes
//! any emission or effect ordering, one of these assertions fails.
//!
//! Note: the run-closing cancel list later gained `ResetWindowTimeout` (timer
//! hygiene, D7 edge 6). That is an emission-invisible, no-op `CancelTimer`
//! effect for undeclared episodes (no such timer is ever armed) — the
//! observable emission subsequence is unchanged.

use waddle_fsm::{Effect, SessionConfig, SessionEvent, SessionFsm, step};
use waddle_types::{
    ActorKind, ClaimId, EpisodeId, HandoffPolicy, LeaseEnforcement, LeaseId, MonoNs,
    ResetVerificationMode, TerminalOutcome, pb::v0 as pb,
};

/// A deterministic driver: applies an event, then auto-answers every
/// `MintLeaseToken` effect with a synthetic token so the multi-step engage /
/// release / terminal handoffs run to completion. Records a compact string
/// for every effect, in order, across the primary step and its follow-ups.
struct Driver {
    cfg: SessionConfig,
    state: SessionFsm,
    trace: Vec<String>,
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
            trace: Vec::new(),
            lease_seq: 0,
            clock: 0,
        }
    }

    fn tick(&mut self) -> MonoNs {
        self.clock += 1_000_000;
        MonoNs(self.clock)
    }

    fn apply(&mut self, ev: SessionEvent) {
        let stepped = step(&self.cfg, &self.state, &ev).expect("scripted event must be legal");
        self.state = stepped.next;
        let mut follow_ups = Vec::new();
        for effect in &stepped.effects {
            self.trace.push(render(effect));
            if let Effect::MintLeaseToken(_) = effect {
                self.lease_seq += 1;
                let at = self.tick();
                follow_ups.push(SessionEvent::LeaseTokenMinted {
                    minted: LeaseId::new(format!("L{}", self.lease_seq)),
                    at,
                });
            }
        }
        for f in follow_ups {
            self.apply(f);
        }
    }
}

fn render(effect: &Effect) -> String {
    match effect {
        Effect::Emit(ev) => match &ev.event {
            Some(pb::episode_event::Event::State(s)) => {
                format!("emit state {}->{} outcome={}", s.from, s.to, s.outcome)
            }
            Some(pb::episode_event::Event::Claim(c)) => format!("emit claim kind={}", c.kind),
            Some(pb::episode_event::Event::Lease(l)) => format!("emit lease kind={}", l.kind),
            Some(pb::episode_event::Event::Intervention(i)) => {
                format!("emit intervention phase={}", i.phase)
            }
            Some(pb::episode_event::Event::Gate(g)) => format!("emit gate {}->{}", g.from, g.to),
            Some(pb::episode_event::Event::Fault(f)) => format!("emit fault kind={}", f.kind),
            Some(pb::episode_event::Event::Mark(m)) => format!("emit mark kind={}", m.kind),
            Some(pb::episode_event::Event::ResetVerification(_)) => {
                "emit reset_verification".to_owned()
            }
            other => format!("emit other {other:?}"),
        },
        Effect::SetGateMode(m) => format!("set_gate {m:?}"),
        Effect::RequestVerb(v) => format!("request_verb {v:?}"),
        Effect::ArmTimer { id, .. } => format!("arm {id:?}"),
        Effect::CancelTimer { id } => format!("cancel {id:?}"),
        Effect::MintLeaseToken(op) => format!("mint {:?}", op.then),
        Effect::OpenSuccessor { .. } => "open_successor".to_owned(),
        Effect::ReprimePolicy => "reprime".to_owned(),
        Effect::SetResetUnverified { .. } => "set_reset_unverified".to_owned(),
        Effect::SetPostResetFailed { .. } => "set_post_reset_failed".to_owned(),
        Effect::RunPostReset { .. } => "run_post_reset".to_owned(),
    }
}

fn open_reset_run(d: &mut Driver) {
    let at = d.tick();
    d.apply(SessionEvent::EpisodeOpen {
        id: EpisodeId::new("ep-g"),
        verification: ResetVerificationMode::Blocking,
        born_claimed: false,
        parent: None,
        post_reset: false,
        pre_window: None,
        post_window: None,
        at,
    });
    let at = d.tick();
    d.apply(SessionEvent::ResetResult {
        ok: true,
        verified: Some(true),
        at,
    });
    let at = d.tick();
    d.apply(SessionEvent::Start { at });
}

fn grant_and_engage(d: &mut Driver) {
    let at = d.tick();
    d.apply(SessionEvent::ClaimGranted {
        id: ClaimId::new("claim-g"),
        source: "teleop".to_owned(),
        actor: ActorKind::Teleoperator,
        self_initiated: false,
        at,
    });
    let at = d.tick();
    d.apply(SessionEvent::Engage {
        claim: ClaimId::new("claim-g"),
        at,
    });
    let at = d.tick();
    d.apply(SessionEvent::VerbResult {
        verb: waddle_types::Verb::Hold,
        ok: true,
        fault: None,
        at,
    });
}

#[test]
fn undeclared_terminate_trace_is_stable() {
    let mut d = Driver::new();
    open_reset_run(&mut d);
    let at = d.tick();
    d.apply(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "done".to_owned(),
        at,
    });
    assert_eq!(
        d.trace,
        vec![
            "emit state 0->1 outcome=0",
            "mint InitialAcquire",
            "emit lease kind=1",
            "emit reset_verification",
            "emit state 1->2 outcome=0",
            "emit state 2->3 outcome=0",
            "emit state 3->5 outcome=1",
            "cancel EngageTimeout",
            "cancel ChunkBoundaryCap",
            "cancel ResetWindowTimeout",
        ]
    );
}

#[test]
fn undeclared_estop_while_claimed_trace_is_stable() {
    let mut d = Driver::new();
    open_reset_run(&mut d);
    grant_and_engage(&mut d);
    let at = d.tick();
    d.apply(SessionEvent::Estop { at });
    assert_eq!(
        d.trace,
        vec![
            "emit state 0->1 outcome=0",
            "mint InitialAcquire",
            "emit lease kind=1",
            "emit reset_verification",
            "emit state 1->2 outcome=0",
            "emit state 2->3 outcome=0",
            "emit claim kind=2",
            "emit state 3->4 outcome=0",
            "emit intervention phase=1",
            "arm EngageTimeout",
            "request_verb Hold",
            "mint EngageComplete",
            "emit lease kind=3",
            "cancel EngageTimeout",
            "emit intervention phase=2",
            "emit gate 1->2",
            "set_gate Intervention",
            "emit lease kind=5",
            "emit fault kind=1",
            "request_verb Estop",
            "emit gate 2->1",
            "set_gate Passthrough",
            "emit state 4->5 outcome=3",
            "emit claim kind=4",
            "cancel EngageTimeout",
            "cancel ChunkBoundaryCap",
            "cancel ResetWindowTimeout",
        ]
    );
}

#[test]
fn undeclared_retake_trace_is_stable() {
    let mut d = Driver::new();
    open_reset_run(&mut d);
    grant_and_engage(&mut d);
    let at = d.tick();
    d.apply(SessionEvent::Retake {
        claim: ClaimId::new("claim-g"),
        initiator: ActorKind::Teleoperator,
        successor: EpisodeId::new("ep-g2"),
        at,
    });
    assert_eq!(
        d.trace,
        vec![
            "emit state 0->1 outcome=0",
            "mint InitialAcquire",
            "emit lease kind=1",
            "emit reset_verification",
            "emit state 1->2 outcome=0",
            "emit state 2->3 outcome=0",
            "emit claim kind=2",
            "emit state 3->4 outcome=0",
            "emit intervention phase=1",
            "arm EngageTimeout",
            "request_verb Hold",
            "mint EngageComplete",
            "emit lease kind=3",
            "cancel EngageTimeout",
            "emit intervention phase=2",
            "emit gate 1->2",
            "set_gate Intervention",
            "emit intervention phase=4",
            "emit state 4->5 outcome=4",
            "cancel EngageTimeout",
            "cancel ChunkBoundaryCap",
            "cancel ResetWindowTimeout",
            "open_successor",
        ]
    );
}
