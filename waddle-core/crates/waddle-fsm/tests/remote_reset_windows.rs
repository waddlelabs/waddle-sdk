//! FSM.md §1.4 — remote reset windows (flag `waddle.v0.reset.remote`), rows
//! E19–E22 and C6/C7. A plane-directed remote actor performs a scene reset
//! through the SDK: a window opens, a reset claim is admitted (C6), the
//! claimant engages (lease → claimant, gate → RESET), and on completion the
//! lease hands back BEFORE the pipeline result applies.

use waddle_fsm::{
    Effect, LeaseState, Phase, SessionConfig, SessionEvent, SessionFsm, TimerId, WindowSpec, step,
};
use waddle_types::{
    ActorKind, ClaimId, EpisodeId, GateMode, HandoffPolicy, LeaseEnforcement, LeaseId, MonoNs,
    ResetVerificationMode, TerminalOutcome, Verb, pb::v0 as pb,
};

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

    fn ok(&mut self, ev: SessionEvent) {
        self.try_apply(ev).expect("scripted event must be legal");
    }

    fn try_apply(&mut self, ev: SessionEvent) -> Result<(), String> {
        let stepped = step(&self.cfg, &self.state, &ev).map_err(|e| e.to_string())?;
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
            self.ok(f);
        }
        Ok(())
    }

    fn phase(&self) -> Phase {
        self.state.episode.as_ref().expect("episode").phase
    }

    fn index_of<F: Fn(&str) -> bool>(&self, pred: F) -> Option<usize> {
        self.trace.iter().position(|s| pred(s))
    }

    fn armed(&self, id: TimerId) -> bool {
        self.trace.contains(&format!("arm {id:?}"))
    }

    /// The ordered `ResetWindowEventKind` values emitted (from the trace).
    fn window_kinds(&self) -> Vec<i32> {
        self.trace
            .iter()
            .filter_map(|s| {
                s.strip_prefix("reset_window kind=")
                    .and_then(|rest| rest.split_whitespace().next())
                    .and_then(|k| k.parse::<i32>().ok())
            })
            .collect()
    }
}

fn render(effect: &Effect) -> String {
    match effect {
        Effect::Emit(ev) => match &ev.event {
            Some(pb::episode_event::Event::State(s)) => {
                format!("state->{} outcome={}", s.to, s.outcome)
            }
            Some(pb::episode_event::Event::Claim(c)) => format!("claim kind={}", c.kind),
            Some(pb::episode_event::Event::Lease(l)) => format!("lease kind={}", l.kind),
            Some(pb::episode_event::Event::Gate(g)) => format!("gate {}->{}", g.from, g.to),
            Some(pb::episode_event::Event::Fault(f)) => format!("fault kind={}", f.kind),
            Some(pb::episode_event::Event::ResetWindow(w)) => {
                format!("reset_window kind={} actor={}", w.kind, w.expected_actor)
            }
            Some(pb::episode_event::Event::PostReset(p)) => {
                format!("post_reset ok={}", p.result.as_ref().unwrap().ok)
            }
            _ => "emit other".to_owned(),
        },
        Effect::SetGateMode(m) => format!("set_gate {m:?}"),
        Effect::ArmTimer { id, .. } => format!("arm {id:?}"),
        Effect::CancelTimer { id } => format!("cancel {id:?}"),
        Effect::MintLeaseToken(op) => format!("mint {:?}", op.then),
        Effect::SetPostResetFailed { .. } => "set_post_reset_failed".to_owned(),
        Effect::RunPostReset { .. } => "run_post_reset".to_owned(),
        _ => "effect other".to_owned(),
    }
}

const WINDOW_ENGAGED: i32 = pb::ResetWindowEventKind::Engaged as i32;
const WINDOW_COMPLETED: i32 = pb::ResetWindowEventKind::Completed as i32;
const WINDOW_TIMED_OUT: i32 = pb::ResetWindowEventKind::TimedOut as i32;
const STATE_READY: i32 = pb::EpisodeState::Ready as i32;

fn teleop_window() -> WindowSpec {
    WindowSpec {
        expected: ActorKind::Teleoperator,
        prompt: "clear the table".to_owned(),
        timeout_ns: 600_000_000_000,
    }
}

fn open_with(d: &mut Driver, pre: Option<WindowSpec>, post: Option<WindowSpec>) {
    let at = d.tick();
    d.ok(SessionEvent::EpisodeOpen {
        id: EpisodeId::new("ep-rw"),
        verification: ResetVerificationMode::Blocking,
        born_claimed: false,
        parent: None,
        post_reset: false,
        pre_window: pre,
        post_window: post,
        at,
    });
}

fn grant(d: &mut Driver, id: &str, actor: ActorKind) -> Result<(), String> {
    let at = d.tick();
    d.try_apply(SessionEvent::ClaimGranted {
        id: ClaimId::new(id),
        source: "teleop".to_owned(),
        actor,
        self_initiated: false,
        at,
    })
}

fn open_pre_and_grant(actor: ActorKind) -> Driver {
    let mut d = Driver::new();
    open_with(&mut d, Some(teleop_window()), None);
    grant(&mut d, "claim-rw", actor).expect("C6 admits this actor");
    d
}

// E19 --------------------------------------------------------------------

#[test]
fn e19_pre_window_opens_and_arms_timer() {
    let mut d = Driver::new();
    open_with(&mut d, Some(teleop_window()), None);
    assert!(d.state.episode.as_ref().unwrap().reset_window.is_some());
    assert!(d.armed(TimerId::ResetWindowTimeout), "timeout armed");
    assert!(
        d.window_kinds()
            .contains(&(pb::ResetWindowEventKind::Opened as i32))
    );
}

#[test]
fn e19_no_window_when_undeclared() {
    let mut d = Driver::new();
    open_with(&mut d, None, None);
    assert!(d.state.episode.as_ref().unwrap().reset_window.is_none());
    assert!(!d.armed(TimerId::ResetWindowTimeout));
}

#[test]
fn e19_no_window_for_born_claimed_successor() {
    // Drive a retake so the claim survives, then open the born-claimed
    // successor with a pre window declared: no window opens (D7 edge 5).
    let mut d = Driver::new();
    open_with(&mut d, None, None);
    let at = d.tick();
    d.ok(SessionEvent::ResetResult {
        ok: true,
        verified: Some(true),
        at,
    });
    let at = d.tick();
    d.ok(SessionEvent::Start { at });
    grant(&mut d, "surviving", ActorKind::Teleoperator).expect("claim in RUNNING");
    let at = d.tick();
    d.ok(SessionEvent::Engage {
        claim: ClaimId::new("surviving"),
        at,
    });
    let at = d.tick();
    d.ok(SessionEvent::VerbResult {
        verb: Verb::Hold,
        ok: true,
        fault: None,
        at,
    });
    let at = d.tick();
    d.ok(SessionEvent::Retake {
        claim: ClaimId::new("surviving"),
        initiator: ActorKind::Teleoperator,
        successor: EpisodeId::new("ep-succ"),
        at,
    });
    assert!(d.state.claim.is_some(), "claim survives the retake");
    // Now the runtime opens the successor born-claimed, with a pre window
    // declared. The surviving claim must suppress the window.
    let at = d.tick();
    d.ok(SessionEvent::EpisodeOpen {
        id: EpisodeId::new("ep-succ"),
        verification: ResetVerificationMode::Blocking,
        born_claimed: true,
        parent: Some(EpisodeId::new("ep-rw")),
        post_reset: false,
        pre_window: Some(teleop_window()),
        post_window: None,
        at,
    });
    assert!(
        d.state.episode.as_ref().unwrap().reset_window.is_none(),
        "born-claimed successor opens no remote pre window"
    );
}

// C6 ---------------------------------------------------------------------

#[test]
fn c6_teleop_window_admits_teleoperator_and_site_operator() {
    assert!(
        open_pre_and_grant(ActorKind::Teleoperator)
            .state
            .claim
            .is_some()
    );
    assert!(
        open_pre_and_grant(ActorKind::SiteOperator)
            .state
            .claim
            .is_some()
    );
}

#[test]
fn c6_agent_window_admits_agent_only() {
    let mut d = Driver::new();
    open_with(
        &mut d,
        Some(WindowSpec {
            expected: ActorKind::Agent,
            prompt: "reset".to_owned(),
            timeout_ns: 600_000_000_000,
        }),
        None,
    );
    assert!(
        grant(&mut d, "c-top", ActorKind::Teleoperator).is_err(),
        "teleoperator denied by agent window (C6)"
    );
    assert!(d.state.claim.is_none());
    grant(&mut d, "c-agent", ActorKind::Agent).expect("agent admitted");
    assert!(d.state.claim.is_some());
}

#[test]
fn c6_wrong_actor_denied() {
    let mut d = Driver::new();
    open_with(&mut d, Some(teleop_window()), None);
    assert!(
        grant(&mut d, "c-agent", ActorKind::Agent).is_err(),
        "agent denied by teleoperator window (C6)"
    );
    assert!(d.state.claim.is_none());
}

#[test]
fn c6_second_claim_denied_while_one_active() {
    let mut d = open_pre_and_grant(ActorKind::Teleoperator);
    assert!(
        grant(&mut d, "c-2", ActorKind::Teleoperator).is_err(),
        "one-claim rule (N18) still holds under C6"
    );
}

// E20 --------------------------------------------------------------------

#[test]
fn e20_engage_routes_lease_and_gate_reset() {
    let mut d = open_pre_and_grant(ActorKind::Teleoperator);
    let at = d.tick();
    d.ok(SessionEvent::ResetWindowEngage {
        claim: ClaimId::new("claim-rw"),
        at,
    });
    assert_eq!(d.state.gate_mode, GateMode::Reset, "gate → RESET on engage");
    assert!(
        d.state
            .episode
            .as_ref()
            .unwrap()
            .reset_window
            .as_ref()
            .unwrap()
            .engaged
    );
    match &d.state.lease {
        LeaseState::Held { client, .. } => assert_eq!(client.as_str(), "teleop"),
        LeaseState::Vacant => panic!("lease must be held by the claimant"),
    }
    // gate→RESET (1->4) precedes the ENGAGED window event.
    let gate_idx = d.index_of(|s| s == "gate 1->4").expect("gate → RESET");
    let engaged_idx = d
        .index_of(|s| s.starts_with(&format!("reset_window kind={WINDOW_ENGAGED}")))
        .expect("window ENGAGED");
    assert!(gate_idx < engaged_idx, "gate→RESET before ENGAGED");
}

// E21 --------------------------------------------------------------------

#[test]
fn e21_complete_hands_lease_back_before_ready_then_applies_result() {
    let mut d = open_pre_and_grant(ActorKind::Teleoperator);
    let at = d.tick();
    d.ok(SessionEvent::ResetWindowEngage {
        claim: ClaimId::new("claim-rw"),
        at,
    });
    let at = d.tick();
    d.ok(SessionEvent::ResetWindowComplete {
        claim: ClaimId::new("claim-rw"),
        ok: true,
        verified: Some(true),
        at,
    });

    assert_eq!(d.phase(), Phase::Ready);
    assert!(d.state.claim.is_none(), "C7: reset claim released");
    assert_eq!(d.state.gate_mode, GateMode::Passthrough);

    // Deferred-apply ordering: gate RESET→PASSTHROUGH (the handback) precedes
    // the →READY transition.
    let gate_back = d
        .index_of(|s| s == "gate 4->1")
        .expect("gate RESET→PASSTHROUGH");
    let ready = d
        .index_of(|s| s.starts_with(&format!("state->{STATE_READY}")))
        .expect("→READY");
    assert!(
        gate_back < ready,
        "handback precedes READY (deferred-apply)"
    );
    assert!(d.window_kinds().contains(&WINDOW_COMPLETED));
    assert!(
        d.trace
            .contains(&format!("cancel {:?}", TimerId::ResetWindowTimeout))
    );
}

// E22 --------------------------------------------------------------------

#[test]
fn e22_pre_window_timeout_aborts() {
    let mut d = open_pre_and_grant(ActorKind::Teleoperator);
    let at = d.tick();
    d.ok(SessionEvent::ResetWindowEngage {
        claim: ClaimId::new("claim-rw"),
        at,
    });
    let at = d.tick();
    d.ok(SessionEvent::TimerFired {
        id: TimerId::ResetWindowTimeout,
        at,
    });
    assert_eq!(
        d.phase(),
        Phase::Terminal(TerminalOutcome::Abort),
        "pre timeout → ABORT (E5)"
    );
    assert!(d.state.claim.is_none(), "C7 release");
    assert!(d.window_kinds().contains(&WINDOW_TIMED_OUT));
}

#[test]
fn e22_post_window_timeout_keeps_pinned_and_flags() {
    let mut d = Driver::new();
    open_with(&mut d, None, Some(teleop_window()));
    let at = d.tick();
    d.ok(SessionEvent::ResetResult {
        ok: true,
        verified: Some(true),
        at,
    });
    let at = d.tick();
    d.ok(SessionEvent::Start { at });
    let at = d.tick();
    d.ok(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "done".to_owned(),
        at,
    });
    assert_eq!(d.phase(), Phase::PostReset, "post window opened at E14");
    let at = d.tick();
    d.ok(SessionEvent::TimerFired {
        id: TimerId::ResetWindowTimeout,
        at,
    });
    assert_eq!(
        d.phase(),
        Phase::Terminal(TerminalOutcome::Success),
        "post timeout keeps the pinned outcome (E16)"
    );
    assert!(d.state.episode.as_ref().unwrap().post_reset_failed);
    assert!(d.window_kinds().contains(&WINDOW_TIMED_OUT));
}
