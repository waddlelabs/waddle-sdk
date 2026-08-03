//! FSM.md §1.4 — remote reset windows (flag `waddle.v0.reset.remote`), rows
//! E19–E22 and C6/C7. A plane-directed remote actor performs a scene reset
//! through the SDK: a window opens, a reset claim is admitted (C6), the
//! claimant engages (lease → claimant, gate → RESET), and on completion the
//! lease hands back BEFORE the pipeline result applies.

use waddle_fsm::{
    Effect, LeaseState, Phase, SessionConfig, SessionEvent, SessionFsm, TimerId, WindowSpec, step,
};
use waddle_types::{
    ActorKind, ActorRef, ClaimId, EpisodeId, GateMode, HandoffPolicy, LeaseEnforcement, LeaseId,
    MonoNs, ResetVerificationMode, TerminalOutcome, Verb, pb::v0 as pb,
};

struct Driver {
    cfg: SessionConfig,
    state: SessionFsm,
    trace: Vec<String>,
    lease_seq: u32,
    clock: i64,
    /// When true, `MintLeaseToken` effects queue an answer instead of the
    /// instant reply `try_apply` gives by default. The production reducer
    /// answers via the TAIL of its single event queue, so any event already
    /// queued (a plane's back-to-back COMPLETE, a racing `claim_released`)
    /// is processed BEFORE the answer — deferral is how tests express
    /// exactly those interleavings.
    defer_mints: bool,
    outstanding_mints: u32,
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
            defer_mints: false,
            outstanding_mints: 0,
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
                if self.defer_mints {
                    self.outstanding_mints += 1;
                } else {
                    self.lease_seq += 1;
                    let at = self.tick();
                    follow_ups.push(SessionEvent::LeaseTokenMinted {
                        minted: LeaseId::new(format!("L{}", self.lease_seq)),
                        at,
                    });
                }
            }
        }
        for f in follow_ups {
            self.ok(f);
        }
        Ok(())
    }

    /// Deliver ONE deferred mint answer. FIFO identity is irrelevant: the
    /// FSM matches every answer to its single current `pending_lease` slot,
    /// exactly as the runtime's uncorrelated `LeaseTokenMinted` does.
    fn answer_mint(&mut self) -> Result<(), String> {
        assert!(self.outstanding_mints > 0, "no deferred mint to answer");
        self.outstanding_mints -= 1;
        self.lease_seq += 1;
        let minted = LeaseId::new(format!("L{}", self.lease_seq));
        let at = self.tick();
        self.try_apply(SessionEvent::LeaseTokenMinted { minted, at })
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
        agent_invite: None,
        at,
    });
}

fn grant(d: &mut Driver, id: &str, actor: ActorKind) -> Result<(), String> {
    let at = d.tick();
    d.try_apply(SessionEvent::ClaimGranted {
        id: ClaimId::new(id),
        source: "teleop".to_owned(),
        actor: ActorRef::of_kind(actor),
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
    // successor with a pre window declared: no window opens (born-claimed suppression).
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
        agent_invite: None,
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

// E19b (found via proptest I13: an open window must own its reset
// exclusively, or the pipeline-hook path can complete around it, abandoning
// the window/claim/lease bookkeeping and leaving gate=RESET stuck alongside
// a phase that has already moved on) --------------------------------------

#[test]
fn e19b_reset_result_rejected_while_pre_window_open() {
    let mut d = Driver::new();
    open_with(&mut d, Some(teleop_window()), None);
    let at = d.tick();
    let res = d.try_apply(SessionEvent::ResetResult {
        ok: true,
        verified: Some(true),
        at,
    });
    assert!(
        res.is_err(),
        "reset_result illegal while a remote pre window is open (E19b)"
    );
    assert_eq!(d.phase(), Phase::Resetting, "unchanged");
    assert!(
        d.state.episode.as_ref().unwrap().reset_window.is_some(),
        "window untouched"
    );
}

#[test]
fn e19b_post_reset_result_rejected_while_post_window_open() {
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
    let res = d.try_apply(SessionEvent::PostResetResult {
        ok: true,
        detail: "hook raced the window".to_owned(),
        at,
    });
    assert!(
        res.is_err(),
        "post_reset_result illegal while a remote post window is open (E19b)"
    );
    assert_eq!(d.phase(), Phase::PostReset, "unchanged");
}

// E20/E21 interaction: an in-flight engage mint vs. racing events -----------
// The production reducer answers `MintLeaseToken` via the TAIL of its single
// event queue, so a COMPLETE the plane sent back-to-back with ENGAGE (or a
// legal `claim_released`) is processed BEFORE the mint answer. This is the
// deferred-mint regression rig for that ordering: pre-fix, an early COMPLETE
// closed the window, released the claim, went READY with the engage's
// `pending_lease` still populated, and the stale answer then handed the
// lease to the released claimant and panicked ("reset claim held"), killing
// the reducer thread.

#[test]
fn e21_complete_rejected_while_engage_mint_in_flight_then_honored() {
    let mut d = open_pre_and_grant(ActorKind::Teleoperator);
    d.defer_mints = true;
    let at = d.tick();
    d.ok(SessionEvent::ResetWindowEngage {
        claim: ClaimId::new("claim-rw"),
        at,
    });
    assert_eq!(d.outstanding_mints, 1, "engage mint in flight");

    // The racing COMPLETE: rejected, state untouched — the window never
    // observably ENGAGED, so there is nothing to honorably complete yet.
    let at = d.tick();
    let res = d.try_apply(SessionEvent::ResetWindowComplete {
        claim: ClaimId::new("claim-rw"),
        ok: true,
        verified: Some(true),
        at,
    });
    assert!(
        res.is_err(),
        "complete must be rejected while the engage mint is in flight"
    );
    assert_eq!(d.phase(), Phase::Resetting, "unchanged");
    assert!(d.state.claim.is_some(), "reset claim untouched");
    assert!(d.state.pending_lease.is_some(), "pending engage untouched");

    // The mint answer applies: window ENGAGED, gate → RESET, claimant holds.
    d.answer_mint().expect("engage mint applies");
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
    assert_eq!(d.state.gate_mode, GateMode::Reset);

    // The plane's retried COMPLETE now works end-to-end.
    let at = d.tick();
    d.ok(SessionEvent::ResetWindowComplete {
        claim: ClaimId::new("claim-rw"),
        ok: true,
        verified: Some(true),
        at,
    });
    d.answer_mint().expect("handback mint applies");
    assert_eq!(d.phase(), Phase::Ready);
    assert!(d.state.claim.is_none(), "C7 release");
    assert_eq!(d.state.gate_mode, GateMode::Passthrough);
    match &d.state.lease {
        LeaseState::Held { client, .. } => assert_eq!(client.as_str(), "loop-client"),
        LeaseState::Vacant => panic!("lease must be home with the loop client"),
    }
}

#[test]
fn stale_engage_mint_after_claim_release_never_panics() {
    let mut d = open_pre_and_grant(ActorKind::Teleoperator);
    d.defer_mints = true;
    let at = d.tick();
    d.ok(SessionEvent::ResetWindowEngage {
        claim: ClaimId::new("claim-rw"),
        at,
    });

    // A LEGAL `claim_released` races the mint answer (nothing closes the
    // window; the claim is simply gone before the token applies).
    let at = d.tick();
    d.ok(SessionEvent::ClaimReleased {
        id: ClaimId::new("claim-rw"),
        at,
    });
    assert!(d.state.claim.is_none());

    // Pre-fix this first applied the lease handoff to the released
    // claimant, then panicked ("reset claim held"). It must degrade: the
    // minted token is discarded and the lease does not move.
    let holder_before = d.state.lease.holder().map(|(_, c)| c.clone());
    d.answer_mint()
        .expect("stale mint answer is consumed, never a panic");
    assert_eq!(
        d.state.lease.holder().map(|(_, c)| c.clone()),
        holder_before,
        "lease must not move on a stale engage mint"
    );
    assert!(d.state.pending_lease.is_none());
    assert!(
        !d.state
            .episode
            .as_ref()
            .unwrap()
            .reset_window
            .as_ref()
            .unwrap()
            .engaged,
        "window still open, un-engaged"
    );
    assert_eq!(d.state.gate_mode, GateMode::Passthrough);

    // The window is still serviceable: a fresh claim engages and completes.
    grant(&mut d, "claim-rw2", ActorKind::Teleoperator).expect("C6 re-admits");
    let at = d.tick();
    d.ok(SessionEvent::ResetWindowEngage {
        claim: ClaimId::new("claim-rw2"),
        at,
    });
    d.answer_mint().expect("fresh engage mint applies");
    assert_eq!(d.state.gate_mode, GateMode::Reset);
    let at = d.tick();
    d.ok(SessionEvent::ResetWindowComplete {
        claim: ClaimId::new("claim-rw2"),
        ok: true,
        verified: Some(true),
        at,
    });
    d.answer_mint().expect("handback applies");
    assert_eq!(d.phase(), Phase::Ready);
}

#[test]
fn engage_overwriting_pending_initial_acquire_stays_sound() {
    // A pre-window episode opens with the loop client's initial acquire
    // still in flight; the window's engage overwrites the single
    // `pending_lease` slot. The first (uncorrelated) mint answer applies as
    // the ENGAGE op; the second finds nothing pending and is rejected.
    // Sound end state: one holder throughout, window engaged, and the
    // handback still routes the lease home.
    let mut d = Driver::new();
    d.defer_mints = true;
    open_with(&mut d, Some(teleop_window()), None);
    assert_eq!(d.outstanding_mints, 1, "initial acquire in flight");
    grant(&mut d, "claim-rw", ActorKind::Teleoperator).expect("C6 admits");
    let at = d.tick();
    d.ok(SessionEvent::ResetWindowEngage {
        claim: ClaimId::new("claim-rw"),
        at,
    });
    assert_eq!(
        d.outstanding_mints, 2,
        "engage overwrote the pending op; both answers still arrive"
    );

    d.answer_mint()
        .expect("first answer applies as the engage acquire");
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
    assert_eq!(d.state.gate_mode, GateMode::Reset);
    match &d.state.lease {
        LeaseState::Held { client, .. } => assert_eq!(client.as_str(), "teleop"),
        LeaseState::Vacant => panic!("claimant must hold the lease"),
    }

    let res = d.answer_mint();
    assert!(
        res.is_err(),
        "second answer finds no pending op and is rejected"
    );

    let at = d.tick();
    d.ok(SessionEvent::ResetWindowComplete {
        claim: ClaimId::new("claim-rw"),
        ok: true,
        verified: Some(true),
        at,
    });
    d.answer_mint().expect("handback applies");
    assert_eq!(d.phase(), Phase::Ready);
    match &d.state.lease {
        LeaseState::Held { client, .. } => assert_eq!(client.as_str(), "loop-client"),
        LeaseState::Vacant => panic!("lease must be home"),
    }
}

#[test]
fn window_timeout_during_engage_mint_stays_sound() {
    // E22 while the engage mint is in flight: the timeout path runs the
    // shared run-closing block, which clears `pending_lease` — the stale
    // answer is then rejected, never applied. (Pinned so a future refactor
    // of the timeout path cannot reintroduce the stale-mint hazard the
    // COMPLETE path had.)
    let mut d = open_pre_and_grant(ActorKind::Teleoperator);
    d.defer_mints = true;
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
        "E22 pre → abort"
    );
    assert!(
        d.state.pending_lease.is_none(),
        "the run-closing block cleared the pending engage"
    );
    let res = d.answer_mint();
    assert!(res.is_err(), "the stale answer is rejected, never applied");
    assert!(d.state.claim.is_none());
}
