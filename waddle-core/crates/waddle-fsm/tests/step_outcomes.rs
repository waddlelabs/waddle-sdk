//! The step-outcome contract behind directive acks (flag
//! `waddle.v0.plane.acks`): `step` returns `Ok(Step)` (Accepted) for every
//! event it applies — including legal no-ops that transition nothing — and
//! `Err(Rejected { reason })` for every illegal event, with the guard-row
//! name in the reason where one exists. The runtime forwards these reason
//! strings verbatim into `DirectiveAck.reason`, so this file doubles as the
//! documentation of every rejection path and pins each exact string.
//!
//! No behavior is new here: these are characterization tests of the
//! rejection surface `step` has always had. If one of these strings must
//! change, the ack surface changes with it — treat that as a reviewed
//! protocol-adjacent decision, not a refactor detail.

use waddle_fsm::{
    Effect, MarkKind, Phase, RejectReason, SessionConfig, SessionEvent, SessionFsm, WindowSpec,
    step,
};
use waddle_types::{
    ActorKind, ActorRef, ClaimId, EpisodeId, HandoffPolicy, LeaseEnforcement, LeaseId, MonoNs,
    ResetVerificationMode, TerminalOutcome, Verb,
};

struct Driver {
    cfg: SessionConfig,
    state: SessionFsm,
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
            lease_seq: 0,
            clock: 0,
        }
    }

    fn at(&mut self) -> MonoNs {
        self.clock += 1_000_000;
        MonoNs(self.clock)
    }

    /// Apply one event; on `MintLeaseToken` feed the completion back, as the
    /// runtime's reducer does.
    fn try_apply(&mut self, ev: SessionEvent) -> Result<(), String> {
        let stepped = step(&self.cfg, &self.state, &ev).map_err(|e| e.reason)?;
        self.state = stepped.next;
        let mut follow_ups = Vec::new();
        for effect in &stepped.effects {
            if let Effect::MintLeaseToken(_) = effect {
                self.lease_seq += 1;
                let at = self.at();
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

    fn ok(&mut self, ev: SessionEvent) {
        self.try_apply(ev).expect("scripted event must be Accepted");
    }

    /// Assert the event is Rejected with exactly `reason`, and that the
    /// rejection left the state unchanged (rejections never transition).
    fn rejected(&mut self, ev: SessionEvent, reason: &str) {
        let before = self.state.clone();
        match step(&self.cfg, &self.state, &ev) {
            Ok(_) => panic!("expected Rejected({reason}), got Accepted for {ev:?}"),
            Err(e) => assert_eq!(e.reason, reason, "rejection reason for {ev:?}"),
        }
        assert_eq!(self.state, before, "a rejection must not change state");
    }

    fn open(&mut self, pre_window: Option<WindowSpec>, post: PostDecl) {
        let at = self.at();
        let (post_reset, post_window) = match post {
            PostDecl::None => (false, None),
            PostDecl::Hook => (true, None),
            PostDecl::Window(w) => (true, Some(w)),
        };
        self.ok(SessionEvent::EpisodeOpen {
            id: EpisodeId::new("ep-outcomes"),
            verification: ResetVerificationMode::Blocking,
            born_claimed: false,
            parent: None,
            post_reset,
            pre_window,
            post_window,
            agent_invite: None,
            at,
        });
    }

    fn reset_ok(&mut self) {
        let at = self.at();
        self.ok(SessionEvent::ResetResult {
            ok: true,
            verified: Some(true),
            at,
        });
    }

    fn start(&mut self) {
        let at = self.at();
        self.ok(SessionEvent::Start { at });
    }

    fn grant(&mut self, id: &str, actor: ActorKind) -> Result<(), String> {
        let at = self.at();
        self.try_apply(SessionEvent::ClaimGranted {
            id: ClaimId::new(id),
            source: "plane".to_owned(),
            actor: ActorRef::of_kind(actor),
            self_initiated: false,
            at,
        })
    }

    fn engage(&mut self, id: &str) -> Result<(), String> {
        let at = self.at();
        self.try_apply(SessionEvent::Engage {
            claim: ClaimId::new(id),
            at,
        })
    }

    /// RUNNING, no claim.
    fn running() -> Self {
        let mut d = Self::new();
        d.open(None, PostDecl::None);
        d.reset_ok();
        d.start();
        d
    }

    /// Intervention SETTLE under claim `c1` (HOLD_FIRST: hold ok, lease
    /// handed off via the auto-fed mint completion).
    fn settled() -> Self {
        let mut d = Self::running();
        d.grant("c1", ActorKind::Teleoperator).expect("grant");
        d.engage("c1").expect("engage");
        let at = d.at();
        d.ok(SessionEvent::VerbResult {
            verb: Verb::Hold,
            ok: true,
            fault: None,
            at,
        });
        assert_eq!(
            d.phase(),
            Phase::Intervention(waddle_types::InterventionPhase::Settle)
        );
        d
    }

    fn phase(&self) -> Phase {
        self.state.episode.as_ref().expect("episode").phase
    }
}

enum PostDecl {
    None,
    Hook,
    Window(WindowSpec),
}

fn teleop_window() -> WindowSpec {
    WindowSpec {
        expected: ActorKind::Teleoperator,
        prompt: "reset the scene".to_owned(),
        timeout_ns: 600_000_000_000,
    }
}

fn ev_terminate(at: MonoNs) -> SessionEvent {
    SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "test".to_owned(),
        at,
    }
}

// --- Episode lifecycle rejections -----------------------------------------

#[test]
fn episode_open_while_active_is_rejected_n18() {
    let mut d = Driver::running();
    let at = d.at();
    d.rejected(
        SessionEvent::EpisodeOpen {
            id: EpisodeId::new("ep-second"),
            verification: ResetVerificationMode::Blocking,
            born_claimed: false,
            parent: None,
            post_reset: false,
            pre_window: None,
            post_window: None,
            agent_invite: None,
            at,
        },
        "one active episode per session (N18)",
    );
}

#[test]
fn born_claimed_open_without_a_held_claim_is_rejected() {
    let mut d = Driver::new();
    let at = d.at();
    d.rejected(
        SessionEvent::EpisodeOpen {
            id: EpisodeId::new("ep-born"),
            verification: ResetVerificationMode::Blocking,
            born_claimed: true,
            parent: None,
            post_reset: false,
            pre_window: None,
            post_window: None,
            agent_invite: None,
            at,
        },
        "a born-claimed successor requires a held claim",
    );
}

#[test]
fn reset_result_outside_resetting_is_rejected() {
    let mut d = Driver::running();
    let at = d.at();
    d.rejected(
        SessionEvent::ResetResult {
            ok: true,
            verified: Some(true),
            at,
        },
        "reset_result outside RESETTING",
    );
}

#[test]
fn reset_result_while_a_pre_window_is_open_is_rejected_e19b() {
    let mut d = Driver::new();
    d.open(Some(teleop_window()), PostDecl::None);
    let at = d.at();
    d.rejected(
        SessionEvent::ResetResult {
            ok: true,
            verified: Some(true),
            at,
        },
        "reset_result illegal while a remote reset window is open (E19b)",
    );
}

#[test]
fn verification_result_without_an_episode_is_rejected() {
    let mut d = Driver::new();
    let at = d.at();
    d.rejected(
        SessionEvent::VerificationResult {
            verified: true,
            invalidated_async: false,
            at,
        },
        "no episode",
    );
}

#[test]
fn start_outside_ready_is_rejected() {
    let mut d = Driver::new();
    d.open(None, PostDecl::None);
    let at = d.at();
    d.rejected(SessionEvent::Start { at }, "start outside READY");
}

// --- Claim rejections ------------------------------------------------------

#[test]
fn claim_request_without_an_active_episode_is_rejected() {
    let mut d = Driver::new();
    let at = d.at();
    d.rejected(
        SessionEvent::ClaimRequested {
            id: ClaimId::new("c1"),
            source: "plane".to_owned(),
            actor: ActorRef::of_kind(ActorKind::Teleoperator),
            self_initiated: false,
            at,
        },
        "claim_request without an active episode",
    );
}

#[test]
fn claim_granted_without_an_active_episode_is_rejected() {
    let mut d = Driver::new();
    assert_eq!(
        d.grant("c1", ActorKind::Teleoperator),
        Err("claim_granted without an active episode".to_owned())
    );
}

#[test]
fn conflicting_claim_grant_is_rejected() {
    let mut d = Driver::running();
    d.grant("c1", ActorKind::Teleoperator).expect("first grant");
    assert_eq!(
        d.grant("c2", ActorKind::Teleoperator),
        Err("conflicting active claim (one claim in v0)".to_owned())
    );
}

#[test]
fn c6_wrong_actor_reset_claim_is_rejected() {
    let mut d = Driver::new();
    d.open(Some(teleop_window()), PostDecl::None);
    // A TELEOPERATOR window admits AGENT never (C6).
    assert_eq!(
        d.grant("c-agent", ActorKind::Agent),
        Err("reset claim actor does not match the window's expected actor (C6)".to_owned())
    );
}

#[test]
fn releasing_an_unknown_or_absent_claim_is_rejected() {
    let mut d = Driver::running();
    let at = d.at();
    d.rejected(
        SessionEvent::ClaimReleased {
            id: ClaimId::new("c-ghost"),
            at,
        },
        "no active claim",
    );
    d.grant("c1", ActorKind::Teleoperator).expect("grant");
    let at = d.at();
    d.rejected(
        SessionEvent::ClaimReleased {
            id: ClaimId::new("c-other"),
            at,
        },
        "claim id mismatch",
    );
}

#[test]
fn releasing_the_claim_mid_intervention_is_rejected() {
    let mut d = Driver::settled();
    let at = d.at();
    d.rejected(
        SessionEvent::ClaimReleased {
            id: ClaimId::new("c1"),
            at,
        },
        "release the intervention (release/retake), not the claim",
    );
}

// --- Engage / release / retake rejections ----------------------------------

#[test]
fn e7_engage_outside_running_is_rejected() {
    // The exact shape the ack e2e exercises: a GRANT directive landing in
    // RESETTING with no window — ClaimGranted is Accepted (additive), the
    // Engage half is Rejected here.
    let mut d = Driver::new();
    d.open(None, PostDecl::None);
    d.grant("c1", ActorKind::Teleoperator)
        .expect("a grant in a reset phase WITHOUT an open window is additive");
    assert_eq!(
        d.engage("c1"),
        Err("engage outside RUNNING (E7)".to_owned())
    );
}

#[test]
fn engage_without_or_for_the_wrong_claim_is_rejected() {
    let mut d = Driver::running();
    assert_eq!(
        d.engage("c1"),
        Err("engage without a granted claim".to_owned())
    );
    d.grant("c1", ActorKind::Teleoperator).expect("grant");
    assert_eq!(
        d.engage("c-other"),
        Err("engage for a claim that is not active".to_owned())
    );
}

#[test]
fn e8_release_outside_settle_is_rejected() {
    let mut d = Driver::running();
    d.grant("c1", ActorKind::Teleoperator).expect("grant");
    let at = d.at();
    d.rejected(
        SessionEvent::Release {
            claim: ClaimId::new("c1"),
            at,
        },
        "release outside SETTLE (E8)",
    );
}

#[test]
fn release_for_a_non_active_claim_is_rejected() {
    let mut d = Driver::settled();
    let at = d.at();
    d.rejected(
        SessionEvent::Release {
            claim: ClaimId::new("c-other"),
            at,
        },
        "release for a claim that is not active",
    );
}

#[test]
fn retake_outside_settle_or_timed_out_engage_is_rejected() {
    let mut d = Driver::running();
    d.grant("c1", ActorKind::Teleoperator).expect("grant");
    let at = d.at();
    d.rejected(
        SessionEvent::Retake {
            claim: ClaimId::new("c1"),
            initiator: ActorKind::Teleoperator,
            successor: EpisodeId::new("ep-succ"),
            at,
        },
        "retake is legal from SETTLE (or ENGAGE after the settle timeout)",
    );
}

#[test]
fn retake_for_a_non_active_claim_is_rejected() {
    let mut d = Driver::settled();
    let at = d.at();
    d.rejected(
        SessionEvent::Retake {
            claim: ClaimId::new("c-other"),
            initiator: ActorKind::Teleoperator,
            successor: EpisodeId::new("ep-succ"),
            at,
        },
        "retake for a claim that is not active",
    );
}

#[test]
fn lease_minted_with_no_pending_operation_is_rejected() {
    let mut d = Driver::running();
    let at = d.at();
    d.rejected(
        SessionEvent::LeaseTokenMinted {
            minted: LeaseId::new("L-ghost"),
            at,
        },
        "no pending lease operation",
    );
}

// --- Terminate / attach rejections ------------------------------------------

#[test]
fn e12_terminate_on_a_terminal_episode_is_rejected() {
    let mut d = Driver::running();
    let at = d.at();
    d.ok(ev_terminate(at));
    assert!(d.phase().is_terminal());
    let at = d.at();
    d.rejected(
        ev_terminate(at),
        "terminate without an active episode (E12)",
    );
}

#[test]
fn e14b_terminate_in_post_reset_is_rejected() {
    let mut d = Driver::new();
    d.open(None, PostDecl::Hook);
    d.reset_ok();
    d.start();
    let at = d.at();
    d.ok(ev_terminate(at));
    assert_eq!(d.phase(), Phase::PostReset);
    let at = d.at();
    d.rejected(ev_terminate(at), "terminate rejected in POST_RESET (E14b)");
}

#[test]
fn judge_result_without_an_episode_is_rejected() {
    let mut d = Driver::new();
    let at = d.at();
    d.rejected(
        SessionEvent::JudgeResult {
            judge_id: "j1".to_owned(),
            passed: Some(true),
            at,
        },
        "no episode",
    );
}

#[test]
fn mark_without_an_active_episode_is_rejected_e12() {
    let mut d = Driver::new();
    let at = d.at();
    d.rejected(
        SessionEvent::Mark {
            kind: MarkKind::EndSuccess,
            at,
        },
        "mark without an active episode (E12)",
    );
}

#[test]
fn dual_write_without_an_active_episode_is_rejected() {
    let mut d = Driver::new();
    let at = d.at();
    d.rejected(
        SessionEvent::DualWrite {
            divergence_metric: 0.5,
            window_ns: 1_000_000,
            trace_ref: "clip-1".to_owned(),
            at,
        },
        "dual-write detection without an active episode",
    );
}

#[test]
fn intervention_rejected_without_an_active_episode_is_rejected() {
    let mut d = Driver::new();
    let at = d.at();
    d.rejected(
        SessionEvent::InterventionRejected {
            source: "media-intake",
            reason: RejectReason::Dims { got: 9, want: 6 },
            at,
        },
        "intervention_rejected without an active episode",
    );
}

// --- Post-reset / reset-window rejections -----------------------------------

#[test]
fn post_reset_result_outside_post_reset_is_rejected() {
    let mut d = Driver::running();
    let at = d.at();
    d.rejected(
        SessionEvent::PostResetResult {
            ok: true,
            detail: String::new(),
            at,
        },
        "post_reset_result outside POST_RESET",
    );
}

#[test]
fn post_reset_result_while_a_post_window_is_open_is_rejected_e19b() {
    let mut d = Driver::new();
    d.open(None, PostDecl::Window(teleop_window()));
    d.reset_ok();
    d.start();
    let at = d.at();
    d.ok(ev_terminate(at));
    assert_eq!(d.phase(), Phase::PostReset);
    let at = d.at();
    d.rejected(
        SessionEvent::PostResetResult {
            ok: true,
            detail: String::new(),
            at,
        },
        "post_reset_result illegal while a remote reset window is open (E19b)",
    );
}

#[test]
fn reset_window_engage_rejections() {
    // Outside a reset phase.
    let mut d = Driver::running();
    let at = d.at();
    d.rejected(
        SessionEvent::ResetWindowEngage {
            claim: ClaimId::new("c1"),
            at,
        },
        "reset_window_engage outside a reset phase",
    );

    // In a reset phase but with no window open.
    let mut d = Driver::new();
    d.open(None, PostDecl::None);
    let at = d.at();
    d.rejected(
        SessionEvent::ResetWindowEngage {
            claim: ClaimId::new("c1"),
            at,
        },
        "no open reset window",
    );

    // Window open but no granted reset claim (the second half of a
    // wrong-actor ENGAGE directive, after C6 already rejected the grant).
    let mut d = Driver::new();
    d.open(Some(teleop_window()), PostDecl::None);
    let at = d.at();
    d.rejected(
        SessionEvent::ResetWindowEngage {
            claim: ClaimId::new("c1"),
            at,
        },
        "reset_window_engage without the granted reset claim",
    );

    // Already engaged.
    let mut d = Driver::new();
    d.open(Some(teleop_window()), PostDecl::None);
    d.grant("c-reset", ActorKind::Teleoperator).expect("C6 ok");
    let at = d.at();
    d.ok(SessionEvent::ResetWindowEngage {
        claim: ClaimId::new("c-reset"),
        at,
    });
    let at = d.at();
    d.rejected(
        SessionEvent::ResetWindowEngage {
            claim: ClaimId::new("c-reset"),
            at,
        },
        "reset window already engaged",
    );
}

#[test]
fn reset_window_complete_rejections() {
    // Outside a reset phase.
    let mut d = Driver::running();
    let at = d.at();
    d.rejected(
        SessionEvent::ResetWindowComplete {
            claim: ClaimId::new("c1"),
            ok: true,
            verified: None,
            at,
        },
        "reset_window_complete outside a reset phase",
    );

    // No window open.
    let mut d = Driver::new();
    d.open(None, PostDecl::None);
    let at = d.at();
    d.rejected(
        SessionEvent::ResetWindowComplete {
            claim: ClaimId::new("c1"),
            ok: true,
            verified: None,
            at,
        },
        "no open reset window",
    );

    // Window open, wrong claim.
    let mut d = Driver::new();
    d.open(Some(teleop_window()), PostDecl::None);
    d.grant("c-reset", ActorKind::Teleoperator).expect("C6 ok");
    let at = d.at();
    d.rejected(
        SessionEvent::ResetWindowComplete {
            claim: ClaimId::new("c-other"),
            ok: true,
            verified: None,
            at,
        },
        "reset_window_complete without the active reset claim",
    );
}

// --- Accepted paths ----------------------------------------------------------

#[test]
fn the_ack_relevant_directive_events_are_accepted_when_legal() {
    // GRANT during RUNNING: both halves (ClaimGranted + Engage) Accepted.
    let mut d = Driver::running();
    assert_eq!(d.grant("c1", ActorKind::Teleoperator), Ok(()));
    assert_eq!(d.engage("c1"), Ok(()));

    // TERMINATE on a live episode: Accepted.
    let mut d = Driver::running();
    let at = d.at();
    assert_eq!(d.try_apply(ev_terminate(at)), Ok(()));

    // Reset-window ENGAGE then COMPLETE with the right actor/claim: both
    // halves Accepted end-to-end (lease completions auto-fed).
    let mut d = Driver::new();
    d.open(Some(teleop_window()), PostDecl::None);
    assert_eq!(d.grant("c-reset", ActorKind::Teleoperator), Ok(()));
    let at = d.at();
    assert_eq!(
        d.try_apply(SessionEvent::ResetWindowEngage {
            claim: ClaimId::new("c-reset"),
            at,
        }),
        Ok(())
    );
    let at = d.at();
    assert_eq!(
        d.try_apply(SessionEvent::ResetWindowComplete {
            claim: ClaimId::new("c-reset"),
            ok: true,
            verified: Some(true),
            at,
        }),
        Ok(())
    );
    assert_eq!(d.phase(), Phase::Ready);
}

#[test]
fn legal_no_op_events_are_accepted_not_rejected() {
    // Ok-with-no-transition is still Accepted — the ack channel treats
    // "applied, nothing to do" as success, never as a rejection.
    let mut d = Driver::new();
    d.open(None, PostDecl::None);
    // A gate tick outside READY transitions nothing.
    let at = d.at();
    assert_eq!(d.try_apply(SessionEvent::GateTick { at }), Ok(()));
    // A chunk boundary with no engage awaiting it transitions nothing.
    let at = d.at();
    assert_eq!(
        d.try_apply(SessionEvent::ChunkBoundaryReached { at }),
        Ok(())
    );
    // A clutch edge outside RUNNING is recorded by the source, not applied.
    let at = d.at();
    assert_eq!(
        d.try_apply(SessionEvent::Clutch { engaged: true, at }),
        Ok(())
    );
    assert!(d.state.claim.is_none(), "no claim minted outside RUNNING");
}
