//! Property suites for the session FSM — the invariants named in the plan:
//!
//! 1. TERMINAL is absorbing (a terminal episode's phase never changes).
//! 2. At most one lease holder; dead tokens never return.
//! 3. Handoff installs a fresh token; holder re-acquire is idempotent.
//! 4. Estop (revoke-all) always leaves the lease vacant.
//! 5. Retake emits exactly one OpenSuccessor carrying the held claim,
//!    born-claimed.
//! 6. A Blocking-mode episode past RESETTING is always verified;
//!    an unverified optimistic episode is flagged, never silently clean.
//! 7. Gate mode INTERVENTION implies an active claim; SETTLE is entered
//!    only from ENGAGE.
//! 8. `step` is deterministic and the state stays serializable.
//! 9. `phase == PostReset ⇒ post_reset_declared` (FSM.md §1.3).
//! 10. `pinned_outcome` is set-once; any Terminal reached from PostReset
//!     carries it unchanged (including via Estop) — PostReset is followed
//!     only by Terminal{pinned}.
//! 11. Estop from PostReset ⇒ next phase Terminal ∧ lease Vacant (E17).
//! 12. `post_reset_failed` is monotone; false at Terminal (from PostReset)
//!     ⇒ the last post-reset result was ok.
//! 13. `gate_mode == Reset ⇒ claim.is_some() ∧ phase ∈ {Resetting, PostReset}`
//!     (the stale-handle contract).
//! 14. Retake acceptance ⇒ predecessor Terminal{ABORTED_RETAKE} with no
//!     intervening PostReset phase (E18 bypass).
//! 15. `agent_engaged ⇒ agent_invited` (§1.5 latch).
//! 16. In an agent-invited episode, every ENGAGEd (RUNNING-phase) claim has
//!     actor AGENT (C8) — reset-window claims (C6) never enter INTERVENTION.
//! 17. Agent-invited ∧ never engaged ∧ invite timer fired ⇒ Terminal{ABORT}
//!     (or PostReset{ABORT pinned}, then Terminal, when post-reset declared —
//!     E25 through E14).
//! 18. `AgentTaskDenied` after engage never changes phase (E26b).
//! 19. `invite_aborted ⇒ agent_invited ∧ ¬agent_engaged` (§1.5: only E25/E26
//!     latch it, and both require the invite open), and it is monotone
//!     within an episode.
//! 20. Gate-plan re-projection: any step that changes the FSM-owned gate-plan
//!     inputs — `(gate_mode, agent_episode_noop())` — leaves an
//!     `Effect::SetGateMode` carrying the step's FINAL mode. Plan derivers
//!     re-project only on that effect, so without it a deriver keeps a stale
//!     plan (the caller's ticks noop'ing, or dispatching, forever).

use std::collections::HashSet;

use proptest::prelude::*;
use waddle_fsm::{
    AgentInvite, Effect, EpisodeState, LeaseState, MarkKind, Phase, ProxySample, Rejected,
    SessionConfig, SessionEvent, SessionFsm, TimerId, WindowSpec, step,
};
use waddle_types::{
    ActorKind, ActorRef, ClaimId, EpisodeId, GateMode, Grant, HandoffPolicy, InterventionPhase,
    LeaseEnforcement, LeaseId, MonoNs, ResetVerificationMode, TerminalOutcome, Verb, pb::v0 as pb,
};

/// The abstract command alphabet the random walk draws from.
#[derive(Debug, Clone)]
enum Cmd {
    Open {
        optimistic: bool,
    },
    /// Declares a post-reset (flag `waddle.v0.reset.phases`) at open,
    /// independently varying a remote PRE window and a remote POST window
    /// (flag `waddle.v0.reset.remote`) across cases.
    OpenPostReset {
        optimistic: bool,
        pre_window: bool,
        post_window: bool,
    },
    /// Opens an agent-invited episode (flag `waddle.v0.agent`, E23),
    /// independently varying a declared post-reset (hook or remote window)
    /// so I17's E14 detour and E26b's POST_RESET inertness get walked, and
    /// so C6 window claims inside an agent-invited episode stay covered.
    OpenAgent {
        optimistic: bool,
        post_reset: bool,
        post_window: bool,
    },
    /// The plane denied the agent task (E26 before engage, E26b after).
    AgentDenied,
    ResetOk {
        verified: Option<bool>,
    },
    ResetFail,
    Verification {
        verified: bool,
        invalidated: bool,
    },
    Start,
    /// A caller-loop gate tick (the stale-handle contract): must not transition the episode
    /// out of RESETTING/POST_RESET — the guard against a stale handle
    /// double-driving a reset while a remote actor (or the pipeline hook)
    /// owns it.
    GateTick,
    /// `agent` picks the granted actor (AGENT vs TELEOPERATOR) so C8
    /// admission on agent-invited episodes is walked from both sides.
    ClaimGranted {
        agent: bool,
    },
    Engage,
    HoldOk,
    ChunkBoundary,
    Release,
    Retake {
        by_teleoperator: bool,
    },
    Clutch {
        engaged: bool,
    },
    Estop,
    Terminate {
        success: bool,
    },
    Mark {
        end: bool,
    },
    /// The post-reset pipeline reported (E15/E16); legal only in PostReset.
    PostResetOk,
    PostResetFail,
    /// The granted reset claim engages the open window (E20).
    WindowEngage,
    /// The remote actor finished (E21).
    WindowComplete {
        ok: bool,
    },
    Proxy {
        p95_ns: i64,
    },
    PartitionStart,
    PartitionEnd,
    Advance {
        ns: i64,
    },
    /// Toggle deferred lease-mint answers. The production reducer answers
    /// `MintLeaseToken` via the TAIL of its single event queue, so any event
    /// already queued (a plane's back-to-back COMPLETE, a `claim_released`)
    /// is processed BEFORE the answer — deferral lets the walk explore
    /// exactly those interleavings.
    DeferMints {
        defer: bool,
    },
    /// Deliver ONE deferred mint answer (a no-op when none is outstanding —
    /// the runtime sends exactly one answer per mint effect, never more).
    AnswerMint,
}

/// A fixed remote-window spec (flag `waddle.v0.reset.remote`): the actor is
/// always TELEOPERATOR to match the walk's only claim-granting actor
/// (`Cmd::ClaimGranted`), so C6 admission is exercised without spuriously
/// rejecting otherwise-legal claims. `timeout_ns` sits inside the walk's
/// `Advance` range so `ResetWindowTimeout` fires under random exploration.
fn reset_window_spec() -> WindowSpec {
    WindowSpec {
        expected: ActorKind::Teleoperator,
        prompt: "proptest reset".to_owned(),
        timeout_ns: 2_000_000_000,
    }
}

fn cmd_strategy() -> impl Strategy<Value = Cmd> {
    prop_oneof![
        3 => any::<bool>().prop_map(|optimistic| Cmd::Open { optimistic }),
        2 => (any::<bool>(), any::<bool>(), any::<bool>()).prop_map(
            |(optimistic, pre_window, post_window)| Cmd::OpenPostReset {
                optimistic,
                pre_window,
                post_window,
            }
        ),
        3 => prop_oneof![
            Just(Cmd::ResetOk { verified: Some(true) }),
            Just(Cmd::ResetOk { verified: Some(false) }),
            Just(Cmd::ResetOk { verified: None }),
        ],
        1 => Just(Cmd::ResetFail),
        2 => (any::<bool>(), any::<bool>())
            .prop_map(|(verified, invalidated)| Cmd::Verification { verified, invalidated }),
        2 => (any::<bool>(), any::<bool>(), any::<bool>()).prop_map(
            |(optimistic, post_reset, post_window)| Cmd::OpenAgent {
                optimistic,
                post_reset,
                post_window,
            }
        ),
        3 => Just(Cmd::Start),
        2 => Just(Cmd::GateTick),
        3 => any::<bool>().prop_map(|agent| Cmd::ClaimGranted { agent }),
        1 => Just(Cmd::AgentDenied),
        3 => Just(Cmd::Engage),
        2 => Just(Cmd::HoldOk),
        2 => Just(Cmd::ChunkBoundary),
        2 => Just(Cmd::Release),
        2 => any::<bool>().prop_map(|by_teleoperator| Cmd::Retake { by_teleoperator }),
        2 => any::<bool>().prop_map(|engaged| Cmd::Clutch { engaged }),
        1 => Just(Cmd::Estop),
        1 => any::<bool>().prop_map(|success| Cmd::Terminate { success }),
        1 => any::<bool>().prop_map(|end| Cmd::Mark { end }),
        1 => Just(Cmd::PostResetOk),
        1 => Just(Cmd::PostResetFail),
        1 => Just(Cmd::WindowEngage),
        2 => any::<bool>().prop_map(|ok| Cmd::WindowComplete { ok }),
        2 => (50_000_000i64..200_000_000).prop_map(|p95_ns| Cmd::Proxy { p95_ns }),
        1 => Just(Cmd::PartitionStart),
        1 => Just(Cmd::PartitionEnd),
        2 => (1_000_000i64..5_000_000_000).prop_map(|ns| Cmd::Advance { ns }),
        2 => any::<bool>().prop_map(|defer| Cmd::DeferMints { defer }),
        3 => Just(Cmd::AnswerMint),
    ]
}

/// A compact rendering of the emission-relevant effects, for the
/// deterministic smoke test's ordering assertions (mirrors
/// `tests/remote_reset_windows.rs`'s `render`).
fn render(effect: &Effect) -> String {
    match effect {
        Effect::Emit(ev) => match &ev.event {
            Some(pb::episode_event::Event::State(s)) => {
                format!("state->{} outcome={}", s.to, s.outcome)
            }
            Some(pb::episode_event::Event::Gate(g)) => format!("gate {}->{}", g.from, g.to),
            Some(pb::episode_event::Event::ResetWindow(w)) => {
                format!("reset_window kind={}", w.kind)
            }
            Some(pb::episode_event::Event::Claim(c)) => format!("claim kind={}", c.kind),
            _ => "emit other".to_owned(),
        },
        _ => "effect other".to_owned(),
    }
}

struct Driver {
    cfg: SessionConfig,
    state: SessionFsm,
    now: MonoNs,
    lease_seq: u32,
    claim_seq: u32,
    episode_seq: u32,
    /// Every token ever installed as current.
    tokens_ever_held: HashSet<LeaseId>,
    /// Tokens that were current once and then stopped being current.
    dead_tokens: HashSet<LeaseId>,
    armed: Vec<(TimerId, MonoNs)>,
    last_phase: Option<(EpisodeId, Phase)>,
    /// I10: the last-seen `pinned_outcome` per episode, to catch it changing
    /// after being set.
    last_pinned_outcome: Option<(EpisodeId, TerminalOutcome)>,
    /// I12: the last-seen `post_reset_failed` per episode, to catch it
    /// reverting from true to false.
    last_post_reset_failed: Option<(EpisodeId, bool)>,
    last_invite_aborted: Option<(EpisodeId, bool)>,
    /// I14: episodes that have ever visited PostReset (retake must never
    /// have passed through it).
    ever_post_reset: HashSet<EpisodeId>,
    /// A rendering of every effect emitted, in commit order, for the
    /// deterministic smoke test's emission-order assertions (unused by the
    /// random walk itself).
    trace: Vec<String>,
    /// `Cmd::DeferMints`: when true, `MintLeaseToken` effects queue instead
    /// of being answered inline (see the Cmd's doc).
    defer_mints: bool,
    /// Mint effects awaiting a `Cmd::AnswerMint`.
    outstanding_mints: u32,
}

impl Driver {
    fn new() -> Self {
        let mut cfg = SessionConfig::minimal(
            "customer-loop",
            HandoffPolicy::Immediate { blend_ns: 0 },
            LeaseEnforcement::Advisory,
        );
        cfg.grants = vec![Grant {
            verb: Verb::Hold,
            send_interfaces: vec![],
            declared_latency_bound_ns: Some(100_000_000),
            hardware: false,
        }];
        let state = SessionFsm::new(&cfg);
        Self {
            cfg,
            state,
            now: MonoNs(1_000_000_000),
            lease_seq: 0,
            claim_seq: 0,
            episode_seq: 0,
            tokens_ever_held: HashSet::new(),
            dead_tokens: HashSet::new(),
            armed: Vec::new(),
            last_phase: None,
            last_pinned_outcome: None,
            last_post_reset_failed: None,
            last_invite_aborted: None,
            ever_post_reset: HashSet::new(),
            trace: Vec::new(),
            defer_mints: false,
            outstanding_mints: 0,
        }
    }

    fn tick(&mut self) -> MonoNs {
        self.now = self.now.saturating_add(1_000_000);
        self.now
    }

    /// The index of the first traced effect matching `pred`, for the
    /// deterministic smoke test's emission-order assertions.
    fn index_of<F: Fn(&str) -> bool>(&self, pred: F) -> Option<usize> {
        self.trace.iter().position(|s| pred(s))
    }

    /// Apply an event; interpret effects; check per-step invariants.
    fn apply(&mut self, ev: SessionEvent) {
        // Invariant 8: determinism — two identical steps agree.
        let first = step(&self.cfg, &self.state, &ev);
        let second = step(&self.cfg, &self.state, &ev);
        match (&first, &second) {
            (Ok(a), Ok(b)) => {
                assert_eq!(a.next, b.next, "step must be deterministic");
                assert_eq!(a.effects.len(), b.effects.len());
            }
            (Err(a), Err(b)) => assert_eq!(a, b),
            _ => panic!("nondeterministic accept/reject"),
        }

        let retake_expected = matches!(ev, SessionEvent::Retake { .. });
        let estop = matches!(ev, SessionEvent::Estop { .. });
        let was_post_reset = matches!(
            self.state.episode.as_ref().map(|e| e.phase),
            Some(Phase::PostReset)
        );
        // the stale-handle contract: a GateTick landing in RESETTING/POST_RESET must not
        // transition the phase — those windows are owned by a remote actor
        // (or the pipeline hook), and the gate is already returning
        // Noop{RESET_ACTIVE} to any stale caller ticking it (waddle-gate's
        // PlanMode::Reset). Captured BEFORE the event so a rejection or a
        // no-op commit is compared against the true prior phase.
        let gate_tick = matches!(ev, SessionEvent::GateTick { .. });
        let phase_before_gate_tick = if gate_tick {
            self.state.episode.as_ref().map(|e| e.phase)
        } else {
            None
        };
        // I17: the invite timer firing while the invite is open must
        // terminate with ABORT (directly, or through the E14 detour with
        // ABORT pinned). Captured BEFORE the event; a stale expiry (invite
        // closed) is exempt — it must be discarded instead.
        let invite_timeout_while_open = matches!(
            ev,
            SessionEvent::TimerFired {
                id: TimerId::AgentInviteTimeout,
                ..
            }
        ) && self
            .state
            .episode
            .as_ref()
            .is_some_and(EpisodeState::invite_open);
        // I18: a DENIED landing after the agent engaged must never change
        // phase (E26b).
        let denied_after_engage = matches!(ev, SessionEvent::AgentTaskDenied { .. })
            && self.state.episode.as_ref().is_some_and(|e| e.agent_engaged);
        let phase_before_denied = if denied_after_engage {
            self.state.episode.as_ref().map(|e| e.phase)
        } else {
            None
        };
        // I20: the gate plan is DERIVED state, re-projected by plan derivers
        // only when they see `Effect::SetGateMode` — captured BEFORE the
        // event so the pair can be compared across the step.
        let plan_inputs_before = (self.state.gate_mode, self.state.agent_episode_noop());
        match first {
            Err(Rejected { .. }) => {
                // Rejections never mutate: nothing to fold in.
            }
            Ok(s) => {
                let prev_holder = self.state.lease.holder().map(|(l, _)| l.clone());
                self.state = s.next;
                let new_holder = self.state.lease.holder().map(|(l, _)| l.clone());
                if prev_holder != new_holder {
                    if let Some(old) = prev_holder {
                        self.dead_tokens.insert(old);
                    }
                    if let Some(new) = &new_holder {
                        // Invariant 2/3: a dead token never becomes current
                        // again; fresh tokens are genuinely fresh.
                        assert!(
                            !self.dead_tokens.contains(new),
                            "dead lease token became current again"
                        );
                        self.tokens_ever_held.insert(new.clone());
                    }
                }

                // I11: estop from PostReset lands directly in Terminal, lease
                // vacant (E17). Checked against the phase just BEFORE this
                // event, since check_invariants only sees the committed
                // (post) state.
                if estop && was_post_reset {
                    assert!(
                        self.state
                            .episode
                            .as_ref()
                            .is_some_and(|e| e.phase.is_terminal()),
                        "I11: estop from PostReset must reach Terminal"
                    );
                    assert_eq!(
                        self.state.lease,
                        LeaseState::Vacant,
                        "I11: estop from PostReset must leave the lease vacant"
                    );
                }

                // I17: E25 fired while the invite was open — the episode
                // terminates with ABORT, directly or via the E14 detour
                // (POST_RESET with ABORT pinned; I10's machinery then pins
                // the eventual Terminal).
                if invite_timeout_while_open {
                    let ep = self.state.episode.as_ref().expect("episode was open");
                    assert!(
                        ep.phase == Phase::Terminal(TerminalOutcome::Abort)
                            || (ep.phase == Phase::PostReset
                                && ep.pinned_outcome == Some(TerminalOutcome::Abort)),
                        "I17: open-invite timeout must reach Terminal{{ABORT}} or \
                         PostReset{{ABORT pinned}}, got {:?} (pinned {:?})",
                        ep.phase,
                        ep.pinned_outcome
                    );
                }

                // I20: the plan inputs moved ⇒ this step's effects carry a
                // re-projection for the mode it ended in. A stale plan is
                // invisible to the FSM's own state assertions — this is the
                // only place it can be caught.
                let plan_inputs_after = (self.state.gate_mode, self.state.agent_episode_noop());
                if plan_inputs_after != plan_inputs_before {
                    let last_mode = s.effects.iter().rev().find_map(|e| match e {
                        Effect::SetGateMode(mode) => Some(*mode),
                        _ => None,
                    });
                    assert_eq!(
                        last_mode,
                        Some(self.state.gate_mode),
                        "I20: gate-plan inputs moved {plan_inputs_before:?} -> \
                         {plan_inputs_after:?} without a SetGateMode re-projection"
                    );
                }

                // I18: an accepted DENIED after engage would be a bug; a
                // rejected one never mutates. Either way the phase stands.
                if denied_after_engage {
                    assert_eq!(
                        self.state.episode.as_ref().map(|e| e.phase),
                        phase_before_denied,
                        "I18: AgentTaskDenied after engage must never change phase (E26b)"
                    );
                }

                // the stale-handle contract: GateTick in RESETTING/POST_RESET is a no-op —
                // it must never drive a transition (unlike a GateTick landing
                // in READY, which is E6's first-gated-action trigger).
                if gate_tick
                    && matches!(
                        phase_before_gate_tick,
                        Some(Phase::Resetting) | Some(Phase::PostReset)
                    )
                {
                    assert_eq!(
                        self.state.episode.as_ref().map(|e| e.phase),
                        phase_before_gate_tick,
                        "the stale-handle contract: GateTick in RESETTING/POST_RESET must not transition"
                    );
                }

                // I12: this step's post-reset completion result (if any),
                // scanned from the emissions BEFORE check_invariants runs —
                // E15/E16 land in the very same step as the Terminal
                // transition they cause.
                let post_reset_ok_this_step = s.effects.iter().find_map(|e| match e {
                    Effect::Emit(ev) => match &ev.event {
                        Some(pb::episode_event::Event::PostReset(p)) => {
                            Some(p.result.as_ref().is_some_and(|r| r.ok))
                        }
                        _ => None,
                    },
                    _ => None,
                });

                // Check (and record) the committed state BEFORE follow-up
                // events run, so intermediate phases are observed.
                self.check_invariants(retake_expected, post_reset_ok_this_step);

                let mut successors = 0;
                let mut follow_ups: Vec<SessionEvent> = Vec::new();
                for effect in &s.effects {
                    self.trace.push(render(effect));
                    match effect {
                        Effect::MintLeaseToken(_) => {
                            if self.defer_mints {
                                self.outstanding_mints += 1;
                            } else {
                                self.lease_seq += 1;
                                follow_ups.push(SessionEvent::LeaseTokenMinted {
                                    minted: LeaseId::new(format!("L{}", self.lease_seq)),
                                    at: self.tick(),
                                });
                            }
                        }
                        Effect::OpenSuccessor {
                            successor,
                            claim,
                            born_claimed,
                            mode,
                            ..
                        } => {
                            successors += 1;
                            // Invariant 5.
                            assert!(born_claimed, "retake successor must be born claimed");
                            assert_eq!(
                                Some(claim),
                                self.state.claim.as_ref().map(|c| &c.id),
                                "successor must carry the still-held claim"
                            );
                            follow_ups.push(SessionEvent::EpisodeOpen {
                                id: successor.clone(),
                                verification: *mode,
                                born_claimed: true,
                                parent: self.state.episode.as_ref().map(|e| e.id.clone()),
                                post_reset: false,
                                pre_window: None,
                                post_window: None,
                                agent_invite: None,
                                at: self.tick(),
                            });
                        }
                        Effect::ArmTimer { id, deadline } => {
                            self.armed.retain(|(t, _)| t != id);
                            self.armed.push((*id, *deadline));
                        }
                        Effect::CancelTimer { id } => {
                            self.armed.retain(|(t, _)| t != id);
                        }
                        _ => {}
                    }
                }
                if retake_expected {
                    assert_eq!(successors, 1, "retake emits exactly one OpenSuccessor");
                }
                for f in follow_ups {
                    self.apply(f);
                }
            }
        }
    }

    fn check_invariants(&mut self, retake_expected: bool, post_reset_ok_this_step: Option<bool>) {
        let s = &self.state;

        // Invariant 1: terminal is absorbing per episode identity.
        if let Some(ep) = &s.episode {
            // I9: PostReset implies the episode declared one.
            if ep.phase == Phase::PostReset {
                assert!(
                    ep.post_reset_declared,
                    "I9: PostReset phase requires post_reset_declared"
                );
                self.ever_post_reset.insert(ep.id.clone());
            }

            // I14: retake acceptance ⇒ the predecessor lands directly in
            // TERMINAL{ABORTED_RETAKE}, never having visited PostReset (E18
            // bypasses it regardless of whether a post-reset is declared).
            if retake_expected {
                assert_eq!(
                    ep.phase,
                    Phase::Terminal(TerminalOutcome::AbortedRetake),
                    "I14: retake must terminate the predecessor with ABORTED_RETAKE"
                );
                assert!(
                    !self.ever_post_reset.contains(&ep.id),
                    "I14: retake predecessor must not have visited PostReset (E18 bypass)"
                );
            }

            if let Some((last_id, last_phase)) = &self.last_phase
                && last_id == &ep.id
            {
                if let Phase::Terminal(prev) = last_phase {
                    assert_eq!(
                        ep.phase,
                        Phase::Terminal(*prev),
                        "terminal episode changed phase"
                    );
                }
                // Invariant 7: SETTLE is entered only from ENGAGE.
                if ep.phase == Phase::Intervention(InterventionPhase::Settle)
                    && *last_phase != Phase::Intervention(InterventionPhase::Settle)
                {
                    assert_eq!(
                        *last_phase,
                        Phase::Intervention(InterventionPhase::Engage),
                        "SETTLE entered from a phase other than ENGAGE"
                    );
                }

                // I10/I12: PostReset may be followed only by Terminal{pinned}.
                if *last_phase == Phase::PostReset && ep.phase != Phase::PostReset {
                    match ep.phase {
                        Phase::Terminal(outcome) => {
                            let pinned = ep
                                .pinned_outcome
                                .expect("I9/I10: PostReset must pin an outcome before leaving");
                            assert_eq!(
                                outcome, pinned,
                                "I10: Terminal-after-PostReset must carry the pinned outcome"
                            );
                            // I12: false at Terminal ⇒ the last post-reset
                            // result was ok.
                            if !ep.post_reset_failed {
                                assert_eq!(
                                    post_reset_ok_this_step,
                                    Some(true),
                                    "I12: post_reset_failed=false at Terminal-from-PostReset \
                                     requires the last post-reset result to be ok"
                                );
                            }
                        }
                        other => panic!(
                            "I10: PostReset must be followed only by Terminal{{pinned}}, got {other:?}"
                        ),
                    }
                }
            }
            self.last_phase = Some((ep.id.clone(), ep.phase));

            // I10: pinned_outcome is set-once.
            if let Some(outcome) = ep.pinned_outcome {
                if let Some((last_id, last_outcome)) = &self.last_pinned_outcome
                    && last_id == &ep.id
                {
                    assert_eq!(
                        *last_outcome, outcome,
                        "I10: pinned_outcome changed after being set"
                    );
                }
                self.last_pinned_outcome = Some((ep.id.clone(), outcome));
            }

            // I12: post_reset_failed is monotone (never true → false).
            if let Some((last_id, last_failed)) = &self.last_post_reset_failed
                && last_id == &ep.id
            {
                assert!(
                    !*last_failed || ep.post_reset_failed,
                    "I12: post_reset_failed must be monotone"
                );
            }
            self.last_post_reset_failed = Some((ep.id.clone(), ep.post_reset_failed));

            // I15: the §1.5 latch — agent_engaged only ever on an
            // agent-invited episode.
            if ep.agent_engaged {
                assert!(ep.agent_invited, "I15: agent_engaged without agent_invited");
            }

            // I19: invite_aborted is E25/E26's latch alone — both rows
            // require the invite open, so it never coexists with an engage
            // and never appears on a non-invited episode. Monotone within
            // the episode (like the other §1.5 latches).
            if ep.invite_aborted {
                assert!(
                    ep.agent_invited && !ep.agent_engaged,
                    "I19: invite_aborted requires agent_invited and no engage"
                );
            }
            if let Some((last_id, last_aborted)) = &self.last_invite_aborted
                && last_id == &ep.id
            {
                assert!(
                    !*last_aborted || ep.invite_aborted,
                    "I19: invite_aborted must be monotone"
                );
            }
            self.last_invite_aborted = Some((ep.id.clone(), ep.invite_aborted));

            // I16: every ENGAGEd (RUNNING-phase, E7) claim in an
            // agent-invited episode is the invited agent's (C8). Reset-window
            // claims (C6) never enter INTERVENTION, so the phase check
            // scopes this exactly to E7 engages.
            if ep.agent_invited && matches!(ep.phase, Phase::Intervention(_)) {
                assert!(
                    s.claim
                        .as_ref()
                        .is_some_and(|c| c.actor.kind == ActorKind::Agent),
                    "I16: engaged claim in an agent-invited episode must be ACTOR_KIND_AGENT"
                );
            }

            // Invariant 6: verified-or-flagged.
            if !matches!(ep.phase, Phase::Resetting) && !ep.phase.is_terminal() {
                match ep.verification {
                    ResetVerificationMode::Blocking => {
                        assert!(ep.verified, "Blocking episode ran unverified");
                    }
                    ResetVerificationMode::OptimisticAsync => {
                        assert!(
                            ep.verified || ep.optimistic_entry || ep.reset_unverified,
                            "optimistic episode neither verified, optimistic, nor flagged"
                        );
                    }
                }
            }
        }

        // Invariant 7: gate INTERVENTION implies an active claim.
        if s.gate_mode == GateMode::Intervention {
            assert!(s.claim.is_some(), "gate claimed with no active claim");
        }

        // I13: gate RESET implies an active claim and phase ∈ {Resetting,
        // PostReset} (the stale-handle contract).
        if s.gate_mode == GateMode::Reset {
            assert!(
                s.claim.is_some(),
                "I13: gate RESET requires an active claim"
            );
            assert!(
                matches!(
                    s.episode.as_ref().map(|e| e.phase),
                    Some(Phase::Resetting) | Some(Phase::PostReset)
                ),
                "I13: gate RESET requires phase in {{Resetting, PostReset}}"
            );
        }
    }

    fn run(&mut self, cmd: &Cmd) {
        let at = self.tick();
        let ev = match cmd {
            Cmd::Open { optimistic } => {
                self.episode_seq += 1;
                SessionEvent::EpisodeOpen {
                    id: EpisodeId::new(format!("ep-{}", self.episode_seq)),
                    verification: if *optimistic {
                        ResetVerificationMode::OptimisticAsync
                    } else {
                        ResetVerificationMode::Blocking
                    },
                    born_claimed: false,
                    parent: None,
                    post_reset: false,
                    pre_window: None,
                    post_window: None,
                    agent_invite: None,
                    at,
                }
            }
            Cmd::OpenPostReset {
                optimistic,
                pre_window,
                post_window,
            } => {
                self.episode_seq += 1;
                SessionEvent::EpisodeOpen {
                    id: EpisodeId::new(format!("ep-{}", self.episode_seq)),
                    verification: if *optimistic {
                        ResetVerificationMode::OptimisticAsync
                    } else {
                        ResetVerificationMode::Blocking
                    },
                    born_claimed: false,
                    parent: None,
                    post_reset: true,
                    pre_window: if *pre_window {
                        Some(reset_window_spec())
                    } else {
                        None
                    },
                    post_window: if *post_window {
                        Some(reset_window_spec())
                    } else {
                        None
                    },
                    agent_invite: None,
                    at,
                }
            }
            Cmd::OpenAgent {
                optimistic,
                post_reset,
                post_window,
            } => {
                self.episode_seq += 1;
                SessionEvent::EpisodeOpen {
                    id: EpisodeId::new(format!("ep-{}", self.episode_seq)),
                    verification: if *optimistic {
                        ResetVerificationMode::OptimisticAsync
                    } else {
                        ResetVerificationMode::Blocking
                    },
                    born_claimed: false,
                    parent: None,
                    post_reset: *post_reset,
                    pre_window: None,
                    post_window: if *post_window {
                        Some(reset_window_spec())
                    } else {
                        None
                    },
                    agent_invite: Some(AgentInvite {
                        prompt: "proptest agent task".to_owned(),
                        // Inside the walk's `Advance` range so the invite
                        // timer fires under random exploration (like the
                        // window timer).
                        timeout_ns: 2_000_000_000,
                        task_metadata: Default::default(),
                    }),
                    at,
                }
            }
            Cmd::AgentDenied => SessionEvent::AgentTaskDenied {
                detail: "denied by plane".to_owned(),
                at,
            },
            Cmd::ResetOk { verified } => SessionEvent::ResetResult {
                ok: true,
                verified: *verified,
                at,
            },
            Cmd::ResetFail => SessionEvent::ResetResult {
                ok: false,
                verified: None,
                at,
            },
            Cmd::Verification {
                verified,
                invalidated,
            } => SessionEvent::VerificationResult {
                verified: *verified,
                invalidated_async: *invalidated,
                at,
            },
            Cmd::Start => SessionEvent::Start { at },
            Cmd::GateTick => SessionEvent::GateTick { at },
            Cmd::ClaimGranted { agent } => {
                self.claim_seq += 1;
                SessionEvent::ClaimGranted {
                    id: ClaimId::new(format!("claim-{}", self.claim_seq)),
                    source: if *agent { "agent" } else { "teleop" }.to_owned(),
                    actor: ActorRef::of_kind(if *agent {
                        ActorKind::Agent
                    } else {
                        ActorKind::Teleoperator
                    }),
                    self_initiated: false,
                    at,
                }
            }
            Cmd::Engage => {
                let claim = self
                    .state
                    .claim
                    .as_ref()
                    .map_or_else(|| ClaimId::new("missing"), |c| c.id.clone());
                SessionEvent::Engage { claim, at }
            }
            Cmd::HoldOk => SessionEvent::VerbResult {
                verb: Verb::Hold,
                ok: true,
                fault: None,
                at,
            },
            Cmd::ChunkBoundary => SessionEvent::ChunkBoundaryReached { at },
            Cmd::Release => {
                let claim = self
                    .state
                    .claim
                    .as_ref()
                    .map_or_else(|| ClaimId::new("missing"), |c| c.id.clone());
                SessionEvent::Release { claim, at }
            }
            Cmd::Retake { by_teleoperator } => {
                let claim = self
                    .state
                    .claim
                    .as_ref()
                    .map_or_else(|| ClaimId::new("missing"), |c| c.id.clone());
                self.episode_seq += 1;
                SessionEvent::Retake {
                    claim,
                    initiator: if *by_teleoperator {
                        ActorKind::Teleoperator
                    } else {
                        ActorKind::Agent
                    },
                    successor: EpisodeId::new(format!("ep-{}", self.episode_seq)),
                    at,
                }
            }
            Cmd::Clutch { engaged } => SessionEvent::Clutch {
                engaged: *engaged,
                at,
            },
            Cmd::Estop => SessionEvent::Estop { at },
            Cmd::Terminate { success } => SessionEvent::Terminate {
                outcome: if *success {
                    TerminalOutcome::Success
                } else {
                    TerminalOutcome::Failure
                },
                reason: "test".to_owned(),
                at,
            },
            Cmd::Mark { end } => SessionEvent::Mark {
                kind: if *end {
                    MarkKind::EndSuccess
                } else {
                    MarkKind::Start
                },
                at,
            },
            Cmd::PostResetOk => SessionEvent::PostResetResult {
                ok: true,
                detail: "cleanup ok".to_owned(),
                at,
            },
            Cmd::PostResetFail => SessionEvent::PostResetResult {
                ok: false,
                detail: "cleanup failed".to_owned(),
                at,
            },
            Cmd::WindowEngage => {
                let claim = self
                    .state
                    .claim
                    .as_ref()
                    .map_or_else(|| ClaimId::new("missing"), |c| c.id.clone());
                SessionEvent::ResetWindowEngage { claim, at }
            }
            Cmd::WindowComplete { ok } => {
                let claim = self
                    .state
                    .claim
                    .as_ref()
                    .map_or_else(|| ClaimId::new("missing"), |c| c.id.clone());
                SessionEvent::ResetWindowComplete {
                    claim,
                    ok: *ok,
                    verified: if *ok { Some(true) } else { None },
                    at,
                }
            }
            Cmd::Proxy { p95_ns } => SessionEvent::ProxySignals {
                sample: ProxySample {
                    gate_tick_p95_ns: *p95_ns,
                    ..Default::default()
                },
                at,
            },
            Cmd::PartitionStart => SessionEvent::PartitionStart { at },
            Cmd::PartitionEnd => SessionEvent::PartitionEnd { at },
            Cmd::DeferMints { defer } => {
                self.defer_mints = *defer;
                return;
            }
            Cmd::AnswerMint => {
                if self.outstanding_mints == 0 {
                    return;
                }
                self.outstanding_mints -= 1;
                self.lease_seq += 1;
                SessionEvent::LeaseTokenMinted {
                    minted: LeaseId::new(format!("L{}", self.lease_seq)),
                    at,
                }
            }
            Cmd::Advance { ns } => {
                self.now = self.now.saturating_add(*ns);
                // Fire due timers in deadline order.
                let mut due: Vec<(TimerId, MonoNs)> = self
                    .armed
                    .iter()
                    .filter(|(_, d)| *d <= self.now)
                    .copied()
                    .collect();
                due.sort_by_key(|(_, d)| *d);
                self.armed.retain(|(_, d)| *d > self.now);
                for (id, d) in due {
                    self.apply(SessionEvent::TimerFired { id, at: d });
                }
                return;
            }
        };
        self.apply(ev);

        // Invariant 4: estop always leaves the lease vacant.
        if matches!(cmd, Cmd::Estop) {
            assert_eq!(self.state.lease, LeaseState::Vacant);
        }

        // Invariant 8 (serializability).
        serde_json::to_value(&self.state).expect("state must serialize");
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn session_invariants_hold_under_random_walks(cmds in proptest::collection::vec(cmd_strategy(), 1..80)) {
        let mut driver = Driver::new();
        for cmd in &cmds {
            driver.run(cmd);
        }
    }
}

/// A deterministic end-to-end lifecycle: open → reset → run → claim →
/// engage(IMMEDIATE) → settle → retake(teleoperator) → born-claimed successor
/// → optimistic entry → late invalidation flag.
#[test]
fn retake_lifecycle_smoke() {
    let mut d = Driver::new();
    d.run(&Cmd::Open { optimistic: false });
    d.run(&Cmd::ResetOk {
        verified: Some(true),
    });
    d.run(&Cmd::Start);
    d.run(&Cmd::ClaimGranted { agent: false });
    d.run(&Cmd::Engage);
    let ep = d.state.episode.as_ref().unwrap();
    assert_eq!(ep.phase, Phase::Intervention(InterventionPhase::Settle));
    assert_eq!(d.state.gate_mode, GateMode::Intervention);

    d.run(&Cmd::Retake {
        by_teleoperator: true,
    });
    // The successor is open (driver interpreted OpenSuccessor), born claimed,
    // under the surviving claim.
    let ep = d.state.episode.as_ref().unwrap();
    assert!(ep.born_claimed);
    assert_eq!(ep.phase, Phase::Resetting);
    assert_eq!(ep.verification, ResetVerificationMode::OptimisticAsync);
    assert!(d.state.claim.is_some(), "claim survives retake");

    // Optimistic entry, then late invalidation.
    d.run(&Cmd::ResetOk { verified: None });
    assert_eq!(d.state.episode.as_ref().unwrap().phase, Phase::Ready);
    d.run(&Cmd::Verification {
        verified: false,
        invalidated: true,
    });
    assert!(d.state.episode.as_ref().unwrap().reset_unverified);
}

/// Blocking successor: an autonomous retake's successor cannot leave
/// RESETTING without a passing verification.
#[test]
fn autonomous_retake_blocks_on_verification() {
    let mut d = Driver::new();
    d.run(&Cmd::Open { optimistic: false });
    d.run(&Cmd::ResetOk {
        verified: Some(true),
    });
    d.run(&Cmd::Start);
    d.run(&Cmd::ClaimGranted { agent: false });
    d.run(&Cmd::Engage);
    d.run(&Cmd::Retake {
        by_teleoperator: false,
    });

    let ep = d.state.episode.as_ref().unwrap();
    assert_eq!(ep.verification, ResetVerificationMode::Blocking);

    d.run(&Cmd::ResetOk { verified: None }); // no verification yet
    assert_eq!(d.state.episode.as_ref().unwrap().phase, Phase::Resetting);
    d.run(&Cmd::Verification {
        verified: true,
        invalidated: false,
    });
    assert_eq!(d.state.episode.as_ref().unwrap().phase, Phase::Ready);
}

/// A deterministic end-to-end reset-phases lifecycle (flags
/// `waddle.v0.reset.phases` + `waddle.v0.reset.remote`): open(post declared,
/// post window remote) → reset ok → run → terminate{SUCCESS} → POST_RESET
/// with the window OPENED → claim granted (C6) → engage (gate → RESET, E20)
/// → complete{ok} → TERMINAL{SUCCESS}, asserting the E21 deferred-apply
/// emission order (handback precedes the pinned →TERMINAL transition).
#[test]
fn remote_post_reset_window_smoke() {
    let mut d = Driver::new();
    d.run(&Cmd::OpenPostReset {
        optimistic: false,
        pre_window: false,
        post_window: true,
    });
    d.run(&Cmd::ResetOk {
        verified: Some(true),
    });
    d.run(&Cmd::Start);
    d.run(&Cmd::Terminate { success: true });

    let ep = d.state.episode.as_ref().unwrap();
    assert_eq!(ep.phase, Phase::PostReset);
    assert!(ep.reset_window.is_some(), "post window opened at E14 (E19)");
    assert_eq!(ep.pinned_outcome, Some(TerminalOutcome::Success));

    d.run(&Cmd::ClaimGranted { agent: false });
    assert!(d.state.claim.is_some(), "reset claim admitted (C6)");
    d.run(&Cmd::WindowEngage);
    assert_eq!(
        d.state.gate_mode,
        GateMode::Reset,
        "gate → RESET on engage (E20)"
    );

    d.run(&Cmd::WindowComplete { ok: true });
    assert_eq!(
        d.state.episode.as_ref().unwrap().phase,
        Phase::Terminal(TerminalOutcome::Success)
    );
    assert!(d.state.claim.is_none(), "C7: reset claim released");
    assert_eq!(d.state.gate_mode, GateMode::Passthrough);

    // E21 emission order.
    let gate_reset = GateMode::Reset.to_pb() as i32;
    let gate_pass = GateMode::Passthrough.to_pb() as i32;
    let state_terminal = pb::EpisodeState::Terminal as i32;
    let outcome_success = TerminalOutcome::Success.to_pb() as i32;

    let engage_marker = format!("gate {gate_pass}->{gate_reset}");
    let handback_marker = format!("gate {gate_reset}->{gate_pass}");
    let terminal_marker = format!("state->{state_terminal} outcome={outcome_success}");

    let engage_idx = d
        .index_of(|s| s == engage_marker)
        .expect("gate → RESET on engage (E20)");
    let handback_idx = d
        .index_of(|s| s == handback_marker)
        .expect("gate RESET→PASSTHROUGH (deferred handback)");
    let terminal_idx = d
        .index_of(|s| s == terminal_marker)
        .expect("→TERMINAL{SUCCESS}");

    assert!(
        engage_idx < handback_idx,
        "engage precedes the later handback"
    );
    assert!(
        handback_idx < terminal_idx,
        "E21: the deferred handback precedes the pinned →TERMINAL transition"
    );
}

/// A deterministic agent-invited lifecycle (flag `waddle.v0.agent`): E23 open
/// (invite emitted, timer armed, E24 predicate on), C8 admission (teleop
/// rejected, agent admitted), E7 engage (timer cancelled, `agent_engaged`
/// latched), E26b inert DENIED, then an ordinary E10 termination.
#[test]
fn agent_invite_lifecycle_smoke() {
    let mut d = Driver::new();
    d.run(&Cmd::OpenAgent {
        optimistic: false,
        post_reset: false,
        post_window: false,
    });
    let ep = d.state.episode.as_ref().unwrap();
    assert!(ep.agent_invited, "E23 marks the episode agent-invited");
    assert!(
        d.armed
            .iter()
            .any(|(t, _)| *t == TimerId::AgentInviteTimeout),
        "E23 arms the invite timer"
    );
    assert!(
        d.state.agent_episode_noop(),
        "E24 predicate holds from open"
    );

    d.run(&Cmd::ResetOk {
        verified: Some(true),
    });
    d.run(&Cmd::Start);
    assert!(
        d.state.agent_episode_noop(),
        "E24: RUNNING with no engaged claim never dispatches"
    );

    // C8: a teleoperator grant is refused — no active claim, and the
    // refusal is RECORDED as `claim{DENIED}` (the plane already sent GRANT,
    // so the SDK's refusal goes on the timeline); the agent's is admitted.
    let claim_denied = pb::ClaimEventKind::Denied as i32;
    let denied_marker = format!("claim kind={claim_denied}");
    assert!(
        d.index_of(|s| s == denied_marker).is_none(),
        "no claim{{DENIED}} before the wrong-actor grant"
    );
    d.run(&Cmd::ClaimGranted { agent: false });
    assert!(d.state.claim.is_none(), "C8 rejects a non-AGENT grant");
    assert!(
        d.index_of(|s| s == denied_marker).is_some(),
        "C8 records the refusal as claim{{DENIED}}"
    );
    d.run(&Cmd::ClaimGranted { agent: true });
    assert!(d.state.claim.is_some(), "C8 admits the AGENT grant");

    d.run(&Cmd::Engage);
    let ep = d.state.episode.as_ref().unwrap();
    assert!(ep.agent_engaged, "E7 latches agent_engaged");
    assert!(
        !d.armed
            .iter()
            .any(|(t, _)| *t == TimerId::AgentInviteTimeout),
        "E7 cancels the invite timer"
    );
    assert!(
        !d.state.agent_episode_noop(),
        "an engaged claim gets ordinary intervention semantics"
    );

    // E26b: a late DENIED is inert.
    d.run(&Cmd::AgentDenied);
    assert!(matches!(
        d.state.episode.as_ref().unwrap().phase,
        Phase::Intervention(_)
    ));

    // Ordinary E10 termination (the agent's MARK_DONE path).
    d.run(&Cmd::Terminate { success: true });
    assert_eq!(
        d.state.episode.as_ref().unwrap().phase,
        Phase::Terminal(TerminalOutcome::Success)
    );
    assert!(d.state.claim.is_none(), "claim released at terminal");
    assert!(
        !d.state.agent_episode_noop(),
        "TERMINAL returns the caller's loop to passthrough"
    );
}

/// E25 both ways: undeclared → straight TERMINAL{ABORT}; with a declared
/// post-reset → the E14 detour with ABORT pinned (I17's second arm), then
/// TERMINAL{ABORT} after cleanup.
#[test]
fn agent_invite_timeout_aborts_or_detours_to_post_reset() {
    let mut d = Driver::new();
    d.run(&Cmd::OpenAgent {
        optimistic: false,
        post_reset: false,
        post_window: false,
    });
    d.run(&Cmd::ResetOk {
        verified: Some(true),
    });
    d.run(&Cmd::Start);
    d.run(&Cmd::Advance { ns: 3_000_000_000 });
    assert_eq!(
        d.state.episode.as_ref().unwrap().phase,
        Phase::Terminal(TerminalOutcome::Abort),
        "E25: open-invite timeout aborts"
    );

    let mut d = Driver::new();
    d.run(&Cmd::OpenAgent {
        optimistic: false,
        post_reset: true,
        post_window: false,
    });
    d.run(&Cmd::ResetOk {
        verified: Some(true),
    });
    d.run(&Cmd::Start);
    d.run(&Cmd::Advance { ns: 3_000_000_000 });
    let ep = d.state.episode.as_ref().unwrap();
    assert_eq!(ep.phase, Phase::PostReset, "E25 detours through E14");
    assert_eq!(ep.pinned_outcome, Some(TerminalOutcome::Abort));
    d.run(&Cmd::PostResetOk);
    assert_eq!(
        d.state.episode.as_ref().unwrap().phase,
        Phase::Terminal(TerminalOutcome::Abort)
    );
}

/// E26: a DENIED before any engage cancels the invite timer and terminates
/// with ABORT through the same routing as E25.
#[test]
fn agent_denied_before_engage_aborts() {
    let mut d = Driver::new();
    d.run(&Cmd::OpenAgent {
        optimistic: false,
        post_reset: false,
        post_window: false,
    });
    d.run(&Cmd::ResetOk {
        verified: Some(true),
    });
    d.run(&Cmd::Start);
    d.run(&Cmd::AgentDenied);
    assert_eq!(
        d.state.episode.as_ref().unwrap().phase,
        Phase::Terminal(TerminalOutcome::Abort),
        "E26: pre-engage DENIED aborts"
    );
    assert!(
        !d.armed
            .iter()
            .any(|(t, _)| *t == TimerId::AgentInviteTimeout),
        "E26 cancels the invite timer"
    );
}

/// C6 admission survives inside an agent-invited episode: the declared
/// teleop POST window still admits the teleoperator's reset claim in
/// POST_RESET (C8 constrains RUNNING-phase claims only).
#[test]
fn agent_episode_post_reset_window_still_admits_teleoperator() {
    let mut d = Driver::new();
    d.run(&Cmd::OpenAgent {
        optimistic: false,
        post_reset: true,
        post_window: true,
    });
    d.run(&Cmd::ResetOk {
        verified: Some(true),
    });
    d.run(&Cmd::Start);
    d.run(&Cmd::ClaimGranted { agent: true });
    d.run(&Cmd::Engage);
    d.run(&Cmd::Terminate { success: true });
    let ep = d.state.episode.as_ref().unwrap();
    assert_eq!(ep.phase, Phase::PostReset);
    assert!(ep.reset_window.is_some(), "post window opened at E14");

    // The teleoperator's reset claim is admitted per C6 even though the
    // episode is agent-invited.
    d.run(&Cmd::ClaimGranted { agent: false });
    assert!(
        d.state.claim.is_some(),
        "C6: teleop reset claim admitted in the agent-invited episode's POST_RESET"
    );
    d.run(&Cmd::WindowEngage);
    assert_eq!(d.state.gate_mode, GateMode::Reset);
    d.run(&Cmd::WindowComplete { ok: true });
    assert_eq!(
        d.state.episode.as_ref().unwrap().phase,
        Phase::Terminal(TerminalOutcome::Success),
        "pinned SUCCESS survives the remote post-reset window"
    );
}
