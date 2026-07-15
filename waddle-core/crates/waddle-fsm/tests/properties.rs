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

use std::collections::HashSet;

use proptest::prelude::*;
use waddle_fsm::{
    Effect, LeaseState, MarkKind, Phase, ProxySample, Rejected, SessionConfig, SessionEvent,
    SessionFsm, TimerId, step,
};
use waddle_types::{
    ActorKind, ClaimId, EpisodeId, GateMode, Grant, HandoffPolicy, InterventionPhase,
    LeaseEnforcement, LeaseId, MonoNs, ResetVerificationMode, TerminalOutcome, Verb,
};

/// The abstract command alphabet the random walk draws from.
#[derive(Debug, Clone)]
enum Cmd {
    Open { optimistic: bool },
    ResetOk { verified: Option<bool> },
    ResetFail,
    Verification { verified: bool, invalidated: bool },
    Start,
    ClaimGranted,
    Engage,
    HoldOk,
    ChunkBoundary,
    Release,
    Retake { by_teleoperator: bool },
    Clutch { engaged: bool },
    Estop,
    Terminate { success: bool },
    Mark { end: bool },
    Proxy { p95_ns: i64 },
    PartitionStart,
    PartitionEnd,
    Advance { ns: i64 },
}

fn cmd_strategy() -> impl Strategy<Value = Cmd> {
    prop_oneof![
        3 => any::<bool>().prop_map(|optimistic| Cmd::Open { optimistic }),
        3 => prop_oneof![
            Just(Cmd::ResetOk { verified: Some(true) }),
            Just(Cmd::ResetOk { verified: Some(false) }),
            Just(Cmd::ResetOk { verified: None }),
        ],
        1 => Just(Cmd::ResetFail),
        2 => (any::<bool>(), any::<bool>())
            .prop_map(|(verified, invalidated)| Cmd::Verification { verified, invalidated }),
        3 => Just(Cmd::Start),
        3 => Just(Cmd::ClaimGranted),
        3 => Just(Cmd::Engage),
        2 => Just(Cmd::HoldOk),
        2 => Just(Cmd::ChunkBoundary),
        2 => Just(Cmd::Release),
        2 => any::<bool>().prop_map(|by_teleoperator| Cmd::Retake { by_teleoperator }),
        2 => any::<bool>().prop_map(|engaged| Cmd::Clutch { engaged }),
        1 => Just(Cmd::Estop),
        1 => any::<bool>().prop_map(|success| Cmd::Terminate { success }),
        1 => any::<bool>().prop_map(|end| Cmd::Mark { end }),
        2 => (50_000_000i64..200_000_000).prop_map(|p95_ns| Cmd::Proxy { p95_ns }),
        1 => Just(Cmd::PartitionStart),
        1 => Just(Cmd::PartitionEnd),
        2 => (1_000_000i64..5_000_000_000).prop_map(|ns| Cmd::Advance { ns }),
    ]
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
        }
    }

    fn tick(&mut self) -> MonoNs {
        self.now = self.now.saturating_add(1_000_000);
        self.now
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

                // Check (and record) the committed state BEFORE follow-up
                // events run, so intermediate phases are observed.
                self.check_invariants(retake_expected);

                let mut successors = 0;
                let mut follow_ups: Vec<SessionEvent> = Vec::new();
                for effect in &s.effects {
                    match effect {
                        Effect::MintLeaseToken(_) => {
                            self.lease_seq += 1;
                            follow_ups.push(SessionEvent::LeaseTokenMinted {
                                minted: LeaseId::new(format!("L{}", self.lease_seq)),
                                at: self.tick(),
                            });
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

    fn check_invariants(&mut self, _retake: bool) {
        let s = &self.state;

        // Invariant 1: terminal is absorbing per episode identity.
        if let Some(ep) = &s.episode {
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
            }
            self.last_phase = Some((ep.id.clone(), ep.phase));

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
                    at,
                }
            }
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
            Cmd::ClaimGranted => {
                self.claim_seq += 1;
                SessionEvent::ClaimGranted {
                    id: ClaimId::new(format!("claim-{}", self.claim_seq)),
                    source: "teleop".to_owned(),
                    actor: ActorKind::Teleoperator,
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
            Cmd::Proxy { p95_ns } => SessionEvent::ProxySignals {
                sample: ProxySample {
                    gate_tick_p95_ns: *p95_ns,
                    ..Default::default()
                },
                at,
            },
            Cmd::PartitionStart => SessionEvent::PartitionStart { at },
            Cmd::PartitionEnd => SessionEvent::PartitionEnd { at },
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
    d.run(&Cmd::ClaimGranted);
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
    d.run(&Cmd::ClaimGranted);
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
