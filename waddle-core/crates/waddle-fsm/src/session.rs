//! The composed session machine: episode × claim × lease × grant health.
//! One active episode per session (N18). Every guard-table row in
//! `waddle-protocol/docs/FSM.md` is implemented here; row ids appear as
//! comments.

use waddle_types::{
    ActorKind, ClaimId, EpisodeStateKind, GateMode, GrantStatus, InterventionPhase, MonoNs,
    ResetVerificationMode, TerminalOutcome, Verb, pb::v0 as pb,
};

use crate::claim::ActiveClaim;
use crate::config::SessionConfig;
use crate::effect::{AfterLease, Effect, LeaseOpKind, PendingLeaseOp};
use crate::emit;
use crate::episode::{EpisodeState, Phase};
use crate::event::{MarkKind, SessionEvent, TimerId};
use crate::granthealth::{GrantHealthEntry, HealthEvent};
use crate::lease::{LeaseCmd, LeaseOutcome, LeaseState};

/// How far an engage has progressed (FSM.md §5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum EngageStage {
    /// HOLD_FIRST: waiting for the hold VerbResult.
    AwaitHoldOk,
    /// CHUNK_BOUNDARY: waiting for the executing chunk to finish (or the
    /// cap timer).
    AwaitChunkBoundary,
    /// Lease token minting/handoff in flight.
    AwaitLease,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct SessionFsm {
    pub episode: Option<EpisodeState>,
    pub claim: Option<ActiveClaim>,
    pub lease: LeaseState,
    /// Between a `MintLeaseToken` effect and its `LeaseTokenMinted` answer.
    pub pending_lease: Option<PendingLeaseOp>,
    pub gate_mode: GateMode,
    pub engage_stage: Option<EngageStage>,
    pub engage_timed_out: bool,
    pub plane_connected: bool,
    /// Events buffered for the control plane while partitioned (bounded by
    /// the runtime; the FSM tracks the count for observability).
    pub buffered_events: u32,
    pub grants: Vec<GrantHealthEntry>,
    clutch_seq: u32,
}

impl SessionFsm {
    #[must_use]
    pub fn new(cfg: &SessionConfig) -> Self {
        Self {
            episode: None,
            claim: None,
            lease: LeaseState::Vacant,
            pending_lease: None,
            gate_mode: GateMode::Passthrough,
            engage_stage: None,
            engage_timed_out: false,
            plane_connected: true,
            buffered_events: 0,
            grants: cfg
                .grants
                .iter()
                .map(GrantHealthEntry::from_grant)
                .collect(),
            clutch_seq: 0,
        }
    }
}

#[derive(Debug)]
pub struct Step {
    pub next: SessionFsm,
    pub effects: Vec<Effect>,
}

/// An expected rejection: the event is illegal in the current state. The
/// state is unchanged (E12: TERMINAL is absorbing).
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("rejected: {reason}")]
pub struct Rejected {
    pub reason: String,
}

fn rejected(reason: impl Into<String>) -> Rejected {
    Rejected {
        reason: reason.into(),
    }
}

struct Ctx<'c> {
    cfg: &'c SessionConfig,
    s: SessionFsm,
    effects: Vec<Effect>,
}

impl Ctx<'_> {
    fn emit(&mut self, ev: pb::EpisodeEvent) {
        if !self.s.plane_connected {
            self.s.buffered_events += 1;
        }
        self.effects.push(Effect::Emit(Box::new(ev)));
    }

    fn episode(&self) -> &EpisodeState {
        self.s.episode.as_ref().expect("guarded by caller")
    }

    fn episode_mut(&mut self) -> &mut EpisodeState {
        self.s.episode.as_mut().expect("guarded by caller")
    }

    fn set_gate(&mut self, at: MonoNs, to: GateMode, reason: &str) {
        if self.s.gate_mode == to {
            return;
        }
        let from = self.s.gate_mode;
        self.s.gate_mode = to;
        let ep = self.episode().id.clone();
        self.emit(emit::gate_mode_change(at, &ep, from, to, reason));
        self.effects.push(Effect::SetGateMode(to));
    }

    fn transition(
        &mut self,
        at: MonoNs,
        to: Phase,
        reason: &str,
        outcome: Option<TerminalOutcome>,
    ) {
        let from = self.episode().phase.kind();
        let to_kind = to.kind();
        self.episode_mut().phase = to;
        let ep = self.episode().id.clone();
        self.emit(emit::state_transition(
            at,
            &ep,
            Some(from),
            to_kind,
            reason,
            outcome,
        ));
    }

    /// Central run-closing block, shared by terminal entry (E5/E9/E10/E11)
    /// and post-reset entry (E14). Drops the gate to PASSTHROUGH, transitions
    /// to `to` (carrying `outcome`), releases the claim, cancels engage/window
    /// timers, clears engage state, and applies deferred demotions — in the
    /// exact order the terminal path has always emitted them (pinned by
    /// `tests/reset_refactor_golden.rs`). `release_claim` is false only for
    /// retake (row C5: the claim survives; the intervenor keeps the lease and
    /// the gate stays claimed for the hand reset).
    fn close_run(
        &mut self,
        at: MonoNs,
        to: Phase,
        reason: &str,
        outcome: Option<TerminalOutcome>,
        release_claim: bool,
    ) {
        if release_claim && self.s.gate_mode != GateMode::Passthrough {
            self.set_gate(at, GateMode::Passthrough, reason);
        }
        self.transition(at, to, reason, outcome);
        if release_claim && let Some(claim) = self.s.claim.take() {
            let ep = self.episode().id.clone();
            self.emit(emit::claim_event(
                at,
                &ep,
                pb::ClaimEventKind::Released,
                &claim,
                reason,
            ));
        }
        for id in [TimerId::EngageTimeout, TimerId::ChunkBoundaryCap] {
            self.effects.push(Effect::CancelTimer { id });
        }
        self.s.engage_stage = None;
        self.s.engage_timed_out = false;
        self.s.pending_lease = None;
        // The next planning decision happens at an episode boundary — apply
        // deferred, signal-driven demotions now (N11: never mid-lease).
        self.apply_pending_demotions(at);
    }

    /// Central terminal entry (rows E5/E9/E10/E11).
    fn enter_terminal(
        &mut self,
        at: MonoNs,
        outcome: TerminalOutcome,
        reason: &str,
        release_claim: bool,
    ) {
        self.close_run(
            at,
            Phase::Terminal(outcome),
            reason,
            Some(outcome),
            release_claim,
        );
    }

    /// The E10 trigger set (terminate / mark END_* / judge / timeout) routes
    /// here so it can detour to POST_RESET when the episode declares one. For
    /// undeclared episodes this is exactly `enter_terminal(.., true)`. Estop
    /// (E11) and pre-reset failure (E5) never route here — nothing ran.
    fn request_terminal(&mut self, at: MonoNs, outcome: TerminalOutcome, reason: &str) {
        self.enter_terminal(at, outcome, reason, true);
    }

    fn apply_pending_demotions(&mut self, at: MonoNs) {
        let mut changes = Vec::new();
        for entry in &mut self.s.grants {
            if let Some(reason) = entry.apply_pending() {
                changes.push((entry.verb, reason));
            }
        }
        for (verb, reason) in changes {
            let ep = self.s.episode.as_ref().map(|e| e.id.clone());
            self.emit(emit::grant_change(
                at,
                ep.as_ref(),
                verb,
                GrantStatus::Active,
                GrantStatus::Demoted,
                &reason,
                at,
            ));
        }
    }

    /// The handoff sub-protocol entry (FSM.md §5). Requires an active claim.
    fn begin_engage(&mut self, at: MonoNs) {
        let claim_id = self.s.claim.as_ref().expect("guarded").id.clone();
        self.transition(
            at,
            Phase::Intervention(InterventionPhase::Engage),
            "engage",
            None,
        );
        let ep = self.episode().id.clone();
        self.emit(emit::intervention(
            at,
            &ep,
            InterventionPhase::Engage,
            &claim_id,
        ));
        self.effects.push(Effect::ArmTimer {
            id: TimerId::EngageTimeout,
            deadline: at.saturating_add(self.cfg.engage_timeout_ns),
        });

        // Delta spaces refuse mid-chunk splice entry: IMMEDIATE degrades to
        // HOLD_FIRST (FSM.md §5).
        let policy = match self.cfg.handoff {
            waddle_types::HandoffPolicy::Immediate { .. } if self.cfg.space_contains_delta => {
                waddle_types::HandoffPolicy::HoldFirst
            }
            p => p,
        };

        match policy {
            waddle_types::HandoffPolicy::HoldFirst => {
                self.effects.push(Effect::RequestVerb(Verb::Hold));
                self.s.engage_stage = Some(EngageStage::AwaitHoldOk);
            }
            waddle_types::HandoffPolicy::Immediate { .. } => {
                self.mint_engage_handoff();
            }
            waddle_types::HandoffPolicy::ChunkBoundary { max_wait_ns } => {
                if max_wait_ns > 0 {
                    self.effects.push(Effect::ArmTimer {
                        id: TimerId::ChunkBoundaryCap,
                        deadline: at.saturating_add(max_wait_ns),
                    });
                }
                self.s.engage_stage = Some(EngageStage::AwaitChunkBoundary);
            }
        }
    }

    /// The soon-to-be-idle writer is already stopped; move the lease (row L6)
    /// via a runtime-minted fresh token.
    fn mint_engage_handoff(&mut self) {
        let to = self.s.claim.as_ref().expect("guarded").client();
        let op = match self.s.lease.holder() {
            Some((from, _)) => LeaseOpKind::Handoff {
                from: from.clone(),
                to,
            },
            None => LeaseOpKind::Acquire { client: to },
        };
        let pending = PendingLeaseOp {
            op,
            then: AfterLease::EngageComplete,
        };
        self.s.pending_lease = Some(pending.clone());
        self.effects.push(Effect::MintLeaseToken(pending));
        self.s.engage_stage = Some(EngageStage::AwaitLease);
    }

    fn begin_release(&mut self, at: MonoNs) {
        let claim_id = self.s.claim.as_ref().expect("guarded").id.clone();
        let ep = self.episode().id.clone();
        self.episode_mut().phase = Phase::Intervention(InterventionPhase::Release);
        self.emit(emit::intervention(
            at,
            &ep,
            InterventionPhase::Release,
            &claim_id,
        ));
        // Policy re-primed on fresh observations BEFORE pass resumes.
        self.effects.push(Effect::ReprimePolicy);
        let from = self
            .s
            .lease
            .holder()
            .map(|(l, _)| l.clone())
            .expect("intervenor holds the lease in SETTLE");
        let pending = PendingLeaseOp {
            op: LeaseOpKind::Handoff {
                from,
                to: self.cfg.loop_client.clone(),
            },
            then: AfterLease::ReleaseComplete,
        };
        self.s.pending_lease = Some(pending.clone());
        self.effects.push(Effect::MintLeaseToken(pending));
    }

    fn finish(self) -> Step {
        Step {
            next: self.s,
            effects: self.effects,
        }
    }
}

/// Retake initiator → successor verification mode (N12).
fn retake_mode(initiator: ActorKind) -> ResetVerificationMode {
    match initiator {
        ActorKind::Teleoperator | ActorKind::SiteOperator => ResetVerificationMode::OptimisticAsync,
        ActorKind::Agent | ActorKind::Policy | ActorKind::System | ActorKind::Custom => {
            ResetVerificationMode::Blocking
        }
    }
}

/// The pure session transition. Rejections leave the state unchanged.
#[allow(clippy::too_many_lines)]
pub fn step(
    cfg: &SessionConfig,
    state: &SessionFsm,
    event: &SessionEvent,
) -> Result<Step, Rejected> {
    let mut ctx = Ctx {
        cfg,
        s: state.clone(),
        effects: Vec::new(),
    };

    // Helper guards -------------------------------------------------------
    let phase = state.episode.as_ref().map(|e| e.phase);
    let active = matches!(
        phase,
        Some(p) if !p.is_terminal()
    );

    match event {
        // E1 ---------------------------------------------------------------
        SessionEvent::EpisodeOpen {
            id,
            verification,
            born_claimed,
            parent,
            at,
            ..
        } => {
            if active {
                return Err(rejected("one active episode per session (N18)"));
            }
            if *born_claimed && state.claim.is_none() {
                return Err(rejected("a born-claimed successor requires a held claim"));
            }
            ctx.s.episode = Some(EpisodeState::open(
                id.clone(),
                *verification,
                *born_claimed,
                parent.clone(),
            ));
            ctx.emit(emit::state_transition(
                *at,
                id,
                None,
                EpisodeStateKind::Resetting,
                "open",
                None,
            ));
            if ctx.s.lease == LeaseState::Vacant && ctx.s.pending_lease.is_none() {
                let pending = PendingLeaseOp {
                    op: LeaseOpKind::Acquire {
                        client: cfg.loop_client.clone(),
                    },
                    then: AfterLease::InitialAcquire,
                };
                ctx.s.pending_lease = Some(pending.clone());
                ctx.effects.push(Effect::MintLeaseToken(pending));
            }
        }

        // E2/E3/E4/E5 --------------------------------------------------------
        SessionEvent::ResetResult { ok, verified, at } => {
            if !matches!(phase, Some(Phase::Resetting)) {
                return Err(rejected("reset_result outside RESETTING"));
            }
            if !*ok {
                // E5: strategies exhausted / reset failed.
                ctx.emit(emit::fault(
                    *at,
                    Some(&ctx.episode().id.clone()),
                    pb::FaultKind::PreconditionFail,
                    "reset-pipeline",
                    "reset failed; strategies exhausted",
                ));
                ctx.enter_terminal(*at, TerminalOutcome::Abort, "reset failed", true);
                return Ok(ctx.finish());
            }
            ctx.episode_mut().reset_ok = true;
            let mode = ctx.episode().verification;
            if let Some(v) = verified {
                let ep = ctx.episode().id.clone();
                ctx.emit(emit::reset_verification(*at, &ep, mode, *v, false));
                if *v {
                    ctx.episode_mut().verified = true;
                }
            }
            match mode {
                ResetVerificationMode::Blocking => {
                    // E2 when verified; E4 (stay) otherwise.
                    if ctx.episode().verified {
                        ctx.transition(*at, Phase::Ready, "reset verified", None);
                    }
                }
                ResetVerificationMode::OptimisticAsync => {
                    // E3: enter READY; verification continues async.
                    ctx.episode_mut().optimistic_entry = !ctx.episode().verified;
                    ctx.transition(*at, Phase::Ready, "optimistic entry", None);
                }
            }
        }

        // E2 (late verification) / E13 --------------------------------------
        SessionEvent::VerificationResult {
            verified,
            invalidated_async,
            at,
        } => {
            let Some(ep_state) = &state.episode else {
                return Err(rejected("no episode"));
            };
            let mode = ep_state.verification;
            let ep = ep_state.id.clone();
            if *verified {
                ctx.emit(emit::reset_verification(*at, &ep, mode, true, false));
                ctx.episode_mut().verified = true;
                if matches!(phase, Some(Phase::Resetting))
                    && mode == ResetVerificationMode::Blocking
                    && ctx.episode().reset_ok
                {
                    ctx.transition(*at, Phase::Ready, "reset verified", None);
                }
            } else if ep_state.optimistic_entry || *invalidated_async {
                // E13: retroactive, PERMANENT flag — legal in any phase,
                // including TERMINAL (the flag is a record correction, not a
                // transition).
                ctx.emit(emit::reset_verification(*at, &ep, mode, false, true));
                ctx.episode_mut().reset_unverified = true;
                ctx.effects.push(Effect::SetResetUnverified { episode: ep });
            } else {
                // Blocking-mode failure: stay in RESETTING (E4).
                ctx.emit(emit::reset_verification(*at, &ep, mode, false, false));
            }
        }

        // E6 ---------------------------------------------------------------
        SessionEvent::Start { at } => {
            if !matches!(phase, Some(Phase::Ready)) {
                return Err(rejected("start outside READY"));
            }
            ctx.transition(*at, Phase::Running, "start", None);
        }
        SessionEvent::GateTick { at } => {
            // First gated action drives READY → RUNNING (E6); otherwise the
            // tick is the caller's business, not a transition.
            if matches!(phase, Some(Phase::Ready)) {
                ctx.transition(*at, Phase::Running, "first gated action", None);
            }
        }

        // C1/C2/C3 -----------------------------------------------------------
        SessionEvent::ClaimRequested {
            id,
            source,
            actor,
            self_initiated,
            at,
        } => {
            if !active {
                return Err(rejected("claim_request without an active episode"));
            }
            let claim = ActiveClaim {
                id: id.clone(),
                source: source.clone(),
                actor: *actor,
                self_initiated: *self_initiated,
            };
            let ep = ctx.episode().id.clone();
            ctx.emit(emit::claim_event(
                *at,
                &ep,
                pb::ClaimEventKind::Requested,
                &claim,
                "",
            ));
        }
        SessionEvent::ClaimGranted {
            id,
            source,
            actor,
            self_initiated,
            at,
        } => {
            if !active {
                return Err(rejected("claim_granted without an active episode"));
            }
            if state.claim.is_some() {
                return Err(rejected("conflicting active claim (one claim in v0)"));
            }
            let claim = ActiveClaim {
                id: id.clone(),
                source: source.clone(),
                actor: *actor,
                self_initiated: *self_initiated,
            };
            let ep = ctx.episode().id.clone();
            ctx.emit(emit::claim_event(
                *at,
                &ep,
                pb::ClaimEventKind::Granted,
                &claim,
                "",
            ));
            ctx.s.claim = Some(claim);
        }
        SessionEvent::ClaimReleased { id, at } => {
            let Some(claim) = &state.claim else {
                return Err(rejected("no active claim"));
            };
            if &claim.id != id {
                return Err(rejected("claim id mismatch"));
            }
            if matches!(phase, Some(Phase::Intervention(_))) {
                return Err(rejected(
                    "release the intervention (release/retake), not the claim",
                ));
            }
            let claim = ctx.s.claim.take().expect("checked");
            let ep = ctx.episode().id.clone();
            ctx.emit(emit::claim_event(
                *at,
                &ep,
                pb::ClaimEventKind::Released,
                &claim,
                "",
            ));
        }

        // E7 / I1 ------------------------------------------------------------
        SessionEvent::Engage { claim, at } => {
            if !matches!(phase, Some(Phase::Running)) {
                return Err(rejected("engage outside RUNNING (E7)"));
            }
            match &state.claim {
                Some(c) if &c.id == claim => {}
                Some(_) => return Err(rejected("engage for a claim that is not active")),
                None => return Err(rejected("engage without a granted claim")),
            }
            ctx.begin_engage(*at);
        }

        SessionEvent::ChunkBoundaryReached { at } => {
            if state.engage_stage == Some(EngageStage::AwaitChunkBoundary) {
                ctx.effects.push(Effect::CancelTimer {
                    id: TimerId::ChunkBoundaryCap,
                });
                ctx.mint_engage_handoff();
                let _ = at;
            }
        }

        // E8 / I3 ------------------------------------------------------------
        SessionEvent::Release { claim, at } => {
            if !matches!(phase, Some(Phase::Intervention(InterventionPhase::Settle))) {
                return Err(rejected("release outside SETTLE (E8)"));
            }
            match &state.claim {
                Some(c) if &c.id == claim => {}
                _ => return Err(rejected("release for a claim that is not active")),
            }
            ctx.begin_release(*at);
        }

        // E9 / I4 / C5 --------------------------------------------------------
        SessionEvent::Retake {
            claim,
            initiator,
            successor,
            at,
        } => {
            let legal = matches!(phase, Some(Phase::Intervention(InterventionPhase::Settle)))
                || (matches!(phase, Some(Phase::Intervention(InterventionPhase::Engage)))
                    && state.engage_timed_out);
            if !legal {
                return Err(rejected(
                    "retake is legal from SETTLE (or ENGAGE after the settle timeout)",
                ));
            }
            match &state.claim {
                Some(c) if &c.id == claim => {}
                _ => return Err(rejected("retake for a claim that is not active")),
            }
            let ep = ctx.episode().id.clone();
            ctx.emit(emit::intervention(
                *at,
                &ep,
                InterventionPhase::Retake,
                claim,
            ));
            // The claim SURVIVES (C5); the intervenor keeps the lease and the
            // gate stays claimed for the hand reset.
            ctx.enter_terminal(*at, TerminalOutcome::AbortedRetake, "retake", false);
            ctx.effects.push(Effect::OpenSuccessor {
                predecessor: ep,
                successor: successor.clone(),
                claim: claim.clone(),
                born_claimed: true,
                mode: retake_mode(*initiator),
            });
        }

        // C-section: engagement-initiated claims ------------------------------
        SessionEvent::Clutch { engaged, at } => {
            if *engaged {
                if state.claim.is_some() || !matches!(phase, Some(Phase::Running)) {
                    // Already claimed or not in a claimable phase: the edge
                    // is recorded by the source, not a transition.
                } else {
                    ctx.s.clutch_seq += 1;
                    let claim = ActiveClaim {
                        id: ClaimId::new(format!("clutch-{}", ctx.s.clutch_seq)),
                        source: cfg.clutch_source.clone(),
                        actor: cfg.clutch_actor,
                        self_initiated: true,
                    };
                    let ep = ctx.episode().id.clone();
                    // Requested and granted in one step: the engaged clutch
                    // IS the authorization (never the envelope).
                    ctx.emit(emit::claim_event(
                        *at,
                        &ep,
                        pb::ClaimEventKind::Requested,
                        &claim,
                        "self-initiated (clutch)",
                    ));
                    ctx.emit(emit::claim_event(
                        *at,
                        &ep,
                        pb::ClaimEventKind::Granted,
                        &claim,
                        "self-initiated (clutch)",
                    ));
                    ctx.s.claim = Some(claim);
                    ctx.begin_engage(*at);
                }
            } else if matches!(phase, Some(Phase::Intervention(InterventionPhase::Settle)))
                && state.claim.as_ref().is_some_and(|c| c.self_initiated)
            {
                ctx.begin_release(*at);
            }
        }

        SessionEvent::VerbResult {
            verb,
            ok,
            fault,
            at,
        } => {
            if state.engage_stage == Some(EngageStage::AwaitHoldOk) && *verb == Verb::Hold {
                if *ok {
                    ctx.mint_engage_handoff();
                } else {
                    // Fail-closed: both writers stay stopped; engage remains
                    // incomplete (retake becomes legal after the timeout).
                    let ep = ctx.episode().id.clone();
                    ctx.emit(emit::fault(
                        *at,
                        Some(&ep),
                        fault.unwrap_or(pb::FaultKind::AdapterError),
                        "engage",
                        "hold failed during HOLD_FIRST engage; writers remain stopped",
                    ));
                }
            } else if !*ok && *fault == Some(pb::FaultKind::LeaseDenied) {
                // A pause signal, never a fault-equivalent abort (FSM.md §3).
                let ep = state.episode.as_ref().map(|e| e.id.clone());
                ctx.emit(emit::fault(
                    *at,
                    ep.as_ref(),
                    pb::FaultKind::LeaseDenied,
                    "verb",
                    "send from non-holder; pause signal",
                ));
            }
        }

        // L1/L6 completions ---------------------------------------------------
        SessionEvent::LeaseTokenMinted { minted, at } => {
            let Some(pending) = state.pending_lease.clone() else {
                return Err(rejected("no pending lease operation"));
            };
            ctx.s.pending_lease = None;
            let cmd = match &pending.op {
                LeaseOpKind::Acquire { client } => LeaseCmd::Acquire {
                    client: client.clone(),
                    minted: minted.clone(),
                },
                LeaseOpKind::Handoff { from, to } => LeaseCmd::Handoff {
                    from: from.clone(),
                    to: to.clone(),
                    minted: minted.clone(),
                },
            };
            let (next_lease, outcome) = crate::lease::step(&state.lease, &cmd);
            ctx.s.lease = next_lease;
            let ep = state.episode.as_ref().map(|e| e.id.clone());
            match outcome {
                LeaseOutcome::Granted { lease, client, .. } => {
                    ctx.emit(emit::lease_event(
                        *at,
                        ep.as_ref(),
                        pb::LeaseEventKind::Acquired,
                        Some((&lease, &client)),
                        cfg.enforcement,
                        "",
                    ));
                }
                LeaseOutcome::HandedOff { new, to, old } => {
                    ctx.emit(emit::lease_event(
                        *at,
                        ep.as_ref(),
                        pb::LeaseEventKind::HandedOff,
                        Some((&new, &to)),
                        cfg.enforcement,
                        &format!("from {old}"),
                    ));
                }
                LeaseOutcome::Denied { detail } => {
                    ctx.emit(emit::lease_event(
                        *at,
                        ep.as_ref(),
                        pb::LeaseEventKind::Denied,
                        None,
                        cfg.enforcement,
                        detail,
                    ));
                    // Fail-closed: nothing else proceeds.
                    return Ok(ctx.finish());
                }
                LeaseOutcome::Released { .. } | LeaseOutcome::RevokedAll { .. } => {
                    unreachable!("not produced by acquire/handoff")
                }
            }
            match pending.then {
                AfterLease::InitialAcquire => {}
                AfterLease::EngageComplete => {
                    ctx.s.engage_stage = None;
                    ctx.s.engage_timed_out = false;
                    ctx.effects.push(Effect::CancelTimer {
                        id: TimerId::EngageTimeout,
                    });
                    let claim_id = ctx.s.claim.as_ref().expect("claim held").id.clone();
                    let ep = ctx.episode().id.clone();
                    ctx.episode_mut().phase = Phase::Intervention(InterventionPhase::Settle);
                    ctx.emit(emit::intervention(
                        *at,
                        &ep,
                        InterventionPhase::Settle,
                        &claim_id,
                    ));
                    ctx.set_gate(*at, GateMode::Intervention, "engage");
                }
                AfterLease::ReleaseComplete => {
                    ctx.set_gate(*at, GateMode::Passthrough, "release");
                    ctx.transition(*at, Phase::Running, "release", None);
                    if let Some(claim) = ctx.s.claim.take() {
                        let ep = ctx.episode().id.clone();
                        ctx.emit(emit::claim_event(
                            *at,
                            &ep,
                            pb::ClaimEventKind::Released,
                            &claim,
                            "release",
                        ));
                    }
                }
                // reset-remote lease completions (E20/E21/E22): implemented in
                // a later commit; no pending op targets them yet.
                AfterLease::ResetEngageComplete | AfterLease::ResetHandback { .. } => {}
            }
        }

        // E11 / L8 ------------------------------------------------------------
        SessionEvent::Estop { at } => {
            let (next_lease, outcome) = crate::lease::step(&state.lease, &LeaseCmd::RevokeAll);
            ctx.s.lease = next_lease;
            let ep = state.episode.as_ref().map(|e| e.id.clone());
            if let LeaseOutcome::RevokedAll { was } = outcome {
                ctx.emit(emit::lease_event(
                    *at,
                    ep.as_ref(),
                    pb::LeaseEventKind::RevokedAll,
                    None,
                    cfg.enforcement,
                    &was.map(|l| format!("revoked {l}")).unwrap_or_default(),
                ));
            }
            ctx.emit(emit::fault(
                *at,
                ep.as_ref(),
                pb::FaultKind::Estop,
                "estop",
                "emergency stop",
            ));
            ctx.effects.push(Effect::RequestVerb(Verb::Estop));
            ctx.s.pending_lease = None;
            if active {
                ctx.enter_terminal(*at, TerminalOutcome::Abort, "estop", true);
            }
        }

        // E10 -----------------------------------------------------------------
        SessionEvent::Terminate {
            outcome,
            reason,
            at,
        } => {
            if !active {
                return Err(rejected("terminate without an active episode (E12)"));
            }
            ctx.request_terminal(*at, *outcome, reason);
        }

        // Async judging attaches labels even after TERMINAL (E10 note); it is
        // an attachment, never a transition.
        SessionEvent::JudgeResult {
            judge_id,
            passed,
            at,
        } => {
            let Some(ep_state) = &state.episode else {
                return Err(rejected("no episode"));
            };
            let ep = ep_state.id.clone();
            ctx.emit(emit::judgment(*at, &ep, judge_id, *passed));
        }

        SessionEvent::Mark { kind, at } => {
            if !active {
                return Err(rejected("mark without an active episode (E12)"));
            }
            let ep = ctx.episode().id.clone();
            ctx.emit(emit::mark(*at, &ep, kind.to_pb()));
            if let Some(outcome) = kind.terminal_outcome() {
                ctx.request_terminal(*at, outcome, "mark");
            }
            let _ = MarkKind::Start; // exhaustiveness documented in event.rs
        }

        // §7 grant liveness ----------------------------------------------------
        SessionEvent::ProxySignals { sample, at } => {
            let mut repromoted = Vec::new();
            for entry in &mut ctx.s.grants {
                if entry.observe(
                    sample,
                    cfg.demote_after,
                    cfg.promote_after,
                    cfg.hysteresis_ratio,
                ) == Some(HealthEvent::Repromoted)
                {
                    repromoted.push(entry.verb);
                }
            }
            let ep = state.episode.as_ref().map(|e| e.id.clone());
            for verb in repromoted {
                ctx.emit(emit::grant_change(
                    *at,
                    ep.as_ref(),
                    verb,
                    GrantStatus::Demoted,
                    GrantStatus::Active,
                    "sustained recovery below hysteresis band",
                    *at,
                ));
            }
            // Signal-driven demotions apply only at a planning boundary.
            if !active {
                ctx.apply_pending_demotions(*at);
            }
        }

        SessionEvent::HeartbeatAck { grant_changes, at } => {
            for change in grant_changes {
                let ep = state.episode.as_ref().map(|e| e.id.clone());
                if let Some(entry) = ctx.s.grants.iter_mut().find(|g| g.verb == change.verb) {
                    let from = entry.status;
                    entry.status = change.to;
                    entry.partition_demoted = false;
                    entry.pending_demote = None;
                    ctx.emit(emit::grant_change(
                        *at,
                        ep.as_ref(),
                        change.verb,
                        from,
                        change.to,
                        &change.reason,
                        *at,
                    ));
                }
            }
        }

        // §8 degraded operation -------------------------------------------------
        SessionEvent::PartitionStart { at } => {
            ctx.s.plane_connected = false;
            ctx.effects.push(Effect::ArmTimer {
                id: TimerId::HeartbeatStale,
                deadline: at.saturating_add(cfg.heartbeat_timeout_ns),
            });
        }
        SessionEvent::PartitionEnd { at } => {
            ctx.s.plane_connected = true;
            ctx.s.buffered_events = 0;
            ctx.effects.push(Effect::CancelTimer {
                id: TimerId::HeartbeatStale,
            });
            let ep = state.episode.as_ref().map(|e| e.id.clone());
            let mut restored = Vec::new();
            for entry in &mut ctx.s.grants {
                if entry.partition_demoted {
                    entry.partition_demoted = false;
                    entry.status = GrantStatus::Active;
                    restored.push(entry.verb);
                }
            }
            for verb in restored {
                ctx.emit(emit::grant_change(
                    *at,
                    ep.as_ref(),
                    verb,
                    GrantStatus::Demoted,
                    GrantStatus::Active,
                    "control plane reconnected",
                    *at,
                ));
            }
        }

        SessionEvent::TimerFired { id, at } => match id {
            TimerId::EngageTimeout => {
                if matches!(phase, Some(Phase::Intervention(InterventionPhase::Engage))) {
                    ctx.s.engage_timed_out = true;
                }
            }
            TimerId::ChunkBoundaryCap => {
                if state.engage_stage == Some(EngageStage::AwaitChunkBoundary) {
                    ctx.mint_engage_handoff();
                }
            }
            TimerId::HeartbeatStale => {
                if !state.plane_connected {
                    let ep = state.episode.as_ref().map(|e| e.id.clone());
                    ctx.emit(emit::tripwire(
                        *at,
                        ep.as_ref(),
                        "control-plane-heartbeat",
                        Verb::Hold,
                        "heartbeat stale during partition; requesting hold",
                    ));
                    ctx.effects.push(Effect::RequestVerb(Verb::Hold));
                    // Cloud-dependent grants degrade immediately: the plane
                    // is gone, the grant is unusable — not merely slow.
                    let mut demoted = Vec::new();
                    for entry in &mut ctx.s.grants {
                        if entry.verb == Verb::Send && entry.status == GrantStatus::Active {
                            entry.status = GrantStatus::Demoted;
                            entry.partition_demoted = true;
                            demoted.push(entry.verb);
                        }
                    }
                    let ep = state.episode.as_ref().map(|e| e.id.clone());
                    for verb in demoted {
                        ctx.emit(emit::grant_change(
                            *at,
                            ep.as_ref(),
                            verb,
                            GrantStatus::Active,
                            GrantStatus::Demoted,
                            "partition",
                            *at,
                        ));
                    }
                }
            }
            // reset-remote window deadline (E22): implemented in a later
            // commit; no window arms this timer yet.
            TimerId::ResetWindowTimeout => {}
        },

        // §6 gate-mode rows: INTERVENTION ⇄ BYPASS -----------------------------
        SessionEvent::StallDetected { at } => {
            if state.gate_mode == GateMode::Intervention
                && state.claim.is_some()
                && matches!(phase, Some(Phase::Intervention(_)))
            {
                ctx.set_gate(*at, GateMode::Bypass, "caller loop stalled while claimed");
            }
        }
        SessionEvent::TicksResumed { at } => {
            if state.gate_mode == GateMode::Bypass {
                ctx.set_gate(*at, GateMode::Intervention, "caller ticks resumed");
            }
        }

        // N14 ------------------------------------------------------------------
        SessionEvent::DualWrite {
            divergence_metric,
            window_ns,
            trace_ref,
            at,
        } => {
            if !active {
                return Err(rejected("dual-write detection without an active episode"));
            }
            let ep = ctx.episode().id.clone();
            ctx.emit(emit::dual_write(
                *at,
                &ep,
                *divergence_metric,
                *window_ns,
                trace_ref,
                Verb::Hold,
            ));
            ctx.effects.push(Effect::RequestVerb(Verb::Hold));
        }

        // Bug 2 (action-space validation) --------------------------------
        SessionEvent::InterventionRejected {
            dims_got,
            dims_want,
            at,
        } => {
            if !active {
                return Err(rejected("intervention_rejected without an active episode"));
            }
            let ep = ctx.episode().id.clone();
            ctx.emit(emit::fault(
                *at,
                Some(&ep),
                pb::FaultKind::ValidationError,
                "media-intake",
                &format!(
                    "teleop action carried {dims_got} dims; declared action \
                     space wants {dims_want}"
                ),
            ));
        }

        // reset-phases (flags waddle.v0.reset.phases / .remote): inert until
        // the post-reset phase and remote windows land in later commits.
        SessionEvent::PostResetResult { .. } => {
            return Err(rejected("post_reset_result not yet implemented"));
        }
        SessionEvent::ResetWindowEngage { .. } => {
            return Err(rejected("reset_window_engage not yet implemented"));
        }
        SessionEvent::ResetWindowComplete { .. } => {
            return Err(rejected("reset_window_complete not yet implemented"));
        }
    }

    Ok(ctx.finish())
}
