//! The scenario targets: `fsm` (the pure session machine, effects observable
//! as emissions) and `gate` (the FSM composed with a real `waddle_gate::Gate`
//! plus the scripted caller loop / intervention stream / bypass pump).
//!
//! Determinism contract: lease tokens mint as `lease-1`, `lease-2`, … in
//! effect order; `Effect::MintLeaseToken` and `Effect::OpenSuccessor` are
//! answered by immediately injecting their completion events; virtual time
//! advances in fixed steps so stall detection, chunk boundaries, jitter
//! playout and the bypass pump behave identically on every run.

use std::collections::BTreeMap;
use std::sync::Arc;

use serde_json::{Map, Value, json};
use waddle_fsm::effect::Effect;
use waddle_fsm::session::EngageStage;
use waddle_fsm::{
    AgentInvite, GrantChangeDirective, MarkKind, ProxySample, SessionConfig, SessionEvent,
    SessionFsm, TimerId, WindowSpec,
};
use waddle_gate::gate::GateShared;
use waddle_gate::{
    BlendSchedule, ChunkMeta, DivergenceDetector, Gate, GateOutput, GatePlan, GateRecord,
    OwnedAction, PlanMode, StreamChannel, TimedAction,
};
use waddle_ingest::FakeClock;
use waddle_types::action::ActionValues;
use waddle_types::{
    ActionChunk, ActionSpace, ActorKind, ActorRef, ClaimId, EpisodeId, GateMode, GrantStatus,
    Interp, LeaseEnforcement, LeaseId, MonoNs, PartPolicy, ProvenanceTag, ReplanPolicy, SpaceSpec,
    TerminalOutcome, TypesError, Verb, pb::v0 as pb,
};

use crate::emissions::{
    Codec, EmissionEntry, effect_to_value, gate_mode_name, grant_status_name,
    intervention_phase_name, verb_name,
};
use crate::scenario::{Scenario, TargetKind, parse_ns, parse_verification_mode};
use crate::{ConformanceError, scenario_err};

/// No `gate_tick` within this window while claimed ⇒ the caller loop has
/// stalled (conformance timing envelope; FSM.md §5 bypass mode).
pub const STALL_THRESHOLD_NS: i64 = 500_000_000;
/// Granularity of virtual-time stepping for periodic work (pump, stall and
/// chunk-boundary detection). All fixture timestamps are multiples of this.
const PERIODIC_STEP_NS: i64 = 10_000_000;
/// Dual-write detection parameters (N14) used by the gate target.
const DIVERGENCE_THRESHOLD: f64 = 0.1;
const DIVERGENCE_WINDOW_NS: i64 = 150_000_000;

const STREAM_CAPACITY: usize = 256;
const RECORD_CAPACITY: usize = 1024;
/// Deterministic playout: an intervention action is due the instant it
/// arrives (virtual time already models network delay).
const PLAYOUT_DELAY_NS: i64 = 0;
/// The intake an `intervention_chunk` refusal is attributed to
/// (`Fault.source`). Scenarios assert it as `$nonempty`, never as this
/// literal — `Fault.source` is implementation-named (control.proto: "who
/// raised it"), so this spelling binds nobody but the reference runner.
const AGENT_CHUNK_SOURCE: &str = "agent-chunk";

/// A once-per-claim-window latch, keyed to the window's IDENTITY rather
/// than to a holder noticing the window shut: two windows meeting inside a
/// gap the holder never polls (a retake hands the window over with no gap
/// at all) must not share one refusal. `raise` answers true the first time
/// it is asked within one window, false for every later ask in the same
/// one.
#[derive(Debug, Default)]
struct WindowLatch {
    generation: u64,
    raised: bool,
}

impl WindowLatch {
    fn raise(&mut self, generation: u64) -> bool {
        if self.generation != generation {
            self.generation = generation;
            self.raised = false;
        }
        !std::mem::replace(&mut self.raised, true)
    }
}

/// The `intervention_chunk` intake's bookkeeping: its own ring-seq counter
/// and one [`WindowLatch`] per refusal reason. The reasons are independent
/// — a chunk that doesn't fit the declared space, one whose width disagrees
/// with it, and one whose steps carry nothing to dispatch each say
/// something different, and a sender is owed each of them once per window.
#[derive(Debug, Default)]
struct ChunkIntake {
    /// Per-STEP seq for the jitter buffer's reorder map, never the chunk's
    /// own `seq` (one value for the whole chunk — every step in it would
    /// collide on the same key).
    next_seq: u64,
    dims: WindowLatch,
    not_executable: WindowLatch,
    inert: WindowLatch,
}

/// The commanded side of dual-write detection (N14), keyed by the SCOPE each
/// command addressed: `""` for a whole-robot action, the part's name for a
/// part-addressed one.
///
/// One vector stopped being enough when `Action.part` became executable. A
/// part-scoped command means "move this part, hold the rest" (docs/FSM.md
/// §4), so what the robot was last told is one command PER PART, and a
/// part-width vector compared against whatever proprioception arrives next
/// scores an intervenor against another part's joints. Hence the two rules
/// here: a whole-robot command commands every part, so it CLEARS the
/// part-scoped commands it supersedes; a part-scoped one replaces its own
/// part only, leaving the last whole-robot command standing as the anchor
/// for the parts it did not address.
#[derive(Debug, Default)]
struct Commanded {
    by_scope: BTreeMap<String, Vec<f64>>,
}

impl Commanded {
    /// Record what just went robot-ward. `part` is the action's own tag
    /// ([`OwnedAction::part`]); `None` is the whole declared space.
    fn record(&mut self, part: Option<&str>, values: &[f64]) {
        if part.is_none() {
            self.by_scope.clear();
        }
        self.by_scope
            .insert(part.unwrap_or_default().to_owned(), values.to_vec());
    }

    /// What a `ProprioSample` for `part` may be compared against, or `None`
    /// when the two describe different scopes and nothing honest can be said
    /// about the pair.
    ///
    /// * A per-part sample answers to that part's own command when one
    ///   stands, and otherwise to the last whole-robot command's slice for
    ///   it: slicing an ACTION vector by declaration order is what the
    ///   declared composite layout means, and "hold the rest" leaves that
    ///   slice as the part's standing command.
    /// * A whole-robot sample (`part == ""`) answers to the last whole-robot
    ///   command, but only while nothing narrower supersedes part of it.
    ///   Past that, the whole-robot expectation is a composition of one
    ///   command per part, and the sample's layout is the customer's
    ///   OBSERVATION layout, which no declaration describes — so the runner
    ///   never re-lays an observation out by action parts, and says nothing
    ///   instead. A part-addressed session reports proprioception per part
    ///   (`ProprioSample.part`), which is the case above.
    fn expected_for<'a>(&'a self, space: Option<&ActionSpace>, part: &str) -> Option<&'a [f64]> {
        if part.is_empty() {
            if self.by_scope.keys().any(|scope| !scope.is_empty()) {
                return None;
            }
            return self.by_scope.get("").map(Vec::as_slice);
        }
        if let Some(values) = self.by_scope.get(part) {
            return Some(values);
        }
        let whole = self.by_scope.get("")?;
        let (offset, dims) = part_span(space?, part)?;
        let end = (offset + dims).min(whole.len());
        (offset < end).then(|| &whole[offset..end])
    }
}

#[derive(Debug)]
struct GateParts {
    shared: Arc<GateShared>,
    gate: Gate<FakeClock>,
    clock: FakeClock,
    producer: rtrb::Producer<TimedAction>,
    /// Kept alive so gate records are droppable without erroring the ring.
    _records: rtrb::Consumer<GateRecord>,
    space: Option<ActionSpace>,
    interp: Interp,
    last_output: Option<GateOutput>,
    /// The Noop reason of the most recent tick, captured from the plan mode
    /// that produced it (`GateOutput::Noop` deliberately carries no reason —
    /// the reducer's marker translation owns it; here the harness plays
    /// that role). `None` when the last output was not a Noop.
    last_noop_reason: Option<pb::NoopReason>,
    last_tick_ns: Option<i64>,
    /// An intervention stream has produced traffic (stall → bypass only
    /// matters when there is someone to starve).
    traffic: bool,
    /// Dims-validation contract (mirroring the media intake's own guard): a
    /// dims-mismatched teleop injection faults at most once per claim
    /// window, not once per packet.
    teleop_dims_fault: WindowLatch,
    /// The `intervention_chunk` intake's seq counter and refusal latches.
    chunk_intake: ChunkIntake,
    /// End of the executing policy chunk (chunk-boundary detection).
    chunk_end_ns: Option<i64>,
    /// One divergence run per proprioception stream, keyed by
    /// `ProprioSample.part` (`""` = the whole robot). Parts are independent
    /// streams of content: a part that arrives where it was told must never
    /// reset — and so mask — the run of a part that is being driven by
    /// someone else.
    detectors: BTreeMap<String, DivergenceDetector>,
    /// What was last dispatched robot-ward (gate pass/substitute/blend or a
    /// bypass-pump send), per addressed scope — the "commanded" side of
    /// dual-write detection.
    commanded: Commanded,
    incident_seq: u32,
}

/// One scenario run's state: the FSM, virtual time, timers, and (for the
/// gate target) the composed gate harness.
#[derive(Debug)]
pub struct Target {
    codec: Codec,
    pub cfg: SessionConfig,
    default_verification: waddle_types::ResetVerificationMode,
    pub fsm: SessionFsm,
    /// Virtual time (ns).
    pub now: i64,
    timers: Vec<(TimerId, i64)>,
    pub emissions: Vec<EmissionEntry>,
    /// Bypass-pump sends (`expect_send`).
    pub send_log: Vec<EmissionEntry>,
    /// Rejected injections (state unchanged; no emissions — E12 et al).
    pub rejections: Vec<String>,
    lease_seq: u32,
    claim_seq: u32,
    /// The claim the current `claim_generation` describes.
    claim_window: Option<ClaimId>,
    /// The claim WINDOW's identity as a counter: it changes whenever the
    /// active claim changes — one opening, one ending, and a retake handing
    /// the window to a different claimant all count. Refreshed after every
    /// accepted FSM step, so no claim change can slip past an intake that
    /// only runs when something is injected. It is what every
    /// "at most once per claim window" refusal is keyed to
    /// ([`WindowLatch`]).
    claim_generation: u64,
    /// The single in-flight verb request (`verb_result` correlates to it).
    pending_verb: Option<Verb>,
    gate: Option<GateParts>,
}

impl Target {
    pub fn new(scenario: &Scenario, codec: Codec) -> Result<Self, ConformanceError> {
        let mut cfg = SessionConfig::minimal(
            "customer-loop",
            scenario.setup.handoff,
            scenario.setup.enforcement,
        );
        if let Some(robot) = &scenario.setup.robot {
            cfg.grants = robot.grants.clone();
            cfg.space_contains_delta = robot.action_space.contains_delta();
            cfg.robot_id = robot.robot_id.clone();
            cfg.cell_id = robot.cell_id.clone();
        }
        let fsm = SessionFsm::new(&cfg);
        let gate = match scenario.target {
            TargetKind::Fsm => None,
            TargetKind::Gate => {
                let clock = FakeClock::default();
                let space = scenario
                    .setup
                    .robot
                    .as_ref()
                    .map(|r| r.action_space.clone());
                let interp = space.as_ref().map_or(Interp::Linear, |s| s.chunking.interp);
                let replan = space
                    .as_ref()
                    .map_or(ReplanPolicy::Immediate, |s| s.chunking.replan);
                let (shared, producer) = GateShared::new(
                    GatePlan::passthrough(MonoNs(0)),
                    STREAM_CAPACITY,
                    PLAYOUT_DELAY_NS,
                    replan,
                );
                let (gate, records) =
                    Gate::new(Arc::clone(&shared), clock.clone(), RECORD_CAPACITY);
                Some(GateParts {
                    shared,
                    gate,
                    clock,
                    producer,
                    _records: records,
                    space,
                    interp,
                    last_output: None,
                    last_noop_reason: None,
                    last_tick_ns: None,
                    traffic: false,
                    teleop_dims_fault: WindowLatch::default(),
                    chunk_intake: ChunkIntake::default(),
                    chunk_end_ns: None,
                    detectors: BTreeMap::new(),
                    commanded: Commanded::default(),
                    incident_seq: 0,
                })
            }
        };
        Ok(Self {
            codec,
            cfg,
            default_verification: scenario.setup.verification,
            fsm,
            now: 0,
            timers: Vec::new(),
            emissions: Vec::new(),
            send_log: Vec::new(),
            rejections: Vec::new(),
            lease_seq: 0,
            claim_seq: 0,
            claim_window: None,
            claim_generation: 0,
            pending_verb: None,
            gate,
        })
    }

    #[must_use]
    pub fn is_gate_target(&self) -> bool {
        self.gate.is_some()
    }

    #[must_use]
    pub fn active_claim_id(&self) -> Option<&str> {
        self.fsm.claim.as_ref().map(|c| c.id.as_str())
    }

    fn at(&self) -> MonoNs {
        MonoNs(self.now)
    }

    // -- FSM stepping and effect interpretation ----------------------------

    /// Feed one event through the session machine and interpret the
    /// resulting effects. Rejections are recorded and produce no emissions.
    fn dispatch(&mut self, event: SessionEvent) -> Result<bool, ConformanceError> {
        match waddle_fsm::step(&self.cfg, &self.fsm, &event) {
            Ok(step) => {
                self.fsm = step.next;
                self.refresh_claim_window();
                self.apply_effects(step.effects)?;
                Ok(true)
            }
            Err(rejected) => {
                self.rejections
                    .push(format!("t={}ns {event:?}: {rejected}", self.now));
                Ok(false)
            }
        }
    }

    /// Re-derive [`Self::claim_generation`] from WHICH claim is active, not
    /// from `claim.is_some()` going false: a retake replaces one claimant
    /// with another with no gap at all, and the successor deserves to hear
    /// a refusal its predecessor already heard.
    fn refresh_claim_window(&mut self) {
        let claim_id = self.fsm.claim.as_ref().map(|c| &c.id);
        if self.claim_window.as_ref() != claim_id {
            self.claim_window = claim_id.cloned();
            self.claim_generation = self.claim_generation.wrapping_add(1);
        }
    }

    fn apply_effects(&mut self, effects: Vec<Effect>) -> Result<(), ConformanceError> {
        for effect in effects {
            if let Effect::Emit(ev) = &effect {
                let value = self.codec.event_to_value(ev)?;
                self.emissions.push(EmissionEntry {
                    at_ns: ev.t_ns,
                    value,
                });
                continue;
            }
            if let Some(value) = effect_to_value(&effect) {
                self.emissions.push(EmissionEntry {
                    at_ns: self.now,
                    value,
                });
            }
            match effect {
                Effect::MintLeaseToken(_) => {
                    // The runtime mints; the conformance target mints
                    // deterministically and answers immediately.
                    self.lease_seq += 1;
                    let minted = LeaseId::new(format!("lease-{}", self.lease_seq));
                    self.dispatch(SessionEvent::LeaseTokenMinted {
                        minted,
                        at: self.at(),
                    })?;
                }
                Effect::OpenSuccessor {
                    predecessor,
                    successor,
                    born_claimed,
                    mode,
                    ..
                } => {
                    self.dispatch(SessionEvent::EpisodeOpen {
                        id: successor,
                        verification: mode,
                        born_claimed,
                        parent: Some(predecessor),
                        post_reset: false,
                        pre_window: None,
                        post_window: None,
                        agent_invite: None,
                        at: self.at(),
                    })?;
                }
                Effect::SetGateMode(mode) => self.apply_gate_plan(mode),
                Effect::RequestVerb(verb) => {
                    self.pending_verb = Some(verb);
                    // A HOLD_FIRST engage stops the caller's writer: the gate
                    // holds until the handoff completes. Holds elsewhere
                    // (tripwire, dual-write) are verb requests to the robot,
                    // not gate-plan changes.
                    if verb == Verb::Hold
                        && self.fsm.engage_stage == Some(EngageStage::AwaitHoldOk)
                        && let Some(gp) = &self.gate
                    {
                        gp.shared.store_plan(GatePlan {
                            mode: PlanMode::Held,
                            since: MonoNs(self.now),
                        });
                    }
                }
                Effect::ArmTimer { id, deadline } => {
                    self.timers.retain(|(t, _)| *t != id);
                    self.timers.push((id, deadline.0));
                }
                Effect::CancelTimer { id } => self.timers.retain(|(t, _)| *t != id),
                Effect::Emit(_)
                | Effect::ReprimePolicy
                | Effect::SetResetUnverified { .. }
                | Effect::SetPostResetFailed { .. }
                | Effect::RunPostReset { .. } => {}
            }
        }
        Ok(())
    }

    /// Interpret `Effect::SetGateMode` into a gate plan (gate target only).
    fn apply_gate_plan(&mut self, mode: GateMode) {
        let provenance = self.claim_provenance();
        let blend = self.blend_schedule();
        let Some(gp) = &self.gate else { return };
        let plan = match mode {
            // E24 (flag `waddle.v0.agent`): an agent-invited episode's
            // PASSTHROUGH projects to Noop{AGENT_EPISODE} while no claim is
            // engaged (predicate on the FSM — hollow-frontend rule).
            GateMode::Passthrough if self.fsm.agent_episode_noop() => GatePlan {
                mode: PlanMode::AgentEpisode { provenance },
                since: MonoNs(self.now),
            },
            GateMode::Passthrough => GatePlan::passthrough(MonoNs(self.now)),
            GateMode::Intervention => GatePlan {
                mode: PlanMode::Claimed { provenance, blend },
                since: MonoNs(self.now),
            },
            GateMode::Bypass => GatePlan {
                mode: PlanMode::Bypass { provenance },
                since: MonoNs(self.now),
            },
            // A remote actor is driving the reset through the SDK; every
            // caller tick gets a Noop{RESET_ACTIVE} (waddle-gate's
            // `PlanMode::Reset` arm), mirroring `Bypass`'s shape.
            GateMode::Reset => GatePlan {
                mode: PlanMode::Reset { provenance },
                since: MonoNs(self.now),
            },
        };
        gp.shared.store_plan(plan);
    }

    fn blend_schedule(&self) -> Option<BlendSchedule> {
        let interp = self.gate.as_ref().map_or(Interp::Linear, |g| g.interp);
        match self.cfg.handoff {
            waddle_types::HandoffPolicy::Immediate { blend_ns } if blend_ns > 0 => {
                Some(BlendSchedule {
                    start: MonoNs(self.now),
                    blend_ns,
                    interp,
                })
            }
            _ => None,
        }
    }

    /// Provenance carried by intervention actions under the active claim —
    /// the claim's own (`ActiveClaim::provenance`), never re-derived here.
    fn claim_provenance(&self) -> ProvenanceTag {
        self.fsm
            .claim
            .as_ref()
            .map_or_else(ProvenanceTag::policy, waddle_fsm::ActiveClaim::provenance)
    }

    // -- Virtual time -------------------------------------------------------

    pub fn advance(&mut self, ns: i64) -> Result<(), ConformanceError> {
        self.advance_to(self.now.saturating_add(ns))
    }

    pub fn advance_to(&mut self, end: i64) -> Result<(), ConformanceError> {
        while self.now < end {
            self.step_once(end)?;
        }
        Ok(())
    }

    /// One deterministic sub-step of an advance: move to the next interesting
    /// instant (periodic grid, timer deadline, or chunk end), fire due timers
    /// in deadline order, then run the gate target's periodic work.
    pub(crate) fn step_once(&mut self, end: i64) -> Result<(), ConformanceError> {
        let mut next = end.min(self.now + PERIODIC_STEP_NS);
        for (_, deadline) in &self.timers {
            if *deadline > self.now {
                next = next.min(*deadline);
            }
        }
        if let Some(gp) = &self.gate
            && let Some(chunk_end) = gp.chunk_end_ns
            && chunk_end > self.now
        {
            next = next.min(chunk_end);
        }
        self.now = next;
        if let Some(gp) = &self.gate {
            gp.clock.set(MonoNs(self.now));
        }
        self.fire_due_timers()?;
        self.periodic()?;
        Ok(())
    }

    fn fire_due_timers(&mut self) -> Result<(), ConformanceError> {
        loop {
            let due = self
                .timers
                .iter()
                .enumerate()
                .filter(|(_, (_, deadline))| *deadline <= self.now)
                .min_by_key(|(idx, (_, deadline))| (*deadline, *idx))
                .map(|(idx, _)| idx);
            let Some(idx) = due else { return Ok(()) };
            let (id, _) = self.timers.remove(idx);
            self.dispatch(SessionEvent::TimerFired { id, at: self.at() })?;
        }
    }

    /// Gate-target periodic work: chunk-boundary detection, stall detection,
    /// and the bypass pump. No-op for the fsm target.
    fn periodic(&mut self) -> Result<(), ConformanceError> {
        if self.gate.is_none() {
            return Ok(());
        }
        let boundary = {
            let gp = self.gate.as_mut().expect("gate target");
            if gp.chunk_end_ns.is_some_and(|end| self.now >= end) {
                gp.chunk_end_ns = None;
                true
            } else {
                false
            }
        };
        if boundary {
            self.dispatch(SessionEvent::ChunkBoundaryReached { at: self.at() })?;
        }
        let stalled = {
            let gp = self.gate.as_ref().expect("gate target");
            self.fsm.gate_mode == GateMode::Intervention
                && self.fsm.claim.is_some()
                && gp.traffic
                && gp
                    .last_tick_ns
                    .is_some_and(|t| self.now - t > STALL_THRESHOLD_NS)
        };
        if stalled {
            self.dispatch(SessionEvent::StallDetected { at: self.at() })?;
        }
        self.pump()?;
        Ok(())
    }

    /// Bypass pump: while the FSM is in BYPASS, due intervention actions are
    /// dispatched through the integrator's declared `send` verb directly —
    /// never starved by the stalled caller loop (FSM.md §5).
    fn pump(&mut self) -> Result<(), ConformanceError> {
        if self.fsm.gate_mode != GateMode::Bypass {
            return Ok(());
        }
        let provenance = self.claim_provenance().to_pb();
        let provenance_json = self.codec.provenance_to_value(&provenance)?;
        let mut sent: Vec<OwnedAction> = Vec::new();
        {
            let gp = self.gate.as_mut().expect("bypass implies gate target");
            while let Some(action) = gp.shared.stream.lock().pop_due(MonoNs(self.now)) {
                sent.push(action);
            }
            if let Some(last) = sent.last() {
                gp.commanded.record(last.part.as_deref(), &last.values);
            }
        }
        for action in &sent {
            self.send_log.push(EmissionEntry {
                at_ns: self.now,
                value: json!({
                    "provenance": provenance_json,
                    "at": self.now.to_string(),
                    "dims": action.values.len(),
                }),
            });
        }
        Ok(())
    }

    // -- Injections ----------------------------------------------------------

    #[allow(clippy::too_many_lines)]
    pub fn inject(&mut self, payload: &Map<String, Value>) -> Result<(), ConformanceError> {
        let kind = payload
            .get("kind")
            .and_then(Value::as_str)
            .ok_or_else(|| scenario_err("inject missing string \"kind\""))?;
        let at = self.at();
        match kind {
            "episode_open" => {
                let id = str_field(payload, "episode_id")?;
                let verification = match payload.get("verification_mode").and_then(Value::as_str) {
                    Some(s) => parse_verification_mode(s)?,
                    None => self.default_verification,
                };
                let born_claimed = payload
                    .get("born_claimed")
                    .and_then(Value::as_bool)
                    .unwrap_or(false);
                let parent = payload
                    .get("parent_episode_id")
                    .and_then(Value::as_str)
                    .map(EpisodeId::new);
                let post_reset = payload
                    .get("post_reset")
                    .and_then(Value::as_bool)
                    .unwrap_or(false);
                let pre_window = payload
                    .get("pre_reset_window")
                    .map(|v| self.parse_window_spec(v))
                    .transpose()?;
                let post_window = payload
                    .get("post_reset_window")
                    .map(|v| self.parse_window_spec(v))
                    .transpose()?;
                let agent_invite = payload
                    .get("agent_invite")
                    .map(parse_agent_invite)
                    .transpose()?;
                self.dispatch(SessionEvent::EpisodeOpen {
                    id: EpisodeId::new(id),
                    verification,
                    born_claimed,
                    parent,
                    post_reset,
                    pre_window,
                    post_window,
                    agent_invite,
                    at,
                })?;
            }
            "reset_result" => {
                let result: pb::ResetResult =
                    self.parse_payload(payload, "result", "waddle.v0.ResetResult")?;
                self.dispatch(SessionEvent::ResetResult {
                    ok: result.ok,
                    verified: result.verification.as_ref().map(|v| v.verified),
                    at,
                })?;
            }
            "verification_result" => {
                let verification: pb::ResetVerification =
                    self.parse_payload(payload, "verification", "waddle.v0.ResetVerification")?;
                self.dispatch(SessionEvent::VerificationResult {
                    verified: verification.verified,
                    invalidated_async: verification.invalidated_async,
                    at,
                })?;
            }
            "start" => {
                self.dispatch(SessionEvent::Start { at })?;
            }
            "gate_tick" => self.inject_gate_tick(payload)?,
            "chunk_arrival" => {
                let chunk: pb::ActionChunk =
                    self.parse_payload(payload, "chunk", "waddle.v0.ActionChunk")?;
                let now = self.now;
                let gp = self.gate_mut("chunk_arrival")?;
                gp.chunk_end_ns = Some(now.saturating_add(chunk.horizon_ns));
            }
            "teleop_action" => self.inject_teleop_action(payload)?,
            "intervention_chunk" => self.inject_intervention_chunk(payload)?,
            "claim_request" => {
                let source = payload
                    .get("source_name")
                    .and_then(Value::as_str)
                    .unwrap_or("")
                    .to_owned();
                // `claim_request` carries a `waddle.v0.ClaimEpisodeRequest`,
                // whose `actor` is a full `ActorRef` — decoded whole, since
                // the claim emission this produces carries it whole.
                let actor = match payload.get("actor") {
                    Some(a) => ActorRef {
                        kind: match a.get("kind").and_then(Value::as_str) {
                            Some(k) => parse_actor_kind(k)?,
                            None => ActorKind::Custom,
                        },
                        id: a.get("id").and_then(Value::as_str).unwrap_or("").to_owned(),
                        display_name: a
                            .get("displayName")
                            .and_then(Value::as_str)
                            .unwrap_or("")
                            .to_owned(),
                    },
                    None => ActorRef::of_kind(ActorKind::Custom),
                };
                let self_initiated = payload
                    .get("self_initiated")
                    .and_then(Value::as_bool)
                    .unwrap_or(false);
                self.claim_seq += 1;
                let id = payload.get("claim_id").and_then(Value::as_str).map_or_else(
                    || ClaimId::new(format!("claim-req-{}", self.claim_seq)),
                    ClaimId::new,
                );
                self.dispatch(SessionEvent::ClaimRequested {
                    id,
                    source,
                    actor,
                    self_initiated,
                    at,
                })?;
            }
            "claim_granted" => {
                let claim: pb::Claim = self.parse_payload(payload, "claim", "waddle.v0.Claim")?;
                // The scenario's `waddle.v0.Claim` carries the actor WHOLE
                // (kind, id, display name) — dispatched whole, so the claim
                // emissions the scenario then asserts against carry it too.
                let actor = claim
                    .actor
                    .as_ref()
                    .map(ActorRef::try_from)
                    .transpose()?
                    .unwrap_or_else(|| ActorRef::of_kind(ActorKind::Custom));
                self.dispatch(SessionEvent::ClaimGranted {
                    id: ClaimId::new(&claim.claim_id),
                    source: claim.source_name.clone(),
                    actor,
                    self_initiated: claim.self_initiated,
                    at,
                })?;
            }
            "claim_released" => {
                let id = str_field(payload, "claim_id")?;
                self.dispatch(SessionEvent::ClaimReleased {
                    id: ClaimId::new(id),
                    at,
                })?;
            }
            "engage" => {
                let claim = str_field(payload, "claim_id")?;
                self.dispatch(SessionEvent::Engage {
                    claim: ClaimId::new(claim),
                    at,
                })?;
            }
            "release" => {
                let claim = str_field(payload, "claim_id")?;
                self.dispatch(SessionEvent::Release {
                    claim: ClaimId::new(claim),
                    at,
                })?;
            }
            "retake" => {
                let claim = str_field(payload, "claim_id")?;
                let initiator = parse_actor_kind(str_field(payload, "initiator")?)?;
                let successor = str_field(payload, "successor_episode_id")?;
                self.dispatch(SessionEvent::Retake {
                    claim: ClaimId::new(claim),
                    initiator,
                    successor: EpisodeId::new(successor),
                    at,
                })?;
            }
            "clutch" => {
                let engaged = payload
                    .get("engaged")
                    .and_then(Value::as_bool)
                    .ok_or_else(|| scenario_err("clutch requires bool \"engaged\""))?;
                self.dispatch(SessionEvent::Clutch { engaged, at })?;
            }
            "verb_result" => {
                let result: pb::VerbResult =
                    self.parse_payload(payload, "result", "waddle.v0.VerbResult")?;
                // Correlate to the single pending verb request; a stray
                // result (e.g. modeling a non-holder send) defaults to SEND.
                let verb = self.pending_verb.take().unwrap_or(Verb::Send);
                let fault = result
                    .fault
                    .as_ref()
                    .and_then(|f| pb::FaultKind::try_from(f.kind).ok());
                self.dispatch(SessionEvent::VerbResult {
                    verb,
                    ok: result.ok,
                    fault,
                    at,
                })?;
            }
            "estop" => {
                self.dispatch(SessionEvent::Estop { at })?;
            }
            "terminate" => {
                let outcome_name = str_field(payload, "outcome")?;
                let outcome_pb = pb::TerminalOutcome::from_str_name(outcome_name)
                    .ok_or_else(|| scenario_err(format!("unknown outcome {outcome_name:?}")))?;
                let outcome = TerminalOutcome::from_pb(outcome_pb as i32)?;
                let reason = payload
                    .get("reason")
                    .and_then(Value::as_str)
                    .unwrap_or("")
                    .to_owned();
                self.dispatch(SessionEvent::Terminate {
                    outcome,
                    reason,
                    at,
                })?;
            }
            "judge_result" => {
                let judgment: pb::Judgment =
                    self.parse_payload(payload, "judgment", "waddle.v0.Judgment")?;
                self.dispatch(SessionEvent::JudgeResult {
                    judge_id: judgment.judge_id.clone(),
                    passed: judgment.passed,
                    at,
                })?;
            }
            "mark" => {
                let mark: pb::MarkEvent =
                    self.parse_payload(payload, "mark", "waddle.v0.MarkEvent")?;
                let kind = match pb::MarkKind::try_from(mark.kind) {
                    Ok(pb::MarkKind::Start) => MarkKind::Start,
                    Ok(pb::MarkKind::EndSuccess) => MarkKind::EndSuccess,
                    Ok(pb::MarkKind::EndFailure) => MarkKind::EndFailure,
                    Ok(pb::MarkKind::EndAbort) => MarkKind::EndAbort,
                    Ok(pb::MarkKind::Retake) => MarkKind::Retake,
                    Ok(pb::MarkKind::Unspecified) | Err(_) => {
                        return Err(scenario_err("mark requires a specified MarkKind"));
                    }
                };
                self.dispatch(SessionEvent::Mark { kind, at })?;
            }
            "proxy_signals" => {
                let signals: pb::ProxySignals =
                    self.parse_payload(payload, "signals", "waddle.v0.ProxySignals")?;
                let sample = ProxySample {
                    control_rtt_ns: signals.control_rtt_ns,
                    gate_tick_p95_ns: signals.gate_tick.as_ref().map_or(0, |j| j.p95_ns),
                    callback_dispatch_p95_ns: signals
                        .callback_dispatch
                        .as_ref()
                        .map_or(0, |j| j.p95_ns),
                    host_load_1m: signals.host_load_1m,
                };
                self.dispatch(SessionEvent::ProxySignals { sample, at })?;
            }
            "heartbeat_ack" => {
                let ack: pb::HeartbeatAck =
                    self.parse_payload(payload, "ack", "waddle.v0.HeartbeatAck")?;
                let mut grant_changes = Vec::with_capacity(ack.grant_changes.len());
                for change in &ack.grant_changes {
                    let verb = Verb::from_pb(change.verb)?;
                    let to = match pb::GrantStatus::try_from(change.to) {
                        Ok(pb::GrantStatus::Active) => GrantStatus::Active,
                        Ok(pb::GrantStatus::Demoted) => GrantStatus::Demoted,
                        Ok(pb::GrantStatus::Revoked) => GrantStatus::Revoked,
                        Ok(pb::GrantStatus::Unspecified) | Err(_) => {
                            return Err(scenario_err("grant change requires a specified status"));
                        }
                    };
                    grant_changes.push(GrantChangeDirective {
                        verb,
                        to,
                        reason: change.reason.clone(),
                    });
                }
                self.dispatch(SessionEvent::HeartbeatAck { grant_changes, at })?;
            }
            "partition_start" => {
                self.dispatch(SessionEvent::PartitionStart { at })?;
            }
            "partition_end" => {
                self.dispatch(SessionEvent::PartitionEnd { at })?;
            }
            "proprio_sample" => self.inject_proprio_sample(payload)?,
            "post_reset_result" => {
                let result: pb::ResetResult =
                    self.parse_payload(payload, "result", "waddle.v0.ResetResult")?;
                self.dispatch(SessionEvent::PostResetResult {
                    ok: result.ok,
                    detail: result.detail.clone(),
                    at,
                })?;
            }
            "reset_window_engage" => {
                let claim = str_field(payload, "claim_id")?;
                self.dispatch(SessionEvent::ResetWindowEngage {
                    claim: ClaimId::new(claim),
                    at,
                })?;
            }
            "reset_window_complete" => {
                let claim = str_field(payload, "claim_id")?;
                let result: pb::ResetResult =
                    self.parse_payload(payload, "result", "waddle.v0.ResetResult")?;
                self.dispatch(SessionEvent::ResetWindowComplete {
                    claim: ClaimId::new(claim),
                    ok: result.ok,
                    verified: result.verification.as_ref().map(|v| v.verified),
                    at,
                })?;
            }
            "agent_task_update" => {
                // The update rides as a `waddle.v0.AgentTaskUpdate` under
                // `update` (the message's own `kind` field cannot ride flat
                // next to the inject dispatcher's `kind` key —
                // scenario-format.md nests it like `reset_result`'s).
                let update: pb::AgentTaskUpdate =
                    self.parse_payload(payload, "update", "waddle.v0.AgentTaskUpdate")?;
                let update_kind = pb::AgentTaskUpdateKind::try_from(update.kind)
                    .map_err(|_| scenario_err(format!("unknown update kind {}", update.kind)))?;
                let targets_active = self
                    .fsm
                    .episode
                    .as_ref()
                    .is_some_and(|ep| ep.id.as_str() == update.episode_id);
                match update_kind {
                    pb::AgentTaskUpdateKind::Unspecified => {
                        return Err(scenario_err("agent_task_update requires a specified kind"));
                    }
                    // E26/E26b: only a DENIED addressed to the active
                    // episode becomes an FSM event; the FSM decides between
                    // the E26 transition and the E26b recorded-only
                    // rejection.
                    pb::AgentTaskUpdateKind::Denied if targets_active => {
                        self.dispatch(SessionEvent::AgentTaskDenied {
                            detail: update.detail.clone(),
                            at,
                        })?;
                    }
                    // QUEUED and COMPLETED are informational on every state
                    // (FSM.md §1.5) — recorded as inert, never a transition;
                    // likewise any update addressed to a non-active episode.
                    _ => {
                        self.rejections.push(format!(
                            "t={}ns agent_task_update{{{}, episode {:?}}}: \
                             informational, recorded inert (FSM.md §1.5)",
                            self.now,
                            update_kind.as_str_name(),
                            update.episode_id,
                        ));
                    }
                }
            }
            other => {
                // Closed set (scenario-format.md): unknown kinds are a
                // scenario error, never silently ignored.
                return Err(scenario_err(format!("unknown inject kind {other:?}")));
            }
        }
        Ok(())
    }

    fn inject_gate_tick(&mut self, payload: &Map<String, Value>) -> Result<(), ConformanceError> {
        let at = self.at();
        let (values, gripper) = match payload.get("action") {
            Some(action) => self.flatten_action_value(action)?,
            None => (vec![0.0; self.default_dims()], None),
        };
        let now = self.now;
        {
            let gp = self.gate_mut("gate_tick")?;
            gp.clock.set(MonoNs(now));
            // The scenario schema has no `obs` field on gate_tick yet;
            // adding one is protocol work.
            let output = gp.gate.gate(&values, gripper, None);
            match &output {
                // The caller's own action always commands the whole declared
                // space; an intervention action carries the part it
                // addresses, or none for a whole-robot one.
                GateOutput::Pass { .. } => gp.commanded.record(None, &values),
                GateOutput::Substitute { action, .. } | GateOutput::Blend { action, .. } => {
                    gp.commanded.record(action.part.as_deref(), &action.values);
                }
                GateOutput::Noop { .. } | GateOutput::Hold => {}
            }
            // `GateOutput::Noop` deliberately carries no reason — the
            // reducer's marker translation derives it from the gate
            // decision. The harness plays that role here, from the plan
            // that produced this tick (captured NOW: a later event may
            // legally change the plan before `expect_output` runs).
            gp.last_noop_reason = match (&output, &gp.shared.load_plan().mode) {
                (GateOutput::Noop { .. }, PlanMode::Bypass { .. }) => {
                    Some(pb::NoopReason::BypassActive)
                }
                (GateOutput::Noop { .. }, PlanMode::Reset { .. }) => {
                    Some(pb::NoopReason::ResetActive)
                }
                (GateOutput::Noop { .. }, PlanMode::AgentEpisode { .. }) => {
                    Some(pb::NoopReason::AgentEpisode)
                }
                _ => None,
            };
            gp.last_output = Some(output);
            gp.last_tick_ns = Some(now);
        }
        // The first gated action drives READY → RUNNING (E6).
        self.dispatch(SessionEvent::GateTick { at })?;
        // A caller tick observed during BYPASS: the stalled loop's tick saw
        // its Noop marker above; ticks resuming flips the mode back (§6).
        if self.fsm.gate_mode == GateMode::Bypass {
            self.dispatch(SessionEvent::TicksResumed { at })?;
        }
        Ok(())
    }

    fn inject_teleop_action(
        &mut self,
        payload: &Map<String, Value>,
    ) -> Result<(), ConformanceError> {
        let packet: pb::TeleopStreamPacket =
            self.parse_payload(payload, "packet", "waddle.v0.TeleopStreamPacket")?;
        let (values, gripper) = flatten_teleop_targets(&packet);
        let now = self.now;
        let at = self.at();
        // The window this packet is admitted into is the window its refusal
        // is owed to ([`WindowLatch`]).
        let generation = self.claim_generation;
        let expected_dims = self
            .gate
            .as_ref()
            .and_then(|g| g.space.as_ref())
            .and_then(ActionSpace::dims);
        let dims_ok = expected_dims.is_none_or(|want| values.len() == want);
        // The harness never models the media-intake thread (production's
        // *unconditional* primary check, already covered by
        // waddle-runtime's e2e suite) — only the gate's own blend-window
        // defense-in-depth (`blend.rs::blend_step` returning `None`) is
        // observable here, and only while a blend is actually in progress.
        // Scoping the harness check to that same window keeps it a no-op
        // for every other teleop_action fixture (bypass, HOLD_FIRST,
        // CHUNK_BOUNDARY never open a blend window), so it cannot alter
        // their observable behavior.
        let blend_active = self.gate.as_ref().is_some_and(|gp| {
            matches!(
                &gp.shared.load_plan().mode,
                PlanMode::Claimed { blend: Some(b), .. } if b.progress(MonoNs(now)) < 1.0
            )
        });
        let mut rejected = None;
        {
            let gp = self.gate_mut("teleop_action")?;
            if dims_ok || !blend_active {
                gp.producer
                    .push(TimedAction {
                        channel: StreamChannel::Teleop,
                        seq: packet.seq,
                        received: MonoNs(now),
                        action: OwnedAction {
                            values: ActionValues::from_slice(&values),
                            gripper,
                            // As in production: a teleop packet is not
                            // part-addressed (`flatten_packet`).
                            part: None,
                        },
                        chunk: None,
                    })
                    .map_err(|_| scenario_err("intervention stream ring full"))?;
            } else if gp.teleop_dims_fault.raise(generation) {
                rejected = Some((values.len(), expected_dims.unwrap_or(0)));
            }
            gp.traffic = true;
        }
        // Dims validation: a dims-mismatched injection during an open blend window is
        // never dispatched — nothing goes to the ring, and the claim
        // window gets exactly one Fault{VALIDATION_ERROR} (waddle-fsm's
        // `InterventionRejected` handling), not one per mismatched packet.
        if let Some((got, want)) = rejected {
            self.dispatch(SessionEvent::InterventionRejected {
                source: "media-intake",
                reason: waddle_fsm::RejectReason::Dims { got, want },
                at,
            })?;
        }
        // Arrival is an event for stall detection and (in bypass) the pump.
        self.periodic()
    }

    /// Agent-chunk intake: validate one wire `intervention_chunk` against
    /// the declared space and buffer its steps on the intervention stream
    /// (`GateServerMessage.intervention_chunk`; scenario-format.md's
    /// `intervention_chunk`, flag `waddle.v0.agent`).
    ///
    /// Admitted on an ACTIVE CLAIM alone — no `GateMode` match, the same
    /// gate the teleop path uses — so a chunk arriving during the ENGAGE
    /// handoff buffers correctly and is ready the instant the handoff
    /// completes. With no claim there is nobody to buffer for and the chunk
    /// is dropped, recorded in the rejection log rather than faulted: an
    /// unclaimed sender is not a sender making a mistake about the space.
    ///
    /// Refusal shape, per FSM.md §4: a step that doesn't fit the declared
    /// space refuses the WHOLE chunk (a partial trajectory from a sender
    /// that disagrees about the space is not a degraded-but-safe thing to
    /// actuate), reported as one `Fault{VALIDATION_ERROR}` per reason per
    /// claim window; INERT steps are skipped, the rest of the chunk still
    /// executes, and the skip is reported the same way.
    ///
    /// Playout is receive-time + each step's `t_offset_ns`, never the
    /// chunk's own `t_emitted_ns` (a remote sender's emit clock is not an
    /// anchor on this side); `t_emitted_ns` and `seq` ride along as
    /// `ChunkMeta` so the jitter buffer can see a chunk boundary and apply
    /// the declared `ReplanPolicy`. Steps are tagged
    /// `StreamChannel::AgentChunk` so this seq space never shares a
    /// reorder cursor with the teleop packets' (`jitter.rs`).
    fn inject_intervention_chunk(
        &mut self,
        payload: &Map<String, Value>,
    ) -> Result<(), ConformanceError> {
        let chunk: pb::ActionChunk =
            self.parse_payload(payload, "chunk", "waddle.v0.ActionChunk")?;
        let now = self.now;
        let at = self.at();
        let generation = self.claim_generation;
        // A gate-target kind whether or not a claim is active: a scenario
        // that injects one at the `fsm` target is a scenario error, not a
        // quietly dropped chunk.
        self.gate_mut("intervention_chunk")?;
        if self.fsm.claim.is_none() {
            self.rejections.push(format!(
                "t={now}ns intervention_chunk{{seq {}}}: no active claim, nothing to buffer for",
                chunk.seq
            ));
            return Ok(());
        }
        let total = chunk.actions.len();
        let mut rejected: Option<waddle_fsm::RejectReason> = None;
        {
            let gp = self.gate_mut("intervention_chunk")?;
            let space = gp.space.as_ref().ok_or_else(|| {
                scenario_err("intervention_chunk requires a declared action space (robot_fixture)")
            })?;
            // Whether `Action.part` is honored (flag `waddle.v0.parts`).
            // A runner has no negotiated connection to read the answer off,
            // so it falls back to the same fact an SDK declares the flag
            // from: whether the declared space has parts at all. A scenario
            // whose steps address parts declares the flag in
            // `requires_features`, which is what got it run at all.
            let parts = part_policy(space);
            match ActionChunk::from_pb(&chunk, space, parts) {
                Ok(flattened) => {
                    let meta = ChunkMeta {
                        chunk_seq: flattened.chunk.seq,
                        t_emitted_ns: flattened.chunk.t_emitted_ns,
                    };
                    for step in &flattened.chunk.steps {
                        gp.chunk_intake.next_seq += 1;
                        gp.producer
                            .push(TimedAction {
                                channel: StreamChannel::AgentChunk,
                                seq: gp.chunk_intake.next_seq,
                                received: MonoNs(now.saturating_add(step.offset_ns)),
                                action: OwnedAction {
                                    values: step.values.clone(),
                                    gripper: step.gripper,
                                    // `Some` only under `PartPolicy::Honor`,
                                    // where the flattener minted the tag from
                                    // the part this step addresses.
                                    part: step.part.clone(),
                                },
                                chunk: Some(meta),
                            })
                            .map_err(|_| scenario_err("intervention stream ring full"))?;
                    }
                    if !flattened.inert.is_empty() && gp.chunk_intake.inert.raise(generation) {
                        rejected = Some(waddle_fsm::RejectReason::InertStepsSkipped {
                            skipped: flattened.inert.len(),
                            of: total,
                        });
                    }
                }
                Err(TypesError::DimensionMismatch { expected, got }) => {
                    if gp.chunk_intake.dims.raise(generation) {
                        rejected = Some(waddle_fsm::RejectReason::Dims {
                            got,
                            want: expected,
                        });
                    }
                }
                // Every other `TypesError` — a target arm the space doesn't
                // have, a part it doesn't declare, a missing field — means
                // this chunk isn't speaking the declared space. Reported in
                // its own words rather than forced into the dims-shaped
                // report, and latched separately: "which parts exist" and
                // "how wide this part is" are different disagreements.
                Err(err) => {
                    if gp.chunk_intake.not_executable.raise(generation) {
                        rejected = Some(waddle_fsm::RejectReason::NotExecutable(err.to_string()));
                    }
                }
            }
            gp.traffic = true;
        }
        if let Some(reason) = rejected {
            self.dispatch(SessionEvent::InterventionRejected {
                source: AGENT_CHUNK_SOURCE,
                reason,
                at,
            })?;
        }
        // Arrival is an event for stall detection and (in bypass) the pump.
        self.periodic()
    }

    fn inject_proprio_sample(
        &mut self,
        payload: &Map<String, Value>,
    ) -> Result<(), ConformanceError> {
        let sample: pb::ProprioSample =
            self.parse_payload(payload, "sample", "waddle.v0.ProprioSample")?;
        let t_ns = match payload.get("t_ns") {
            Some(v) => parse_ns(v)?,
            None => self.now,
        };
        // Commanded and observed must describe the same scope before they
        // are compared at all ([`Commanded::expected_for`]), and each
        // stream's divergence run stands on its own.
        let verdict = {
            let gp = self.gate_mut("proprio_sample")?;
            match gp.commanded.expected_for(gp.space.as_ref(), &sample.part) {
                Some(commanded) => gp
                    .detectors
                    .entry(sample.part.clone())
                    .or_insert_with(|| {
                        DivergenceDetector::new(DIVERGENCE_THRESHOLD, DIVERGENCE_WINDOW_NS)
                    })
                    .feed(commanded, &sample.joint_pos, MonoNs(t_ns)),
                None => None,
            }
        };
        if let Some(verdict) = verdict {
            let trace_ref = {
                let gp = self.gate_mut("proprio_sample")?;
                gp.incident_seq += 1;
                format!("incident-{}", gp.incident_seq)
            };
            self.dispatch(SessionEvent::DualWrite {
                divergence_metric: verdict.divergence_metric,
                window_ns: verdict.window_ns,
                trace_ref,
                at: self.at(),
            })?;
        }
        Ok(())
    }

    fn gate_mut(&mut self, kind: &str) -> Result<&mut GateParts, ConformanceError> {
        self.gate
            .as_mut()
            .ok_or_else(|| scenario_err(format!("inject kind {kind:?} requires the gate target")))
    }

    fn parse_payload<T: prost::Message + Default>(
        &self,
        payload: &Map<String, Value>,
        field: &str,
        full_name: &str,
    ) -> Result<T, ConformanceError> {
        let value = payload
            .get(field)
            .ok_or_else(|| scenario_err(format!("inject payload missing {field:?}")))?;
        self.codec.parse(full_name, value)
    }

    /// Parse an `episode_open` window declaration (`{expected_actor, prompt?,
    /// timeout_ns}`, scenario-format.md's `pre_reset_window`/
    /// `post_reset_window` keys).
    fn parse_window_spec(&self, value: &Value) -> Result<WindowSpec, ConformanceError> {
        let obj = value
            .as_object()
            .ok_or_else(|| scenario_err("reset window spec must be an object"))?;
        let expected = parse_actor_kind(str_field(obj, "expected_actor")?)?;
        let prompt = obj
            .get("prompt")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_owned();
        let timeout_ns = parse_ns(
            obj.get("timeout_ns")
                .ok_or_else(|| scenario_err("reset window spec missing \"timeout_ns\""))?,
        )?;
        Ok(WindowSpec {
            expected,
            prompt,
            timeout_ns,
        })
    }

    fn default_dims(&self) -> usize {
        self.gate
            .as_ref()
            .and_then(|g| g.space.as_ref())
            .and_then(ActionSpace::dims)
            .unwrap_or(1)
    }

    /// Flatten a caller action for `gate()`. Strict flattening against the
    /// declared space is attempted first; fixture actions that elide gripper
    /// channels fall back to a documented permissive extraction (numbers in
    /// payload order: joint vectors, twists as 6 values, poses as
    /// `[x,y,z,qw,qx,qy,qz]`, composite parts in message order).
    fn flatten_action_value(
        &self,
        value: &Value,
    ) -> Result<(Vec<f64>, Option<f64>), ConformanceError> {
        if let Some(space) = self.gate.as_ref().and_then(|g| g.space.as_ref())
            && let Ok(action) = self.codec.parse::<pb::Action>("waddle.v0.Action", value)
            && let Ok(step) =
                waddle_types::action::flatten_action(&action, space, PartPolicy::Ignore)
        {
            return Ok((step.values.to_vec(), step.gripper));
        }
        let mut values = Vec::new();
        let mut gripper = None;
        collect_action_numbers(value, &mut values, &mut gripper);
        if values.is_empty() {
            return Err(scenario_err("gate_tick action carries no numeric target"));
        }
        Ok((values, gripper))
    }

    // -- Observation surface -------------------------------------------------

    /// The scenario-format state-snapshot document.
    #[must_use]
    pub fn snapshot(&self) -> Value {
        let episode = match &self.fsm.episode {
            Some(ep) => {
                let outcome = match ep.phase {
                    waddle_fsm::Phase::Terminal(o) => o.to_pb().as_str_name(),
                    _ => pb::TerminalOutcome::Unspecified.as_str_name(),
                };
                let intervention_phase = match ep.phase {
                    waddle_fsm::Phase::Intervention(p) => intervention_phase_name(p),
                    _ => pb::InterventionPhase::Unspecified.as_str_name(),
                };
                let pinned_outcome = ep
                    .pinned_outcome
                    .map_or(pb::TerminalOutcome::Unspecified.as_str_name(), |o| {
                        o.to_pb().as_str_name()
                    });
                json!({
                    "id": ep.id.as_str(),
                    "state": ep.phase.kind().to_pb().as_str_name(),
                    "outcome": outcome,
                    "intervention_phase": intervention_phase,
                    "born_claimed": ep.born_claimed,
                    "reset_unverified": ep.reset_unverified,
                    "parent_episode_id": ep.parent.as_ref().map_or("", |p| p.as_str()),
                    "post_reset_declared": ep.post_reset_declared,
                    "post_reset_failed": ep.post_reset_failed,
                    "pinned_outcome": pinned_outcome,
                    "agent_invited": ep.agent_invited,
                    "agent_engaged": ep.agent_engaged,
                })
            }
            None => json!({
                "id": "",
                "state": pb::EpisodeState::Unspecified.as_str_name(),
                "outcome": pb::TerminalOutcome::Unspecified.as_str_name(),
                "intervention_phase": pb::InterventionPhase::Unspecified.as_str_name(),
                "born_claimed": false,
                "reset_unverified": false,
                "parent_episode_id": "",
                "post_reset_declared": false,
                "post_reset_failed": false,
                "pinned_outcome": pb::TerminalOutcome::Unspecified.as_str_name(),
                "agent_invited": false,
                "agent_engaged": false,
            }),
        };
        let reset_window = match self
            .fsm
            .episode
            .as_ref()
            .and_then(|ep| ep.reset_window.as_ref())
        {
            Some(w) => json!({
                "open": true,
                "kind": w.kind.to_pb().as_str_name(),
                "expected_actor": w.expected.to_pb().as_str_name(),
                "claim_id": self.fsm.claim.as_ref().map_or("", |c| c.id.as_str()),
            }),
            None => json!({
                "open": false,
                "kind": pb::ResetKind::Unspecified.as_str_name(),
                "expected_actor": pb::ActorKind::Unspecified.as_str_name(),
                "claim_id": "",
            }),
        };
        let (lease_id, holder) = match self.fsm.lease.holder() {
            Some((lease, client)) => (lease.as_str(), client.as_str()),
            None => ("", ""),
        };
        let enforcement = match self.cfg.enforcement {
            LeaseEnforcement::Enforced => pb::LeaseEnforcement::Enforced.as_str_name(),
            LeaseEnforcement::Advisory => pb::LeaseEnforcement::Advisory.as_str_name(),
        };
        let (claim_id, source_name, self_initiated) = match &self.fsm.claim {
            Some(c) => (c.id.as_str(), c.source.as_str(), c.self_initiated),
            None => ("", "", false),
        };
        let grants: Vec<Value> = self
            .fsm
            .grants
            .iter()
            .zip(&self.cfg.grants)
            .map(|(entry, grant)| {
                json!({
                    "verb": verb_name(entry.verb),
                    "send_interfaces": grant
                        .send_interfaces
                        .iter()
                        .map(|k| k.to_pb().as_str_name())
                        .collect::<Vec<_>>(),
                    "status": grant_status_name(entry.status),
                })
            })
            .collect();
        json!({
            "episode": episode,
            "gate": { "mode": gate_mode_name(self.fsm.gate_mode) },
            "lease": {
                "holder_client_id": holder,
                "lease_id": lease_id,
                "enforcement": enforcement,
            },
            "claim": {
                "active_claim_id": claim_id,
                "source_name": source_name,
                "self_initiated": self_initiated,
            },
            "grants": grants,
            "plane": {
                "connected": self.fsm.plane_connected,
                "buffered_events": self.fsm.buffered_events,
            },
            "reset_window": reset_window,
        })
    }

    /// The most recent `gate_tick` result as a matchable document
    /// (`expect_output`).
    pub fn last_output_value(&self) -> Result<Option<Value>, ConformanceError> {
        let Some(gp) = &self.gate else {
            return Err(scenario_err("expect_output requires the gate target"));
        };
        let Some(output) = &gp.last_output else {
            return Ok(None);
        };
        let value = match output {
            GateOutput::Pass { provenance } => json!({
                "kind": "pass",
                "provenance": self.provenance_json(provenance)?,
            }),
            // `part` (flag `waddle.v0.parts`) rides the two kinds that
            // return a Waddle-sourced action, and only those: the addressed
            // part's name, or `""` for an action that commands the whole
            // declared space — present either way, so a scenario can pin
            // "untagged" as a fact rather than by omission.
            GateOutput::Substitute { action, provenance } => json!({
                "kind": "substitute",
                "part": action.part.as_deref().unwrap_or(""),
                "provenance": self.provenance_json(provenance)?,
            }),
            GateOutput::Blend {
                action,
                progress,
                provenance,
            } => json!({
                "kind": "blend",
                "part": action.part.as_deref().unwrap_or(""),
                "progress": progress,
                "provenance": self.provenance_json(provenance)?,
            }),
            GateOutput::Noop { provenance } => json!({
                "kind": "noop",
                "reason": gp
                    .last_noop_reason
                    .unwrap_or(pb::NoopReason::Unspecified)
                    .as_str_name(),
                "provenance": self.provenance_json(provenance)?,
            }),
            GateOutput::Hold => json!({ "kind": "hold" }),
        };
        Ok(Some(value))
    }

    fn provenance_json(&self, tag: &ProvenanceTag) -> Result<Value, ConformanceError> {
        self.codec.provenance_to_value(&tag.to_pb())
    }
}

/// Parse an `episode_open` agent-invite declaration (`{prompt, timeout_ns}`,
/// scenario-format.md's `agent_invite` key — flag `waddle.v0.agent`).
fn parse_agent_invite(value: &Value) -> Result<AgentInvite, ConformanceError> {
    let obj = value
        .as_object()
        .ok_or_else(|| scenario_err("agent_invite must be an object"))?;
    let prompt = str_field(obj, "prompt")?.to_owned();
    let timeout_ns = parse_ns(
        obj.get("timeout_ns")
            .ok_or_else(|| scenario_err("agent_invite missing \"timeout_ns\""))?,
    )?;
    Ok(AgentInvite { prompt, timeout_ns })
}

/// Whether an intervention intake honors `Action.part` (flag
/// `waddle.v0.parts`): only a `Composite` declaration has parts to address,
/// and on every other space `""` — the sole part — is what the pre-flag
/// reading already meant, so the two policies agree there.
fn part_policy(space: &ActionSpace) -> PartPolicy {
    match space.spec {
        SpaceSpec::Composite { .. } => PartPolicy::Honor,
        _ => PartPolicy::Ignore,
    }
}

/// Where a declared part's values sit in a WHOLE-ROBOT action vector: a
/// composite action is its parts' values concatenated in declaration order
/// (`waddle_types::action`'s flatten), so a part's span is the widths
/// declared before it, and its own. `None` when the space declares no parts,
/// or no part by this name.
///
/// Layout arithmetic over the DECLARATION, and applied to commands only —
/// an observation's layout is the customer's own, declared nowhere, so it is
/// never sliced this way.
fn part_span(space: &ActionSpace, name: &str) -> Option<(usize, usize)> {
    let SpaceSpec::Composite { parts } = &space.spec else {
        return None;
    };
    let mut offset = 0;
    for (part, part_space) in parts {
        let dims = part_space.dims()?;
        if part == name {
            return Some((offset, dims));
        }
        offset += dims;
    }
    None
}

fn parse_actor_kind(s: &str) -> Result<ActorKind, ConformanceError> {
    let value = pb::ActorKind::from_str_name(s)
        .ok_or_else(|| scenario_err(format!("unknown actor kind {s:?}")))?;
    Ok(ActorKind::from_pb(value as i32)?)
}

fn str_field<'m>(
    payload: &'m Map<String, Value>,
    field: &str,
) -> Result<&'m str, ConformanceError> {
    payload
        .get(field)
        .and_then(Value::as_str)
        .ok_or_else(|| scenario_err(format!("inject payload missing string {field:?}")))
}

/// Simple documented teleop flattening: ALL part targets concatenate in
/// packet order — pose → `[x, y, z, qw, qx, qy, qz]`, twist → 6 values —
/// matching production `flatten_packet` semantics
/// (`waddle-runtime/src/pumps.rs`); the first declared gripper channel rides
/// along. scenario-format.md's `teleop_action` payload is a plain
/// `waddle.v0.TeleopStreamPacket` and does not pin "first target only" — a
/// runner that only read `targets[0]` was a runner defect (see
/// `handoff_immediate_mid_chunk`'s amendment).
fn flatten_teleop_targets(packet: &pb::TeleopStreamPacket) -> (Vec<f64>, Option<f64>) {
    let mut values = Vec::new();
    let mut gripper = None;
    for target in &packet.targets {
        match &target.target {
            Some(pb::part_target::Target::Pose(pose)) => {
                if let Some(p) = &pose.position {
                    values.extend([p.x, p.y, p.z]);
                }
                if let Some(r) = &pose.rotation {
                    values.extend([r.w, r.x, r.y, r.z]);
                }
            }
            Some(pb::part_target::Target::Twist(twist)) => {
                if let Some(l) = &twist.linear {
                    values.extend([l.x, l.y, l.z]);
                }
                if let Some(a) = &twist.angular {
                    values.extend([a.x, a.y, a.z]);
                }
            }
            None => {}
        }
        if gripper.is_none() {
            gripper = target.gripper;
        }
    }
    (values, gripper)
}

/// Permissive action flattening for scripted caller ticks whose fixtures
/// elide channels the strict flattener requires: collect the numeric targets
/// in payload order.
fn collect_action_numbers(value: &Value, out: &mut Vec<f64>, gripper: &mut Option<f64>) {
    let Some(obj) = value.as_object() else { return };
    for key in ["jointPosition", "jointVelocity"] {
        if let Some(values) = obj
            .get(key)
            .and_then(|j| j.get("values"))
            .and_then(Value::as_array)
        {
            out.extend(values.iter().filter_map(Value::as_f64));
        }
    }
    for key in ["eeDelta", "baseTwist"] {
        if let Some(twist) = obj.get(key) {
            for part in ["linear", "angular"] {
                if let Some(v) = twist.get(part) {
                    for axis in ["x", "y", "z"] {
                        out.push(v.get(axis).and_then(Value::as_f64).unwrap_or(0.0));
                    }
                }
            }
        }
    }
    if let Some(pose) = obj.get("eeAbsolute") {
        if let Some(p) = pose.get("position") {
            for axis in ["x", "y", "z"] {
                out.push(p.get(axis).and_then(Value::as_f64).unwrap_or(0.0));
            }
        }
        if let Some(r) = pose.get("rotation") {
            for axis in ["w", "x", "y", "z"] {
                out.push(r.get(axis).and_then(Value::as_f64).unwrap_or(0.0));
            }
        }
    }
    if let Some(parts) = obj
        .get("composite")
        .and_then(|c| c.get("parts"))
        .and_then(Value::as_array)
    {
        for part in parts {
            if let Some(action) = part.get("action") {
                collect_action_numbers(action, out, gripper);
            }
        }
    }
    if let Some(g) = obj
        .get("gripper")
        .and_then(|g| g.get("position"))
        .and_then(Value::as_f64)
    {
        *gripper = Some(g);
    }
}
