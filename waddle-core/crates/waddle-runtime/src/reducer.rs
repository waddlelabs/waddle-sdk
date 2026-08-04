//! The reducer: one thread owns the `SessionFsm`, funnels every event source
//! through a single channel, and interprets effects. Single-writer FSM is
//! structural — nothing else ever steps the machine.

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender};
use std::time::Duration;

use waddle_controlplane::{ClientMsg, ControlPlaneClient};
use waddle_fsm::{Effect, Phase, SessionConfig, SessionEvent, SessionFsm, TimerId, step};
use waddle_gate::gate::GateShared;
use waddle_gate::plan::{BlendSchedule, GatePlan, PlanMode};
use waddle_gate::record::{GateDecision, GateRecord};
use waddle_ingest::SessionClock;
use waddle_sidecar::{ManifestWriter, McapEpisodeWriter, SidecarBuilder, write_sidecar};
use waddle_types::pb::v0 as pb;
use waddle_types::time::Clock;
use waddle_types::{
    ActionSpace, EpisodeId, GateMode, HandoffPolicy, LeaseId, MonoNs, ProvenanceTag, VerbRequest,
    unflatten_action,
};

use crate::ack::Injected;
use crate::mirror::Mirror;
use crate::pumps::{BYPASS_PUMP_SOURCE, DispatchedAction};
use crate::session::{ObsSlot, ProprioReport, RecordSlot, ResetSpec, TaskSlot};
use crate::verbs::VerbDispatch;

/// The `StreamObservations` uplink cadence: no dedicated
/// "observation rate" field exists on `RobotDescription` to key off —
/// `series` entries are arbitrary customer-named channels (no canonical
/// "proprio" name), and `action_space.rate_hz` is the CONTROL cadence, not a
/// bandwidth budget for this control-plane RPC (`services.proto`'s header is
/// explicit that "nothing high-bandwidth ever touches these RPCs"; a
/// declared control rate of e.g. 500 Hz would blow straight through that for
/// what is meant to be a low-rate status summary). So this always uses the
/// conservative default rather than risk keying a chatty control-plane send
/// off an unrelated field.
const DEFAULT_OBSERVATION_UPLINK_HZ: f64 = 10.0;
const OBSERVATION_UPLINK_PERIOD_NS: i64 = (1_000_000_000.0 / DEFAULT_OBSERVATION_UPLINK_HZ) as i64;

/// The latest reported proprio extras, maintained by the reducer
/// and merged into every subsequent gate-tick's `ProprioSample` (both the
/// MCAP recording and the `StreamObservations` uplink). See
/// [`crate::session::ProprioReport`] for the per-field patch semantics this
/// mirrors exactly.
#[derive(Clone, Debug, Default)]
struct ProprioExtras {
    joint_vel: Vec<f64>,
    ee_pose: Option<pb::Pose>,
    gripper: Option<f64>,
}

impl ProprioExtras {
    fn merge(&mut self, report: &ProprioReport) {
        if let Some(v) = &report.joint_vel {
            self.joint_vel = v.clone();
        }
        if let Some(pose) = &report.ee_pose {
            self.ee_pose = Some(pose.to_pb());
        }
        if let Some(g) = report.gripper {
            self.gripper = Some(g);
        }
    }

    fn is_empty(&self) -> bool {
        self.joint_vel.is_empty() && self.ee_pose.is_none() && self.gripper.is_none()
    }
}

/// Everything the reducer owns.
pub(crate) struct Reducer {
    pub cfg: SessionConfig,
    pub fsm: SessionFsm,
    pub clock: SessionClock,
    pub gate_shared: Arc<GateShared>,
    pub verbs: Arc<VerbDispatch>,
    pub mirror: Arc<Mirror>,
    pub plane: Option<Arc<ControlPlaneClient>>,
    /// Local recording (None = SidecarOnly-ish, still writes sidecars when
    /// dir set).
    pub recording_dir: Option<PathBuf>,
    pub project: String,
    pub task: TaskSlot,
    pub robot_description_digest: String,
    pub space: ActionSpace,
    /// The session-level `post_reset` default. `Effect::OpenSuccessor`
    /// consults this to open a reducer-opened retake successor with the
    /// same post-reset config as any other episode — successors never go
    /// through `start_episode_with`, so this is their only source of
    /// post-reset config (a predecessor's per-episode override never
    /// reaches here, and must not: see `EpisodeOptions`'s rustdoc).
    post_reset: Option<ResetSpec>,
    /// Fresh gate-record consumers arrive here from `start_episode`.
    record_slot: RecordSlot,
    /// The active episode's gate-record consumer, drained every wake.
    records_rx: Option<rtrb::Consumer<GateRecord>>,
    /// Tripwire `ObsSource` wiring: every ring-drained record
    /// carrying an obs publishes it here, regardless of whether local MCAP
    /// recording is even on.
    obs_slot: ObsSlot,
    /// `Session::report_proprio`'s side channel — drained every
    /// wake, same discipline as `record_slot`/`records_rx`.
    proprio_rx: Receiver<ProprioReport>,
    /// The bypass pump's dispatch side channel: what it drove straight to
    /// `send` without passing through the caller's gate, drained every wake
    /// onto the episode recording (`write_dispatched`).
    dispatch_rx: Receiver<DispatchedAction>,
    /// The latest joint_pos from a ring-drained gate record (what reported
    /// proprio extras merge with), independent of `write_record`'s own
    /// per-tick `obs` — this is what the periodic `StreamObservations`
    /// uplink reads between ticks.
    latest_joint_pos: Vec<f64>,
    /// The latest reported proprio extras, merged into every
    /// subsequent gate-tick's `ProprioSample` and the periodic uplink.
    latest_extras: ProprioExtras,
    /// Last `StreamObservations` send time, for the cadence check.
    last_obs_uplink_ns: Option<i64>,

    // Per-episode state.
    sidecar: Option<SidecarBuilder>,
    mcap: Option<McapEpisodeWriter>,
    manifest: Option<ManifestWriter>,
    armed: Vec<(TimerId, MonoNs)>,
}

impl Reducer {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        cfg: SessionConfig,
        clock: SessionClock,
        gate_shared: Arc<GateShared>,
        verbs: Arc<VerbDispatch>,
        mirror: Arc<Mirror>,
        plane: Option<Arc<ControlPlaneClient>>,
        recording_dir: Option<PathBuf>,
        project: String,
        robot_description_digest: String,
        space: ActionSpace,
        record_slot: RecordSlot,
        task: TaskSlot,
        post_reset: Option<ResetSpec>,
        obs_slot: ObsSlot,
        proprio_rx: Receiver<ProprioReport>,
        dispatch_rx: Receiver<DispatchedAction>,
    ) -> Self {
        let fsm = SessionFsm::new(&cfg);
        let manifest = recording_dir
            .as_ref()
            .and_then(|d| ManifestWriter::open(d).ok());
        Self {
            cfg,
            fsm,
            clock,
            gate_shared,
            verbs,
            mirror,
            plane,
            recording_dir,
            project,
            task,
            robot_description_digest,
            space,
            post_reset,
            record_slot,
            records_rx: None,
            obs_slot,
            proprio_rx,
            dispatch_rx,
            latest_joint_pos: Vec::new(),
            latest_extras: ProprioExtras::default(),
            last_obs_uplink_ns: None,
            sidecar: None,
            mcap: None,
            manifest,
            armed: Vec::new(),
        }
    }

    /// The reducer loop. Exits on channel close or shutdown event.
    pub fn run(mut self, rx: &Receiver<Injected>, self_tx: &Sender<Injected>) {
        loop {
            // Every wake (≤20 ms cadence): drain the gate-record ring onto
            // the episode recording, and any queued `report_proprio` calls
            // onto the reducer's own latest-known state.
            self.drain_gate_records();
            self.drain_dispatched_actions();
            self.drain_proprio_reports();
            if self.mirror.read().shutdown {
                self.finalize_episode_if_terminal(true);
                return;
            }
            // Fire due timers first.
            let now = self.clock.stamp_now().mono_ns();
            self.maybe_uplink_observation(now);
            let mut due: Vec<(TimerId, MonoNs)> = self
                .armed
                .iter()
                .filter(|(_, d)| *d <= now)
                .copied()
                .collect();
            due.sort_by_key(|(_, d)| *d);
            self.armed.retain(|(_, d)| *d > now);
            for (id, d) in due {
                self.step_and_apply(SessionEvent::TimerFired { id, at: d }.into(), self_tx);
            }

            let timeout = self
                .armed
                .iter()
                .map(|(_, d)| (d.0 - now.0).max(0))
                .min()
                .map_or(Duration::from_millis(20), |ns| {
                    #[allow(clippy::cast_sign_loss)]
                    Duration::from_nanos((ns as u64).min(20_000_000))
                });

            match rx.recv_timeout(timeout) {
                Ok(injected) => self.step_and_apply(injected, self_tx),
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return,
            }
        }
    }

    fn step_and_apply(&mut self, injected: Injected, self_tx: &Sender<Injected>) {
        let event = &injected.event;
        // Episode bookkeeping BEFORE the step so the sidecar exists when the
        // open event's emissions arrive.
        if let SessionEvent::EpisodeOpen {
            id,
            born_claimed,
            parent,
            post_reset,
            ..
        } = event
        {
            self.open_episode_records(id, *born_claimed, parent.as_ref(), *post_reset);
        }

        let outcome = match step(&self.cfg, &self.fsm, event) {
            Err(rejected) => {
                // The expected outcome for illegal events. State unchanged;
                // observable to the plane only through the directive ack
                // below (never an EpisodeEvent emission).
                Err(rejected.reason)
            }
            Ok(stepped) => {
                self.fsm = stepped.next;
                for effect in stepped.effects {
                    self.apply_effect(effect, self_tx);
                }
                self.publish_mirror();
                self.finalize_episode_if_terminal(false);
                Ok(())
            }
        };

        // Directive acks (flag `waddle.v0.plane.acks`): the pump attached a
        // group only when the directive carried an id AND the flag was
        // negotiated, so emission needs no further gating here. A directive
        // decoding into two events acks once, when its last event lands.
        if let Some(group) = &injected.ack
            && let Some(fin) = group.record(outcome)
            && let Some(plane) = &self.plane
        {
            plane.send(ClientMsg::Gate(pb::GateClientMessage {
                msg: Some(pb::gate_client_message::Msg::Ack(pb::DirectiveAck {
                    directive_id: fin.directive_id,
                    accepted: fin.accepted,
                    reason: fin.reason,
                })),
            }));
        }
    }

    fn apply_effect(&mut self, effect: Effect, self_tx: &Sender<Injected>) {
        match effect {
            Effect::SetGateMode(mode) => {
                // Not always a mode CHANGE: the FSM also emits this
                // mode-unchanged to re-project a plan whose non-mode inputs
                // moved (E24's agent-episode Noop plan — an invite opening,
                // an agent-invited run closing). The plan is derived from
                // the post-step FSM either way.
                let plan = self.plan_for(mode);
                self.gate_shared.store_plan(plan);
                // The transition back to PASSTHROUGH is the one point every
                // claim/reset-window teardown funnels through (release,
                // ordinary episode end, or `close_reset_window` before it
                // applies the window's result) — Bypass<->Intervention
                // toggling for the SAME live claim never passes through
                // here, so this never discards actions that are still
                // legitimately in flight. Whatever is left in the
                // intervention ring's per-channel pending map at this
                // instant was pushed under the claim/window that just
                // ended; left alone it would sit there until some LATER,
                // unrelated claim/window starts polling the ring and pop it
                // under THAT claimant's mirror provenance (see
                // `jitter.rs`'s module doc and `StreamIntake::clear`). The
                // re-projections above land here too, and the same reasoning
                // holds: an invite opening has an empty ring, and a run
                // closing (retake included — its claim survives, but into a
                // successor that resets the scene first) leaves nothing an
                // intervenor could still legitimately want dispatched.
                if mode == GateMode::Passthrough {
                    self.gate_shared.stream.lock().clear();
                }
            }
            Effect::RequestVerb(verb) => {
                let req = match verb {
                    waddle_types::Verb::Hold => VerbRequest::Hold,
                    waddle_types::Verb::Resume => VerbRequest::Resume,
                    waddle_types::Verb::Home => VerbRequest::Home,
                    waddle_types::Verb::Estop => VerbRequest::Estop,
                    waddle_types::Verb::Send => return, // FSM never requests sends
                };
                self.verbs.request(req);
            }
            Effect::ArmTimer { id, deadline } => {
                self.armed.retain(|(t, _)| *t != id);
                self.armed.push((id, deadline));
            }
            Effect::CancelTimer { id } => self.armed.retain(|(t, _)| *t != id),
            Effect::MintLeaseToken(_) => {
                let minted = LeaseId::new(uuid::Uuid::new_v4().to_string());
                let _ = self_tx.send(
                    SessionEvent::LeaseTokenMinted {
                        minted,
                        at: self.clock.stamp_now().mono_ns(),
                    }
                    .into(),
                );
            }
            Effect::OpenSuccessor {
                predecessor,
                successor,
                mode,
                ..
            } => {
                // Retake successors carry a surviving claim (born-claimed):
                // no remote pre-window opens (born-claimed suppression; `EpisodeOpen`'s
                // `pre_window` arm only opens one when `ctx.s.claim.is_none()`,
                // which a born-claimed episode never satisfies) — the reset
                // pump services PRE the same way it does for any episode with
                // no per-episode override slot, falling back to the
                // session-level default. A declared `Remote` PRE spec for a
                // successor is still a known gap (pending the closed-side
                // retake/hand-reset flow); untouched here.
                //
                // POST is different: nothing suppresses the successor's own
                // POST window or hook at E14 (`enter_post_reset` opens a
                // declared `post_window` unconditionally, with no claim
                // check), so the successor must inherit the SESSION's
                // declared `post_reset` config here — there is no
                // `start_episode_with` call for a reducer-opened successor to
                // have resolved it earlier.
                let post_window = self.post_reset.as_ref().and_then(ResetSpec::window);
                let post_reset_declared = self.post_reset.is_some();
                let _ = self_tx.send(
                    SessionEvent::EpisodeOpen {
                        id: successor,
                        verification: mode,
                        born_claimed: true,
                        parent: Some(predecessor),
                        post_reset: post_reset_declared,
                        pre_window: None,
                        post_window,
                        agent_invite: None,
                        at: self.clock.stamp_now().mono_ns(),
                    }
                    .into(),
                );
            }
            Effect::ReprimePolicy => {
                // The policy is re-primed by the caller's next observation;
                // nothing to do runtime-side yet (policy-server integrations
                // hook here).
            }
            Effect::SetResetUnverified { .. } => {
                if let Some(sc) = &mut self.sidecar {
                    sc.mark_reset_unverified();
                }
            }
            Effect::SetPostResetFailed { .. } => {
                if let Some(sc) = &mut self.sidecar {
                    sc.mark_post_reset_failed();
                }
            }
            Effect::RunPostReset { .. } => {
                // Deliberate no-op: the post-reset hook runs on the reset
                // pump (`pumps::spawn_reset_pump`, mirror-watch — it sees
                // `Phase::PostReset` from the same transition that produced
                // this effect), never on the reducer thread. A user hook
                // here would block the single event funnel for its whole
                // duration — the same reason verbs run on their own
                // dispatch thread.
            }
            Effect::Emit(event) => {
                if let Some(sc) = &mut self.sidecar {
                    sc.push_event((*event).clone());
                }
                if let Some(mcap) = &mut self.mcap {
                    let _ = mcap.write_event(&event);
                }
                if let Some(plane) = &self.plane {
                    plane.send(ClientMsg::Gate(pb::GateClientMessage {
                        msg: Some(pb::gate_client_message::Msg::Event(*event)),
                    }));
                }
            }
        }
    }

    /// Map the FSM's gate mode to a gate plan, carrying the active claim's
    /// provenance and the declared blend schedule.
    fn plan_for(&self, mode: GateMode) -> GatePlan {
        let now = self.clock.stamp_now().mono_ns();
        let provenance = self.claim_provenance();
        let mode = match mode {
            // E24 (flag `waddle.v0.agent`): an agent-invited episode's
            // PASSTHROUGH projects to Noop{AGENT_EPISODE} while no claim is
            // engaged — the caller's own ticks never dispatch; the invited
            // agent drives via the ordinary claim machinery. The predicate
            // lives on the FSM (hollow-frontend rule); this is projection,
            // not policy.
            GateMode::Passthrough if self.fsm.agent_episode_noop() => {
                PlanMode::AgentEpisode { provenance }
            }
            GateMode::Passthrough => PlanMode::Passthrough,
            GateMode::Bypass => PlanMode::Bypass { provenance },
            GateMode::Intervention => PlanMode::Claimed {
                provenance,
                blend: match self.cfg.handoff {
                    HandoffPolicy::Immediate { blend_ns } if blend_ns > 0 => Some(BlendSchedule {
                        start: now,
                        blend_ns,
                        interp: self.space.chunking.interp,
                    }),
                    _ => None,
                },
            },
            // A remote reset window's claimant holds the lease;
            // the caller's own gate() handle is stale and must dispatch
            // nothing (Noop{RESET_ACTIVE}), same shape as Bypass.
            GateMode::Reset => PlanMode::Reset { provenance },
        };
        GatePlan { mode, since: now }
    }

    /// The provenance of every action driven under the active claim — the
    /// claim's own ([`waddle_fsm::ActiveClaim::provenance`]), never
    /// re-derived here. No claim means the policy is driving.
    fn claim_provenance(&self) -> ProvenanceTag {
        self.fsm
            .claim
            .as_ref()
            .map_or_else(ProvenanceTag::policy, waddle_fsm::ActiveClaim::provenance)
    }

    fn open_episode_records(
        &mut self,
        id: &EpisodeId,
        born_claimed: bool,
        parent: Option<&EpisodeId>,
        post_reset_declared: bool,
    ) {
        // Finalize a leftover terminal episode first (retake path).
        self.finalize_episode_if_terminal(true);

        let anchor = self.clock.anchor();
        let mut builder = SidecarBuilder::new(
            id.clone(),
            self.project.clone(),
            self.cfg.session_id.clone(),
            self.cfg.robot_id.clone(),
            self.cfg.cell_id.clone(),
            self.task.lock().clone(),
            anchor,
            if self.recording_dir.is_some() {
                pb::RecordingMode::Local
            } else {
                pb::RecordingMode::SidecarOnly
            },
        );
        builder.open_bounds(self.clock.stamp_now());
        builder.set_born_claimed(born_claimed);
        builder.set_post_reset_declared(post_reset_declared);
        if let Some(parent) = parent {
            builder.set_retake(parent, id);
        }
        self.sidecar = Some(builder);

        self.mcap = self.recording_dir.as_ref().and_then(|dir| {
            McapEpisodeWriter::create(&dir.join(format!("{id}.mcap")), anchor).ok()
        });
    }

    /// Drain gate records onto the episode recording. A fresh consumer in
    /// the slot means a NEW episode's ring (only `start_episode` places
    /// one): whatever is left in the old ring was pushed by a stale handle
    /// after its episode finalized (finalize drains the legitimate tail),
    /// so it is discarded, never written into the new episode's file. A
    /// retake successor keeps the same ring — the caller's loop and its
    /// records carry over. Records arriving after finalize with no new
    /// episode hit `mcap == None` and are discarded (same policy as
    /// events).
    fn drain_gate_records(&mut self) {
        let fresh = self.record_slot.lock().take();
        if let Some(fresh) = fresh {
            if let Some(rx) = &mut self.records_rx {
                while rx.pop().is_ok() {}
            }
            self.records_rx = Some(fresh);
        }
        self.drain_current_ring();
    }

    fn drain_current_ring(&mut self) {
        while let Some(rec) = self.records_rx.as_mut().and_then(|rx| rx.pop().ok()) {
            if let Some(obs) = &rec.obs {
                self.publish_obs(rec.stamp.mono_ns(), obs);
                self.latest_joint_pos = obs.to_vec();
            }
            self.write_record(&rec);
        }
    }

    /// Drain the bypass pump's dispatches onto the episode recording. Same
    /// discipline as the gate-record ring: a dispatch arriving after
    /// finalize, with no episode open, hits `mcap == None` and is discarded.
    fn drain_dispatched_actions(&mut self) {
        while let Ok(dispatched) = self.dispatch_rx.try_recv() {
            self.write_dispatched(&dispatched);
        }
    }

    /// Drain `Session::report_proprio` calls onto the reducer's
    /// own latest-known proprio state — merged into every subsequent
    /// gate-tick's recorded `ProprioSample` (`write_record`) and into the
    /// periodic `StreamObservations` uplink (`maybe_uplink_observation`).
    /// Deliberately NOT the `Injected`/`SessionEvent` funnel: a proprio
    /// report carries no FSM guard, so it never touches `step()` — the
    /// hollow-frontend rule is about claim/lease/handoff/timeline logic,
    /// none of which this is.
    ///
    /// Called every wake AND from `finalize_episode_if_terminal`: a report
    /// still queued when the episode goes terminal is part of that episode,
    /// so it must be written before the MCAP closes rather than surfacing on
    /// a later wake with nowhere to go.
    fn drain_proprio_reports(&mut self) {
        while let Ok(report) = self.proprio_rx.try_recv() {
            self.latest_extras.merge(&report);
            // A reported sample IS an observation, so it lands on
            // `/waddle/observations` in its own right — stamped here, by the
            // session clock, at the moment the reducer learned it. Whether
            // the caller ALSO passes obs to `gate()` cannot decide whether
            // an observation is recorded: an agent-invited episode has a
            // caller that never ticks at all (FSM.md E24), and its
            // recording was coming out with zero observations. `joint_pos`
            // rides the latest known one (the same field the periodic
            // uplink carries), since `report_proprio` has no joint_pos of
            // its own.
            let stamp = self.clock.stamp_now();
            let sample = self.latest_proprio_sample(self.latest_joint_pos.clone());
            if let Some(mcap) = &mut self.mcap {
                let _ = mcap.write_observation(&pb::ObservationUpdate {
                    t_ns: stamp.mono_ns().0,
                    payload: Some(pb::observation_update::Payload::Proprio(sample)),
                });
            }
        }
    }

    /// The reducer's latest known proprioceptive state, over the caller's
    /// `joint_pos` of the moment. THE `ProprioSample` builder: the periodic
    /// uplink, a gate tick's recorded observation, and a `report_proprio`
    /// call's own recorded observation all differ only in which `joint_pos`
    /// they hand it and where the result goes.
    fn latest_proprio_sample(&self, joint_pos: Vec<f64>) -> pb::ProprioSample {
        pb::ProprioSample {
            joint_pos,
            joint_vel: self.latest_extras.joint_vel.clone(),
            ee_pose: self.latest_extras.ee_pose.clone(),
            gripper: self.latest_extras.gripper,
            part: String::new(),
        }
    }

    /// One bypass-pump dispatch → an `/waddle/actions` row. The pump is the
    /// point where an intervenor's action actually reaches the robot without
    /// passing through the caller's `gate()`, so without this an
    /// agent-driven episode — whose caller never ticks — recorded no actions
    /// at all, leaving a recording that cannot be judged or trained on. Its
    /// own `source_id`/seq space (`BYPASS_PUMP_SOURCE`), because
    /// `ActionChunk.seq` is monotone per stream and the caller's gate is a
    /// different stream into the same episode.
    fn write_dispatched(&mut self, dispatched: &DispatchedAction) {
        let action = match unflatten_action(
            &dispatched.action.values,
            dispatched.action.gripper,
            None,
            &self.space,
        ) {
            Ok(action) => vec![action],
            // Same contract as a gate tick's: an action that does not fit
            // the declared space (a raw teleop stream ahead of closed-side
            // retargeting) still gets its row, with no decodable action,
            // rather than vanishing from the trace.
            Err(_) => Vec::new(),
        };
        let t_ns = dispatched.stamp.mono_ns().0;
        if let Some(mcap) = &mut self.mcap {
            let _ = mcap.write_action(&pb::ActionChunk {
                actions: action,
                horizon_ns: 0,
                t_emitted_ns: t_ns,
                // The pump dispatches from the intervention ring, not from
                // an observation the caller handed it.
                t_obs_ns: 0,
                seq: dispatched.seq,
                source_id: BYPASS_PUMP_SOURCE.into(),
                provenance: Some(dispatched.provenance.to_pb()),
            });
        }
    }

    /// `StreamObservations`: a periodic summary of the reducer's
    /// latest known proprio state, sent whenever a transport is configured.
    /// Buffering/dropping while disconnected is entirely the client's
    /// existing `ClientMsg::buffer_when_offline` classification (unchanged
    /// by this task) — this only decides WHEN to send.
    fn maybe_uplink_observation(&mut self, now: MonoNs) {
        if self.plane.is_none() {
            return;
        }
        if self.latest_joint_pos.is_empty() && self.latest_extras.is_empty() {
            return; // nothing observed yet
        }
        if let Some(last) = self.last_obs_uplink_ns
            && now.0 - last < OBSERVATION_UPLINK_PERIOD_NS
        {
            return;
        }
        self.last_obs_uplink_ns = Some(now.0);
        let sample = self.latest_proprio_sample(self.latest_joint_pos.clone());
        let Some(plane) = &self.plane else { return };
        plane.send(ClientMsg::Observation(pb::ObservationUpdate {
            t_ns: now.0,
            payload: Some(pb::observation_update::Payload::Proprio(sample)),
        }));
    }

    /// Tripwire `ObsSource` wiring: every gate tick's `obs` (the
    /// customer's `gate(obs=...)` argument) becomes the latest snapshot a
    /// declared tripwire evaluates — this runs unconditionally per drained
    /// record, before `write_record`'s own (local-recording-only) early
    /// return, so tripwires evaluate real obs even when `recording_dir` is
    /// unset. The flat customer vector maps onto `ObsSnapshot::joint_pos`
    /// verbatim; `ee_pos`/`force_n` stay `None` (this seam carries a flat
    /// vector, not semantically-tagged fields). Runs on the reducer thread,
    /// never `Gate::gate()`'s fast path.
    fn publish_obs(&self, at: MonoNs, obs: &waddle_types::ObsValues) {
        self.obs_slot.publish(waddle_tripwire::ObsSnapshot {
            at,
            joint_pos: obs.iter().copied().collect(),
            ee_pos: None,
            force_n: None,
        });
    }

    /// One gate record → the episode MCAP, via the canonical wire messages:
    /// the obs (when present) as an `ObservationUpdate` on
    /// `/waddle/observations`, and the decision as a single-step
    /// `ActionChunk` on `/waddle/actions`. Noop and Hold write `NoopMarker`
    /// actions rather than being skipped, so `/waddle/actions` is the
    /// complete per-tick trace (provenance spans, bypass windows, holds).
    ///
    /// The `ProprioSample` carries `joint_pos` from this tick's own `obs`
    /// merged with the latest `Session::report_proprio` extras. A
    /// `report_proprio` call gets its OWN row too
    /// (`drain_proprio_reports`), so a caller who reports proprio and never
    /// passes `obs` to `gate()` still has its proprioception recorded; a
    /// caller who does both records both, each stamped when it happened.
    fn write_record(&mut self, rec: &GateRecord) {
        let t_ns = rec.stamp.mono_ns().0;
        let observation = rec.obs.as_ref().map(|obs| pb::ObservationUpdate {
            t_ns,
            payload: Some(pb::observation_update::Payload::Proprio(
                self.latest_proprio_sample(obs.to_vec()),
            )),
        });

        let Some(mcap) = &mut self.mcap else { return };

        if let Some(update) = &observation {
            let _ = mcap.write_observation(update);
        }

        let noop = |reason: pb::NoopReason| pb::Action {
            target: Some(pb::action::Target::Noop(pb::NoopMarker {
                reason: reason as i32,
            })),
            ..Default::default()
        };
        let actions = match (rec.decision, &rec.action) {
            (GateDecision::Pass | GateDecision::Substitute | GateDecision::Blend, Some(action)) => {
                match unflatten_action(&action.values, action.gripper, None, &self.space) {
                    Ok(action) => vec![action],
                    // An action left the gate but does not fit the declared
                    // space (e.g. a raw teleop stream ahead of closed-side
                    // retargeting). Write the chunk with no decodable action
                    // rather than skipping the tick, so /waddle/actions stays
                    // a complete, obs-aligned per-tick trace.
                    Err(_) => Vec::new(),
                }
            }
            (GateDecision::Noop, _) => vec![noop(pb::NoopReason::BypassActive)],
            (GateDecision::Hold, _) => vec![noop(pb::NoopReason::HoldActive)],
            (GateDecision::ResetActive, _) => vec![noop(pb::NoopReason::ResetActive)],
            (GateDecision::AgentEpisode, _) => vec![noop(pb::NoopReason::AgentEpisode)],
            // Pass/Substitute/Blend always carry an action.
            (_, None) => return,
        };
        let _ = mcap.write_action(&pb::ActionChunk {
            actions,
            horizon_ns: 0,
            t_emitted_ns: t_ns,
            t_obs_ns: if rec.obs.is_some() { t_ns } else { 0 },
            seq: rec.seq,
            source_id: "waddle.gate".into(),
            provenance: Some(rec.provenance.to_pb()),
        });
    }

    fn finalize_episode_if_terminal(&mut self, force: bool) {
        let terminal_outcome = match self.fsm.episode.as_ref().map(|e| e.phase) {
            Some(Phase::Terminal(outcome)) => Some(outcome),
            _ if force => None,
            _ => return,
        };
        // The episode tail must land in the file before finish(). EVERY
        // side channel that writes into the episode, in the same order the
        // reducer loop drains them: whatever a caller handed us before the
        // episode went terminal belongs in that episode's file, and after
        // `self.mcap` is taken below there is nowhere left to put it.
        self.drain_gate_records();
        self.drain_dispatched_actions();
        self.drain_proprio_reports();
        let Some(mut builder) = self.sidecar.take() else {
            return;
        };
        if let Some(outcome) = terminal_outcome {
            builder.set_outcome(outcome, "");
        } else if !force {
            return;
        }
        builder.close_bounds(self.clock.stamp_now());
        if let Ok(sidecar) = builder.finish(&self.robot_description_digest)
            && let Some(dir) = &self.recording_dir
            && let Ok(path) = write_sidecar(dir, &sidecar)
            && let Some(manifest) = &mut self.manifest
        {
            let _ = manifest.append(&sidecar, &path);
        }
        if let Some(mcap) = self.mcap.take() {
            let _ = mcap.finish();
        }
    }

    fn publish_mirror(&self) {
        let episode_id = self.fsm.episode.as_ref().map(|e| e.id.clone());
        let episode_state = self.fsm.episode.as_ref().map(|e| e.phase);
        let outcome = match episode_state {
            Some(Phase::Terminal(o)) => Some(o),
            _ => None,
        };
        let pinned_outcome = self.fsm.episode.as_ref().and_then(|e| e.pinned_outcome);
        let post_reset_failed = self
            .fsm
            .episode
            .as_ref()
            .is_some_and(|e| e.post_reset_failed);
        // Agent-invited progress (flag `waddle.v0.agent`): published so a
        // caller blocked in `Session::run_agent` (mirror-watch, like the
        // reset waits) can observe invited → engaged → terminal.
        let agent_invited = self.fsm.episode.as_ref().is_some_and(|e| e.agent_invited);
        let agent_engaged = self.fsm.episode.as_ref().is_some_and(|e| e.agent_engaged);
        let agent_invite_aborted = self.fsm.episode.as_ref().is_some_and(|e| e.invite_aborted);
        let gate_mode = Some(self.fsm.gate_mode);
        let claim_active = self.fsm.claim.is_some();
        let provenance = claim_active.then(|| self.claim_provenance());
        let plane_connected = self.fsm.plane_connected;
        self.mirror.update(|s| {
            s.episode_id = episode_id;
            s.episode_state = episode_state;
            s.gate_mode = gate_mode;
            s.claim_active = claim_active;
            s.provenance = provenance;
            s.outcome = outcome;
            s.pinned_outcome = pinned_outcome;
            s.post_reset_failed = post_reset_failed;
            s.agent_invited = agent_invited;
            s.agent_engaged = agent_engaged;
            s.agent_invite_aborted = agent_invite_aborted;
            s.plane_connected = plane_connected;
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::mirror::Mirror;
    use crate::verbs::{ControlRegistry, VerbDispatch};
    use prost::Message as _;
    use waddle_types::{LeaseEnforcement, ReplanPolicy, RobotDescription};

    /// The episode TAIL, pinned where it is deterministic: `finalize` is
    /// called directly, so no reducer wake can drain the channel first and
    /// hide the miss (in a live session the ≤20 ms wake cadence usually
    /// drains it, which is exactly what makes an integration test of this
    /// unable to fail reliably — and what let the hole ship).
    ///
    /// A `report_proprio` still queued when the episode goes terminal is
    /// part of THAT episode: gate records and bypass-pump dispatches are
    /// tail-drained for the same reason, and a report is an observation
    /// like any other. Without the drain, it surfaces on a later wake with
    /// `mcap == None` and is discarded — or, worse, lands in whatever
    /// episode opened next.
    #[test]
    fn finalize_writes_reports_still_queued_at_the_episode_tail() {
        let dir = tempfile::tempdir().unwrap();
        let clock = SessionClock::capture();
        let robot = RobotDescription::try_from(&pb::RobotDescription {
            name: "tail-bot".into(),
            robot_id: "tail-01".into(),
            cell_id: "cell-tail".into(),
            action_space: Some(pb::ActionSpace {
                space: Some(pb::action_space::Space::JointPosition(pb::JointPosition {
                    joints: (0..3)
                        .map(|i| pb::JointDescriptor {
                            name: format!("j{i}"),
                            ..Default::default()
                        })
                        .collect(),
                })),
                rate_hz: 50.0,
                chunking: None,
                gripper: None,
            }),
            ..Default::default()
        })
        .unwrap();

        let (gate_shared, _stream_tx) = GateShared::new(
            GatePlan::passthrough(MonoNs(0)),
            8,
            0,
            ReplanPolicy::Immediate,
        );
        let (outcome_tx, _outcome_rx) = std::sync::mpsc::channel();
        let verbs = Arc::new(VerbDispatch::spawn(
            ControlRegistry::default(),
            clock.clone(),
            outcome_tx,
        ));
        let (proprio_tx, proprio_rx) = std::sync::mpsc::channel();
        let (_dispatch_tx, dispatch_rx) = std::sync::mpsc::channel();

        let mut reducer = Reducer::new(
            SessionConfig::minimal("loop", HandoffPolicy::HoldFirst, LeaseEnforcement::Advisory),
            clock,
            gate_shared,
            verbs,
            Mirror::new(),
            None,
            Some(dir.path().to_path_buf()),
            "tail-project".to_owned(),
            "digest".to_owned(),
            robot.action_space,
            Arc::new(parking_lot::Mutex::new(None)),
            Arc::new(parking_lot::Mutex::new("task".to_owned())),
            None,
            Arc::new(waddle_ingest::LatestSlot::new()),
            proprio_rx,
            dispatch_rx,
        );

        let id = EpisodeId::new("ep-tail");
        reducer.open_episode_records(&id, false, None, false);
        // Queued, never drained by a wake: this IS the tail.
        proprio_tx
            .send(ProprioReport {
                joint_vel: Some(vec![7.0, 8.0, 9.0]),
                ee_pose: None,
                gripper: Some(0.25),
            })
            .unwrap();
        reducer.finalize_episode_if_terminal(true);

        let buf = std::fs::read(dir.path().join(format!("{id}.mcap"))).unwrap();
        let mut samples = Vec::new();
        for message in mcap::MessageStream::new(&buf).unwrap() {
            let message = message.unwrap();
            if message.channel.topic == waddle_sidecar::mcaprec::OBSERVATIONS_TOPIC {
                let update = pb::ObservationUpdate::decode(message.data.as_ref()).unwrap();
                if let Some(pb::observation_update::Payload::Proprio(p)) = update.payload {
                    samples.push(p);
                }
            }
        }
        assert_eq!(
            samples.len(),
            1,
            "a report queued when the episode ended must be in that episode's recording"
        );
        assert_eq!(samples[0].joint_vel, vec![7.0, 8.0, 9.0]);
        assert_eq!(samples[0].gripper, Some(0.25));
    }
}
