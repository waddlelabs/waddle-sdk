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
    ActionSpace, EpisodeId, GateMode, HandoffPolicy, LeaseId, MonoNs, Provenance, ProvenanceTag,
    VerbRequest, unflatten_action,
};

use crate::ack::Injected;
use crate::mirror::Mirror;
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
                // `jitter.rs`'s module doc and `StreamIntake::clear`).
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

    fn claim_provenance(&self) -> ProvenanceTag {
        match &self.fsm.claim {
            None => ProvenanceTag::policy(),
            Some(claim) => ProvenanceTag {
                provenance: match claim.actor {
                    waddle_types::ActorKind::Teleoperator => Provenance::Teleop,
                    waddle_types::ActorKind::Agent => Provenance::Agent,
                    waddle_types::ActorKind::Policy => Provenance::Policy,
                    _ => Provenance::Custom(claim.source.clone()),
                },
                actor: None,
                bypass_approval: claim.self_initiated,
            },
        }
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

    /// Drain `Session::report_proprio` calls onto the reducer's
    /// own latest-known proprio state — merged into every subsequent
    /// gate-tick's recorded `ProprioSample` (`write_record`) and into the
    /// periodic `StreamObservations` uplink (`maybe_uplink_observation`).
    /// Deliberately NOT the `Injected`/`SessionEvent` funnel: a proprio
    /// report carries no FSM guard, so it never touches `step()` — the
    /// hollow-frontend rule is about claim/lease/handoff/timeline logic,
    /// none of which this is.
    fn drain_proprio_reports(&mut self) {
        while let Ok(report) = self.proprio_rx.try_recv() {
            self.latest_extras.merge(&report);
        }
    }

    /// `StreamObservations`: a periodic summary of the reducer's
    /// latest known proprio state, sent whenever a transport is configured.
    /// Buffering/dropping while disconnected is entirely the client's
    /// existing `ClientMsg::buffer_when_offline` classification (unchanged
    /// by this task) — this only decides WHEN to send.
    fn maybe_uplink_observation(&mut self, now: MonoNs) {
        let Some(plane) = &self.plane else { return };
        if self.latest_joint_pos.is_empty() && self.latest_extras.is_empty() {
            return; // nothing observed yet
        }
        if let Some(last) = self.last_obs_uplink_ns
            && now.0 - last < OBSERVATION_UPLINK_PERIOD_NS
        {
            return;
        }
        self.last_obs_uplink_ns = Some(now.0);
        plane.send(ClientMsg::Observation(pb::ObservationUpdate {
            t_ns: now.0,
            payload: Some(pb::observation_update::Payload::Proprio(
                pb::ProprioSample {
                    joint_pos: self.latest_joint_pos.clone(),
                    joint_vel: self.latest_extras.joint_vel.clone(),
                    ee_pose: self.latest_extras.ee_pose.clone(),
                    gripper: self.latest_extras.gripper,
                    part: String::new(),
                },
            )),
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
    /// merged with the latest `Session::report_proprio` extras —
    /// exactly the same per-tick cadence `joint_pos` alone used before this
    /// task; a `report_proprio` call with no further gate tick afterward
    /// still reaches the periodic `StreamObservations` uplink
    /// (`maybe_uplink_observation`), just never gains its own MCAP row (a
    /// tick with no `obs` at all was never recorded either).
    fn write_record(&mut self, rec: &GateRecord) {
        let t_ns = rec.stamp.mono_ns().0;
        let observation = rec.obs.as_ref().map(|obs| pb::ObservationUpdate {
            t_ns,
            payload: Some(pb::observation_update::Payload::Proprio(
                pb::ProprioSample {
                    joint_pos: obs.to_vec(),
                    joint_vel: self.latest_extras.joint_vel.clone(),
                    ee_pose: self.latest_extras.ee_pose.clone(),
                    gripper: self.latest_extras.gripper,
                    part: String::new(),
                },
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
                match unflatten_action(&action.values, action.gripper, &self.space) {
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
        // The episode tail must land in the file before finish().
        self.drain_gate_records();
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
            s.plane_connected = plane_connected;
        });
    }
}
