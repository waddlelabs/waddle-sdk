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
use waddle_ingest::SessionClock;
use waddle_sidecar::{ManifestWriter, McapEpisodeWriter, SidecarBuilder, write_sidecar};
use waddle_types::pb::v0 as pb;
use waddle_types::time::Clock;
use waddle_types::{
    EpisodeId, GateMode, HandoffPolicy, Interp, LeaseId, MonoNs, Provenance, ProvenanceTag,
    VerbRequest,
};

use crate::mirror::Mirror;
use crate::verbs::VerbDispatch;

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
    pub task: parking_lot::Mutex<String>,
    pub robot_description_digest: String,
    pub interp: Interp,

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
        interp: Interp,
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
            task: parking_lot::Mutex::new(String::new()),
            robot_description_digest,
            interp,
            sidecar: None,
            mcap: None,
            manifest,
            armed: Vec::new(),
        }
    }

    /// The reducer loop. Exits on channel close or shutdown event.
    pub fn run(mut self, rx: &Receiver<SessionEvent>, self_tx: &Sender<SessionEvent>) {
        loop {
            if self.mirror.read().shutdown {
                self.finalize_episode_if_terminal(true);
                return;
            }
            // Fire due timers first.
            let now = self.clock.stamp_now().mono_ns();
            let mut due: Vec<(TimerId, MonoNs)> = self
                .armed
                .iter()
                .filter(|(_, d)| *d <= now)
                .copied()
                .collect();
            due.sort_by_key(|(_, d)| *d);
            self.armed.retain(|(_, d)| *d > now);
            for (id, d) in due {
                self.step_and_apply(&SessionEvent::TimerFired { id, at: d }, self_tx);
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
                Ok(event) => self.step_and_apply(&event, self_tx),
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return,
            }
        }
    }

    fn step_and_apply(&mut self, event: &SessionEvent, self_tx: &Sender<SessionEvent>) {
        // Episode bookkeeping BEFORE the step so the sidecar exists when the
        // open event's emissions arrive.
        if let SessionEvent::EpisodeOpen {
            id,
            born_claimed,
            parent,
            ..
        } = event
        {
            self.open_episode_records(id, *born_claimed, parent.as_ref());
        }

        match step(&self.cfg, &self.fsm, event) {
            Err(_rejected) => {
                // Expected outcome for illegal events; recorded nowhere yet.
            }
            Ok(stepped) => {
                self.fsm = stepped.next;
                for effect in stepped.effects {
                    self.apply_effect(effect, self_tx);
                }
                self.publish_mirror();
                self.finalize_episode_if_terminal(false);
            }
        }
    }

    fn apply_effect(&mut self, effect: Effect, self_tx: &Sender<SessionEvent>) {
        match effect {
            Effect::SetGateMode(mode) => {
                let plan = self.plan_for(mode);
                self.gate_shared.store_plan(plan);
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
                let _ = self_tx.send(SessionEvent::LeaseTokenMinted {
                    minted,
                    at: self.clock.stamp_now().mono_ns(),
                });
            }
            Effect::OpenSuccessor {
                predecessor,
                successor,
                mode,
                ..
            } => {
                let _ = self_tx.send(SessionEvent::EpisodeOpen {
                    id: successor,
                    verification: mode,
                    born_claimed: true,
                    parent: Some(predecessor),
                    at: self.clock.stamp_now().mono_ns(),
                });
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
                        interp: self.interp,
                    }),
                    _ => None,
                },
            },
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
        if let Some(parent) = parent {
            builder.set_retake(parent, id);
        }
        self.sidecar = Some(builder);

        self.mcap = self.recording_dir.as_ref().and_then(|dir| {
            McapEpisodeWriter::create(&dir.join(format!("{id}.mcap")), anchor).ok()
        });
    }

    fn finalize_episode_if_terminal(&mut self, force: bool) {
        let terminal_outcome = match self.fsm.episode.as_ref().map(|e| e.phase) {
            Some(Phase::Terminal(outcome)) => Some(outcome),
            _ if force => None,
            _ => return,
        };
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
            s.plane_connected = plane_connected;
        });
    }
}
