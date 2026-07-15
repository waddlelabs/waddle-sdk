//! The session's background pumps: verb outcomes → FSM events, bypass
//! supervision (claimed-while-stalled), media intake, and plane directives.

use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender};
use std::thread::JoinHandle;
use std::time::Duration;

use waddle_controlplane::{ControlPlaneClient, PlaneEvent, ServerMsg};
use waddle_fsm::{GrantChangeDirective, SessionEvent};
use waddle_gate::gate::{GateShared, OwnedAction};
use waddle_gate::jitter::TimedAction;
use waddle_ingest::SessionClock;
use waddle_media::{DataTopic, MediaPlane};
use waddle_types::pb::v0 as pb;
use waddle_types::time::Clock;
use waddle_types::{
    ActionChunk, ActorKind, ClaimId, EpisodeId, GateMode, GrantStatus, MonoNs, Step, VerbRequest,
};

use crate::RuntimeError;
use crate::mirror::Mirror;
use crate::verbs::{VerbDispatch, VerbOutcome};

/// How long the caller's loop may go quiet while claimed before bypass
/// engages (the claimed-while-stalled contract).
pub const STALL_THRESHOLD_NS: i64 = 500_000_000;

/// Verb outcomes → `SessionEvent::VerbResult`.
pub(crate) fn spawn_outcome_pump(
    outcomes: Receiver<VerbOutcome>,
    inject: Sender<SessionEvent>,
    clock: SessionClock,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("waddle-verb-outcomes".into())
        .spawn(move || {
            while let Ok(outcome) = outcomes.recv() {
                let _ = clock.stamp_now();
                let _ = inject.send(SessionEvent::VerbResult {
                    verb: outcome.verb,
                    ok: outcome.result.is_ok(),
                    fault: outcome
                        .result
                        .is_err()
                        .then_some(pb::FaultKind::AdapterError),
                    at: outcome.at,
                });
            }
        })
        .expect("spawn verb-outcome pump")
}

/// Bypass supervision: detects the stalled caller loop (no gate tick within
/// the threshold while claimed), lets the FSM flip to BYPASS, and while in
/// BYPASS drives the declared `send` verb directly from the intervention
/// stream — the integrator's loop is a spectator receiving NOOPs.
pub(crate) fn spawn_bypass_pump(
    gate_shared: Arc<GateShared>,
    mirror: Arc<Mirror>,
    verbs: Arc<VerbDispatch>,
    inject: Sender<SessionEvent>,
    clock: SessionClock,
    dims: usize,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("waddle-bypass-pump".into())
        .spawn(move || {
            loop {
                let status = mirror.read();
                if status.shutdown {
                    return;
                }
                let now = clock.stamp_now().mono_ns();
                let last_tick = gate_shared.stats.last_tick();

                match status.gate_mode {
                    Some(GateMode::Intervention) if status.claim_active => {
                        if let Some(last) = last_tick
                            && now.0 - last.0 > STALL_THRESHOLD_NS
                        {
                            let _ = inject.send(SessionEvent::StallDetected { at: now });
                        }
                    }
                    Some(GateMode::Bypass) => {
                        // Ticks resumed?
                        if let Some(last) = last_tick
                            && now.0 - last.0 <= STALL_THRESHOLD_NS
                        {
                            let _ = inject.send(SessionEvent::TicksResumed { at: now });
                        } else {
                            // Drive due intervention actions straight to send.
                            let due: Option<OwnedAction> = gate_shared.stream.lock().pop_due(now);
                            if let Some(action) = due {
                                let provenance = status
                                    .provenance
                                    .clone()
                                    .unwrap_or_else(waddle_types::ProvenanceTag::policy);
                                let chunk = ActionChunk {
                                    steps: vec![Step {
                                        offset_ns: 0,
                                        values: action.values,
                                        gripper: action.gripper,
                                    }],
                                    dims: if dims > 0 { dims } else { 0 },
                                    horizon_ns: 0,
                                    t_emitted_ns: now.0,
                                    t_obs_ns: now.0,
                                    seq: 0,
                                    source: waddle_types::SourceId::new("bypass-pump"),
                                    provenance,
                                };
                                verbs.request(VerbRequest::Send {
                                    chunk: Arc::new(chunk),
                                });
                            }
                        }
                    }
                    _ => {}
                }
                std::thread::sleep(Duration::from_millis(2));
            }
        })
        .expect("spawn bypass pump")
}

/// Media intake: decode teleop stream packets into the gate's intervention
/// ring; clutch transitions become FSM events (self-initiated claims).
pub(crate) fn spawn_media_intake(
    media: Arc<dyn MediaPlane>,
    mut stream_tx: rtrb::Producer<TimedAction>,
    inject: Sender<SessionEvent>,
    clock: SessionClock,
    mirror: Arc<Mirror>,
) -> Result<JoinHandle<()>, RuntimeError> {
    let pose_rx = media
        .open_data_rx(DataTopic::TeleopPose)
        .map_err(RuntimeError::Media)?;
    let clutch_rx = media
        .open_data_rx(DataTopic::TeleopClutch)
        .map_err(RuntimeError::Media)?;

    let handle = std::thread::Builder::new()
        .name("waddle-media-intake".into())
        .spawn(move || {
            loop {
                let status = mirror.read();
                if status.shutdown {
                    return;
                }
                let mut idle = true;
                if let Ok(Some(packet)) = pose_rx.try_recv_pose() {
                    idle = false;
                    let now = clock.stamp_now().mono_ns();
                    // Bug 1 (stale-backlog replay): the gate only drains the
                    // ring in a mode that consumes it (claim active —
                    // Intervention/Bypass today, a future Reset mode too).
                    // Pushing while unclaimed just stockpiles a backlog that
                    // would all be immediately "due" the instant a claim
                    // engages; drop it at intake instead. The few-ms mirror
                    // lag between clutch-engage and this thread observing
                    // `claim_active` is acceptable (one or two 60 Hz
                    // packets).
                    if status.claim_active
                        && let Some(action) = flatten_packet(&packet)
                    {
                        let _ = stream_tx.push(TimedAction {
                            seq: packet.seq,
                            received: now,
                            action,
                        });
                    }
                    // The clutch state rides every pose packet; edges also
                    // arrive on the reliable topic below.
                }
                if let Some(bytes) = clutch_rx.try_recv() {
                    idle = false;
                    if let Ok(clutch) =
                        <pb::ClutchTransition as prost::Message>::decode(bytes.as_ref())
                    {
                        let _ = inject.send(SessionEvent::Clutch {
                            engaged: clutch.engaged,
                            at: clock.stamp_now().mono_ns(),
                        });
                    }
                }
                if idle {
                    std::thread::sleep(Duration::from_millis(1));
                }
            }
        })
        .expect("spawn media intake");
    Ok(handle)
}

/// Flatten a teleop packet's part targets in order (pose → 7 values wxyz,
/// twist → 6); the first declared gripper rides along. Retargeting into the
/// robot's action space is the closed side's job — this is the raw stream.
fn flatten_packet(packet: &pb::TeleopStreamPacket) -> Option<OwnedAction> {
    let mut values = smallvec::SmallVec::new();
    let mut gripper = None;
    for target in &packet.targets {
        match &target.target {
            Some(pb::part_target::Target::Pose(p)) => {
                let pos = p.position.as_ref()?;
                let rot = p.rotation.as_ref()?;
                values.extend_from_slice(&[pos.x, pos.y, pos.z, rot.w, rot.x, rot.y, rot.z]);
            }
            Some(pb::part_target::Target::Twist(t)) => {
                let lin = t.linear.as_ref()?;
                let ang = t.angular.as_ref()?;
                values.extend_from_slice(&[lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]);
            }
            None => return None,
        }
        if gripper.is_none() {
            gripper = target.gripper;
        }
    }
    (!values.is_empty()).then_some(OwnedAction { values, gripper })
}

/// Plane events → FSM events (claim directives, episode directives,
/// partitions, heartbeat-carried grant changes).
pub(crate) fn spawn_plane_pump(
    plane: Arc<ControlPlaneClient>,
    inject: Sender<SessionEvent>,
    clock: SessionClock,
    mirror: Arc<Mirror>,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("waddle-plane-pump".into())
        .spawn(move || {
            let mut was_connected = true;
            loop {
                if mirror.read().shutdown {
                    return;
                }
                let Some(event) = plane.recv_event_timeout(Duration::from_millis(20)) else {
                    continue;
                };
                let at = clock.stamp_now().mono_ns();
                match event {
                    PlaneEvent::Connected | PlaneEvent::Registered(_) => {
                        if !was_connected {
                            was_connected = true;
                            let _ = inject.send(SessionEvent::PartitionEnd { at });
                        }
                    }
                    PlaneEvent::Disconnected => {
                        if was_connected {
                            was_connected = false;
                            let _ = inject.send(SessionEvent::PartitionStart { at });
                        }
                    }
                    PlaneEvent::BufferOverflowed { .. } => {}
                    PlaneEvent::Server(msg) => forward_server_msg(msg, &inject, at),
                }
            }
        })
        .expect("spawn plane pump")
}

fn forward_server_msg(msg: ServerMsg, inject: &Sender<SessionEvent>, at: MonoNs) {
    match msg {
        ServerMsg::Gate(gate_msg) => match gate_msg.msg {
            Some(pb::gate_server_message::Msg::Claim(directive)) => {
                let Some(claim) = directive.claim else { return };
                let claim_id = ClaimId::new(&claim.claim_id);
                let actor = claim
                    .actor
                    .as_ref()
                    .and_then(|a| ActorKind::from_pb(a.kind).ok())
                    .unwrap_or(ActorKind::Teleoperator);
                match pb::ClaimDirectiveKind::try_from(directive.kind) {
                    Ok(pb::ClaimDirectiveKind::Grant) => {
                        let _ = inject.send(SessionEvent::ClaimGranted {
                            id: claim_id.clone(),
                            source: claim.source_name.clone(),
                            actor,
                            self_initiated: claim.self_initiated,
                            at,
                        });
                        let _ = inject.send(SessionEvent::Engage {
                            claim: claim_id,
                            at,
                        });
                    }
                    Ok(pb::ClaimDirectiveKind::Release) => {
                        let _ = inject.send(SessionEvent::Release {
                            claim: claim_id,
                            at,
                        });
                    }
                    Ok(pb::ClaimDirectiveKind::Retake) => {
                        let successor =
                            EpisodeId::new(format!("ep-{}", uuid::Uuid::new_v4().simple()));
                        let _ = inject.send(SessionEvent::Retake {
                            claim: claim_id,
                            initiator: actor,
                            successor,
                            at,
                        });
                    }
                    _ => {}
                }
            }
            Some(pb::gate_server_message::Msg::Episode(directive)) => {
                let outcome = waddle_types::TerminalOutcome::from_pb(directive.outcome)
                    .unwrap_or(waddle_types::TerminalOutcome::Abort);
                let _ = inject.send(SessionEvent::Terminate {
                    outcome,
                    reason: directive.reason,
                    at,
                });
            }
            _ => {}
        },
        ServerMsg::HeartbeatAck(ack) => {
            let changes: Vec<GrantChangeDirective> = ack
                .grant_changes
                .iter()
                .filter_map(|c| {
                    Some(GrantChangeDirective {
                        verb: waddle_types::Verb::from_pb(c.verb).ok()?,
                        to: match pb::GrantStatus::try_from(c.to) {
                            Ok(pb::GrantStatus::Demoted) => GrantStatus::Demoted,
                            Ok(pb::GrantStatus::Revoked) => GrantStatus::Revoked,
                            _ => GrantStatus::Active,
                        },
                        reason: c.reason.clone(),
                    })
                })
                .collect();
            if !changes.is_empty() {
                let _ = inject.send(SessionEvent::HeartbeatAck {
                    grant_changes: changes,
                    at,
                });
            }
        }
        _ => {}
    }
}
