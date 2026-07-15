//! The session's background pumps: verb outcomes → FSM events, bypass
//! supervision (claimed-while-stalled), media intake, and plane directives.

use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender};
use std::thread::JoinHandle;
use std::time::Duration;

use waddle_controlplane::{ControlPlaneClient, PlaneEvent, ServerMsg};
use waddle_fsm::{GrantChangeDirective, Phase, SessionEvent};
use waddle_gate::gate::{GateShared, OwnedAction};
use waddle_gate::jitter::{StreamChannel, TimedAction};
use waddle_ingest::SessionClock;
use waddle_media::{DataTopic, MediaPlane};
use waddle_types::pb::v0 as pb;
use waddle_types::time::Clock;
use waddle_types::{
    ActionChunk, ActionSpace, ActorKind, ClaimId, EpisodeId, GateMode, GrantStatus, MonoNs, Step,
    VerbRequest,
};

use crate::RuntimeError;
use crate::mirror::Mirror;
use crate::session::{
    EpisodeResetSpecs, ResetOwnerSlot, ResetSpec, ResetSpecSlot, StreamProducer, TaskSlot,
};
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

/// Pop one due intervention action off the gate's stream ring (if any) and
/// dispatch it straight to `send`, tagged with the mirror's claim
/// provenance — the shared mechanics behind both the BYPASS arm
/// (claimed-while-stalled) and the RESET arm (remote reset-window
/// actuation) of [`spawn_bypass_pump`]: same chunk shape, same source id.
/// Neither caller re-derives legality here — that's the FSM's job, encoded
/// entirely in which `GateMode` the mirror shows.
fn dispatch_due_intervention(
    gate_shared: &GateShared,
    status: &crate::mirror::Status,
    verbs: &VerbDispatch,
    dims: usize,
    now: MonoNs,
) {
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

/// Bypass supervision: detects the stalled caller loop (no gate tick within
/// the threshold while claimed), lets the FSM flip to BYPASS, and while in
/// BYPASS drives the declared `send` verb directly from the intervention
/// stream — the integrator's loop is a spectator receiving NOOPs.
///
/// Also the reset-window actuation site (FSM.md E20/E21, flag
/// `waddle.v0.reset.remote`): while the mirror shows `GateMode::Reset` with
/// an active claim, the engaged reset claimant's actions (teleop via media
/// intake, agent chunks via `forward_server_msg`) land in the SAME
/// intervention ring and get driven to `send` here too — identical
/// mechanics, no stall detection (a reset window has no "ticks resumed"
/// recovery path; the window's own timeout is the FSM's, not this pump's).
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
                            dispatch_due_intervention(&gate_shared, &status, &verbs, dims, now);
                        }
                    }
                    Some(GateMode::Reset) if status.claim_active => {
                        dispatch_due_intervention(&gate_shared, &status, &verbs, dims, now);
                    }
                    _ => {}
                }
                std::thread::sleep(Duration::from_millis(2));
            }
        })
        .expect("spawn bypass pump")
}

/// The effective reset spec for episode `id`, for one phase (`pick` selects
/// pre or post): the per-episode resolved entry when `start_episode_with`
/// published one for this id, else the session-level default (the only case
/// is a reducer-opened retake successor, which has no per-episode entry).
fn effective_spec(
    slot: &ResetSpecSlot,
    id: &EpisodeId,
    session_default: &Option<ResetSpec>,
    pick: impl Fn(&EpisodeResetSpecs) -> Option<ResetSpec>,
) -> Option<ResetSpec> {
    let guard = slot.lock();
    match &*guard {
        Some(specs) if &specs.id == id => pick(specs),
        _ => session_default.clone(),
    }
}

/// The reset pump: the single scripted-hook invocation site (mirror-watch,
/// like the bypass pump). Two arms, both keyed per episode id so the
/// "already serviced" bookkeeping resets across episodes:
///
/// - A LIVE episode in RESETTING that nobody is driving inline
///   (`inline_reset_owner`) gets the effective PRE hook run here and its
///   `ResetResult` injected. This is what fixes the reducer-opened
///   retake-successor gap: plane directives / marks / judge results
///   terminate with no blocked caller, so a caller-thread-only invocation
///   site left successors hanging in RESETTING forever.
/// - A LIVE episode in POST_RESET with an effective POST spec of `Hook`
///   gets that hook run here and its `PostResetResult` injected (E15/E16).
///
/// `ResetSpec::Remote` phases are none of the pump's business — the FSM's
/// window machinery (E19–E22, including the window timeout) owns them.
///
/// Hooks run OFF the caller thread here, so they must be `Send + Sync`
/// (the `ResetHook` type already requires it) and must return: session
/// shutdown joins this thread.
#[allow(clippy::too_many_arguments)]
pub(crate) fn spawn_reset_pump(
    mirror: Arc<Mirror>,
    inject: Sender<SessionEvent>,
    clock: SessionClock,
    task: TaskSlot,
    inline_owner: ResetOwnerSlot,
    episode_specs: ResetSpecSlot,
    session_pre: Option<ResetSpec>,
    session_post: Option<ResetSpec>,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("waddle-reset-hooks".into())
        .spawn(move || {
            let mut serviced_pre: Option<EpisodeId> = None;
            let mut serviced_post: Option<EpisodeId> = None;
            loop {
                let status = mirror.read();
                if status.shutdown {
                    return;
                }
                match (status.episode_state, status.episode_id) {
                    (Some(Phase::Resetting), Some(id)) if serviced_pre.as_ref() != Some(&id) => {
                        let pre =
                            effective_spec(&episode_specs, &id, &session_pre, |s| s.pre.clone());
                        if matches!(pre, Some(ResetSpec::Remote { .. })) {
                            // The window machinery owns a remote pre-reset;
                            // injecting a pipeline result here would be
                            // rejected anyway (E19b).
                            serviced_pre = Some(id);
                        } else if inline_owner.lock().as_ref() != Some(&id) {
                            // `Hook` or the no-spec default, and no
                            // `start_episode_with` call owns it inline.
                            // Re-check the mirror AFTER the owner read: the
                            // owner is set before the mirror can ever show
                            // this id and cleared only after the mirror
                            // showed it leaving RESETTING, so an id that is
                            // (a) un-owned at the read above and (b) still
                            // RESETTING now was never inline-owned at all —
                            // without the re-check, a stale RESETTING
                            // snapshot from before an inline reset finished
                            // could double-run its hook.
                            let recheck = mirror.read();
                            if recheck.episode_id.as_ref() == Some(&id)
                                && matches!(recheck.episode_state, Some(Phase::Resetting))
                            {
                                serviced_pre = Some(id);
                                let (ok, verified) = match &pre {
                                    Some(ResetSpec::Hook(hook)) => {
                                        let task = task.lock().clone();
                                        hook(&task)
                                    }
                                    // No spec: the placeholder default — a
                                    // scene reset by hand (same default the
                                    // inline path injects).
                                    _ => (true, Some(true)),
                                };
                                let _ = inject.send(SessionEvent::ResetResult {
                                    ok,
                                    verified,
                                    at: clock.stamp_now().mono_ns(),
                                });
                            }
                        }
                    }
                    (Some(Phase::PostReset), Some(id)) if serviced_post.as_ref() != Some(&id) => {
                        serviced_post = Some(id.clone());
                        let post =
                            effective_spec(&episode_specs, &id, &session_post, |s| s.post.clone());
                        match post {
                            Some(ResetSpec::Hook(hook)) => {
                                let task = task.lock().clone();
                                let (ok, _verified) = hook(&task);
                                let _ = inject.send(SessionEvent::PostResetResult {
                                    ok,
                                    detail: String::new(),
                                    at: clock.stamp_now().mono_ns(),
                                });
                            }
                            Some(ResetSpec::Remote { .. }) | None => {
                                // Remote: the window machinery owns it
                                // (E19–E22). None is unreachable — POST_RESET
                                // requires `post_reset_declared`, which is
                                // only ever stamped from a resolved
                                // `Some(spec)` — but if config were somehow
                                // lost, leaving the phase to the FSM (which
                                // still honors estop and directives) beats
                                // fabricating a hook result.
                            }
                        }
                    }
                    _ => {}
                }
                std::thread::sleep(Duration::from_millis(2));
            }
        })
        .expect("spawn reset pump")
}

/// Media intake: decode teleop stream packets into the gate's intervention
/// ring; clutch transitions become FSM events (self-initiated claims).
///
/// `expected_dims` is the session's declared action-space width (the same
/// source of truth `spawn_bypass_pump` uses for `ActionChunk.dims`); `None`
/// means the declared space has no fixed width to validate against (e.g. an
/// Opaque space without a declared dim), in which case flattened actions
/// pass through unchecked, same as before Bug 2's fix. `gripper_spec` is the
/// session's declared `GripperSpec` (Bug 3): the raw teleop gripper command
/// (normalized 0..1, 1 = open — the media-plane convention) is mapped
/// through it before the action reaches the ring; `None` passes it through
/// unchanged.
///
/// `stream_tx` is shared (`StreamProducer`, not an owned `rtrb::Producer`):
/// the same intervention ring also takes agent-chunk pushes from
/// `forward_server_msg` during a reset window (`rtrb` is strictly SPSC, so
/// the one real producer is Mutex-shared rather than duplicated — mirrors
/// how `GateShared.stream`'s consumer side is already shared between the
/// caller thread and the bypass pump). Every `TimedAction` pushed here is
/// tagged `StreamChannel::Teleop`, so this producer's seq space (the wire
/// `TeleopStreamPacket.seq`) never shares a reorder/late-drop cursor with
/// the agent-chunk producer's (`JitterBuffer` keeps one per channel).
pub(crate) fn spawn_media_intake(
    media: Arc<dyn MediaPlane>,
    stream_tx: StreamProducer,
    inject: Sender<SessionEvent>,
    clock: SessionClock,
    mirror: Arc<Mirror>,
    expected_dims: Option<usize>,
    gripper_spec: Option<waddle_types::GripperKind>,
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
            // Bug 2 (action-space validation): a fault fires at most once
            // per claim window, not once per mismatched packet at
            // 60-90 Hz. Reset the guard the instant the claim ends so the
            // next claim window gets its own chance to fault.
            let mut validation_fault_sent = false;
            loop {
                let status = mirror.read();
                if status.shutdown {
                    return;
                }
                if !status.claim_active {
                    validation_fault_sent = false;
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
                        && let Some(mut action) = flatten_packet(&packet)
                    {
                        let dims_ok = match expected_dims {
                            Some(want) => action.values.len() == want,
                            None => true,
                        };
                        if dims_ok {
                            // Bug 3 (GripperSpec never applied): map the raw
                            // teleop gripper through the declared spec
                            // before this reaches the ring.
                            if let Some(g) = action.gripper {
                                action.gripper = Some(match &gripper_spec {
                                    Some(spec) => spec.map_normalized(g),
                                    None => g,
                                });
                            }
                            let _ = stream_tx.lock().push(TimedAction {
                                channel: StreamChannel::Teleop,
                                seq: packet.seq,
                                received: now,
                                action,
                            });
                        } else if !validation_fault_sent {
                            validation_fault_sent = true;
                            let _ = inject.send(SessionEvent::InterventionRejected {
                                dims_got: action.values.len(),
                                dims_want: expected_dims.unwrap_or(0),
                                at: now,
                            });
                        }
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
/// partitions, heartbeat-carried grant changes, reset-window directives,
/// reset-window agent-chunk actuation).
pub(crate) fn spawn_plane_pump(
    plane: Arc<ControlPlaneClient>,
    inject: Sender<SessionEvent>,
    clock: SessionClock,
    mirror: Arc<Mirror>,
    stream: StreamProducer,
    action_space: Arc<ActionSpace>,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("waddle-plane-pump".into())
        .spawn(move || {
            let mut was_connected = true;
            // Monotonic ring-seq counter for agent-chunk steps pushed by the
            // `InterventionChunk` arm below — see that arm's doc comment for
            // why this can't just reuse `chunk.seq`.
            let mut next_chunk_seq: u64 = 0;
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
                    PlaneEvent::Server(msg) => forward_server_msg(
                        msg,
                        &inject,
                        at,
                        &mirror,
                        &stream,
                        &action_space,
                        &mut next_chunk_seq,
                    ),
                }
            }
        })
        .expect("spawn plane pump")
}

#[allow(clippy::too_many_arguments)]
fn forward_server_msg(
    msg: ServerMsg,
    inject: &Sender<SessionEvent>,
    at: MonoNs,
    mirror: &Mirror,
    stream: &StreamProducer,
    space: &ActionSpace,
    next_chunk_seq: &mut u64,
) {
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
            // Remote reset windows (flag `waddle.v0.reset.remote`): the
            // claim carried on the directive is who performs the reset —
            // populated on every kind, the same convention `ClaimDirective`
            // already uses for Grant/Release/Retake — so ENGAGE, COMPLETE,
            // and CANCEL all identify their window's claim from it.
            Some(pb::gate_server_message::Msg::ResetWindow(directive)) => {
                let Some(claim) = directive.claim else { return };
                let claim_id = ClaimId::new(&claim.claim_id);
                match pb::ResetWindowDirectiveKind::try_from(directive.kind) {
                    Ok(pb::ResetWindowDirectiveKind::Engage) => {
                        // C6 admission and the "one open window" check are
                        // the FSM's; this just relays the plane's directive
                        // as the same two events a local claim/engage would
                        // produce (Task 5's `ClaimGranted` then
                        // `ResetWindowEngage`, in that order).
                        let actor = claim
                            .actor
                            .as_ref()
                            .and_then(|a| ActorKind::from_pb(a.kind).ok())
                            .unwrap_or(ActorKind::Teleoperator);
                        let _ = inject.send(SessionEvent::ClaimGranted {
                            id: claim_id.clone(),
                            source: claim.source_name.clone(),
                            actor,
                            self_initiated: claim.self_initiated,
                            at,
                        });
                        let _ = inject.send(SessionEvent::ResetWindowEngage {
                            claim: claim_id,
                            at,
                        });
                    }
                    Ok(pb::ResetWindowDirectiveKind::Complete) => {
                        let Some(result) = directive.result else {
                            return;
                        };
                        let verified = result.verification.as_ref().map(|v| v.verified);
                        let _ = inject.send(SessionEvent::ResetWindowComplete {
                            claim: claim_id,
                            ok: result.ok,
                            verified,
                            at,
                        });
                    }
                    Ok(pb::ResetWindowDirectiveKind::Cancel) => {
                        // No dedicated FSM event for a plane-initiated
                        // cancel: it is observably a failed completion
                        // (E21 with ok=false) from the session's point of
                        // view — the same event COMPLETE{ok:false} uses.
                        let _ = inject.send(SessionEvent::ResetWindowComplete {
                            claim: claim_id,
                            ok: false,
                            verified: None,
                            at,
                        });
                    }
                    _ => {}
                }
            }
            // Agent-chunk reset actuation ONLY (flag `waddle.v0.reset.remote`):
            // while the mirror shows `GateMode::Reset`, an engaged AGENT
            // claimant's chunk steps join the same intervention ring the
            // bypass pump's RESET arm drains, timed off this arrival (`at`
            // + each step's declared offset). The general Claimed-mode
            // chunk intake (jitter horizon, ReplanPolicy) is a later
            // milestone — every other gate mode still drops this arm
            // silently, unchanged from before this task.
            //
            // Ring seq: `next_chunk_seq` is a fresh per-pump counter, not
            // `chunk.seq` — the jitter buffer's reorder map is keyed by seq
            // *per step*, and a chunk's own `seq` is one value for the
            // whole chunk (every step in it would collide on the same key).
            // Tagged `StreamChannel::AgentChunk` so this counter's seq space
            // never shares a reorder/late-drop cursor with the media
            // intake's teleop-packet seq space, even though both producers
            // push into the same physical ring (`JitterBuffer` keeps one
            // cursor per channel — see `jitter.rs`'s module doc): an earlier
            // teleop claim (ordinary Intervention/Bypass, or an earlier
            // teleop-driven reset window) advancing its own cursor high must
            // never cause THIS window's first agent-chunk step to look
            // "late" and be dropped.
            Some(pb::gate_server_message::Msg::InterventionChunk(chunk)) => {
                if !matches!(mirror.read().gate_mode, Some(GateMode::Reset)) {
                    return;
                }
                match ActionChunk::from_pb(&chunk, space) {
                    Ok(action_chunk) => {
                        let mut producer = stream.lock();
                        for step in &action_chunk.steps {
                            *next_chunk_seq += 1;
                            let _ = producer.push(TimedAction {
                                channel: StreamChannel::AgentChunk,
                                seq: *next_chunk_seq,
                                received: MonoNs(at.0.saturating_add(step.offset_ns)),
                                action: OwnedAction {
                                    values: step.values.clone(),
                                    gripper: step.gripper,
                                },
                            });
                        }
                    }
                    Err(err) => {
                        // Unlike the media-intake teleop path (which raises
                        // `SessionEvent::InterventionRejected` on a dims
                        // mismatch), there is no FSM event shaped for an
                        // arbitrary wire-validation error: `TypesError` has
                        // several non-dims variants (missing field, wrong
                        // target arm, Opaque space, …) that a dims-only
                        // event would misreport. This is the ONLY actuation
                        // channel for an Agent-kind reset window (no teleop
                        // fallback), so a silent drop here is a real
                        // operability gap — surface it loudly instead of
                        // fabricating a fault event that would lie about
                        // what actually went wrong.
                        tracing::warn!(
                            error = %err,
                            "reset-window intervention_chunk rejected: incompatible \
                             with the declared action space"
                        );
                    }
                }
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
