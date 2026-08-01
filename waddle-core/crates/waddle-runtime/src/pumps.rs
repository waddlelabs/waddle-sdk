//! The session's background pumps: verb outcomes → FSM events, bypass
//! supervision (claimed-while-stalled), media intake, and plane directives.

use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender};
use std::thread::JoinHandle;
use std::time::Duration;

use waddle_controlplane::{ControlPlaneClient, PlaneEvent, ServerMsg};
use waddle_fsm::{GrantChangeDirective, Phase, SessionEvent};
use waddle_gate::gate::{GateShared, OwnedAction};
use waddle_gate::jitter::{ChunkMeta, StreamChannel, TimedAction};
use waddle_ingest::SessionClock;
use waddle_media::{DataTopic, MediaPlane};
use waddle_types::pb::v0 as pb;
use waddle_types::time::Clock;
use waddle_types::{
    ActionChunk, ActionSpace, ActorKind, ClaimId, EpisodeId, GateMode, GrantStatus, MonoNs, Step,
    TypesError, VerbRequest,
};

use crate::RuntimeError;
use crate::ack::{ACKS_FLAG, AckGroup, Injected};
use crate::media_uplink::STILLS_FLAG;
use crate::mirror::{
    AgentTaskKind, AgentTaskStatus, Mirror, ResetProgressPhase, ResetProgressStatus,
};
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
    inject: Sender<Injected>,
    clock: SessionClock,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("waddle-verb-outcomes".into())
        .spawn(move || {
            while let Ok(outcome) = outcomes.recv() {
                let _ = clock.stamp_now();
                let _ = inject.send(
                    SessionEvent::VerbResult {
                        verb: outcome.verb,
                        ok: outcome.result.is_ok(),
                        fault: outcome
                            .result
                            .is_err()
                            .then_some(pb::FaultKind::AdapterError),
                        at: outcome.at,
                    }
                    .into(),
                );
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
    inject: Sender<Injected>,
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
                        // `None` means no tick has EVER landed on this
                        // session's gate — definitionally stalled, not
                        // exempt: an agent-invited episode driven through
                        // `Session::run_agent` (flag `waddle.v0.agent`)
                        // reaches RUNNING via `Start` with the caller's
                        // thread blocked, so its very first engage must trip
                        // this without waiting for a tick that will never
                        // come. `Some` keeps the threshold contract
                        // unchanged. Either way the FSM's own guard (engaged
                        // claim, INTERVENTION phase) decides — this only
                        // reports the stall.
                        let stalled =
                            last_tick.is_none_or(|last| now.0 - last.0 > STALL_THRESHOLD_NS);
                        if stalled {
                            let _ = inject.send(SessionEvent::StallDetected { at: now }.into());
                        }
                    }
                    Some(GateMode::Bypass) => {
                        // Ticks resumed?
                        if let Some(last) = last_tick
                            && now.0 - last.0 <= STALL_THRESHOLD_NS
                        {
                            let _ = inject.send(SessionEvent::TicksResumed { at: now }.into());
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
    inject: Sender<Injected>,
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
                                let _ = inject.send(
                                    SessionEvent::ResetResult {
                                        ok,
                                        verified,
                                        at: clock.stamp_now().mono_ns(),
                                    }
                                    .into(),
                                );
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
                                let _ = inject.send(
                                    SessionEvent::PostResetResult {
                                        ok,
                                        detail: String::new(),
                                        at: clock.stamp_now().mono_ns(),
                                    }
                                    .into(),
                                );
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
/// pass through unchecked (no dims-validation contract applies). `gripper_spec`
/// is the session's declared `GripperSpec`: the raw teleop gripper command
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
    inject: Sender<Injected>,
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
            // Dims-validation contract: a fault fires at most once
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
                    // Stale-backlog replay guard: the gate only drains the
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
                            // GripperSpec mapping contract: map the raw
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
                                chunk: None,
                            });
                        } else if !validation_fault_sent {
                            validation_fault_sent = true;
                            let _ = inject.send(
                                SessionEvent::InterventionRejected {
                                    dims_got: action.values.len(),
                                    dims_want: expected_dims.unwrap_or(0),
                                    source: "media-intake",
                                    at: now,
                                }
                                .into(),
                            );
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
                        let _ = inject.send(
                            SessionEvent::Clutch {
                                engaged: clutch.engaged,
                                at: clock.stamp_now().mono_ns(),
                            }
                            .into(),
                        );
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
/// agent task updates, agent-chunk actuation — both the Reset-mode window
/// actuation and the general Claimed-mode intake). Also the attachment point for
/// directive-ack correlation (flag `waddle.v0.plane.acks`): the
/// pump tracks whether the plane accepted the flag at Register and, when it
/// did, wraps id-carrying directives' events in a shared [`AckGroup`] the
/// reducer completes.
pub(crate) fn spawn_plane_pump(
    plane: Arc<ControlPlaneClient>,
    inject: Sender<Injected>,
    clock: SessionClock,
    mirror: Arc<Mirror>,
    stream: StreamProducer,
    action_space: Arc<ActionSpace>,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("waddle-plane-pump".into())
        .spawn(move || {
            let mut was_connected = true;
            // Directive acks: whether the CURRENT connection negotiated
            // `waddle.v0.plane.acks`. Flags are (re-)negotiated at every
            // Register (the client re-registers on each reconnect), so each
            // `Registered` refreshes this — per VERSIONING §3, a behavior
            // the connection did not accept is never emitted.
            let mut acks_negotiated = false;
            // Monotonic ring-seq counter for agent-chunk steps pushed by the
            // `InterventionChunk` arm below — see that arm's doc comment for
            // why this can't just reuse `chunk.seq`.
            let mut next_chunk_seq: u64 = 0;
            // Dims-validation guard for the agent-chunk
            // path, mirroring `spawn_media_intake`'s `validation_fault_sent`:
            // a dims-mismatched chunk faults at most once per claim window.
            let mut chunk_dims_fault_sent = false;
            loop {
                let status = mirror.read();
                if status.shutdown {
                    return;
                }
                if !status.claim_active {
                    chunk_dims_fault_sent = false;
                }
                let Some(event) = plane.recv_event_timeout(Duration::from_millis(20)) else {
                    continue;
                };
                let at = clock.stamp_now().mono_ns();
                match event {
                    PlaneEvent::Connected | PlaneEvent::Registered(_) => {
                        if let PlaneEvent::Registered(resp) = &event {
                            acks_negotiated =
                                resp.accepted_feature_flags.iter().any(|f| f == ACKS_FLAG);
                            // Control-plane stills (flag
                            // `waddle.v0.obs.stills`) are emitted by the
                            // media uplink pump, not this one, so this
                            // acceptance crosses threads on the mirror —
                            // same per-connection refresh rule as
                            // `acks_negotiated` above.
                            let stills =
                                resp.accepted_feature_flags.iter().any(|f| f == STILLS_FLAG);
                            mirror.update(|s| s.stills_negotiated = stills);
                        }
                        if !was_connected {
                            was_connected = true;
                            let _ = inject.send(SessionEvent::PartitionEnd { at }.into());
                        }
                    }
                    PlaneEvent::Disconnected => {
                        if was_connected {
                            was_connected = false;
                            let _ = inject.send(SessionEvent::PartitionStart { at }.into());
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
                        &mut chunk_dims_fault_sent,
                        acks_negotiated,
                    ),
                }
            }
        })
        .expect("spawn plane pump")
}

/// The ack correlation for one directive, when there is one: `Some` only
/// when the connection negotiated `waddle.v0.plane.acks` AND the directive
/// carried a `directive_id`; `None` keeps the pre-flag fire-and-forget path
/// byte-for-byte. `events` is how many session events the directive decodes
/// into (the group acks once, when the last lands).
fn ack_group(
    acks_negotiated: bool,
    directive_id: Option<&String>,
    events: u32,
) -> Option<Arc<AckGroup>> {
    if !acks_negotiated {
        return None;
    }
    directive_id.map(|id| AckGroup::new(id.clone(), events))
}

#[allow(clippy::too_many_arguments)]
fn forward_server_msg(
    msg: ServerMsg,
    inject: &Sender<Injected>,
    at: MonoNs,
    mirror: &Mirror,
    stream: &StreamProducer,
    space: &ActionSpace,
    next_chunk_seq: &mut u64,
    chunk_dims_fault_sent: &mut bool,
    acks_negotiated: bool,
) {
    match msg {
        ServerMsg::Gate(gate_msg) => match gate_msg.msg {
            Some(pb::gate_server_message::Msg::Claim(directive)) => {
                // Directive acks: a directive too malformed to decode into
                // session events at all (no claim, unknown kind) produces no
                // ack — only FSM step outcomes are acked (services.proto's
                // DirectiveAck doc pins this).
                let Some(claim) = directive.claim else { return };
                let claim_id = ClaimId::new(&claim.claim_id);
                let actor = claim
                    .actor
                    .as_ref()
                    .and_then(|a| ActorKind::from_pb(a.kind).ok())
                    .unwrap_or(ActorKind::Teleoperator);
                match pb::ClaimDirectiveKind::try_from(directive.kind) {
                    Ok(pb::ClaimDirectiveKind::Grant) => {
                        // TWO events, ONE ack: accepted iff both accepted,
                        // reason from the first rejection.
                        let ack = ack_group(acks_negotiated, directive.directive_id.as_ref(), 2);
                        let _ = inject.send(Injected {
                            event: SessionEvent::ClaimGranted {
                                id: claim_id.clone(),
                                source: claim.source_name.clone(),
                                actor,
                                self_initiated: claim.self_initiated,
                                at,
                            },
                            ack: ack.clone(),
                        });
                        let _ = inject.send(Injected {
                            event: SessionEvent::Engage {
                                claim: claim_id,
                                at,
                            },
                            ack,
                        });
                    }
                    Ok(pb::ClaimDirectiveKind::Release) => {
                        let ack = ack_group(acks_negotiated, directive.directive_id.as_ref(), 1);
                        let _ = inject.send(Injected {
                            event: SessionEvent::Release {
                                claim: claim_id,
                                at,
                            },
                            ack,
                        });
                    }
                    Ok(pb::ClaimDirectiveKind::Retake) => {
                        let successor =
                            EpisodeId::new(format!("ep-{}", uuid::Uuid::new_v4().simple()));
                        let ack = ack_group(acks_negotiated, directive.directive_id.as_ref(), 1);
                        let _ = inject.send(Injected {
                            event: SessionEvent::Retake {
                                claim: claim_id,
                                initiator: actor,
                                successor,
                                at,
                            },
                            ack,
                        });
                    }
                    _ => {}
                }
            }
            Some(pb::gate_server_message::Msg::Episode(directive)) => {
                let outcome = waddle_types::TerminalOutcome::from_pb(directive.outcome)
                    .unwrap_or(waddle_types::TerminalOutcome::Abort);
                let ack = ack_group(acks_negotiated, directive.directive_id.as_ref(), 1);
                let _ = inject.send(Injected {
                    event: SessionEvent::Terminate {
                        outcome,
                        reason: directive.reason,
                        at,
                    },
                    ack,
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
                        // produce (`ClaimGranted` then
                        // `ResetWindowEngage`, in that order). TWO events,
                        // ONE ack, same as a claim GRANT.
                        let actor = claim
                            .actor
                            .as_ref()
                            .and_then(|a| ActorKind::from_pb(a.kind).ok())
                            .unwrap_or(ActorKind::Teleoperator);
                        let ack = ack_group(acks_negotiated, directive.directive_id.as_ref(), 2);
                        let _ = inject.send(Injected {
                            event: SessionEvent::ClaimGranted {
                                id: claim_id.clone(),
                                source: claim.source_name.clone(),
                                actor,
                                self_initiated: claim.self_initiated,
                                at,
                            },
                            ack: ack.clone(),
                        });
                        let _ = inject.send(Injected {
                            event: SessionEvent::ResetWindowEngage {
                                claim: claim_id,
                                at,
                            },
                            ack,
                        });
                    }
                    Ok(pb::ResetWindowDirectiveKind::Complete) => {
                        let Some(result) = directive.result else {
                            return;
                        };
                        let verified = result.verification.as_ref().map(|v| v.verified);
                        let ack = ack_group(acks_negotiated, directive.directive_id.as_ref(), 1);
                        let _ = inject.send(Injected {
                            event: SessionEvent::ResetWindowComplete {
                                claim: claim_id,
                                ok: result.ok,
                                verified,
                                at,
                            },
                            ack,
                        });
                    }
                    Ok(pb::ResetWindowDirectiveKind::Cancel) => {
                        // No dedicated FSM event for a plane-initiated
                        // cancel: it is observably a failed completion
                        // (E21 with ok=false) from the session's point of
                        // view — the same event COMPLETE{ok:false} uses.
                        let ack = ack_group(acks_negotiated, directive.directive_id.as_ref(), 1);
                        let _ = inject.send(Injected {
                            event: SessionEvent::ResetWindowComplete {
                                claim: claim_id,
                                ok: false,
                                verified: None,
                                at,
                            },
                            ack,
                        });
                    }
                    _ => {}
                }
            }
            // Agent-chunk intake (Reset-mode window actuation and the
            // general Claimed-mode intake): an `intervention_chunk`
            // is accepted whenever a claim is active — the SAME gate
            // `spawn_media_intake`'s teleop path already uses (`claim_active`
            // alone, no `GateMode` match), so a chunk arriving during the
            // ENGAGE handoff sub-phase (claim granted, lease not yet handed
            // over — gate mode hasn't flipped to `Intervention`/`Reset` yet)
            // still buffers correctly and is ready the instant the handoff
            // completes, exactly like a teleop packet would. Legality
            // (which claim, which mode) is never re-derived here — only a
            // plain mirror read (hollow-frontend); the jitter buffer is what
            // actually plays these out only while `Claimed`/`Reset`/`Bypass`
            // is polling it.
            //
            // Ring seq: `next_chunk_seq` is a fresh per-pump counter, not
            // `chunk.seq` — the jitter buffer's per-item reorder map is
            // keyed by seq *per step*, and a chunk's own `seq` is one value
            // for the whole chunk (every step in it would collide on the
            // same key). Tagged `StreamChannel::AgentChunk` so this
            // counter's seq space never shares a reorder/late-drop cursor
            // with the media intake's teleop-packet seq space, even though
            // both producers push into the same physical ring (`JitterBuffer`
            // keeps one cursor per channel — see `jitter.rs`'s module doc).
            //
            // `chunk.seq`/`chunk.t_emitted_ns` (the WIRE chunk's own
            // identity, distinct from `next_chunk_seq` above) ride along on
            // every step as a `ChunkMeta` so the jitter buffer can detect a
            // chunk boundary and apply the declared `ReplanPolicy`
            // (`jitter.rs`'s module doc) — a newer chunk arriving mid-horizon
            // supersedes the executing one's still-pending steps (IMMEDIATE/
            // BLEND) or queues behind them (CHUNK_BOUNDARY).
            //
            // Playout scheduling stays session-receive-time (`at`) + each
            // step's declared `t_offset_ns` — NOT `chunk.t_emitted_ns` +
            // offset — matching the Reset-mode arm's convention: `ActionChunk`'s
            // `_ns` fields are session-timeline per `VERSIONING.md` §7 (not
            // `_client_ns`, so no cross-clock offset-estimator mapping
            // applies), but nothing guarantees a remote agent's own
            // `t_emitted_ns` is usable as an absolute playout anchor on this
            // side, so it is used only for the chunk-boundary/staleness
            // decision above, never for scheduling.
            Some(pb::gate_server_message::Msg::InterventionChunk(chunk)) => {
                if !mirror.read().claim_active {
                    return;
                }
                match ActionChunk::from_pb(&chunk, space) {
                    Ok(action_chunk) => {
                        let meta = ChunkMeta {
                            chunk_seq: action_chunk.seq,
                            t_emitted_ns: action_chunk.t_emitted_ns,
                        };
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
                                chunk: Some(meta),
                            });
                        }
                    }
                    // Dims-validation contract, mirroring
                    // `spawn_media_intake`'s teleop path: a genuine dims
                    // mismatch faults once per claim window, chunk dropped.
                    Err(TypesError::DimensionMismatch { expected, got }) => {
                        if !*chunk_dims_fault_sent {
                            *chunk_dims_fault_sent = true;
                            let _ = inject.send(
                                SessionEvent::InterventionRejected {
                                    dims_got: got,
                                    dims_want: expected,
                                    source: "agent-chunk",
                                    at,
                                }
                                .into(),
                            );
                        }
                    }
                    Err(err) => {
                        // Every other `TypesError` variant (missing field,
                        // wrong target arm, Opaque space, …) doesn't fit the
                        // dims-shaped `InterventionRejected` event without
                        // misreporting what actually went wrong — surfaced
                        // as a trace warning instead of a fabricated fault.
                        tracing::warn!(
                            error = %err,
                            "intervention_chunk rejected: incompatible with the \
                             declared action space"
                        );
                    }
                }
            }
            // Agent task updates (flag `waddle.v0.agent`): every update is
            // retained on the mirror — QUEUED/COMPLETED are runtime-side
            // information only, never FSM events (FSM.md §1.5), and
            // COMPLETED's `recording_ref`/`detail` are what
            // `Session::run_agent` assembles its result from. A DENIED
            // addressed to the ACTIVE episode additionally dispatches
            // `AgentTaskDenied`; the FSM alone picks E26 (invite open:
            // abort) vs E26b (late: recorded-only rejection) — the
            // episode-id filter here is addressing, never legality (a DENIED
            // for some other episode has no event for the FSM to judge, so
            // it stays mirror-only, exactly like the conformance runner's
            // inert-record contract).
            Some(pb::gate_server_message::Msg::AgentUpdate(update)) => {
                let kind = AgentTaskKind::from_pb(update.kind);
                mirror.update(|s| {
                    s.agent_task = Some(AgentTaskStatus {
                        episode_id: update.episode_id.clone(),
                        kind,
                        detail: update.detail.clone(),
                        recording_ref: update.recording_ref.clone(),
                    });
                });
                if kind == AgentTaskKind::Denied
                    && mirror
                        .read()
                        .episode_id
                        .as_ref()
                        .is_some_and(|id| id.as_str() == update.episode_id)
                {
                    let ack = ack_group(acks_negotiated, update.directive_id.as_ref(), 1);
                    let _ = inject.send(Injected {
                        event: SessionEvent::AgentTaskDenied {
                            detail: update.detail,
                            at,
                        },
                        ack,
                    });
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
                let _ = inject.send(
                    SessionEvent::HeartbeatAck {
                        grant_changes: changes,
                        at,
                    }
                    .into(),
                );
            }
        }
        // A plane-EXECUTED reset (the `RequestReset`/`ResetProgress` RPCs,
        // `waddle.v0.reset` — distinct from the SDK-executed remote reset
        // WINDOW above, which is `ResetWindowDirective`/`ResetWindowEvent`
        // under `waddle.v0.reset.remote`): every message is observational
        // (mirror only — `episode.proto` doesn't model this as an
        // `EpisodeEvent`), and DONE additionally injects the same
        // `ResetResult` event the inline caller-thread path and the reset
        // pump already inject, completing the pipeline. No episode-id
        // filtering: `ResetProgress` carries none (session-scoped, like
        // `HeartbeatAck`), and the FSM's own guard (`ResetResult` requires
        // `Phase::Resetting` with no open remote window, E19b) is what makes
        // an out-of-order or stray DONE harmless — never decided here
        // (hollow-frontend).
        ServerMsg::ResetProgress(progress) => {
            let phase = ResetProgressPhase::from_pb(progress.phase);
            mirror.update(|s| {
                s.reset_progress = Some(ResetProgressStatus {
                    phase,
                    strategy: progress.strategy.clone(),
                    detail: progress.detail.clone(),
                });
            });
            if phase == ResetProgressPhase::Done
                && let Some(result) = progress.result
            {
                let _ = inject.send(
                    SessionEvent::ResetResult {
                        ok: result.ok,
                        verified: result.verification.as_ref().map(|v| v.verified),
                        at,
                    }
                    .into(),
                );
            }
        }
        _ => {}
    }
}
