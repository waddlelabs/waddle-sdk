//! Task 10 — reset-window actuation + plane directives + runtime e2e
//! (design §D4, brief scope 1–5): the bypass pump's RESET arm (teleop/agent
//! actions during a reset window go straight to `send`, same mechanics as
//! BYPASS); `forward_server_msg`'s `reset_window` and (Reset-mode-only)
//! `intervention_chunk` arms; and the full remote-reset flow driven through
//! a REAL `ControlPlaneClient` + `InMemoryTransport` script, not direct FSM
//! injection (Tasks 8/9 already covered the FSM-level mechanics that way).
//!
//! Ordering note (Task 8's report, Concern 2): a real plane's ENGAGE and
//! COMPLETE are seconds apart, never back-to-back — the scripts below wait
//! for `claim_active` then `gate_mode == Reset` before ever sending
//! COMPLETE, exactly as a real plane client would, to stay clear of the
//! documented `pending_lease` single-slot hazard.

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use prost::Message as _;
use waddle_controlplane::{ClientMsg, InMemoryTransport, ServerMsg};
use waddle_fsm::{Phase, SessionEvent};
use waddle_gate::gate::GateOutput;
use waddle_media::{DataTopic, LoopbackMedia};
use waddle_runtime::{
    ControlRegistry, EpisodeOptions, ResetSpec, Session, VerbError, grant_and_engage, release_claim,
};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActorKind, ClaimId, EpisodeId, GateMode, MonoNs, Provenance, TerminalOutcome};

/// A 6-dim `BaseTwist` robot: teleop `Twist` packets and agent `BaseTwist`
/// chunk steps both flatten to exactly its declared width, so intake/chunk
/// dims validation never rejects them (mirrors `e2e.rs`'s `twist_robot`).
fn twist_robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "reset-actuation-bot".into(),
        robot_id: "reset-actuation-01".into(),
        cell_id: "cell-reset-actuation".into(),
        action_space: Some(pb::ActionSpace {
            space: Some(pb::action_space::Space::BaseTwist(pb::BaseTwist {
                frame_id: "base".into(),
                max_linear_mps: None,
                max_angular_radps: None,
            })),
            rate_hz: 50.0,
            chunking: None,
            gripper: None,
        }),
        grants: vec![
            pb::Grant {
                verb: pb::Verb::Hold as i32,
                declared_latency_bound_ns: Some(50_000_000),
                ..Default::default()
            },
            pb::Grant {
                verb: pb::Verb::Send as i32,
                send_interfaces: vec![pb::SpaceKind::BaseTwist as i32],
                ..Default::default()
            },
        ],
        ..Default::default()
    }
}

/// A 3-dim `JointPosition` robot for the timeout test (no actuation
/// exercised, matches `e2e.rs`'s/`reset_pump.rs`'s plain `robot()`).
fn joint_robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "reset-actuation-joint-bot".into(),
        robot_id: "reset-actuation-02".into(),
        cell_id: "cell-reset-actuation".into(),
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
        grants: vec![
            pb::Grant {
                verb: pb::Verb::Hold as i32,
                declared_latency_bound_ns: Some(50_000_000),
                ..Default::default()
            },
            pb::Grant {
                verb: pb::Verb::Send as i32,
                send_interfaces: vec![pb::SpaceKind::JointPosition as i32],
                ..Default::default()
            },
        ],
        ..Default::default()
    }
}

type SendLog = Arc<Mutex<Vec<(Provenance, Vec<f64>)>>>;

fn registry(send_log: &SendLog) -> ControlRegistry {
    let log = send_log.clone();
    ControlRegistry {
        send: Some(Arc::new(
            move |chunk: &waddle_types::ActionChunk| -> Result<(), VerbError> {
                for step in &chunk.steps {
                    log.lock()
                        .push((chunk.provenance.provenance.clone(), step.values.to_vec()));
                }
                Ok(())
            },
        )),
        hold: Some(Arc::new(|| Ok(()))),
        resume: Some(Arc::new(|| Ok(()))),
        home: Some(Arc::new(|| Ok(()))),
        ..Default::default()
    }
}

fn wait_for(session: &Session, pred: impl Fn(&waddle_runtime::Status) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if pred(&session.status()) {
            return;
        }
        assert!(Instant::now() < deadline, "timed out waiting on status");
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Spawns the plane-side half of a reset-window scenario on its own thread:
/// waits for `is_open` to hold on the mirror (the window is open), ENGAGEs
/// with `(claim_id, actor)`, waits for the engage to actually land
/// (`claim_active` then `gate_mode == Reset` — the ordering discipline the
/// module doc explains), runs `after_engage` (e.g. push an agent chunk),
/// then blocks until `ready_to_complete` flips before sending
/// COMPLETE{ok:true, verified:true}. `session_cell` decouples this from
/// `SessionBuilder::build()`'s return, since the transport's server thread
/// can start running before the test has a `Session` handle to publish.
#[allow(clippy::too_many_arguments)]
fn script_reset_window(
    session_cell: Arc<Mutex<Option<Session>>>,
    tx: Sender<ServerMsg>,
    claim_id: &'static str,
    actor: pb::ActorKind,
    reset_kind: pb::ResetKind,
    is_open: impl Fn(&waddle_runtime::Status) -> bool + Send + 'static,
    after_engage: impl FnOnce(&Sender<ServerMsg>) + Send + 'static,
    ready_to_complete: Arc<AtomicBool>,
) {
    std::thread::spawn(move || {
        let session = loop {
            if let Some(s) = session_cell.lock().clone() {
                break s;
            }
            std::thread::sleep(Duration::from_millis(2));
        };
        loop {
            let st = session.status();
            if st.shutdown {
                return;
            }
            if is_open(&st) {
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
            msg: Some(pb::gate_server_message::Msg::ResetWindow(
                pb::ResetWindowDirective {
                    kind: pb::ResetWindowDirectiveKind::Engage as i32,
                    reset: reset_kind as i32,
                    claim: Some(pb::Claim {
                        claim_id: claim_id.to_owned(),
                        actor: Some(pb::ActorRef {
                            kind: actor as i32,
                            ..Default::default()
                        }),
                        source_name: "plane-script".into(),
                        self_initiated: false,
                        ..Default::default()
                    }),
                    result: None,
                },
            )),
        }));
        loop {
            let st = session.status();
            if st.shutdown {
                return;
            }
            if st.claim_active && st.gate_mode == Some(GateMode::Reset) {
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        after_engage(&tx);
        loop {
            if ready_to_complete.load(Ordering::SeqCst) {
                break;
            }
            if session.status().shutdown {
                return;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
            msg: Some(pb::gate_server_message::Msg::ResetWindow(
                pb::ResetWindowDirective {
                    kind: pb::ResetWindowDirectiveKind::Complete as i32,
                    reset: reset_kind as i32,
                    claim: Some(pb::Claim {
                        claim_id: claim_id.to_owned(),
                        ..Default::default()
                    }),
                    result: Some(pb::ResetResult {
                        ok: true,
                        verification: Some(pb::ResetVerification {
                            verified: true,
                            ..Default::default()
                        }),
                        ..Default::default()
                    }),
                },
            )),
        }));
    });
}

/// A transport that just registers on connect, then hands every `Register`
/// off to `on_register` (spawns the plane script) exactly once.
fn scripted_transport(
    on_register: impl Fn(&Sender<ServerMsg>) + Send + Sync + 'static,
) -> Arc<InMemoryTransport> {
    InMemoryTransport::new(move |msg, tx| {
        if let ClientMsg::Register(_) = &msg {
            let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse::default()));
            on_register(tx);
        }
    })
}

// --- Remote PRE window: teleop actuation ----------------------------------

#[test]
fn remote_pre_reset_window_dispatches_teleop_then_completes_to_ready() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (media, far) = LoopbackMedia::new();
    let session_cell: Arc<Mutex<Option<Session>>> = Arc::new(Mutex::new(None));
    let ready_to_complete = Arc::new(AtomicBool::new(false));

    let cell_for_script = session_cell.clone();
    let ready_for_script = ready_to_complete.clone();
    let transport = scripted_transport(move |tx| {
        script_reset_window(
            cell_for_script.clone(),
            tx.clone(),
            "reset-claim-pre",
            pb::ActorKind::Teleoperator,
            pb::ResetKind::Pre,
            |s| matches!(s.episode_state, Some(Phase::Resetting)),
            |_tx| {}, // teleop rides the media plane, not the plane's tx
            ready_for_script.clone(),
        );
    });

    let session = Session::builder("e2e-reset-pre")
        .robot(twist_robot())
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .media(media)
        .transport(transport)
        .pre_reset(ResetSpec::Remote {
            actor: ActorKind::Teleoperator,
            prompt: "clear the table".into(),
            timeout_ns: 30_000_000_000,
        })
        .build()
        .unwrap();
    *session_cell.lock() = Some(session.clone());

    // A warmup episode with the session's Remote pre-reset default
    // disabled (inline, immediate) — its own handle stays valid (and
    // shares the session's one `GateShared`) after it terminates, so it's
    // the "stale caller loop" that ticks `gate()` during the SECOND
    // episode's remote pre-reset window below (there is no `Episode` for
    // that window's own episode yet — `start_episode_with` is still
    // blocked in RESETTING).
    let mut ep0 = session
        .start_episode_with(
            "warmup",
            EpisodeOptions {
                pre_reset: Some(None),
                ..Default::default()
            },
        )
        .unwrap();
    let _ = ep0.gate(&[0.0; 6], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    ep0.terminate(TerminalOutcome::Success, "warmup done");

    let handle = {
        let session = session.clone();
        std::thread::spawn(move || session.start_episode_with("towel", EpisodeOptions::default()))
    };

    // The window engages (the script above): gate flips to RESET.
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Reset));

    // The stale ep0 handle's gate ticks are spectator NOOPs during the
    // window (D7 edge 3) — and, since `Gate`/`GateShared` is shared across
    // the whole session, they land on the NEW episode's now-open MCAP as
    // RESET_ACTIVE NoopMarkers tagged with the claimant's provenance
    // (checked at the bottom of this test).
    for _ in 0..3 {
        assert!(matches!(
            ep0.gate(&[0.0; 6], None, None),
            GateOutput::Noop { .. }
        ));
        std::thread::sleep(Duration::from_millis(5));
    }

    // Push teleop poses via the media plane; the bypass pump's RESET arm
    // must relay them to `send` (same mechanics as BYPASS).
    let seq = Arc::new(AtomicU32::new(1));
    let push_pose = |value: f64| {
        far.push(
            DataTopic::TeleopPose,
            &pb::TeleopStreamPacket {
                t_client_ns: 1,
                seq: u64::from(seq.fetch_add(1, Ordering::SeqCst)),
                targets: vec![pb::PartTarget {
                    part: String::new(),
                    target: Some(pb::part_target::Target::Twist(pb::Twist {
                        linear: Some(pb::Vec3 {
                            x: value,
                            y: 0.0,
                            z: 0.0,
                        }),
                        angular: Some(pb::Vec3::default()),
                    })),
                    gripper: None,
                }],
                clutch_engaged: true,
                inputs: None,
            },
        )
        .unwrap();
    };

    let deadline = Instant::now() + Duration::from_secs(5);
    let mut dispatched = false;
    while Instant::now() < deadline {
        push_pose(0.4);
        if send_log
            .lock()
            .iter()
            .any(|(p, _)| *p == Provenance::Teleop)
        {
            dispatched = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        dispatched,
        "reset-window teleop actuation never reached `send`"
    );

    // Tell the script to complete the window; the episode reaches READY.
    ready_to_complete.store(true, Ordering::SeqCst);
    let mut ep = handle
        .join()
        .unwrap()
        .expect("remote pre-reset window must resolve to READY");
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Passthrough));

    // Normal rollout proceeds on the returned Episode.
    assert!(matches!(
        ep.gate(&[0.0; 6], None, None),
        GateOutput::Pass { .. }
    ));
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    // The MCAP read-back requirement (the reset-window actuation appears
    // on /waddle/actions with the claimant's provenance) is checked in
    // `remote_post_reset_window_dispatches_agent_chunk_then_completes_to_terminal`
    // below, not here: `drain_gate_records` deliberately discards whatever
    // is left in a stale handle's OWN record ring the moment a NEWER
    // episode's fresh ring shows up (reducer.rs's doc comment on that
    // function) — exactly so a predecessor's stray ticks can never land in
    // a successor's MCAP. `ep0` above is a stale handle from BEFORE this
    // window's own episode, so its ticks (correctly returning `Noop`, per
    // the caller-facing contract just asserted) are recorded nowhere. The
    // POST-window test below ticks the SAME episode whose window is open,
    // so its ticks legitimately land in that episode's own MCAP.
}

// --- Remote POST window: agent-chunk actuation ----------------------------

#[test]
fn remote_post_reset_window_dispatches_agent_chunk_then_completes_to_terminal() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session_cell: Arc<Mutex<Option<Session>>> = Arc::new(Mutex::new(None));
    let ready_to_complete = Arc::new(AtomicBool::new(false));

    let cell_for_script = session_cell.clone();
    let ready_for_script = ready_to_complete.clone();
    let transport = scripted_transport(move |tx| {
        script_reset_window(
            cell_for_script.clone(),
            tx.clone(),
            "reset-claim-post",
            pb::ActorKind::Agent,
            pb::ResetKind::Post,
            |s| matches!(s.episode_state, Some(Phase::PostReset)),
            |tx| {
                // The agent's chunk arrives over the plane, not the media
                // plane: one BaseTwist step, due ~20ms (the gate's playout
                // delay) after this arrival.
                let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
                    msg: Some(pb::gate_server_message::Msg::InterventionChunk(
                        pb::ActionChunk {
                            actions: vec![pb::Action {
                                target: Some(pb::action::Target::BaseTwist(pb::Twist {
                                    linear: Some(pb::Vec3 {
                                        x: 0.5,
                                        y: 0.0,
                                        z: 0.0,
                                    }),
                                    angular: Some(pb::Vec3::default()),
                                })),
                                gripper: None,
                                t_offset_ns: 0,
                                part: String::new(),
                            }],
                            horizon_ns: 0,
                            t_emitted_ns: 0,
                            t_obs_ns: 0,
                            seq: 1,
                            source_id: "agent-script".into(),
                            provenance: None,
                        },
                    )),
                }));
            },
            ready_for_script.clone(),
        );
    });

    let session = Session::builder("e2e-reset-post")
        .robot(twist_robot())
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .transport(transport)
        .post_reset(ResetSpec::Remote {
            actor: ActorKind::Agent,
            prompt: "stow the tool".into(),
            timeout_ns: 30_000_000_000,
        })
        .build()
        .unwrap();
    *session_cell.lock() = Some(session.clone());

    let mut ep = session.start_episode("stow").unwrap();
    let id = ep.id().clone();
    let _ = ep.gate(&[0.0; 6], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    // NOT `ep.terminate(...)`: that call blocks until `Phase::Terminal` (the
    // design contract — see `session.rs::terminate_episode`'s rustdoc), and
    // this test's own thread is what must later flip `ready_to_complete` to
    // let the window resolve. Inject the same event non-blocking instead —
    // the identical seam `terminate_episode` itself uses internally, and
    // the same one Task 9's own remote-post-reset test uses for this exact
    // reason.
    session.inject(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "rollout done".to_owned(),
        at: MonoNs(1_000_000),
    });

    // POST_RESET opens the declared window; the script above ENGAGEs as
    // Agent, pushes the chunk, and the bypass pump's RESET arm relays it.
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut dispatched = false;
    while Instant::now() < deadline {
        if send_log.lock().iter().any(|(p, _)| *p == Provenance::Agent) {
            dispatched = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        dispatched,
        "reset-window agent-chunk actuation never reached `send`"
    );

    // A stale gate tick during the window is a spectator NOOP (D7 edge 3).
    assert!(matches!(
        ep.gate(&[0.0; 6], None, None),
        GateOutput::Noop { .. }
    ));

    ready_to_complete.store(true, Ordering::SeqCst);
    wait_for(&session, |s| {
        matches!(
            s.episode_state,
            Some(Phase::Terminal(TerminalOutcome::Success))
        )
    });
    assert!(ep.done());
    assert_eq!(ep.outcome(), Some(TerminalOutcome::Success));
    session.shutdown();

    // Sidecar: the post-reset span and pinned outcome are recorded.
    let sidecar_path = dir.path().join(format!("{id}.sidecar.json"));
    let sidecar =
        waddle_sidecar::sidecar_from_json(&std::fs::read_to_string(&sidecar_path).unwrap())
            .unwrap();
    assert!(sidecar.post_reset_declared);
    assert!(!sidecar.post_reset_failed);
    assert!(sidecar.post_reset_result.as_ref().unwrap().ok);
    assert_eq!(sidecar.outcome, pb::TerminalOutcome::Success as i32);
    let bounds = sidecar.post_reset_bounds.as_ref().unwrap();
    assert!(bounds.t_end_ns >= bounds.t_start_ns);

    // MCAP read-back: the reset-window actuation appears on
    // /waddle/actions with the claimant's provenance — the stale gate tick
    // above recorded as a RESET_ACTIVE NoopMarker tagged `Provenance::Agent`
    // (the gate's own per-tick record is the only writer onto that topic;
    // the bypass pump's direct `send` dispatch is a separate verb call, not
    // itself an MCAP record — unchanged from BYPASS before this task).
    let buf = std::fs::read(dir.path().join(format!("{id}.mcap"))).unwrap();
    let mut reset_active_agent_noops = 0;
    for message in mcap::MessageStream::new(&buf).unwrap() {
        let message = message.unwrap();
        if message.channel.topic != waddle_sidecar::mcaprec::ACTIONS_TOPIC {
            continue;
        }
        let chunk = pb::ActionChunk::decode(message.data.as_ref()).unwrap();
        let is_agent =
            chunk.provenance.as_ref().map(|p| p.kind) == Some(pb::ProvenanceKind::Agent as i32);
        for action in &chunk.actions {
            if let Some(pb::action::Target::Noop(marker)) = &action.target
                && marker.reason == pb::NoopReason::ResetActive as i32
                && is_agent
            {
                reset_active_agent_noops += 1;
            }
        }
    }
    assert!(
        reset_active_agent_noops > 0,
        "expected the reset-window actuation (RESET_ACTIVE noops, agent provenance) \
         on /waddle/actions, got {reset_active_agent_noops}"
    );
}

// --- Window timeout: the FSM's own timer, real elapsed time ---------------

#[test]
fn post_reset_window_timeout_pins_outcome_and_flags_failure() {
    let session = Session::builder("e2e-reset-post-timeout")
        .robot(joint_robot())
        .post_reset(ResetSpec::Remote {
            actor: ActorKind::Teleoperator,
            prompt: "clear the table".into(),
            // Short and real: nobody ever ENGAGEs, so the FSM's own
            // `ResetWindowTimeout` must fire from real elapsed time — not a
            // `session.inject(TimerFired)` shortcut (that path is already
            // covered by Task 8's `remote_pre_reset_window_timeout_...`).
            timeout_ns: 150_000_000,
        })
        .build()
        .unwrap();

    let mut ep = session.start_episode("never-cleaned-up").unwrap();
    let _ = ep.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    // `Episode::terminate` blocks to `Phase::Terminal`; the post-reset
    // window's real timeout is what lets it return at all.
    ep.terminate(TerminalOutcome::Success, "rollout done");
    assert!(ep.done());

    let status = session.status();
    assert!(
        matches!(
            status.episode_state,
            Some(Phase::Terminal(TerminalOutcome::Success))
        ),
        "expected Terminal{{SUCCESS}}, got {:?}",
        status.episode_state
    );
    assert_eq!(
        status.pinned_outcome,
        Some(TerminalOutcome::Success),
        "a timed-out post-reset window must never alter the pinned outcome"
    );
    assert!(
        status.post_reset_failed,
        "a timed-out post-reset window must flag post_reset_failed"
    );
    session.shutdown();
}

// --- Timer hygiene: one episode, two remote windows -----------------------

/// Design D7 edge 6 ("timer leak → cancel-then-arm on reuse"), verified at
/// the runtime level, not reimplemented: one episode declares BOTH a
/// Remote pre- and post-reset. The PRE window engages and completes well
/// inside its own deadline; the POST window is never engaged and times out
/// for real on ITS OWN short deadline. Both windows arm the same
/// `TimerId::ResetWindowTimeout` slot — if the PRE window's cancellation
/// leaked, a stale entry could misfire during POST_RESET; the correct,
/// independent POST timeout (not a `TimerFired` shortcut — real elapsed
/// time) is the proof it didn't.
#[test]
fn remote_pre_and_post_windows_reuse_the_window_timer_independently() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (media, far) = LoopbackMedia::new();
    let session_cell: Arc<Mutex<Option<Session>>> = Arc::new(Mutex::new(None));
    let ready_to_complete = Arc::new(AtomicBool::new(false));

    let cell_for_script = session_cell.clone();
    let ready_for_script = ready_to_complete.clone();
    let transport = scripted_transport(move |tx| {
        script_reset_window(
            cell_for_script.clone(),
            tx.clone(),
            "reset-claim-hygiene-pre",
            pb::ActorKind::Teleoperator,
            pb::ResetKind::Pre,
            |s| matches!(s.episode_state, Some(Phase::Resetting)),
            |_tx| {},
            ready_for_script.clone(),
        );
        // POST is deliberately NOT scripted: it must time out on its own.
    });

    let session = Session::builder("e2e-reset-timer-hygiene")
        .robot(twist_robot())
        .control(registry(&send_log))
        .media(media)
        .transport(transport)
        .pre_reset(ResetSpec::Remote {
            actor: ActorKind::Teleoperator,
            prompt: "clear the table".into(),
            // Short but ample: the script below completes it in well
            // under a second.
            timeout_ns: 5_000_000_000,
        })
        .post_reset(ResetSpec::Remote {
            actor: ActorKind::Teleoperator,
            prompt: "stow the tool".into(),
            // Short and real: nothing ever engages it.
            timeout_ns: 150_000_000,
        })
        .build()
        .unwrap();
    *session_cell.lock() = Some(session.clone());

    let handle = {
        let session = session.clone();
        std::thread::spawn(move || session.start_episode_with("towel", EpisodeOptions::default()))
    };

    wait_for(&session, |s| s.gate_mode == Some(GateMode::Reset));
    let seq = Arc::new(AtomicU32::new(1));
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut dispatched = false;
    while Instant::now() < deadline {
        far.push(
            DataTopic::TeleopPose,
            &pb::TeleopStreamPacket {
                t_client_ns: 1,
                seq: u64::from(seq.fetch_add(1, Ordering::SeqCst)),
                targets: vec![pb::PartTarget {
                    part: String::new(),
                    target: Some(pb::part_target::Target::Twist(pb::Twist {
                        linear: Some(pb::Vec3 {
                            x: 0.4,
                            y: 0.0,
                            z: 0.0,
                        }),
                        angular: Some(pb::Vec3::default()),
                    })),
                    gripper: None,
                }],
                clutch_engaged: true,
                inputs: None,
            },
        )
        .unwrap();
        if send_log
            .lock()
            .iter()
            .any(|(p, _)| *p == Provenance::Teleop)
        {
            dispatched = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        dispatched,
        "the PRE window's teleop actuation never reached `send`"
    );

    ready_to_complete.store(true, Ordering::SeqCst);
    let mut ep = handle
        .join()
        .unwrap()
        .expect("the PRE window must resolve to READY well inside its own deadline");
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Passthrough));

    // A normal rollout, then terminate into the declared POST window.
    let _ = ep.gate(&[0.0; 6], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    session.inject(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "rollout done".to_owned(),
        at: MonoNs(1_000_000),
    });

    // Nobody engages the POST window: its own 150ms deadline fires for
    // real. If the PRE window's timer had leaked, this is where a stale
    // entry would misfire — the correct outcome here (pinned SUCCESS,
    // post_reset_failed) is the proof it didn't.
    wait_for(&session, |s| {
        matches!(
            s.episode_state,
            Some(Phase::Terminal(TerminalOutcome::Success))
        )
    });
    let status = session.status();
    assert_eq!(status.pinned_outcome, Some(TerminalOutcome::Success));
    assert!(
        status.post_reset_failed,
        "the POST window's own independent timeout must have fired"
    );
    session.shutdown();
}

// --- Born-claimed guard: no remote pre-window for a retake successor ------

/// Design C6/D7 edge 5, verified at the runtime level, not reimplemented: a
/// retake successor is born-claimed (the surviving claim keeps driving the
/// hand reset), so it never gets a remote pre-window even though the
/// session's default is `Remote` — `reducer.rs`'s `Effect::OpenSuccessor`
/// handling hardcodes `pre_window: None` for exactly this reason. Verified
/// two ways: the surviving claim's `gate_mode` never flips to `Reset`, and
/// an attempted `ResetWindowEngage` for that claim has no effect (there is
/// no window open to engage).
#[test]
fn born_claimed_retake_successor_gets_no_remote_pre_window() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session = Session::builder("e2e-reset-born-claimed")
        .robot(twist_robot())
        .control(registry(&send_log))
        .pre_reset(ResetSpec::Remote {
            actor: ActorKind::Teleoperator,
            prompt: "clear the table".into(),
            timeout_ns: 30_000_000_000,
        })
        .build()
        .unwrap();

    // The FIRST episode disables the session's Remote pre-reset default
    // (no plane is wired in this test) so it can reach RUNNING on its own;
    // the born-claimed guard under test applies to its RETAKE SUCCESSOR,
    // which never asks `start_episode_with` for anything (the reducer
    // opens it directly).
    let mut ep1 = session
        .start_episode_with(
            "first",
            EpisodeOptions {
                pre_reset: Some(None),
                ..Default::default()
            },
        )
        .unwrap();
    let first_id = ep1.id().clone();
    let _ = ep1.gate(&[0.0; 6], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(&session, "claim-born", "teleop", ActorKind::Teleoperator);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));

    let successor = EpisodeId::new("ep-born-claimed-successor");
    session.inject(SessionEvent::Retake {
        claim: ClaimId::new("claim-born"),
        initiator: ActorKind::Teleoperator,
        successor: successor.clone(),
        at: MonoNs(3_000_000),
    });
    wait_for(&session, |s| s.episode_id.as_ref() == Some(&successor));
    assert!(session.episode_done(&first_id));

    // The surviving claim keeps driving Intervention — never a reset
    // window — for the whole time the successor sits in RESETTING.
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(
        session.status().gate_mode,
        Some(GateMode::Intervention),
        "a born-claimed successor's surviving claim must keep driving \
         Intervention, never a reset window"
    );

    // Confirming there is truly no window to engage (not merely "not yet
    // engaged"): `gate_mode` is the only externally-observable effect
    // `ResetWindowEngage` could have, and it does not change.
    session.inject(SessionEvent::ResetWindowEngage {
        claim: ClaimId::new("claim-born"),
        at: MonoNs(4_000_000),
    });
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(
        session.status().gate_mode,
        Some(GateMode::Intervention),
        "no reset window exists for a born-claimed successor to engage"
    );

    session.shutdown();
}

// --- Cross-producer seq isolation: teleop activity, then an agent window --

/// Review finding (CRITICAL): the intervention ring's `JitterBuffer` used to
/// keep ONE shared `last_popped_seq` watermark for the whole session, but
/// two independent producers write into it — the media-intake thread
/// (teleop, seq = wire `TeleopStreamPacket.seq`) and the plane pump's
/// `InterventionChunk` arm (agent chunks, seq = a fresh pump-local counter
/// starting at 0). An ordinary teleop claim EARLIER in the session (nothing
/// to do with any reset window) would advance that single shared cursor
/// past 1, so the FIRST agent-chunk step of a LATER reset window — exactly
/// the `pre_reset=TeleopReset`/`post_reset=AgentReset` shape the design's
/// own D5 examples suggest as normal — would look "late" and be silently,
/// permanently dropped (`ingest`'s `dropped_late` counter has zero readers
/// anywhere, so this failed with no diagnostic trail: the window would just
/// time out).
///
/// This test reproduces exactly that precondition: episode 1 runs an
/// ordinary (non-reset) teleop Intervention claim to completion first —
/// pushing enough packets that the teleop channel's cursor climbs well past
/// any small number — then episode 2 opens a Remote POST window with an
/// AGENT claimant and must still see its chunk dispatched. Against the
/// pre-fix single-cursor `JitterBuffer` this hangs until the 5s assertion
/// deadline; against the fix (one reorder cursor per `StreamChannel`) it
/// passes the same way the other agent-chunk test does.
///
/// Review finding (IMPORTANT, follow-up): per-channel cursors alone don't
/// close the whole gap. Episode 1's claim releases with several teleop
/// packets still in-flight (pushed, not yet due — the 20ms playout delay
/// vs. this test's 3ms push cadence guarantees that); nothing pops the
/// ring again until episode 2's Reset-mode pump starts polling it, at which
/// point those leftovers become due and get dispatched tagged with episode
/// 2's CURRENT mirror provenance (`Agent`) — not the wrong-channel seq
/// collision the cursor fix targets, but the SAME channel's own residue
/// outliving the claim that produced it. This test's final assertion
/// checks the dispatched VALUES, not just provenance, to prove that
/// residue never reaches `send` at all (see `Effect::SetGateMode`'s
/// clear-on-Passthrough in `waddle-runtime`'s `reducer.rs` and
/// `StreamIntake::clear`/`JitterBuffer::clear_pending` in `waddle-gate`).
#[test]
fn remote_post_reset_window_agent_chunk_survives_prior_teleop_claim_activity() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (media, far) = LoopbackMedia::new();
    let session_cell: Arc<Mutex<Option<Session>>> = Arc::new(Mutex::new(None));
    let ready_to_complete = Arc::new(AtomicBool::new(false));

    let cell_for_script = session_cell.clone();
    let ready_for_script = ready_to_complete.clone();
    let transport = scripted_transport(move |tx| {
        script_reset_window(
            cell_for_script.clone(),
            tx.clone(),
            "reset-claim-post-mixed",
            pb::ActorKind::Agent,
            pb::ResetKind::Post,
            |s| matches!(s.episode_state, Some(Phase::PostReset)),
            |tx| {
                let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
                    msg: Some(pb::gate_server_message::Msg::InterventionChunk(
                        pb::ActionChunk {
                            actions: vec![pb::Action {
                                target: Some(pb::action::Target::BaseTwist(pb::Twist {
                                    linear: Some(pb::Vec3 {
                                        x: 0.5,
                                        y: 0.0,
                                        z: 0.0,
                                    }),
                                    angular: Some(pb::Vec3::default()),
                                })),
                                gripper: None,
                                t_offset_ns: 0,
                                part: String::new(),
                            }],
                            horizon_ns: 0,
                            t_emitted_ns: 0,
                            t_obs_ns: 0,
                            seq: 1,
                            source_id: "agent-script".into(),
                            provenance: None,
                        },
                    )),
                }));
            },
            ready_for_script.clone(),
        );
    });

    let session = Session::builder("e2e-reset-post-mixed-producers")
        .robot(twist_robot())
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .media(media)
        .transport(transport)
        .post_reset(ResetSpec::Remote {
            actor: ActorKind::Agent,
            prompt: "stow the tool".into(),
            timeout_ns: 30_000_000_000,
        })
        .build()
        .unwrap();
    *session_cell.lock() = Some(session.clone());

    // Episode 1: an ORDINARY teleop claim (no reset window at all) drives
    // the teleop channel's cursor well past any small number, then
    // terminates directly (POST disabled for THIS episode only).
    let mut ep1 = session
        .start_episode_with(
            "teleop-warmup",
            EpisodeOptions {
                post_reset: Some(None),
                ..Default::default()
            },
        )
        .unwrap();
    let _ = ep1.gate(&[0.0; 6], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(
        &session,
        "claim-teleop-first",
        "teleop",
        ActorKind::Teleoperator,
    );
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));

    let seq = Arc::new(AtomicU32::new(1));
    let mut substitutions = 0u32;
    let deadline = Instant::now() + Duration::from_secs(5);
    // Push (and drain, via this episode's own `gate()` calls — the
    // Claimed-mode fast path pop_due's the ring directly) at least 40
    // packets, well past the agent-chunk arm's own counter, which always
    // starts fresh at 1 for a new window.
    while substitutions < 40 && Instant::now() < deadline {
        far.push(
            DataTopic::TeleopPose,
            &pb::TeleopStreamPacket {
                t_client_ns: 1,
                seq: u64::from(seq.fetch_add(1, Ordering::SeqCst)),
                targets: vec![pb::PartTarget {
                    part: String::new(),
                    target: Some(pb::part_target::Target::Twist(pb::Twist {
                        linear: Some(pb::Vec3 {
                            x: 0.3,
                            y: 0.0,
                            z: 0.0,
                        }),
                        angular: Some(pb::Vec3::default()),
                    })),
                    gripper: None,
                }],
                clutch_engaged: true,
                inputs: None,
            },
        )
        .unwrap();
        std::thread::sleep(Duration::from_millis(3));
        if matches!(
            ep1.gate(&[0.0; 6], None, None),
            GateOutput::Substitute { .. } | GateOutput::Blend { .. }
        ) {
            substitutions += 1;
        }
    }
    assert!(
        substitutions >= 40,
        "expected the teleop channel's cursor to advance well past a small \
         seq before the agent window opens, got {substitutions} substitutions"
    );

    release_claim(&session, "claim-teleop-first");
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Passthrough));
    assert!(matches!(
        ep1.gate(&[0.0; 6], None, None),
        GateOutput::Pass { .. }
    ));
    ep1.terminate(TerminalOutcome::Success, "warmup done");
    assert!(ep1.done());

    // Episode 2: a Remote POST window with an AGENT claimant. Its chunk's
    // steps use the plane pump's own fresh `next_chunk_seq` counter
    // (starting at 1) — with a single shared jitter-buffer cursor this
    // would collide with episode 1's teleop activity above and be dropped
    // forever; with per-channel cursors it must dispatch normally.
    let mut ep2 = session.start_episode("stow").unwrap();
    let _ = ep2.gate(&[0.0; 6], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    session.inject(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "rollout done".to_owned(),
        at: MonoNs(1_000_000),
    });

    // Checked on the DISPATCHED VALUE, not just the provenance tag: once
    // episode 2's window starts polling the ring, it could in principle
    // also drain and dispatch any of episode 1's own teleop packets that
    // were still in-flight (pushed but not yet popped) when the claim
    // released — those would get tagged `Provenance::Agent` too
    // (provenance comes from the CURRENT mirror status at pop time, not
    // from the pushed item), so a provenance-only check could pass on
    // stale teleop residue instead of proving the agent chunk itself got
    // through. The agent chunk's `x` is 0.5; every teleop packet pushed
    // above used 0.3 — distinguishable.
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut dispatched = false;
    while Instant::now() < deadline {
        if send_log
            .lock()
            .iter()
            .any(|(p, v)| *p == Provenance::Agent && v.first().is_some_and(|x| *x > 0.4))
        {
            dispatched = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        dispatched,
        "the agent-chunk actuation of a LATER reset window must not be \
         dropped because an EARLIER, unrelated teleop claim already \
         advanced a shared jitter-buffer cursor"
    );

    ready_to_complete.store(true, Ordering::SeqCst);
    wait_for(&session, |s| {
        matches!(
            s.episode_state,
            Some(Phase::Terminal(TerminalOutcome::Success))
        )
    });
    assert!(ep2.done());

    // Review finding (IMPORTANT): with a 20ms playout delay and packets
    // pushed every 3ms, episode 1's teleop claim reliably still has
    // in-flight (pushed but not yet due) packets sitting in the ring's
    // Teleop channel the instant it releases — this loop's own assertion
    // above (`substitutions >= 40`) guarantees the ramp-up ran long enough
    // for that to be true. Before the leak was closed
    // (`Effect::SetGateMode`'s clear-on-Passthrough / `StreamIntake::clear`
    // in `waddle-gate`), those leftovers sat in the buffer until episode
    // 2's Reset-mode pump started polling it, at which point they were
    // popped and dispatched tagged with episode 2's CURRENT mirror
    // provenance (`Agent`) — passing the provenance-only check above for
    // the wrong reason. Assert directly that NONE of episode 1's teleop
    // values (`x` == 0.3) ever reached `send`, under ANY provenance: this
    // is the proof the leak itself is closed, not just that the intended
    // agent chunk happened to also get through.
    assert!(
        send_log
            .lock()
            .iter()
            .all(|(_, v)| !v.first().is_some_and(|x| (*x - 0.3).abs() < 0.05)),
        "a stale teleop packet from episode 1's already-released claim \
         reached `send` during episode 2's reset window — the intervention \
         ring's pending map must be cleared on the transition back to \
         Passthrough, not merely reordered without cross-contamination"
    );

    session.shutdown();
}

// --- Malformed agent chunk during a reset window: rejected, not dropped ---

/// Review finding (IMPORTANT): unlike the media-intake teleop path (which
/// raises `SessionEvent::InterventionRejected` on a dims mismatch), a
/// malformed or action-space-incompatible `intervention_chunk` arriving
/// during a Reset-mode window used to be dropped by
/// `ActionChunk::from_pb`'s `Err` arm with zero signal and, more
/// importantly, with no guarantee the window could still recover. This test
/// proves the behavioral half of the fix: a chunk whose target doesn't match
/// the declared action space is safely ignored — not dispatched, and not
/// fatal to the window, which still resolves normally once the plane sends
/// COMPLETE. (The diagnostic half of the fix — a `tracing::warn!` naming the
/// rejection — isn't asserted here: this workspace has no tracing subscriber
/// wired yet, and installing one process-wide would conflict with the other
/// tests in this binary running in parallel.)
#[test]
fn remote_post_reset_window_ignores_malformed_agent_chunk_and_still_completes() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session_cell: Arc<Mutex<Option<Session>>> = Arc::new(Mutex::new(None));
    let ready_to_complete = Arc::new(AtomicBool::new(false));

    let cell_for_script = session_cell.clone();
    let ready_for_script = ready_to_complete.clone();
    let transport = scripted_transport(move |tx| {
        script_reset_window(
            cell_for_script.clone(),
            tx.clone(),
            "reset-claim-post-malformed",
            pb::ActorKind::Agent,
            pb::ResetKind::Post,
            |s| matches!(s.episode_state, Some(Phase::PostReset)),
            |tx| {
                // Wrong target arm for a BaseTwist-space robot: `from_pb`
                // must reject this (`TypesError::InvalidValue`), not panic.
                let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
                    msg: Some(pb::gate_server_message::Msg::InterventionChunk(
                        pb::ActionChunk {
                            actions: vec![pb::Action {
                                target: Some(pb::action::Target::JointPosition(pb::JointVector {
                                    values: vec![0.0; 3],
                                })),
                                gripper: None,
                                t_offset_ns: 0,
                                part: String::new(),
                            }],
                            horizon_ns: 0,
                            t_emitted_ns: 0,
                            t_obs_ns: 0,
                            seq: 1,
                            source_id: "agent-script".into(),
                            provenance: None,
                        },
                    )),
                }));
            },
            ready_for_script.clone(),
        );
    });

    let session = Session::builder("e2e-reset-post-malformed-chunk")
        .robot(twist_robot())
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .transport(transport)
        .post_reset(ResetSpec::Remote {
            actor: ActorKind::Agent,
            prompt: "stow the tool".into(),
            timeout_ns: 30_000_000_000,
        })
        .build()
        .unwrap();
    *session_cell.lock() = Some(session.clone());

    let mut ep = session.start_episode("stow").unwrap();
    let _ = ep.gate(&[0.0; 6], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    session.inject(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "rollout done".to_owned(),
        at: MonoNs(1_000_000),
    });

    // The window opens and the malformed chunk arrives; give it a real
    // chance to (incorrectly) dispatch before asserting it didn't.
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Reset));
    std::thread::sleep(Duration::from_millis(200));
    assert!(
        send_log.lock().is_empty(),
        "a malformed intervention_chunk must never reach `send`"
    );

    // The window still resolves normally: the malformed chunk didn't
    // corrupt the ring, the pump thread, or the window's own state machine.
    ready_to_complete.store(true, Ordering::SeqCst);
    wait_for(&session, |s| {
        matches!(
            s.episode_state,
            Some(Phase::Terminal(TerminalOutcome::Success))
        )
    });
    assert!(ep.done());
    assert_eq!(ep.outcome(), Some(TerminalOutcome::Success));
    session.shutdown();
}
