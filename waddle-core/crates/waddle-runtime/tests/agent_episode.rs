//! Agent-invited episodes end to end (flag `waddle.v0.agent`, FSM.md §1.5):
//! `Session::run_agent` opens the episode through the normal start path, the
//! invite reaches the plane as an ordinary emission, a scripted plane drives
//! the EXISTING intervention machinery (`ClaimDirective{GRANT, actor AGENT}`
//! → engage → `intervention_chunk` → `MARK_DONE` + `RELEASE`), and the
//! blocked caller's thread + the session's pumps carry the actuation — the
//! claimed-while-stalled BYPASS pump drives the registered `send`, exactly
//! like a remote reset window's claimant (same doctrine, zero new
//! authority). Scripts ride a REAL `ControlPlaneClient` + `InMemoryTransport`
//! like `reset_window_actuation.rs`, never direct FSM injection (except the
//! local `grant_and_engage` bypass-rig test, which mirrors `e2e.rs`'s
//! claimed-while-stalled test on an agent-invited episode).

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use prost::Message as _;
use waddle_controlplane::{ClientMsg, InMemoryTransport, ServerMsg};
use waddle_fsm::Phase;
use waddle_gate::gate::GateOutput;
use waddle_media::{DataTopic, LoopbackMedia};
use waddle_runtime::{
    AgentInvite, AgentTaskKind, ControlRegistry, EpisodeOptions, Session, VerbError,
    grant_and_engage,
};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActorKind, GateMode, Provenance, TerminalOutcome};

/// A 6-dim `BaseTwist` robot (mirrors `reset_window_actuation.rs`): agent
/// `BaseTwist` chunk steps and teleop `Twist` packets both flatten to
/// exactly its declared width, so chunk/intake dims validation never
/// rejects them.
fn twist_robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "agent-episode-bot".into(),
        robot_id: "agent-episode-01".into(),
        cell_id: "cell-agent-episode".into(),
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

/// Block the plane-side script until `pred` holds on the mirror (or the
/// session shuts down, in which case the script gives up quietly — the main
/// thread's own assertions fail the test).
fn script_wait(session: &Session, pred: impl Fn(&waddle_runtime::Status) -> bool) -> bool {
    loop {
        let st = session.status();
        if st.shutdown {
            return false;
        }
        if pred(&st) {
            return true;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
}

/// A transport that registers on connect, then hands the server tx to
/// `on_register` (spawns the plane script) — and tees every episode-event
/// emission carrying an `agent_invite` into `invite_seen`, so the tests can
/// assert the invite reached the (fake) plane as an ordinary emission.
fn scripted_transport(
    invite_seen: Arc<Mutex<Option<pb::AgentInviteEvent>>>,
    on_register: impl Fn(&Sender<ServerMsg>) + Send + Sync + 'static,
) -> Arc<InMemoryTransport> {
    InMemoryTransport::new(move |msg, tx| match &msg {
        ClientMsg::Register(_) => {
            let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse::default()));
            on_register(tx);
        }
        ClientMsg::Gate(gate_msg) => {
            if let Some(pb::gate_client_message::Msg::Event(ev)) = &gate_msg.msg
                && let Some(pb::episode_event::Event::AgentInvite(invite)) = &ev.event
            {
                *invite_seen.lock() = Some(invite.clone());
            }
        }
        _ => {}
    })
}

fn grant_directive(
    claim_id: &str,
    kind: pb::ClaimDirectiveKind,
    actor: pb::ActorKind,
) -> ServerMsg {
    ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::Claim(pb::ClaimDirective {
            kind: kind as i32,
            claim: Some(pb::Claim {
                claim_id: claim_id.to_owned(),
                actor: Some(pb::ActorRef {
                    kind: actor as i32,
                    ..Default::default()
                }),
                source_name: "waddle-agent".into(),
                self_initiated: false,
                ..Default::default()
            }),
            directive_id: None,
        })),
    })
}

fn agent_update(
    episode_id: &str,
    kind: pb::AgentTaskUpdateKind,
    detail: &str,
    rec: &str,
) -> ServerMsg {
    ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::AgentUpdate(
            pb::AgentTaskUpdate {
                episode_id: episode_id.to_owned(),
                kind: kind as i32,
                detail: detail.to_owned(),
                recording_ref: rec.to_owned(),
                directive_id: None,
            },
        )),
    })
}

fn mark_done(episode_id: &str, outcome: pb::TerminalOutcome, reason: &str) -> ServerMsg {
    ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::Episode(
            pb::EpisodeDirective {
                kind: pb::EpisodeDirectiveKind::MarkDone as i32,
                episode_id: episode_id.to_owned(),
                outcome: outcome as i32,
                reason: reason.to_owned(),
                directive_id: None,
            },
        )),
    })
}

fn base_twist_chunk(x: f64, seq: u64) -> ServerMsg {
    ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::InterventionChunk(
            pb::ActionChunk {
                actions: vec![pb::Action {
                    target: Some(pb::action::Target::BaseTwist(pb::Twist {
                        linear: Some(pb::Vec3 { x, y: 0.0, z: 0.0 }),
                        angular: Some(pb::Vec3::default()),
                    })),
                    gripper: None,
                    t_offset_ns: 0,
                    part: String::new(),
                }],
                horizon_ns: 0,
                t_emitted_ns: 0,
                t_obs_ns: 0,
                seq,
                source_id: "agent-script".into(),
                provenance: None,
            },
        )),
    })
}

// --- Happy path: invite → GRANT+engage → chunks via BYPASS → MARK_DONE ----

#[test]
fn run_agent_returns_success_with_recording_ref_from_completed_update() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session_cell: Arc<Mutex<Option<Session>>> = Arc::new(Mutex::new(None));
    let invite_seen: Arc<Mutex<Option<pb::AgentInviteEvent>>> = Arc::new(Mutex::new(None));

    let cell_for_script = session_cell.clone();
    let log_for_script = send_log.clone();
    let transport = scripted_transport(invite_seen.clone(), move |tx| {
        let cell = cell_for_script.clone();
        let log = log_for_script.clone();
        let tx = tx.clone();
        std::thread::spawn(move || {
            let session = loop {
                if let Some(s) = cell.lock().clone() {
                    break s;
                }
                std::thread::sleep(Duration::from_millis(2));
            };
            // The invited agent claims once the episode is RUNNING
            // (`run_agent` injects `Start` — the blocked caller never
            // ticks; E7 admits engage from RUNNING only).
            if !script_wait(&session, |s| {
                s.agent_invited && matches!(s.episode_state, Some(Phase::Running))
            }) {
                return;
            }
            let episode_id = session.status().episode_id.unwrap().to_string();
            // GRANT + engage: the one existing intervention directive.
            let _ = tx.send(grant_directive(
                "agent-claim-1",
                pb::ClaimDirectiveKind::Grant,
                pb::ActorKind::Agent,
            ));
            // A chunk buffers from claim-granted onward (same intake gate
            // as a teleop packet); the caller is blocked, so the
            // claimed-while-stalled BYPASS pump is what drives it to the
            // registered `send`.
            if !script_wait(&session, |s| s.claim_active) {
                return;
            }
            let _ = tx.send(base_twist_chunk(0.5, 1));
            // Wait until the chunk actually reached `send` before finishing
            // the task — a real agent works, then reports.
            loop {
                if session.status().shutdown {
                    return;
                }
                if log.lock().iter().any(|(p, v)| {
                    *p == Provenance::Agent && v.first().is_some_and(|x| (*x - 0.5).abs() < 1e-9)
                }) {
                    break;
                }
                std::thread::sleep(Duration::from_millis(2));
            }
            // COMPLETED before MARK_DONE (the expected plane ordering — the
            // pump processes them in arrival order, so the recording_ref is
            // retained before the terminal wakes the blocked caller).
            let _ = tx.send(agent_update(
                &episode_id,
                pb::AgentTaskUpdateKind::Completed,
                "cups stacked",
                "rec-123",
            ));
            let _ = tx.send(mark_done(
                &episode_id,
                pb::TerminalOutcome::Success,
                "agent task complete",
            ));
            // The plane's trailing RELEASE: the terminal close already
            // released the claim, so the FSM rejects this inert — sent
            // anyway to mirror the real sequence.
            let _ = tx.send(grant_directive(
                "agent-claim-1",
                pb::ClaimDirectiveKind::Release,
                pb::ActorKind::Agent,
            ));
        });
    });

    let session = Session::builder("agent-happy")
        .robot(twist_robot())
        .control(registry(&send_log))
        .transport(transport)
        .build()
        .unwrap();
    *session_cell.lock() = Some(session.clone());

    let result = session
        .run_agent(
            "clear the table and stack the cups",
            30_000_000_000,
            EpisodeOptions::default(),
        )
        .unwrap();
    assert_eq!(result.outcome, TerminalOutcome::Success);
    assert_eq!(result.recording_ref.as_deref(), Some("rec-123"));
    assert_eq!(result.detail, "cups stacked");

    // The invite reached the (fake) plane as an ordinary emission.
    let invite = invite_seen
        .lock()
        .clone()
        .expect("the agent_invite emission must reach the plane");
    assert_eq!(invite.prompt, "clear the table and stack the cups");
    assert_eq!(invite.timeout_ns, 30_000_000_000);

    // The agent's chunk was dispatched by the BYPASS pump with agent
    // provenance (the mirror's claim provenance at pop time).
    assert!(
        send_log
            .lock()
            .iter()
            .any(|(p, v)| *p == Provenance::Agent
                && v.first().is_some_and(|x| (*x - 0.5).abs() < 1e-9)),
        "the agent chunk never reached the registered `send`"
    );

    // The mirror observed (and latched) the engage; the retained update is
    // the COMPLETED one.
    let status = session.status();
    assert!(status.agent_invited);
    assert!(status.agent_engaged);
    assert_eq!(
        status.agent_task.as_ref().map(|t| t.kind),
        Some(AgentTaskKind::Completed)
    );
    session.shutdown();
}

// --- Invite timeout: nobody engages; the FSM's timer aborts ---------------

#[test]
fn run_agent_invite_timeout_aborts() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    // No transport at all: nobody can ever engage, so the invite deadline
    // (E25, armed at open and serviced by the reducer's ordinary timer
    // wheel) must fire from real elapsed time — not an injected
    // `TimerFired` shortcut.
    let session = Session::builder("agent-timeout")
        .robot(twist_robot())
        .control(registry(&send_log))
        .build()
        .unwrap();

    let result = session
        .run_agent("nobody home", 150_000_000, EpisodeOptions::default())
        .unwrap();
    assert_eq!(result.outcome, TerminalOutcome::Abort);
    assert!(result.recording_ref.is_none());
    assert!(result.detail.is_empty());
    assert!(send_log.lock().is_empty(), "nothing may actuate");
    let status = session.status();
    assert!(status.agent_invited && !status.agent_engaged);
    session.shutdown();
}

// --- DENIED before engage: aborts with the plane's detail (E26) -----------

#[test]
fn run_agent_denied_before_engage_aborts_with_detail() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session_cell: Arc<Mutex<Option<Session>>> = Arc::new(Mutex::new(None));
    let invite_seen: Arc<Mutex<Option<pb::AgentInviteEvent>>> = Arc::new(Mutex::new(None));

    let cell_for_script = session_cell.clone();
    let transport = scripted_transport(invite_seen, move |tx| {
        let cell = cell_for_script.clone();
        let tx = tx.clone();
        std::thread::spawn(move || {
            let session = loop {
                if let Some(s) = cell.lock().clone() {
                    break s;
                }
                std::thread::sleep(Duration::from_millis(2));
            };
            if !script_wait(&session, |s| {
                s.agent_invited && matches!(s.episode_state, Some(Phase::Running))
            }) {
                return;
            }
            let episode_id = session.status().episode_id.unwrap().to_string();
            let _ = tx.send(agent_update(
                &episode_id,
                pb::AgentTaskUpdateKind::Denied,
                "no agent available for this cell",
                "",
            ));
        });
    });

    let session = Session::builder("agent-denied-pre")
        .robot(twist_robot())
        .control(registry(&send_log))
        .transport(transport)
        .build()
        .unwrap();
    *session_cell.lock() = Some(session.clone());

    let result = session
        .run_agent("stack the cups", 30_000_000_000, EpisodeOptions::default())
        .unwrap();
    assert_eq!(result.outcome, TerminalOutcome::Abort);
    assert_eq!(result.detail, "no agent available for this cell");
    assert!(result.recording_ref.is_none());
    let status = session.status();
    assert!(!status.agent_engaged, "no claim ever engaged");
    assert!(send_log.lock().is_empty(), "nothing may actuate");
    session.shutdown();
}

// --- DENIED after engage: inert (E26b); the task still finishes -----------

#[test]
fn run_agent_denied_after_engage_is_inert() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session_cell: Arc<Mutex<Option<Session>>> = Arc::new(Mutex::new(None));
    let invite_seen: Arc<Mutex<Option<pb::AgentInviteEvent>>> = Arc::new(Mutex::new(None));

    let cell_for_script = session_cell.clone();
    let transport = scripted_transport(invite_seen, move |tx| {
        let cell = cell_for_script.clone();
        let tx = tx.clone();
        std::thread::spawn(move || {
            let session = loop {
                if let Some(s) = cell.lock().clone() {
                    break s;
                }
                std::thread::sleep(Duration::from_millis(2));
            };
            if !script_wait(&session, |s| {
                s.agent_invited && matches!(s.episode_state, Some(Phase::Running))
            }) {
                return;
            }
            let episode_id = session.status().episode_id.unwrap().to_string();
            let _ = tx.send(grant_directive(
                "agent-claim-2",
                pb::ClaimDirectiveKind::Grant,
                pb::ActorKind::Agent,
            ));
            // The engage latches `agent_engaged` (E7, §1.5) — from here a
            // DENIED is E26b's recorded-only rejection, never an abort.
            if !script_wait(&session, |s| s.agent_engaged) {
                return;
            }
            let _ = tx.send(agent_update(
                &episode_id,
                pb::AgentTaskUpdateKind::Denied,
                "too late",
                "",
            ));
            // Give an (incorrect) abort a real chance to land before
            // finishing the task: if the DENIED transitioned anything, the
            // MARK_DONE below arrives at a terminal episode and `run_agent`
            // reports ABORT instead of SUCCESS.
            std::thread::sleep(Duration::from_millis(300));
            let _ = tx.send(mark_done(
                &episode_id,
                pb::TerminalOutcome::Success,
                "agent task complete",
            ));
        });
    });

    let session = Session::builder("agent-denied-post")
        .robot(twist_robot())
        .control(registry(&send_log))
        .transport(transport)
        .build()
        .unwrap();
    *session_cell.lock() = Some(session.clone());

    let result = session
        .run_agent("stack the cups", 30_000_000_000, EpisodeOptions::default())
        .unwrap();
    assert_eq!(
        result.outcome,
        TerminalOutcome::Success,
        "a DENIED after engage must be inert (E26b)"
    );
    // The retained update IS the late DENIED (retention is "last update
    // addressed to this episode", not outcome-filtered) — observability of
    // the rejection, never authority over the episode.
    assert_eq!(result.detail, "too late");
    let status = session.status();
    assert_eq!(
        status.agent_task.as_ref().map(|t| t.kind),
        Some(AgentTaskKind::Denied)
    );
    session.shutdown();
}

// --- E24: the caller's own gate() ticks never dispatch --------------------

#[test]
fn caller_gate_ticks_noop_during_unengaged_agent_episode() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session = Session::builder("agent-caller-noop")
        .robot(twist_robot())
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .build()
        .unwrap();

    // Setting `agent_invite` directly (not through `run_agent`) opens the
    // episode without blocking: the caller keeps the handle — and its own
    // ticks must never dispatch while no claim is engaged (E24).
    let mut ep = session
        .start_episode_with(
            "the agent will drive",
            EpisodeOptions {
                agent_invite: Some(AgentInvite {
                    prompt: "the agent will drive".into(),
                    timeout_ns: 30_000_000_000,
                }),
                ..Default::default()
            },
        )
        .unwrap();
    let id = ep.id().clone();
    let status = session.status();
    assert!(status.agent_invited && !status.agent_engaged);

    // The first tick still drives READY → RUNNING (E6) — E24 suppresses
    // dispatch, never the caller's liveness signal.
    for _ in 0..3 {
        assert!(
            matches!(
                ep.gate(&[0.1; 6], None, Some(&[0.0; 6])),
                GateOutput::Noop { .. }
            ),
            "a caller tick in an unengaged agent episode must be a Noop (E24)"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    assert!(matches!(
        ep.gate(&[0.1; 6], None, Some(&[0.0; 6])),
        GateOutput::Noop { .. }
    ));
    assert!(send_log.lock().is_empty(), "nothing may actuate");

    ep.terminate(TerminalOutcome::Abort, "test over");
    session.shutdown();

    // The ticks landed on /waddle/actions as AGENT_EPISODE NoopMarkers —
    // the same surface NOOP_REASON_RESET_ACTIVE uses (reducer projection of
    // `GateDecision::AgentEpisode`).
    let buf = std::fs::read(dir.path().join(format!("{id}.mcap"))).unwrap();
    let mut agent_episode_noops = 0;
    for message in mcap::MessageStream::new(&buf).unwrap() {
        let message = message.unwrap();
        if message.channel.topic != waddle_sidecar::mcaprec::ACTIONS_TOPIC {
            continue;
        }
        let chunk = pb::ActionChunk::decode(message.data.as_ref()).unwrap();
        for action in &chunk.actions {
            if let Some(pb::action::Target::Noop(marker)) = &action.target
                && marker.reason == pb::NoopReason::AgentEpisode as i32
            {
                agent_episode_noops += 1;
            }
        }
    }
    assert!(
        agent_episode_noops > 0,
        "expected AGENT_EPISODE NoopMarkers on /waddle/actions, got none"
    );
}

// --- Stalled caller during an engaged agent episode: BYPASS drives send ---

/// The existing claimed-while-stalled rig (`e2e.rs`) replayed on an
/// agent-invited episode: once the AGENT claim engages, ordinary
/// intervention semantics — BYPASS eligibility included — apply unchanged
/// (E24 is scoped to the unengaged phases). The caller ticks (Noops), the
/// agent engages locally (`grant_and_engage`, C8 admits AGENT), the caller
/// stalls, and the BYPASS pump must drive the registered `send` from the
/// intervention ring with the claim's agent provenance.
#[test]
fn stalled_caller_bypass_drives_send_during_engaged_agent_episode() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (media, far) = LoopbackMedia::new();
    let session = Session::builder("agent-bypass")
        .robot(twist_robot())
        .control(registry(&send_log))
        .media(media)
        .build()
        .unwrap();

    let mut ep = session
        .start_episode_with(
            "agent bypass",
            EpisodeOptions {
                agent_invite: Some(AgentInvite {
                    prompt: "agent bypass".into(),
                    timeout_ns: 30_000_000_000,
                }),
                ..Default::default()
            },
        )
        .unwrap();
    for _ in 0..5 {
        // E24 Noops — but they still feed the stall detector's last-tick
        // clock, so the stall below is a REAL detected stall, not the
        // never-ticked fast path.
        assert!(matches!(
            ep.gate(&[0.0; 6], None, None),
            GateOutput::Noop { .. }
        ));
        std::thread::sleep(Duration::from_millis(2));
    }
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(
        &session,
        "agent-claim-local",
        "waddle-agent",
        ActorKind::Agent,
    );
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));
    wait_for(&session, |s| s.agent_engaged);

    // The caller stalls (no more ticks); the agent streams. BYPASS engages
    // and the pump drives `send` directly, tagged with the claim's agent
    // provenance.
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
    while session.status().gate_mode != Some(GateMode::Bypass) {
        push_pose(0.7);
        assert!(Instant::now() < deadline, "bypass never engaged");
        std::thread::sleep(Duration::from_millis(10));
    }
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if send_log.lock().iter().any(|(p, _)| *p == Provenance::Agent) {
            break;
        }
        push_pose(0.7);
        assert!(Instant::now() < deadline, "bypass pump never sent");
        std::thread::sleep(Duration::from_millis(10));
    }

    // A late caller tick observes a NOOP with the claimant's provenance
    // (the spectator contract, unchanged in an agent episode).
    match ep.gate(&[0.0; 6], None, None) {
        GateOutput::Noop { provenance } => {
            assert_eq!(provenance.provenance, Provenance::Agent);
        }
        other => panic!("expected NOOP for the stalled loop's tick, got {other:?}"),
    }
    session.shutdown();
}
