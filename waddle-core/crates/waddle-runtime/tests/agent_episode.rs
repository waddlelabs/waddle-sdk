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
    AgentInvite, AgentTaskKind, ControlRegistry, EpisodeOptions, ProprioReport, ResetSpec,
    RuntimeError, Session, VerbError, grant_and_engage,
};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActorKind, GateMode, Provenance, TerminalOutcome};

/// The forward speeds the scripted agent commands, one chunk each. Three,
/// not one, so "every dispatch produced exactly one recorded action" is a
/// claim about a count and not about a boolean.
const AGENT_CHUNK_XS: [f64; 3] = [0.5, 0.6, 0.7];

/// The `source_id` the bypass pump's recorded dispatches carry. Spelled out
/// here rather than imported: it is a WIRE value consumers key on, so the
/// test pins the string, not the constant.
const BYPASS_PUMP_SOURCE: &str = "waddle.bypass-pump";

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

/// The id a real plane stamps on the agent it hosts. Fixed here so the
/// recording assertions can name it.
const AGENT_ACTOR_ID: &str = "agent:ws-1@plane";

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
                // A plane grants with a FULL ActorRef; the SDK must carry it
                // whole onto the claim events it journals.
                actor: Some(pb::ActorRef {
                    kind: actor as i32,
                    id: AGENT_ACTOR_ID.to_owned(),
                    display_name: "Waddle agent".to_owned(),
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
            // The robot keeps reporting its own state while the agent
            // drives — the toy example's `RobotPump` shape, and the only
            // proprioception an agent-invited episode can have (its caller
            // is blocked inside `run_agent` and never ticks `gate`).
            session.report_proprio(ProprioReport {
                joint_vel: Some(vec![0.01, 0.02, 0.03, 0.04, 0.05, 0.06]),
                ee_pose: None,
                gripper: Some(0.25),
            });
            // One chunk at a time, each awaited before the next is sent.
            // Under the declared IMMEDIATE replan a chunk arriving while
            // another is still pending SUPERSEDES it (by design), so a burst
            // would prove nothing about how many dispatches get recorded —
            // and a real agent streams this way anyway: it acts, observes,
            // acts again.
            for (i, x) in AGENT_CHUNK_XS.iter().enumerate() {
                let _ = tx.send(base_twist_chunk(*x, i as u64 + 1));
                loop {
                    if session.status().shutdown {
                        return;
                    }
                    let arrived = log.lock().iter().any(|(p, v)| {
                        *p == Provenance::Agent && v.first().is_some_and(|v| (*v - *x).abs() < 1e-9)
                    });
                    if arrived {
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(2));
                }
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

    let dir = tempfile::tempdir().unwrap();
    let session = Session::builder("agent-happy")
        .robot(twist_robot())
        .control(registry(&send_log))
        .transport(transport)
        .recording_dir(dir.path())
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

    // The agent's chunks were dispatched by the BYPASS pump with agent
    // provenance (the mirror's claim provenance at pop time).
    for x in AGENT_CHUNK_XS {
        assert!(
            send_log.lock().iter().any(|(p, v)| *p == Provenance::Agent
                && v.first().is_some_and(|v| (*v - x).abs() < 1e-9)),
            "the agent chunk {x} never reached the registered `send`"
        );
    }

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

    // --- What the recording says happened, and who did it ----------------
    //
    // The exit criterion for an agent-driven episode is a recording with
    // correct PROVENANCE: it must name the agent that drove, not merely note
    // that "something intervened".
    let sidecar = read_sidecar(dir.path(), result.episode_id.as_str());

    // The claim span carries the plane's ActorRef whole — kind AND the id it
    // stamped. `sourceName` names the stream, never the actor.
    let claim = sidecar
        .claims
        .iter()
        .find_map(|c| c.claim.as_ref())
        .expect("the agent's claim must appear as a claim span");
    let claim_actor = claim
        .actor
        .as_ref()
        .expect("a claim span with no actor cannot be attributed to anyone");
    assert_eq!(claim_actor.kind, pb::ActorKind::Agent as i32);
    assert_eq!(claim_actor.id, AGENT_ACTOR_ID);
    assert_eq!(claim.source_name, "waddle-agent");

    // Same for the journaled claim EVENT the plane and any judge replay.
    let journaled = sidecar
        .events
        .iter()
        .filter_map(|e| match &e.event {
            Some(pb::episode_event::Event::Claim(c)) => c.claim.as_ref(),
            _ => None,
        })
        .collect::<Vec<_>>();
    assert!(
        !journaled.is_empty(),
        "the episode journal must carry claim events"
    );
    for claim in journaled {
        let actor = claim
            .actor
            .as_ref()
            .expect("every journaled claim event names its claimant");
        assert_eq!(actor.kind, pb::ActorKind::Agent as i32);
        assert_eq!(actor.id, AGENT_ACTOR_ID);
    }

    // The provenance spans say AGENT — they used to say TELEOP for every
    // claimed span, whoever was actually driving.
    let claimed_spans: Vec<&pb::ProvenanceTag> = sidecar
        .provenance
        .iter()
        .filter_map(|p| p.tag.as_ref())
        .filter(|t| t.kind != pb::ProvenanceKind::Policy as i32)
        .collect();
    assert!(
        !claimed_spans.is_empty(),
        "an engaged agent claim must open a non-policy provenance span"
    );
    for tag in claimed_spans {
        assert_eq!(
            tag.kind,
            pb::ProvenanceKind::Agent as i32,
            "an agent-driven span must not be labeled teleop"
        );
        assert_eq!(
            tag.actor.as_ref().map(|a| a.id.as_str()),
            Some(AGENT_ACTOR_ID)
        );
    }
    // --- What the recording CONTAINS -------------------------------------
    //
    // The caller of an agent-invited episode never ticks `gate()` (E24), and
    // actions/observations used to be written only on the gate-tick path —
    // so the recording of the run above came out with zero actions and zero
    // observations. An episode with neither cannot be judged or trained on;
    // it is not a data product at all.
    let (actions, observations) = read_mcap(dir.path(), result.episode_id.as_str());

    let dispatched = send_log.lock().len();
    assert_eq!(
        dispatched,
        AGENT_CHUNK_XS.len(),
        "the script drives exactly one send per chunk"
    );
    let pump_rows: Vec<&pb::ActionChunk> = actions
        .iter()
        .filter(|c| c.source_id == BYPASS_PUMP_SOURCE)
        .collect();
    assert_eq!(
        pump_rows.len(),
        dispatched,
        "every bypass dispatch must produce exactly one recorded action"
    );
    // `seq` is monotone per stream, and the pump is its own stream.
    let seqs: Vec<u64> = pump_rows.iter().map(|c| c.seq).collect();
    let mut sorted = seqs.clone();
    sorted.sort_unstable();
    sorted.dedup();
    assert_eq!(seqs, sorted, "the pump's recorded seq must be monotone");

    for (row, x) in pump_rows.iter().zip(AGENT_CHUNK_XS) {
        let tag = row
            .provenance
            .as_ref()
            .expect("a recorded action with no provenance names no driver");
        assert_eq!(tag.kind, pb::ProvenanceKind::Agent as i32);
        assert_eq!(
            tag.actor.as_ref().map(|a| a.id.as_str()),
            Some(AGENT_ACTOR_ID)
        );
        // The action the robot was actually asked to perform, decoded into
        // the declared space.
        let Some(pb::action::Target::BaseTwist(twist)) = &row.actions[0].target else {
            panic!(
                "expected the declared BaseTwist space, got {:?}",
                row.actions[0].target
            );
        };
        assert!((twist.linear.as_ref().unwrap().x - x).abs() < 1e-9);
    }

    // Proprioception reported while the agent drove is recorded, even though
    // no gate tick ever carried an obs.
    let reported: Vec<&pb::ProprioSample> = observations
        .iter()
        .filter_map(|o| match &o.payload {
            Some(pb::observation_update::Payload::Proprio(p)) => Some(p),
            _ => None,
        })
        .collect();
    assert!(
        !reported.is_empty(),
        "an agent-driven episode recorded no observations"
    );
    assert!(
        reported
            .iter()
            .any(|p| p.gripper == Some(0.25)
                && p.joint_vel == vec![0.01, 0.02, 0.03, 0.04, 0.05, 0.06]),
        "the reported proprio sample never reached /waddle/observations"
    );
}

/// The episode's recorded actions and observations, in file order.
fn read_mcap(
    dir: &std::path::Path,
    episode_id: &str,
) -> (Vec<pb::ActionChunk>, Vec<pb::ObservationUpdate>) {
    let buf = std::fs::read(dir.join(format!("{episode_id}.mcap"))).expect("mcap must exist");
    let mut actions = Vec::new();
    let mut observations = Vec::new();
    for message in mcap::MessageStream::new(&buf).unwrap() {
        let message = message.unwrap();
        match message.channel.topic.as_str() {
            t if t == waddle_sidecar::mcaprec::ACTIONS_TOPIC => {
                actions.push(pb::ActionChunk::decode(message.data.as_ref()).unwrap());
            }
            t if t == waddle_sidecar::mcaprec::OBSERVATIONS_TOPIC => {
                observations.push(pb::ObservationUpdate::decode(message.data.as_ref()).unwrap());
            }
            _ => {}
        }
    }
    (actions, observations)
}

/// The episode's sidecar, read back from the recording directory.
fn read_sidecar(dir: &std::path::Path, episode_id: &str) -> pb::Sidecar {
    let path = dir.join(format!("{episode_id}.sidecar.json"));
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("sidecar {} must exist: {e}", path.display()));
    waddle_sidecar::sidecar_from_json(&text).expect("sidecar must parse")
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

// --- Invite timeout during a slow pre-reset: still an agent outcome -------

/// E25 can fire while `start_episode_with` is still blocked in RESETTING
/// (the pre-reset hook outlives the invite deadline). That close is the
/// invite's own (`agent_invite_aborted`, FSM.md §1.5), so `run_agent`
/// recovers it from the mirror as a normal ABORT outcome instead of
/// surfacing the start path's `ResetFailed`.
#[test]
fn run_agent_recovers_invite_timeout_during_slow_pre_reset() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session = Session::builder("agent-timeout-slow-reset")
        .robot(twist_robot())
        .control(registry(&send_log))
        .pre_reset(ResetSpec::Hook(Arc::new(|_| {
            // Outlive the 100 ms invite deadline; the reducer's timer wheel
            // services E25 on its own thread while this blocks the caller.
            std::thread::sleep(Duration::from_millis(400));
            (true, Some(true))
        })))
        .build()
        .unwrap();

    let result = session
        .run_agent("nobody home", 100_000_000, EpisodeOptions::default())
        .unwrap();
    assert_eq!(result.outcome, TerminalOutcome::Abort);
    assert!(result.recording_ref.is_none());
    assert!(send_log.lock().is_empty(), "nothing may actuate");
    let status = session.status();
    assert!(status.agent_invite_aborted, "E25 latches the invite abort");
    session.shutdown();
}

// --- A genuine pre-reset failure is an ERROR, never an agent outcome ------

/// The recovery above is scoped to E25/E26 exactly: a pre-reset hook that
/// FAILS (E5) on an agent-invited episode must surface the same
/// `RuntimeError::ResetFailed` the non-agent start path surfaces — a
/// customer with a broken reset rig must see the error channel, not a
/// normal-looking ABORT with empty detail.
#[test]
fn run_agent_surfaces_pre_reset_failure_as_error() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session = Session::builder("agent-reset-failure")
        .robot(twist_robot())
        .control(registry(&send_log))
        .pre_reset(ResetSpec::Hook(Arc::new(|_| (false, Some(false)))))
        .build()
        .unwrap();

    let result = session.run_agent("stack the cups", 30_000_000_000, EpisodeOptions::default());
    assert!(
        matches!(result, Err(RuntimeError::ResetFailed(_))),
        "a genuine E5 reset failure must surface as ResetFailed, got {result:?}"
    );
    let status = session.status();
    assert!(
        !status.agent_invite_aborted,
        "E5 must never latch the invite abort"
    );
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
