//! Claimed-mode agent-chunk intake + jitter-buffer horizon +
//! `ReplanPolicy`: before this landed, `forward_server_msg`'s
//! `InterventionChunk` arm only fed a Reset-mode window; an
//! ordinary (non-reset) `GateMode::Intervention` claim silently dropped
//! agent chunks. Driven through a REAL `ControlPlaneClient` +
//! `InMemoryTransport`, exactly like `reset_window_actuation.rs`.
//!
//! Claim/engage uses `grant_and_engage` (the same direct-FSM-injection seam
//! production's own non-plane `InterventionSource` plugins use — see its own
//! rustdoc) rather than a scripted `ClaimDirective`, since granting the
//! claim itself is not what this task changes; a transport is wired anyway
//! so `forward_server_msg` exists to relay the wire `intervention_chunk`
//! messages this test sends directly once the claim is up.

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use waddle_controlplane::{ClientMsg, InMemoryTransport, ServerMsg};
use waddle_fsm::Phase;
use waddle_gate::gate::GateOutput;
use waddle_runtime::{ControlRegistry, Session, VerbError, grant_and_engage, release_claim};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActorKind, GateMode, Provenance, TerminalOutcome};

/// A 6-dim `BaseTwist` robot (matches `reset_window_actuation.rs`'s
/// `twist_robot`), with a caller-declared `ReplanPolicy` so both variants
/// this task cares about can be exercised end-to-end.
fn twist_robot(replan: pb::ReplanPolicy) -> pb::RobotDescription {
    pb::RobotDescription {
        name: "claimed-chunk-bot".into(),
        robot_id: "claimed-chunk-01".into(),
        cell_id: "cell-claimed-chunk".into(),
        action_space: Some(pb::ActionSpace {
            space: Some(pb::action_space::Space::BaseTwist(pb::BaseTwist {
                frame_id: "base".into(),
                max_linear_mps: None,
                max_angular_radps: None,
            })),
            rate_hz: 50.0,
            chunking: Some(pb::ChunkingSemantics {
                horizon_steps: 8,
                replan: replan as i32,
                interpolation: pb::Interpolation::Hold as i32,
            }),
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

/// A 3-dim `JointPosition` robot for the dims-mismatch test: a `BaseTwist`
/// target always flattens to a fixed 6 values, so it can never itself
/// exercise `TypesError::DimensionMismatch` (see the task report).
fn joint_robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "claimed-chunk-joint-bot".into(),
        robot_id: "claimed-chunk-02".into(),
        cell_id: "cell-claimed-chunk".into(),
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

type TxCell = Arc<Mutex<Option<Sender<ServerMsg>>>>;

/// A transport that registers, then stashes the plane's `Sender<ServerMsg>`
/// so the test can send wire messages directly, whenever it wants — no
/// scripted background thread needed (the claim itself is granted via
/// `grant_and_engage`, a direct FSM injection, not a plane directive).
fn tx_transport() -> (Arc<InMemoryTransport>, TxCell) {
    let cell: TxCell = Arc::new(Mutex::new(None));
    let cell2 = cell.clone();
    let transport = InMemoryTransport::new(move |msg, tx| {
        if let ClientMsg::Register(_) = &msg {
            let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse::default()));
            *cell2.lock() = Some(tx.clone());
        }
    });
    (transport, cell)
}

fn wait_for_tx(cell: &TxCell) -> Sender<ServerMsg> {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if let Some(tx) = cell.lock().clone() {
            return tx;
        }
        assert!(Instant::now() < deadline, "transport never registered");
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn twist_step(x: f64, offset_ns: i64) -> pb::Action {
    pb::Action {
        target: Some(pb::action::Target::BaseTwist(pb::Twist {
            linear: Some(pb::Vec3 { x, y: 0.0, z: 0.0 }),
            angular: Some(pb::Vec3::default()),
        })),
        gripper: None,
        t_offset_ns: offset_ns,
        part: String::new(),
    }
}

fn joint_action(values: Vec<f64>) -> pb::Action {
    pb::Action {
        target: Some(pb::action::Target::JointPosition(pb::JointVector {
            values,
        })),
        gripper: None,
        t_offset_ns: 0,
        part: String::new(),
    }
}

fn send_chunk(tx: &Sender<ServerMsg>, actions: Vec<pb::Action>, seq: u64, t_emitted_ns: i64) {
    let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::InterventionChunk(
            pb::ActionChunk {
                actions,
                horizon_ns: 0,
                t_emitted_ns,
                t_obs_ns: t_emitted_ns,
                seq,
                source_id: "agent-script".into(),
                provenance: None,
            },
        )),
    }));
}

// --- Claimed-mode (ordinary Intervention) agent-chunk actuation -----------

#[test]
fn claimed_mode_agent_chunk_substitutes_five_steps_via_callers_gate() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (transport, tx_cell) = tx_transport();
    let session = Session::builder("e2e-claimed-chunk")
        .robot(twist_robot(pb::ReplanPolicy::Immediate))
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .transport(transport)
        .build()
        .unwrap();

    let mut ep = session.start_episode("claimed-chunk").unwrap();
    let id = ep.id().clone();
    let _ = ep.gate(&[0.0; 6], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(
        &session,
        "claim-agent-chunk",
        "agent-plane",
        ActorKind::Agent,
    );
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));

    let tx = wait_for_tx(&tx_cell);

    // 5-step chunk, steps spaced 40ms apart (well past the 20ms playout
    // delay); values 1.0..5.0 so substitution order is directly observable.
    let steps: Vec<pb::Action> = (0..5)
        .map(|i| twist_step(1.0 + f64::from(i), i64::from(i) * 40_000_000))
        .collect();
    send_chunk(&tx, steps, 1, 0);

    let mut seen = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(5);
    while seen.len() < 5 && Instant::now() < deadline {
        if let GateOutput::Substitute { action, provenance } = ep.gate(&[0.0; 6], None, None) {
            assert_eq!(
                provenance.provenance,
                Provenance::Agent,
                "substituted action must carry the claimant's provenance"
            );
            seen.push(action.values[0]);
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(
        seen,
        vec![1.0, 2.0, 3.0, 4.0, 5.0],
        "chunk steps must substitute in order via the caller's own gate()"
    );

    release_claim(&session, "claim-agent-chunk");
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Passthrough));
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    // MCAP read-back: the substituted steps appear on `/waddle/actions`
    // tagged with agent provenance (`Gate::gate()`'s own record push is the
    // only writer onto that topic, driven by every `Substitute` decision).
    let buf = std::fs::read(dir.path().join(format!("{id}.mcap"))).unwrap();
    let mut agent_action_records = 0;
    for message in mcap::MessageStream::new(&buf).unwrap() {
        let message = message.unwrap();
        if message.channel.topic != waddle_sidecar::mcaprec::ACTIONS_TOPIC {
            continue;
        }
        let chunk = <pb::ActionChunk as prost::Message>::decode(message.data.as_ref()).unwrap();
        if chunk
            .provenance
            .as_ref()
            .is_some_and(|p| p.kind == pb::ProvenanceKind::Agent as i32)
        {
            agent_action_records += 1;
        }
    }
    assert!(
        agent_action_records >= 5,
        "expected the agent-chunk substitutions on /waddle/actions, got {agent_action_records}"
    );
}

/// `REPLAN_POLICY_IMMEDIATE` (declared on the robot's `ChunkingSemantics`):
/// a chunk arriving mid-horizon supersedes the executing one — its
/// still-pending steps never substitute, however long the loop keeps
/// ticking.
#[test]
fn claimed_mode_newer_chunk_supersedes_the_executing_one_under_immediate_replan() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (transport, tx_cell) = tx_transport();
    let session = Session::builder("e2e-claimed-chunk-supersede")
        .robot(twist_robot(pb::ReplanPolicy::Immediate))
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .transport(transport)
        .build()
        .unwrap();

    let mut ep = session.start_episode("claimed-chunk-supersede").unwrap();
    let _ = ep.gate(&[0.0; 6], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(&session, "claim-supersede", "agent-plane", ActorKind::Agent);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));
    let tx = wait_for_tx(&tx_cell);

    // Chunk 1: 4 steps, 300ms apart — ample room to observe step 1
    // substitute, then supersede before steps 2-4 ever become due.
    let chunk1: Vec<pb::Action> = (0..4)
        .map(|i| twist_step(1.0 + f64::from(i), i64::from(i) * 300_000_000))
        .collect();
    send_chunk(&tx, chunk1, 1, 0);

    // Wait for step 1 (value 1.0) to substitute.
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut saw_step1 = false;
    let mut all_values: Vec<f64> = Vec::new();
    while Instant::now() < deadline {
        if let GateOutput::Substitute { action, .. } = ep.gate(&[0.0; 6], None, None) {
            all_values.push(action.values[0]);
            if (action.values[0] - 1.0).abs() < 1e-9 {
                saw_step1 = true;
                break;
            }
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(saw_step1, "chunk 1's first step must substitute");

    // Chunk 2 supersedes mid-horizon on a newer `seq` ALONE — `t_emitted_ns`
    // is left at the proto3 default 0 on both chunks (a wire-legal producer
    // that never bothers to set it), proving `seq` is sufficient on its own
    // and an unset/tied `t_emitted_ns` never wrongly vetoes the supersede
    // (see `jitter.rs`'s staleness rule and its regression test).
    send_chunk(
        &tx,
        vec![twist_step(9.0, 0), twist_step(10.0, 100_000_000)],
        2,
        0,
    );

    // Keep ticking well past chunk 1's remaining steps' due times (900ms
    // total) — none of {2.0, 3.0, 4.0} may ever appear; chunk 2's values
    // must eventually appear.
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut saw_chunk2 = false;
    while Instant::now() < deadline {
        if let GateOutput::Substitute { action, .. } = ep.gate(&[0.0; 6], None, None) {
            all_values.push(action.values[0]);
            if (action.values[0] - 9.0).abs() < 1e-9 || (action.values[0] - 10.0).abs() < 1e-9 {
                saw_chunk2 = true;
            }
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        saw_chunk2,
        "chunk 2's steps must substitute after superseding"
    );
    for stale in [2.0, 3.0, 4.0] {
        assert!(
            !all_values.iter().any(|v| (v - stale).abs() < 1e-9),
            "chunk 1's superseded step {stale} must never substitute, got {all_values:?}"
        );
    }

    release_claim(&session, "claim-supersede");
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Passthrough));
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
}

/// The dims-validation contract, mirroring
/// `e2e.rs::mismatched_action_dims_are_dropped_with_one_fault_per_claim`: a
/// dims-mismatched agent chunk never substitutes, faults exactly once for
/// the whole claim window (not once per chunk), and a subsequent
/// dims-correct chunk still substitutes normally.
#[test]
fn claimed_mode_dims_mismatch_chunk_is_dropped_with_one_fault_per_claim() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (transport, tx_cell) = tx_transport();
    let session = Session::builder("e2e-claimed-chunk-dims")
        .robot(joint_robot())
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .transport(transport)
        .build()
        .unwrap();

    let mut ep = session.start_episode("claimed-chunk-dims").unwrap();
    let id = ep.id().clone();
    let _ = ep.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(&session, "claim-dims", "agent-plane", ActorKind::Agent);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));
    let tx = wait_for_tx(&tx_cell);

    // Wrong arity for a 3-joint space: 2 values, not 3. Sent repeatedly to
    // prove the fault dedupes to once per claim window.
    for seq in 1..=5u64 {
        send_chunk(&tx, vec![joint_action(vec![0.0; 2])], seq, 0);
        std::thread::sleep(Duration::from_millis(5));
    }

    for _ in 0..10 {
        assert!(
            matches!(ep.gate(&[0.0; 3], None, None), GateOutput::Hold),
            "a dims-mismatched agent chunk must never substitute"
        );
    }

    // A matching chunk still substitutes normally: validation doesn't wedge
    // the stream.
    send_chunk(&tx, vec![joint_action(vec![1.0, 2.0, 3.0])], 100, 0);
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut substituted = false;
    while Instant::now() < deadline {
        if let GateOutput::Substitute { action, .. } = ep.gate(&[0.0; 3], None, None) {
            assert_eq!(action.values.as_slice(), &[1.0, 2.0, 3.0]);
            substituted = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(substituted, "a dims-correct chunk must still substitute");

    release_claim(&session, "claim-dims");
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Passthrough));
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    let sidecar_path = dir.path().join(format!("{id}.sidecar.json"));
    let sidecar =
        waddle_sidecar::sidecar_from_json(&std::fs::read_to_string(&sidecar_path).unwrap())
            .unwrap();
    let validation_faults = sidecar
        .events
        .iter()
        .filter(|e| {
            matches!(
                &e.event,
                Some(pb::episode_event::Event::Fault(f))
                    if f.kind == pb::FaultKind::ValidationError as i32
            )
        })
        .count();
    assert_eq!(
        validation_faults, 1,
        "expected exactly one validation fault for the claim window, got {validation_faults}"
    );
}
