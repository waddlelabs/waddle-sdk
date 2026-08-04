//! Part-addressed control end to end (`Action.part` / `ProprioSample.part`,
//! flag `waddle.v0.parts`): one arm of a bimanual cell is intervened on, and
//! reported on, without inventing values for the other.
//!
//! Driven through a REAL `ControlPlaneClient` + `InMemoryTransport`, exactly
//! like `claimed_chunk_intake.rs` — and for the same reason the flag matters
//! here at all: the transport's `Registered` response is what decides whether
//! this connection negotiated `waddle.v0.parts`, so a test that wants the
//! pre-flag reading simply accepts nothing.
//!
//! Claim/engage uses `grant_and_engage` (the direct-FSM-injection seam, see
//! its own rustdoc) — granting a claim is not what this file exercises.

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
use waddle_types::{ActorKind, GateMode, TerminalOutcome};

/// The registry row this file is about (docs/VERSIONING.md §3). Spelled out
/// rather than imported: a test that pins a wire-visible flag name must fail
/// if the constant behind it is ever renamed.
const PARTS_FLAG: &str = "waddle.v0.parts";

/// Seven rows per arm — six joints plus the gripper folded in as the last
/// row (`GripperSpec.parallel{action_dim: -1}`), which is the canonical
/// bimanual declaration (`fixtures/wire/robot_description_bimanual_composite.json`)
/// and why this stage needs no per-part gripper sidechannel.
const ARM_DIMS: usize = 7;

fn arm(prefix: &str) -> pb::ActionSpace {
    pb::ActionSpace {
        space: Some(pb::action_space::Space::JointPosition(pb::JointPosition {
            joints: (0..ARM_DIMS)
                .map(|i| pb::JointDescriptor {
                    name: format!("{prefix}{i}"),
                    min_position: Some(-3.05),
                    max_position: Some(3.05),
                    ..Default::default()
                })
                .collect(),
        })),
        rate_hz: 50.0,
        chunking: None,
        gripper: None,
    }
}

fn grants() -> Vec<pb::Grant> {
    vec![
        pb::Grant {
            verb: pb::Verb::Hold as i32,
            declared_latency_bound_ns: Some(40_000_000),
            ..Default::default()
        },
        pb::Grant {
            verb: pb::Verb::Send as i32,
            send_interfaces: vec![pb::SpaceKind::JointPosition as i32],
            ..Default::default()
        },
    ]
}

/// The declaration this whole file is about: two named 7-row parts, in
/// declaration order (which IS the concatenated 14-row layout).
fn bimanual_robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "yam-bimanual".into(),
        robot_id: "yam-01".into(),
        cell_id: "cell-parts".into(),
        action_space: Some(pb::ActionSpace {
            space: Some(pb::action_space::Space::Composite(pb::Composite {
                parts: vec![
                    pb::composite::Part {
                        name: "left".into(),
                        space: Some(arm("l")),
                    },
                    pb::composite::Part {
                        name: "right".into(),
                        space: Some(arm("r")),
                    },
                ],
            })),
            rate_hz: 50.0,
            chunking: Some(pb::ChunkingSemantics {
                horizon_steps: 20,
                replan: pb::ReplanPolicy::Immediate as i32,
                interpolation: pb::Interpolation::Hold as i32,
            }),
            gripper: None,
        }),
        grants: grants(),
        ..Default::default()
    }
}

/// A single-part (non-`Composite`) declaration: it has no addressable parts
/// at all, which is exactly why it must not declare the flag.
fn single_part_robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "one-arm".into(),
        robot_id: "one-arm-01".into(),
        cell_id: "cell-parts".into(),
        action_space: Some(arm("j")),
        grants: grants(),
        ..Default::default()
    }
}

/// Every step the declared `send` verb was handed, as (part, values).
type SendLog = Arc<Mutex<Vec<(Option<String>, Vec<f64>)>>>;

fn registry(send_log: &SendLog) -> ControlRegistry {
    let log = send_log.clone();
    ControlRegistry {
        send: Some(Arc::new(
            move |chunk: &waddle_types::ActionChunk| -> Result<(), VerbError> {
                for step in &chunk.steps {
                    log.lock().push((
                        step.part.as_ref().map(|p| p.to_string()),
                        step.values.to_vec(),
                    ));
                }
                Ok(())
            },
        )),
        hold: Some(Arc::new(|| Ok(()))),
        resume: Some(Arc::new(|| Ok(()))),
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
type Declared = Arc<Mutex<Vec<String>>>;

struct Rig {
    transport: Arc<InMemoryTransport>,
    tx: TxCell,
    /// The `feature_flags` the SDK declared on its `RegisterRequest`.
    declared: Declared,
}

/// A transport that answers Register by ACCEPTING exactly the flags the SDK
/// declared (or, with `accept_parts` false, everything except
/// `waddle.v0.parts` — the pre-flag connection), and stashes the plane's
/// `Sender<ServerMsg>` so a test can push wire messages whenever it wants.
fn rig(accept_parts: bool) -> Rig {
    let tx: TxCell = Arc::new(Mutex::new(None));
    let declared: Declared = Arc::new(Mutex::new(Vec::new()));
    let (tx_in, declared_in) = (tx.clone(), declared.clone());
    let transport = InMemoryTransport::new(move |msg, plane_tx: &Sender<ServerMsg>| {
        if let ClientMsg::Register(req) = &msg {
            *declared_in.lock() = req.feature_flags.clone();
            let accepted = req
                .feature_flags
                .iter()
                .filter(|f| accept_parts || f.as_str() != PARTS_FLAG)
                .cloned()
                .collect();
            let _ = plane_tx.send(ServerMsg::Registered(pb::RegisterResponse {
                accepted_feature_flags: accepted,
                ..Default::default()
            }));
            *tx_in.lock() = Some(plane_tx.clone());
        }
    });
    Rig {
        transport,
        tx,
        declared,
    }
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

/// One wire action addressing a single declared part by name.
fn part_action(part: &str, values: Vec<f64>, offset_ns: i64) -> pb::Action {
    pb::Action {
        target: Some(pb::action::Target::JointPosition(pb::JointVector {
            values,
        })),
        gripper: None,
        t_offset_ns: offset_ns,
        part: part.into(),
    }
}

fn send_chunk(tx: &Sender<ServerMsg>, actions: Vec<pb::Action>, seq: u64) {
    let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::InterventionChunk(
            pb::ActionChunk {
                actions,
                horizon_ns: 0,
                t_emitted_ns: 0,
                t_obs_ns: 0,
                seq,
                source_id: "agent-script".into(),
                provenance: None,
            },
        )),
    }));
}

fn actions_topic(dir: &std::path::Path, episode_id: &str) -> Vec<pb::ActionChunk> {
    let buf = std::fs::read(dir.join(format!("{episode_id}.mcap"))).unwrap();
    let mut chunks = Vec::new();
    for message in mcap::MessageStream::new(&buf).unwrap() {
        let message = message.unwrap();
        if message.channel.topic == waddle_sidecar::mcaprec::ACTIONS_TOPIC {
            chunks
                .push(<pb::ActionChunk as prost::Message>::decode(message.data.as_ref()).unwrap());
        }
    }
    chunks
}

fn validation_faults(dir: &std::path::Path, episode_id: &str) -> Vec<pb::Fault> {
    let path = dir.join(format!("{episode_id}.sidecar.json"));
    let sidecar =
        waddle_sidecar::sidecar_from_json(&std::fs::read_to_string(&path).unwrap()).unwrap();
    sidecar
        .events
        .iter()
        .filter_map(|e| match &e.event {
            Some(pb::episode_event::Event::Fault(f))
                if f.kind == pb::FaultKind::ValidationError as i32 =>
            {
                Some(f.clone())
            }
            _ => None,
        })
        .collect()
}

// --- Declaration ----------------------------------------------------------

/// VERSIONING.md's declaration rule: the flag is declared **iff** the
/// declared action space is `Composite`. A single-part robot has no
/// addressable part, so declaring it would claim a behavior the session can
/// never exhibit (the `waddle.v0.obs.stills` rule, applied to parts).
#[test]
fn the_parts_flag_is_declared_only_for_a_composite_declaration() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));

    let composite = rig(true);
    let session = Session::builder("parts-declare-composite")
        .robot(bimanual_robot())
        .control(registry(&send_log))
        .transport(composite.transport.clone())
        .build()
        .unwrap();
    let _ = wait_for_tx(&composite.tx);
    assert!(
        composite.declared.lock().iter().any(|f| f == PARTS_FLAG),
        "a Composite declaration has addressable parts and must declare the flag: {:?}",
        composite.declared.lock()
    );
    wait_for(&session, |s| s.parts_negotiated);
    session.shutdown();

    let single = rig(true);
    let session = Session::builder("parts-declare-single")
        .robot(single_part_robot())
        .control(registry(&send_log))
        .transport(single.transport.clone())
        .build()
        .unwrap();
    let _ = wait_for_tx(&single.tx);
    assert!(
        !single.declared.lock().iter().any(|f| f == PARTS_FLAG),
        "a single-part declaration has no part to address: {:?}",
        single.declared.lock()
    );
    assert!(
        !session.status().parts_negotiated,
        "a flag never declared can never be negotiated"
    );
    session.shutdown();
}

// --- The intake -----------------------------------------------------------

/// The headline: an intervention chunk addressing ONE declared part
/// substitutes that part's rows alone, tagged with the part it commands —
/// "move this part, hold the rest" (FSM.md §4). The recording says the same,
/// which is what makes the row honest: an untagged 7-wide row on a 14-wide
/// robot would claim the whole robot moved (and, before `unflatten_action`
/// learned the tag, would not have decoded at all).
#[test]
fn part_scoped_chunk_substitutes_the_addressed_part_alone() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let rig = rig(true);
    let session = Session::builder("parts-substitute")
        .robot(bimanual_robot())
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .transport(rig.transport.clone())
        .build()
        .unwrap();

    let mut ep = session.start_episode("part-scoped").unwrap();
    let id = ep.id().clone();
    let _ = ep.gate(&[0.0; 2 * ARM_DIMS], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    wait_for(&session, |s| s.parts_negotiated);

    grant_and_engage(&session, "claim-parts", "agent-plane", ActorKind::Agent);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));
    let tx = wait_for_tx(&rig.tx);

    send_chunk(&tx, vec![part_action("left", vec![0.5; ARM_DIMS], 0)], 1);

    let deadline = Instant::now() + Duration::from_secs(5);
    let mut substituted = None;
    while Instant::now() < deadline && substituted.is_none() {
        if let GateOutput::Substitute { action, .. } = ep.gate(&[0.0; 2 * ARM_DIMS], None, None) {
            substituted = Some(action);
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    let action = substituted.expect("a part-scoped chunk must substitute under a negotiated flag");
    assert_eq!(
        action.part.as_deref(),
        Some("left"),
        "the substituted action must carry the part it commands"
    );
    assert_eq!(
        action.values.as_slice(),
        &[0.5; ARM_DIMS],
        "a part-scoped action carries THAT part's width, not the whole robot's"
    );

    release_claim(&session, "claim-parts");
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Passthrough));
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    assert!(
        validation_faults(dir.path(), id.as_str()).is_empty(),
        "a part-scoped action on a negotiated connection is legal, not a validation failure"
    );
    let rows: Vec<pb::Action> = actions_topic(dir.path(), id.as_str())
        .into_iter()
        .flat_map(|c| c.actions)
        .filter(|a| a.part == "left")
        .collect();
    assert!(
        !rows.is_empty(),
        "the recording must name the part that moved"
    );
    for row in &rows {
        let Some(pb::action::Target::JointPosition(v)) = &row.target else {
            panic!("expected the part's own JointPosition space, got {row:?}");
        };
        assert_eq!(v.values, vec![0.5; ARM_DIMS]);
    }
}

/// The pre-flag reading, pinned as a CHARACTERIZATION test: it passes both
/// before and after this stage. A connection that did not negotiate
/// `waddle.v0.parts` reads every action against the WHOLE declared space, so
/// a 7-value part-scoped action on a 14-row robot is refused — deterministic,
/// once per claim window, nothing dispatched. VERSIONING §3 is what makes
/// this a contract rather than a bug: a plane must be able to tell "will
/// execute" from "will fault" before it sends one.
#[test]
fn part_scoped_chunk_is_refused_when_the_flag_was_not_negotiated() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let rig = rig(false);
    let session = Session::builder("parts-unnegotiated")
        .robot(bimanual_robot())
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .transport(rig.transport.clone())
        .build()
        .unwrap();

    let mut ep = session.start_episode("part-scoped-unnegotiated").unwrap();
    let id = ep.id().clone();
    let _ = ep.gate(&[0.0; 2 * ARM_DIMS], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(&session, "claim-noflag", "agent-plane", ActorKind::Agent);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));
    let tx = wait_for_tx(&rig.tx);
    assert!(
        !session.status().parts_negotiated,
        "the plane refused the flag; the session must not act as if it accepted"
    );

    // Sent repeatedly to prove the refusal dedupes to once per claim window.
    for seq in 1..=5u64 {
        send_chunk(&tx, vec![part_action("left", vec![0.5; ARM_DIMS], 0)], seq);
        std::thread::sleep(Duration::from_millis(5));
    }
    for _ in 0..20 {
        assert!(
            matches!(ep.gate(&[0.0; 2 * ARM_DIMS], None, None), GateOutput::Hold),
            "an unnegotiated part-scoped chunk must never substitute"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        send_log.lock().is_empty(),
        "nothing may reach the robot from a refused chunk"
    );

    release_claim(&session, "claim-noflag");
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Passthrough));
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    let faults = validation_faults(dir.path(), id.as_str());
    assert_eq!(
        faults.len(),
        1,
        "the refusal is reported once per claim window, got {faults:?}"
    );
    assert!(
        !faults[0].source.is_empty(),
        "a fault must name the intake that raised it"
    );
}

/// The BYPASS path — a claimed session whose caller loop has stalled, and the
/// only path an agent-invited episode ever takes (its caller never ticks at
/// all). The pump dispatches straight to `send`, so the part tag has to
/// survive the ring → pump → verb hop, and the row it records has to name the
/// part: before this, `unflatten_action` could not decode a part-width row and
/// the recording said the tick commanded NOTHING.
#[test]
fn bypass_dispatch_sends_and_records_a_part_scoped_row() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let rig = rig(true);
    let session = Session::builder("parts-bypass")
        .robot(bimanual_robot())
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .transport(rig.transport.clone())
        .build()
        .unwrap();

    let mut ep = session.start_episode("part-scoped-bypass").unwrap();
    let id = ep.id().clone();
    let _ = ep.gate(&[0.0; 2 * ARM_DIMS], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    wait_for(&session, |s| s.parts_negotiated);

    grant_and_engage(&session, "claim-bypass", "agent-plane", ActorKind::Agent);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));
    // The caller stops ticking: the stall detector flips the session to
    // BYPASS, where the pump — not the caller's gate — drives `send`.
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Bypass));

    let tx = wait_for_tx(&rig.tx);
    send_chunk(&tx, vec![part_action("right", vec![1.25; ARM_DIMS], 0)], 1);

    let deadline = Instant::now() + Duration::from_secs(5);
    while send_log.lock().is_empty() {
        assert!(Instant::now() < deadline, "the bypass pump never sent");
        std::thread::sleep(Duration::from_millis(5));
    }
    let sent = send_log.lock().clone();
    assert_eq!(
        sent[0],
        (Some("right".to_owned()), vec![1.25; ARM_DIMS]),
        "the dispatched step must name the part it commands, at that part's width"
    );

    release_claim(&session, "claim-bypass");
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    let pump_rows: Vec<pb::ActionChunk> = actions_topic(dir.path(), id.as_str())
        .into_iter()
        .filter(|c| c.source_id == "waddle.bypass-pump")
        .collect();
    assert!(
        !pump_rows.is_empty(),
        "every bypass dispatch must produce a recorded action"
    );
    let decoded: Vec<&pb::Action> = pump_rows.iter().flat_map(|c| c.actions.iter()).collect();
    assert!(
        !decoded.is_empty(),
        "the recorded row must carry the action that was dispatched, not an empty list"
    );
    for action in decoded {
        assert_eq!(action.part, "right");
        let Some(pb::action::Target::JointPosition(v)) = &action.target else {
            panic!("expected the part's own JointPosition space, got {action:?}");
        };
        assert_eq!(v.values, vec![1.25; ARM_DIMS]);
    }
}
