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

use std::collections::BTreeMap;
use std::sync::Arc;
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use waddle_controlplane::{ClientMsg, InMemoryTransport, ServerMsg};
use waddle_fsm::Phase;
use waddle_gate::gate::GateOutput;
use waddle_runtime::{
    ControlRegistry, ProprioReport, Session, VerbError, grant_and_engage, release_claim,
};
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
type Uplinked = Arc<Mutex<Vec<pb::ObservationUpdate>>>;

struct Rig {
    transport: Arc<InMemoryTransport>,
    tx: TxCell,
    /// The `feature_flags` the SDK declared on its `RegisterRequest`.
    declared: Declared,
    /// Every `StreamObservations` uplink the plane received.
    uplinked: Uplinked,
}

/// A transport that answers Register by ACCEPTING exactly the flags the SDK
/// declared (or, with `accept_parts` false, everything except
/// `waddle.v0.parts` — the pre-flag connection), stashes the plane's
/// `Sender<ServerMsg>` so a test can push wire messages whenever it wants,
/// and records the observation uplink.
fn rig(accept_parts: bool) -> Rig {
    let tx: TxCell = Arc::new(Mutex::new(None));
    let declared: Declared = Arc::new(Mutex::new(Vec::new()));
    let uplinked: Uplinked = Arc::new(Mutex::new(Vec::new()));
    let (tx_in, declared_in, uplinked_in) = (tx.clone(), declared.clone(), uplinked.clone());
    let transport = InMemoryTransport::new(move |msg, plane_tx: &Sender<ServerMsg>| match msg {
        ClientMsg::Register(req) => {
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
        ClientMsg::Observation(update) => uplinked_in.lock().push(update),
        _ => {}
    });
    Rig {
        transport,
        tx,
        declared,
        uplinked,
    }
}

/// Every proprio sample the plane received on the observation uplink, with
/// the session-timeline stamp it was sent under.
fn uplinked_samples(uplinked: &Uplinked) -> Vec<(i64, pb::ProprioSample)> {
    uplinked
        .lock()
        .iter()
        .filter_map(|u| match &u.payload {
            Some(pb::observation_update::Payload::Proprio(p)) => Some((u.t_ns, p.clone())),
            _ => None,
        })
        .collect()
}

fn proprio_rows(dir: &std::path::Path, episode_id: &str) -> Vec<pb::ProprioSample> {
    let buf = std::fs::read(dir.join(format!("{episode_id}.mcap"))).unwrap();
    let mut samples = Vec::new();
    for message in mcap::MessageStream::new(&buf).unwrap() {
        let message = message.unwrap();
        if message.channel.topic != waddle_sidecar::mcaprec::OBSERVATIONS_TOPIC {
            continue;
        }
        let update =
            <pb::ObservationUpdate as prost::Message>::decode(message.data.as_ref()).unwrap();
        if let Some(pb::observation_update::Payload::Proprio(sample)) = update.payload {
            samples.push(sample);
        }
    }
    samples
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

/// The local injection seam (`push_intervention_chunk`, the
/// `grant_and_engage` precedent): a session with NO control-plane transport
/// still takes an intervention chunk, through the same intake the plane pump
/// runs. It is what lets a test — and the shim's testing hooks — drive a
/// part-scoped intervention without standing up a plane. With no connection
/// to have negotiated the flag with, the declaration decides: this robot
/// declares parts, so `Action.part` is honored; the refusal for a part the
/// declaration does NOT have is the same one either intake gives.
#[test]
fn a_locally_pushed_chunk_addresses_a_part_without_any_plane() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session = Session::builder("parts-local-push")
        .robot(bimanual_robot())
        .control(registry(&send_log))
        .build()
        .unwrap();

    let mut ep = session.start_episode("local-push").unwrap();
    let _ = ep.gate(&[0.0; 2 * ARM_DIMS], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    assert!(
        !session.status().parts_negotiated,
        "there is no connection here to negotiate anything with"
    );

    grant_and_engage(&session, "claim-local", "agent-plane", ActorKind::Agent);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));

    waddle_runtime::push_intervention_chunk(
        &session,
        pb::ActionChunk {
            actions: vec![part_action("right", vec![0.75; ARM_DIMS], 0)],
            seq: 1,
            source_id: "local".into(),
            ..Default::default()
        },
    );

    let deadline = Instant::now() + Duration::from_secs(5);
    let mut substituted = None;
    while Instant::now() < deadline && substituted.is_none() {
        if let GateOutput::Substitute { action, .. } = ep.gate(&[0.0; 2 * ARM_DIMS], None, None) {
            substituted = Some(action);
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    let action = substituted.expect("a locally pushed chunk must substitute");
    assert_eq!(action.part.as_deref(), Some("right"));
    assert_eq!(action.values.as_slice(), &[0.75; ARM_DIMS]);

    release_claim(&session, "claim-local");
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
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

// --- Per-part proprioception ---------------------------------------------

/// `report_proprio(part=, joint_pos=)`: a per-part sample keys its own
/// recording row and its own uplink sample. The part-keyed `joint_pos` is
/// load-bearing — a per-part sample cannot ride the gate's flat `obs` vector,
/// since the observation layout is not the action layout and slicing one by
/// the other would invent a mapping the customer never declared.
#[test]
fn report_proprio_part_keys_recording_rows_and_uplink() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let rig = rig(true);
    let session = Session::builder("parts-proprio")
        .robot(bimanual_robot())
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .transport(rig.transport.clone())
        .build()
        .unwrap();

    let ep = session.start_episode("part-scoped-proprio").unwrap();
    let id = ep.id().clone();
    wait_for(&session, |s| s.parts_negotiated);

    // Never ticks the gate: every observation in the file came from a
    // report, and each names its own part.
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        session
            .report_proprio(ProprioReport {
                part: "left".into(),
                joint_pos: Some(vec![0.1; ARM_DIMS]),
                gripper: Some(0.25),
                ..Default::default()
            })
            .unwrap();
        session
            .report_proprio(ProprioReport {
                part: "right".into(),
                joint_pos: Some(vec![0.9; ARM_DIMS]),
                gripper: Some(0.75),
                ..Default::default()
            })
            .unwrap();
        let parts: Vec<String> = rig
            .uplinked
            .lock()
            .iter()
            .filter_map(|u| match &u.payload {
                Some(pb::observation_update::Payload::Proprio(p)) => Some(p.part.clone()),
                _ => None,
            })
            .collect();
        if parts.iter().any(|p| p == "left") && parts.iter().any(|p| p == "right") {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "both reported parts must reach the uplink, got {parts:?}"
        );
        std::thread::sleep(Duration::from_millis(10));
    }

    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    let rows = proprio_rows(dir.path(), id.as_str());
    let left: Vec<&pb::ProprioSample> = rows.iter().filter(|s| s.part == "left").collect();
    let right: Vec<&pb::ProprioSample> = rows.iter().filter(|s| s.part == "right").collect();
    assert!(
        !left.is_empty() && !right.is_empty(),
        "each reported part gets its own recorded row: {rows:?}"
    );
    assert_eq!(left[0].joint_pos, vec![0.1; ARM_DIMS]);
    assert_eq!(left[0].gripper, Some(0.25));
    assert_eq!(
        right[0].joint_pos,
        vec![0.9; ARM_DIMS],
        "one part's report must never overwrite another's"
    );
    assert_eq!(right[0].gripper, Some(0.75));
}

/// The uplink cadence is keyed PER PART, not spent from one shared budget:
/// the 10 Hz cap bounds each part's own stream (bandwidth stays bounded —
/// parts+1 tiny samples), and no part's staleness can be masked, or its
/// samples starved, by another part's chatter. The plane's freshness checks
/// key on exactly the part they ask about.
#[test]
fn uplink_cadence_is_per_part() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let rig = rig(true);
    let session = Session::builder("parts-cadence")
        .robot(bimanual_robot())
        .control(registry(&send_log))
        .transport(rig.transport.clone())
        .build()
        .unwrap();

    let ep = session.start_episode("part-cadence").unwrap();
    wait_for(&session, |s| s.parts_negotiated);

    // Report both parts continuously. Once both are past their own period,
    // one reducer wake sends BOTH — the stamp they share is what proves the
    // cap is per part and not a single slot the parts take turns in.
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        for (part, v) in [("left", 0.1), ("right", 0.2)] {
            session
                .report_proprio(ProprioReport {
                    part: part.into(),
                    joint_pos: Some(vec![v; ARM_DIMS]),
                    ..Default::default()
                })
                .unwrap();
        }
        let samples = uplinked_samples(&rig.uplinked);
        let shared_wake = samples
            .iter()
            .filter(|(_, s)| s.part == "left")
            .any(|(t, _)| samples.iter().any(|(t2, s2)| s2.part == "right" && t2 == t));
        if shared_wake {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "two parts must be able to uplink on the same wake: {samples:?}"
        );
        std::thread::sleep(Duration::from_millis(10));
    }

    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    // And the cap itself still holds, per part: 10 Hz each.
    let mut by_part: BTreeMap<String, Vec<i64>> = BTreeMap::new();
    for (t, sample) in uplinked_samples(&rig.uplinked) {
        by_part.entry(sample.part.clone()).or_default().push(t);
    }
    assert!(
        by_part.contains_key("left") && by_part.contains_key("right"),
        "every reported part must reach the uplink: {:?}",
        by_part.keys().collect::<Vec<_>>()
    );
    for (part, stamps) in &by_part {
        for pair in stamps.windows(2) {
            assert!(
                pair[1] - pair[0] >= 80_000_000,
                "part {part:?} uplinked faster than the declared 10 Hz: {}ns apart",
                pair[1] - pair[0]
            );
        }
    }
}

/// VERSIONING.md's pre-flag rule for the uplink, in the direction that is
/// easy to get wrong: a connection that did not negotiate the flag gets the
/// `""` sample and NOTHING else. Named-part samples are WITHHELD, never
/// relabeled `""` — relabeling would put one arm's joint vector on the wire
/// as the whole robot's, and let the parts overwrite each other. Local
/// recording is not connection-scoped and still records every part.
#[test]
fn named_part_samples_are_withheld_from_an_unnegotiated_uplink() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let rig = rig(false);
    let session = Session::builder("parts-withheld")
        .robot(bimanual_robot())
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .transport(rig.transport.clone())
        .build()
        .unwrap();

    let ep = session.start_episode("part-withheld").unwrap();
    let id = ep.id().clone();
    let _ = wait_for_tx(&rig.tx);
    assert!(!session.status().parts_negotiated);

    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        session
            .report_proprio(ProprioReport {
                part: "left".into(),
                joint_pos: Some(vec![0.1; ARM_DIMS]),
                ..Default::default()
            })
            .unwrap();
        session
            .report_proprio(ProprioReport {
                gripper: Some(0.5),
                ..Default::default()
            })
            .unwrap();
        let samples = uplinked_samples(&rig.uplinked);
        assert!(
            samples.iter().all(|(_, s)| s.part.is_empty()),
            "a connection without the flag must never receive a named part: {samples:?}"
        );
        if samples.len() >= 2 {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "the sole-part sample must still uplink"
        );
        std::thread::sleep(Duration::from_millis(10));
    }

    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    let recorded = proprio_rows(dir.path(), id.as_str());
    assert!(
        recorded.iter().any(|s| s.part == "left"),
        "withholding is an UPLINK rule: the local recording still names every part"
    );
}

/// A report naming a part the robot never declared is refused BY NAME, at
/// the call, rather than silently landing under a key nothing will ever read.
/// This is declaration validation — not claim, lease, or timeline logic — so
/// it is the session's to answer.
#[test]
fn unknown_part_report_is_refused_by_name() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session = Session::builder("parts-unknown")
        .robot(bimanual_robot())
        .control(registry(&send_log))
        .build()
        .unwrap();

    let err = session
        .report_proprio(ProprioReport {
            part: "waist".into(),
            joint_pos: Some(vec![0.0; ARM_DIMS]),
            ..Default::default()
        })
        .expect_err("an undeclared part must be refused");
    assert!(
        err.to_string().contains("waist"),
        "the refusal must name the part the caller asked for: {err}"
    );

    for part in ["", "left", "right"] {
        session
            .report_proprio(ProprioReport {
                part: part.into(),
                ..Default::default()
            })
            .unwrap_or_else(|e| panic!("part {part:?} is declared, but was refused: {e}"));
    }
    session.shutdown();

    // A single-part robot declares no part by any name; `""` — the sole
    // part, and already core — stays legal.
    let session = Session::builder("parts-unknown-single")
        .robot(single_part_robot())
        .control(registry(&send_log))
        .build()
        .unwrap();
    assert!(
        session
            .report_proprio(ProprioReport {
                part: "left".into(),
                ..Default::default()
            })
            .is_err()
    );
    session
        .report_proprio(ProprioReport::default())
        .expect("the sole part is always legal");
    session.shutdown();
}
