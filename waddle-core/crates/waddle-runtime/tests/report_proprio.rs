//! Task 19 — `Session::report_proprio` merges a richer proprioceptive
//! sample (`joint_vel`, `ee_pose`, `gripper`) into the reducer's own
//! `joint_pos` (from the caller's `gate(obs=...)` stream): the merged
//! `ProprioSample` lands in Local-mode MCAP `/waddle/observations` exactly
//! as `joint_pos` alone does today, and is also what the periodic
//! `StreamObservations` uplink sends whenever a transport is configured.

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use prost::Message as _;
use waddle_controlplane::{ClientMsg, InMemoryTransport, ServerMsg};
use waddle_runtime::{ControlRegistry, EePose, ProprioReport, Session, VerbError};
use waddle_types::TerminalOutcome;
use waddle_types::pb::v0 as pb;

fn robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "proprio-bot".into(),
        robot_id: "proprio-01".into(),
        cell_id: "cell-proprio".into(),
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

fn registry() -> ControlRegistry {
    ControlRegistry {
        send: Some(Arc::new(
            |_chunk: &waddle_types::ActionChunk| -> Result<(), VerbError> { Ok(()) },
        )),
        hold: Some(Arc::new(|| Ok(()))),
        resume: Some(Arc::new(|| Ok(()))),
        ..Default::default()
    }
}

/// The headline regression: a reported sample's `joint_vel`/`ee_pose`/
/// `gripper` ride every subsequent gate-tick's recorded `ProprioSample`
/// alongside the tick's own `joint_pos`.
#[test]
fn report_proprio_merges_into_recorded_observations() {
    let dir = tempfile::tempdir().unwrap();
    let session = Session::builder("proprio-mcap")
        .robot(robot())
        .control(registry())
        .recording_dir(dir.path())
        .build()
        .unwrap();

    let mut ep = session.start_episode("stack the blocks").unwrap();
    let id = ep.id().clone();

    session.report_proprio(ProprioReport {
        joint_vel: Some(vec![0.01, 0.02, 0.03]),
        ee_pose: Some(EePose::new([1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0], "ee").unwrap()),
        gripper: Some(0.5),
    });
    // The report crosses a side channel the reducer drains once per wake
    // (<=20ms cadence); settle well past that before the first gate tick so
    // every recorded tick below sees the merge, not just a later one.
    std::thread::sleep(Duration::from_millis(60));

    let obs = [0.9f64, 0.8, 0.7];
    for _ in 0..10 {
        let _ = ep.gate(&[0.1, 0.2, 0.3], None, Some(&obs));
    }
    ep.terminate(TerminalOutcome::Success, "test done");
    session.shutdown();

    let buf = std::fs::read(dir.path().join(format!("{id}.mcap"))).unwrap();
    let mut observations = Vec::new();
    for message in mcap::MessageStream::new(&buf).unwrap() {
        let message = message.unwrap();
        if message.channel.topic == waddle_sidecar::mcaprec::OBSERVATIONS_TOPIC {
            observations.push(pb::ObservationUpdate::decode(message.data.as_ref()).unwrap());
        }
    }
    assert!(!observations.is_empty());
    let Some(pb::observation_update::Payload::Proprio(proprio)) =
        &observations.last().unwrap().payload
    else {
        panic!("expected a proprio payload");
    };
    assert_eq!(proprio.joint_pos, obs);
    assert_eq!(proprio.joint_vel, vec![0.01, 0.02, 0.03]);
    assert_eq!(proprio.gripper, Some(0.5));
    let pose = proprio.ee_pose.as_ref().expect("ee_pose merged in");
    assert_eq!(pose.frame_id, "ee");
    assert_eq!(
        pose.position,
        Some(pb::Vec3 {
            x: 1.0,
            y: 2.0,
            z: 3.0
        })
    );
    assert_eq!(
        pose.rotation,
        Some(pb::Quat {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0
        })
    );
}

/// A `report_proprio` call with no further field patches a later report's
/// fields, never clears them (there is no "unset" in v0) — reported here
/// against the SAME merged sample the recording test above pins, via a
/// second, partial report.
#[test]
fn report_proprio_patches_only_the_fields_it_carries() {
    let dir = tempfile::tempdir().unwrap();
    let session = Session::builder("proprio-patch")
        .robot(robot())
        .control(registry())
        .recording_dir(dir.path())
        .build()
        .unwrap();

    let mut ep = session.start_episode("task").unwrap();
    let id = ep.id().clone();

    session.report_proprio(ProprioReport {
        joint_vel: Some(vec![1.0, 1.0, 1.0]),
        ee_pose: None,
        gripper: Some(0.1),
    });
    std::thread::sleep(Duration::from_millis(60));
    // A second report patches only `gripper`; `joint_vel` must survive.
    session.report_proprio(ProprioReport {
        joint_vel: None,
        ee_pose: None,
        gripper: Some(0.9),
    });
    std::thread::sleep(Duration::from_millis(60));

    let _ = ep.gate(&[0.0; 3], None, Some(&[0.0; 3]));
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    let buf = std::fs::read(dir.path().join(format!("{id}.mcap"))).unwrap();
    let mut last: Option<pb::ProprioSample> = None;
    for message in mcap::MessageStream::new(&buf).unwrap() {
        let message = message.unwrap();
        if message.channel.topic == waddle_sidecar::mcaprec::OBSERVATIONS_TOPIC {
            let update = pb::ObservationUpdate::decode(message.data.as_ref()).unwrap();
            if let Some(pb::observation_update::Payload::Proprio(p)) = update.payload {
                last = Some(p);
            }
        }
    }
    let last = last.expect("at least one observation recorded");
    assert_eq!(
        last.joint_vel,
        vec![1.0, 1.0, 1.0],
        "patch must not clear it"
    );
    assert_eq!(last.gripper, Some(0.9));
}

/// `StreamObservations`: whenever a transport is configured, the reducer
/// periodically sends the same merged sample as a `ClientMsg::Observation`
/// (conservative default cadence — no per-robot rate to key off; see the
/// reducer's `DEFAULT_OBSERVATION_UPLINK_HZ` doc).
#[test]
fn stream_observations_uplinks_periodically_with_merged_fields() {
    let received: Arc<Mutex<Vec<pb::ObservationUpdate>>> = Arc::new(Mutex::new(Vec::new()));
    let received_in = received.clone();
    let transport = InMemoryTransport::new(move |msg, tx: &Sender<ServerMsg>| match msg {
        ClientMsg::Register(_) => {
            let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse::default()));
        }
        ClientMsg::Observation(update) => received_in.lock().push(update),
        _ => {}
    });

    let session = Session::builder("proprio-uplink")
        .robot(robot())
        .control(registry())
        .transport(transport)
        .build()
        .unwrap();

    let mut ep = session.start_episode("task").unwrap();
    session.report_proprio(ProprioReport {
        joint_vel: Some(vec![1.0, 2.0, 3.0]),
        ee_pose: None,
        gripper: Some(0.75),
    });
    let _ = ep.gate(&[0.0; 3], None, Some(&[0.1, 0.2, 0.3]));

    // 10 Hz default cadence (100ms period): wait comfortably past two.
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        if received.lock().len() >= 2 {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "timed out waiting for uplinked observations"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    session.shutdown();

    let snapshot = received.lock().clone();
    for pair in snapshot.windows(2) {
        let gap = pair[1].t_ns - pair[0].t_ns;
        assert!(
            gap >= 80_000_000,
            "uplink cadence faster than the declared 10 Hz: {gap}ns apart"
        );
    }
    let Some(pb::observation_update::Payload::Proprio(sample)) = &snapshot[0].payload else {
        panic!("expected a proprio payload");
    };
    assert_eq!(sample.joint_vel, vec![1.0, 2.0, 3.0]);
    assert_eq!(sample.gripper, Some(0.75));
}
