//! End-to-end runtime tests over the loopback media plane: nominal episode
//! with Local recording, claim/engage/release with teleop substitution, and
//! the claimed-while-stalled bypass contract.

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use waddle_fsm::Phase;
use waddle_gate::gate::GateOutput;
use waddle_media::{DataTopic, LoopbackMedia};
use waddle_runtime::{ControlRegistry, Session, VerbError, grant_and_engage};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActorKind, GateMode, Provenance, TerminalOutcome};

fn robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "e2e-bot".into(),
        robot_id: "e2e-01".into(),
        cell_id: "cell-e2e".into(),
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

#[test]
fn nominal_episode_records_sidecar_and_mcap() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session = Session::builder("e2e-project")
        .robot(robot())
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .build()
        .unwrap();

    let mut ep = session.start_episode("stack the blocks").unwrap();
    let id = ep.id().clone();
    for _ in 0..50 {
        let out = ep.gate(&[0.1, 0.2, 0.3], None);
        assert!(matches!(out, GateOutput::Pass { .. }));
    }
    let records = ep.drain_records();
    assert!(records.len() >= 50);

    ep.terminate(TerminalOutcome::Success, "test done");
    assert!(ep.done());
    session.shutdown();

    let sidecar_path = dir.path().join(format!("{id}.sidecar.json"));
    assert!(sidecar_path.exists(), "sidecar file must be written");
    let sidecar =
        waddle_sidecar::sidecar_from_json(&std::fs::read_to_string(&sidecar_path).unwrap())
            .unwrap();
    assert_eq!(sidecar.outcome, pb::TerminalOutcome::Success as i32);
    assert_eq!(sidecar.robot_id, "e2e-01");
    assert!(sidecar.bounds.is_some());
    assert!(
        sidecar.t_start_unix_ns > 1_500_000_000_000_000_000,
        "epoch twin captured at stamp time"
    );
    assert!(dir.path().join(format!("{id}.mcap")).exists());
    assert!(dir.path().join("manifest.jsonl").exists());
}

#[test]
fn engage_substitutes_teleop_actions_then_release_restores_passthrough() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (media, far) = LoopbackMedia::new();
    let session = Session::builder("e2e-intervention")
        .robot(robot())
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .media(media)
        .build()
        .unwrap();

    let mut ep = session.start_episode("towel").unwrap();
    for _ in 0..5 {
        assert!(matches!(ep.gate(&[0.0; 3], None), GateOutput::Pass { .. }));
    }
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(&session, "claim-1", "teleop", ActorKind::Teleoperator);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));

    // Teleop stream flows through the media plane into the gate.
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

    // Keep ticking (the loop is healthy); expect a substitution once the
    // playout delay elapses.
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut substituted = false;
    while Instant::now() < deadline {
        push_pose(0.7);
        match ep.gate(&[0.0; 3], None) {
            GateOutput::Substitute { provenance, .. } | GateOutput::Blend { provenance, .. } => {
                assert_eq!(provenance.provenance, Provenance::Teleop);
                substituted = true;
                break;
            }
            GateOutput::Pass { .. } | GateOutput::Hold | GateOutput::Noop { .. } => {
                std::thread::sleep(Duration::from_millis(5));
            }
        }
    }
    assert!(substituted, "teleop stream never substituted");

    session.inject(waddle_fsm::SessionEvent::Release {
        claim: waddle_types::ClaimId::new("claim-1"),
        at: waddle_types::MonoNs(0),
    });
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Passthrough));
    assert!(matches!(ep.gate(&[0.0; 3], None), GateOutput::Pass { .. }));

    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    // The sidecar carries the intervention record.
    let files: Vec<_> = std::fs::read_dir(dir.path())
        .unwrap()
        .filter_map(Result::ok)
        .filter(|e| e.file_name().to_string_lossy().ends_with(".sidecar.json"))
        .collect();
    assert_eq!(files.len(), 1);
    let sidecar =
        waddle_sidecar::sidecar_from_json(&std::fs::read_to_string(files[0].path()).unwrap())
            .unwrap();
    assert!(!sidecar.claims.is_empty(), "claim span recorded");
    assert!(
        sidecar.leases.len() >= 3,
        "loop → teleop → loop lease spans, got {}",
        sidecar.leases.len()
    );
    assert!(!sidecar.interventions.is_empty());
}

#[test]
fn claimed_while_stalled_bypass_drives_send_directly() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (media, far) = LoopbackMedia::new();
    let session = Session::builder("e2e-bypass")
        .robot(robot())
        .control(registry(&send_log))
        .media(media)
        .build()
        .unwrap();

    let mut ep = session.start_episode("bypass").unwrap();
    for _ in 0..5 {
        let _ = ep.gate(&[0.0; 3], None);
    }
    grant_and_engage(&session, "claim-b", "teleop", ActorKind::Teleoperator);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));

    // The caller's loop stalls (no more ticks). Teleop keeps streaming.
    let mut seq = 0u64;
    let deadline = Instant::now() + Duration::from_secs(5);
    while session.status().gate_mode != Some(GateMode::Bypass) {
        seq += 1;
        far.push(
            DataTopic::TeleopPose,
            &pb::TeleopStreamPacket {
                t_client_ns: 1,
                seq,
                targets: vec![pb::PartTarget {
                    part: String::new(),
                    target: Some(pb::part_target::Target::Twist(pb::Twist {
                        linear: Some(pb::Vec3 {
                            x: 1.0,
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
        assert!(Instant::now() < deadline, "bypass never engaged");
        std::thread::sleep(Duration::from_millis(20));
    }

    // The pump drives send directly with teleop provenance.
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        {
            let log = send_log.lock();
            if log.iter().any(|(p, _)| *p == Provenance::Teleop) {
                break;
            }
        }
        seq += 1;
        far.push(
            DataTopic::TeleopPose,
            &pb::TeleopStreamPacket {
                t_client_ns: 1,
                seq,
                targets: vec![pb::PartTarget {
                    part: String::new(),
                    target: Some(pb::part_target::Target::Twist(pb::Twist {
                        linear: Some(pb::Vec3 {
                            x: 2.0,
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
        assert!(Instant::now() < deadline, "bypass pump never sent");
        std::thread::sleep(Duration::from_millis(10));
    }

    // A late caller tick observes a NOOP marker (spectator contract).
    match ep.gate(&[0.0; 3], None) {
        GateOutput::Noop { provenance } => {
            assert_eq!(provenance.provenance, Provenance::Teleop);
        }
        other => panic!("expected NOOP for the stalled loop's tick, got {other:?}"),
    }

    session.shutdown();
}
