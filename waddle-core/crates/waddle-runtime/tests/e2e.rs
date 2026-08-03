//! End-to-end runtime tests over the loopback media plane: nominal episode
//! with Local recording, claim/engage/release with teleop substitution, and
//! the claimed-while-stalled bypass contract.

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use prost::Message as _;
use waddle_fsm::Phase;
use waddle_gate::gate::GateOutput;
use waddle_media::{DataTopic, LoopbackMedia};
use waddle_runtime::{
    ControlRegistry, RuntimeError, Session, VerbError, grant_and_engage, release_claim,
};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActorKind, GateMode, HandoffPolicy, Provenance, TerminalOutcome};

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

/// A 6-dim `BaseTwist` robot: matches the `Twist` teleop packets the
/// intervention tests push (see `flatten_packet`), so the intake dims
/// validation never rejects them. `gripper` plumbs a declared
/// `GripperSpec` through for the gripper-mapping tests; `None` reproduces the
/// other intervention tests' ungripped fixture.
fn twist_robot(gripper: Option<pb::GripperSpec>) -> pb::RobotDescription {
    pb::RobotDescription {
        name: "e2e-twist-bot".into(),
        robot_id: "e2e-02".into(),
        cell_id: "cell-e2e".into(),
        action_space: Some(pb::ActionSpace {
            space: Some(pb::action_space::Space::BaseTwist(pb::BaseTwist {
                frame_id: "base".into(),
                max_linear_mps: None,
                max_angular_radps: None,
            })),
            rate_hz: 50.0,
            chunking: None,
            gripper,
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

/// A delta action space (`EePoseDelta`): FSM.md §5 refuses mid-chunk splice
/// entry for delta spaces, so `begin_engage` silently degrades a declared
/// `HandoffPolicy::Immediate` to `HoldFirst` on the very first engage
/// (`waddle-fsm/src/session.rs`). Used to pin that the build-time `hold`
/// check reasons about this *effective* policy, not the raw declared enum
/// variant.
fn ee_delta_robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "e2e-delta-bot".into(),
        robot_id: "e2e-03".into(),
        cell_id: "cell-e2e".into(),
        action_space: Some(pb::ActionSpace {
            space: Some(pb::action_space::Space::EeDelta(pb::EePoseDelta {
                frame_id: "base".into(),
                rotation_encoding: pb::RotationEncoding::QuatWxyz as i32,
                delta_frame: pb::DeltaFrame::Base as i32,
                max_linear_step_m: None,
                max_angular_step_rad: None,
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
                send_interfaces: vec![pb::SpaceKind::EePoseDelta as i32],
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
    let obs = [0.9f64, 0.8, 0.7];
    for _ in 0..50 {
        let out = ep.gate(&[0.1, 0.2, 0.3], None, Some(&obs));
        assert!(matches!(out, GateOutput::Pass { .. }));
    }
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
    assert_eq!(sidecar.task, "stack the blocks");
    assert!(sidecar.bounds.is_some());
    assert!(
        sidecar.t_start_unix_ns > 1_500_000_000_000_000_000,
        "epoch twin captured at stamp time"
    );
    assert!(dir.path().join("manifest.jsonl").exists());

    // The reducer persisted every gated tick to the episode MCAP: the
    // (obs, action) pairs land on /waddle/observations + /waddle/actions.
    let buf = std::fs::read(dir.path().join(format!("{id}.mcap"))).unwrap();
    let mut actions = Vec::new();
    let mut observations = Vec::new();
    for message in mcap::MessageStream::new(&buf).unwrap() {
        let message = message.unwrap();
        match message.channel.topic.as_str() {
            waddle_sidecar::mcaprec::ACTIONS_TOPIC => {
                actions.push(pb::ActionChunk::decode(message.data.as_ref()).unwrap());
            }
            waddle_sidecar::mcaprec::OBSERVATIONS_TOPIC => {
                observations.push(pb::ObservationUpdate::decode(message.data.as_ref()).unwrap());
            }
            _ => {}
        }
    }
    assert!(actions.len() >= 50, "got {} action chunks", actions.len());
    assert!(
        observations.len() >= 50,
        "got {} observations",
        observations.len()
    );
    let chunk = &actions[0];
    assert_eq!(chunk.source_id, "waddle.gate");
    assert_eq!(chunk.provenance.as_ref().unwrap().kind, {
        pb::ProvenanceKind::Policy as i32
    });
    assert_eq!(
        chunk.actions[0].target,
        Some(pb::action::Target::JointPosition(pb::JointVector {
            values: vec![0.1, 0.2, 0.3],
        }))
    );
    let Some(pb::observation_update::Payload::Proprio(proprio)) = &observations[0].payload else {
        panic!("expected a proprio payload");
    };
    assert_eq!(proprio.joint_pos, obs);
}

#[test]
fn second_open_is_rejected_and_stale_terminate_is_a_no_op() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session = Session::builder("e2e-guards")
        .robot(robot())
        .control(registry(&send_log))
        .build()
        .unwrap();

    let mut ep1 = session.start_episode("first").unwrap();
    let _ = ep1.gate(&[0.0; 3], None, None);
    // One active episode per session (N18): the guard errors instead of
    // destroying the live episode's recording and hanging.
    assert!(matches!(
        session.start_episode("second"),
        Err(waddle_runtime::RuntimeError::EpisodeActive)
    ));

    ep1.terminate(TerminalOutcome::Failure, "done");
    assert_eq!(ep1.outcome(), Some(TerminalOutcome::Failure));

    // A stale handle never terminates a later episode.
    let ep2 = session.start_episode("third").unwrap();
    ep1.terminate(TerminalOutcome::Abort, "stale");
    assert!(
        !ep2.done(),
        "stale terminate must not touch the live episode"
    );
    ep2.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
}

#[test]
fn engage_substitutes_teleop_actions_then_release_restores_passthrough() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (media, far) = LoopbackMedia::new();
    let session = Session::builder("e2e-intervention")
        .robot(twist_robot(None))
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .media(media)
        .build()
        .unwrap();

    let mut ep = session.start_episode("towel").unwrap();
    for _ in 0..5 {
        assert!(matches!(
            ep.gate(&[0.0; 3], None, None),
            GateOutput::Pass { .. }
        ));
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
        match ep.gate(&[0.0; 3], None, None) {
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

    release_claim(&session, "claim-1");
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Passthrough));
    assert!(matches!(
        ep.gate(&[0.0; 3], None, None),
        GateOutput::Pass { .. }
    ));

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

    // The claim names its claimant, and the claimed span is TELEOP — the
    // flow that always worked, pinned so deriving provenance from the actor
    // can never regress it. (A local grant has no plane-stamped id, so the
    // kind is all there is to carry.)
    let claim = sidecar.claims[0].claim.as_ref().expect("claim recorded");
    assert_eq!(
        claim.actor.as_ref().map(|a| a.kind),
        Some(pb::ActorKind::Teleoperator as i32)
    );
    let claimed: Vec<&pb::ProvenanceTag> = sidecar
        .provenance
        .iter()
        .filter_map(|p| p.tag.as_ref())
        .filter(|t| t.kind != pb::ProvenanceKind::Policy as i32)
        .collect();
    assert!(!claimed.is_empty(), "an engaged claim opens a claimed span");
    for tag in claimed {
        assert_eq!(tag.kind, pb::ProvenanceKind::Teleop as i32);
    }
}

#[test]
fn clutch_engage_is_recorded_with_teleoperator_provenance_by_default() {
    // The leader-arm/console-clutch takeover path: a self-initiated claim
    // over the reliable clutch topic, never an explicit ClaimGranted/Engage
    // pair. Before this fix, waddle-fsm hardcoded ActorKind::SiteOperator
    // for this path, so the reducer's provenance mapping recorded these
    // interventions as NOT teleop; SessionBuilder now sets the honest
    // runtime default (Teleoperator / "teleop-clutch") that waddle-fsm
    // itself deliberately does not default to (fixture stability).
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (media, far) = LoopbackMedia::new();
    let session = Session::builder("e2e-clutch")
        .robot(twist_robot(None))
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .media(media)
        .build()
        .unwrap();

    let mut ep = session.start_episode("towel").unwrap();
    for _ in 0..5 {
        assert!(matches!(
            ep.gate(&[0.0; 3], None, None),
            GateOutput::Pass { .. }
        ));
    }
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    // Engage the clutch over the reliable topic (production's
    // self-initiated-claim path).
    far.push(
        DataTopic::TeleopClutch,
        &pb::ClutchTransition {
            engaged: true,
            t_client_ns: 1,
            part: String::new(),
        },
    )
    .unwrap();
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

    let deadline = Instant::now() + Duration::from_secs(5);
    let mut substituted = false;
    while Instant::now() < deadline {
        push_pose(0.7);
        match ep.gate(&[0.0; 3], None, None) {
            GateOutput::Substitute { provenance, .. } | GateOutput::Blend { provenance, .. } => {
                assert_eq!(
                    provenance.provenance,
                    Provenance::Teleop,
                    "clutch-initiated intervention must be recorded as teleop, not custom"
                );
                substituted = true;
                break;
            }
            GateOutput::Pass { .. } | GateOutput::Hold | GateOutput::Noop { .. } => {
                std::thread::sleep(Duration::from_millis(5));
            }
        }
    }
    assert!(substituted, "clutch-driven teleop stream never substituted");

    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    // The sidecar's claim span carries the runtime's honest clutch
    // identity (source), not waddle-fsm's fixture-stable "custom" default.
    let files: Vec<_> = std::fs::read_dir(dir.path())
        .unwrap()
        .filter_map(Result::ok)
        .filter(|e| e.file_name().to_string_lossy().ends_with(".sidecar.json"))
        .collect();
    assert_eq!(files.len(), 1);
    let sidecar =
        waddle_sidecar::sidecar_from_json(&std::fs::read_to_string(files[0].path()).unwrap())
            .unwrap();
    let claim = sidecar
        .claims
        .first()
        .and_then(|c| c.claim.as_ref())
        .expect("claim span recorded");
    assert!(claim.self_initiated, "clutch claims are self-initiated");
    assert_eq!(claim.source_name, "teleop-clutch");
    // …and the actor kind the runtime declared for the clutch, carried onto
    // the claim event. A clutch is local: there is no stamped id to carry.
    let actor = claim.actor.as_ref().expect("the claim names its claimant");
    assert_eq!(actor.kind, pb::ActorKind::Teleoperator as i32);
    assert!(actor.id.is_empty());
}

#[test]
fn claimed_while_stalled_bypass_drives_send_directly() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (media, far) = LoopbackMedia::new();
    let session = Session::builder("e2e-bypass")
        .robot(twist_robot(None))
        .control(registry(&send_log))
        .media(media)
        .build()
        .unwrap();

    let mut ep = session.start_episode("bypass").unwrap();
    for _ in 0..5 {
        let _ = ep.gate(&[0.0; 3], None, None);
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
    match ep.gate(&[0.0; 3], None, None) {
        GateOutput::Noop { provenance } => {
            assert_eq!(provenance.provenance, Provenance::Teleop);
        }
        other => panic!("expected NOOP for the stalled loop's tick, got {other:?}"),
    }

    session.shutdown();
}

/// Stale-backlog replay: media intake must gate its ring push on the mirror's claim state.
/// Poses arriving before any claim exists must be dropped at intake, never
/// stockpiled and replayed the instant a claim engages.
#[test]
fn stale_pre_claim_poses_never_replay_after_engage() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (media, far) = LoopbackMedia::new();
    let session = Session::builder("e2e-stale-backlog")
        .robot(twist_robot(None))
        .control(registry(&send_log))
        .media(media)
        .build()
        .unwrap();

    let mut ep = session.start_episode("stale-backlog").unwrap();
    for _ in 0..5 {
        let _ = ep.gate(&[0.0; 6], None, None);
    }
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    let seq = Arc::new(AtomicU32::new(1));
    let push_twist = |value: f64| {
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
                clutch_engaged: false,
                inputs: None,
            },
        )
        .unwrap();
    };

    // A backlog of clearly-stale poses arrives before any claim exists.
    // None of these may ever be substituted once a claim engages.
    for i in 0..200 {
        push_twist(1_000.0 + f64::from(i));
    }
    // Give the intake thread a moment to drain the backlog while unclaimed.
    std::thread::sleep(Duration::from_millis(100));

    grant_and_engage(&session, "claim-stale", "teleop", ActorKind::Teleoperator);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));

    // The one fresh, post-claim pose.
    push_twist(7.0);

    let deadline = Instant::now() + Duration::from_secs(5);
    let mut first_substitute: Option<Vec<f64>> = None;
    while Instant::now() < deadline {
        match ep.gate(&[0.0; 6], None, None) {
            GateOutput::Substitute { action, .. } | GateOutput::Blend { action, .. } => {
                first_substitute = Some(action.values.to_vec());
                break;
            }
            _ => std::thread::sleep(Duration::from_millis(5)),
        }
    }
    let values = first_substitute.expect("teleop stream never substituted");
    assert_eq!(
        values[0], 7.0,
        "first substitution must be the fresh post-claim pose, not a stale pre-claim one"
    );

    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
}

/// Dims-validation contract: a teleop action whose flattened width doesn't match the robot's
/// declared action space must never reach the ring, and the mismatch must
/// surface as exactly one Fault (not one per 60-90 Hz packet) for the whole
/// claim window. A subsequent matching packet must still substitute
/// normally — validation must not wedge the stream.
#[test]
fn mismatched_action_dims_are_dropped_with_one_fault_per_claim() {
    let dir = tempfile::tempdir().unwrap();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (media, far) = LoopbackMedia::new();
    let session = Session::builder("e2e-dims-validation")
        .robot(twist_robot(None))
        .control(registry(&send_log))
        .recording_dir(dir.path())
        .media(media)
        .build()
        .unwrap();

    let mut ep = session.start_episode("dims-validation").unwrap();
    for _ in 0..5 {
        let _ = ep.gate(&[0.0; 6], None, None);
    }
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(&session, "claim-dims", "teleop", ActorKind::Teleoperator);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));

    let seq = Arc::new(AtomicU32::new(1));
    // A Pose target flattens to 7 values; the declared BaseTwist space wants
    // 6. Spam this well past a single 60-90 Hz packet to prove the fault
    // dedupes to once per claim window.
    for _ in 0..20 {
        far.push(
            DataTopic::TeleopPose,
            &pb::TeleopStreamPacket {
                t_client_ns: 1,
                seq: u64::from(seq.fetch_add(1, Ordering::SeqCst)),
                targets: vec![pb::PartTarget {
                    part: String::new(),
                    target: Some(pb::part_target::Target::Pose(pb::Pose {
                        position: Some(pb::Vec3 {
                            x: 9.0,
                            y: 9.0,
                            z: 9.0,
                        }),
                        rotation: Some(pb::Quat {
                            w: 1.0,
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        }),
                        frame_id: String::new(),
                    })),
                    gripper: None,
                }],
                clutch_engaged: true,
                inputs: None,
            },
        )
        .unwrap();
        std::thread::sleep(Duration::from_millis(5));
    }

    // None of the mismatched packets ever substitute; the loop only ever
    // observes Hold.
    for _ in 0..10 {
        assert!(
            matches!(ep.gate(&[0.0; 6], None, None), GateOutput::Hold),
            "a dims-mismatched teleop action must never be substituted"
        );
    }

    // A matching packet still substitutes normally: validation doesn't
    // wedge the stream.
    far.push(
        DataTopic::TeleopPose,
        &pb::TeleopStreamPacket {
            t_client_ns: 1,
            seq: u64::from(seq.fetch_add(1, Ordering::SeqCst)),
            targets: vec![pb::PartTarget {
                part: String::new(),
                target: Some(pb::part_target::Target::Twist(pb::Twist {
                    linear: Some(pb::Vec3 {
                        x: 3.0,
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

    let deadline = Instant::now() + Duration::from_secs(5);
    let mut substituted = false;
    while Instant::now() < deadline {
        match ep.gate(&[0.0; 6], None, None) {
            GateOutput::Substitute { action, .. } | GateOutput::Blend { action, .. } => {
                assert_eq!(action.values[0], 3.0);
                substituted = true;
                break;
            }
            _ => std::thread::sleep(Duration::from_millis(5)),
        }
    }
    assert!(substituted, "a matching packet must still substitute");

    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();

    // Exactly one Fault{VALIDATION_ERROR} for the whole claim window, no
    // matter how many mismatched packets arrived.
    let files: Vec<_> = std::fs::read_dir(dir.path())
        .unwrap()
        .filter_map(Result::ok)
        .filter(|e| e.file_name().to_string_lossy().ends_with(".sidecar.json"))
        .collect();
    assert_eq!(files.len(), 1);
    let sidecar =
        waddle_sidecar::sidecar_from_json(&std::fs::read_to_string(files[0].path()).unwrap())
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
        "expected exactly one validation fault for the claim window"
    );
}

/// GripperSpec mapping contract: the declared `GripperSpec` must be applied to the teleop gripper
/// command at intake, not copied verbatim. `open_value=0.04,
/// closed_value=0.0` against a fully-open (1.0) teleop command must carry
/// 0.04 into the ring, not 1.0.
#[test]
fn declared_gripper_spec_maps_teleop_gripper_at_intake() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (media, far) = LoopbackMedia::new();
    let gripper = pb::GripperSpec {
        kind: Some(pb::gripper_spec::Kind::Parallel(
            pb::gripper_spec::Parallel {
                open_value: 0.04,
                closed_value: 0.0,
                action_dim: -1,
            },
        )),
    };
    let session = Session::builder("e2e-gripper")
        .robot(twist_robot(Some(gripper)))
        .control(registry(&send_log))
        .media(media)
        .build()
        .unwrap();

    let mut ep = session.start_episode("gripper").unwrap();
    for _ in 0..5 {
        let _ = ep.gate(&[0.0; 6], None, None);
    }
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(&session, "claim-gripper", "teleop", ActorKind::Teleoperator);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));

    far.push(
        DataTopic::TeleopPose,
        &pb::TeleopStreamPacket {
            t_client_ns: 1,
            seq: 1,
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
                gripper: Some(1.0), // fully open, media-plane convention
            }],
            clutch_engaged: true,
            inputs: None,
        },
    )
    .unwrap();

    let deadline = Instant::now() + Duration::from_secs(5);
    let mut mapped_gripper: Option<Option<f64>> = None;
    while Instant::now() < deadline {
        match ep.gate(&[0.0; 6], None, None) {
            GateOutput::Substitute { action, .. } | GateOutput::Blend { action, .. } => {
                mapped_gripper = Some(action.gripper);
                break;
            }
            _ => std::thread::sleep(Duration::from_millis(5)),
        }
    }
    match mapped_gripper.expect("teleop stream never substituted") {
        Some(g) => assert!(
            (g - 0.04).abs() < 1e-9,
            "expected the declared open_value (0.04), got {g}"
        ),
        None => panic!("expected a mapped gripper value"),
    }

    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
}

/// GripperSpec mapping contract: no declared `GripperSpec` means passthrough, unchanged.
#[test]
fn absent_gripper_spec_passes_teleop_gripper_through_unchanged() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let (media, far) = LoopbackMedia::new();
    let session = Session::builder("e2e-gripper-passthrough")
        .robot(twist_robot(None))
        .control(registry(&send_log))
        .media(media)
        .build()
        .unwrap();

    let mut ep = session.start_episode("gripper-passthrough").unwrap();
    for _ in 0..5 {
        let _ = ep.gate(&[0.0; 6], None, None);
    }
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(
        &session,
        "claim-passthrough",
        "teleop",
        ActorKind::Teleoperator,
    );
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));

    far.push(
        DataTopic::TeleopPose,
        &pb::TeleopStreamPacket {
            t_client_ns: 1,
            seq: 1,
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
                gripper: Some(0.73),
            }],
            clutch_engaged: true,
            inputs: None,
        },
    )
    .unwrap();

    let deadline = Instant::now() + Duration::from_secs(5);
    let mut mapped_gripper: Option<Option<f64>> = None;
    while Instant::now() < deadline {
        match ep.gate(&[0.0; 6], None, None) {
            GateOutput::Substitute { action, .. } | GateOutput::Blend { action, .. } => {
                mapped_gripper = Some(action.gripper);
                break;
            }
            _ => std::thread::sleep(Duration::from_millis(5)),
        }
    }
    match mapped_gripper.expect("teleop stream never substituted") {
        Some(g) => assert!(
            (g - 0.73).abs() < 1e-9,
            "expected passthrough 0.73, got {g}"
        ),
        None => panic!("expected a passthrough gripper value"),
    }

    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
}

/// Verb-registration validation: the default handoff policy is HOLD_FIRST — every engage issues
/// `Verb::Hold` before the intervenor's first action lands. Building a
/// media-wired session (a real engage path: the teleoperator's clutch)
/// without a registered `hold` callable must fail loudly at build time,
/// never silently at the 10s engage timeout.
#[test]
fn build_fails_fast_when_hold_first_and_media_wired_without_hold() {
    let (media, _far) = LoopbackMedia::new();
    let registry = ControlRegistry {
        send: Some(Arc::new(
            |_chunk: &waddle_types::ActionChunk| -> Result<(), VerbError> { Ok(()) },
        )),
        ..Default::default()
    };
    let err = Session::builder("e2e-missing-hold")
        .robot(twist_robot(None))
        .control(registry)
        .media(media)
        .build()
        .expect_err("HOLD_FIRST + media wired + no hold must fail the build");
    assert!(
        matches!(
            &err,
            RuntimeError::MissingVerb { verb, .. } if *verb == "hold"
        ),
        "expected MissingVerb{{verb: \"hold\", ..}}, got {err:?}"
    );
    assert_eq!(
        err.to_string(),
        "handoff HOLD_FIRST requires a registered `hold` verb — register one \
         in your Control, or choose a different handoff policy"
    );
}

/// The green counterpart: HOLD_FIRST + media wired + hold registered stays
/// buildable (the existing e2e paths above already cover this in depth; this
/// is the focused regression for the build-time verb-registration check).
#[test]
fn build_ok_when_hold_first_and_media_wired_with_hold_registered() {
    let (media, _far) = LoopbackMedia::new();
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let session = Session::builder("e2e-hold-registered")
        .robot(twist_robot(None))
        .control(registry(&send_log))
        .media(media)
        .build()
        .expect("HOLD_FIRST + media wired + hold registered must build");
    session.shutdown();
}

/// Back-compat (verb-registration validation): a session built with no Control at all and no media
/// plane — the descriptors-only / minimal-local integration — must keep
/// working. Nothing wires a real engage path (no media, no transport), so
/// there is no dispatch for the build-time check to protect against; the
/// PyO3 shim's `create_session` has always accepted all-None verbs and must
/// not regress.
#[test]
fn build_ok_with_no_verbs_and_no_media() {
    let session = Session::builder("e2e-no-control")
        .robot(robot())
        .build()
        .expect("no verbs + no media must still build (back-compat)");
    session.shutdown();
}

/// Verb-registration validation: HOLD_FIRST is the only policy that unconditionally issues
/// `Verb::Hold` on engage — IMMEDIATE and CHUNK_BOUNDARY never do, so `hold`
/// is not a build-time requirement under them even with a media plane wired.
#[test]
fn build_ok_with_immediate_handoff_and_no_hold() {
    let (media, _far) = LoopbackMedia::new();
    let registry = ControlRegistry {
        send: Some(Arc::new(
            |_chunk: &waddle_types::ActionChunk| -> Result<(), VerbError> { Ok(()) },
        )),
        ..Default::default()
    };
    let session = Session::builder("e2e-immediate-no-hold")
        .robot(twist_robot(None))
        .control(registry)
        .media(media)
        .handoff(HandoffPolicy::Immediate { blend_ns: 0 })
        .build()
        .expect("IMMEDIATE handoff never requires hold");
    session.shutdown();
}

/// Verb-registration validation (delta-space degrade): FSM.md §5 refuses mid-chunk splice entry for
/// delta action spaces, so `waddle_fsm::begin_engage` silently degrades a
/// declared `HandoffPolicy::Immediate` to `HoldFirst` on the very first
/// engage whenever `space_contains_delta` is set (see
/// `waddle-fsm/src/session.rs`, and the conformance fixture
/// `handoff_immediate_mid_chunk.json`, which deliberately picks a
/// joint-position composite space to *avoid* this same degrade). The
/// build-time `hold` check above (`build_ok_with_immediate_handoff_and_no_hold`)
/// must not be fooled by the raw declared policy: over an `EePoseDelta`
/// space, declared IMMEDIATE + no `hold` is the exact bug this task closes,
/// just reached through the declared-IMMEDIATE path instead of the
/// declared-HOLD_FIRST default — it must fail the build the same way.
#[test]
fn build_fails_fast_when_immediate_over_delta_space_and_no_hold() {
    let (media, _far) = LoopbackMedia::new();
    let registry = ControlRegistry {
        send: Some(Arc::new(
            |_chunk: &waddle_types::ActionChunk| -> Result<(), VerbError> { Ok(()) },
        )),
        ..Default::default()
    };
    let err = Session::builder("e2e-immediate-delta-missing-hold")
        .robot(ee_delta_robot())
        .control(registry)
        .media(media)
        .handoff(HandoffPolicy::Immediate { blend_ns: 0 })
        .build()
        .expect_err(
            "IMMEDIATE over a delta space degrades to HOLD_FIRST at engage \
             and must require hold at build time",
        );
    assert!(
        matches!(
            &err,
            RuntimeError::MissingVerb { verb, .. } if *verb == "hold"
        ),
        "expected MissingVerb{{verb: \"hold\", ..}}, got {err:?}"
    );
}

/// Verb-registration validation (send side): the bypass pump can drive `Verb::Send` directly once a
/// claimed loop stalls (`claimed_while_stalled_bypass_drives_send_directly`
/// above) — that path exists the moment a media plane is wired, regardless
/// of handoff policy. An unregistered `send` must fail the build the same
/// way `hold` does.
#[test]
fn build_fails_fast_when_media_wired_without_send() {
    let (media, _far) = LoopbackMedia::new();
    let registry = ControlRegistry {
        hold: Some(Arc::new(|| Ok(()))),
        ..Default::default()
    };
    let err = Session::builder("e2e-missing-send")
        .robot(twist_robot(None))
        .control(registry)
        .media(media)
        .build()
        .expect_err("media wired + no send must fail the build");
    assert!(
        matches!(
            &err,
            RuntimeError::MissingVerb { verb, .. } if *verb == "send"
        ),
        "expected MissingVerb{{verb: \"send\", ..}}, got {err:?}"
    );
}

/// Verb-registration validation (review follow-up): `grant_and_engage` is a real, exported,
/// always-live engage path (used by "tests and local intervention sources",
/// per its own doc comment) with zero dependency on `self.media` — a
/// session that registers `send` directly for local intervention, with no
/// `.media(...)` call at all, is exactly as live an engage path as one
/// wired to a media plane. The `hold` check must not key on
/// `self.media.is_some()` alone, or this exact shape builds clean and then
/// reproduces the "clutch press, nothing happens" stall the moment
/// `grant_and_engage` is called (the originally reported stall, reached
/// without any media plane at all).
#[test]
fn build_fails_fast_when_hold_first_and_send_registered_without_media() {
    let registry = ControlRegistry {
        send: Some(Arc::new(
            |_chunk: &waddle_types::ActionChunk| -> Result<(), VerbError> { Ok(()) },
        )),
        ..Default::default()
    };
    let err = Session::builder("e2e-send-no-media-missing-hold")
        .robot(twist_robot(None))
        .control(registry)
        .build()
        .expect_err(
            "send registered without media is still a live grant_and_engage \
             path and must require hold",
        );
    assert!(
        matches!(
            &err,
            RuntimeError::MissingVerb { verb, .. } if *verb == "hold"
        ),
        "expected MissingVerb{{verb: \"hold\", ..}}, got {err:?}"
    );
}

/// The symmetric case: `hold` registered with no `send` and no media wired
/// is just as live a `grant_and_engage` path (engage doesn't need media to
/// reach a claimed/intervention state) — the bypass pump can still try to
/// drive `Verb::Send` once that loop stalls. Under `Immediate` handoff so
/// the `hold` requirement itself never fires here; this isolates the `send`
/// side of the same local-intervention gap.
#[test]
fn build_fails_fast_when_hold_registered_without_send_and_no_media() {
    let registry = ControlRegistry {
        hold: Some(Arc::new(|| Ok(()))),
        ..Default::default()
    };
    let err = Session::builder("e2e-hold-no-media-missing-send")
        .robot(twist_robot(None))
        .control(registry)
        .handoff(HandoffPolicy::Immediate { blend_ns: 0 })
        .build()
        .expect_err(
            "hold registered without media is still a live grant_and_engage \
             path and must require send",
        );
    assert!(
        matches!(
            &err,
            RuntimeError::MissingVerb { verb, .. } if *verb == "send"
        ),
        "expected MissingVerb{{verb: \"send\", ..}}, got {err:?}"
    );
}

/// Verb-registration validation (estop side): a missing `estop` must never fail the build (unlike
/// `hold`/`send`) — but the degradation must stay observable on the status
/// mirror the caller already polls, not silently swallowed.
#[test]
fn build_records_estop_unregistered_on_status_mirror() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let mut partial = registry(&send_log);
    partial.estop = None;
    let session = Session::builder("e2e-no-estop")
        .robot(robot())
        .control(partial)
        .build()
        .expect("missing estop must never fail the build");
    assert!(
        session.status().estop_unregistered,
        "missing estop must be recorded as observable degradation"
    );
    session.shutdown();
}

/// The counterpart: estop registered means no degradation is recorded.
#[test]
fn build_does_not_flag_estop_when_registered() {
    let send_log: SendLog = Arc::new(Mutex::new(Vec::new()));
    let mut full = registry(&send_log);
    full.estop = Some((Arc::new(|| Ok(())), waddle_runtime::EstopDecl::default()));
    let session = Session::builder("e2e-with-estop")
        .robot(robot())
        .control(full)
        .build()
        .expect("build with estop registered must succeed");
    assert!(!session.status().estop_unregistered);
    session.shutdown();
}
