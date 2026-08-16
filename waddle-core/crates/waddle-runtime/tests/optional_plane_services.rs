//! Optional task, calibration, and artifact services through the real client.

#![allow(clippy::disallowed_methods)] // wall-clock deadline is test-only

use std::sync::Arc;
use std::time::{Duration, Instant};

use waddle_controlplane::{ClientMsg, InMemoryTransport, ServerMsg};
use waddle_runtime::{ControlRegistry, Session, VerbError};
use waddle_types::pb::v0 as pb;

fn robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "optional-service-bot".into(),
        robot_id: "optional-01".into(),
        cell_id: "cell-optional".into(),
        action_space: Some(pb::ActionSpace {
            space: Some(pb::action_space::Space::JointPosition(pb::JointPosition {
                joints: vec![pb::JointDescriptor {
                    name: "joint".into(),
                    ..Default::default()
                }],
            })),
            rate_hz: 50.0,
            ..Default::default()
        }),
        ..Default::default()
    }
}

fn registry() -> ControlRegistry {
    ControlRegistry {
        send: Some(Arc::new(
            |_: &waddle_types::ActionChunk| -> Result<(), VerbError> { Ok(()) },
        )),
        hold: Some(Arc::new(|| Ok(()))),
        ..Default::default()
    }
}

fn wait_for_negotiation(session: &Session) {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let status = session.status();
        if status.task_sessions_negotiated
            && status.calibration_measurements_negotiated
            && status.workspace_artifacts_negotiated
        {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "optional services never negotiated"
        );
        std::thread::yield_now();
    }
}

#[test]
fn optional_services_are_negotiated_correlated_and_bounded() {
    let transport = InMemoryTransport::new(|msg, tx| match msg {
        ClientMsg::Register(_) => {
            let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse {
                accepted_feature_flags: vec![
                    waddle_controlplane::flags::TASK_SESSIONS.into(),
                    waddle_controlplane::flags::CALIBRATION_MEASUREMENTS.into(),
                    waddle_controlplane::flags::WORKSPACE_ARTIFACTS.into(),
                ],
                ..Default::default()
            }));
            let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
                msg: Some(pb::gate_server_message::Msg::CalibrationMeasurementRequest(
                    pb::CalibrationMeasurementRequest {
                        calibration_id: "cal-1".into(),
                        sample_id: "sample-1".into(),
                        camera: "wrist".into(),
                        frame_seq: 7,
                        x: 10,
                        y: 20,
                    },
                )),
            }));
        }
        ClientMsg::Gate(gate) => match gate.msg {
            Some(pb::gate_client_message::Msg::TaskSessionRequest(request)) => {
                let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
                    msg: Some(pb::gate_server_message::Msg::TaskSessionEvent(
                        pb::TaskSessionEvent {
                            request_id: request.request_id,
                            task_session_id: "named-id".into(),
                            name: request.name,
                            sequence: 1,
                            kind: pb::TaskSessionEventKind::Text as i32,
                            text: "ready".into(),
                            role: "assistant".into(),
                            ..Default::default()
                        },
                    )),
                }));
            }
            Some(pb::gate_client_message::Msg::CalibrationMeasurement(measurement)) => {
                let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
                    msg: Some(pb::gate_server_message::Msg::CalibrationUpdate(
                        pb::CalibrationUpdate {
                            calibration_id: measurement.calibration_id,
                            kind: pb::CalibrationUpdateKind::Accepted as i32,
                            camera: measurement.camera,
                            frame_seq: measurement.frame_seq,
                            sequence: 1,
                            ..Default::default()
                        },
                    )),
                }));
            }
            Some(pb::gate_client_message::Msg::WorkspaceArtifactRequest(request)) => {
                let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
                    msg: Some(pb::gate_server_message::Msg::WorkspaceArtifactReady(
                        pb::WorkspaceArtifactReady {
                            request_id: request.request_id,
                            artifact_id: "artifact".into(),
                            sha256: "00".repeat(32),
                            size_bytes: 12,
                            download_ref: "one-time".into(),
                            expires_unix_ns: 123,
                            ..Default::default()
                        },
                    )),
                }));
            }
            _ => {}
        },
        _ => {}
    });

    let session = Session::builder("optional-services")
        .robot(robot())
        .control(registry())
        .transport(transport)
        .build()
        .unwrap();
    wait_for_negotiation(&session);

    let requests = session.calibration_measurement_requests(0, Duration::from_secs(1));
    assert_eq!(requests.len(), 1);
    assert_eq!(requests[0].0, 1);
    assert_eq!(requests[0].1.sample_id, "sample-1");

    session
        .submit_task_session(pb::TaskSessionRequest {
            request_id: "create-1".into(),
            name: "inspection".into(),
            kind: pb::TaskSessionRequestKind::Create as i32,
            ..Default::default()
        })
        .unwrap();
    let task = session.task_session_events("create-1", 0, Duration::from_secs(1));
    assert_eq!(task.len(), 1);
    assert_eq!(task[0].task_session_id, "named-id");
    assert_eq!(task[0].text, "ready");

    session
        .submit_calibration_measurement(pb::CalibrationMeasurement {
            calibration_id: "cal-1".into(),
            sample_id: "sample-1".into(),
            camera: "wrist".into(),
            frame_seq: 7,
            t_ns: 100,
            frame_id: "wrist_optical".into(),
            point: Some(pb::Vec3 {
                x: 0.1,
                y: 0.2,
                z: 0.3,
            }),
            depth_m: Some(0.3),
        })
        .unwrap();
    let calibration = session.calibration_updates("cal-1", 0, Duration::from_secs(1));
    assert_eq!(calibration.len(), 1);
    assert_eq!(calibration[0].frame_seq, 7);

    session
        .request_workspace_artifact(pb::WorkspaceArtifactRequest {
            request_id: "export-1".into(),
            graph_ids: vec!["inspect".into()],
            calibration_names: vec!["wrist".into()],
        })
        .unwrap();
    let artifact = session.workspace_artifact_events("export-1", Duration::from_secs(1));
    assert_eq!(artifact.len(), 1);
    assert_eq!(artifact[0].download_ref, "one-time");

    let invalid = pb::CalibrationMeasurement {
        calibration_id: "cal-1".into(),
        sample_id: "sample-2".into(),
        camera: "wrist".into(),
        frame_id: "wrist_optical".into(),
        point: Some(pb::Vec3 {
            x: f64::NAN,
            ..Default::default()
        }),
        depth_m: Some(0.3),
        ..Default::default()
    };
    assert!(session.submit_calibration_measurement(invalid).is_err());

    session
        .submit_task_session(pb::TaskSessionRequest {
            request_id: "wide-name".into(),
            name: "n".repeat(200),
            kind: pb::TaskSessionRequestKind::Create as i32,
            ..Default::default()
        })
        .unwrap();
    assert!(
        session
            .submit_task_session(pb::TaskSessionRequest {
                request_id: "too-wide-name".into(),
                name: "n".repeat(201),
                kind: pb::TaskSessionRequestKind::Create as i32,
                ..Default::default()
            })
            .is_err()
    );
    assert!(
        session
            .submit_task_session(pb::TaskSessionRequest {
                request_id: "too-wide-id".into(),
                task_session_id: "s".repeat(201),
                kind: pb::TaskSessionRequestKind::History as i32,
                ..Default::default()
            })
            .is_err()
    );
    session.shutdown();
}
