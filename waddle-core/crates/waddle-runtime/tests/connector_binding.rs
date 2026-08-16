//! Exact connector registration and per-connection negotiation.

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use waddle_controlplane::{ClientMsg, InMemoryTransport, ServerMsg};
use waddle_runtime::{ConnectorBinding, Session};
use waddle_types::pb::v0 as pb;

fn robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "connector-probe".into(),
        robot_id: "connector-probe".into(),
        cell_id: "connector-cell".into(),
        action_space: Some(pb::ActionSpace {
            space: Some(pb::action_space::Space::JointPosition(pb::JointPosition {
                joints: vec![pb::JointDescriptor {
                    name: "joint".into(),
                    ..Default::default()
                }],
            })),
            rate_hz: 10.0,
            ..Default::default()
        }),
        ..Default::default()
    }
}

fn wait_for(session: &Session, pred: impl Fn(&waddle_runtime::Status) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let status = session.status();
        if pred(&status) {
            return;
        }
        assert!(Instant::now() < deadline, "timed out on status {status:?}");
        std::thread::sleep(Duration::from_millis(5));
    }
}

#[test]
fn authorization_probe_carries_exact_binding_and_renegotiates_after_disconnect() {
    let registrations = Arc::new(Mutex::new(Vec::<pb::RegisterRequest>::new()));
    let registrations_in = registrations.clone();
    let connection = Arc::new(AtomicUsize::new(0));
    let connection_in = connection.clone();
    let heartbeats = Arc::new(AtomicUsize::new(0));
    let heartbeats_in = heartbeats.clone();
    let transport = InMemoryTransport::new(move |msg, tx| match msg {
        ClientMsg::Register(request) => {
            registrations_in.lock().push(request);
            let nth = connection_in.fetch_add(1, Ordering::SeqCst);
            let accepted_feature_flags = if nth == 0 {
                vec![waddle_controlplane::flags::CONNECTOR_BINDING.into()]
            } else {
                Vec::new()
            };
            let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse {
                session_id: "connector-session".into(),
                accepted_feature_flags,
                ..Default::default()
            }));
        }
        ClientMsg::Heartbeat(ping) => {
            assert_eq!(ping.session_id, "connector-session");
            assert!(ping.t_ns > 0);
            heartbeats_in.fetch_add(1, Ordering::SeqCst);
        }
        _ => {}
    });
    let session = Session::builder("local-site-id")
        .robot(robot())
        .connector_binding(
            ConnectorBinding::new("customer-1", "project-1", "workspace-1")
                .authorization_only(true),
        )
        .transport(transport.clone())
        .build()
        .unwrap();

    wait_for(&session, |status| {
        status.plane_registered && status.connector_binding_negotiated
    });
    wait_for(&session, |_| heartbeats.load(Ordering::SeqCst) > 0);
    let first = registrations.lock()[0].clone();
    assert_eq!(first.customer_id, "customer-1");
    assert_eq!(first.project, "project-1");
    assert_eq!(first.workspace_id, "workspace-1");
    assert!(first.authorization_only);
    assert!(
        first
            .feature_flags
            .iter()
            .any(|flag| flag == waddle_controlplane::flags::CONNECTOR_BINDING)
    );
    assert!(
        !first
            .feature_flags
            .iter()
            .any(|flag| flag == waddle_controlplane::flags::HOSTED_RUNS)
    );

    transport.refuse_connections();
    transport.drop_connections();
    wait_for(&session, |status| !status.plane_registered);
    assert!(!session.status().connector_binding_negotiated);

    transport.allow_connections();
    wait_for(&session, |status| {
        status.plane_registered && !status.connector_binding_negotiated
    });
    assert!(registrations.lock().len() >= 2);
    session.shutdown();
}
