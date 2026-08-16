//! Hosted-run admission through the real client and runtime pumps.

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use waddle_controlplane::{ClientMsg, InMemoryTransport, ServerMsg};
use waddle_fsm::Phase;
use waddle_runtime::{ControlRegistry, Session, VerbError};
use waddle_types::pb::v0 as pb;

type StatusLog = Arc<Mutex<Vec<pb::HostedRunStatus>>>;
type TxSlot = Arc<Mutex<Option<Sender<ServerMsg>>>>;

fn robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "hosted-run-bot".into(),
        robot_id: "hosted-run-01".into(),
        cell_id: "cell-hosted-run".into(),
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
        grants: vec![
            pb::Grant {
                verb: pb::Verb::Hold as i32,
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

fn plane() -> (Arc<InMemoryTransport>, StatusLog, TxSlot) {
    let statuses: StatusLog = Arc::new(Mutex::new(Vec::new()));
    let tx_slot: TxSlot = Arc::new(Mutex::new(None));
    let statuses_in = statuses.clone();
    let tx_in = tx_slot.clone();
    let transport = InMemoryTransport::new(move |msg, tx| match msg {
        ClientMsg::Register(_) => {
            *tx_in.lock() = Some(tx.clone());
            let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse {
                accepted_feature_flags: vec![waddle_controlplane::flags::HOSTED_RUNS.into()],
                ..Default::default()
            }));
        }
        ClientMsg::Gate(gate) => {
            if let Some(pb::gate_client_message::Msg::HostedRunStatus(status)) = gate.msg {
                statuses_in.lock().push(status);
            }
        }
        _ => {}
    });
    (transport, statuses, tx_slot)
}

fn wait_for(what: &str, pred: impl Fn() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while !pred() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        std::thread::sleep(Duration::from_millis(5));
    }
}

fn server_tx(slot: &TxSlot) -> Sender<ServerMsg> {
    wait_for("server sender", || slot.lock().is_some());
    slot.lock().clone().unwrap()
}

fn request(tx: &Sender<ServerMsg>, id: &str, timeout_ns: i64, task: &str) {
    let _ = tx.send(ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::HostedRunRequest(
            pb::HostedRunRequest {
                request_id: id.into(),
                task_metadata: [("task".into(), task.into())].into_iter().collect(),
                timeout_ns,
            },
        )),
    }));
}

#[test]
fn duplicate_and_reconnect_are_idempotent_and_disconnect_holds() {
    let holds = Arc::new(AtomicUsize::new(0));
    let holds_in = holds.clone();
    let registry = ControlRegistry {
        send: Some(Arc::new(
            |_: &waddle_types::ActionChunk| -> Result<(), VerbError> { Ok(()) },
        )),
        hold: Some(Arc::new(move || {
            holds_in.fetch_add(1, Ordering::SeqCst);
            Ok(())
        })),
        ..Default::default()
    };
    let (transport, statuses, tx_slot) = plane();
    let session = Session::builder("hosted-runs")
        .robot(robot())
        .control(registry)
        .transport(transport.clone())
        .build()
        .unwrap();

    wait_for("hosted-run negotiation", || {
        session.status().hosted_runs_negotiated
    });
    let tx = server_tx(&tx_slot);
    request(&tx, "run-1", 30_000_000_000, "first");
    wait_for("accepted status", || !statuses.lock().is_empty());

    let first = statuses.lock()[0].clone();
    assert_eq!(
        first.kind,
        pb::HostedRunStatusKind::Accepted as i32,
        "{first:?}"
    );
    assert!(!first.episode_id.is_empty());
    wait_for("ordinary RUNNING episode", || {
        matches!(session.status().episode_state, Some(Phase::Running))
    });

    request(&tx, "run-1", 1, "different payload");
    wait_for("duplicate status", || statuses.lock().len() >= 2);
    assert_eq!(
        statuses.lock()[1],
        first,
        "a duplicate returns the original admission verbatim"
    );

    request(&tx, "run-2", 30_000_000_000, "second");
    wait_for("busy status", || statuses.lock().len() >= 3);
    let busy = statuses.lock()[2].clone();
    assert_eq!(busy.kind, pb::HostedRunStatusKind::Busy as i32);
    assert_eq!(busy.episode_id, first.episode_id);

    // The stashed server sender and this test's clone both keep the client
    // receive side alive, so release them before severing the transport.
    drop(tx);
    *tx_slot.lock() = None;
    transport.refuse_connections();
    transport.drop_connections();
    wait_for("partition", || !session.status().plane_connected);
    wait_for("priority hold", || holds.load(Ordering::SeqCst) >= 1);
    wait_for("abort terminal", || {
        matches!(
            session.status().episode_state,
            Some(Phase::Terminal(waddle_types::TerminalOutcome::Abort))
        )
    });

    transport.allow_connections();
    wait_for("reconnect and renegotiation", || {
        session.status().plane_connected && session.status().hosted_runs_negotiated
    });
    let retry_tx = server_tx(&tx_slot);
    request(&retry_tx, "run-1", 30_000_000_000, "retry");
    wait_for("explicit retry result", || statuses.lock().len() >= 4);
    assert_eq!(
        statuses.lock()[3],
        first,
        "explicit retry learns the original result without reopening motion"
    );
    assert!(matches!(
        session.status().episode_state,
        Some(Phase::Terminal(waddle_types::TerminalOutcome::Abort))
    ));

    drop(retry_tx);
    *tx_slot.lock() = None;
    session.shutdown();
}

#[test]
fn admission_cache_is_bounded_without_losing_cached_duplicates() {
    let registry = ControlRegistry {
        send: Some(Arc::new(
            |_: &waddle_types::ActionChunk| -> Result<(), VerbError> { Ok(()) },
        )),
        hold: Some(Arc::new(|| Ok(()))),
        ..Default::default()
    };
    let (transport, statuses, tx_slot) = plane();
    let session = Session::builder("hosted-run-capacity")
        .robot(robot())
        .control(registry)
        .transport(transport)
        .build()
        .unwrap();

    wait_for("hosted-run negotiation", || {
        session.status().hosted_runs_negotiated
    });
    let tx = server_tx(&tx_slot);
    for index in 0..1024 {
        request(&tx, &format!("cached-{index}"), 0, "invalid");
    }
    wait_for("filled admission cache", || statuses.lock().len() >= 1024);
    let first = statuses.lock()[0].clone();
    assert_eq!(
        first.detail.as_ref().map(|detail| detail.code.as_str()),
        Some("invalid_timeout")
    );

    request(&tx, "cached-0", 30_000_000_000, "changed");
    request(&tx, "overflow", 30_000_000_000, "new");
    request(&tx, "overflow", 1, "changed");
    wait_for("cache overflow answers", || statuses.lock().len() >= 1027);
    let got = statuses.lock();
    assert_eq!(got[1024], first, "cached ids remain verbatim-idempotent");
    assert_eq!(
        got[1025].detail.as_ref().map(|detail| detail.code.as_str()),
        Some("capacity_exceeded")
    );
    assert_eq!(
        got[1026], got[1025],
        "uncached overflow ids still receive a deterministic rejection"
    );
    drop(got);

    drop(tx);
    *tx_slot.lock() = None;
    session.shutdown();
}
