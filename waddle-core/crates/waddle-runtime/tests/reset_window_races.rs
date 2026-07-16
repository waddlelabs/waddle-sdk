//! Regression rig for the deferred-mint race: the reducer answers
//! `Effect::MintLeaseToken` via the TAIL of its own event queue, so a plane
//! that sends ENGAGE and COMPLETE back-to-back gets its COMPLETE processed
//! BEFORE the engage's lease-mint answer. Pre-fix the FSM honored that
//! COMPLETE — closing the window and releasing the reset claim underneath
//! the in-flight mint — and then panicked applying the stale answer
//! ("reset claim held"), killing the reducer thread with no catch_unwind, so
//! every blocked caller (`start_episode*`, `terminate_episode`) hung
//! forever. These tests drive that exact production ordering end-to-end and
//! assert the session always resolves: the early COMPLETE is rejected while
//! the engage is in flight, the plane's retried COMPLETE lands after
//! ENGAGED, and the reducer never dies.

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::mpsc;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use waddle_controlplane::{ClientMsg, InMemoryTransport, ServerMsg};
use waddle_fsm::Phase;
use waddle_runtime::{ResetSpec, Session, reset_window_complete, reset_window_engage};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActorKind, GateMode, TerminalOutcome};

fn joint_robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "reset-race-bot".into(),
        robot_id: "reset-race-01".into(),
        cell_id: "cell-reset-race".into(),
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
        grants: vec![pb::Grant {
            verb: pb::Verb::Hold as i32,
            declared_latency_bound_ns: Some(50_000_000),
            ..Default::default()
        }],
        ..Default::default()
    }
}

fn wait_for(session: &Session, what: &str, pred: impl Fn(&waddle_runtime::Status) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        if pred(&session.status()) {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "timed out waiting on status: {what} (reducer dead or hung?)"
        );
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn engage_directive(claim_id: &str) -> ServerMsg {
    ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::ResetWindow(
            pb::ResetWindowDirective {
                kind: pb::ResetWindowDirectiveKind::Engage as i32,
                reset: pb::ResetKind::Pre as i32,
                claim: Some(pb::Claim {
                    claim_id: claim_id.to_owned(),
                    actor: Some(pb::ActorRef {
                        kind: pb::ActorKind::Teleoperator as i32,
                        ..Default::default()
                    }),
                    source_name: "plane-script".into(),
                    self_initiated: false,
                    ..Default::default()
                }),
                result: None,
                directive_id: None,
            },
        )),
    })
}

fn complete_directive(claim_id: &str) -> ServerMsg {
    ServerMsg::Gate(pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::ResetWindow(
            pb::ResetWindowDirective {
                kind: pb::ResetWindowDirectiveKind::Complete as i32,
                reset: pb::ResetKind::Pre as i32,
                claim: Some(pb::Claim {
                    claim_id: claim_id.to_owned(),
                    ..Default::default()
                }),
                result: Some(pb::ResetResult {
                    ok: true,
                    verification: Some(pb::ResetVerification {
                        verified: true,
                        ..Default::default()
                    }),
                    ..Default::default()
                }),
                directive_id: None,
            },
        )),
    })
}

/// The full production path: a scripted plane transport whose
/// `forward_server_msg`-decoded ENGAGE and COMPLETE land back-to-back in the
/// reducer's queue — ahead of the engage's own mint answer — then the
/// COMPLETE is retried (as a real plane would after seeing no COMPLETED
/// event) until the window resolves. Asserts no hang and a sane end state.
#[test]
fn back_to_back_engage_complete_directives_resolve_without_hanging() {
    let session_cell: Arc<Mutex<Option<Session>>> = Arc::new(Mutex::new(None));

    let cell_for_script = session_cell.clone();
    let transport = InMemoryTransport::new(move |msg, tx| {
        if let ClientMsg::Register(_) = &msg {
            let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse::default()));
            let cell = cell_for_script.clone();
            let tx = tx.clone();
            std::thread::spawn(move || {
                let session = loop {
                    if let Some(s) = cell.lock().clone() {
                        break s;
                    }
                    std::thread::sleep(Duration::from_millis(2));
                };
                // Wait only for the window to be OPEN (RESETTING), then send
                // ENGAGE and COMPLETE back-to-back — deliberately NOT waiting
                // for the ENGAGED/gate-RESET observation a well-behaved plane
                // client would wait for.
                loop {
                    let st = session.status();
                    if st.shutdown {
                        return;
                    }
                    if matches!(st.episode_state, Some(Phase::Resetting)) {
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(2));
                }
                let _ = tx.send(engage_directive("race-claim"));
                let _ = tx.send(complete_directive("race-claim"));
                // The plane's retry loop: keep re-sending COMPLETE until the
                // episode leaves the reset phase.
                loop {
                    let st = session.status();
                    if st.shutdown || !matches!(st.episode_state, Some(Phase::Resetting)) {
                        return;
                    }
                    let _ = tx.send(complete_directive("race-claim"));
                    std::thread::sleep(Duration::from_millis(5));
                }
            });
        }
    });

    let session = Session::builder("e2e-reset-race-transport")
        .robot(joint_robot())
        .transport(transport)
        .pre_reset(ResetSpec::Remote {
            actor: ActorKind::Teleoperator,
            prompt: "clear the table".into(),
            timeout_ns: 30_000_000_000,
        })
        .build()
        .unwrap();
    *session_cell.lock() = Some(session.clone());

    // `start_episode` blocks in RESETTING until the window resolves; a dead
    // reducer would hang it forever, so join it through a channel with a
    // deadline.
    let (done_tx, done_rx) = mpsc::channel();
    {
        let session = session.clone();
        std::thread::spawn(move || {
            let _ = done_tx.send(session.start_episode("towel"));
        });
    }
    let mut ep = done_rx
        .recv_timeout(Duration::from_secs(10))
        .expect("start_episode must resolve: the reducer died or the window never closed")
        .expect("the retried COMPLETE must resolve the window to READY");

    // Sane end state: claim released, gate home, and the session still runs
    // a normal rollout to terminal (the reducer thread is alive).
    wait_for(&session, "gate back to PASSTHROUGH", |s| {
        s.gate_mode == Some(GateMode::Passthrough)
    });
    assert!(!session.status().claim_active, "reset claim released (C7)");
    let _ = ep.gate(&[0.0; 3], None, None);
    wait_for(&session, "RUNNING", |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    ep.terminate(TerminalOutcome::Success, "done");
    assert!(ep.done());
    session.shutdown();
}

/// Many rolls of the same race without a transport: the
/// `reset_window_engage`/`reset_window_complete` helpers inject the exact
/// two-event decode `forward_server_msg` produces, back-to-back from one
/// thread — so COMPLETE is queued ahead of the engage's mint answer far more
/// reliably than the transport path can arrange. Ten episodes; each must
/// resolve. Pre-fix, the first race hit panicked the reducer and every
/// subsequent wait timed out.
#[test]
fn repeated_back_to_back_engage_complete_never_kills_the_reducer() {
    let session = Session::builder("e2e-reset-race-repeated")
        .robot(joint_robot())
        .pre_reset(ResetSpec::Remote {
            actor: ActorKind::Teleoperator,
            prompt: "clear the table".into(),
            timeout_ns: 30_000_000_000,
        })
        .build()
        .unwrap();

    for i in 0..10 {
        let claim = format!("race-claim-{i}");
        let (done_tx, done_rx) = mpsc::channel();
        {
            let session = session.clone();
            let task = format!("towel-{i}");
            std::thread::spawn(move || {
                let _ = done_tx.send(session.start_episode(&task));
            });
        }
        wait_for(&session, "window open (RESETTING)", |s| {
            matches!(s.episode_state, Some(Phase::Resetting))
        });

        // The production ordering: ENGAGE's two events then COMPLETE, all
        // queued before the reducer can answer the engage's mint.
        reset_window_engage(&session, &claim, "teleop", ActorKind::Teleoperator);
        reset_window_complete(&session, &claim, true, Some(true));

        // The plane's retry: re-send COMPLETE until the episode resolves.
        let deadline = Instant::now() + Duration::from_secs(10);
        let mut ep = loop {
            match done_rx.recv_timeout(Duration::from_millis(20)) {
                Ok(res) => break res.expect("window must resolve to READY"),
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    assert!(
                        Instant::now() < deadline,
                        "episode {i} never resolved: the reducer died (deferred-mint race)"
                    );
                    reset_window_complete(&session, &claim, true, Some(true));
                }
                Err(mpsc::RecvTimeoutError::Disconnected) => {
                    panic!("start_episode thread died without a result")
                }
            }
        };

        assert!(!session.status().claim_active, "reset claim released (C7)");
        let _ = ep.gate(&[0.0; 3], None, None);
        wait_for(&session, "RUNNING", |s| {
            matches!(s.episode_state, Some(Phase::Running))
        });
        ep.terminate(TerminalOutcome::Success, "done");
        assert!(ep.done());
    }
    session.shutdown();
}
