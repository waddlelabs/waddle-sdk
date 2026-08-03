//! `ServerMsg::ResetProgress` handling (the `RequestReset` RPC's
//! inbound half): the plane-executed reset completion path, distinct from
//! the SDK-executed remote reset WINDOW machinery in
//! `reset_window_actuation.rs` (a window is `ResetWindowDirective`/
//! `ResetWindowEvent`, negotiated per FSM.md E19–E22; `ResetProgress` is a
//! services message that never touches the FSM's window guards at all).
//!
//! This closes a long-documented gap: a retake successor
//! opened under a session-level `Remote` PRE spec is born-claimed, so its
//! `EpisodeOpen`'s `pre_window` never opens (born-claimed suppression — the guard requires
//! `claim.is_none()`, and a born-claimed successor's claim survives the
//! retake) — nothing else in the runtime can ever complete that successor's
//! RESETTING. A plane-executed reset's `ResetProgress{DONE, result}`
//! injecting `SessionEvent::ResetResult` (the same event the inline/pump
//! paths already inject) is exactly the completion the design's "hand-reset
//! under the surviving claim, the plane confirms" story needs.

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use waddle_controlplane::{ClientMsg, InMemoryTransport, ServerMsg};
use waddle_fsm::{Phase, SessionEvent};
use waddle_runtime::{
    ControlRegistry, EpisodeOptions, ResetProgressPhase, ResetSpec, Session, VerbError,
    grant_and_engage,
};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActorKind, ClaimId, EpisodeId, GateMode};

fn robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "reset-progress-bot".into(),
        robot_id: "reset-progress-01".into(),
        cell_id: "cell-reset-progress".into(),
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

fn wait_for(session: &Session, pred: impl Fn(&waddle_runtime::Status) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let status = session.status();
        if pred(&status) {
            return;
        }
        if Instant::now() >= deadline {
            panic!("timed out waiting on status: {status:?}");
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// A retake successor under a session-level `Remote` PRE spec sits in
/// RESETTING with no window (born-claimed suppression) —
/// only a plane-executed `ResetProgress{DONE, result}` can complete it.
/// Progress before DONE must never transition anything.
#[test]
fn reset_progress_done_completes_retake_successor_with_no_window() {
    let successor_id: Arc<Mutex<Option<EpisodeId>>> = Arc::new(Mutex::new(None));
    let successor_for_script = successor_id.clone();
    let sent_progress = Arc::new(AtomicBool::new(false));
    let sent_for_script = sent_progress.clone();

    // The script watches the gate stream for the successor's OWN RESETTING
    // transition (its born-claimed `EpisodeOpen` emission) and then pushes
    // the plane-executed reset's progress — `InMemoryTransport` doesn't
    // model per-RPC framing, so (like every other scripted-transport test
    // in this suite) this pushes `ServerMsg`s directly rather than routing
    // through a real `RequestReset` server-stream.
    let transport = InMemoryTransport::new(move |msg, tx: &Sender<ServerMsg>| {
        if let ClientMsg::Register(_) = &msg {
            let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse::default()));
            return;
        }
        let ClientMsg::Gate(pb::GateClientMessage {
            msg: Some(pb::gate_client_message::Msg::Event(ev)),
        }) = &msg
        else {
            return;
        };
        let Some(pb::episode_event::Event::State(state)) = &ev.event else {
            return;
        };
        if state.to != pb::EpisodeState::Resetting as i32 {
            return;
        }
        let is_successor = successor_for_script
            .lock()
            .as_ref()
            .is_some_and(|id| id.as_str() == ev.episode_id);
        if !is_successor || sent_for_script.swap(true, Ordering::SeqCst) {
            return;
        }
        // Progress before DONE must not transition anything. A deliberate
        // gap before DONE (this handler runs on the transport's own server
        // thread, never a runtime thread) gives the test a real window to
        // observe the EXECUTING phase land on the mirror before DONE
        // supersedes it — back-to-back sends would race the test's poll.
        let _ = tx.send(ServerMsg::ResetProgress(pb::ResetProgress {
            phase: pb::ResetPhase::Executing as i32,
            strategy: "auto-reset".into(),
            detail: "planning".into(),
            result: None,
        }));
        std::thread::sleep(Duration::from_millis(150));
        let _ = tx.send(ServerMsg::ResetProgress(pb::ResetProgress {
            phase: pb::ResetPhase::Done as i32,
            strategy: "auto-reset".into(),
            detail: "scene reset".into(),
            result: Some(pb::ResetResult {
                ok: true,
                verification: Some(pb::ResetVerification {
                    verified: true,
                    ..Default::default()
                }),
                ..Default::default()
            }),
        }));
    });

    let session = Session::builder("reset-progress-retake")
        .robot(robot())
        .control(registry())
        .transport(transport)
        .pre_reset(ResetSpec::Remote {
            actor: ActorKind::Teleoperator,
            prompt: "clear the table".into(),
            timeout_ns: 30_000_000_000,
        })
        .build()
        .unwrap();

    // The FIRST episode disables the session-level Remote pre-reset for
    // itself (per-episode override — `EpisodeOptions`) so it proceeds
    // straight to Running; the retake successor below has no such override
    // (reducer-opened successors never go through `start_episode_with`) and
    // so falls back to the session's Remote default — exactly the
    // born-claimed gap this test exercises.
    let mut ep1 = session
        .start_episode_with(
            "first",
            EpisodeOptions {
                pre_reset: Some(None),
                ..EpisodeOptions::default()
            },
        )
        .unwrap();
    let _ = ep1.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(&session, "claim-rp", "teleop", ActorKind::Teleoperator);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));

    let successor = EpisodeId::new("ep-reset-progress-successor");
    *successor_id.lock() = Some(successor.clone());
    session.inject(SessionEvent::Retake {
        claim: ClaimId::new("claim-rp"),
        initiator: ActorKind::Teleoperator,
        successor: successor.clone(),
        at: waddle_types::MonoNs(3_000_000),
    });

    // Born-claimed suppression (born-claimed suppression): the successor reaches RESETTING
    // with no window ever opening — the pre-existing gap this task closes.
    wait_for(&session, |s| {
        s.episode_id.as_ref() == Some(&successor)
            && matches!(s.episode_state, Some(Phase::Resetting))
    });
    assert_ne!(
        session.status().gate_mode,
        Some(GateMode::Reset),
        "no reset window ever opens for a born-claimed successor's Remote PRE spec"
    );

    // The EXECUTING progress message lands on the mirror but transitions
    // nothing: still RESETTING a moment later.
    wait_for(&session, |s| {
        s.reset_progress.as_ref().map(|p| p.phase) == Some(ResetProgressPhase::Executing)
    });
    std::thread::sleep(Duration::from_millis(50));
    assert!(
        matches!(session.status().episode_state, Some(Phase::Resetting)),
        "progress before DONE must not transition anything"
    );

    // DONE completes it exactly like the inline/pump paths would.
    wait_for(&session, |s| {
        s.episode_id.as_ref() == Some(&successor) && matches!(s.episode_state, Some(Phase::Ready))
    });
    assert_eq!(
        session.status().reset_progress.map(|p| p.phase),
        Some(ResetProgressPhase::Done)
    );
    session.shutdown();
}

/// A `ResetProgress{DONE}` with no episode currently RESETTING (e.g. late
/// or duplicated) is rejected by the FSM guard (E19b/"outside RESETTING")
/// like any other illegal `ResetResult` — it must not panic, crash the
/// pump, or otherwise disturb the mirror beyond the progress field itself.
#[test]
fn reset_progress_done_outside_resetting_is_harmless() {
    let tx_slot: Arc<Mutex<Option<Sender<ServerMsg>>>> = Arc::new(Mutex::new(None));
    let slot_in = tx_slot.clone();
    let transport = InMemoryTransport::new(move |msg, tx: &Sender<ServerMsg>| {
        if let ClientMsg::Register(_) = &msg {
            *slot_in.lock() = Some(tx.clone());
            let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse::default()));
        }
    });

    let session = Session::builder("reset-progress-stray")
        .robot(robot())
        .control(registry())
        .transport(transport)
        .build()
        .unwrap();

    let mut ep = session.start_episode("task").unwrap();
    let _ = ep.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    let tx = {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if let Some(tx) = tx_slot.lock().clone() {
                break tx;
            }
            assert!(Instant::now() < deadline, "never registered");
            std::thread::sleep(Duration::from_millis(5));
        }
    };
    let _ = tx.send(ServerMsg::ResetProgress(pb::ResetProgress {
        phase: pb::ResetPhase::Done as i32,
        strategy: "auto-reset".into(),
        detail: "stray".into(),
        result: Some(pb::ResetResult {
            ok: true,
            ..Default::default()
        }),
    }));

    wait_for(&session, |s| {
        s.reset_progress.as_ref().map(|p| p.phase) == Some(ResetProgressPhase::Done)
    });
    std::thread::sleep(Duration::from_millis(50));
    assert!(
        matches!(session.status().episode_state, Some(Phase::Running)),
        "a stray DONE outside RESETTING must be rejected, not crash or transition anything"
    );
    session.shutdown();
}
