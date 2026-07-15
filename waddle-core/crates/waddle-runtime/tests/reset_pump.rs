//! Task 9 — reset pump, reducer effects, mirror semantics (design §D4):
//! `episode_done` flips at `Phase::PostReset` (the outcome is pinned, so the
//! rollout is over from the caller's view — and `terminate_episode` becomes
//! a no-op there, fixing the `_Rollout.__exit__` race); the reducer routes
//! `Effect::SetPostResetFailed` onto the sidecar and stamps
//! `post_reset_declared` from `EpisodeOpen`; and `spawn_reset_pump` is the
//! single scripted-hook invocation site (mirror-watch), fixing the
//! reducer-opened retake-successor gap (successors never received a
//! `ResetResult` before it).

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, Sender};
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use waddle_fsm::{Phase, SessionEvent};
use waddle_runtime::{ResetHook, ResetSpec, Session};
use waddle_types::TerminalOutcome;
use waddle_types::pb::v0 as pb;

fn robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "reset-pump-bot".into(),
        robot_id: "reset-pump-01".into(),
        cell_id: "cell-reset-pump".into(),
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

/// A hook that blocks until the test releases it, so `Phase::PostReset` (or
/// `Phase::Resetting`) can be observed deterministically no matter how fast
/// the pump services it. Returns `(ok, Some(true))`.
fn gated_hook(ok: bool, invoked: Arc<AtomicUsize>) -> (ResetHook, Sender<()>) {
    let (release_tx, release_rx) = std::sync::mpsc::channel::<()>();
    let release_rx: Arc<Mutex<Receiver<()>>> = Arc::new(Mutex::new(release_rx));
    let hook: ResetHook = Arc::new(move |_task: &str| {
        invoked.fetch_add(1, Ordering::SeqCst);
        // Wait for the test's release (or its teardown dropping the sender).
        let _ = release_rx.lock().recv_timeout(Duration::from_secs(5));
        (ok, Some(true))
    });
    (hook, release_tx)
}

// --- episode_done / terminate_episode at POST_RESET ----------------------

#[test]
fn episode_done_flips_at_post_reset_and_terminate_is_a_noop() {
    let invoked = Arc::new(AtomicUsize::new(0));
    let (hook, release) = gated_hook(true, invoked);
    let session = Session::builder("reset-pump-done")
        .robot(robot())
        .post_reset(ResetSpec::Hook(hook))
        .build()
        .unwrap();

    let mut ep = session.start_episode("cleanup").unwrap();
    let _ = ep.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    session.inject(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "done".to_owned(),
        at: waddle_types::MonoNs(0),
    });
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::PostReset))
    });

    // The outcome is pinned at POST_RESET entry: the rollout is over from
    // the caller's view.
    assert!(ep.done(), "episode_done must flip at Phase::PostReset");
    let status = session.status();
    assert_eq!(status.pinned_outcome, Some(TerminalOutcome::Success));
    assert_eq!(
        ep.outcome(),
        Some(TerminalOutcome::Success),
        "the pinned outcome is the episode's outcome once done"
    );

    // terminate during POST_RESET is a no-op (the `_Rollout.__exit__` race
    // fix): it must return immediately — not block, not inject a second
    // Terminate that could disturb the pinned outcome.
    ep.terminate(TerminalOutcome::Failure, "late abort");
    assert!(
        matches!(session.status().episode_state, Some(Phase::PostReset)),
        "a late terminate must not move the episode out of POST_RESET"
    );

    // Let the post-reset resolve; the pinned outcome stands.
    drop(release);
    session.inject(SessionEvent::PostResetResult {
        ok: true,
        detail: String::new(),
        at: waddle_types::MonoNs(0),
    });
    wait_for(&session, |s| {
        matches!(
            s.episode_state,
            Some(Phase::Terminal(TerminalOutcome::Success))
        )
    });
    assert_eq!(ep.outcome(), Some(TerminalOutcome::Success));
    session.shutdown();
}

// --- SetPostResetFailed → sidecar ----------------------------------------

#[test]
fn post_reset_failure_sets_sidecar_flag_and_never_alters_outcome() {
    let dir = tempfile::tempdir().unwrap();
    let session = Session::builder("reset-pump-failed")
        .robot(robot())
        .recording_dir(dir.path())
        .post_reset(ResetSpec::Hook(Arc::new(|_| (false, None))))
        .build()
        .unwrap();

    let mut ep = session.start_episode("cleanup-fails").unwrap();
    let id = ep.id().clone();
    let _ = ep.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    // Nonzero stamps: the sidecar's post_reset_bounds are copied from these
    // events' t_ns (open at →POST_RESET, close at →TERMINAL).
    session.inject(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "done".to_owned(),
        at: waddle_types::MonoNs(1_000_000),
    });
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::PostReset))
    });
    session.inject(SessionEvent::PostResetResult {
        ok: false,
        detail: "bin jammed".to_owned(),
        at: waddle_types::MonoNs(2_000_000),
    });

    // E16: the failure flags permanently but the pinned outcome stands.
    wait_for(&session, |s| {
        matches!(
            s.episode_state,
            Some(Phase::Terminal(TerminalOutcome::Success))
        )
    });
    assert!(
        session.status().post_reset_failed,
        "the mirror carries the permanent post_reset_failed flag"
    );
    session.shutdown();

    let sidecar_path = dir.path().join(format!("{id}.sidecar.json"));
    let sidecar =
        waddle_sidecar::sidecar_from_json(&std::fs::read_to_string(&sidecar_path).unwrap())
            .unwrap();
    assert!(sidecar.post_reset_declared);
    assert!(sidecar.post_reset_failed);
    assert_eq!(
        sidecar.outcome,
        pb::TerminalOutcome::Success as i32,
        "post_reset_failed NEVER alters the pinned outcome"
    );
    assert!(!sidecar.post_reset_result.as_ref().unwrap().ok);
    let bounds = sidecar.post_reset_bounds.as_ref().unwrap();
    assert!(bounds.t_start_ns > 0);
    assert!(bounds.t_end_ns >= bounds.t_start_ns);
}
