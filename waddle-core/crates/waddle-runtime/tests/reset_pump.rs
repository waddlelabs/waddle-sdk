//! Reset pump, reducer effects, mirror semantics:
//! `episode_done` flips at `Phase::PostReset` (the outcome is pinned, so the
//! rollout is over from the caller's view — and `terminate_episode` becomes
//! a no-op there, fixing the `_Rollout.__exit__` race); the reducer routes
//! `Effect::SetPostResetFailed` onto the sidecar and stamps
//! `post_reset_declared` from `EpisodeOpen`; and `spawn_reset_pump` is the
//! single scripted-hook invocation site (mirror-watch), fixing the
//! reducer-opened retake-successor gap (successors never received a
//! `ResetResult` before it).
//!
//! Also: a reducer-opened retake successor inherits the SESSION's
//! declared `post_reset` config (a per-episode override does NOT carry
//! across a retake — see the successor tests below).

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, Sender};
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use prost::Message as _;
use waddle_fsm::{Phase, SessionEvent};
use waddle_gate::gate::GateOutput;
use waddle_runtime::{
    ControlRegistry, EpisodeOptions, ResetHook, ResetSpec, Session, VerbError, grant_and_engage,
};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActorKind, ClaimId, EpisodeId, GateMode, TerminalOutcome};

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

/// A registry with `hold`/`send` so `grant_and_engage` has a live engage
/// path (retake requires reaching SETTLE).
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
    // Nonzero stamp: the sidecar's post_reset_bounds open at this event's
    // t_ns (→POST_RESET). The reset pump runs the declared failing hook and
    // injects `PostResetResult { ok: false }` itself; POST_RESET may resolve
    // faster than a poll can observe it, so wait straight for Terminal.
    session.inject(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "done".to_owned(),
        at: waddle_types::MonoNs(1_000_000),
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

// --- The reset pump -------------------------------------------------------

/// THE headline regression (the reset-window design): a reducer-opened retake successor
/// never received a `ResetResult` — only `start_episode`'s inline path ran
/// the pre-reset pipeline, so the successor hung in RESETTING forever. The
/// reset pump (mirror-watch) services it now, running the effective PRE
/// hook (session config for reducer-opened episodes) and injecting the
/// result.
#[test]
fn retake_successor_passes_through_reset_via_the_pump() {
    let pre_runs = Arc::new(AtomicUsize::new(0));
    let pre_runs2 = pre_runs.clone();
    let session = Session::builder("reset-pump-retake")
        .robot(robot())
        .control(registry())
        .pre_reset(ResetSpec::Hook(Arc::new(move |_task: &str| {
            pre_runs2.fetch_add(1, Ordering::SeqCst);
            (true, Some(true))
        })))
        .build()
        .unwrap();

    let mut ep1 = session.start_episode("first").unwrap();
    let first_id = ep1.id().clone();
    assert_eq!(pre_runs.load(Ordering::SeqCst), 1, "inline pre-reset ran");
    let _ = ep1.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    // Reach SETTLE (retake is legal from there) and retake: the claim
    // survives, the reducer opens the successor born-claimed in RESETTING.
    grant_and_engage(&session, "claim-rt", "teleop", ActorKind::Teleoperator);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));
    let successor = EpisodeId::new("ep-retake-successor");
    session.inject(SessionEvent::Retake {
        claim: ClaimId::new("claim-rt"),
        initiator: ActorKind::Teleoperator,
        successor: successor.clone(),
        at: waddle_types::MonoNs(3_000_000),
    });

    // Before the pump existed this waited forever: nothing ran the pre
    // hook for the successor, so it never left RESETTING.
    wait_for(&session, |s| {
        s.episode_id.as_ref() == Some(&successor) && matches!(s.episode_state, Some(Phase::Ready))
    });
    assert_eq!(
        pre_runs.load(Ordering::SeqCst),
        2,
        "the pump ran the effective PRE hook for the successor"
    );
    assert!(session.episode_done(&first_id));
    session.shutdown();
}

/// The pump runs the effective POST hook exactly once per episode (the
/// bookkeeping resets across episodes), its `PostResetResult` drives the
/// episode to Terminal with the pinned outcome, and the sidecar carries
/// the result + closed bounds. `terminate` (the blocking helper) now works
/// on post-reset-declared episodes — the pump auto-completes POST_RESET.
#[test]
fn post_reset_hook_runs_exactly_once_per_episode() {
    let dir = tempfile::tempdir().unwrap();
    let post_runs = Arc::new(AtomicUsize::new(0));
    let post_runs2 = post_runs.clone();
    let session = Session::builder("reset-pump-post-once")
        .robot(robot())
        .recording_dir(dir.path())
        .post_reset(ResetSpec::Hook(Arc::new(move |_task: &str| {
            post_runs2.fetch_add(1, Ordering::SeqCst);
            (true, Some(true))
        })))
        .build()
        .unwrap();

    let terminate_bounded = |session: &Session, id: &EpisodeId| {
        let (tx, rx) = std::sync::mpsc::channel();
        let session = session.clone();
        let id = id.clone();
        std::thread::spawn(move || {
            session.terminate_episode(&id, TerminalOutcome::Success, "done");
            let _ = tx.send(());
        });
        rx.recv_timeout(Duration::from_secs(5))
            .expect("terminate_episode must unblock once the pump completes POST_RESET");
    };

    let mut ep1 = session.start_episode("cleanup-1").unwrap();
    let id1 = ep1.id().clone();
    let _ = ep1.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    terminate_bounded(&session, &id1);
    wait_for(&session, |s| {
        matches!(
            s.episode_state,
            Some(Phase::Terminal(TerminalOutcome::Success))
        )
    });
    assert_eq!(post_runs.load(Ordering::SeqCst), 1, "post hook ran once");

    // A second episode gets its own single run (per-episode bookkeeping).
    let mut ep2 = session.start_episode("cleanup-2").unwrap();
    let id2 = ep2.id().clone();
    let _ = ep2.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    terminate_bounded(&session, &id2);
    wait_for(&session, |s| {
        s.episode_id.as_ref() == Some(&id2)
            && matches!(
                s.episode_state,
                Some(Phase::Terminal(TerminalOutcome::Success))
            )
    });
    assert_eq!(
        post_runs.load(Ordering::SeqCst),
        2,
        "each episode's post hook runs exactly once"
    );
    session.shutdown();

    // The first episode's sidecar carries the whole post-reset record.
    let sidecar_path = dir.path().join(format!("{id1}.sidecar.json"));
    let sidecar =
        waddle_sidecar::sidecar_from_json(&std::fs::read_to_string(&sidecar_path).unwrap())
            .unwrap();
    assert!(sidecar.post_reset_declared);
    assert!(!sidecar.post_reset_failed);
    assert_eq!(sidecar.outcome, pb::TerminalOutcome::Success as i32);
    assert!(sidecar.post_reset_result.as_ref().unwrap().ok);
    let bounds = sidecar.post_reset_bounds.as_ref().unwrap();
    assert!(bounds.t_start_ns > 0);
    assert!(bounds.t_end_ns >= bounds.t_start_ns);
}

/// Remote specs are none of the pump's business: a `ResetSpec::Remote`
/// post-reset is driven by the FSM's window machinery (E19–E22), and the
/// caller's own stale `gate()` handle during the engaged window records
/// RESET_ACTIVE NoopMarkers on /waddle/actions, like BYPASS/HOLD do.
#[test]
fn remote_post_reset_window_records_reset_active_noop_markers() {
    let dir = tempfile::tempdir().unwrap();
    let session = Session::builder("reset-pump-remote-post")
        .robot(robot())
        .recording_dir(dir.path())
        .post_reset(ResetSpec::Remote {
            actor: ActorKind::Teleoperator,
            prompt: "clear the table".into(),
            timeout_ns: 600_000_000_000,
        })
        .build()
        .unwrap();

    let mut ep = session.start_episode("remote-post").unwrap();
    let id = ep.id().clone();
    let _ = ep.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    session.inject(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "done".to_owned(),
        at: waddle_types::MonoNs(1_000_000),
    });
    // E14 opens the declared POST window; the pump must NOT inject anything
    // for a Remote spec — the phase stays POST_RESET until the window
    // resolves.
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::PostReset))
    });
    std::thread::sleep(Duration::from_millis(50));
    assert!(
        matches!(session.status().episode_state, Some(Phase::PostReset)),
        "the pump must leave remote resets to the window machinery"
    );

    // C6 → E20: the expected actor claims and engages; the gate flips to
    // RESET (the loop's own handle is stale).
    session.inject(SessionEvent::ClaimGranted {
        id: ClaimId::new("reset-claim"),
        source: "teleop".to_owned(),
        actor: ActorKind::Teleoperator,
        self_initiated: false,
        at: waddle_types::MonoNs(2_000_000),
    });
    wait_for(&session, |s| s.claim_active);
    session.inject(SessionEvent::ResetWindowEngage {
        claim: ClaimId::new("reset-claim"),
        at: waddle_types::MonoNs(3_000_000),
    });
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Reset));

    for _ in 0..3 {
        assert!(
            matches!(ep.gate(&[0.0; 3], None, None), GateOutput::Noop { .. }),
            "a Reset-mode gate tick is a noop for the stale handle"
        );
        std::thread::sleep(Duration::from_millis(5));
    }

    // E21: completion hands the lease back and applies the result as the
    // post-reset pipeline (Terminal, pinned outcome).
    session.inject(SessionEvent::ResetWindowComplete {
        claim: ClaimId::new("reset-claim"),
        ok: true,
        verified: Some(true),
        at: waddle_types::MonoNs(4_000_000),
    });
    wait_for(&session, |s| {
        matches!(
            s.episode_state,
            Some(Phase::Terminal(TerminalOutcome::Success))
        )
    });
    session.shutdown();

    // MCAP: the Reset-mode ticks landed as RESET_ACTIVE NoopMarkers.
    let buf = std::fs::read(dir.path().join(format!("{id}.mcap"))).unwrap();
    let mut reset_noops = 0;
    for message in mcap::MessageStream::new(&buf).unwrap() {
        let message = message.unwrap();
        if message.channel.topic != waddle_sidecar::mcaprec::ACTIONS_TOPIC {
            continue;
        }
        let chunk = pb::ActionChunk::decode(message.data.as_ref()).unwrap();
        for action in &chunk.actions {
            if let Some(pb::action::Target::Noop(marker)) = &action.target
                && marker.reason == pb::NoopReason::ResetActive as i32
            {
                reset_noops += 1;
            }
        }
    }
    assert!(
        reset_noops >= 3,
        "expected the Reset-mode ticks as RESET_ACTIVE noops, got {reset_noops}"
    );

    // Sidecar: the window completion applied as the post-reset pipeline.
    let sidecar_path = dir.path().join(format!("{id}.sidecar.json"));
    let sidecar =
        waddle_sidecar::sidecar_from_json(&std::fs::read_to_string(&sidecar_path).unwrap())
            .unwrap();
    assert!(sidecar.post_reset_declared);
    assert!(sidecar.post_reset_result.as_ref().unwrap().ok);
    let bounds = sidecar.post_reset_bounds.as_ref().unwrap();
    assert_eq!(bounds.t_start_ns, 1_000_000);
    assert!(bounds.t_end_ns >= bounds.t_start_ns);
}

// --- Retake successors inherit the session's post_reset config -----------

/// Regression: before the fix, `Effect::OpenSuccessor` hardcoded
/// `post_reset: false`, so a retaken episode's own termination skipped
/// straight to `Terminal` with no cleanup at all, silently, even though the
/// session declared a `post_reset` hook. The successor must detour through
/// `Phase::PostReset` and run the SESSION's declared hook exactly like any
/// other episode would.
#[test]
fn retake_successor_inherits_session_post_reset_hook() {
    let invoked = Arc::new(AtomicUsize::new(0));
    let (hook, release) = gated_hook(true, invoked.clone());
    let session = Session::builder("reset-pump-retake-post")
        .robot(robot())
        .control(registry())
        .post_reset(ResetSpec::Hook(hook))
        .build()
        .unwrap();

    let mut ep1 = session.start_episode("first").unwrap();
    let _ = ep1.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    // Retake: the claim survives, the reducer opens the successor
    // born-claimed in RESETTING. No per-episode post_reset override is in
    // play anywhere here — the successor's only source of post-reset config
    // is the session-level default declared above.
    grant_and_engage(&session, "claim-rt", "teleop", ActorKind::Teleoperator);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));
    let successor = EpisodeId::new("ep-retake-post-successor");
    session.inject(SessionEvent::Retake {
        claim: ClaimId::new("claim-rt"),
        initiator: ActorKind::Teleoperator,
        successor: successor.clone(),
        at: waddle_types::MonoNs(3_000_000),
    });
    wait_for(&session, |s| {
        s.episode_id.as_ref() == Some(&successor) && matches!(s.episode_state, Some(Phase::Ready))
    });
    // The POST_RESET detour only applies once the episode has actually run
    // (Running/Intervention) — drive READY → RUNNING directly (E6), since
    // there is no `Episode` handle for a reducer-opened successor to `gate()`
    // with.
    session.inject(SessionEvent::Start {
        at: waddle_types::MonoNs(3_500_000),
    });
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    // The successor's own termination must detour through POST_RESET and run
    // the session's declared hook — before the fix this went straight to
    // Terminal and `invoked` never incremented.
    session.inject(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "done".to_owned(),
        at: waddle_types::MonoNs(4_000_000),
    });
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::PostReset))
    });
    assert_eq!(
        invoked.load(Ordering::SeqCst),
        1,
        "the successor's inherited session post-reset hook ran"
    );

    drop(release);
    wait_for(&session, |s| {
        matches!(
            s.episode_state,
            Some(Phase::Terminal(TerminalOutcome::Success))
        )
    });
    session.shutdown();
}

/// A retake successor with a SESSION-level `Remote` post-reset spec opens
/// the window on the successor's own E14, exactly as any other episode
/// would: the born-claimed suppression (born-claimed suppression) is a PRE-window-only
/// guard (checked only at `EpisodeOpen`'s `pre_window` arm) and does not
/// apply to the POST window opened later at `enter_post_reset`.
#[test]
fn retake_successor_inherits_session_post_reset_remote_window() {
    let session = Session::builder("reset-pump-retake-remote-post")
        .robot(robot())
        .control(registry())
        .post_reset(ResetSpec::Remote {
            actor: ActorKind::Teleoperator,
            prompt: "clear the table".into(),
            timeout_ns: 600_000_000_000,
        })
        .build()
        .unwrap();

    let mut ep1 = session.start_episode("first").unwrap();
    let _ = ep1.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(&session, "claim-rt", "teleop", ActorKind::Teleoperator);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));
    let successor = EpisodeId::new("ep-retake-remote-post-successor");
    session.inject(SessionEvent::Retake {
        claim: ClaimId::new("claim-rt"),
        initiator: ActorKind::Teleoperator,
        successor: successor.clone(),
        at: waddle_types::MonoNs(3_000_000),
    });
    wait_for(&session, |s| {
        s.episode_id.as_ref() == Some(&successor) && matches!(s.episode_state, Some(Phase::Ready))
    });
    // The POST_RESET detour only applies once the episode has actually run
    // (Running/Intervention) — drive READY → RUNNING directly (E6), since
    // there is no `Episode` handle for a reducer-opened successor to `gate()`
    // with.
    session.inject(SessionEvent::Start {
        at: waddle_types::MonoNs(3_500_000),
    });
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    session.inject(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "done".to_owned(),
        at: waddle_types::MonoNs(4_000_000),
    });
    // Before the fix, `post_window` was hardcoded to `None` for successors,
    // so `post_reset_declared` never became true and this never entered
    // POST_RESET at all.
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::PostReset))
    });
    std::thread::sleep(Duration::from_millis(50));
    assert!(
        matches!(session.status().episode_state, Some(Phase::PostReset)),
        "the pump must leave a remote post-reset to the window machinery"
    );

    // The retake's surviving claim was released on entry to POST_RESET
    // (`close_run(.., release_claim=true)`, same as any other episode's
    // close), so a fresh claim can now engage the window (C6).
    session.inject(SessionEvent::ClaimGranted {
        id: ClaimId::new("reset-claim"),
        source: "teleop".to_owned(),
        actor: ActorKind::Teleoperator,
        self_initiated: false,
        at: waddle_types::MonoNs(5_000_000),
    });
    wait_for(&session, |s| s.claim_active);
    session.inject(SessionEvent::ResetWindowEngage {
        claim: ClaimId::new("reset-claim"),
        at: waddle_types::MonoNs(6_000_000),
    });
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Reset));
    session.inject(SessionEvent::ResetWindowComplete {
        claim: ClaimId::new("reset-claim"),
        ok: true,
        verified: Some(true),
        at: waddle_types::MonoNs(7_000_000),
    });
    wait_for(&session, |s| {
        s.episode_id.as_ref() == Some(&successor)
            && matches!(
                s.episode_state,
                Some(Phase::Terminal(TerminalOutcome::Success))
            )
    });
    session.shutdown();
}

/// A predecessor's per-episode `post_reset` override belongs to the
/// predecessor only: it must never leak to a retake successor, which sees
/// only the session-level default (here: none at all, so the successor's
/// own termination skips POST_RESET entirely).
#[test]
fn retake_successor_does_not_inherit_predecessor_per_episode_post_reset_override() {
    let invoked = Arc::new(AtomicUsize::new(0));
    let invoked2 = invoked.clone();
    let session = Session::builder("reset-pump-retake-no-leak")
        .robot(robot())
        .control(registry())
        .build()
        .unwrap();

    let mut ep1 = session
        .start_episode_with(
            "first",
            EpisodeOptions {
                post_reset: Some(Some(ResetSpec::Hook(Arc::new(move |_task: &str| {
                    invoked2.fetch_add(1, Ordering::SeqCst);
                    (true, Some(true))
                })))),
                ..Default::default()
            },
        )
        .unwrap();
    let _ = ep1.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });

    grant_and_engage(&session, "claim-rt", "teleop", ActorKind::Teleoperator);
    wait_for(&session, |s| s.gate_mode == Some(GateMode::Intervention));
    let successor = EpisodeId::new("ep-retake-no-leak-successor");
    session.inject(SessionEvent::Retake {
        claim: ClaimId::new("claim-rt"),
        initiator: ActorKind::Teleoperator,
        successor: successor.clone(),
        at: waddle_types::MonoNs(3_000_000),
    });
    wait_for(&session, |s| {
        s.episode_id.as_ref() == Some(&successor) && matches!(s.episode_state, Some(Phase::Ready))
    });

    session.inject(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "done".to_owned(),
        at: waddle_types::MonoNs(4_000_000),
    });
    wait_for(&session, |s| {
        s.episode_id.as_ref() == Some(&successor)
            && matches!(
                s.episode_state,
                Some(Phase::Terminal(TerminalOutcome::Success))
            )
    });
    assert_eq!(
        invoked.load(Ordering::SeqCst),
        0,
        "the predecessor's per-episode post_reset override must not leak to the successor"
    );
    session.shutdown();
}
