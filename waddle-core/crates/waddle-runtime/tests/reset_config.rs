//! Runtime reset config surface: `ResetSpec`, the `SessionBuilder`
//! setters (`pre_reset`/`post_reset`/`verification_mode`, and the deprecated
//! `reset_hook` alias), `EpisodeOptions` per-episode overrides, and
//! `start_episode`/`start_episode_with`'s pre-reset routing + the
//! predecessor-in-`Phase::PostReset` wait (design §D4, first two bullets).
//!
//! The reset pump (`waddle-reset-hooks`) is the single post-reset hook
//! invocation site: a declared POST hook runs there the moment the mirror
//! shows `Phase::PostReset`. Tests that need to *observe* POST_RESET hold it
//! open with a gated hook (one that blocks until the test releases it) —
//! otherwise the pump resolves the phase faster than a poll can see it.

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use waddle_controlplane::{ClientMsg, InMemoryTransport};
use waddle_fsm::{Phase, SessionEvent};
use waddle_runtime::{EpisodeOptions, ResetHook, ResetSpec, RuntimeError, Session};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActorKind, ClaimId, ResetVerificationMode, TerminalOutcome};

fn robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "reset-config-bot".into(),
        robot_id: "reset-config-01".into(),
        cell_id: "cell-reset-config".into(),
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

/// Run `start_episode_with` on a background thread and wait up to `timeout`
/// for it to return — never hangs the test suite if a config-resolution bug
/// leaves the call blocked forever (e.g. a `verification_mode` or `ResetSpec`
/// that silently didn't take effect).
fn start_bounded(
    session: &Session,
    task: &'static str,
    opts: EpisodeOptions,
    timeout: Duration,
) -> Result<waddle_runtime::Episode, RuntimeError> {
    let (tx, rx) = std::sync::mpsc::channel();
    let session = session.clone();
    std::thread::spawn(move || {
        let _ = tx.send(session.start_episode_with(task, opts));
    });
    rx.recv_timeout(timeout)
        .unwrap_or_else(|_| panic!("start_episode_with did not return within {timeout:?}"))
}

fn counting_hook(count: Arc<AtomicUsize>) -> ResetHook {
    Arc::new(move |_task: &str| {
        count.fetch_add(1, Ordering::SeqCst);
        (true, Some(true))
    })
}

// --- Builder ----------------------------------------------------------

#[test]
fn verification_mode_setter_takes_effect() {
    // Blocking (the default) never reaches READY on `verified: None` — only
    // an inline verification (or a later async `VerificationResult`, not
    // exercised here) can. OptimisticAsync must reach READY immediately
    // regardless, proving the setter's value actually reached the FSM
    // (rather than silently defaulting to Blocking).
    let count = Arc::new(AtomicUsize::new(0));
    let session = Session::builder("reset-config-verification")
        .robot(robot())
        .verification_mode(ResetVerificationMode::OptimisticAsync)
        .pre_reset(ResetSpec::Hook(Arc::new({
            let count = count.clone();
            move |_task: &str| {
                count.fetch_add(1, Ordering::SeqCst);
                (true, None) // ok, but not inline-verified
            }
        })))
        .build()
        .unwrap();

    let ep = start_bounded(
        &session,
        "verify",
        EpisodeOptions::default(),
        Duration::from_secs(5),
    )
    .expect("OptimisticAsync must reach READY without an inline verification");
    assert_eq!(count.load(Ordering::SeqCst), 1);
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
}

#[test]
fn pre_reset_hook_is_invoked_by_start_episode() {
    let count = Arc::new(AtomicUsize::new(0));
    let session = Session::builder("reset-config-pre-hook")
        .robot(robot())
        .pre_reset(ResetSpec::Hook(counting_hook(count.clone())))
        .build()
        .unwrap();

    let ep = session.start_episode("hook").unwrap();
    assert_eq!(count.load(Ordering::SeqCst), 1, "the configured hook ran");
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
}

#[test]
fn post_reset_hook_declared_detours_through_post_reset_phase() {
    // A gated hook holds POST_RESET open so the detour is observable: the
    // reset pump invokes it as soon as the mirror shows the phase, and
    // would otherwise resolve it faster than this test's polling.
    let (release_tx, release_rx) = std::sync::mpsc::channel::<()>();
    let release_rx = Arc::new(Mutex::new(release_rx));
    let session = Session::builder("reset-config-post-hook")
        .robot(robot())
        .post_reset(ResetSpec::Hook(Arc::new(move |_| {
            let _ = release_rx.lock().recv_timeout(Duration::from_secs(5));
            (true, Some(true))
        })))
        .build()
        .unwrap();

    let mut ep = session.start_episode("post-hook").unwrap();
    // E14 only detours RUNNING/INTERVENTION terminates through POST_RESET;
    // a tick moves READY -> RUNNING first (as a real rollout would).
    let _ = ep.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    // Inject Terminate directly rather than calling the blocking `terminate`
    // helper: that helper blocks until Terminal, and this test wants to
    // observe the POST_RESET detour while the gated hook holds it open.
    session.inject(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "done".to_owned(),
        at: waddle_types::MonoNs(0),
    });
    // Declaring post_reset (a hook) makes terminate detour through
    // Phase::PostReset instead of going straight to Terminal (E14).
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::PostReset))
    });
    // Release the hook: the pump injects the PostResetResult itself.
    drop(release_tx);
    wait_for(&session, |s| {
        matches!(
            s.episode_state,
            Some(Phase::Terminal(TerminalOutcome::Success))
        )
    });
    assert!(ep.done());
    session.shutdown();
}

#[test]
#[allow(deprecated)]
fn reset_hook_alias_behaves_like_pre_reset_hook() {
    let count = Arc::new(AtomicUsize::new(0));
    let session = Session::builder("reset-config-alias")
        .robot(robot())
        .reset_hook(counting_hook(count.clone()))
        .build()
        .unwrap();

    let ep = session.start_episode("alias").unwrap();
    assert_eq!(
        count.load(Ordering::SeqCst),
        1,
        "the deprecated alias must still wire the hook inline"
    );
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
}

#[test]
fn feature_flags_declare_reset_phases_and_remote_from_session_config() {
    let captured: Arc<Mutex<Vec<pb::RegisterRequest>>> = Arc::new(Mutex::new(Vec::new()));
    let captured2 = captured.clone();
    let transport = InMemoryTransport::new(move |msg, _tx| {
        if let ClientMsg::Register(req) = msg {
            captured2.lock().push(req);
        }
    });

    let session = Session::builder("reset-config-flags")
        .robot(robot())
        .transport(transport)
        .pre_reset(ResetSpec::Remote {
            actor: ActorKind::Teleoperator,
            prompt: "clear the table".into(),
            timeout_ns: 1_000_000_000,
        })
        .post_reset(ResetSpec::Hook(Arc::new(|_| (true, Some(true)))))
        .build()
        .unwrap();

    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if !captured.lock().is_empty() {
            break;
        }
        assert!(Instant::now() < deadline, "register message never arrived");
        std::thread::sleep(Duration::from_millis(5));
    }
    let req = captured.lock()[0].clone();
    for flag in [
        "waddle.v0.core",
        "waddle.v0.reset",
        "waddle.v0.reset.phases",
        "waddle.v0.reset.remote",
    ] {
        assert!(
            req.feature_flags.iter().any(|f| f == flag),
            "expected {flag} in {:?}",
            req.feature_flags
        );
    }
    session.shutdown();
}

#[test]
fn feature_flags_omit_phases_and_remote_when_unconfigured() {
    let captured: Arc<Mutex<Vec<pb::RegisterRequest>>> = Arc::new(Mutex::new(Vec::new()));
    let captured2 = captured.clone();
    let transport = InMemoryTransport::new(move |msg, _tx| {
        if let ClientMsg::Register(req) = msg {
            captured2.lock().push(req);
        }
    });

    let session = Session::builder("reset-config-flags-off")
        .robot(robot())
        .transport(transport)
        .build()
        .unwrap();

    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if !captured.lock().is_empty() {
            break;
        }
        assert!(Instant::now() < deadline, "register message never arrived");
        std::thread::sleep(Duration::from_millis(5));
    }
    let req = captured.lock()[0].clone();
    assert!(req.feature_flags.iter().any(|f| f == "waddle.v0.core"));
    assert!(req.feature_flags.iter().any(|f| f == "waddle.v0.reset"));
    assert!(
        !req.feature_flags
            .iter()
            .any(|f| f == "waddle.v0.reset.phases")
    );
    assert!(
        !req.feature_flags
            .iter()
            .any(|f| f == "waddle.v0.reset.remote")
    );
    session.shutdown();
}

// --- EpisodeOptions -----------------------------------------------------

#[test]
fn episode_options_pre_reset_inner_none_disables_session_hook_for_this_episode() {
    let count = Arc::new(AtomicUsize::new(0));
    let session = Session::builder("reset-config-disable-pre")
        .robot(robot())
        .pre_reset(ResetSpec::Hook(counting_hook(count.clone())))
        .build()
        .unwrap();

    let ep = session
        .start_episode_with(
            "disabled",
            EpisodeOptions {
                pre_reset: Some(None),
                post_reset: None,
            },
        )
        .expect("disabled pre-reset still resolves to the trivial default");
    assert_eq!(
        count.load(Ordering::SeqCst),
        0,
        "the session's configured hook must not run when disabled per-episode"
    );
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
}

#[test]
fn episode_options_pre_reset_outer_none_inherits_session_hook() {
    let count = Arc::new(AtomicUsize::new(0));
    let session = Session::builder("reset-config-inherit-pre")
        .robot(robot())
        .pre_reset(ResetSpec::Hook(counting_hook(count.clone())))
        .build()
        .unwrap();

    let ep = session
        .start_episode_with("inherited", EpisodeOptions::default())
        .expect("inherited pre-reset must run the session hook");
    assert_eq!(count.load(Ordering::SeqCst), 1);
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
}

#[test]
fn episode_options_post_reset_inner_none_disables_session_post_reset_for_this_episode() {
    let session = Session::builder("reset-config-disable-post")
        .robot(robot())
        .post_reset(ResetSpec::Hook(Arc::new(|_| (true, Some(true)))))
        .build()
        .unwrap();

    let ep = session
        .start_episode_with(
            "disabled-post",
            EpisodeOptions {
                pre_reset: None,
                post_reset: Some(None),
            },
        )
        .unwrap();
    ep.terminate(TerminalOutcome::Success, "done");
    // Disabled for this episode: terminate must go straight to Terminal,
    // never detouring through PostReset, even though the session default
    // declares one.
    wait_for(&session, |s| {
        matches!(
            s.episode_state,
            Some(Phase::Terminal(TerminalOutcome::Success))
        )
    });
    session.shutdown();
}

// --- Remote pre-reset window --------------------------------------------

fn remote_pre_reset_session(project: &str, timeout_ns: i64) -> Session {
    Session::builder(project)
        .robot(robot())
        .pre_reset(ResetSpec::Remote {
            actor: ActorKind::Teleoperator,
            prompt: "clear the table".into(),
            timeout_ns,
        })
        .build()
        .unwrap()
}

#[test]
fn remote_pre_reset_blocks_then_window_complete_releases_it_to_ready() {
    let session = remote_pre_reset_session("reset-config-remote-ok", 600_000_000_000);

    let handle = {
        let session = session.clone();
        std::thread::spawn(move || session.start_episode_with("remote", EpisodeOptions::default()))
    };

    // The pre-reset window opened at EpisodeOpen and start_episode must
    // still be blocked in RESETTING (no ResetResult was ever injected for a
    // Remote spec).
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Resetting))
    });
    std::thread::sleep(Duration::from_millis(100));
    assert!(
        !handle.is_finished(),
        "must block until the window resolves"
    );

    session.inject(SessionEvent::ClaimGranted {
        id: ClaimId::new("reset-claim"),
        source: "teleop".to_owned(),
        actor: ActorKind::Teleoperator,
        self_initiated: false,
        at: waddle_types::MonoNs(1),
    });
    wait_for(&session, |s| s.claim_active);
    session.inject(SessionEvent::ResetWindowEngage {
        claim: ClaimId::new("reset-claim"),
        at: waddle_types::MonoNs(2),
    });
    // Wait for the engage's deferred lease handoff to actually complete
    // (gate -> RESET) before sending Complete: both are queued through the
    // same reducer channel as the engage's own follow-up `LeaseTokenMinted`,
    // and firing Complete before that follow-up is processed would mint a
    // second lease token out from under the first, clobbering the pending
    // lease op the engage is still waiting on.
    wait_for(&session, |s| {
        s.gate_mode == Some(waddle_types::GateMode::Reset)
    });
    session.inject(SessionEvent::ResetWindowComplete {
        claim: ClaimId::new("reset-claim"),
        ok: true,
        verified: Some(true),
        at: waddle_types::MonoNs(3),
    });

    let ep = handle
        .join()
        .unwrap()
        .expect("window completion must release start_episode to READY");
    assert!(matches!(session.status().episode_state, Some(Phase::Ready)));
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
}

#[test]
fn remote_pre_reset_window_timeout_yields_reset_failed() {
    let session = remote_pre_reset_session("reset-config-remote-timeout", 600_000_000_000);

    let handle = {
        let session = session.clone();
        std::thread::spawn(move || {
            session.start_episode_with("remote-timeout", EpisodeOptions::default())
        })
    };
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Resetting))
    });

    // Force the window's deadline without waiting out a real 600s timeout:
    // inject the timer event directly (E22), exactly as the FSM's own reducer
    // would once the real deadline elapsed. start_episode_with adds no
    // runtime-side timeout of its own — the FSM window timer owns it.
    session.inject(SessionEvent::TimerFired {
        id: waddle_fsm::TimerId::ResetWindowTimeout,
        at: waddle_types::MonoNs(4),
    });

    let result = handle
        .join()
        .unwrap()
        .expect_err("a timed-out pre-reset window must fail start_episode");
    assert!(
        matches!(result, RuntimeError::ResetFailed(_)),
        "expected ResetFailed, got {result:?}"
    );
    session.shutdown();
}

// --- Predecessor in POST_RESET -------------------------------------------

#[test]
fn predecessor_in_post_reset_is_waited_out_before_next_open() {
    // A gated hook: the reset pump invokes it (flipping `cleanup_started`)
    // as soon as A enters POST_RESET, then blocks until the test releases
    // it — holding A's cleanup outstanding while B tries to open.
    let cleanup_started = Arc::new(AtomicBool::new(false));
    let (release_tx, release_rx) = std::sync::mpsc::channel::<()>();
    let release_rx = Arc::new(Mutex::new(release_rx));
    let session = Session::builder("reset-config-post-reset-wait")
        .robot(robot())
        .post_reset(ResetSpec::Hook(Arc::new({
            let cleanup_started = cleanup_started.clone();
            move |_task: &str| {
                cleanup_started.store(true, Ordering::SeqCst);
                let _ = release_rx.lock().recv_timeout(Duration::from_secs(5));
                (true, Some(true))
            }
        })))
        .build()
        .unwrap();

    let mut ep_a = session.start_episode("first").unwrap();
    // E14 only detours RUNNING/INTERVENTION terminates through POST_RESET.
    let _ = ep_a.gate(&[0.0; 3], None, None);
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::Running))
    });
    // Non-blocking inject, not `ep_a.terminate(..)`: that helper blocks
    // until Terminal, and A's Terminal is deliberately held open by the
    // gated hook for the duration of this test.
    session.inject(SessionEvent::Terminate {
        outcome: TerminalOutcome::Success,
        reason: "done".to_owned(),
        at: waddle_types::MonoNs(0),
    });
    wait_for(&session, |s| {
        matches!(s.episode_state, Some(Phase::PostReset))
    });
    // The reset pump is the post-reset hook's single invocation site: it
    // picks A's cleanup up off the mirror.
    let deadline = Instant::now() + Duration::from_secs(5);
    while !cleanup_started.load(Ordering::SeqCst) {
        assert!(Instant::now() < deadline, "the pump never invoked the hook");
        std::thread::sleep(Duration::from_millis(5));
    }

    // Start B on a background thread; it must block while A's POST_RESET
    // cleanup is outstanding, not error EpisodeActive. B disables post-reset
    // for itself (this test is about waiting out A, not B's own cleanup) so
    // the final `ep_b.terminate` below can use the ordinary blocking helper.
    let handle = {
        let session = session.clone();
        std::thread::spawn(move || {
            session.start_episode_with(
                "second",
                EpisodeOptions {
                    pre_reset: None,
                    post_reset: Some(None),
                },
            )
        })
    };
    std::thread::sleep(Duration::from_millis(150));
    assert!(
        !handle.is_finished(),
        "must wait for the predecessor's PostReset to resolve, not error or race ahead"
    );

    // Release the gated hook: the pump injects A's PostResetResult, A
    // reaches Terminal, and B opens over it.
    drop(release_tx);

    let ep_b = handle
        .join()
        .unwrap()
        .expect("B must open once A's PostReset resolves to Terminal");
    assert!(matches!(session.status().episode_state, Some(Phase::Ready)));
    assert_ne!(ep_b.id(), ep_a.id());
    ep_b.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
}

/// The existing guard (unchanged behavior): a predecessor in any other
/// non-terminal, non-PostReset phase still errors EpisodeActive rather than
/// waiting — only PostReset self-resolves.
#[test]
fn predecessor_running_still_errors_episode_active() {
    let session = Session::builder("reset-config-still-active")
        .robot(robot())
        .build()
        .unwrap();
    let mut ep = session.start_episode("first").unwrap();
    let _ = ep.gate(&[0.0; 3], None, None);
    assert!(matches!(
        session.start_episode("second"),
        Err(RuntimeError::EpisodeActive)
    ));
    ep.terminate(TerminalOutcome::Success, "done");
    session.shutdown();
}
