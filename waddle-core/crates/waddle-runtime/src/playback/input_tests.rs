//! Input arrival is independent of planning, feedback reads and status replies.

use super::*;
use crate::playback::PlaybackInput;

fn input(h: &Harness) -> PlaybackInput {
    h.dispatch
        .start_playback_input(
            EpisodeId::new("episode"),
            Arc::new(h.io.clock.0.clone()),
            250_000_000,
        )
        .unwrap()
}

fn input_request(input: &PlaybackInput) -> FiniteReferences {
    let mut references =
        references_with_watchdog(Some("left"), false, Some(input.snapshot().watchdog));
    references.queue.input = Some(input.clone());
    references
}

fn start_input(h: &Harness, input: &PlaybackInput) -> ExecutionHandle {
    h.dispatch
        .start_playback(
            h.mirror.clone(),
            EpisodeId::new("episode"),
            input_request(input),
            h.io.clone(),
            10_000_000,
            0,
        )
        .unwrap()
}

#[test]
fn input_renews_before_admission_without_feedback_or_status_polling() {
    let h = Harness::new();
    let input = input(&h);
    let initial = input.snapshot().watchdog;
    for sequence in 1..=10 {
        h.io.clock.0.advance(160_000_000);
        let accepted = input.refresh(sequence).unwrap();
        assert_eq!(accepted.sequence, sequence);
        assert_eq!(accepted.watchdog.timeout_ns, 250_000_000);
        assert_eq!(accepted.watchdog.observed_ns, h.io.clock.0.now().0);
    }
    assert!(input.snapshot().watchdog.observed_ns > initial.deadline_ns());
    assert!(h.io.writes.lock().is_empty());
    let handle = start_input(&h, &input);
    let result = handle.wait();
    assert_eq!(result.result, Some(Ok(Completion::Arrived)));
    assert!(result.dispatched > 0);
    h.dispatch.shutdown();
}

#[test]
fn duplicate_and_reordered_input_cannot_extend_expiry_or_revive_it() {
    let h = Harness::new();
    let input = input(&h);
    h.io.clock.0.advance(80_000_000);
    let accepted = input.refresh(2).unwrap();
    h.io.clock.0.advance(160_000_000);
    for sequence in [2, 1, 0] {
        let duplicate = input.refresh(sequence).unwrap();
        assert_eq!(duplicate.sequence, 2);
        assert_eq!(duplicate.watchdog, accepted.watchdog);
    }
    h.io.clock.0.advance(90_000_000);
    assert_eq!(
        input.refresh(3).unwrap_err(),
        PlaybackError::WatchdogExpired
    );
    assert_eq!(
        input.snapshot().failure,
        Some(PlaybackError::WatchdogExpired)
    );
    assert_eq!(
        input.refresh(4).unwrap_err(),
        PlaybackError::WatchdogExpired
    );
    assert!(h.io.writes.lock().is_empty());
    h.dispatch.shutdown();
}

#[test]
fn release_before_delayed_admission_refuses_without_dispatch() {
    let h = Harness::new();
    let input = input(&h);
    let request = input_request(&input);
    input.release("button released");
    assert!(matches!(input.refresh(1), Err(PlaybackError::Cancelled(_))));
    let result = h.dispatch.start_playback(
        h.mirror.clone(),
        EpisodeId::new("episode"),
        request,
        h.io.clone(),
        10_000_000,
        0,
    );
    assert!(matches!(result, Err(PlaybackError::Cancelled(_))));
    assert!(h.io.events.lock().is_empty());
    h.dispatch.shutdown();
}

#[test]
fn input_release_during_feedback_uses_existing_hold_and_never_writes() {
    let h = Harness::new();
    let input = input(&h);
    let (barrier, entered, resume) = pause();
    *h.io.observation_pause.lock() = Some(barrier);
    let handle = start_input(&h, &input);
    entered.recv_timeout(Duration::from_secs(2)).unwrap();
    input.release("button released");
    resume.send(()).unwrap();
    let result = handle.wait();
    assert!(matches!(
        result.result,
        Some(Err(ExecutionFailure::Playback(PlaybackError::Cancelled(_))))
    ));
    assert_eq!(result.dispatched, 0);
    assert!(result.cleanup.unwrap().result.is_ok());
    assert!(h.io.writes.lock().is_empty());
    h.dispatch.shutdown();
}

#[test]
fn input_expiry_during_feedback_cannot_be_renewed_before_worker_drains() {
    let h = Harness::new();
    let input = input(&h);
    let (barrier, entered, resume) = pause();
    *h.io.observation_pause.lock() = Some(barrier);
    let handle = start_input(&h, &input);
    entered.recv_timeout(Duration::from_secs(2)).unwrap();
    h.io.clock.0.advance(250_000_000);
    assert_eq!(
        input.refresh(1).unwrap_err(),
        PlaybackError::WatchdogExpired
    );
    resume.send(()).unwrap();
    let result = handle.wait();
    assert_eq!(
        result.result,
        Some(Err(ExecutionFailure::Playback(
            PlaybackError::WatchdogExpired
        )))
    );
    assert_eq!(result.dispatched, 0);
    assert!(result.cleanup.unwrap().result.is_ok());
    h.dispatch.shutdown();
}

#[test]
fn global_stop_and_run_close_fence_inputs_even_before_any_buffer_exists() {
    for stop in [true, false] {
        let h = Harness::new();
        let input = input(&h);
        if stop {
            let receipt = h.dispatch.request_completed_stop(false);
            assert!(
                receipt
                    .recv_timeout(Duration::from_secs(2))
                    .unwrap()
                    .result
                    .is_ok()
            );
        } else {
            h.dispatch.drain_playback(&EpisodeId::new("episode"));
        }
        assert!(matches!(input.refresh(1), Err(PlaybackError::Cancelled(_))));
        assert!(h.io.writes.lock().is_empty());
        h.dispatch.shutdown();
    }
}

#[test]
fn input_binding_requires_the_same_sdk_and_run() {
    let h = Harness::new();
    let other = Harness::new();
    let foreign = input(&other);
    let wrong_run = h
        .dispatch
        .start_playback_input(
            EpisodeId::new("other-run"),
            Arc::new(h.io.clock.0.clone()),
            250_000_000,
        )
        .unwrap();
    for input in [foreign, wrong_run] {
        let result = h.dispatch.start_playback(
            h.mirror.clone(),
            EpisodeId::new("episode"),
            input_request(&input),
            h.io.clone(),
            10_000_000,
            0,
        );
        assert!(matches!(result, Err(PlaybackError::Invalid(_))));
    }
    assert!(h.io.writes.lock().is_empty());
    h.dispatch.shutdown();
    other.dispatch.shutdown();
}

#[test]
fn release_after_movement_drains_without_another_write_and_keeps_hold_failure() {
    for hold_fails in [false, true] {
        let h = Harness::new();
        h.hold_error.store(hold_fails, Ordering::SeqCst);
        let input = input(&h);
        let (write_pause, writing, resume_write) = pause();
        *h.io.dispatch_pause.lock() = Some(write_pause);
        let handle = start_input(&h, &input);
        writing.recv_timeout(Duration::from_secs(2)).unwrap();
        let (read_pause, reading, resume_read) = pause();
        *h.io.observation_pause.lock() = Some(read_pause);
        resume_write.send(()).unwrap();
        reading.recv_timeout(Duration::from_secs(2)).unwrap();
        // The first reference is the initial position. Resume through a second
        // write and pause the next read so this witnesses actual mock motion.
        let (write_pause, writing, resume_write) = pause();
        *h.io.dispatch_pause.lock() = Some(write_pause);
        resume_read.send(()).unwrap();
        writing.recv_timeout(Duration::from_secs(2)).unwrap();
        let (read_pause, reading, resume_read) = pause();
        *h.io.observation_pause.lock() = Some(read_pause);
        resume_write.send(()).unwrap();
        reading.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(h.io.writes.lock().len(), 2);
        assert!(*h.io.q.lock() > 0.0);
        assert!(matches!(
            handle.refresh_watchdog(0),
            Err(PlaybackError::Invalid(_))
        ));
        input.release("button released after dispatch");
        resume_read.send(()).unwrap();
        let result = handle.wait();
        assert_eq!(result.dispatched, 2);
        assert!(matches!(
            result.result,
            Some(Err(ExecutionFailure::Playback(PlaybackError::Cancelled(_))))
        ));
        assert_eq!(result.cleanup.unwrap().result.is_err(), hold_fails);
        assert_eq!(h.io.writes.lock().len(), 2);
        assert!(matches!(input.refresh(1), Err(PlaybackError::Cancelled(_))));
        h.dispatch.shutdown();
    }
}

#[test]
fn input_timeout_and_retention_are_bounded() {
    let h = Harness::new();
    for timeout in [0, -1, 180_000_000_001, i64::MAX] {
        assert!(matches!(
            h.dispatch.start_playback_input(
                EpisodeId::new("episode"),
                Arc::new(h.io.clock.0.clone()),
                timeout,
            ),
            Err(PlaybackError::Invalid(_))
        ));
    }
    let inputs: Vec<_> = (0..128).map(|_| input(&h)).collect();
    assert!(matches!(
        h.dispatch.start_playback_input(
            EpisodeId::new("episode"),
            Arc::new(h.io.clock.0.clone()),
            250_000_000,
        ),
        Err(PlaybackError::Busy)
    ));
    inputs[0].release("released");
    let replacement = input(&h);
    assert_ne!(replacement.snapshot().id, inputs[0].snapshot().id);
    assert!(matches!(
        inputs[0].refresh(1),
        Err(PlaybackError::Cancelled(_))
    ));
    h.dispatch.shutdown();
}

#[test]
fn pending_stop_fences_new_input_until_callback_completes() {
    let h = Harness::new();
    let previous = input(&h);
    let (barrier, entered, resume) = pause();
    *h.hold_pause.lock() = Some(barrier);
    let receipt = h.dispatch.request_completed_stop(false);
    entered.recv_timeout(Duration::from_secs(2)).unwrap();
    assert!(matches!(
        h.dispatch.start_playback_input(
            EpisodeId::new("episode"),
            Arc::new(h.io.clock.0.clone()),
            250_000_000,
        ),
        Err(PlaybackError::Busy)
    ));
    resume.send(()).unwrap();
    assert!(
        receipt
            .recv_timeout(Duration::from_secs(2))
            .unwrap()
            .result
            .is_ok()
    );
    let next = input(&h);
    assert!(next.refresh(1).is_ok());
    assert!(matches!(
        previous.refresh(1),
        Err(PlaybackError::Cancelled(_))
    ));
    h.dispatch.shutdown();
}

#[test]
fn scoped_buffer_cancel_keeps_live_input_only_after_confirmed_hold() {
    for hold_fails in [false, true] {
        let h = Harness::new();
        if hold_fails {
            *h.io.part_hold_error.lock() = Some(PlaybackIoError {
                detail: "scoped Hold failed".into(),
                evidence: "original Hold failure".into(),
                outcome_unknown: true,
            });
        }
        let input = input(&h);
        let mut request =
            references_with_watchdog(Some("left"), true, Some(input.snapshot().watchdog));
        request.queue.input = Some(input.clone());
        let (barrier, entered, resume) = pause();
        *h.io.observation_pause.lock() = Some(barrier);
        let handle = h
            .dispatch
            .start_playback(
                h.mirror.clone(),
                EpisodeId::new("episode"),
                request,
                h.io.clone(),
                10_000_000,
                0,
            )
            .unwrap();
        entered.recv_timeout(Duration::from_secs(2)).unwrap();
        handle.cancel("replace checked direction");
        resume.send(()).unwrap();
        let result = handle.wait();
        assert!(matches!(
            result.result,
            Some(Err(ExecutionFailure::Playback(PlaybackError::Cancelled(_))))
        ));
        let cleanup = result.cleanup.unwrap();
        assert_eq!(cleanup.part.as_deref(), Some("left"));
        assert_eq!(cleanup.result.is_err(), hold_fails);
        assert_eq!(result.dispatched, 0);
        if hold_fails {
            assert!(matches!(input.refresh(1), Err(PlaybackError::Cancelled(_))));
        } else {
            input.refresh(1).unwrap();
            let successor = start_input(&h, &input).wait();
            assert_eq!(successor.result, Some(Ok(Completion::Arrived)));
            assert!(successor.dispatched > 0);
            input.release("operator released");
            assert!(matches!(input.refresh(2), Err(PlaybackError::Cancelled(_))));
        }
        h.dispatch.shutdown();
    }
}
