//! Dependent execution advances on the same worker without caller polling.

use super::*;
use crate::playback::{PlaybackDependency, PlaybackQueue};

fn queued(
    h: &Harness,
    parent: &ExecutionHandle,
    offset: f64,
    outcomes: Vec<Completion>,
    start_before_ns: Option<i64>,
) -> Result<ExecutionHandle, PlaybackError> {
    let (mut chunk, policy) = reference_inputs(Some("left"), true, None);
    for step in &mut chunk.steps {
        step.values = step
            .values
            .iter()
            .map(|q| q + offset)
            .collect::<Vec<_>>()
            .into();
    }
    let mut references = FiniteReferences::new(chunk, policy).unwrap();
    references.queue = PlaybackQueue {
        after: vec![PlaybackDependency {
            handle: parent.clone(),
            outcomes,
        }],
        start_before_ns,
    };
    h.dispatch.start_playback(
        h.mirror.clone(),
        EpisodeId::new("episode"),
        references,
        h.io.clone(),
        10_000_000,
        0,
    )
}

#[test]
fn queued_curves_finish_in_order_without_caller_round_trips() {
    let h = Harness::new();
    let (p, entered, release) = pause();
    *h.io.observation_pause.lock() = Some(p);
    let first = start_named_policy(&h, "left", h.io.clone(), true).unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    let second = queued(&h, &first, 0.2, vec![Completion::Arrived], None);
    let third = second
        .as_ref()
        .map(|second| queued(&h, second, 0.4, vec![Completion::Arrived], None));
    let fork = queued(&h, &first, 0.2, vec![Completion::Arrived], None);
    release.send(()).unwrap();
    let third = third.unwrap().unwrap();
    let second = second.unwrap();
    assert!(matches!(fork, Err(PlaybackError::Busy)));
    assert_eq!(finished(&third).result, Some(Ok(Completion::Arrived)));
    assert_eq!(first.snapshot().result, Some(Ok(Completion::Arrived)));
    assert_eq!(second.snapshot().result, Some(Ok(Completion::Arrived)));
    assert!(third.snapshot().waiting_for.is_empty());
    let writes: Vec<_> = h.io.writes.lock().iter().map(|(_, _, q)| *q).collect();
    assert_eq!(writes.len(), 9);
    for (actual, expected) in writes
        .iter()
        .zip([0.0, 0.1, 0.2, 0.2, 0.3, 0.4, 0.4, 0.5, 0.6])
    {
        assert!((actual - expected).abs() < 1e-12);
    }
    assert!(
        !h.io
            .events
            .lock()
            .iter()
            .any(|event| matches!(*event, "hold" | "part_hold"))
    );
}

#[test]
fn cancelling_or_expiring_queued_work_leaves_its_predecessor_running() {
    for expire in [false, true] {
        let h = Harness::new();
        let (p, entered, release) = pause();
        *h.io.observation_pause.lock() = Some(p);
        let first = start_named_policy(&h, "left", h.io.clone(), true).unwrap();
        entered.recv_timeout(Duration::from_secs(5)).unwrap();
        let next = queued(
            &h,
            &first,
            0.2,
            vec![Completion::Arrived],
            expire.then_some(1),
        )
        .unwrap();
        assert_eq!(next.snapshot().waiting_for, vec![first.snapshot().id]);
        assert_eq!(next.snapshot().elapsed_ns, None);
        if !expire {
            next.cancel("cancel queued command only");
        }
        release.send(()).unwrap();
        let result = finished(&next);
        assert!(matches!(
            result.result,
            Some(Err(ExecutionFailure::Playback(
                PlaybackError::StartExpired | PlaybackError::Cancelled(_)
            )))
        ));
        assert_eq!(result.dispatched, 0);
        assert_eq!(result.elapsed_ns, None);
        assert!(result.cleanup.is_none());
        assert_eq!(finished(&first).result, Some(Ok(Completion::Arrived)));
        assert_eq!(h.io.writes.lock().len(), 3);
        assert!(
            !h.io
                .events
                .lock()
                .iter()
                .any(|event| matches!(*event, "hold" | "part_hold"))
        );
    }
}

#[test]
fn unaccepted_predecessor_outcome_never_starts_or_adds_a_hold() {
    let h = Harness::new();
    let first = start_named_policy(&h, "left", h.io.clone(), true).unwrap();
    assert_eq!(finished(&first).result, Some(Ok(Completion::Arrived)));
    let next = queued(&h, &first, 0.2, vec![Completion::ForceReached], None).unwrap();
    let result = finished(&next);
    assert!(matches!(
        result.result,
        Some(Err(ExecutionFailure::Playback(
            PlaybackError::DependencyFailed(_)
        )))
    ));
    assert_eq!(result.dispatched, 0);
    assert!(result.cleanup.is_none());
    assert_eq!(h.io.writes.lock().len(), 3);
}

#[test]
fn queued_activation_checks_the_actual_measured_start() {
    let h = Harness::new();
    let first = start_named_policy(&h, "left", h.io.clone(), true).unwrap();
    assert_eq!(finished(&first).result, Some(Ok(Completion::Arrived)));
    let next = queued(&h, &first, 0.5, vec![Completion::Arrived], None).unwrap();
    let result = finished(&next);
    assert_eq!(
        result.result,
        Some(Err(ExecutionFailure::Playback(PlaybackError::StaleStart)))
    );
    assert_eq!(result.dispatched, 0);
    assert!(result.cleanup.unwrap().result.is_ok());
    assert_eq!(h.io.writes.lock().len(), 3);
}

#[test]
fn another_sdk_cannot_supply_a_dependency_even_with_the_same_episode_label() {
    let first_sdk = Harness::new();
    let parent = start_named(&first_sdk, "left", first_sdk.io.clone()).unwrap();
    assert_eq!(finished(&parent).result, Some(Ok(Completion::Arrived)));
    let other_sdk = Harness::new();
    assert!(matches!(
        queued(&other_sdk, &parent, 0.2, vec![Completion::Arrived], None),
        Err(PlaybackError::Invalid(_))
    ));
    assert!(other_sdk.io.writes.lock().is_empty());
}

#[test]
fn failed_predecessor_retains_unknown_cleanup_and_never_starts_its_child() {
    let h = Harness::new();
    let (p, entered, release) = pause();
    *h.io.dispatch_pause.lock() = Some(p);
    let original = PlaybackIoError {
        detail: "write failed".into(),
        evidence: "original write evidence".into(),
        outcome_unknown: true,
    };
    *h.io.error.lock() = Some(original.clone());
    *h.io.part_hold_error.lock() = Some(PlaybackIoError {
        detail: "Hold failed".into(),
        evidence: "original cleanup evidence".into(),
        outcome_unknown: true,
    });
    let parent = start_named_policy(&h, "left", h.io.clone(), true).unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    let child = queued(&h, &parent, 0.2, vec![Completion::Arrived], None).unwrap();
    release.send(()).unwrap();
    let result = finished(&child);
    assert!(matches!(
        result.result,
        Some(Err(ExecutionFailure::Playback(
            PlaybackError::DependencyFailed(_)
        )))
    ));
    assert_eq!(result.dispatched, 0);
    assert!(result.cleanup.is_none());
    let parent = parent.snapshot();
    assert_eq!(
        parent.result,
        Some(Err(ExecutionFailure::Dispatch(original)))
    );
    assert!(parent.cleanup.unwrap().result.is_err());
    assert_eq!(*h.io.events.lock(), ["dispatch", "part_hold"]);
    assert!(h.io.writes.lock().is_empty());
}

#[test]
fn site_stop_drains_active_and_dependent_work_before_releasing_parts() {
    for stop in [VerbRequest::Hold, VerbRequest::Estop] {
        let h = Harness::new();
        let (p, entered, release) = pause();
        *h.io.observation_pause.lock() = Some(p);
        let parent = start_named_policy(&h, "left", h.io.clone(), true).unwrap();
        entered.recv_timeout(Duration::from_secs(5)).unwrap();
        let child = queued(&h, &parent, 0.2, vec![Completion::Arrived], None).unwrap();
        let verb = stop.verb();
        h.dispatch.request(stop);
        assert!(parent.snapshot().result.is_none());
        assert!(child.snapshot().result.is_none());
        assert!(
            h.dispatch
                .playback
                .active(&EpisodeId::new("episode"), Some("left"))
        );
        release.send(()).unwrap();
        for handle in [parent, child] {
            let result = finished(&handle);
            assert!(result.result.unwrap().is_err());
            let cleanup = result.cleanup.unwrap();
            assert_eq!(cleanup.verb, verb);
            assert!(cleanup.result.is_ok());
        }
        assert!(h.io.writes.lock().is_empty());
        assert!(
            !h.dispatch
                .playback
                .active(&EpisodeId::new("episode"), Some("left"))
        );
    }
}

#[test]
fn queued_admission_is_bounded_and_requires_the_complete_reservation_chain() {
    let h = Harness::new();
    let (p, entered, release) = pause();
    *h.io.observation_pause.lock() = Some(p);
    let first = start_named_policy(&h, "left", h.io.clone(), true).unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    let mut handles = vec![first];
    for i in 1..=16 {
        let next = queued(
            &h,
            handles.last().unwrap(),
            f64::from(i) * 0.2,
            vec![Completion::Arrived],
            None,
        )
        .unwrap();
        handles.push(next);
    }
    let overflow = queued(
        &h,
        handles.last().unwrap(),
        3.4,
        vec![Completion::Arrived],
        None,
    );
    h.dispatch.request(VerbRequest::Hold);
    release.send(()).unwrap();
    assert!(matches!(overflow, Err(PlaybackError::Busy)));
    for handle in handles {
        assert!(finished(&handle).result.unwrap().is_err());
    }
    assert!(h.io.writes.lock().is_empty());
}
