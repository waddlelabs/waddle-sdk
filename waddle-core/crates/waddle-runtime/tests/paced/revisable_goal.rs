//! Checked corrections keep one local goal, result identity and deadline.

use super::*;
use waddle_runtime::playback::PlaybackGoal;

fn goal_inputs() -> (ActionChunk, PlaybackPolicy) {
    let (chunk, mut policy) = inputs();
    policy.allow_curve_updates = true;
    policy.timeout_ns = Some(10_000_000_000);
    policy.goal = Some(PlaybackGoal {
        allow_updates: false,
        dwell: None,
        joint_positions: Some(vec![1.0]),
        pose: None,
        position_tolerance_m: 0.001,
        orientation_tolerance_rad: None,
    });
    (chunk, policy)
}

fn correction() -> CurveUpdate {
    let mut spec = curve();
    spec.times_s = vec![0.0, 0.3];
    spec.positions = vec![vec![1.0], vec![1.02]];
    spec.coefficients = vec![[1.0, 0.0, 0.0, 0.2, -0.3, 0.12].map(|x| vec![x]).into()];
    CurveUpdate::new(0, 1_000_000_000, spec).unwrap()
}

fn drive_to_endpoint(player: &mut Playback, now: &mut i64, q: &mut f64) {
    for _ in 0..200 {
        match player.poll(*now, &measured(*now, *q)).unwrap() {
            PlaybackPoll::Dispatch(index) => {
                *q = player.reference(index).unwrap().values[0] - 0.02;
                player.acknowledge(index, *now).unwrap();
            }
            PlaybackPoll::WaitUntil(at) => {
                *now = at;
                continue;
            }
            PlaybackPoll::Observe => (),
            PlaybackPoll::Complete(outcome) => panic!("completed before endpoint: {outcome:?}"),
        }
        *now += 100_000_000;
        if player.curve_progress().unwrap().endpoint_acknowledged {
            return;
        }
    }
    panic!("curve did not finish");
}

#[test]
fn correction_preserves_original_goal_and_local_measured_completion() {
    let (chunk, policy) = goal_inputs();
    let mut player = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
    let (mut now, mut q) = (0, 0.0);
    drive_to_endpoint(&mut player, &mut now, &mut q);
    assert!((q - 0.98).abs() < 1e-9);
    assert_eq!(
        player.poll(now, &measured(now, q)).unwrap(),
        PlaybackPoll::Observe
    );
    player.update_curve(correction()).unwrap();
    drive_to_endpoint(&mut player, &mut now, &mut q);
    assert!((q - 1.0).abs() < 1e-9);
    assert_eq!(player.curve_progress().unwrap().revision, 1);
    assert_eq!(
        player.poll(now, &measured(now, q)).unwrap(),
        PlaybackPoll::Observe
    );
    now += 100_000_000;
    assert_eq!(
        player.poll(now, &measured(now, q)).unwrap(),
        PlaybackPoll::Complete(Completion::Arrived)
    );
    assert_eq!(
        player
            .poll(20_000_000_000, &measured(20_000_000_000, q))
            .unwrap(),
        PlaybackPoll::Complete(Completion::Arrived)
    );
}

#[test]
fn accepted_or_withdrawn_correction_cannot_renew_the_total_deadline() {
    for withdraw in [false, true] {
        let (chunk, policy) = goal_inputs();
        let mut player = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
        let (mut now, mut q) = (0, 0.0);
        drive_to_endpoint(&mut player, &mut now, &mut q);
        assert_eq!(
            player.poll(now, &measured(now, q)).unwrap(),
            PlaybackPoll::Observe
        );
        player.update_curve(correction()).unwrap();
        if withdraw {
            player.withdraw_curve(0).unwrap();
            assert!(player.curve_progress().unwrap().endpoint_acknowledged);
        }
        let dispatched = player.dispatched();
        assert_eq!(
            player.poll(10_000_000_000, &measured(10_000_000_000, q)),
            Err(PlaybackError::Deadline)
        );
        assert_eq!(player.dispatched(), dispatched);
        assert!(player.update_curve(correction()).is_err());
    }
}

#[test]
fn original_checkpoints_finish_before_a_correction_and_are_never_repeated() {
    let (chunk, mut policy) = goal_inputs();
    policy.checkpoint_offsets_ns = vec![500_000_000, 1_000_000_000];
    let mut player = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
    // Replacing an as-yet-unobserved original waypoint would falsify its evidence.
    assert!(player.update_curve(correction()).is_err());
    let (mut now, mut q) = (0, 0.0);
    drive_to_endpoint(&mut player, &mut now, &mut q);
    assert_eq!(
        player.poll(now, &measured(now, q)).unwrap(),
        PlaybackPoll::Observe
    );
    assert_eq!(player.checkpoints().len(), 2);
    let stamps: Vec<_> = player
        .checkpoints()
        .iter()
        .map(|c| c.feedback.observed_ns)
        .collect();
    player.update_curve(correction()).unwrap();
    drive_to_endpoint(&mut player, &mut now, &mut q);
    player.poll(now, &measured(now, q)).unwrap();
    assert_eq!(
        player
            .checkpoints()
            .iter()
            .map(|c| c.feedback.observed_ns)
            .collect::<Vec<_>>(),
        stamps
    );
}

#[test]
fn accepted_correction_must_finish_even_if_measurement_reaches_the_old_endpoint() {
    for withdraw in [false, true] {
        let (chunk, policy) = goal_inputs();
        let mut player = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
        let (mut now, mut q) = (0, 0.0);
        drive_to_endpoint(&mut player, &mut now, &mut q);
        player.poll(now, &measured(now, q)).unwrap();
        now += 100_000_000;
        player.update_curve(correction()).unwrap();
        if withdraw {
            player.withdraw_curve(0).unwrap();
            assert_eq!(
                player.poll(now, &measured(now, 1.0)).unwrap(),
                PlaybackPoll::Observe
            );
            now += 100_000_000;
            assert_eq!(
                player.poll(now, &measured(now, 1.0)).unwrap(),
                PlaybackPoll::Complete(Completion::Arrived)
            );
        } else {
            assert!(matches!(
                player.poll(now, &measured(now, 1.0)).unwrap(),
                PlaybackPoll::Dispatch(_)
            ));
            assert_eq!(player.curve_progress().unwrap().pending_revision, Some(1));
        }
    }
}

#[test]
fn refused_discontinuity_leaves_the_original_goal_and_curve_unchanged() {
    let (chunk, policy) = goal_inputs();
    let mut player = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
    let (mut now, mut q) = (0, 0.0);
    drive_to_endpoint(&mut player, &mut now, &mut q);
    let changed = CurveUpdate::new(0, 1_000_000_000, curve()).unwrap();
    assert!(player.update_curve(changed).is_err());
    assert_eq!(player.curve_progress().unwrap().revision, 0);
    assert_eq!(player.curve_progress().unwrap().pending_revision, None);
    assert!(player.curve_progress().unwrap().endpoint_acknowledged);
    assert_eq!(
        player.poll(now, &measured(now, 1.0)).unwrap(),
        PlaybackPoll::Observe
    );
    now += 100_000_000;
    assert_eq!(
        player.poll(now, &measured(now, 1.0)).unwrap(),
        PlaybackPoll::Complete(Completion::Arrived)
    );
}

#[test]
fn measured_goal_cannot_become_an_unbounded_stream() {
    let (chunk, mut policy) = goal_inputs();
    policy.timeout_ns = None;
    policy.watchdog = Some(waddle_runtime::playback::PlaybackWatchdog {
        observed_ns: 0,
        timeout_ns: 250_000_000,
    });
    assert!(FiniteReferences::new(chunk, policy).is_err());
}
