//! Goal completion may omit constant rest, never accepted future motion.

use super::*;
use waddle_runtime::playback::{FeedbackPolicy, PlaybackGoal, PositionWindow};

fn resting_inputs(enabled: bool, window: bool) -> (ActionChunk, PlaybackPolicy) {
    let (mut chunk, mut policy) = inputs();
    chunk.horizon_ns = 8_000_000_000;
    chunk.steps[1].offset_ns = chunk.horizon_ns;
    policy.timeout_ns = Some(20_000_000_000);
    policy.allow_curve_updates = true;
    policy.complete_in_resting_tail = enabled;
    policy.minimum_settle_samples = 3;
    policy.goal = Some(PlaybackGoal {
        joint_positions: Some(vec![1.0]),
        pose: None,
        position_tolerance_m: 0.001,
        orientation_tolerance_rad: None,
    });
    let curve = policy.paced_curve.as_mut().unwrap();
    curve.times_s.push(8.0);
    curve.positions.push(vec![1.0]);
    curve
        .coefficients
        .push([1.0, 0.0, 0.0, 0.0, 0.0, 0.0].map(|x| vec![x]).into());
    if window {
        policy.feedback = Some(FeedbackPolicy {
            position_window: Some(PositionWindow {
                max_drift: vec![0.0005],
                max_gap_ns: Some(200_000_000),
                during_motion: false,
                consecutive_samples: 3,
            }),
            ..Default::default()
        });
    }
    (chunk, policy)
}

fn tick(player: &mut Playback, now: &mut i64, q: &mut f64) -> PlaybackPoll {
    let result = player.poll(*now, &measured(*now, *q)).unwrap();
    if let PlaybackPoll::Dispatch(index) = result {
        *q = player.reference(index).unwrap().values[0];
        player.acknowledge(index, *now).unwrap();
    }
    *now += 100_000_000;
    result
}

fn to_rest(player: &mut Playback) -> (i64, f64) {
    let (mut now, mut q) = (0, 0.0);
    for _ in 0..100 {
        assert!(!matches!(
            tick(player, &mut now, &mut q),
            PlaybackPoll::Complete(_)
        ));
        if player
            .curve_progress()
            .unwrap()
            .acknowledged_curve_time_s
            .unwrap_or(0.0)
            >= 1.0
        {
            return (now, q);
        }
    }
    panic!("moving prefix did not finish");
}

#[test]
fn local_goal_finishes_during_constant_tail_with_both_settling_methods() {
    for window in [false, true] {
        let (chunk, policy) = resting_inputs(true, window);
        let mut player = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
        let (mut now, mut q) = to_rest(&mut player);
        assert!((q - 1.0).abs() < 1e-9);
        for _ in 0..if window { 4 } else { 2 } {
            assert_eq!(tick(&mut player, &mut now, &mut q), PlaybackPoll::Observe);
        }
        assert_eq!(
            tick(&mut player, &mut now, &mut q),
            PlaybackPoll::Complete(Completion::Arrived)
        );
        assert!(!player.curve_progress().unwrap().endpoint_acknowledged);
    }
}

#[test]
fn accepted_future_motion_must_run_or_be_withdrawn_before_completion() {
    for withdraw in [false, true] {
        let (chunk, policy) = resting_inputs(true, false);
        let mut player = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
        let (mut now, mut q) = to_rest(&mut player);
        let mut suffix = curve();
        suffix.times_s = vec![0.0, 0.4, 0.8, 5.0];
        suffix.positions = vec![vec![1.0], vec![1.1], vec![1.0], vec![1.0]];
        suffix.coefficients = vec![
            [1.0, 0.0, 0.0, 1.0, -1.5, 0.6].map(|x| vec![x]).into(),
            [1.1, 0.0, 0.0, -1.0, 1.5, -0.6].map(|x| vec![x]).into(),
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0].map(|x| vec![x]).into(),
        ];
        player
            .update_curve(CurveUpdate::new(0, 1_600_000_000, suffix).unwrap())
            .unwrap();
        for _ in 0..3 {
            assert!(!matches!(
                tick(&mut player, &mut now, &mut q),
                PlaybackPoll::Complete(_)
            ));
        }
        assert_eq!(player.curve_progress().unwrap().pending_revision, Some(1));
        if withdraw {
            player.withdraw_curve(0).unwrap();
        }
        let mut moved = false;
        let mut outcome = None;
        for _ in 0..100 {
            let result = tick(&mut player, &mut now, &mut q);
            moved |= q > 1.05;
            if let PlaybackPoll::Complete(result) = result {
                outcome = Some(result);
                break;
            }
        }
        assert_eq!(outcome, Some(Completion::Arrived));
        assert_eq!(moved, !withdraw);
        assert_eq!(
            player.curve_progress().unwrap().revision,
            u64::from(!withdraw)
        );
    }
}

#[test]
fn future_checkpoint_is_observed_before_resting_goal_completes() {
    let (chunk, mut policy) = resting_inputs(true, false);
    policy.checkpoint_offsets_ns = vec![2_000_000_000];
    let mut player = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
    let (mut now, mut q) = to_rest(&mut player);
    while player.checkpoints().is_empty() {
        assert!(!matches!(
            tick(&mut player, &mut now, &mut q),
            PlaybackPoll::Complete(_)
        ));
    }
    assert!(player.checkpoints()[0].acknowledged_offset_ns >= 2_000_000_000);
    assert_eq!(tick(&mut player, &mut now, &mut q), PlaybackPoll::Observe);
    assert_eq!(
        tick(&mut player, &mut now, &mut q),
        PlaybackPoll::Complete(Completion::Arrived)
    );
}

#[test]
fn resting_goal_preserves_tracking_cancellation_and_original_deadline() {
    for failure in ["tracking", "cancel", "deadline"] {
        let (chunk, policy) = resting_inputs(true, false);
        let mut player = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
        let (mut now, mut q) = to_rest(&mut player);
        let expected = match failure {
            "tracking" => {
                q = 0.8;
                PlaybackError::Tracking
            }
            "cancel" => {
                player.cancel("test cancellation");
                PlaybackError::Cancelled("test cancellation".into())
            }
            _ => {
                now = 20_000_000_000;
                PlaybackError::Deadline
            }
        };
        assert_eq!(player.poll(now, &measured(now, q)), Err(expected));
    }
}

#[test]
fn default_goal_still_preserves_an_explicit_rest_duration() {
    let (chunk, policy) = resting_inputs(false, false);
    let mut player = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
    let (mut now, mut q) = to_rest(&mut player);
    for _ in 0..5 {
        assert!(!matches!(
            tick(&mut player, &mut now, &mut q),
            PlaybackPoll::Complete(_)
        ));
    }
}
