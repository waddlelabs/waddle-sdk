//! Report-only trajectories settle against their accepted, revised endpoint.

use super::*;
use waddle_runtime::playback::{FeedbackPolicy, PositionWindow};

fn report(window: bool) -> Playback {
    let (chunk, mut policy) = inputs();
    policy.allow_curve_updates = true;
    policy.allow_settled = true;
    policy.credit_pacing_delay = true;
    policy.timeout_ns = Some(10_000_000_000);
    policy.settle_ns = if window { 200_000_000 } else { 0 };
    if window {
        policy.minimum_settle_samples = 3;
        policy.feedback = Some(FeedbackPolicy {
            position_window: Some(PositionWindow {
                max_drift: vec![0.0005],
                max_gap_ns: Some(200_000_000),
                during_motion: true,
                consecutive_samples: 3,
            }),
            ..Default::default()
        });
    }
    let mut player = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
    let suffix = PacedCurve {
        times_s: vec![0.0, 0.5],
        positions: vec![vec![1.0], vec![1.1]],
        coefficients: vec![[1.0, 0.0, 0.0, 1.0, -1.5, 0.6].map(|x| vec![x]).into()],
        ..curve()
    };
    player
        .update_curve(CurveUpdate::new(0, 1_000_000_000, suffix).unwrap())
        .unwrap();
    player
}

#[test]
fn revised_report_locally_finishes_with_either_settling_method_and_a_miss() {
    for window in [false, true] {
        for miss in [0.0, 0.02] {
            let mut player = report(window);
            let (mut now, mut q) = (0, 0.0);
            let mut outcome = None;
            for _ in 0..200 {
                let sample = measured(now, q);
                match player.poll(now, &sample).unwrap() {
                    PlaybackPoll::Dispatch(index) => {
                        q = player.reference(index).unwrap().values[0] - miss;
                        player.acknowledge(index, now).unwrap();
                    }
                    PlaybackPoll::Complete(value) => {
                        outcome = Some(value);
                        break;
                    }
                    _ => (),
                }
                now += 100_000_000;
            }
            assert!((q - (1.1 - miss)).abs() < 1e-9);
            assert_eq!(player.curve_progress().unwrap().revision, 1);
            assert!(player.curve_progress().unwrap().endpoint_acknowledged);
            assert_eq!(
                outcome,
                Some(if miss == 0.0 {
                    Completion::Arrived
                } else {
                    Completion::Settled
                })
            );
            player.cancel("late cancel must retain local result");
            assert_eq!(
                player.poll(now + 20_000_000_000, &measured(now, q)),
                Ok(PlaybackPoll::Complete(outcome.unwrap()))
            );
        }
    }
}

#[test]
fn revised_report_preserves_cancellation_tracking_and_deadline() {
    for failure in ["cancel", "tracking", "deadline"] {
        let mut player = report(false);
        assert_eq!(
            player.poll(0, &measured(0, 0.0)),
            Ok(PlaybackPoll::Dispatch(0))
        );
        player.acknowledge(0, 0).unwrap();
        let (now, q, expected) = match failure {
            "cancel" => {
                player.cancel("cancel report");
                (
                    100_000_000,
                    0.0,
                    PlaybackError::Cancelled("cancel report".into()),
                )
            }
            "tracking" => (100_000_000, 0.5, PlaybackError::Tracking),
            _ => (10_000_000_000, 0.0, PlaybackError::Deadline),
        };
        assert_eq!(player.poll(now, &measured(now, q)), Err(expected));
    }
}
