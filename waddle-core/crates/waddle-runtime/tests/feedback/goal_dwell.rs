//! Port of Metal's tracked-arrival regressions onto the existing SDK worker.

use super::*;
use waddle_runtime::playback::GoalDwell;

fn sample(at: i64, position_error: f64, angle: f64, moving: bool) -> Feedback {
    let mut value = measured(at, 0.0);
    value.velocities[0] = if moving { 0.1 } else { 0.0 };
    value.pose = Some(Pose {
        position_m: [position_error, 0.0, 0.0],
        quaternion_wxyz: [(angle / 2.0).cos(), 0.0, 0.0, (angle / 2.0).sin()],
        frame_id: "base".into(),
    });
    value
}

fn player(window: bool) -> Playback {
    let (chunk, mut policy) = inputs(&[0.0]);
    policy.timeout_ns = Some(5_000_000_000);
    policy.settle_ns = if window { 500_000_000 } else { 0 };
    policy.goal = Some(PlaybackGoal {
        dwell: Some(GoalDwell {
            duration_ns: 300_000_000,
            exit_tolerance_multiplier: 2.0,
        }),
        joint_positions: None,
        pose: Some(Pose {
            position_m: [0.0; 3],
            quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
            frame_id: "base".into(),
        }),
        position_tolerance_m: 0.002,
        orientation_tolerance_rad: Some(std::f64::consts::PI / 6.0),
    });
    if window {
        policy.feedback.as_mut().unwrap().position_window = Some(PositionWindow {
            max_drift: vec![0.001],
            max_gap_ns: Some(120_000_000),
            consecutive_samples: 1,
            during_motion: true,
        });
    }
    let mut player = start(chunk, policy);
    dispatch(&mut player, &sample(0, 0.01, 0.0, false));
    // Stationarity is measured before pose-goal entry, just as in Metal.
    for tick in 1..25 {
        let at = tick * 40_000_000;
        assert_eq!(
            player.poll(at, &sample(at, 0.01, 0.0, false)).unwrap(),
            PlaybackPoll::Observe
        );
    }
    player
}

#[test]
fn nominal_band_jitter_preserves_pose_dwell_and_independent_stationarity() {
    for window in [false, true] {
        for orientation in [false, true] {
            let mut player = player(window);
            for tick in 0..9 {
                let at = 1_000_000_000 + tick * 40_000_000;
                let position = if orientation {
                    0.001
                } else if tick % 2 == 0 {
                    0.0019
                } else {
                    0.0021
                };
                let angle = if orientation {
                    if tick % 2 == 0 { 29.9_f64 } else { 30.1_f64 }.to_radians()
                } else {
                    0.0
                };
                let mut feedback = sample(at, position, angle, false);
                feedback.positions[0] = (tick % 2) as f64 * 0.0001;
                assert_eq!(
                    player.poll(at, &feedback).unwrap(),
                    if tick == 8 {
                        PlaybackPoll::Complete(Completion::Arrived)
                    } else {
                        PlaybackPoll::Observe
                    }
                );
            }
            assert_eq!(player.goal_dwell().unwrap().since_ns, Some(1_000_000_000));
            assert_eq!(player.goal_dwell().unwrap().restarts, 0);
        }
    }
}

#[test]
fn large_target_change_rearms_dwell_and_never_completes_outside_nominal_band() {
    for orientation in [false, true] {
        let mut player = player(false);
        for (offset, position, angle) in [
            (0, 0.001, 0.0),
            (
                200_000_000,
                if orientation { 0.001 } else { 0.01 },
                if orientation {
                    61_f64.to_radians()
                } else {
                    0.0
                },
            ),
            (240_000_000, 0.001, 0.0),
            (560_000_000, 0.0021, 0.0),
        ] {
            let at = 1_000_000_000 + offset;
            assert_eq!(
                player
                    .poll(at, &sample(at, position, angle, false))
                    .unwrap(),
                PlaybackPoll::Observe
            );
        }
        let at = 1_600_000_000;
        assert_eq!(
            player.poll(at, &sample(at, 0.0019, 0.0, false)).unwrap(),
            PlaybackPoll::Complete(Completion::Arrived)
        );
        assert_eq!(player.goal_dwell().unwrap().since_ns, Some(1_240_000_000));
        assert_eq!(player.goal_dwell().unwrap().restarts, 1);
    }
}

#[test]
fn motion_prevents_completion_without_restarting_pose_dwell() {
    let mut player = player(false);
    for at in [1_000_000_000, 1_320_000_000] {
        assert_eq!(
            player.poll(at, &sample(at, 0.001, 0.0, true)).unwrap(),
            PlaybackPoll::Observe
        );
    }
    let at = 1_360_000_000;
    assert_eq!(
        player.poll(at, &sample(at, 0.001, 0.0, false)).unwrap(),
        PlaybackPoll::Complete(Completion::Arrived)
    );
    assert_eq!(player.goal_dwell().unwrap().since_ns, Some(1_000_000_000));
}

#[test]
fn pose_dwell_cannot_replace_cancellation_or_renew_the_total_deadline() {
    for cancel in [false, true] {
        let mut player = player(false);
        assert_eq!(
            player
                .poll(1_000_000_000, &sample(1_000_000_000, 0.001, 0.0, false))
                .unwrap(),
            PlaybackPoll::Observe
        );
        let (at, expected) = if cancel {
            player.cancel("cancel during goal dwell");
            (
                1_100_000_000,
                PlaybackError::Cancelled("cancel during goal dwell".into()),
            )
        } else {
            (5_000_000_000, PlaybackError::Deadline)
        };
        assert_eq!(
            player.poll(at, &sample(at, 0.001, 0.0, false)),
            Err(expected)
        );
    }
}
