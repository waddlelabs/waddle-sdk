//! Target acceptance shares the existing worker's I/O, deadline and stop boundaries.

use super::*;
use crate::playback::{GoalDwell, GoalUpdate, PlaybackGoal, Pose};

struct PoseIo {
    io: Arc<MemoryIo>,
    pause: Mutex<Option<Pause>>,
}

fn pose(x: f64) -> Pose {
    Pose {
        position_m: [x, 0.0, 0.0],
        quaternion_wxyz: [1.0, 0.0, 0.0, 0.0],
        frame_id: "base".into(),
    }
}

impl PlaybackIo for PoseIo {
    fn observe(&self) -> Result<Feedback, PlaybackIoError> {
        let mut measured = self.io.observe()?;
        measured.pose = Some(pose(measured.positions[0]));
        Ok(measured)
    }

    fn dispatch(
        &self,
        reference: &Step,
        measured: &Feedback,
        force_n: Option<f64>,
    ) -> Result<ReferenceReceipt, PlaybackIoError> {
        let result = self.io.dispatch(reference, measured, force_n)?;
        if reference.offset_ns == 80_000_000 {
            *self.io.observation_pause.lock() = self.pause.lock().take();
        }
        Ok(result)
    }
}

#[test]
fn queued_target_uses_local_feedback_and_preserves_deadline_and_cancel() {
    for end in ["arrive", "cancel", "deadline", "stale", "dispatch_failure"] {
        let h = Harness::new();
        let (p, entered, release) = pause();
        let pause = if end == "dispatch_failure" {
            *h.io.dispatch_pause.lock() = Some(p);
            None
        } else {
            Some(p)
        };
        let io = Arc::new(PoseIo {
            io: h.io.clone(),
            pause: Mutex::new(pause),
        });
        let (chunk, mut policy) = reference_inputs(Some("left"), false, None);
        policy.goal = Some(PlaybackGoal {
            allow_updates: true,
            dwell: Some(GoalDwell {
                duration_ns: 40_000_000,
                exit_tolerance_multiplier: 2.0,
            }),
            joint_positions: None,
            pose: Some(pose(0.5)),
            position_tolerance_m: 0.001,
            orientation_tolerance_rad: None,
        });
        let handle = h
            .dispatch
            .start_playback(
                h.mirror.clone(),
                EpisodeId::new("episode"),
                FiniteReferences::new(chunk, policy).unwrap(),
                io,
                10_000_000,
                0,
            )
            .unwrap();
        entered.recv_timeout(Duration::from_secs(5)).unwrap();
        let update = GoalUpdate {
            expected_revision: if end == "stale" { 1 } else { 0 },
            pose: pose(0.2),
        };
        assert_eq!(handle.update_goal(update.clone()), Ok(1));
        assert_eq!(handle.update_goal(update), Err(PlaybackError::Busy));
        if end == "cancel" {
            handle.cancel("cancel before goal acceptance");
        }
        if end == "deadline" {
            h.io.clock.0.advance(501_000_000);
        }
        let dispatch_error = PlaybackIoError {
            detail: "original write failure".into(),
            evidence: "retained driver fault".into(),
            outcome_unknown: true,
        };
        if end == "dispatch_failure" {
            *h.io.error.lock() = Some(dispatch_error.clone());
        }
        release.send(()).unwrap();
        let snapshot = finished(&handle);
        let target = snapshot.goal_target.as_ref().unwrap();
        let receipt = snapshot.goal_update.as_ref().unwrap();
        assert_eq!(receipt.request_id, 1);
        if end == "arrive" {
            assert_eq!(receipt.result, Some(Ok(1)));
            assert_eq!(target.revision, 1);
            assert_eq!(target.pose.position_m, [0.2, 0.0, 0.0]);
            assert_eq!(snapshot.result, Some(Ok(Completion::Arrived)));
            let dwell = snapshot.goal_dwell.as_ref().unwrap();
            assert_eq!(
                dwell.observed_ns,
                Some(snapshot.feedback.as_ref().unwrap().observed_ns)
            );
            assert!(dwell.observed_ns.unwrap() - dwell.since_ns.unwrap() >= 40_000_000);
        } else {
            assert!(receipt.result.as_ref().unwrap().is_err());
            assert_eq!(target.revision, 0);
            assert_eq!(target.pose.position_m, [0.5, 0.0, 0.0]);
            assert_eq!(
                snapshot.result,
                Some(Err(if end == "dispatch_failure" {
                    ExecutionFailure::Dispatch(dispatch_error)
                } else {
                    ExecutionFailure::Playback(if end == "cancel" {
                        PlaybackError::Cancelled("cancel before goal acceptance".into())
                    } else {
                        PlaybackError::Deadline
                    })
                }))
            );
            assert_eq!(snapshot.cleanup.as_ref().unwrap().result, Ok(()));
        }
        assert!(
            handle
                .update_goal(GoalUpdate {
                    expected_revision: target.revision,
                    pose: pose(0.3)
                })
                .is_err()
        );
        handle.cancel("late cancel");
        assert_eq!(format!("{:?}", handle.snapshot()), format!("{snapshot:?}"));
    }
}
