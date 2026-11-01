//! A delayed observation cannot let a queued correction revive an expired goal.

use super::*;
use crate::playback::{CurveUpdate, PacedCurve, PlaybackGoal};

struct PauseAfterEndpoint {
    io: Arc<MemoryIo>,
    pause: Mutex<Option<Pause>>,
}

impl PlaybackIo for PauseAfterEndpoint {
    fn observe(&self) -> Result<Feedback, PlaybackIoError> {
        self.io.observe()
    }

    fn dispatch(
        &self,
        reference: &Step,
        measured: &Feedback,
        force_n: Option<f64>,
    ) -> Result<ReferenceReceipt, PlaybackIoError> {
        let receipt = self.io.dispatch(reference, measured, force_n)?;
        if reference.offset_ns == 80_000_000 {
            *self.io.observation_pause.lock() = self.pause.lock().take();
        }
        Ok(receipt)
    }
}

#[test]
fn correction_waiting_on_observation_cannot_revive_either_deadline() {
    for settling_deadline in [false, true] {
        let h = Harness::new();
        let (p, entered, release) = pause();
        let io = Arc::new(PauseAfterEndpoint {
            io: h.io.clone(),
            pause: Mutex::new(Some(p)),
        });
        let (chunk, mut policy) = reference_inputs(Some("left"), false, None);
        policy.allow_curve_updates = true;
        policy.goal = Some(PlaybackGoal {
            joint_positions: Some(vec![0.3]),
            pose: None,
            position_tolerance_m: 0.001,
            orientation_tolerance_rad: None,
        });
        policy.settle_timeout_ns = settling_deadline.then_some(50_000_000);
        let curve = PacedCurve {
            times_s: vec![0.0, 0.04, 0.08],
            positions: vec![vec![0.0], vec![0.1], vec![0.2]],
            coefficients: vec![
                [0.0, 0.0, 0.0, 1.0, -1.5, 0.6].map(|x| vec![x]).into(),
                [0.1, 0.0, 0.0, 1.0, -1.5, 0.6].map(|x| vec![x]).into(),
            ],
            period_ns: 10_000_000,
            position_error: vec![0.2],
            tracking_tolerance: vec![0.2],
            sample_times_s: None,
        };
        policy.paced_curve = Some(curve.clone());
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
        let count = h.io.writes.lock().len();
        let update = CurveUpdate::new(
            0,
            80_000_000,
            PacedCurve {
                times_s: vec![0.0, 0.04],
                positions: vec![vec![0.2], vec![0.3]],
                coefficients: vec![[0.2, 0.0, 0.0, 1.0, -1.5, 0.6].map(|x| vec![x]).into()],
                ..curve
            },
        )
        .unwrap();
        handle.update_curve(update).unwrap();
        h.io.clock.0.advance(if settling_deadline {
            51_000_000
        } else {
            501_000_000
        });
        release.send(()).unwrap();
        let result = finished(&handle);
        assert_eq!(
            result.result,
            Some(Err(ExecutionFailure::Playback(PlaybackError::Deadline)))
        );
        assert_eq!(
            result.curve_update.unwrap().result,
            Some(Err(PlaybackError::Deadline))
        );
        assert_eq!(result.cleanup.unwrap().result, Ok(()));
        assert_eq!(h.io.writes.lock().len(), count);
        assert_eq!(result.curve_progress.unwrap().pending_revision, None);
    }
}
