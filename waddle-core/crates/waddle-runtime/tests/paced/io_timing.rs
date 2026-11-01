//! Original bent-path/IO regressions at the local playback boundary.

use super::*;

const PERIOD: i64 = 20_000_000;

fn bent_path(corner: f64, goal: f64, segment_s: f64, hints: bool) -> Playback {
    let (mut chunk, mut policy) = inputs();
    chunk.dims = 3;
    chunk.horizon_ns = (2.0 * segment_s * 1e9).round() as i64;
    chunk.steps = [[0.0, 0.0], [corner, 0.0], [corner, goal]]
        .into_iter()
        .enumerate()
        .map(|(index, q)| Step {
            values: vec![q[0], q[1], 0.5].into(),
            offset_ns: (index as f64 * segment_s * 1e9).round() as i64,
            velocity_feedforward: hints.then(|| vec![0.0; 3].into()),
            part: None,
            gripper: None,
        })
        .collect();
    policy.indices = vec![0, 1];
    policy.start_tolerance = vec![0.001; 2];
    policy.position_tolerance = vec![0.006; 2];
    policy.stationary_velocity = vec![0.01; 2];
    policy.timeout_ns = Some(chunk.horizon_ns + 2_000_000_000);
    policy.paced_curve = Some(PacedCurve {
        times_s: vec![0.0, segment_s, 2.0 * segment_s],
        positions: vec![vec![0.0, 0.0], vec![corner, 0.0], vec![corner, goal]],
        coefficients: vec![
            vec![
                vec![0.0, 0.0],
                vec![0.0; 2],
                vec![0.0; 2],
                vec![10.0 * corner, 0.0],
                vec![-15.0 * corner, 0.0],
                vec![6.0 * corner, 0.0],
            ],
            vec![
                vec![corner, 0.0],
                vec![0.0; 2],
                vec![0.0; 2],
                vec![0.0, 10.0 * goal],
                vec![0.0, -15.0 * goal],
                vec![0.0, 6.0 * goal],
            ],
        ],
        period_ns: PERIOD,
        position_error: vec![0.04; 2],
        tracking_tolerance: vec![0.05; 2],
        // Ordinary rest-to-rest paths retain the planner's exact sample grid,
        // including every corner; revisable analytic paths use their own proof.
        sample_times_s: Some({
            let ticks = (segment_s * 1e9 / PERIOD as f64).round() as usize;
            (0..=2 * ticks)
                .map(|i| i as f64 / ticks as f64 * segment_s)
                .collect()
        }),
    });
    Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0)
}

fn feedback(at: i64, q: [f64; 2]) -> Feedback {
    Feedback {
        observed_ns: at,
        positions: vec![q[0], q[1], 0.5],
        velocities: vec![0.0; 3],
        gripper_force: None,
        pose: None,
    }
}

#[test]
fn delayed_read_or_write_preserves_the_corner_and_exact_endpoint() {
    for delay_write in [false, true] {
        let mut p = bent_path(0.04, 0.04, 0.4, true);
        let mut now = 0;
        let mut q = [0.0; 2];
        let mut writes = Vec::new();
        let mut delayed = false;
        loop {
            if writes.len() == 3 && !delayed && !delay_write {
                now += 160_000_000;
                delayed = true;
            }
            match p.poll(now, &feedback(now, q)).unwrap() {
                PlaybackPoll::Dispatch(index) => {
                    let step = p.reference(index).unwrap().clone();
                    q = [step.values[0], step.values[1]];
                    writes.push((now, q));
                    let submitted = now;
                    if writes.len() == 4 && delay_write {
                        now += 160_000_000;
                        delayed = true;
                    }
                    p.acknowledge_at(index, submitted, now).unwrap();
                    now += PERIOD;
                }
                PlaybackPoll::WaitUntil(at) => now = at,
                PlaybackPoll::Observe => now += PERIOD,
                PlaybackPoll::Complete(outcome) => {
                    assert_eq!(outcome, Completion::Arrived);
                    break;
                }
            }
        }
        assert!(delayed);
        assert_eq!(q, [0.04, 0.04]);
        let bend = writes.iter().position(|(_, q)| *q == [0.04, 0.0]).unwrap();
        assert!(writes[..bend].iter().all(|(_, q)| q[1] == 0.0));
        assert!(writes[bend..].iter().all(|(_, q)| q[0] == 0.04));
        for pair in writes.windows(2) {
            assert!(pair[1].0 - pair[0].0 >= PERIOD);
            for j in 0..2 {
                let movement = pair[1].1[j] - pair[0].1[j];
                assert!(movement >= -1e-12);
                assert!(movement <= 0.5 * (pair[1].0 - pair[0].0) as f64 / 1e9 + 1e-12);
            }
        }
        assert!(p.curve_progress().unwrap().endpoint_acknowledged);
    }
}

#[test]
fn lag_keeps_headroom_and_zero_velocity_at_the_checked_corner() {
    for hints in [false, true] {
        let mut p = bent_path(0.12, -0.08, 1.0, hints);
        let mut now = 0;
        let mut q = [0.0; 2];
        let mut writes = Vec::new();
        loop {
            match p.poll(now, &feedback(now, q)).unwrap() {
                PlaybackPoll::Dispatch(index) => {
                    let step = p.reference(index).unwrap().clone();
                    for (measured, target) in q.iter_mut().zip(&step.values) {
                        let delta = target - *measured;
                        assert!(delta.abs() <= 0.036 + 1e-10);
                        *measured += delta * 0.04;
                    }
                    assert_eq!(step.values[2], 0.5);
                    writes.push(step);
                    // Driver overhead consumes time inside the control period.
                    p.acknowledge_at(index, now, now + 6_000_000).unwrap();
                    now += PERIOD;
                }
                PlaybackPoll::WaitUntil(at) => now = at,
                PlaybackPoll::Observe => {
                    // The physical position loop continues toward the held
                    // endpoint after playback finishes sending references.
                    let target = &writes.last().unwrap().values;
                    for (measured, target) in q.iter_mut().zip(target) {
                        *measured += (target - *measured) * 0.04;
                    }
                    now += PERIOD;
                }
                PlaybackPoll::Complete(outcome) => {
                    assert_eq!(outcome, Completion::Arrived);
                    break;
                }
            }
        }
        let bend = writes
            .iter()
            .position(|s| s.values[..2] == [0.12, 0.0])
            .unwrap();
        assert!(writes[..bend].iter().all(|s| s.values[1] == 0.0));
        assert!(writes[bend..].iter().all(|s| s.values[0] == 0.12));
        assert_eq!(writes.last().unwrap().values[..2], [0.12, -0.08]);
        assert!(p.curve_progress().unwrap().partial_ticks > 0);
        if hints {
            assert!(writes.iter().any(|s| {
                s.velocity_feedforward
                    .as_ref()
                    .unwrap()
                    .iter()
                    .any(|v| *v != 0.0)
            }));
            for step in [&writes[bend], writes.last().unwrap()] {
                assert_eq!(&step.velocity_feedforward.as_ref().unwrap()[..], &[0.0; 3]);
            }
        } else {
            assert!(writes.iter().all(|s| s.velocity_feedforward.is_none()));
        }
    }
}
