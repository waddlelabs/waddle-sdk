//! Local curve progress follows acknowledgements, never a late wall clock.

use waddle_runtime::playback::{
    Completion, CurveUpdate, Feedback, FiniteReferences, PacedCurve, Playback, PlaybackError,
    PlaybackPolicy, PlaybackPoll,
};
use waddle_types::{ActionChunk, ProvenanceTag, SourceId, Step};

fn curve() -> PacedCurve {
    PacedCurve {
        times_s: vec![0.0, 1.0],
        positions: vec![vec![0.0], vec![1.0]],
        coefficients: vec![[0.0, 0.0, 0.0, 10.0, -15.0, 6.0].map(|x| vec![x]).into()],
        period_ns: 100_000_000,
        position_error: vec![0.05],
        tracking_tolerance: vec![0.06],
        sample_times_s: None,
    }
}

fn inputs() -> (ActionChunk, PlaybackPolicy) {
    let steps = [0.0, 1.0]
        .into_iter()
        .enumerate()
        .map(|(index, q)| Step {
            values: vec![q, 0.5].into(),
            velocity_feedforward: Some(vec![0.0, 0.0].into()),
            offset_ns: index as i64 * 1_000_000_000,
            part: None,
            gripper: None,
        })
        .collect();
    (
        ActionChunk {
            dims: 2,
            steps,
            horizon_ns: 1_000_000_000,
            t_emitted_ns: 0,
            t_obs_ns: 0,
            seq: 1,
            source: SourceId::new("test"),
            provenance: ProvenanceTag::policy(),
        },
        PlaybackPolicy {
            indices: vec![0],
            start_tolerance: vec![0.01],
            position_tolerance: vec![0.001],
            stationary_velocity: vec![0.01],
            settle_ns: 50_000_000,
            minimum_settle_samples: 1,
            max_lateness_ns: 1,
            timeout_ns: 2_000_000_000,
            allow_settled: false,
            reference_only: false,
            checkpoint_offsets_ns: vec![],
            allow_curve_updates: false,
            paced_curve: Some(curve()),
            feedback: None,
            gripper: None,
        },
    )
}

fn player() -> Playback {
    let (chunk, policy) = inputs();
    Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0)
}

fn measured(at: i64, q: f64) -> Feedback {
    Feedback {
        observed_ns: at,
        positions: vec![q, 0.5],
        velocities: vec![0.0, 0.0],
        gripper_force: None,
    }
}

fn dispatch(p: &mut Playback, at: i64, q: f64) -> Step {
    let index = p.dispatched();
    assert_eq!(
        p.poll(at, &measured(at, q)).unwrap(),
        PlaybackPoll::Dispatch(index)
    );
    p.reference(index).unwrap().clone()
}

#[test]
fn late_feedback_does_not_skip_curve_phase_or_allow_catchup_bursts() {
    let mut p = player();
    dispatch(&mut p, 0, 0.0);
    p.acknowledge_at(0, 20_000_000, 30_000_000).unwrap();
    assert_eq!(
        p.poll(100_000_000, &measured(100_000_000, 0.0)).unwrap(),
        PlaybackPoll::WaitUntil(120_000_000)
    );
    let step = dispatch(&mut p, 500_000_000, 0.0);
    // Exactly phase 0.1, despite half a second on the local clock.
    assert!((step.values[0] - 0.00856).abs() < 1e-12);
    p.acknowledge_at(1, 550_000_000, 570_000_000).unwrap();
    assert_eq!(
        p.poll(600_000_000, &measured(600_000_000, step.values[0]))
            .unwrap(),
        PlaybackPoll::WaitUntil(650_000_000)
    );
    assert_eq!(
        p.curve_progress().unwrap().acknowledged_curve_time_s,
        Some(0.1)
    );
}

#[test]
fn lag_shortens_the_exact_polynomial_and_scales_its_velocity_hint() {
    let mut p = player();
    dispatch(&mut p, 0, 0.0);
    p.acknowledge(0, 0).unwrap();
    dispatch(&mut p, 100_000_000, 0.0);
    p.acknowledge(1, 100_000_000).unwrap();
    let step = dispatch(&mut p, 200_000_000, 0.0);
    assert!(step.values[0] <= 0.045 + 1e-12 && step.values[0] > 0.044);
    p.acknowledge(2, 200_000_000).unwrap();
    let progress = p.curve_progress().unwrap();
    let t = progress.acknowledged_curve_time_s.unwrap();
    assert!(t > 0.1 && t < 0.2);
    let expected = 10.0 * t.powi(3) - 15.0 * t.powi(4) + 6.0 * t.powi(5);
    assert!((step.values[0] - expected).abs() < 1e-12);
    let velocity = (30.0 * t.powi(2) - 60.0 * t.powi(3) + 30.0 * t.powi(4)) * (t - 0.1) / 0.1;
    assert!((step.velocity_feedforward.unwrap()[0] - velocity).abs() < 1e-12);
    assert_eq!(progress.partial_ticks, 1);
    assert_eq!(progress.next_curve_time_s, Some(t + 0.1));
    assert_eq!(step.values[1], 0.5);
}

#[test]
fn a_repeated_reference_has_zero_velocity_and_cannot_extend_the_deadline() {
    let mut p = player();
    dispatch(&mut p, 0, 0.0);
    p.acknowledge(0, 0).unwrap();
    let step = dispatch(&mut p, 100_000_000, -0.048);
    assert_eq!(step.values[0], 0.0);
    assert_eq!(step.velocity_feedforward.unwrap()[0], 0.0);
    p.acknowledge(1, 100_000_000).unwrap();
    assert_eq!(p.curve_progress().unwrap().held_ticks, 1);
    assert_eq!(
        p.poll(2_000_000_000, &measured(2_000_000_000, 0.0)),
        Err(PlaybackError::Deadline)
    );
}

#[test]
fn tracking_failure_is_terminal_even_after_feedback_recovers() {
    let mut p = player();
    dispatch(&mut p, 0, 0.0);
    p.acknowledge(0, 0).unwrap();
    assert_eq!(
        p.poll(100_000_000, &measured(100_000_000, -0.1)),
        Err(PlaybackError::Tracking)
    );
    assert_eq!(
        p.poll(200_000_000, &measured(200_000_000, 0.0)),
        Err(PlaybackError::Tracking)
    );
    assert_eq!(p.dispatched(), 1);
}

#[test]
fn checkpoints_capture_fresh_feedback_after_acknowledged_phase_not_wall_time() {
    let (chunk, mut policy) = inputs();
    policy.checkpoint_offsets_ns = vec![100_000_000, 150_000_000, 1_000_000_000];
    policy.paced_curve.as_mut().unwrap().position_error = vec![0.1];
    policy.paced_curve.as_mut().unwrap().tracking_tolerance = vec![0.2];
    let mut p = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
    dispatch(&mut p, 0, 0.0);
    p.acknowledge(0, 0).unwrap();
    // The wall clock has crossed every marker, but curve phase has not.
    let first = dispatch(&mut p, 1_100_000_000, 0.0);
    assert!(p.checkpoints().is_empty());
    p.acknowledge(1, 1_100_000_000).unwrap();
    let sample = measured(1_100_000_000, first.values[0]);
    assert_eq!(
        p.poll(1_100_000_001, &sample).unwrap(),
        PlaybackPoll::Observe
    );
    assert!(p.checkpoints().is_empty());
    // Preserve the measured miss, not the nominal polynomial value.
    let observed = first.values[0] - 0.001;
    p.poll(1_100_000_002, &measured(1_100_000_002, observed))
        .unwrap();
    let checkpoint = &p.checkpoints()[0];
    assert_eq!(checkpoint.offset_ns, 100_000_000);
    assert_eq!(checkpoint.acknowledged_offset_ns, 100_000_000);
    assert_eq!(checkpoint.acknowledged_ns, 1_100_000_000);
    assert_eq!(checkpoint.elapsed_ns, 1_100_000_002);
    assert_eq!(checkpoint.feedback.positions[0], observed);
    let second = dispatch(&mut p, 1_200_000_000, first.values[0]);
    p.acknowledge(2, 1_200_000_000).unwrap();
    p.poll(1_200_000_001, &measured(1_200_000_001, second.values[0]))
        .unwrap();
    assert_eq!(p.checkpoints().len(), 2);
    assert_eq!(p.checkpoints()[1].offset_ns, 150_000_000);
    assert_eq!(p.checkpoints()[1].acknowledged_offset_ns, 200_000_000);
    p.cancel("stop before endpoint");
    assert!(
        p.poll(1_300_000_000, &measured(1_300_000_000, 1.0))
            .is_err()
    );
    assert_eq!(p.checkpoints().len(), 2);
}

#[test]
fn checkpoint_offsets_are_bounded_and_ordered() {
    for offsets in [
        vec![-1],
        vec![1_000_000_001],
        vec![2, 1],
        vec![1, 1],
        vec![0; 257],
    ] {
        let (chunk, mut policy) = inputs();
        policy.checkpoint_offsets_ns = offsets;
        assert!(FiniteReferences::new(chunk, policy).is_err());
    }
}

fn revisable() -> Playback {
    let (chunk, mut policy) = inputs();
    policy.reference_only = true;
    policy.allow_curve_updates = true;
    let curve = policy.paced_curve.as_mut().unwrap();
    curve.position_error = vec![0.3];
    curve.tracking_tolerance = vec![0.4];
    Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0)
}

fn suffix() -> PacedCurve {
    // At original phase 0.5: q=0.5, v=1.875, a=0. The changed endpoint
    // is 1.25 at phase 1.0, with v=a=0. This is a moving PVA handoff.
    PacedCurve {
        times_s: vec![0.0, 0.5],
        positions: vec![vec![0.5], vec![1.25]],
        coefficients: vec![
            [0.5, 0.9375, 0.0, 1.875, -3.75, 1.6875]
                .map(|x| vec![x])
                .into(),
        ],
        period_ns: 100_000_000,
        position_error: vec![0.3],
        tracking_tolerance: vec![0.4],
        sample_times_s: None,
    }
}

#[test]
fn future_pva_suffix_keeps_original_phase_pacing_held_rows_and_reference_completion() {
    let mut p = revisable();
    p.update_curve(CurveUpdate::new(0, 500_000_000, suffix()).unwrap())
        .unwrap();
    assert_eq!(p.curve_progress().unwrap().pending_revision, Some(1));
    let mut q = 0.0;
    for tick in 0..14 {
        let now = tick * 100_000_000;
        match p.poll(now, &measured(now, q)).unwrap() {
            PlaybackPoll::Dispatch(index) => {
                let step = p.reference(index).unwrap().clone();
                q = step.values[0];
                assert_eq!(step.values[1], 0.5);
                let phase = step.offset_ns as f64 * 1e-9;
                if phase < 0.5 {
                    assert!(
                        (q - (10.0 * phase.powi(3) - 15.0 * phase.powi(4) + 6.0 * phase.powi(5)))
                            .abs()
                            < 1e-12
                    );
                } else {
                    let u = (phase - 0.5) / 0.5;
                    let expected = 0.5 + 0.9375 * u + 1.875 * u.powi(3) - 3.75 * u.powi(4)
                        + 1.6875 * u.powi(5);
                    assert!((q - expected).abs() < 1e-12);
                }
                p.acknowledge(index, now).unwrap();
                assert_eq!(
                    p.curve_progress().unwrap().revision,
                    u64::from(phase >= 0.5)
                );
            }
            PlaybackPoll::Complete(outcome) => {
                assert_eq!(outcome, Completion::ReferencesDispatched);
                assert_eq!(q, 1.25);
                assert_eq!(p.curve_progress().unwrap().pending_revision, None);
                return;
            }
            other => panic!("unexpected {other:?}"),
        }
    }
    panic!("did not complete");
}

#[test]
fn update_refusals_keep_the_incumbent_and_never_overwrite_an_unacknowledged_write() {
    let mut p = revisable();
    dispatch(&mut p, 0, 0.0);
    assert_eq!(
        p.update_curve(CurveUpdate::new(0, 500_000_000, suffix()).unwrap()),
        Err(PlaybackError::DispatchPending)
    );
    p.acknowledge(0, 0).unwrap();
    assert!(
        p.update_curve(CurveUpdate::new(1, 500_000_000, suffix()).unwrap())
            .is_err()
    );
    for kind in ["position", "velocity", "acceleration", "cadence", "limit"] {
        let mut curve = suffix();
        match kind {
            "position" => {
                curve.positions[0][0] += 0.01;
                curve.coefficients[0][0][0] += 0.01;
                curve.coefficients[0][3][0] -= 0.1;
                curve.coefficients[0][4][0] += 0.15;
                curve.coefficients[0][5][0] -= 0.06;
            }
            "velocity" => {
                curve.coefficients[0][1][0] += 0.01;
                curve.coefficients[0][3][0] -= 0.06;
                curve.coefficients[0][4][0] += 0.08;
                curve.coefficients[0][5][0] -= 0.03;
            }
            "acceleration" => {
                curve.coefficients[0][2][0] += 0.01;
                curve.coefficients[0][3][0] -= 0.03;
                curve.coefficients[0][4][0] += 0.03;
                curve.coefficients[0][5][0] -= 0.01;
            }
            "cadence" => curve.period_ns *= 2,
            "limit" => curve.tracking_tolerance[0] *= 2.0,
            _ => unreachable!(),
        }
        assert!(
            p.update_curve(CurveUpdate::new(0, 500_000_000, curve).unwrap())
                .is_err(),
            "{kind}"
        );
    }
    assert_eq!(p.curve_progress().unwrap().pending_revision, None);
    assert!((dispatch(&mut p, 100_000_000, 0.0).values[0] - 0.00856).abs() < 1e-12);
    p.acknowledge(1, 100_000_000).unwrap();
    assert!(
        p.update_curve(CurveUpdate::new(0, 100_000_000, suffix()).unwrap())
            .is_err()
    );
    p.update_curve(CurveUpdate::new(0, 500_000_000, suffix()).unwrap())
        .unwrap();
    assert_eq!(
        p.update_curve(CurveUpdate::new(0, 500_000_000, suffix()).unwrap()),
        Err(PlaybackError::Busy)
    );
    // An accepted suffix neither renews the absolute deadline nor clears a fault.
    assert_eq!(
        p.poll(2_000_000_000, &measured(2_000_000_000, 0.00856)),
        Err(PlaybackError::Deadline)
    );
    assert!(
        p.update_curve(CurveUpdate::new(1, 700_000_000, suffix()).unwrap())
            .is_err()
    );
}

#[test]
fn revisable_policy_requires_reference_only_ungridded_curves_without_markers() {
    for invalid in ["completion", "grid", "marker", "no_curve"] {
        let (chunk, mut policy) = inputs();
        policy.reference_only = true;
        policy.allow_curve_updates = true;
        match invalid {
            "completion" => policy.reference_only = false,
            "grid" => policy.paced_curve.as_mut().unwrap().sample_times_s = Some(vec![0.0, 1.0]),
            "marker" => policy.checkpoint_offsets_ns = vec![0],
            "no_curve" => policy.paced_curve = None,
            _ => unreachable!(),
        }
        assert!(FiniteReferences::new(chunk, policy).is_err(), "{invalid}");
    }
    for switch in [-1, 0, 120_000_000_000] {
        assert!(CurveUpdate::new(0, switch, suffix()).is_err());
    }
    let mut empty = suffix();
    empty.times_s.clear();
    assert!(CurveUpdate::new(0, 500_000_000, empty).is_err());
}

#[test]
fn successive_revisions_preserve_global_phase_and_check_the_current_suffix_pva() {
    let mut p = revisable();
    p.update_curve(CurveUpdate::new(0, 500_000_000, suffix()).unwrap())
        .unwrap();
    let mut q = 0.0;
    for tick in 0..16 {
        let now = tick * 100_000_000;
        if tick == 7 {
            assert_eq!(p.curve_progress().unwrap().revision, 1);
            // Existing suffix at original phase 0.8 (local 0.3): measured
            // analytical PVA, joined to 1.5 at original phase 1.2.
            let (start, v, a, end, duration) = (1.11272, 1.632, -8.64, 1.5, 0.4);
            let (b1, b2) = (v * duration, 0.5 * a * duration * duration);
            let (dq, dv, da) = (end - start - b1 - b2, -b1 - 2.0 * b2, -2.0 * b2);
            let next = PacedCurve {
                times_s: vec![0.0, duration],
                positions: vec![vec![start], vec![end]],
                coefficients: vec![
                    [
                        start,
                        b1,
                        b2,
                        10.0 * dq - 4.0 * dv + 0.5 * da,
                        -15.0 * dq + 7.0 * dv - da,
                        6.0 * dq - 3.0 * dv + 0.5 * da,
                    ]
                    .map(|c| vec![c])
                    .into(),
                ],
                ..suffix()
            };
            assert!(
                p.update_curve(CurveUpdate::new(0, 800_000_000, next.clone()).unwrap())
                    .is_err()
            );
            p.update_curve(CurveUpdate::new(1, 800_000_000, next).unwrap())
                .unwrap();
        }
        match p.poll(now, &measured(now, q)).unwrap() {
            PlaybackPoll::Dispatch(index) => {
                q = p.reference(index).unwrap().values[0];
                p.acknowledge(index, now).unwrap();
            }
            PlaybackPoll::Complete(Completion::ReferencesDispatched) => {
                let progress = p.curve_progress().unwrap();
                assert_eq!(progress.revision, 2);
                assert!((progress.acknowledged_curve_time_s.unwrap() - 1.2).abs() < 1e-12);
                assert_eq!(q, 1.5);
                return;
            }
            other => panic!("unexpected {other:?}"),
        }
    }
    panic!("did not complete");
}

#[test]
fn exact_grid_boundary_remains_pending_after_a_partial_acknowledgement() {
    let (chunk, mut policy) = inputs();
    policy.paced_curve.as_mut().unwrap().sample_times_s =
        Some((0..=10).map(|i| f64::from(i) / 10.0).collect());
    let mut p = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
    dispatch(&mut p, 0, 0.0);
    p.acknowledge(0, 0).unwrap();
    dispatch(&mut p, 100_000_000, 0.0);
    p.acknowledge(1, 100_000_000).unwrap();
    dispatch(&mut p, 200_000_000, 0.0);
    p.acknowledge(2, 200_000_000).unwrap();
    let progress = p.curve_progress().unwrap();
    assert!(progress.acknowledged_curve_time_s.unwrap() < 0.2);
    assert_eq!(progress.next_curve_time_s, Some(0.2));
}

#[test]
fn polynomial_admission_refuses_mismatched_geometry_timing_and_dimensions() {
    for change in 0..7 {
        let (mut chunk, mut policy) = inputs();
        let c = policy.paced_curve.as_mut().unwrap();
        match change {
            0 => c.positions[1][0] = 2.0,
            1 => c.coefficients[0][3][0] = f64::NAN,
            2 => c.period_ns = 0,
            3 => c.times_s[1] = 0.9,
            4 => c.position_error.clear(),
            5 => c.sample_times_s = Some(vec![0.0, 0.2]),
            _ => chunk.steps[1].values[0] = 0.9,
        }
        assert!(
            FiniteReferences::new(chunk, policy).is_err(),
            "case {change}"
        );
    }
}

#[test]
fn final_reference_acknowledgement_still_requires_fresh_measured_settling() {
    let (chunk, mut policy) = inputs();
    policy.paced_curve.as_mut().unwrap().position_error = vec![2.0];
    policy.paced_curve.as_mut().unwrap().tracking_tolerance = vec![2.0];
    let mut p = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
    let mut q = 0.0;
    let mut at = 0;
    while !p.curve_progress().unwrap().endpoint_acknowledged {
        q = dispatch(&mut p, at, q).values[0];
        p.acknowledge(p.dispatched(), at).unwrap();
        at += 100_000_000;
    }
    assert_eq!(q, 1.0);
    assert_eq!(p.poll(at, &measured(at, q)).unwrap(), PlaybackPoll::Observe);
    assert_eq!(
        p.poll(at + 50_000_000, &measured(at + 50_000_000, q))
            .unwrap(),
        PlaybackPoll::Complete(waddle_runtime::playback::Completion::Arrived)
    );
}

#[test]
fn reference_only_curve_retains_tracking_through_fresh_endpoint_feedback() {
    for tracking_fault in [false, true] {
        let (chunk, mut policy) = inputs();
        policy.reference_only = true;
        policy.timeout_ns = 20_000_000_000;
        let mut p = Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0);
        let mut q = 0.0;
        let mut now = 0;
        loop {
            let step = dispatch(&mut p, now, q);
            q = step.values[0];
            p.acknowledge(p.dispatched(), now).unwrap();
            if p.curve_progress().unwrap().endpoint_acknowledged {
                break;
            }
            now += 100_000_000;
        }
        let mut observation = measured(now + 1, q - if tracking_fault { 0.1 } else { 0.02 });
        observation.velocities[0] = 1.0;
        let result = p.poll(now + 1, &observation);
        if tracking_fault {
            assert_eq!(result, Err(PlaybackError::Tracking));
        } else {
            assert_eq!(
                result,
                Ok(PlaybackPoll::Complete(
                    waddle_runtime::playback::Completion::ReferencesDispatched
                ))
            );
        }
    }
}
