//! Local curve progress follows acknowledgements, never a late wall clock.

use waddle_runtime::playback::{
    Feedback, FiniteReferences, PacedCurve, Playback, PlaybackError, PlaybackPolicy, PlaybackPoll,
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
            max_lateness_ns: 1,
            timeout_ns: 2_000_000_000,
            allow_settled: false,
            paced_curve: Some(curve()),
            feedback: None,
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
