//! Deterministic measured holding, hint direction and encoder settling.

use waddle_runtime::playback::{
    Completion, Feedback, FeedbackPolicy, FiniteReferences, HeldJointGroup, Playback,
    PlaybackError, PlaybackPolicy, PlaybackPoll, PositionWindow, VelocityHints,
};
use waddle_types::{ActionChunk, ProvenanceTag, SourceId, Step};

fn inputs(targets: &[f64]) -> (ActionChunk, PlaybackPolicy) {
    let steps: Vec<_> = targets
        .iter()
        .enumerate()
        .map(|(i, q)| Step {
            values: vec![*q, 0.5, -0.5, 0.8].into(),
            velocity_feedforward: Some(vec![0.0; 4].into()),
            offset_ns: i as i64 * 100,
            part: None,
            gripper: None,
        })
        .collect();
    (
        ActionChunk {
            dims: 4,
            horizon_ns: steps.last().unwrap().offset_ns,
            steps,
            t_emitted_ns: 0,
            t_obs_ns: 0,
            seq: 1,
            source: SourceId::new("feedback-test"),
            provenance: ProvenanceTag::policy(),
        },
        PlaybackPolicy {
            indices: vec![0],
            start_tolerance: vec![0.01],
            position_tolerance: vec![0.001],
            stationary_velocity: vec![0.01],
            settle_ns: 50,
            minimum_settle_samples: 1,
            max_lateness_ns: 10,
            timeout_ns: 1_000,
            allow_settled: false,
            reference_only: false,
            checkpoint_offsets_ns: vec![],
            allow_curve_updates: false,
            credit_pacing_delay: false,
            part_hold: false,
            paced_curve: None,
            feedback: Some(FeedbackPolicy::default()),
            gripper: None,
        },
    )
}

fn measured(at: i64, q: f64) -> Feedback {
    Feedback {
        observed_ns: at,
        positions: vec![q, 0.5, -0.5, 0.8],
        velocities: vec![0.0; 4],
        gripper_force: None,
    }
}

fn start(chunk: ActionChunk, policy: PlaybackPolicy) -> Playback {
    Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0)
}

fn dispatch(p: &mut Playback, feedback: &Feedback) -> Step {
    let index = p.dispatched();
    assert_eq!(
        p.poll(feedback.observed_ns, feedback).unwrap(),
        PlaybackPoll::Dispatch(index)
    );
    let step = p.reference(index).unwrap().clone();
    p.acknowledge(index, feedback.observed_ns).unwrap();
    step
}

fn window_policy(policy: &mut PlaybackPolicy) -> &mut PositionWindow {
    policy
        .feedback
        .as_mut()
        .unwrap()
        .position_window
        .get_or_insert(PositionWindow {
            max_drift: vec![0.0005],
            max_gap_ns: None,
            during_motion: false,
            consecutive_samples: 1,
        })
}

#[test]
fn held_group_compensates_opposing_velocity_without_changing_moving_curve_rows() {
    let (chunk, mut policy) = inputs(&[0.0]);
    policy
        .feedback
        .as_mut()
        .unwrap()
        .held_groups
        .push(HeldJointGroup {
            indices: vec![1, 2],
            position_error: vec![0.1, 0.1],
            period_ns: 100_000_000,
        });
    let mut p = start(chunk, policy);
    let mut feedback = measured(0, 0.0);
    feedback.positions[1] = 0.3;
    feedback.positions[2] = -0.4;
    feedback.velocities[1] = -0.5;
    let step = dispatch(&mut p, &feedback);
    // First row has .09 - .05 headroom; both held rows share scale .2.
    assert!((step.values[1] - 0.34).abs() < 1e-12);
    assert!((step.values[2] + 0.42).abs() < 1e-12);
    assert_eq!(step.values[0], 0.0);
    assert_eq!(step.values[3], 0.8);
}

#[test]
fn measured_rows_follow_feedback_and_admissible_held_targets_keep_exact_bits() {
    let (chunk, mut policy) = inputs(&[0.0]);
    let spec = policy.feedback.as_mut().unwrap();
    spec.measured_indices = vec![3];
    spec.held_groups.push(HeldJointGroup {
        indices: vec![1, 2],
        position_error: vec![0.1, 0.1],
        period_ns: 100_000_000,
    });
    let mut p = start(chunk, policy);
    let mut feedback = measured(0, 0.0);
    feedback.positions = vec![0.0, 0.499, -0.501, 0.21];
    assert_eq!(
        dispatch(&mut p, &feedback).values.as_ref(),
        &[0.0, 0.5, -0.5, 0.21]
    );
}

#[test]
fn velocity_hint_modes_preserve_reversals_and_latch_only_when_requested() {
    for (mode, expected) in [
        (VelocityHints::Curve, [0.2, -0.2]),
        (VelocityHints::TowardReference, [0.0, -0.2]),
        (VelocityHints::TowardGoal, [0.0, 0.0]),
    ] {
        let (mut chunk, mut policy) = inputs(&[0.0, 0.4, 0.3, 1.0]);
        chunk.steps[1].velocity_feedforward = Some(vec![0.2, 0.0, 0.0, 0.0].into());
        chunk.steps[2].velocity_feedforward = Some(vec![-0.2, 0.0, 0.0, 0.0].into());
        policy.feedback.as_mut().unwrap().velocity_hints = mode;
        let mut p = start(chunk, policy);
        dispatch(&mut p, &measured(0, 0.0));
        assert_eq!(
            dispatch(&mut p, &measured(100, 1.1))
                .velocity_feedforward
                .unwrap()[0],
            expected[0]
        );
        assert_eq!(
            dispatch(&mut p, &measured(200, 0.6))
                .velocity_feedforward
                .unwrap()[0],
            expected[1]
        );
    }
}

#[test]
fn encoder_window_accepts_stable_positions_with_noisy_velocity_and_retains_evidence() {
    let (chunk, mut policy) = inputs(&[0.0]);
    window_policy(&mut policy);
    let mut p = start(chunk, policy);
    dispatch(&mut p, &measured(0, 0.0));
    for at in [1, 26, 51] {
        let mut feedback = measured(at, 0.0001);
        feedback.velocities[0] = 4.0;
        let result = p.poll(at, &feedback).unwrap();
        assert_eq!(
            result,
            if at == 51 {
                PlaybackPoll::Complete(Completion::Arrived)
            } else {
                PlaybackPoll::Observe
            }
        );
    }
    let evidence = p.settling_evidence().unwrap();
    assert_eq!(evidence.samples, 3);
    assert_eq!(evidence.span_ns, 50);
    assert_eq!(evidence.raw_velocity_peak, 4.0);
}

#[test]
fn a_short_window_still_retains_three_samples_on_a_slower_feedback_cadence() {
    let (chunk, mut policy) = inputs(&[0.0]);
    window_policy(&mut policy);
    let mut p = start(chunk, policy);
    dispatch(&mut p, &measured(0, 0.0));
    assert_eq!(p.poll(1, &measured(1, 0.0)).unwrap(), PlaybackPoll::Observe);
    assert_eq!(
        p.poll(101, &measured(101, 0.0)).unwrap(),
        PlaybackPoll::Observe
    );
    assert_eq!(
        p.poll(201, &measured(201, 0.0)).unwrap(),
        PlaybackPoll::Complete(Completion::Arrived)
    );
    assert_eq!(p.settling_evidence().unwrap().samples, 3);
    assert_eq!(p.settling_evidence().unwrap().span_ns, 200);
}

#[test]
fn a_requested_sample_count_survives_short_window_pruning() {
    let (chunk, mut policy) = inputs(&[0.0]);
    window_policy(&mut policy);
    policy.minimum_settle_samples = 4;
    let mut p = start(chunk, policy);
    dispatch(&mut p, &measured(0, 0.0));
    for at in [1, 101, 201] {
        assert_eq!(
            p.poll(at, &measured(at, 0.0)).unwrap(),
            PlaybackPoll::Observe
        );
    }
    assert_eq!(
        p.poll(301, &measured(301, 0.0)).unwrap(),
        PlaybackPoll::Complete(Completion::Arrived)
    );
    assert_eq!(p.settling_evidence().unwrap().samples, 4);
    assert_eq!(p.settling_evidence().unwrap().span_ns, 300);
}

#[test]
fn encoder_drift_refuses_false_stationarity_despite_zero_velocity() {
    let (chunk, mut policy) = inputs(&[0.0]);
    window_policy(&mut policy);
    policy.allow_settled = true;
    let mut p = start(chunk, policy);
    dispatch(&mut p, &measured(0, 0.0));
    for (at, q) in [(1, 0.0), (26, 0.001), (51, 0.002)] {
        assert_eq!(p.poll(at, &measured(at, q)).unwrap(), PlaybackPoll::Observe);
    }
    assert_eq!(
        p.poll(1_000, &measured(1_000, 0.0)),
        Err(PlaybackError::Deadline)
    );
}

#[test]
fn a_single_goal_crossing_cannot_relabel_a_stationary_miss_as_arrival() {
    let (chunk, mut policy) = inputs(&[0.0]);
    window_policy(&mut policy);
    policy.allow_settled = true;
    let mut p = start(chunk, policy);
    dispatch(&mut p, &measured(0, 0.0));
    p.poll(1, &measured(1, 0.0011)).unwrap();
    p.poll(26, &measured(26, 0.0011)).unwrap();
    assert_eq!(
        p.poll(51, &measured(51, 0.0009)).unwrap(),
        PlaybackPoll::Complete(Completion::Settled)
    );
}

#[test]
fn repeated_feedback_and_declared_gaps_restart_the_full_encoder_window() {
    for repeat in [false, true] {
        let (chunk, mut policy) = inputs(&[0.0]);
        window_policy(&mut policy).max_gap_ns = Some(30);
        let mut p = start(chunk, policy);
        dispatch(&mut p, &measured(0, 0.0));
        p.poll(1, &measured(1, 0.0)).unwrap();
        p.poll(26, &measured(26, 0.0)).unwrap();
        let restart = if repeat {
            p.poll(27, &measured(26, 0.0)).unwrap();
            51
        } else {
            57
        };
        assert_eq!(
            p.poll(restart, &measured(restart, 0.0)).unwrap(),
            PlaybackPoll::Observe
        );
        assert_eq!(p.settling_evidence().unwrap().samples, 1);
        p.poll(restart + 25, &measured(restart + 25, 0.0)).unwrap();
        assert_eq!(
            p.poll(restart + 50, &measured(restart + 50, 0.0)).unwrap(),
            PlaybackPoll::Complete(Completion::Arrived)
        );
    }
}

#[test]
fn qualifying_windows_can_require_multiple_consecutive_post_endpoint_samples() {
    let (chunk, mut policy) = inputs(&[0.0]);
    window_policy(&mut policy).consecutive_samples = 3;
    let mut p = start(chunk, policy);
    dispatch(&mut p, &measured(0, 0.0));
    for at in [1, 26, 51, 76, 101] {
        assert_eq!(
            p.poll(at, &measured(at, 0.0)).unwrap(),
            if at == 101 {
                PlaybackPoll::Complete(Completion::Arrived)
            } else {
                PlaybackPoll::Observe
            }
        );
    }
}

#[test]
fn optional_pre_endpoint_history_survives_ack_but_cannot_complete_before_it() {
    let (chunk, mut policy) = inputs(&[0.0, 0.0]);
    window_policy(&mut policy).during_motion = true;
    policy.allow_settled = true;
    let mut p = start(chunk, policy);
    dispatch(&mut p, &measured(0, 0.0));
    for at in [25, 50, 75] {
        assert_eq!(
            p.poll(at, &measured(at, 0.002)).unwrap(),
            PlaybackPoll::WaitUntil(100)
        );
    }
    dispatch(&mut p, &measured(100, 0.002));
    assert_eq!(
        p.poll(101, &measured(101, 0.002)).unwrap(),
        PlaybackPoll::Complete(Completion::Settled)
    );
}

#[test]
fn feedback_admission_refuses_overlapping_rows_and_invalid_bounds() {
    for invalid in 0..8 {
        let (mut chunk, mut policy) = inputs(&[0.0, 0.1]);
        let spec = policy.feedback.as_mut().unwrap();
        match invalid {
            0 => spec.measured_indices = vec![0],
            1 => spec.measured_indices = vec![3, 3],
            2 => spec.held_groups.push(HeldJointGroup {
                indices: vec![1],
                position_error: vec![f64::NAN],
                period_ns: 100_000_000,
            }),
            3 => {
                window_policy(&mut policy).max_drift = vec![];
            }
            4 => {
                window_policy(&mut policy).max_gap_ns = Some(0);
            }
            5 => {
                window_policy(&mut policy).consecutive_samples = 0;
            }
            6 => chunk.steps[1].values = vec![0.1, 0.6, -0.5, 0.8].into(),
            _ => chunk.steps[0].velocity_feedforward = Some(vec![0.0, 0.2, 0.0, 0.0].into()),
        }
        assert!(
            FiniteReferences::new(chunk, policy).is_err(),
            "case {invalid}"
        );
    }
}
