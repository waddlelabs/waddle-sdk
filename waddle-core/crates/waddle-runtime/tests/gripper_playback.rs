//! Jaw outcomes use local measured evidence, never buffer exhaustion.

use waddle_runtime::playback::{
    Completion, ContactCondition, Feedback, FiniteReferences, ForceCondition, GripperPolicy,
    MeasuredForce, Playback, PlaybackError, PlaybackPolicy, PlaybackPoll,
};
use waddle_types::{ActionChunk, ProvenanceTag, SourceId, Step};

const PERIOD: i64 = 50_000_000;

fn inputs() -> (ActionChunk, PlaybackPolicy) {
    let steps = [1.0, 0.0]
        .into_iter()
        .enumerate()
        .map(|(i, q)| Step {
            values: vec![0.2, q].into(),
            velocity_feedforward: None,
            offset_ns: i as i64 * PERIOD,
            part: None,
            gripper: None,
        })
        .collect();
    (
        ActionChunk {
            dims: 2,
            steps,
            horizon_ns: PERIOD,
            t_emitted_ns: 0,
            t_obs_ns: 0,
            seq: 1,
            source: SourceId::new("gripper-test"),
            provenance: ProvenanceTag::policy(),
        },
        PlaybackPolicy {
            indices: vec![1],
            start_tolerance: vec![0.001],
            position_tolerance: vec![0.001],
            stationary_velocity: vec![0.01],
            settle_ns: 10_000_000,
            minimum_settle_samples: 1,
            max_lateness_ns: 1,
            timeout_ns: 2_000_000_000,
            allow_settled: false,
            reference_only: false,
            paced_curve: None,
            feedback: None,
            gripper: Some(GripperPolicy {
                period_ns: PERIOD,
                position_error: 0.2,
                tracking_tolerance: 0.01,
                closed_position: 0.0,
                open_position: 1.0,
                closed_m: 0.0,
                open_m: 0.08,
                contact: None,
                force: None,
            }),
        },
    )
}

fn contact(policy: &mut PlaybackPolicy) {
    policy.gripper.as_mut().unwrap().contact = Some(ContactCondition {
        minimum_opening_m: 0.0025,
        minimum_progress_m: 0.002,
        target_shortfall_m: 0.0005,
        commanded_shortfall_m: 0.0005,
        max_drift_m: 0.0002,
        window_ns: 250_000_000,
    });
}

fn force(policy: &mut PlaybackPolicy, target: f64) {
    policy.gripper.as_mut().unwrap().force = Some(ForceCondition {
        target_n: target,
        tolerance_n: 0.5,
        closed_tolerance_m: 0.0005,
    });
}

fn player(chunk: ActionChunk, policy: PlaybackPolicy) -> Playback {
    Playback::new(FiniteReferences::new(chunk, policy).unwrap(), 0)
}

fn measured(at: i64, q: f64, force_n: Option<f64>) -> Feedback {
    Feedback {
        observed_ns: at,
        positions: vec![0.2, q],
        velocities: vec![0.0; 2],
        gripper_force: force_n.map(|force_n| MeasuredForce {
            force_n,
            source: "motor.estimate".into(),
            estimated: true,
        }),
    }
}

fn dispatch(p: &mut Playback, at: i64, q: f64, force_n: Option<f64>) -> Step {
    let index = p.dispatched();
    assert_eq!(
        p.poll(at, &measured(at, q, force_n)).unwrap(),
        PlaybackPoll::Dispatch(index)
    );
    let step = p.reference(index).unwrap().clone();
    p.acknowledge(index, at).unwrap();
    step
}

#[test]
fn jaw_spacing_uses_completed_acknowledgement_and_never_catches_up() {
    let (chunk, policy) = inputs();
    let mut p = player(chunk, policy);
    assert_eq!(
        p.poll(0, &measured(0, 1.0, None)).unwrap(),
        PlaybackPoll::Dispatch(0)
    );
    p.acknowledge_at(0, 10_000_000, 70_000_000).unwrap();
    assert_eq!(
        p.poll(100_000_000, &measured(100_000_000, 1.0, None))
            .unwrap(),
        PlaybackPoll::WaitUntil(120_000_000)
    );
    let step = dispatch(&mut p, 500_000_000, 1.0, None);
    assert_eq!(step.values[0], 0.2);
    assert!((step.values[1] - 0.82).abs() < 1e-12);
    assert_eq!(
        p.poll(510_000_000, &measured(510_000_000, 0.82, None))
            .unwrap(),
        PlaybackPoll::WaitUntil(550_000_000)
    );
}

#[test]
fn position_arrival_needs_three_fresh_stationary_samples_after_nominal_end() {
    let (chunk, mut policy) = inputs();
    policy.gripper.as_mut().unwrap().position_error = 2.0;
    let mut p = player(chunk, policy);
    dispatch(&mut p, 0, 1.0, None);
    assert_eq!(dispatch(&mut p, PERIOD, 1.0, None).values[1], 0.0);
    dispatch(&mut p, 2 * PERIOD, 0.0, None);
    dispatch(&mut p, 3 * PERIOD, 0.0, None);
    assert_eq!(
        p.poll(4 * PERIOD, &measured(4 * PERIOD, 0.0, None))
            .unwrap(),
        PlaybackPoll::Complete(Completion::Arrived)
    );
    assert_eq!(p.gripper_evidence().unwrap().arrival_samples, 3);
}

#[test]
fn closing_progress_and_sustained_demand_qualify_contact_despite_noisy_velocity() {
    let (chunk, mut policy) = inputs();
    contact(&mut policy);
    let mut p = player(chunk, policy);
    dispatch(&mut p, 0, 1.0, None);
    dispatch(&mut p, PERIOD, 1.0, None);
    dispatch(&mut p, 2 * PERIOD, 0.82, None);
    for tick in 3..=8 {
        let at = tick * PERIOD;
        let mut feedback = measured(at, 0.7, None);
        feedback.velocities[1] = -0.5;
        let result = p.poll(at, &feedback).unwrap();
        if tick == 8 {
            assert_eq!(result, PlaybackPoll::Complete(Completion::ContactStall));
        } else {
            let index = p.dispatched();
            assert_eq!(result, PlaybackPoll::Dispatch(index));
            p.acknowledge(index, at).unwrap();
        }
    }
    let evidence = p.gripper_evidence().unwrap();
    assert_eq!(evidence.window_ns, 250_000_000);
    assert_eq!(evidence.drift_m, 0.0);
    assert!(evidence.progress_m > 0.002 && evidence.commanded_shortfall_m.unwrap() > 0.0005);
}

#[test]
fn unexplained_initial_stall_retains_tracking_failure_even_with_contact_requested() {
    let (chunk, mut policy) = inputs();
    contact(&mut policy);
    let mut p = player(chunk, policy);
    dispatch(&mut p, 0, 1.0, None);
    dispatch(&mut p, PERIOD, 1.0, None);
    assert_eq!(
        p.poll(2 * PERIOD, &measured(2 * PERIOD, 1.0, None)),
        Err(PlaybackError::Tracking)
    );
}

#[test]
fn target_only_stall_cannot_become_contact_and_preserves_original_deadline() {
    let (chunk, mut policy) = inputs();
    policy.gripper.as_mut().unwrap().tracking_tolerance = 0.3;
    let mut p = player(chunk, policy);
    dispatch(&mut p, 0, 1.0, None);
    for tick in 1..40 {
        dispatch(&mut p, tick * PERIOD, 1.0, None);
    }
    assert_eq!(
        p.poll(40 * PERIOD, &measured(40 * PERIOD, 1.0, None)),
        Err(PlaybackError::Deadline)
    );
}

#[test]
fn force_pursues_final_closure_and_requires_three_sourced_stationary_samples() {
    let (chunk, mut policy) = inputs();
    force(&mut policy, 5.0);
    let mut p = player(chunk, policy);
    assert_eq!(dispatch(&mut p, 0, 1.0, Some(0.0)).values[1], 0.9);
    dispatch(&mut p, PERIOD, 0.7, Some(4.5));
    dispatch(&mut p, 2 * PERIOD, 0.7, Some(4.5));
    assert_eq!(
        p.poll(3 * PERIOD, &measured(3 * PERIOD, 0.7, Some(4.5)))
            .unwrap(),
        PlaybackPoll::Complete(Completion::ForceReached)
    );
    assert_eq!(p.gripper_evidence().unwrap().force_samples, 3);
}

#[test]
fn reversed_action_mapping_keeps_physical_force_and_empty_jaw_semantics() {
    let (mut chunk, mut policy) = inputs();
    for step in &mut chunk.steps {
        step.values[1] = 1.0 - step.values[1];
    }
    let spec = policy.gripper.as_mut().unwrap();
    spec.closed_position = 1.0;
    spec.open_position = 0.0;
    force(&mut policy, 5.0);
    let mut p = player(chunk.clone(), policy.clone());
    assert!((dispatch(&mut p, 0, 0.0, Some(0.0)).values[1] - 0.1).abs() < 1e-12);
    dispatch(&mut p, PERIOD, 0.3, Some(5.0));
    dispatch(&mut p, 2 * PERIOD, 0.3, Some(5.0));
    assert_eq!(
        p.poll(3 * PERIOD, &measured(3 * PERIOD, 0.3, Some(5.0)))
            .unwrap(),
        PlaybackPoll::Complete(Completion::ForceReached)
    );
    assert!((p.gripper_evidence().unwrap().measured_opening_m - 0.056).abs() < 1e-12);
    let mut empty = player(chunk, policy);
    dispatch(&mut empty, 0, 0.0, Some(0.0));
    assert_eq!(
        empty.poll(PERIOD, &measured(PERIOD, 1.0, Some(8.0))),
        Err(PlaybackError::EmptyGripper)
    );
}

#[test]
fn repeated_feedback_resets_force_count_instead_of_reusing_evidence() {
    let (chunk, mut policy) = inputs();
    force(&mut policy, 5.0);
    let mut p = player(chunk, policy);
    dispatch(&mut p, 0, 1.0, Some(0.0));
    dispatch(&mut p, PERIOD, 0.7, Some(5.0));
    dispatch(&mut p, 2 * PERIOD, 0.7, Some(5.0));
    assert_eq!(
        p.poll(3 * PERIOD, &measured(2 * PERIOD, 0.7, Some(5.0)))
            .unwrap(),
        PlaybackPoll::Observe
    );
    dispatch(&mut p, 4 * PERIOD, 0.7, Some(5.0));
    dispatch(&mut p, 5 * PERIOD, 0.7, Some(5.0));
    assert_eq!(
        p.poll(6 * PERIOD, &measured(6 * PERIOD, 0.7, Some(5.0)))
            .unwrap(),
        PlaybackPoll::Complete(Completion::ForceReached)
    );
}

#[test]
fn force_unavailable_and_closed_stop_win_over_apparent_effort_success() {
    for missing in [false, true] {
        let (chunk, mut policy) = inputs();
        force(&mut policy, 5.0);
        let mut p = player(chunk, policy);
        dispatch(&mut p, 0, 1.0, Some(0.0));
        let q = if missing { 0.7 } else { 0.0 };
        let value = if missing { None } else { Some(9.0) };
        assert_eq!(
            p.poll(PERIOD, &measured(PERIOD, q, value)),
            Err(if missing {
                PlaybackError::ForceUnavailable
            } else {
                PlaybackError::EmptyGripper
            })
        );
        assert_eq!(p.gripper_evidence().unwrap().force_samples, 0);
    }
}

#[test]
fn moving_feedback_resets_force_evidence_instead_of_counting_effort_alone() {
    let (chunk, mut policy) = inputs();
    force(&mut policy, 5.0);
    let mut p = player(chunk, policy);
    dispatch(&mut p, 0, 1.0, Some(0.0));
    dispatch(&mut p, PERIOD, 0.7, Some(5.0));
    let mut moving = measured(2 * PERIOD, 0.7, Some(5.0));
    moving.velocities[1] = 0.1;
    assert_eq!(
        p.poll(2 * PERIOD, &moving).unwrap(),
        PlaybackPoll::Dispatch(2)
    );
    p.acknowledge(2, 2 * PERIOD).unwrap();
    assert_eq!(p.gripper_evidence().unwrap().force_samples, 0);
    dispatch(&mut p, 3 * PERIOD, 0.7, Some(5.0));
    dispatch(&mut p, 4 * PERIOD, 0.7, Some(5.0));
    assert_eq!(
        p.poll(5 * PERIOD, &measured(5 * PERIOD, 0.7, Some(5.0)))
            .unwrap(),
        PlaybackPoll::Complete(Completion::ForceReached)
    );
}

#[test]
fn zero_force_uses_position_completion_and_never_claims_force_reached() {
    let (chunk, mut policy) = inputs();
    force(&mut policy, 0.0);
    policy.gripper.as_mut().unwrap().position_error = 2.0;
    let mut p = player(chunk, policy);
    dispatch(&mut p, 0, 1.0, None);
    dispatch(&mut p, PERIOD, 1.0, None);
    dispatch(&mut p, 2 * PERIOD, 0.0, None);
    dispatch(&mut p, 3 * PERIOD, 0.0, None);
    assert_eq!(
        p.poll(4 * PERIOD, &measured(4 * PERIOD, 0.0, None))
            .unwrap(),
        PlaybackPoll::Complete(Completion::Arrived)
    );
}

#[test]
fn malformed_gripper_declarations_are_refused_before_execution() {
    for bad in 0..8 {
        let (mut chunk, mut policy) = inputs();
        match bad {
            0 => policy.allow_settled = true,
            1 => policy.gripper.as_mut().unwrap().position_error = f64::NAN,
            2 => policy.gripper.as_mut().unwrap().open_position = 0.0,
            3 => policy.gripper.as_mut().unwrap().period_ns = 0,
            4 => {
                force(&mut policy, 5.0);
                contact(&mut policy);
            }
            5 => {
                force(&mut policy, 5.0);
                policy
                    .gripper
                    .as_mut()
                    .unwrap()
                    .force
                    .as_mut()
                    .unwrap()
                    .tolerance_n = 5.0;
            }
            6 => chunk.steps[1].values = vec![0.2, -0.1].into(),
            _ => chunk.steps[1].values = vec![0.3, 0.0].into(),
        }
        assert!(FiniteReferences::new(chunk, policy).is_err(), "case {bad}");
    }
}
