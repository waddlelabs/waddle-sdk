//! Deterministic finite-reference timing: no wall-clock races or hardware.

use waddle_runtime::playback::{
    Completion, Feedback, FiniteReferences, Playback, PlaybackError, PlaybackPolicy, PlaybackPoll,
};
use waddle_types::{ActionChunk, ProvenanceTag, SourceId, Step};

fn chunk() -> ActionChunk {
    ActionChunk {
        steps: [0.0, 0.1, 0.2]
            .into_iter()
            .enumerate()
            .map(|(i, q)| Step {
                offset_ns: i as i64 * 40_000_000,
                values: vec![q, 0.5].into(),
                velocity_feedforward: None,
                gripper: None,
                part: Some("arm".into()),
            })
            .collect(),
        dims: 2,
        horizon_ns: 80_000_000,
        t_emitted_ns: 0,
        t_obs_ns: 0,
        seq: 1,
        source: SourceId::new("test"),
        provenance: ProvenanceTag::policy(),
    }
}

fn policy() -> PlaybackPolicy {
    PlaybackPolicy {
        indices: vec![0, 1],
        start_tolerance: vec![0.01, 0.01],
        position_tolerance: vec![0.005, 0.005],
        stationary_velocity: vec![0.01, 0.01],
        settle_ns: 20_000_000,
        max_lateness_ns: 10_000_000,
        timeout_ns: 200_000_000,
        allow_settled: false,
    }
}

fn player() -> Playback {
    Playback::new(FiniteReferences::new(chunk(), policy()).unwrap(), 0)
}

fn feedback(at: i64, q: f64) -> Feedback {
    Feedback {
        observed_ns: at,
        positions: vec![q, 0.5],
        velocities: vec![0.0, 0.0],
    }
}

fn dispatch(player: &mut Playback, at: i64, q: f64, index: usize) {
    assert_eq!(
        player.poll(at, &feedback(at, q)).unwrap(),
        PlaybackPoll::Dispatch(index)
    );
    player.acknowledge(index, at).unwrap();
}

#[test]
fn admission_and_last_dispatch_are_not_measured_completion() {
    let mut player = player();
    dispatch(&mut player, 0, 0.0, 0);
    assert_eq!(
        player.poll(20_000_000, &feedback(20_000_000, 0.0)).unwrap(),
        PlaybackPoll::WaitUntil(40_000_000)
    );
    dispatch(&mut player, 40_000_000, 0.0, 1);
    dispatch(&mut player, 80_000_000, 0.1, 2);
    assert_eq!(player.completion(), None);
    // A pre-dispatch sample may never establish arrival.
    assert_eq!(
        player.poll(90_000_000, &feedback(80_000_000, 0.2)).unwrap(),
        PlaybackPoll::Observe
    );
    assert_eq!(
        player.poll(90_000_000, &feedback(90_000_000, 0.2)).unwrap(),
        PlaybackPoll::Observe
    );
    assert_eq!(
        player
            .poll(110_000_000, &feedback(110_000_000, 0.2))
            .unwrap(),
        PlaybackPoll::Complete(Completion::Arrived)
    );
    // A mistaken late receipt or cancellation cannot rewrite the terminal result.
    assert_eq!(
        player.acknowledge(2, 120_000_000),
        Err(PlaybackError::InvalidAcknowledgement)
    );
    player.cancel("late disconnect");
    assert_eq!(
        player
            .poll(210_000_000, &feedback(210_000_000, 0.0))
            .unwrap(),
        PlaybackPoll::Complete(Completion::Arrived)
    );
}

#[test]
fn late_reference_is_refused_without_skipping_or_resending() {
    let mut player = player();
    dispatch(&mut player, 0, 0.0, 0);
    assert_eq!(
        player.poll(60_000_000, &feedback(60_000_000, 0.0)),
        Err(PlaybackError::LateReference {
            index: 1,
            lateness_ns: 20_000_000
        })
    );
    assert_eq!(player.dispatched(), 1);
    assert_eq!(
        player.poll(80_000_000, &feedback(80_000_000, 0.2)),
        Err(PlaybackError::LateReference {
            index: 1,
            lateness_ns: 20_000_000
        })
    );
}

#[test]
fn start_mismatch_refuses_the_entire_buffer() {
    let mut player = player();
    assert_eq!(
        player.poll(0, &feedback(0, 0.1)),
        Err(PlaybackError::StaleStart)
    );
    assert_eq!(player.dispatched(), 0);
}

#[test]
fn cancellation_is_terminal_and_never_releases_a_later_reference() {
    let mut player = player();
    dispatch(&mut player, 0, 0.0, 0);
    player.cancel("connection lost");
    assert_eq!(
        player.poll(40_000_000, &feedback(40_000_000, 0.0)),
        Err(PlaybackError::Cancelled("connection lost".into()))
    );
    assert_eq!(player.dispatched(), 1);
}

#[test]
fn an_unknown_dispatch_stays_pending_instead_of_being_replayed() {
    let mut player = player();
    assert_eq!(
        player.poll(0, &feedback(0, 0.0)).unwrap(),
        PlaybackPoll::Dispatch(0)
    );
    assert_eq!(
        player.poll(1, &feedback(1, 0.0)),
        Err(PlaybackError::DispatchPending)
    );
    assert_eq!(player.dispatched(), 0);
}

#[test]
fn dispatch_receipts_must_match_the_outstanding_reference() {
    let mut unmatched = player();
    assert!(unmatched.acknowledge(0, 0).is_err());
    let mut player = player();
    player.poll(0, &feedback(0, 0.0)).unwrap();
    assert!(player.acknowledge(1, 0).is_err());
}

#[test]
fn stationary_miss_is_distinct_from_arrival_and_opt_in() {
    for allow_settled in [false, true] {
        let mut selected = policy();
        selected.allow_settled = allow_settled;
        let mut player = Playback::new(FiniteReferences::new(chunk(), selected).unwrap(), 0);
        dispatch(&mut player, 0, 0.0, 0);
        dispatch(&mut player, 40_000_000, 0.0, 1);
        dispatch(&mut player, 80_000_000, 0.1, 2);
        player
            .poll(90_000_000, &feedback(90_000_000, 0.18))
            .unwrap();
        let poll = player
            .poll(110_000_000, &feedback(110_000_000, 0.18))
            .unwrap();
        assert_eq!(
            poll,
            if allow_settled {
                PlaybackPoll::Complete(Completion::Settled)
            } else {
                PlaybackPoll::Observe
            }
        );
        if !allow_settled {
            assert_eq!(
                player.poll(200_000_000, &feedback(200_000_000, 0.18)),
                Err(PlaybackError::Deadline)
            );
        }
    }
}

#[test]
fn repeated_feedback_does_not_accumulate_settling_time() {
    let mut player = player();
    dispatch(&mut player, 0, 0.0, 0);
    dispatch(&mut player, 40_000_000, 0.0, 1);
    dispatch(&mut player, 80_000_000, 0.1, 2);
    let sample = feedback(90_000_000, 0.2);
    player.poll(90_000_000, &sample).unwrap();
    assert_eq!(
        player.poll(150_000_000, &sample).unwrap(),
        PlaybackPoll::Observe
    );
    assert_eq!(player.completion(), None);
}

#[test]
fn invalid_dimensions_nan_offsets_and_mixed_parts_are_rejected_before_playback() {
    for change in 0..6 {
        let mut candidate = chunk();
        match change {
            0 => candidate.steps[1].values.pop().map(|_| ()).unwrap(),
            1 => candidate.steps[1].values[0] = f64::NAN,
            2 => candidate.steps[1].offset_ns = 0,
            3 => candidate.steps[1].part = Some("other".into()),
            4 => candidate.steps[1].gripper = Some(0.0),
            _ => candidate.steps[2].velocity_feedforward = Some(vec![1.0, 0.0].into()),
        }
        assert!(FiniteReferences::new(candidate, policy()).is_err());
    }
}

#[test]
fn old_feedback_cannot_admit_the_next_reference() {
    let mut player = player();
    dispatch(&mut player, 0, 0.0, 0);
    assert_eq!(
        player.poll(40_000_000, &feedback(0, 0.0)).unwrap(),
        PlaybackPoll::Observe
    );
    assert_eq!(player.dispatched(), 1);
    assert!(matches!(
        player.poll(60_000_000, &feedback(0, 0.0)),
        Err(PlaybackError::LateReference { .. })
    ));
}

#[test]
fn final_in_tolerance_sample_does_not_relabel_a_settled_miss_as_arrival() {
    let mut selected = policy();
    selected.allow_settled = true;
    let mut player = Playback::new(FiniteReferences::new(chunk(), selected).unwrap(), 0);
    dispatch(&mut player, 0, 0.0, 0);
    dispatch(&mut player, 40_000_000, 0.0, 1);
    dispatch(&mut player, 80_000_000, 0.1, 2);
    player
        .poll(90_000_000, &feedback(90_000_000, 0.18))
        .unwrap();
    assert_eq!(
        player
            .poll(110_000_000, &feedback(110_000_000, 0.2))
            .unwrap(),
        PlaybackPoll::Complete(Completion::Settled)
    );
}

#[test]
fn held_jaw_contact_does_not_become_an_arm_start_or_arrival_test() {
    let mut selected = policy();
    selected.indices = vec![0];
    selected.start_tolerance.truncate(1);
    selected.position_tolerance.truncate(1);
    selected.stationary_velocity.truncate(1);
    let mut player = Playback::new(FiniteReferences::new(chunk(), selected).unwrap(), 0);
    let mut measured = feedback(0, 0.0);
    measured.positions[1] = 0.3;
    assert_eq!(
        player.poll(0, &measured).unwrap(),
        PlaybackPoll::Dispatch(0)
    );
    player.acknowledge(0, 0).unwrap();
    for index in 1..=2 {
        measured.observed_ns = index as i64 * 40_000_000;
        assert_eq!(
            player.poll(measured.observed_ns, &measured).unwrap(),
            PlaybackPoll::Dispatch(index)
        );
        player.acknowledge(index, measured.observed_ns).unwrap();
    }
    measured.positions[0] = 0.2;
    measured.observed_ns = 90_000_000;
    player.poll(measured.observed_ns, &measured).unwrap();
    measured.observed_ns = 110_000_000;
    assert_eq!(
        player.poll(measured.observed_ns, &measured).unwrap(),
        PlaybackPoll::Complete(Completion::Arrived)
    );
}
