//! Task 15 — ObsSource wiring: a declared tripwire evaluates real
//! observations fed from the customer's own `gate(obs=...)` calls (the gate
//! record stream, tapped on the reducer thread — never `Gate::gate()`'s fast
//! path), and requests its declared verb through dispatch when it fires.

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use waddle_runtime::{ControlRegistry, Session, VerbError};
use waddle_tripwire::{Tripwire, TripwireKind};
use waddle_types::pb::v0 as pb;

fn robot() -> pb::RobotDescription {
    pb::RobotDescription {
        name: "tripwire-obs-bot".into(),
        robot_id: "tripwire-obs-01".into(),
        cell_id: "cell-tripwire-obs".into(),
        action_space: Some(pb::ActionSpace {
            space: Some(pb::action_space::Space::JointPosition(pb::JointPosition {
                joints: vec![pb::JointDescriptor {
                    name: "j0".into(),
                    ..Default::default()
                }],
            })),
            rate_hz: 50.0,
            chunking: None,
            gripper: None,
        }),
        ..Default::default()
    }
}

fn wait_until(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    loop {
        if pred() {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
}

/// The headline regression this task fixes: before it, the tripwire
/// evaluator was wired to an `EmptySource` that always returned `None` — a
/// declared tripwire could never fire no matter what the customer observed.
#[test]
fn violating_obs_from_gate_calls_fires_a_hold_request() {
    let hold_calls = Arc::new(AtomicUsize::new(0));
    let hold_calls2 = hold_calls.clone();
    let registry = ControlRegistry {
        send: Some(Arc::new(
            |_chunk: &waddle_types::ActionChunk| -> Result<(), VerbError> { Ok(()) },
        )),
        hold: Some(Arc::new(move || {
            hold_calls2.fetch_add(1, Ordering::SeqCst);
            Ok(())
        })),
        ..Default::default()
    };

    let session = Session::builder("tripwire-obs")
        .robot(robot())
        .control(registry)
        .tripwires(vec![Tripwire::holds(
            "single-joint-margin",
            TripwireKind::JointLimitMargin {
                margin_rad: 0.05,
                limits: vec![(-1.0, 1.0)],
            },
        )])
        .build()
        .unwrap();

    let mut ep = session.start_episode("watch the joint").unwrap();
    // 0.99 rad is within the 0.05 rad margin of the declared 1.0 rad limit —
    // every `gate(obs=...)` call feeds this straight to the tripwire
    // evaluator through the record-stream `ObsSource` this task wires.
    let violating_obs = [0.99];
    assert!(
        wait_until(
            || {
                let _ = ep.gate(&[0.0], None, Some(&violating_obs));
                hold_calls.load(Ordering::SeqCst) > 0
            },
            Duration::from_secs(5),
        ),
        "a JointLimitMargin tripwire fed violating obs must request HOLD"
    );
    session.shutdown();
}

/// Compliant obs (well inside every declared margin) must never fire —
/// the evaluator is edge-triggered and the source must not fabricate a
/// snapshot the customer never actually observed.
#[test]
fn compliant_obs_never_fires() {
    let hold_calls = Arc::new(AtomicUsize::new(0));
    let hold_calls2 = hold_calls.clone();
    let registry = ControlRegistry {
        send: Some(Arc::new(
            |_chunk: &waddle_types::ActionChunk| -> Result<(), VerbError> { Ok(()) },
        )),
        hold: Some(Arc::new(move || {
            hold_calls2.fetch_add(1, Ordering::SeqCst);
            Ok(())
        })),
        ..Default::default()
    };

    let session = Session::builder("tripwire-obs-compliant")
        .robot(robot())
        .control(registry)
        .tripwires(vec![Tripwire::holds(
            "single-joint-margin",
            TripwireKind::JointLimitMargin {
                margin_rad: 0.05,
                limits: vec![(-1.0, 1.0)],
            },
        )])
        .build()
        .unwrap();

    let mut ep = session.start_episode("watch the joint").unwrap();
    let compliant_obs = [0.0];
    let deadline = Instant::now() + Duration::from_millis(200);
    while Instant::now() < deadline {
        let _ = ep.gate(&[0.0], None, Some(&compliant_obs));
        std::thread::sleep(Duration::from_millis(2));
    }
    assert_eq!(
        hold_calls.load(Ordering::SeqCst),
        0,
        "compliant obs must never fire the tripwire"
    );
    session.shutdown();
}
