//! Worker tests use an advancing fake clock and explicit callback barriers.
//! There is no hardware, remote planner, wall-clock timing assertion or sleep
//! chosen to win a race. The simulated owner still uses the actual native gate.

use super::*;
use crate::playback::{
    Completion, ExecutionSnapshot, Feedback, PlaybackIoError, PlaybackPolicy, ReferenceReceipt,
};
use waddle_gate::{Gate, GateOutput, GatePlan, gate::GateShared, plan::PlanMode};
use waddle_ingest::FakeClock;
use waddle_types::time::Stamp;
use waddle_types::{MonoNs, ProvenanceTag, ReplanPolicy, SourceId, Step};

#[derive(Clone, Default)]
struct TestClock(FakeClock);

impl Clock for TestClock {
    fn stamp_now(&self) -> Stamp {
        self.0.advance(1_000_000);
        self.0.stamp_now()
    }
}

struct Pause {
    entered: Sender<()>,
    release: Receiver<()>,
}

impl Pause {
    fn wait(self) {
        self.entered.send(()).unwrap();
        let _ = self.release.recv();
    }
}

fn pause() -> (Pause, Receiver<()>, Sender<()>) {
    let (entered, rx) = std::sync::mpsc::channel();
    let (tx, release) = std::sync::mpsc::channel();
    (Pause { entered, release }, rx, tx)
}

struct MemoryIo {
    clock: TestClock,
    gate: Mutex<Gate<TestClock>>,
    q: Mutex<f64>,
    events: Arc<Mutex<Vec<&'static str>>>,
    writes: Mutex<Vec<(i64, i64, f64)>>,
    observation_pause: Mutex<Option<Pause>>,
    dispatch_pause: Mutex<Option<Pause>>,
    error: Mutex<Option<PlaybackIoError>>,
    panic: AtomicBool,
    part_hold_supported: AtomicBool,
    part_hold_pause: Mutex<Option<Pause>>,
    part_hold_error: Mutex<Option<PlaybackIoError>>,
}

impl PlaybackIo for MemoryIo {
    fn supports_part_hold(&self) -> bool {
        self.part_hold_supported.load(Ordering::SeqCst)
    }

    fn hold_part(&self, part: &str) -> Result<(), PlaybackIoError> {
        assert_eq!(std::thread::current().name(), Some("waddle-verbs"));
        assert_eq!(part, "left");
        if let Some(pause) = self.part_hold_pause.lock().take() {
            pause.wait();
        }
        self.events.lock().push("part_hold");
        match self.part_hold_error.lock().clone() {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }

    fn observe(&self) -> Result<Feedback, PlaybackIoError> {
        if let Some(pause) = self.observation_pause.lock().take() {
            pause.wait();
        }
        Ok(Feedback {
            observed_ns: self.clock.stamp_now().mono_ns().0,
            positions: vec![*self.q.lock()],
            velocities: vec![0.0],
            gripper_force: None,
        })
    }

    fn dispatch(
        &self,
        reference: &Step,
        measured: &Feedback,
        _force_n: Option<f64>,
    ) -> Result<ReferenceReceipt, PlaybackIoError> {
        assert_eq!(std::thread::current().name(), Some("waddle-verbs"));
        self.events.lock().push("dispatch");
        if let Some(pause) = self.dispatch_pause.lock().take() {
            pause.wait();
        }
        assert!(
            !self.panic.load(Ordering::SeqCst),
            "unknown callback side effect"
        );
        if let Some(error) = self.error.lock().clone() {
            return Err(error);
        }
        let decision = self
            .gate
            .lock()
            .gate(&reference.values, None, Some(&measured.positions));
        match decision {
            GateOutput::Pass { .. } => {
                *self.q.lock() = reference.values[0];
                self.writes.lock().push((
                    self.clock.0.now().0,
                    reference.offset_ns,
                    reference.values[0],
                ));
                self.events.lock().push("write");
                Ok(ReferenceReceipt::Dispatched)
            }
            _ => Ok(ReferenceReceipt::Superseded(format!("{decision:?}"))),
        }
    }
}

struct Harness {
    dispatch: VerbDispatch,
    mirror: Arc<Mirror>,
    io: Arc<MemoryIo>,
    outcomes: Receiver<VerbOutcome>,
    hold_pause: Arc<Mutex<Option<Pause>>>,
    hold_error: Arc<AtomicBool>,
}

impl Harness {
    fn new() -> Self {
        let clock = TestClock::default();
        let (shared, _) = GateShared::new(
            GatePlan::passthrough(MonoNs(0)),
            16,
            0,
            ReplanPolicy::Immediate,
        );
        let (gate, _) = Gate::new(shared, clock.clone(), 32);
        let events = Arc::new(Mutex::new(Vec::new()));
        let io = Arc::new(MemoryIo {
            clock: clock.clone(),
            gate: Mutex::new(gate),
            q: Mutex::new(0.0),
            events: events.clone(),
            writes: Mutex::new(Vec::new()),
            observation_pause: Mutex::new(None),
            dispatch_pause: Mutex::new(None),
            error: Mutex::new(None),
            panic: AtomicBool::new(false),
            part_hold_supported: AtomicBool::new(true),
            part_hold_pause: Mutex::new(None),
            part_hold_error: Mutex::new(None),
        });
        let hold_pause: Arc<Mutex<Option<Pause>>> = Arc::default();
        let hold_error = Arc::new(AtomicBool::new(false));
        let hold = {
            let (events, pause, error) = (events.clone(), hold_pause.clone(), hold_error.clone());
            Arc::new(move || {
                if let Some(pause) = pause.lock().take() {
                    pause.wait();
                }
                events.lock().push("hold");
                if error.load(Ordering::SeqCst) {
                    Err(VerbError::Failed("original Hold failure".into()))
                } else {
                    Ok(())
                }
            })
        };
        let estop = {
            let events = events.clone();
            Arc::new(move || {
                events.lock().push("estop");
                Ok(())
            })
        };
        let home = Arc::new(move || {
            events.lock().push("home");
            Ok(())
        });
        let (tx, outcomes) = std::sync::mpsc::channel();
        let dispatch = VerbDispatch::spawn(
            ControlRegistry {
                hold: Some(hold),
                estop: Some((estop, EstopDecl::default())),
                home: Some(home),
                ..Default::default()
            },
            clock,
            tx,
        );
        let mirror = Mirror::new();
        mirror.update(|s| {
            s.episode_id = Some(EpisodeId::new("episode"));
            s.episode_state = Some(waddle_fsm::Phase::Running);
        });
        Self {
            dispatch,
            mirror,
            io,
            outcomes,
            hold_pause,
            hold_error,
        }
    }

    fn start(&self) -> Result<ExecutionHandle, PlaybackError> {
        self.dispatch.start_playback(
            self.mirror.clone(),
            EpisodeId::new("episode"),
            references(),
            self.io.clone(),
            10_000_000,
            0,
        )
    }

    fn outcome(&self) -> VerbOutcome {
        self.outcomes
            .recv_timeout(Duration::from_secs(5))
            .expect("verb must finish")
    }
}

fn references() -> FiniteReferences {
    references_on(None)
}

fn references_on(part: Option<&str>) -> FiniteReferences {
    references_with_hold(part, false)
}

fn references_with_hold(part: Option<&str>, part_hold: bool) -> FiniteReferences {
    FiniteReferences::new(
        ActionChunk {
            steps: [0.0, 0.1, 0.2]
                .into_iter()
                .enumerate()
                .map(|(i, q)| Step {
                    offset_ns: i as i64 * 40_000_000,
                    values: vec![q].into(),
                    velocity_feedforward: None,
                    gripper: None,
                    part: part.map(Into::into),
                })
                .collect(),
            dims: 1,
            horizon_ns: 80_000_000,
            t_emitted_ns: 0,
            t_obs_ns: 0,
            seq: 1,
            source: SourceId::new("test"),
            provenance: ProvenanceTag::policy(),
        },
        PlaybackPolicy {
            indices: vec![0],
            start_tolerance: vec![0.01],
            position_tolerance: vec![0.005],
            stationary_velocity: vec![0.01],
            settle_ns: 20_000_000,
            minimum_settle_samples: 1,
            max_lateness_ns: 50_000_000,
            timeout_ns: 500_000_000,
            allow_settled: false,
            reference_only: false,
            checkpoint_offsets_ns: vec![],
            allow_curve_updates: false,
            credit_pacing_delay: false,
            part_hold,
            paced_curve: None,
            feedback: None,
            gripper: None,
        },
    )
    .unwrap()
}

fn sibling_io(h: &Harness) -> Arc<MemoryIo> {
    let (shared, _) = GateShared::new(
        GatePlan::passthrough(MonoNs(0)),
        16,
        0,
        ReplanPolicy::Immediate,
    );
    let (gate, _) = Gate::new(shared, h.io.clock.clone(), 32);
    Arc::new(MemoryIo {
        clock: h.io.clock.clone(),
        gate: Mutex::new(gate),
        q: Mutex::new(0.0),
        events: h.io.events.clone(),
        writes: Mutex::new(Vec::new()),
        observation_pause: Mutex::new(None),
        dispatch_pause: Mutex::new(None),
        error: Mutex::new(None),
        panic: AtomicBool::new(false),
        part_hold_supported: AtomicBool::new(true),
        part_hold_pause: Mutex::new(None),
        part_hold_error: Mutex::new(None),
    })
}

fn start_named(
    h: &Harness,
    part: &str,
    io: Arc<MemoryIo>,
) -> Result<ExecutionHandle, PlaybackError> {
    start_named_policy(h, part, io, false)
}

fn start_named_policy(
    h: &Harness,
    part: &str,
    io: Arc<MemoryIo>,
    part_hold: bool,
) -> Result<ExecutionHandle, PlaybackError> {
    h.dispatch.start_playback(
        h.mirror.clone(),
        EpisodeId::new("episode"),
        references_with_hold(Some(part), part_hold),
        io,
        10_000_000,
        0,
    )
}

#[test]
fn disjoint_buffers_share_the_worker_but_reject_overlapping_scopes() {
    let h = Harness::new();
    let right_io = sibling_io(&h);
    let (p, entered, release) = pause();
    *h.io.observation_pause.lock() = Some(p);
    let left = start_named(&h, "left", h.io.clone()).unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    let right = start_named(&h, "right", right_io.clone());
    // Always release the callback even when the old single-slot baseline refuses.
    let duplicate = start_named(&h, "left", h.io.clone());
    let whole_site = h.start();
    release.send(()).unwrap();
    let right = right.unwrap();
    assert!(matches!(duplicate, Err(PlaybackError::Busy)));
    assert!(matches!(whole_site, Err(PlaybackError::Busy)));
    assert_eq!(finished(&left).result, Some(Ok(Completion::Arrived)));
    assert_eq!(finished(&right).result, Some(Ok(Completion::Arrived)));
    assert_eq!(h.io.writes.lock().len(), 3);
    assert_eq!(right_io.writes.lock().len(), 3);
    assert!(h.outcomes.try_recv().is_err());
}

#[test]
fn global_stop_drains_active_and_queued_named_buffers() {
    for stop in [VerbRequest::Hold, VerbRequest::Estop] {
        let h = Harness::new();
        let right_io = sibling_io(&h);
        let (p, entered, release) = pause();
        *h.io.observation_pause.lock() = Some(p);
        let left = start_named(&h, "left", h.io.clone()).unwrap();
        entered.recv_timeout(Duration::from_secs(5)).unwrap();
        let right = start_named(&h, "right", right_io.clone()).unwrap();
        let verb = stop.verb();
        h.dispatch.request(stop);
        release.send(()).unwrap();
        for handle in [&left, &right] {
            let result = finished(handle);
            assert!(matches!(
                result.result,
                Some(Err(ExecutionFailure::Playback(PlaybackError::Cancelled(_))))
            ));
            let cleanup = result.cleanup.unwrap();
            assert_eq!(cleanup.verb, verb);
            assert!(cleanup.result.is_ok());
        }
        assert!(h.io.writes.lock().is_empty());
        assert!(right_io.writes.lock().is_empty());
        assert_eq!(h.outcome().verb, verb);
        assert!(h.outcomes.try_recv().is_err());
    }
}

#[test]
fn failed_part_fences_admission_and_cancels_queued_siblings_before_hold() {
    let h = Harness::new();
    let right_io = sibling_io(&h);
    let error = PlaybackIoError {
        detail: "left reply lost".into(),
        evidence: "left motor".into(),
        outcome_unknown: true,
    };
    *h.io.error.lock() = Some(error.clone());
    let (p, entered, release) = pause();
    *h.io.dispatch_pause.lock() = Some(p);
    let (hold, hold_entered, hold_release) = pause();
    *h.hold_pause.lock() = Some(hold);
    let left = start_named(&h, "left", h.io.clone()).unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    let right = start_named(&h, "right", right_io.clone()).unwrap();
    release.send(()).unwrap();
    hold_entered.recv_timeout(Duration::from_secs(5)).unwrap();
    let third = start_named(&h, "third", sibling_io(&h));
    assert!(left.snapshot().result.is_none());
    assert!(right.snapshot().result.is_none());
    hold_release.send(()).unwrap();
    assert!(matches!(third, Err(PlaybackError::Busy)));
    let left = finished(&left);
    let right = finished(&right);
    assert_eq!(left.result, Some(Err(ExecutionFailure::Dispatch(error))));
    assert!(matches!(
        right.result,
        Some(Err(ExecutionFailure::Playback(PlaybackError::Cancelled(_))))
    ));
    assert!(left.cleanup.unwrap().result.is_ok());
    assert!(right.cleanup.unwrap().result.is_ok());
    assert!(right_io.writes.lock().is_empty());
    assert_eq!(h.outcome().verb, Verb::Hold);
    assert!(h.outcomes.try_recv().is_err());
}

fn finished(handle: &ExecutionHandle) -> ExecutionSnapshot {
    let (tx, rx) = std::sync::mpsc::channel();
    let handle = handle.clone();
    std::thread::spawn(move || {
        let _ = tx.send(handle.wait());
    });
    rx.recv_timeout(Duration::from_secs(5))
        .expect("reference loop must drain")
}

#[test]
fn native_worker_advances_complete_buffer_without_planner_round_trips() {
    let h = Harness::new();
    let handle = h.start().unwrap();
    let result = finished(&handle);
    assert_eq!(result.result, Some(Ok(Completion::Arrived)));
    assert_eq!((result.dispatched, result.references), (3, 3));
    let writes = h.io.writes.lock();
    assert_eq!(
        writes.iter().map(|(_, _, q)| *q).collect::<Vec<_>>(),
        [0.0, 0.1, 0.2]
    );
    let start = writes[0].0;
    for (at, offset, _) in writes.iter().skip(1) {
        assert!(*at - start >= *offset - 3_000_000); // first ordinary gate receipt follows the origin
        assert!(*at - start <= *offset + 50_000_000);
    }
    assert!(result.feedback.unwrap().observed_ns > writes.last().unwrap().0);
    assert!(!h.dispatch.playback_active(&EpisodeId::new("episode"), None));
    assert!(h.outcomes.try_recv().is_err());
}

#[test]
fn feedback_keeps_its_preceding_reference_when_another_dispatch_completes() {
    let h = Harness::new();
    let (p, first_entered, first_release) = pause();
    *h.io.dispatch_pause.lock() = Some(p);
    let handle = h.start().unwrap();
    first_entered.recv_timeout(Duration::from_secs(5)).unwrap();
    let (p, observed, observe_release) = pause();
    *h.io.observation_pause.lock() = Some(p);
    first_release.send(()).unwrap();
    observed.recv_timeout(Duration::from_secs(5)).unwrap();
    let first = handle.snapshot();
    assert_eq!(first.dispatched, 1);
    assert!(first.feedback_reference.is_none()); // first sample preceded any write
    assert!(first.last_reference.is_some());

    let (p, second_entered, second_release) = pause();
    *h.io.dispatch_pause.lock() = Some(p);
    observe_release.send(()).unwrap();
    second_entered.recv_timeout(Duration::from_secs(5)).unwrap();
    let (p, observed, observe_release) = pause();
    *h.io.observation_pause.lock() = Some(p);
    second_release.send(()).unwrap();
    observed.recv_timeout(Duration::from_secs(5)).unwrap();
    let second = handle.snapshot();
    assert_eq!(second.dispatched, 2);
    assert_eq!(second.feedback_reference.as_ref().unwrap().offset_ns, 0);
    assert_eq!(second.feedback.as_ref().unwrap().positions, [0.0]);
    assert_eq!(
        second.last_reference.as_ref().unwrap().offset_ns,
        40_000_000
    );
    assert!(second.feedback.as_ref().unwrap().observed_ns < second.last_acknowledged_ns.unwrap());
    observe_release.send(()).unwrap();
    let result = finished(&handle);
    assert_eq!(result.result, Some(Ok(Completion::Arrived)));
    assert_eq!(result.feedback_reference.unwrap().offset_ns, 80_000_000);
    assert_eq!(result.feedback.unwrap().positions, [0.2]);
}

#[test]
fn delayed_fixed_group_dispatch_cannot_catch_up_faster_than_its_cadence() {
    let h = Harness::new();
    let (blocked, entered, release) = pause();
    *h.io.dispatch_pause.lock() = Some(blocked);
    let handle = h
        .dispatch
        .start_playback(
            h.mirror.clone(),
            EpisodeId::new("episode"),
            references(),
            h.io.clone(),
            10_000_000,
            40_000_000,
        )
        .unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    h.io.clock.0.advance(30_000_000);
    release.send(()).unwrap();
    assert_eq!(finished(&handle).result, Some(Ok(Completion::Arrived)));
    let writes = h.io.writes.lock();
    assert_eq!(writes.len(), 3);
    assert!(writes.windows(2).all(|w| w[1].0 - w[0].0 >= 40_000_000));
}

#[test]
fn slow_declared_feedback_rate_cannot_postpone_the_buffer_deadline() {
    let h = Harness::new();
    let handle = h
        .dispatch
        .start_playback(
            h.mirror.clone(),
            EpisodeId::new("episode"),
            references(),
            h.io.clone(),
            1_000_000_000_000,
            0,
        )
        .unwrap();
    assert_eq!(
        finished(&handle).result,
        Some(Err(ExecutionFailure::Playback(PlaybackError::Deadline)))
    );
    assert_eq!(h.io.writes.lock().len(), 3);
    assert_eq!(h.outcome().verb, Verb::Hold);
}

#[test]
fn cancellation_between_observation_and_dispatch_writes_nothing_and_drains_hold() {
    let h = Harness::new();
    let (p, entered, release) = pause();
    *h.io.observation_pause.lock() = Some(p);
    let handle = h.start().unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(h.start().unwrap_err(), PlaybackError::Busy);
    handle.cancel("disconnect");
    assert!(handle.snapshot().result.is_none());
    release.send(()).unwrap();
    assert_eq!(
        finished(&handle).result,
        Some(Err(ExecutionFailure::Playback(PlaybackError::Cancelled(
            "disconnect".into()
        ))))
    );
    assert!(h.io.writes.lock().is_empty());
    assert_eq!(h.outcome().verb, Verb::Hold);
}

#[test]
fn inflight_reference_finishes_before_stop_and_buffer_never_resumes() {
    for estop in [false, true] {
        let h = Harness::new();
        let (p, entered, release) = pause();
        *h.io.dispatch_pause.lock() = Some(p);
        let handle = h.start().unwrap();
        entered.recv_timeout(Duration::from_secs(5)).unwrap();
        if estop {
            h.dispatch.request(VerbRequest::Estop);
        } else {
            h.dispatch.request_priority_hold();
        }
        assert!(handle.snapshot().result.is_none());
        release.send(()).unwrap();
        let result = finished(&handle);
        assert!(result.result.unwrap().is_err());
        let cleanup = result.cleanup.unwrap();
        assert_eq!(cleanup.verb, if estop { Verb::Estop } else { Verb::Hold });
        assert_eq!(cleanup.result, Ok(()));
        assert_eq!(
            *h.io.events.lock(),
            ["dispatch", "write", if estop { "estop" } else { "hold" }]
        );
        assert_eq!(h.io.writes.lock().len(), 1);
    }
}

#[test]
fn unknown_dispatch_preserves_original_fault_even_when_hold_fails() {
    let h = Harness::new();
    let error = PlaybackIoError {
        detail: "write timed out".into(),
        evidence: "{\"motor\":4}".into(),
        outcome_unknown: true,
    };
    *h.io.error.lock() = Some(error.clone());
    h.hold_error.store(true, Ordering::SeqCst);
    let result = finished(&h.start().unwrap());
    assert_eq!(result.result, Some(Err(ExecutionFailure::Dispatch(error))));
    let cleanup = result.cleanup.unwrap();
    assert_eq!(cleanup.verb, Verb::Hold);
    assert_eq!(
        cleanup.result,
        Err(PlaybackIoError {
            detail: VerbError::Failed("original Hold failure".into()).to_string(),
            evidence: String::new(),
            outcome_unknown: true,
        })
    );
    assert_eq!(*h.io.events.lock(), ["dispatch", "hold"]);
    assert_eq!(
        h.outcome().result,
        Err(VerbError::Failed("original Hold failure".into()))
    );
}

#[test]
fn gate_supersession_does_not_ack_or_override_the_selected_stream() {
    let h = Harness::new();
    h.io.gate.lock().shared().store_plan(GatePlan {
        mode: PlanMode::Bypass {
            provenance: ProvenanceTag::policy(),
        },
        since: MonoNs(0),
    });
    let result = finished(&h.start().unwrap());
    assert!(matches!(
        result.result,
        Some(Err(ExecutionFailure::Superseded(_)))
    ));
    assert_eq!(result.dispatched, 0);
    assert!(result.cleanup.is_none());
    assert!(h.io.writes.lock().is_empty());
    assert!(h.outcomes.try_recv().is_err());
}

#[test]
fn old_terminal_handle_cannot_cancel_a_successor_or_change_its_result() {
    let h = Harness::new();
    let first = h.start().unwrap();
    assert_eq!(finished(&first).result, Some(Ok(Completion::Arrived)));
    *h.io.q.lock() = 0.0;
    let (p, entered, release) = pause();
    *h.io.observation_pause.lock() = Some(p);
    let second = h.start().unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    first.cancel("stale stop");
    h.dispatch.drain_playback(&EpisodeId::new("old episode"));
    release.send(()).unwrap();
    assert_eq!(finished(&second).result, Some(Ok(Completion::Arrived)));
    assert_eq!(first.snapshot().result, Some(Ok(Completion::Arrived)));
    assert!(h.outcomes.try_recv().is_err());
}

#[test]
fn pending_safety_callback_blocks_admission_until_it_finishes() {
    let h = Harness::new();
    let (p, entered, release) = pause();
    *h.hold_pause.lock() = Some(p);
    h.dispatch.request_priority_hold();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    assert_eq!(h.start().unwrap_err(), PlaybackError::Busy);
    release.send(()).unwrap();
    assert_eq!(h.outcome().verb, Verb::Hold);
}

#[test]
fn failed_playback_publishes_cleanup_only_after_the_stop_callback_returns() {
    let h = Harness::new();
    *h.io.error.lock() = Some(PlaybackIoError {
        detail: "original write failure".into(),
        evidence: String::new(),
        outcome_unknown: true,
    });
    let (p, entered, release) = pause();
    *h.hold_pause.lock() = Some(p);
    let handle = h.start().unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    let pending = handle.snapshot();
    assert!(pending.result.is_none());
    assert!(pending.cleanup.is_none());
    assert_eq!(h.start().unwrap_err(), PlaybackError::Busy);
    release.send(()).unwrap();
    let result = finished(&handle);
    assert!(matches!(
        result.result,
        Some(Err(ExecutionFailure::Dispatch(_)))
    ));
    let cleanup = result.cleanup.unwrap();
    assert_eq!(cleanup.verb, Verb::Hold);
    assert_eq!(cleanup.result, Ok(()));
    assert!(cleanup.at.0 > 0);
}

#[test]
fn ordinary_control_verb_cancels_buffer_before_taking_over() {
    let h = Harness::new();
    let (p, entered, release) = pause();
    *h.io.observation_pause.lock() = Some(p);
    let handle = h.start().unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    h.dispatch.request(VerbRequest::Home);
    release.send(()).unwrap();
    assert!(finished(&handle).result.unwrap().is_err());
    assert_eq!(*h.io.events.lock(), ["home"]);
}

#[test]
fn panic_stops_locally_and_an_old_episode_cannot_stop_its_successor() {
    for panic in [false, true] {
        let h = Harness::new();
        h.io.panic.store(panic, Ordering::SeqCst);
        if !panic {
            h.mirror
                .update(|s| s.episode_id = Some(EpisodeId::new("successor")));
        }
        let result = finished(&h.start().unwrap());
        if panic {
            assert_eq!(result.result, Some(Err(ExecutionFailure::Panicked)));
            assert_eq!(h.outcome().verb, Verb::Hold);
        } else {
            assert_eq!(
                result.result,
                Some(Err(ExecutionFailure::Superseded("episode ended".into())))
            );
            assert!(h.outcomes.try_recv().is_err());
        }
        assert!(h.io.writes.lock().is_empty());
    }
}

#[test]
fn cancellation_during_episode_replacement_does_not_hold_successor() {
    let h = Harness::new();
    let (p, entered, release) = pause();
    *h.io.observation_pause.lock() = Some(p);
    let handle = h.start().unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    h.mirror
        .update(|s| s.episode_id = Some(EpisodeId::new("successor")));
    handle.cancel("old episode closed");
    release.send(()).unwrap();
    assert_eq!(
        finished(&handle).result,
        Some(Err(ExecutionFailure::Superseded("episode ended".into())))
    );
    assert!(h.io.writes.lock().is_empty());
    assert!(h.outcomes.try_recv().is_err());
}

#[test]
fn concurrent_stop_cannot_erase_original_unknown_write_failure() {
    let h = Harness::new();
    let (p, entered, release) = pause();
    *h.io.dispatch_pause.lock() = Some(p);
    let error = PlaybackIoError {
        detail: "original write failure".into(),
        evidence: "original evidence".into(),
        outcome_unknown: true,
    };
    *h.io.error.lock() = Some(error.clone());
    let handle = h.start().unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    h.dispatch.request(VerbRequest::Estop);
    release.send(()).unwrap();
    assert_eq!(
        finished(&handle).result,
        Some(Err(ExecutionFailure::Dispatch(error)))
    );
    assert_eq!(*h.io.events.lock(), ["dispatch", "estop"]);
}

#[test]
fn shutdown_joins_inflight_dispatch_and_requests_hold_before_completing() {
    let h = Harness::new();
    let (p, entered, release) = pause();
    *h.io.dispatch_pause.lock() = Some(p);
    let handle = h.start().unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    h.dispatch.stop();
    assert!(handle.snapshot().result.is_none());
    release.send(()).unwrap();
    h.dispatch.shutdown();
    assert!(handle.snapshot().result.unwrap().is_err());
    assert_eq!(*h.io.events.lock(), ["dispatch", "write", "hold"]);
}

#[test]
fn shutdown_does_not_replace_a_pending_estop_with_hold() {
    let h = Harness::new();
    let (p, entered, release) = pause();
    *h.io.dispatch_pause.lock() = Some(p);
    let handle = h.start().unwrap();
    entered.recv_timeout(Duration::from_secs(5)).unwrap();
    h.dispatch.request(VerbRequest::Estop);
    h.dispatch.stop();
    release.send(()).unwrap();
    h.dispatch.shutdown();
    assert!(handle.snapshot().result.unwrap().is_err());
    assert_eq!(*h.io.events.lock(), ["dispatch", "write", "estop"]);
}

#[test]
fn scoped_cancellation_or_failure_holds_only_its_part_and_retains_unknown_cleanup() {
    for fail_motion in [false, true] {
        for fail_cleanup in [false, true] {
            let h = Harness::new();
            let right_io = sibling_io(&h);
            let original = PlaybackIoError {
                detail: "original left write failure".into(),
                evidence: "left controller evidence".into(),
                outcome_unknown: true,
            };
            let cleanup_error = PlaybackIoError {
                detail: "left Hold reply lost".into(),
                evidence: "left Hold evidence".into(),
                outcome_unknown: true,
            };
            if fail_motion {
                *h.io.error.lock() = Some(original.clone());
            }
            if fail_cleanup {
                *h.io.part_hold_error.lock() = Some(cleanup_error.clone());
            }
            let (p, entered, release) = pause();
            *h.io.dispatch_pause.lock() = Some(p);
            let left = start_named_policy(&h, "left", h.io.clone(), true).unwrap();
            entered.recv_timeout(Duration::from_secs(5)).unwrap();
            let right = start_named_policy(&h, "right", right_io.clone(), true).unwrap();
            if !fail_motion {
                left.cancel("left only");
            }
            release.send(()).unwrap();
            let result = finished(&left);
            if fail_motion {
                assert_eq!(
                    result.result,
                    Some(Err(ExecutionFailure::Dispatch(original)))
                );
            } else {
                assert!(matches!(
                    result.result,
                    Some(Err(ExecutionFailure::Playback(PlaybackError::Cancelled(_))))
                ));
            }
            let cleanup = result.cleanup.unwrap();
            assert_eq!(cleanup.part.as_deref(), Some("left"));
            assert_eq!(
                cleanup.result,
                if fail_cleanup {
                    Err(cleanup_error)
                } else {
                    Ok(())
                }
            );
            let right = finished(&right);
            assert_eq!(right.result, Some(Ok(Completion::Arrived)));
            assert!(right.cleanup.is_none());
            assert_eq!(right_io.writes.lock().len(), 3);
            assert!(h.outcomes.try_recv().is_err());
        }
    }
}

#[test]
fn global_stop_during_part_hold_drains_both_with_global_receipts() {
    for stop in [VerbRequest::Hold, VerbRequest::Estop] {
        let h = Harness::new();
        let right_io = sibling_io(&h);
        let original = PlaybackIoError {
            detail: "left write lost".into(),
            evidence: "original".into(),
            outcome_unknown: true,
        };
        *h.io.error.lock() = Some(original.clone());
        let (p, entered, release) = pause();
        *h.io.part_hold_pause.lock() = Some(p);
        let left = start_named_policy(&h, "left", h.io.clone(), true).unwrap();
        entered.recv_timeout(Duration::from_secs(5)).unwrap();
        let right = start_named_policy(&h, "right", right_io.clone(), true).unwrap();
        let verb = stop.verb();
        h.dispatch.request(stop);
        assert!(left.snapshot().result.is_none());
        release.send(()).unwrap();
        let left = finished(&left);
        assert_eq!(left.result, Some(Err(ExecutionFailure::Dispatch(original))));
        let right = finished(&right);
        assert!(right.result.as_ref().unwrap().is_err());
        for result in [left, right] {
            let cleanup = result.cleanup.unwrap();
            assert_eq!(cleanup.verb, verb);
            assert_eq!(cleanup.part, None);
            assert_eq!(cleanup.result, Ok(()));
        }
        assert!(right_io.writes.lock().is_empty());
        assert_eq!(h.outcome().verb, verb);
    }
}

#[test]
fn scoped_hold_requires_supported_io_before_admission() {
    let h = Harness::new();
    h.io.part_hold_supported.store(false, Ordering::SeqCst);
    assert!(matches!(
        start_named_policy(&h, "left", h.io.clone(), true),
        Err(PlaybackError::Invalid(_))
    ));
    assert!(h.io.events.lock().is_empty());
    assert_eq!(
        finished(&start_named(&h, "left", h.io.clone()).unwrap()).result,
        Some(Ok(Completion::Arrived))
    );
}

#[test]
fn delayed_cancelled_admission_cannot_override_completed_control_takeover() {
    for part_hold in [false, true] {
        let h = Harness::new();
        let references = references_with_hold(Some("left"), part_hold);
        // Hold the worker message after native admission, reproducing a control
        // request overtaking delivery without relying on a wall-clock race.
        let handle = h
            .dispatch
            .playback
            .register(
                EpisodeId::new("episode"),
                Some("left".into()),
                references.len(),
                || false,
            )
            .unwrap();
        let task = ExecutionTask::new(
            h.mirror.clone(),
            EpisodeId::new("episode"),
            references,
            h.io.clone(),
            &handle,
            10_000_000,
            0,
        );
        handle.cancel("caller cancelled before worker admission");
        h.dispatch.request(VerbRequest::Home);
        assert_eq!(h.outcome().verb, Verb::Home);
        h.dispatch.tx.send(Work::Playback(Box::new(task))).unwrap();
        let result = finished(&handle);
        assert!(matches!(
            result.result,
            Some(Err(ExecutionFailure::Playback(PlaybackError::Cancelled(_))))
        ));
        assert!(
            result.cleanup.is_none(),
            "Home takeover is not a Hold receipt"
        );
        assert_eq!(*h.io.events.lock(), ["home"]);
        assert!(h.io.writes.lock().is_empty());
    }
}
