//! Finite local execution on the existing serialized control thread.

use std::collections::HashMap;
use std::panic::AssertUnwindSafe;
use std::sync::Arc;
use std::time::Duration;

use parking_lot::{Condvar, Mutex};
use waddle_types::time::Clock;
use waddle_types::{EpisodeId, MonoNs, Step, Verb};

use super::{
    CheckpointEvidence, Completion, CurveProgress, CurveUpdate, Feedback, FiniteReferences,
    GripperEvidence, Playback, PlaybackError, PlaybackPoll, PlaybackWatchdog, SettlingEvidence,
};
use crate::mirror::Mirror;
use crate::verbs::VerbOutcome;

/// The original owner failure and optional opaque structured evidence. The
/// runtime neither interprets nor replaces that evidence. Unknown writes are
/// never retried, and a failed HOLD cannot erase the original dispatch failure.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PlaybackIoError {
    pub detail: String,
    pub evidence: String,
    pub outcome_unknown: bool,
}

/// A dispatch receipt, never a measured motion outcome.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReferenceReceipt {
    Dispatched,
    /// Local session-clock start of the actual owner submission, for pacing.
    DispatchedAt(i64),
    /// The ordinary gate selected another stream, Hold or Noop. The adapter
    /// applies that decision normally; this buffer cannot continue afterward.
    Superseded(String),
}

/// Local device I/O, invoked only on the session's serialized control thread.
/// `dispatch` MUST use the existing episode gate and owner envelope, preserving
/// their original result. It must not send directly to a device, skip the gate,
/// wait on a remote planner, or wait for a verb on this same control thread.
pub trait PlaybackIo: Send + Sync + 'static {
    fn supports_part_hold(&self) -> bool {
        false
    }

    /// The same local owner's existing per-part Hold, never a planner request.
    fn hold_part(&self, _part: &str) -> Result<(), PlaybackIoError> {
        Err(PlaybackIoError {
            detail: "part Hold is unsupported".into(),
            evidence: String::new(),
            outcome_unknown: false,
        })
    }

    fn observe(&self) -> Result<Feedback, PlaybackIoError>;
    fn dispatch(
        &self,
        reference: &Step,
        measured: &Feedback,
        force_n: Option<f64>,
    ) -> Result<ReferenceReceipt, PlaybackIoError>;
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExecutionFailure {
    Playback(PlaybackError),
    Observation(PlaybackIoError),
    Dispatch(PlaybackIoError),
    Superseded(String),
    /// A callback panic can leave its physical side effects unknown.
    Panicked,
}

pub type ExecutionResult = Result<Completion, ExecutionFailure>;

/// Completed local cleanup, separate from the original motion failure.
/// `part == None` denotes a site-wide control callback. An acknowledgement is
/// not a measured physical stop. Unknown owner evidence is retained verbatim.
#[derive(Clone, Debug)]
pub struct PlaybackCleanup {
    pub verb: Verb,
    pub part: Option<String>,
    pub result: Result<(), PlaybackIoError>,
    pub latency_ns: i64,
    pub at: MonoNs,
}

impl From<&VerbOutcome> for PlaybackCleanup {
    fn from(outcome: &VerbOutcome) -> Self {
        Self {
            verb: outcome.verb,
            part: None,
            result: outcome
                .result
                .as_ref()
                .copied()
                .map_err(|error| PlaybackIoError {
                    detail: error.to_string(),
                    evidence: String::new(),
                    outcome_unknown: true,
                }),
            latency_ns: outcome.latency_ns,
            at: outcome.at,
        }
    }
}

/// Latest suffix request. Acceptance schedules a future curve; it does not
/// prove that its handoff was acknowledged or that the device arrived.
#[derive(Clone, Debug)]
pub struct CurveUpdateReceipt {
    pub request_id: u64,
    /// Set only for a withdrawal, identifying the accepted suffix request.
    pub withdraws_request_id: Option<u64>,
    /// None until the local worker processes the request.
    pub result: Option<Result<(), PlaybackError>>,
}

/// Renewal is confirmed only when the local worker accepts its feedback stamp.
#[derive(Clone, Debug)]
pub struct WatchdogReceipt {
    pub observed_ns: i64,
    pub result: Option<Result<i64, PlaybackError>>,
}

/// An execution snapshot. A terminal result means the reference loop has ended;
/// it is not proof that a requested safety verb physically stopped the device.
#[derive(Clone, Debug)]
pub struct ExecutionSnapshot {
    pub id: String,
    pub episode_id: EpisodeId,
    pub dispatched: usize,
    pub references: usize,
    pub feedback: Option<Feedback>,
    /// Last acknowledged reference before acquiring `feedback`. Later dispatches
    /// may advance `last_reference` without changing this causal pair.
    pub feedback_reference: Option<Step>,
    pub curve_progress: Option<CurveProgress>,
    pub curve_update: Option<CurveUpdateReceipt>,
    pub watchdog: Option<PlaybackWatchdog>,
    pub watchdog_refresh: Option<WatchdogReceipt>,
    pub settling: Option<SettlingEvidence>,
    pub gripper: Option<GripperEvidence>,
    pub checkpoints: Vec<CheckpointEvidence>,
    pub last_reference: Option<Step>,
    /// Session-clock completion of the last acknowledged owner submission.
    /// This is neither a cloud receipt time nor a measured arrival timestamp.
    pub last_acknowledged_ns: Option<i64>,
    /// The completed stop callback, separate from the original motion result.
    /// Callback success alone does not establish measured physical stopping.
    pub cleanup: Option<PlaybackCleanup>,
    pub result: Option<ExecutionResult>,
}

#[derive(Debug)]
struct State {
    snapshot: ExecutionSnapshot,
    cancelled: Option<String>,
    cancelled_by_control: bool,
    curve_update: Option<CurveRequest>,
}

#[derive(Debug)]
enum CurveRequest {
    Update(CurveUpdate),
    Withdraw {
        request_id: u64,
        expected_revision: u64,
    },
}

#[derive(Debug)]
struct Control {
    state: Mutex<State>,
    changed: Condvar,
}

impl Control {
    fn mark_cancelled(&self, reason: &str, by_control: bool) {
        let mut state = self.state.lock();
        if state.snapshot.result.is_none() {
            state.cancelled.get_or_insert_with(|| reason.to_owned());
            state.cancelled_by_control |= by_control;
        }
    }

    fn finish(&self, result: ExecutionResult) -> bool {
        let mut state = self.state.lock();
        if state.snapshot.result.is_none() {
            // Let the worker drain cancellation through its ordinary Hold
            // before publishing a terminal result. A racing arrival is not it.
            if state.cancelled.is_some() && result.is_ok() {
                return false;
            }
            state.snapshot.result = Some(result);
            state.curve_update = None;
            if let Some(receipt) = state
                .snapshot
                .watchdog_refresh
                .as_mut()
                .filter(|r| r.result.is_none())
            {
                receipt.result = Some(Err(PlaybackError::Cancelled(
                    "execution ended before watchdog renewal".into(),
                )));
            }
            if let Some(receipt) = state
                .snapshot
                .curve_update
                .as_mut()
                .filter(|r| r.result.is_none())
            {
                receipt.result = Some(Err(PlaybackError::Cancelled(
                    "execution ended before curve update".into(),
                )));
            }
            self.changed.notify_all();
        }
        true
    }
}

/// Retained local identity/status; losing a remote response never resubmits work.
#[derive(Clone, Debug)]
pub struct ExecutionHandle {
    control: Arc<Control>,
}

impl ExecutionHandle {
    #[must_use]
    pub fn snapshot(&self) -> ExecutionSnapshot {
        self.control.state.lock().snapshot.clone()
    }

    /// Cancel future references; the worker applies the registered Hold. A
    /// terminal handle cannot stop a later execution or rewrite its own result.
    pub fn cancel(&self, reason: &str) {
        self.control.mark_cancelled(reason, false);
    }

    /// Queue a fresh session-clock feedback stamp. Repeating the same stamp is
    /// idempotent and cannot extend the deadline. No timeout or authority changes.
    pub fn refresh_watchdog(&self, observed_ns: i64) -> Result<(), PlaybackError> {
        let mut state = self.control.state.lock();
        if state.snapshot.result.is_some() || state.cancelled.is_some() {
            return Err(PlaybackError::Invalid("execution is closing or terminal"));
        }
        if state.snapshot.watchdog.is_none() {
            return Err(PlaybackError::Invalid("execution has no watchdog"));
        }
        if let Some(previous) = &state.snapshot.watchdog_refresh {
            if previous.observed_ns == observed_ns {
                return Ok(());
            }
            if previous.result.is_none() {
                return Err(PlaybackError::Busy);
            }
        }
        state.snapshot.watchdog_refresh = Some(WatchdogReceipt {
            observed_ns,
            result: None,
        });
        Ok(())
    }

    /// Enqueue one suffix for validation by the existing local worker. Returns
    /// promptly even while device I/O is in flight. Poll `curve_update` for the
    /// result and `curve_progress.revision` for the acknowledged handoff. A lost
    /// response never authorizes retry; reconcile the retained execution first.
    pub fn update_curve(&self, update: CurveUpdate) -> Result<u64, PlaybackError> {
        self.queue_curve(CurveRequest::Update(update))
    }

    /// Queue withdrawal of one accepted suffix. Only the serialized worker can
    /// confirm it has not crossed the handoff; queueing is not withdrawal.
    /// Request identity prevents a delayed withdrawal from removing a replacement.
    pub fn withdraw_curve(
        &self,
        request_id: u64,
        expected_revision: u64,
    ) -> Result<u64, PlaybackError> {
        self.queue_curve(CurveRequest::Withdraw {
            request_id,
            expected_revision,
        })
    }

    fn queue_curve(&self, request: CurveRequest) -> Result<u64, PlaybackError> {
        let mut state = self.control.state.lock();
        if state.snapshot.result.is_some() || state.cancelled.is_some() {
            return Err(PlaybackError::Invalid("execution is closing or terminal"));
        }
        if state
            .snapshot
            .curve_update
            .as_ref()
            .is_some_and(|r| r.result.is_none())
        {
            return Err(PlaybackError::Busy);
        }
        let id = state
            .snapshot
            .curve_update
            .as_ref()
            .map_or(Some(1), |r| r.request_id.checked_add(1))
            .ok_or(PlaybackError::Invalid("curve request sequence exhausted"))?;
        let withdraws_request_id = match &request {
            CurveRequest::Update(_) => None,
            CurveRequest::Withdraw { request_id, .. } => Some(*request_id),
        };
        state.curve_update = Some(request);
        state.snapshot.curve_update = Some(CurveUpdateReceipt {
            request_id: id,
            withdraws_request_id,
            result: None,
        });
        Ok(id)
    }

    /// Wait for the local reference loop to finish, without a polling thread.
    /// Bindings must release their language lock while calling this method.
    pub fn wait(&self) -> ExecutionSnapshot {
        let mut state = self.control.state.lock();
        while state.snapshot.result.is_none() {
            self.control.changed.wait(&mut state);
        }
        state.snapshot.clone()
    }
}

#[derive(Debug, Default)]
pub(crate) struct ExecutionSlot {
    current: Mutex<HashMap<Option<Vec<String>>, Arc<Control>>>,
}

impl ExecutionSlot {
    pub(crate) fn register(
        &self,
        episode_id: EpisodeId,
        scope: Option<Vec<String>>,
        references: usize,
        unavailable: impl FnOnce() -> bool,
    ) -> Result<ExecutionHandle, PlaybackError> {
        let mut current = self.current.lock();
        current.retain(|_, c| c.state.lock().snapshot.result.is_none());
        if unavailable()
            || current
                .keys()
                .any(|part| overlaps(part.as_deref(), scope.as_deref()))
        {
            return Err(PlaybackError::Busy);
        }
        let control = Arc::new(Control {
            state: Mutex::new(State {
                snapshot: ExecutionSnapshot {
                    id: uuid::Uuid::new_v4().to_string(),
                    episode_id,
                    dispatched: 0,
                    references,
                    feedback: None,
                    feedback_reference: None,
                    curve_progress: None,
                    curve_update: None,
                    watchdog: None,
                    watchdog_refresh: None,
                    settling: None,
                    gripper: None,
                    checkpoints: Vec::new(),
                    last_reference: None,
                    last_acknowledged_ns: None,
                    cleanup: None,
                    result: None,
                },
                cancelled: None,
                cancelled_by_control: false,
                curve_update: None,
            }),
            changed: Condvar::new(),
        });
        current.insert(scope, control.clone());
        Ok(ExecutionHandle { control })
    }

    pub(crate) fn interrupt(&self, reason: &str, request: impl FnOnce()) {
        let current = self.current.lock();
        // Make the pending-control fence visible before marking cancellations.
        // Completed callbacks acquire this same registry lock for their receipts.
        request();
        for control in current.values() {
            control.mark_cancelled(reason, true);
        }
    }

    pub(crate) fn record_cleanup(&self, outcome: &VerbOutcome) {
        if matches!(
            outcome.verb,
            waddle_types::Verb::Hold | waddle_types::Verb::Estop
        ) {
            for control in self.current.lock().values() {
                let mut state = control.state.lock();
                if state.snapshot.result.is_none() {
                    state.snapshot.cleanup = Some(outcome.into());
                }
            }
        }
    }

    pub(crate) fn any_active(&self) -> bool {
        self.current
            .lock()
            .values()
            .any(|c| c.state.lock().snapshot.result.is_none())
    }

    pub(crate) fn active(&self, episode: &EpisodeId, scope: Option<&str>) -> bool {
        self.current.lock().iter().any(|(part, control)| {
            let state = control.state.lock();
            part.as_ref()
                .is_none_or(|parts| scope.is_none_or(|name| parts.iter().any(|p| p == name)))
                && &state.snapshot.episode_id == episode
                && state.snapshot.result.is_none()
        })
    }

    pub(crate) fn cancel_episode(&self, episode: &EpisodeId) -> Vec<ExecutionHandle> {
        self.current
            .lock()
            .values()
            .filter_map(|control| {
                if &control.state.lock().snapshot.episode_id != episode {
                    return None;
                }
                let handle = ExecutionHandle {
                    control: control.clone(),
                };
                handle.cancel("episode ended");
                Some(handle)
            })
            .collect()
    }
}

fn overlaps(left: Option<&[String]>, right: Option<&[String]>) -> bool {
    match (left, right) {
        (Some(left), Some(right)) => left.iter().any(|part| right.contains(part)),
        _ => true,
    }
}

/// Owned by `waddle-verbs`, not a second device/authority thread.
pub(crate) struct ExecutionTask {
    mirror: Arc<Mirror>,
    episode: EpisodeId,
    references: Option<FiniteReferences>,
    player: Option<Playback>,
    io: Arc<dyn PlaybackIo>,
    control: Arc<Control>,
    next_poll_ns: i64,
    sample_period_ns: i64,
    dispatch_period_ns: i64,
    failure: Option<ExecutionResult>,
    pending_update_id: Option<u64>,
    hold_part: Option<String>,
}

impl ExecutionTask {
    pub(crate) fn new(
        mirror: Arc<Mirror>,
        episode: EpisodeId,
        references: FiniteReferences,
        io: Arc<dyn PlaybackIo>,
        handle: &ExecutionHandle,
        sample_period_ns: i64,
        dispatch_period_ns: i64,
    ) -> Self {
        handle.control.state.lock().snapshot.watchdog = references.policy.watchdog;
        Self {
            mirror,
            episode,
            hold_part: references
                .policy
                .part_hold
                .then(|| references.chunk.steps[0].part.as_ref().unwrap().to_string()),
            references: Some(references),
            player: None,
            io,
            control: handle.control.clone(),
            next_poll_ns: 0,
            sample_period_ns,
            dispatch_period_ns,
            failure: None,
            pending_update_id: None,
        }
    }

    pub(crate) fn wait_duration(&self, now_ns: i64) -> Duration {
        Duration::from_nanos(self.next_poll_ns.saturating_sub(now_ns).clamp(0, 5_000_000) as u64)
    }

    pub(crate) fn cancelled_result(&self) -> Option<ExecutionResult> {
        self.control.state.lock().cancelled.as_ref().map(|reason| {
            Err(ExecutionFailure::Playback(PlaybackError::Cancelled(
                reason.clone(),
            )))
        })
    }

    pub(crate) fn terminal_failure(&self) -> Option<ExecutionResult> {
        self.failure.clone()
    }

    pub(crate) fn scope_ended(&self) -> bool {
        let status = self.mirror.read();
        status.shutdown
            || status.episode_id.as_ref() != Some(&self.episode)
            || matches!(
                status.episode_state,
                Some(waddle_fsm::Phase::Terminal(_) | waddle_fsm::Phase::PostReset)
            )
    }

    pub(crate) fn finish(&self, result: ExecutionResult) -> bool {
        self.control.finish(result)
    }

    pub(crate) fn record_cleanup(&self, outcome: &VerbOutcome) {
        if matches!(
            outcome.verb,
            waddle_types::Verb::Hold | waddle_types::Verb::Estop
        ) {
            self.control.state.lock().snapshot.cleanup = Some(outcome.into());
        }
    }

    pub(crate) fn cancelled_by_control(&self) -> bool {
        self.control.state.lock().cancelled_by_control
    }

    pub(crate) fn hold_part<C: Clock>(&self, clock: &C) -> bool {
        let Some(part) = &self.hold_part else {
            return false;
        };
        let started = clock.stamp_now().mono_ns();
        let result = std::panic::catch_unwind(AssertUnwindSafe(|| self.io.hold_part(part)))
            .unwrap_or_else(|_| {
                Err(PlaybackIoError {
                    detail: "part Hold callback panicked".into(),
                    evidence: String::new(),
                    outcome_unknown: true,
                })
            });
        let at = clock.stamp_now().mono_ns();
        self.control.state.lock().snapshot.cleanup = Some(PlaybackCleanup {
            verb: Verb::Hold,
            part: Some(part.clone()),
            result,
            latency_ns: at.0 - started.0,
            at,
        });
        true
    }

    /// `Some` is terminal. Callback panics retain uncertainty and never retry.
    pub(crate) fn tick<C: Clock>(&mut self, clock: &C) -> Option<ExecutionResult> {
        if let Some(failure) = self.failure.as_ref() {
            return Some(failure.clone());
        }
        let result = match std::panic::catch_unwind(AssertUnwindSafe(|| self.tick_inner(clock))) {
            Ok(result) => result,
            Err(_) => Some(Err(ExecutionFailure::Panicked)),
        };
        if result.as_ref().is_some_and(Result::is_err) {
            self.failure.clone_from(&result);
        }
        result
    }

    pub(crate) fn check_watchdog<C: Clock>(&self, clock: &C) -> Option<ExecutionResult> {
        let mut state = self.control.state.lock();
        let watchdog = state.snapshot.watchdog?;
        let now = clock.stamp_now().mono_ns().0;
        // Check the incumbent first: a queued or delayed renewal cannot revive
        // an expired buffer, including after an in-flight device callback.
        let failure = if watchdog.observed_ns > now {
            Some(PlaybackError::Invalid("watchdog feedback is in the future"))
        } else if now >= watchdog.deadline_ns() {
            Some(PlaybackError::WatchdogExpired)
        } else {
            None
        };
        if let Some(error) = failure {
            if let Some(receipt) = state
                .snapshot
                .watchdog_refresh
                .as_mut()
                .filter(|r| r.result.is_none())
            {
                receipt.result = Some(Err(error.clone()));
            }
            return Some(Err(ExecutionFailure::Playback(error)));
        }
        let receipt = state
            .snapshot
            .watchdog_refresh
            .as_mut()
            .filter(|r| r.result.is_none())?;
        let next = PlaybackWatchdog {
            observed_ns: receipt.observed_ns,
            ..watchdog
        };
        if next.observed_ns < watchdog.observed_ns
            || next.observed_ns > now
            || next.observed_ns.checked_add(next.timeout_ns).is_none()
        {
            receipt.result = Some(Err(PlaybackError::Invalid(
                "watchdog requires current session-clock feedback",
            )));
        } else {
            receipt.result = Some(Ok(next.deadline_ns()));
            state.snapshot.watchdog = Some(next);
        }
        None
    }

    fn tick_inner<C: Clock>(&mut self, clock: &C) -> Option<ExecutionResult> {
        if self.scope_ended() {
            return Some(Err(ExecutionFailure::Superseded("episode ended".into())));
        }
        if let Some(result) = self.cancelled_result() {
            return Some(result);
        }
        if let Some(result) = self.check_watchdog(clock) {
            return Some(result);
        }
        if clock.stamp_now().mono_ns().0 < self.next_poll_ns {
            return None;
        }
        let measured = match self.io.observe() {
            Ok(measured) => measured,
            Err(error) => return Some(Err(ExecutionFailure::Observation(error))),
        };
        if self.scope_ended() {
            return Some(Err(ExecutionFailure::Superseded("episode ended".into())));
        }
        if let Some(result) = self.cancelled_result() {
            return Some(result);
        }
        if let Some(result) = self.check_watchdog(clock) {
            return Some(result);
        }
        let now = clock.stamp_now().mono_ns().0;
        let player = self.player.get_or_insert_with(|| {
            Playback::new(
                self.references.take().expect("one admitted buffer"),
                measured.observed_ns,
            )
        });
        let update = self.control.state.lock().curve_update.take();
        if let Some(update) = update {
            let mut state = self.control.state.lock();
            let receipt = state
                .snapshot
                .curve_update
                .as_mut()
                .expect("queued receipt");
            let result = match update {
                CurveRequest::Update(update) => player.update_curve(update).inspect(|()| {
                    self.pending_update_id = Some(receipt.request_id);
                }),
                CurveRequest::Withdraw {
                    request_id,
                    expected_revision,
                } => {
                    if self.pending_update_id != Some(request_id) {
                        Err(PlaybackError::Invalid(
                            "curve withdrawal request identity changed",
                        ))
                    } else {
                        player.withdraw_curve(expected_revision).inspect(|()| {
                            self.pending_update_id = None;
                        })
                    }
                }
            };
            receipt.result = Some(result);
        }
        let polled = player.poll(now, &measured);
        {
            let mut state = self.control.state.lock();
            state.snapshot.feedback = Some(measured.clone());
            state.snapshot.feedback_reference = state.snapshot.last_reference.clone();
            state.snapshot.curve_progress = player.curve_progress();
            state.snapshot.settling = player.settling_evidence();
            state.snapshot.gripper = player.gripper_evidence();
            let recorded = state.snapshot.checkpoints.len();
            state
                .snapshot
                .checkpoints
                .extend_from_slice(&player.checkpoints()[recorded..]);
        }
        let action = match polled {
            Ok(action) => action,
            Err(error) => return Some(Err(ExecutionFailure::Playback(error))),
        };
        match action {
            PlaybackPoll::Dispatch(index) => {
                let reference = player.reference(index).expect("admitted reference").clone();
                match self.io.dispatch(&reference, &measured, player.force_n()) {
                    Ok(
                        receipt
                        @ (ReferenceReceipt::Dispatched | ReferenceReceipt::DispatchedAt(_)),
                    ) => {
                        let completed = clock.stamp_now().mono_ns().0;
                        let started = match receipt {
                            ReferenceReceipt::DispatchedAt(at) => at,
                            _ => completed,
                        };
                        if let Err(error) = player.acknowledge_at(index, started, completed) {
                            return Some(Err(ExecutionFailure::Playback(error)));
                        }
                        let mut state = self.control.state.lock();
                        state.snapshot.dispatched = player.dispatched();
                        state.snapshot.curve_progress = player.curve_progress();
                        if state
                            .snapshot
                            .curve_progress
                            .as_ref()
                            .is_some_and(|p| p.pending_revision.is_none())
                        {
                            self.pending_update_id = None;
                        }
                        state.snapshot.last_reference = Some(reference);
                        state.snapshot.last_acknowledged_ns = Some(completed);
                        // The next poll establishes its exact due time; it never
                        // skips an entry or catches up by dispatching a batch.
                        self.next_poll_ns =
                            now.max(started.saturating_add(self.dispatch_period_ns));
                    }
                    Ok(ReferenceReceipt::Superseded(gate)) => {
                        return Some(Err(ExecutionFailure::Superseded(gate)));
                    }
                    Err(error) => return Some(Err(ExecutionFailure::Dispatch(error))),
                }
            }
            PlaybackPoll::WaitUntil(at) => self.next_poll_ns = at,
            PlaybackPoll::Observe => self.next_poll_ns = now.saturating_add(self.sample_period_ns),
            PlaybackPoll::Complete(result) => return Some(Ok(result)),
        }
        self.next_poll_ns = self.next_poll_ns.min(player.check_by_ns());
        None
    }
}

impl Drop for ExecutionTask {
    fn drop(&mut self) {
        self.control.finish(
            self.failure
                .clone()
                .or_else(|| self.cancelled_result())
                .unwrap_or_else(|| {
                    Err(ExecutionFailure::Playback(PlaybackError::Cancelled(
                        "local execution worker stopped".into(),
                    )))
                }),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coordinated_subset_reserves_exact_members_until_terminal() {
        let slot = ExecutionSlot::default();
        let episode = EpisodeId::new("group");
        let selected = Some(vec!["left".into(), "right".into()]);
        let group = slot
            .register(episode.clone(), selected.clone(), 2, || false)
            .unwrap();
        for part in ["left", "right"] {
            assert!(slot.active(&episode, Some(part)));
            assert!(matches!(
                slot.register(episode.clone(), Some(vec![part.into()]), 2, || false),
                Err(PlaybackError::Busy)
            ));
        }
        assert!(!slot.active(&episode, Some("unused")));
        let third = slot
            .register(episode.clone(), Some(vec!["unused".into()]), 2, || false)
            .unwrap();
        assert!(matches!(
            slot.register(episode.clone(), None, 2, || false),
            Err(PlaybackError::Busy)
        ));
        assert!(matches!(
            slot.register(
                episode.clone(),
                Some(vec!["right".into(), "unused".into()]),
                2,
                || false
            ),
            Err(PlaybackError::Busy)
        ));
        group.cancel("selected group stopped");
        assert!(matches!(
            slot.register(episode.clone(), selected.clone(), 2, || false),
            Err(PlaybackError::Busy)
        ));
        assert!(
            group
                .control
                .finish(Err(ExecutionFailure::Playback(PlaybackError::Cancelled(
                    "selected group stopped".into()
                ))))
        );
        assert!(!slot.active(&episode, Some("left")));
        assert!(third.snapshot().result.is_none());
        assert!(slot.register(episode, selected, 2, || false).is_ok());
    }

    #[test]
    fn cancellation_winning_completion_waits_for_worker_cleanup() {
        let slot = ExecutionSlot::default();
        let handle = slot
            .register(EpisodeId::new("episode"), None, 1, || false)
            .unwrap();
        handle.cancel("stop before completion");
        assert!(!handle.control.finish(Ok(Completion::Arrived)));
        assert!(handle.snapshot().result.is_none());
        let cancelled = Err(ExecutionFailure::Playback(PlaybackError::Cancelled(
            "stop before completion".into(),
        )));
        assert!(handle.control.finish(cancelled.clone()));
        assert_eq!(handle.wait().result, Some(cancelled));
        // Cleanup that already established a terminal result stays immutable.
        assert!(handle.control.finish(Ok(Completion::Arrived)));
        assert!(handle.snapshot().result.unwrap().is_err());
    }
}
