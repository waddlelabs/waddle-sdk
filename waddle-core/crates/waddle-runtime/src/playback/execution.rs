//! Finite local execution on the existing serialized control thread.

use std::panic::AssertUnwindSafe;
use std::sync::Arc;
use std::time::Duration;

use parking_lot::{Condvar, Mutex};
use waddle_types::time::Clock;
use waddle_types::{EpisodeId, Step};

use super::{
    Completion, CurveProgress, Feedback, FiniteReferences, GripperEvidence, Playback,
    PlaybackError, PlaybackPoll, SettlingEvidence,
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

/// An execution snapshot. A terminal result means the reference loop has ended;
/// it is not proof that a requested safety verb physically stopped the device.
#[derive(Clone, Debug)]
pub struct ExecutionSnapshot {
    pub id: String,
    pub episode_id: EpisodeId,
    pub dispatched: usize,
    pub references: usize,
    pub feedback: Option<Feedback>,
    pub curve_progress: Option<CurveProgress>,
    pub settling: Option<SettlingEvidence>,
    pub gripper: Option<GripperEvidence>,
    pub last_reference: Option<Step>,
    /// The completed stop callback, separate from the original motion result.
    /// Callback success alone does not establish measured physical stopping.
    pub cleanup: Option<VerbOutcome>,
    pub result: Option<ExecutionResult>,
}

#[derive(Debug)]
struct State {
    snapshot: ExecutionSnapshot,
    cancelled: Option<String>,
}

#[derive(Debug)]
struct Control {
    state: Mutex<State>,
    changed: Condvar,
}

impl Control {
    fn mark_cancelled(&self, reason: &str) {
        let mut state = self.state.lock();
        if state.snapshot.result.is_none() {
            state.cancelled.get_or_insert_with(|| reason.to_owned());
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
        self.control.mark_cancelled(reason);
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
    current: Mutex<Option<Arc<Control>>>,
}

impl ExecutionSlot {
    pub(crate) fn register(
        &self,
        episode_id: EpisodeId,
        references: usize,
        unavailable: impl FnOnce() -> bool,
    ) -> Result<ExecutionHandle, PlaybackError> {
        let mut current = self.current.lock();
        if unavailable()
            || current
                .as_ref()
                .is_some_and(|c| c.state.lock().snapshot.result.is_none())
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
                    curve_progress: None,
                    settling: None,
                    gripper: None,
                    last_reference: None,
                    cleanup: None,
                    result: None,
                },
                cancelled: None,
            }),
            changed: Condvar::new(),
        });
        *current = Some(control.clone());
        Ok(ExecutionHandle { control })
    }

    pub(crate) fn interrupt(&self, reason: &str, request: impl FnOnce()) {
        let current = self.current.lock();
        if let Some(control) = current.as_ref() {
            control.mark_cancelled(reason);
        }
        request();
    }

    pub(crate) fn active(&self, episode: &EpisodeId) -> bool {
        self.current.lock().as_ref().is_some_and(|control| {
            let state = control.state.lock();
            &state.snapshot.episode_id == episode && state.snapshot.result.is_none()
        })
    }

    pub(crate) fn cancel_episode(&self, episode: &EpisodeId) -> Option<ExecutionHandle> {
        let current = self.current.lock();
        let control = current.as_ref()?;
        if &control.state.lock().snapshot.episode_id != episode {
            return None;
        }
        let handle = ExecutionHandle {
            control: control.clone(),
        };
        handle.cancel("episode ended");
        Some(handle)
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
    failure: Option<ExecutionResult>,
}

impl ExecutionTask {
    pub(crate) fn new(
        mirror: Arc<Mirror>,
        episode: EpisodeId,
        references: FiniteReferences,
        io: Arc<dyn PlaybackIo>,
        handle: &ExecutionHandle,
        sample_period_ns: i64,
    ) -> Self {
        Self {
            mirror,
            episode,
            references: Some(references),
            player: None,
            io,
            control: handle.control.clone(),
            next_poll_ns: 0,
            sample_period_ns,
            failure: None,
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
            self.control.state.lock().snapshot.cleanup = Some(outcome.clone());
        }
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

    fn tick_inner<C: Clock>(&mut self, clock: &C) -> Option<ExecutionResult> {
        if self.scope_ended() {
            return Some(Err(ExecutionFailure::Superseded("episode ended".into())));
        }
        if let Some(result) = self.cancelled_result() {
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
        let now = clock.stamp_now().mono_ns().0;
        let player = self.player.get_or_insert_with(|| {
            Playback::new(
                self.references.take().expect("one admitted buffer"),
                measured.observed_ns,
            )
        });
        self.control.state.lock().snapshot.feedback = Some(measured.clone());
        let polled = player.poll(now, &measured);
        self.control.state.lock().snapshot.curve_progress = player.curve_progress();
        self.control.state.lock().snapshot.settling = player.settling_evidence();
        self.control.state.lock().snapshot.gripper = player.gripper_evidence();
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
                        state.snapshot.last_reference = Some(reference);
                        // The next poll establishes its exact due time; it never
                        // skips an entry or catches up by dispatching a batch.
                        self.next_poll_ns = now;
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
    fn cancellation_winning_completion_waits_for_worker_cleanup() {
        let slot = ExecutionSlot::default();
        let handle = slot
            .register(EpisodeId::new("episode"), 1, || false)
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
