//! Bounded, connection-scoped replies for optional control-plane services.
//!
//! These services are intentionally outside the FSM: task conversation,
//! calibration measurement collection, and artifact delivery carry no
//! claim, lease, handoff, or timeline decision. This inbox only correlates
//! replies and keeps bounded public-safe tails for the authenticated local
//! UI.

use std::collections::{HashMap, VecDeque};
use std::sync::Arc;
use std::time::Duration;

use parking_lot::{Condvar, Mutex};
use waddle_ingest::SessionClock;
use waddle_types::pb::v0 as pb;

const EVENT_CAPACITY: usize = 256;
const EVENT_TEXT_MAX_BYTES: usize = 16 * 1024;
const EVENT_DETAIL_MAX_BYTES: usize = 1024;

#[derive(Debug)]
struct PendingTask {
    task_session_id: String,
    name: String,
    last_sequence: u64,
}

#[derive(Debug, Default)]
struct State {
    pending_tasks: HashMap<String, PendingTask>,
    task_events: VecDeque<pb::TaskSessionEvent>,
    last_calibration_sequence: HashMap<String, u64>,
    calibration_updates: VecDeque<pb::CalibrationUpdate>,
    artifact_ready: VecDeque<pb::WorkspaceArtifactReady>,
}

#[derive(Debug, Default)]
pub(crate) struct PlaneEvents {
    state: Mutex<State>,
    changed: Condvar,
}

impl PlaneEvents {
    pub(crate) fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    pub(crate) fn begin_task(
        &self,
        request_id: &str,
        task_session_id: &str,
        name: &str,
    ) -> Result<(), &'static str> {
        let mut state = self.state.lock();
        if state.pending_tasks.contains_key(request_id) {
            return Err("this task request_id is already outstanding");
        }
        state.pending_tasks.insert(
            request_id.to_owned(),
            PendingTask {
                task_session_id: task_session_id.to_owned(),
                name: name.to_owned(),
                last_sequence: 0,
            },
        );
        Ok(())
    }

    pub(crate) fn push_task(&self, event: pb::TaskSessionEvent) {
        let Ok(kind) = pb::TaskSessionEventKind::try_from(event.kind) else {
            return;
        };
        let terminal = matches!(
            kind,
            pb::TaskSessionEventKind::Done
                | pb::TaskSessionEventKind::Interrupted
                | pb::TaskSessionEventKind::Unavailable
                | pb::TaskSessionEventKind::Error
                | pb::TaskSessionEventKind::HistoryComplete
        );
        let mut state = self.state.lock();
        let Some(pending) = state.pending_tasks.get_mut(&event.request_id) else {
            return;
        };
        if event.request_id.len() > 128
            || event.task_session_id.is_empty()
            || event.task_session_id.len() > 200
            || event.name.len() > 200
            || (!pending.task_session_id.is_empty()
                && event.task_session_id != pending.task_session_id)
            || (!pending.name.is_empty() && event.name != pending.name)
            || event.sequence <= pending.last_sequence
            || event.text.len() > EVENT_TEXT_MAX_BYTES
            || event.detail.len() > EVENT_DETAIL_MAX_BYTES
            || kind == pb::TaskSessionEventKind::Unspecified
            || (kind == pb::TaskSessionEventKind::Text
                && (event.text.is_empty() || !matches!(event.role.as_str(), "user" | "assistant")))
            || (kind != pb::TaskSessionEventKind::Text
                && (!event.text.is_empty() || !event.role.is_empty()))
            || (kind == pb::TaskSessionEventKind::HistoryComplete
                && event.detail != "history page complete")
            || (kind != pb::TaskSessionEventKind::HistoryComplete && event.history_cursor != 0)
        {
            return;
        }
        pending.last_sequence = event.sequence;
        state.task_events.push_back(event.clone());
        trim(&mut state.task_events);
        if terminal {
            state.pending_tasks.remove(&event.request_id);
        }
        self.changed.notify_all();
    }

    pub(crate) fn push_calibration(&self, update: pb::CalibrationUpdate) {
        if update.calibration_id.is_empty()
            || update.detail.len() > EVENT_DETAIL_MAX_BYTES
            || update.sequence == 0
            || pb::CalibrationUpdateKind::try_from(update.kind).is_err()
            || update.kind == pb::CalibrationUpdateKind::Unspecified as i32
        {
            return;
        }
        let mut state = self.state.lock();
        let last = state
            .last_calibration_sequence
            .entry(update.calibration_id.clone())
            .or_default();
        if update.sequence <= *last {
            return;
        }
        *last = update.sequence;
        state.calibration_updates.push_back(update);
        trim(&mut state.calibration_updates);
        self.changed.notify_all();
    }

    pub(crate) fn push_artifact(&self, ready: pb::WorkspaceArtifactReady) {
        if ready.request_id.is_empty()
            || ready.detail.len() > EVENT_DETAIL_MAX_BYTES
            || ready.sha256.len() > 128
            || ready.download_ref.len() > 1024
        {
            return;
        }
        let mut state = self.state.lock();
        state.artifact_ready.push_back(ready);
        trim(&mut state.artifact_ready);
        self.changed.notify_all();
    }

    pub(crate) fn unavailable(&self, detail: &str) {
        let mut state = self.state.lock();
        let detail: String = detail.chars().take(EVENT_DETAIL_MAX_BYTES).collect();
        let pending = std::mem::take(&mut state.pending_tasks);
        for (request_id, task) in pending {
            state.task_events.push_back(pb::TaskSessionEvent {
                request_id,
                task_session_id: task.task_session_id,
                name: task.name,
                sequence: task.last_sequence.saturating_add(1),
                kind: pb::TaskSessionEventKind::Unavailable as i32,
                text: String::new(),
                detail: detail.clone(),
                role: String::new(),
                history_cursor: 0,
            });
        }
        while state.task_events.len() > EVENT_CAPACITY {
            state.task_events.pop_front();
        }
        self.changed.notify_all();
    }

    pub(crate) fn wait_tasks(
        &self,
        clock: &SessionClock,
        request_id: &str,
        after_sequence: u64,
        timeout: Duration,
    ) -> Vec<pb::TaskSessionEvent> {
        self.wait(clock, timeout, |state| {
            state
                .task_events
                .iter()
                .filter(|event| event.request_id == request_id && event.sequence > after_sequence)
                .cloned()
                .collect()
        })
    }

    pub(crate) fn wait_calibrations(
        &self,
        clock: &SessionClock,
        calibration_id: &str,
        after_sequence: u64,
        timeout: Duration,
    ) -> Vec<pb::CalibrationUpdate> {
        self.wait(clock, timeout, |state| {
            state
                .calibration_updates
                .iter()
                .filter(|update| {
                    update.calibration_id == calibration_id && update.sequence > after_sequence
                })
                .cloned()
                .collect()
        })
    }

    pub(crate) fn wait_artifact(
        &self,
        clock: &SessionClock,
        request_id: &str,
        timeout: Duration,
    ) -> Vec<pb::WorkspaceArtifactReady> {
        self.wait(clock, timeout, |state| {
            state
                .artifact_ready
                .iter()
                .filter(|ready| ready.request_id == request_id)
                .cloned()
                .collect()
        })
    }

    fn wait<T, F>(&self, clock: &SessionClock, timeout: Duration, find: F) -> Vec<T>
    where
        F: Fn(&State) -> Vec<T>,
    {
        let timeout_ns = i64::try_from(timeout.as_nanos()).unwrap_or(i64::MAX);
        let deadline_ns = clock.now().0.saturating_add(timeout_ns);
        let mut state = self.state.lock();
        loop {
            let found = find(&state);
            if !found.is_empty() || timeout.is_zero() {
                return found;
            }
            let remaining_ns = deadline_ns.saturating_sub(clock.now().0);
            if remaining_ns == 0 {
                return Vec::new();
            }
            #[allow(clippy::cast_sign_loss)]
            let remaining = Duration::from_nanos(remaining_ns as u64);
            if self.changed.wait_for(&mut state, remaining).timed_out() {
                return Vec::new();
            }
        }
    }
}

fn trim<T>(events: &mut VecDeque<T>) {
    while events.len() > EVENT_CAPACITY {
        events.pop_front();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn task_events_are_correlated_ordered_and_closed_on_disconnect() {
        let events = PlaneEvents::new();
        events.begin_task("one", "session", "demo").unwrap();
        events.push_task(pb::TaskSessionEvent {
            request_id: "one".into(),
            task_session_id: "session".into(),
            name: "demo".into(),
            sequence: 1,
            kind: pb::TaskSessionEventKind::Text as i32,
            text: "hello".into(),
            role: "assistant".into(),
            ..Default::default()
        });
        events.unavailable("connection lost");
        let found = events.wait_tasks(&SessionClock::capture(), "one", 0, Duration::ZERO);
        assert_eq!(found.len(), 2);
        assert_eq!(found[1].kind, pb::TaskSessionEventKind::Unavailable as i32);
    }

    #[test]
    fn history_page_is_cursor_bearing_and_terminal() {
        let events = PlaneEvents::new();
        events.begin_task("history", "session", "demo").unwrap();
        events.push_task(pb::TaskSessionEvent {
            request_id: "history".into(),
            task_session_id: "session".into(),
            name: "demo".into(),
            sequence: 8,
            kind: pb::TaskSessionEventKind::Text as i32,
            text: "restored".into(),
            role: "assistant".into(),
            ..Default::default()
        });
        events.push_task(pb::TaskSessionEvent {
            request_id: "history".into(),
            task_session_id: "session".into(),
            name: "demo".into(),
            sequence: 9,
            kind: pb::TaskSessionEventKind::HistoryComplete as i32,
            detail: "history page complete".into(),
            history_cursor: 8,
            ..Default::default()
        });
        // The terminal marker removes the pending request, so late output is ignored.
        events.push_task(pb::TaskSessionEvent {
            request_id: "history".into(),
            task_session_id: "session".into(),
            name: "demo".into(),
            sequence: 10,
            kind: pb::TaskSessionEventKind::Done as i32,
            ..Default::default()
        });
        let found = events.wait_tasks(&SessionClock::capture(), "history", 0, Duration::ZERO);
        assert_eq!(found.len(), 2);
        assert_eq!(found[1].history_cursor, 8);
    }

    #[test]
    fn calibration_and_artifact_replies_are_correlated() {
        let events = PlaneEvents::new();
        events.push_calibration(pb::CalibrationUpdate {
            calibration_id: "cal".into(),
            frame_seq: 3,
            sequence: 1,
            kind: pb::CalibrationUpdateKind::Accepted as i32,
            ..Default::default()
        });
        events.push_artifact(pb::WorkspaceArtifactReady {
            request_id: "export".into(),
            sha256: "00".repeat(32),
            download_ref: "one-time".into(),
            ..Default::default()
        });
        assert_eq!(
            events
                .wait_calibrations(&SessionClock::capture(), "cal", 0, Duration::ZERO,)
                .len(),
            1
        );
        assert_eq!(
            events
                .wait_artifact(&SessionClock::capture(), "export", Duration::ZERO)
                .len(),
            1
        );
    }
}
