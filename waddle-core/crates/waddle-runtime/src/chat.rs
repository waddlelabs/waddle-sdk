//! Bounded, connection-scoped UI chat state.
//!
//! Chat is deliberately outside the FSM: it carries no claim, lease, handoff,
//! or timeline decision. The control-plane connection owns delivery; this
//! object only correlates one outstanding request and retains a bounded event
//! tail for a local authenticated UI to long-poll.

use std::collections::VecDeque;
use std::sync::Arc;
use std::time::Duration;

use parking_lot::{Condvar, Mutex};
use waddle_ingest::SessionClock;
use waddle_types::pb::v0 as pb;

pub(crate) const CHAT_REQUEST_ID_MAX_BYTES: usize = 128;
pub(crate) const CHAT_REQUEST_TEXT_MAX_BYTES: usize = 4 * 1024;
const CHAT_EVENT_TEXT_MAX_BYTES: usize = 16 * 1024;
const CHAT_EVENT_DETAIL_MAX_BYTES: usize = 1024;
const CHAT_EVENT_CAPACITY: usize = 128;

#[derive(Debug)]
struct Pending {
    request_id: String,
    last_sequence: u64,
}

#[derive(Debug, Default)]
struct State {
    pending: Option<Pending>,
    events: VecDeque<pb::ChatEvent>,
}

#[derive(Debug, Default)]
pub(crate) struct ChatInbox {
    state: Mutex<State>,
    changed: Condvar,
}

impl ChatInbox {
    pub(crate) fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    pub(crate) fn begin(&self, request_id: &str) -> Result<(), &'static str> {
        let mut state = self.state.lock();
        if state.pending.is_some() {
            return Err("a chat turn is already outstanding for this session");
        }
        state.pending = Some(Pending {
            request_id: request_id.to_owned(),
            last_sequence: 0,
        });
        Ok(())
    }

    /// Accept an event only for the live request, with strictly increasing
    /// sequence and bounded public fields. The plane is expected to enforce
    /// the same boundary; this second check prevents malformed input from
    /// reaching the browser even if a peer is buggy.
    pub(crate) fn push(&self, event: pb::ChatEvent) {
        let terminal = matches!(
            pb::ChatEventKind::try_from(event.kind),
            Ok(pb::ChatEventKind::Done | pb::ChatEventKind::Unavailable | pb::ChatEventKind::Error)
        );
        let mut state = self.state.lock();
        let Some(pending) = state.pending.as_mut() else {
            return;
        };
        if event.request_id != pending.request_id
            || event.sequence <= pending.last_sequence
            || event.text.len() > CHAT_EVENT_TEXT_MAX_BYTES
            || event.detail.len() > CHAT_EVENT_DETAIL_MAX_BYTES
            || pb::ChatEventKind::try_from(event.kind).is_err()
            || event.kind == pb::ChatEventKind::Unspecified as i32
        {
            return;
        }
        pending.last_sequence = event.sequence;
        state.events.push_back(event);
        while state.events.len() > CHAT_EVENT_CAPACITY {
            state.events.pop_front();
        }
        if terminal {
            state.pending = None;
        }
        self.changed.notify_all();
    }

    /// Close the current turn at a connection boundary. This is a local,
    /// public-safe lifecycle event; it is never replayed to the plane.
    pub(crate) fn unavailable(&self, detail: &str) {
        let mut state = self.state.lock();
        let Some(pending) = state.pending.take() else {
            return;
        };
        let event = pb::ChatEvent {
            request_id: pending.request_id,
            sequence: pending.last_sequence.saturating_add(1),
            kind: pb::ChatEventKind::Unavailable as i32,
            text: String::new(),
            detail: detail.chars().take(CHAT_EVENT_DETAIL_MAX_BYTES).collect(),
        };
        state.events.push_back(event);
        while state.events.len() > CHAT_EVENT_CAPACITY {
            state.events.pop_front();
        }
        self.changed.notify_all();
    }

    pub(crate) fn wait(
        &self,
        clock: &SessionClock,
        request_id: &str,
        after_sequence: u64,
        timeout: Duration,
    ) -> Vec<pb::ChatEvent> {
        let timeout_ns = i64::try_from(timeout.as_nanos()).unwrap_or(i64::MAX);
        let deadline_ns = clock.now().0.saturating_add(timeout_ns);
        let mut state = self.state.lock();
        loop {
            let found: Vec<_> = state
                .events
                .iter()
                .filter(|event| event.request_id == request_id && event.sequence > after_sequence)
                .cloned()
                .collect();
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_request_is_correlated_ordered_and_terminal_reopens_the_slot() {
        let inbox = ChatInbox::new();
        inbox.begin("a").unwrap();
        assert!(inbox.begin("b").is_err());
        inbox.push(pb::ChatEvent {
            request_id: "wrong".into(),
            sequence: 1,
            kind: pb::ChatEventKind::Text as i32,
            text: "secret".into(),
            ..Default::default()
        });
        inbox.push(pb::ChatEvent {
            request_id: "a".into(),
            sequence: 1,
            kind: pb::ChatEventKind::Text as i32,
            text: "hello".into(),
            ..Default::default()
        });
        inbox.push(pb::ChatEvent {
            request_id: "a".into(),
            sequence: 2,
            kind: pb::ChatEventKind::Done as i32,
            ..Default::default()
        });
        let events = inbox.wait(&SessionClock::capture(), "a", 0, Duration::ZERO);
        assert_eq!(events.len(), 2);
        assert_eq!(events[0].text, "hello");
        inbox.begin("b").unwrap();
    }

    #[test]
    fn disconnect_finishes_the_live_request_without_waiting() {
        let inbox = ChatInbox::new();
        inbox.begin("a").unwrap();
        inbox.unavailable("chat connection lost; local controls remain available");
        let events = inbox.wait(&SessionClock::capture(), "a", 0, Duration::ZERO);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].kind, pb::ChatEventKind::Unavailable as i32);
        inbox.begin("b").unwrap();
    }
}
