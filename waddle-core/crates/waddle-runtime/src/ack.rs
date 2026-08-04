//! Directive-ack correlation (flag `waddle.v0.plane.acks`): the plane pump
//! wraps the session event(s) a plane directive decodes into in an
//! [`Injected`] envelope carrying a shared [`AckGroup`]; the reducer — the
//! only place the FSM's step outcome is visible — records each event's
//! outcome and emits exactly one `DirectiveAck` when the group completes.
//!
//! No legality is decided here (hollow-frontend): the FSM's `step` result is
//! forwarded verbatim. The pump attaches a group only when the directive
//! carried a `directive_id` AND the connection negotiated the flag, so an
//! id-less directive (or an un-negotiated connection) flows through the
//! exact pre-flag fire-and-forget path: `ack: None`, nothing emitted.

use std::sync::Arc;

use parking_lot::Mutex;
use waddle_fsm::SessionEvent;

/// The feature flag gating directive acks (`VERSIONING.md` registry).
/// Declared at Register whenever a transport is configured — always safe:
/// emission still requires the directive to carry a `directive_id` and the
/// plane to have accepted the flag. Named once, in the crate that
/// negotiates it and classifies by it.
pub(crate) use waddle_controlplane::flags::ACKS as ACKS_FLAG;

/// One event on the reducer's single funnel, optionally carrying the
/// directive-ack correlation for the plane directive it decoded from.
#[derive(Debug)]
pub(crate) struct Injected {
    pub event: SessionEvent,
    /// `Some` only for plane-directive events whose directive carried a
    /// `directive_id` on a connection that negotiated `waddle.v0.plane.acks`.
    pub ack: Option<Arc<AckGroup>>,
}

impl From<SessionEvent> for Injected {
    fn from(event: SessionEvent) -> Self {
        Self { event, ack: None }
    }
}

/// The pending ack for one plane directive. A directive that decodes into
/// more than one session event (a claim GRANT → `ClaimGranted` + `Engage`;
/// a reset-window ENGAGE → `ClaimGranted` + `ResetWindowEngage`) shares one
/// group across its events and acks once: accepted iff every event was
/// accepted, reason from the first rejection.
#[derive(Debug)]
pub(crate) struct AckGroup {
    directive_id: String,
    state: Mutex<GroupState>,
}

#[derive(Debug)]
struct GroupState {
    /// Events still awaiting their step outcome.
    remaining: u32,
    /// The first rejection's reason, if any event was rejected.
    rejection: Option<String>,
}

/// A finished group, ready to emit as a `DirectiveAck`.
pub(crate) struct FinishedAck {
    pub directive_id: String,
    pub accepted: bool,
    pub reason: String,
}

impl AckGroup {
    /// A group expecting `events` step outcomes (one per enveloped event).
    pub fn new(directive_id: String, events: u32) -> Arc<Self> {
        debug_assert!(events > 0, "an ack group must cover at least one event");
        Arc::new(Self {
            directive_id,
            state: Mutex::new(GroupState {
                remaining: events,
                rejection: None,
            }),
        })
    }

    /// Record one enveloped event's step outcome (`Err` carries the FSM's
    /// rejection reason). Returns the finished ack when this was the group's
    /// last outstanding event, `None` while others are still in flight — and
    /// `None` forever after it finished, so a group can never emit twice
    /// (unreachable when the pump sizes `events` correctly, but an ack is a
    /// protocol answer: at-most-once is worth being structural).
    pub fn record(&self, outcome: Result<(), String>) -> Option<FinishedAck> {
        let mut st = self.state.lock();
        if st.remaining == 0 {
            return None;
        }
        if let Err(reason) = outcome
            && st.rejection.is_none()
        {
            st.rejection = Some(reason);
        }
        st.remaining -= 1;
        (st.remaining == 0).then(|| {
            let reason = st.rejection.take();
            FinishedAck {
                directive_id: self.directive_id.clone(),
                accepted: reason.is_none(),
                reason: reason.unwrap_or_default(),
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_single_event_group_finishes_on_its_outcome() {
        let g = AckGroup::new("d1".into(), 1);
        let fin = g.record(Ok(())).expect("finished");
        assert!(fin.accepted);
        assert_eq!(fin.directive_id, "d1");
        assert_eq!(fin.reason, "");
    }

    #[test]
    fn a_two_event_group_acks_once_with_the_first_rejection() {
        let g = AckGroup::new("d2".into(), 2);
        assert!(g.record(Err("first reason".into())).is_none());
        let fin = g
            .record(Err("second reason".into()))
            .expect("finished on the last event");
        assert!(!fin.accepted);
        assert_eq!(fin.reason, "first reason");
    }

    #[test]
    fn a_finished_group_never_emits_twice() {
        let g = AckGroup::new("d5".into(), 1);
        assert!(g.record(Ok(())).is_some());
        assert!(g.record(Ok(())).is_none(), "at-most-once is structural");
        assert!(g.record(Err("late".into())).is_none());
    }

    #[test]
    fn accepted_iff_every_event_accepted() {
        let g = AckGroup::new("d3".into(), 2);
        assert!(g.record(Ok(())).is_none());
        let fin = g.record(Err("late reject".into())).expect("finished");
        assert!(!fin.accepted);
        assert_eq!(fin.reason, "late reject");

        let g = AckGroup::new("d4".into(), 2);
        assert!(g.record(Ok(())).is_none());
        let fin = g.record(Ok(())).expect("finished");
        assert!(fin.accepted);
    }
}
