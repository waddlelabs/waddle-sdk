//! The teleop jitter buffer: reorders a lossy latest-wins action stream and
//! plays out "the action due now" after a fixed playout delay.
//!
//! Pure and deterministic: time is an argument. The media intake thread
//! calls [`JitterBuffer::ingest`]; the consumer calls
//! [`JitterBuffer::pop_due`] each tick.

use std::collections::BTreeMap;

use waddle_types::MonoNs;

use crate::gate::OwnedAction;

#[derive(Debug, Clone)]
pub struct TimedAction {
    pub seq: u64,
    pub received: MonoNs,
    pub action: OwnedAction,
}

#[derive(Debug)]
pub struct JitterBuffer {
    playout_delay_ns: i64,
    /// Reorder window keyed by sequence number.
    pending: BTreeMap<u64, TimedAction>,
    last_popped_seq: Option<u64>,
    dropped_late: u64,
}

impl JitterBuffer {
    #[must_use]
    pub fn new(playout_delay_ns: i64) -> Self {
        Self {
            playout_delay_ns: playout_delay_ns.max(0),
            pending: BTreeMap::new(),
            last_popped_seq: None,
            dropped_late: 0,
        }
    }

    /// Ingest an arrival. Actions at-or-before the playout cursor are late —
    /// dropped and counted, never reordered backwards (a late pose is a
    /// wrong pose).
    pub fn ingest(&mut self, action: TimedAction) {
        if let Some(last) = self.last_popped_seq
            && action.seq <= last
        {
            self.dropped_late += 1;
            return;
        }
        self.pending.insert(action.seq, action);
    }

    /// Pop the next in-order action whose playout delay has elapsed.
    pub fn pop_due(&mut self, now: MonoNs) -> Option<OwnedAction> {
        let (&seq, first) = self.pending.iter().next()?;
        if first.received.0 + self.playout_delay_ns > now.0 {
            return None;
        }
        let action = self.pending.remove(&seq).expect("first key exists");
        self.last_popped_seq = Some(seq);
        Some(action.action)
    }

    #[must_use]
    pub fn dropped_late(&self) -> u64 {
        self.dropped_late
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.pending.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use smallvec::smallvec;

    fn ta(seq: u64, received: i64) -> TimedAction {
        TimedAction {
            seq,
            received: MonoNs(received),
            action: OwnedAction {
                #[allow(clippy::cast_precision_loss)]
                values: smallvec![seq as f64],
                gripper: None,
            },
        }
    }

    #[test]
    fn respects_playout_delay() {
        let mut jb = JitterBuffer::new(1_000);
        jb.ingest(ta(1, 0));
        assert!(jb.pop_due(MonoNs(999)).is_none());
        assert!(jb.pop_due(MonoNs(1_000)).is_some());
    }

    #[test]
    fn reorders_within_the_window_and_drops_late() {
        let mut jb = JitterBuffer::new(100);
        jb.ingest(ta(2, 0));
        jb.ingest(ta(1, 10));
        assert_eq!(jb.pop_due(MonoNs(200)).unwrap().values[0], 1.0);
        assert_eq!(jb.pop_due(MonoNs(200)).unwrap().values[0], 2.0);
        // seq 1 again: behind the cursor → dropped.
        jb.ingest(ta(1, 50));
        assert!(jb.pop_due(MonoNs(500)).is_none());
        assert_eq!(jb.dropped_late(), 1);
    }

    proptest! {
        /// Pops are strictly seq-increasing regardless of arrival order, and
        /// nothing pops before its playout delay.
        #[test]
        fn pops_are_ordered_and_delayed(
            arrivals in proptest::collection::vec((0u64..64, 0i64..10_000), 1..64),
            delay in 0i64..2_000,
        ) {
            let mut jb = JitterBuffer::new(delay);
            let mut now = 0i64;
            let mut last_seq: Option<u64> = None;
            for (seq, gap) in arrivals {
                now += gap;
                jb.ingest(ta(seq, now));
                while let Some(a) = jb.pop_due(MonoNs(now)) {
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    let popped = a.values[0] as u64;
                    if let Some(prev) = last_seq {
                        prop_assert!(popped > prev, "out-of-order pop");
                    }
                    last_seq = Some(popped);
                }
            }
        }
    }
}
