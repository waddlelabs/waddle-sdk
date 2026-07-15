//! The teleop jitter buffer: reorders a lossy latest-wins action stream and
//! plays out "the action due now" after a fixed playout delay.
//!
//! Pure and deterministic: time is an argument. The media intake thread
//! calls [`JitterBuffer::ingest`]; the consumer calls
//! [`JitterBuffer::pop_due`] each tick.
//!
//! Two independent producers write into the ONE ring `waddle-runtime` builds
//! per session (`StreamProducer`, Mutex-shared): the media-intake thread
//! (teleop packets, wire-ordered by `TeleopStreamPacket.seq`, genuinely
//! unordered over the network) and the reset-window agent-chunk arm
//! (`forward_server_msg`'s `InterventionChunk`, seq assigned by a pump-local
//! counter). Each [`TimedAction`] carries a [`StreamChannel`] tag so this
//! buffer keeps a SEPARATE reorder/late-drop cursor per producer — never one
//! shared high-water mark — so one producer's activity (e.g. an ordinary
//! teleop claim earlier in the session) can never starve or permanently drop
//! the other's arrivals (e.g. the first agent chunk of a later reset
//! window). See waddle-runtime's `pumps.rs` for the producer side.

use std::collections::BTreeMap;

use waddle_types::MonoNs;

use crate::gate::OwnedAction;

/// Which producer supplied an arrival — see the module doc for why this
/// can't be inferred from `seq` alone (the two producers' seq spaces are
/// independent and may overlap).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StreamChannel {
    /// `spawn_media_intake`'s teleop pose stream.
    Teleop,
    /// `forward_server_msg`'s reset-window `InterventionChunk` arm.
    AgentChunk,
}

#[derive(Debug, Clone)]
pub struct TimedAction {
    pub channel: StreamChannel,
    pub seq: u64,
    pub received: MonoNs,
    pub action: OwnedAction,
}

/// One channel's own reorder window and playout cursor.
#[derive(Debug, Default)]
struct ChannelState {
    pending: BTreeMap<u64, TimedAction>,
    last_popped_seq: Option<u64>,
}

#[derive(Debug)]
pub struct JitterBuffer {
    playout_delay_ns: i64,
    teleop: ChannelState,
    agent_chunk: ChannelState,
    dropped_late: u64,
}

impl JitterBuffer {
    #[must_use]
    pub fn new(playout_delay_ns: i64) -> Self {
        Self {
            playout_delay_ns: playout_delay_ns.max(0),
            teleop: ChannelState::default(),
            agent_chunk: ChannelState::default(),
            dropped_late: 0,
        }
    }

    fn state_mut(&mut self, channel: StreamChannel) -> &mut ChannelState {
        match channel {
            StreamChannel::Teleop => &mut self.teleop,
            StreamChannel::AgentChunk => &mut self.agent_chunk,
        }
    }

    /// Ingest an arrival. Actions at-or-before their OWN channel's playout
    /// cursor are late — dropped and counted, never reordered backwards (a
    /// late pose is a wrong pose) — but a channel's cursor never looks at
    /// another channel's arrivals.
    pub fn ingest(&mut self, action: TimedAction) {
        let state = self.state_mut(action.channel);
        if let Some(last) = state.last_popped_seq
            && action.seq <= last
        {
            self.dropped_late += 1;
            return;
        }
        state.pending.insert(action.seq, action);
    }

    /// Pop the next in-order action whose playout delay has elapsed, across
    /// both channels. In practice only one channel is ever populated at a
    /// time (one engaged claimant drives either teleop or agent-chunk
    /// actions, never both) but this makes no assumption of that: each
    /// channel's own head is checked independently, and whichever is ready
    /// and arrived first wins ties.
    pub fn pop_due(&mut self, now: MonoNs) -> Option<OwnedAction> {
        let playout_delay_ns = self.playout_delay_ns;
        let mut best: Option<(StreamChannel, u64, MonoNs)> = None;
        for channel in [StreamChannel::Teleop, StreamChannel::AgentChunk] {
            let state = self.state_mut(channel);
            if let Some((&seq, head)) = state.pending.iter().next()
                && head.received.0 + playout_delay_ns <= now.0
                && best.is_none_or(|(_, _, received)| head.received.0 < received.0)
            {
                best = Some((channel, seq, head.received));
            }
        }
        let (channel, seq, _) = best?;
        let state = self.state_mut(channel);
        let action = state.pending.remove(&seq).expect("first key exists");
        state.last_popped_seq = Some(seq);
        Some(action.action)
    }

    #[must_use]
    pub fn dropped_late(&self) -> u64 {
        self.dropped_late
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.teleop.pending.is_empty() && self.agent_chunk.pending.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use smallvec::smallvec;

    fn ta(seq: u64, received: i64) -> TimedAction {
        ta_on(StreamChannel::Teleop, seq, received)
    }

    fn ta_on(channel: StreamChannel, seq: u64, received: i64) -> TimedAction {
        TimedAction {
            channel,
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

    /// The critical regression this module exists to prevent: two producers
    /// share one buffer, but NOT one cursor. A teleop claimant's activity
    /// (advancing `Teleop`'s cursor arbitrarily high) must never cause the
    /// FIRST agent-chunk arrival of a later reset window — whose own counter
    /// starts fresh and independent (see `pumps.rs`'s `next_chunk_seq` doc
    /// comment) — to collide with that high-water mark and be dropped as
    /// late.
    #[test]
    fn channels_have_independent_reorder_cursors() {
        let mut jb = JitterBuffer::new(0);
        // A teleop claimant runs for a while: seq climbs well past any small
        // number an unrelated later producer might start from.
        for seq in 1..=50u64 {
            jb.ingest(ta_on(StreamChannel::Teleop, seq, 0));
            assert!(jb.pop_due(MonoNs(0)).is_some());
        }
        assert_eq!(jb.dropped_late(), 0);

        // A LATER reset window's agent-chunk producer starts its own seq
        // space fresh at 1 — with a single shared cursor this would be
        // "seq <= 50" and silently dropped forever; with independent
        // cursors it must ingest and pop normally.
        jb.ingest(ta_on(StreamChannel::AgentChunk, 1, 100));
        let popped = jb.pop_due(MonoNs(100));
        assert!(
            popped.is_some(),
            "agent-chunk seq=1 must not collide with the teleop channel's cursor"
        );
        assert_eq!(jb.dropped_late(), 0, "no cross-channel drop must occur");

        // And the teleop channel's own late-drop discipline still holds:
        // replaying an already-popped teleop seq is still dropped.
        jb.ingest(ta_on(StreamChannel::Teleop, 30, 200));
        assert!(jb.pop_due(MonoNs(200)).is_none());
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
