//! The in-flight half of the droppability contract.
//!
//! [`ClientMsg::is_droppable`](crate::ClientMsg::is_droppable) splits this
//! client's outbound traffic in two: **history** (gate messages, episode
//! events, `ProprioSample` observations — small, and losing one loses the
//! record) and **perception/liveness** (control-plane stills, heartbeats —
//! large and/or worthless once stale). That single classification governs
//! two different moments:
//!
//! - plane OFFLINE: a droppable message is never buffered
//!   (`ClientMsg::buffer_when_offline`), so a partition's worth of stills can
//!   never evict real episode history from the bounded offline buffer;
//! - plane CONNECTED but not draining: this module. A droppable message is
//!   admitted only while fewer than `cap` of them are still in flight on that
//!   sink; the next one is dropped and counted.
//!
//! Why the bound lives here and not in the client: the client hands a message
//! to `ControlConn::tx` and it is gone. A transport that buffers internally
//! is the only place that knows when a message has actually left — the gRPC
//! transport holds a channel per stream, and h2 stops taking messages the
//! moment the peer's flow-control window closes, which is exactly the
//! connected-but-stalled plane this bounds. So the meter sits at the terminal
//! sink: [`InflightLimit::admit`] takes a slot on the way in, and the slot is
//! released when the transport takes the message back out
//! ([`Inflight::into_inner`]) — the bound therefore tracks what the transport
//! has actually accepted, never what was merely offered to it.
//!
//! History is deliberately NOT bounded here: it is small, and the only thing
//! allowed to lose it is the connection dying — which is precisely what the
//! offline buffer exists to replay. A plane that stays connected and never
//! drains forever is a liveness problem (no client-side heartbeat deadline
//! severs it today); this module only guarantees that such a plane cannot
//! turn perception into unbounded memory growth.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};

/// How many droppable messages one sink may hold in flight before the next is
/// shed. Small on purpose: a `FrameStill` is orders of magnitude larger than
/// anything else on this wire (a 720p JPEG is tens of KB), and a still that
/// has been waiting behind a few others is already a picture of a world that
/// has moved on — the same freshest-wins logic as the SDK's own latest-wins
/// stills slot, continued past the transport seam.
pub const DEFAULT_INFLIGHT_CAP: usize = 4;

/// One sink's bound on droppable messages in flight.
#[derive(Debug)]
pub struct InflightLimit {
    cap: usize,
    live: AtomicUsize,
    dropped: Arc<AtomicU64>,
}

impl InflightLimit {
    /// A limit for one sink. `dropped` is shared (typically one counter per
    /// transport) so a single number answers "how much perception did we
    /// shed?" across every sink and every connection.
    ///
    /// Sinks get their OWN limit rather than sharing one: they drain
    /// independently, and a saturated observation stream must not shed the
    /// heartbeats that keep the session alive.
    #[must_use]
    pub fn new(cap: usize, dropped: Arc<AtomicU64>) -> Arc<Self> {
        Arc::new(Self {
            cap: cap.max(1),
            live: AtomicUsize::new(0),
            dropped,
        })
    }

    /// Meter one outbound message.
    ///
    /// A non-droppable message ALWAYS passes and takes no slot — history is
    /// never dropped in flight. A droppable one passes only while fewer than
    /// `cap` droppable messages are still in flight on this sink; otherwise
    /// `None` is returned (the caller drops it, which is not a connection
    /// failure) and the shed message is counted.
    pub fn admit<T>(self: &Arc<Self>, value: T, droppable: bool) -> Option<Inflight<T>> {
        if !droppable {
            return Some(Inflight { value, _slot: None });
        }
        // fetch_add-then-back-off rather than a CAS loop: whichever caller
        // observes a pre-value below `cap` holds a real slot, so the live
        // count never exceeds `cap` even when several threads race.
        if self.live.fetch_add(1, Ordering::Relaxed) >= self.cap {
            self.live.fetch_sub(1, Ordering::Relaxed);
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return None;
        }
        Some(Inflight {
            value,
            _slot: Some(Slot(self.clone())),
        })
    }

    /// Droppable messages currently in flight on this sink.
    #[must_use]
    pub fn live(&self) -> usize {
        self.live.load(Ordering::Relaxed)
    }

    /// Droppable messages shed by every sink sharing this counter.
    #[must_use]
    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

/// One metered message: the payload, plus (for a droppable one) the in-flight
/// slot it holds. The slot is released when this is unwrapped or dropped, so
/// a transport releases capacity exactly where it takes the message on.
#[derive(Debug)]
pub struct Inflight<T> {
    value: T,
    _slot: Option<Slot>,
}

impl<T> Inflight<T> {
    /// Take the payload back, releasing the slot. Call this exactly where the
    /// transport hands the message onward — for the gRPC transport, in the
    /// `map` its outbound streams are polled through.
    #[must_use]
    pub fn into_inner(self) -> T {
        self.value
    }
}

/// The RAII half: dropping it returns capacity to the limit.
#[derive(Debug)]
struct Slot(Arc<InflightLimit>);

impl Drop for Slot {
    fn drop(&mut self) {
        self.0.live.fetch_sub(1, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn limit(cap: usize) -> Arc<InflightLimit> {
        InflightLimit::new(cap, Arc::new(AtomicU64::new(0)))
    }

    /// History is never metered: any number passes, none takes a slot, none
    /// is ever counted as shed.
    #[test]
    fn non_droppable_messages_are_never_bounded() {
        let limit = limit(2);
        let held: Vec<Inflight<u32>> = (0..100)
            .map(|i| limit.admit(i, false).expect("history always passes"))
            .collect();
        assert_eq!(limit.live(), 0, "history takes no in-flight slot");
        assert_eq!(limit.dropped(), 0);
        drop(held);
    }

    /// The headline bound: droppable messages queue up to `cap`, and the
    /// next one is shed rather than queued — the freshest-wins degradation,
    /// counted, never a connection failure.
    #[test]
    fn droppable_messages_stop_at_the_cap_and_are_counted() {
        let limit = limit(2);
        let a = limit.admit(1, true).expect("first fits");
        let b = limit.admit(2, true).expect("second fits");
        assert_eq!(limit.live(), 2);
        assert!(limit.admit(3, true).is_none(), "over cap: shed");
        assert!(limit.admit(4, true).is_none());
        assert_eq!(limit.dropped(), 2);
        assert_eq!(limit.live(), 2, "a shed message never occupies a slot");
        // History still gets through while the droppable class is saturated.
        assert!(limit.admit(5, false).is_some());
        drop((a, b));
    }

    /// Capacity comes back when the transport takes a message on — which is
    /// what makes this a bound on what is really in flight, not a quota.
    #[test]
    fn unwrapping_a_message_returns_its_slot() {
        let limit = limit(1);
        let held = limit.admit(7u32, true).expect("first fits");
        assert!(limit.admit(8, true).is_none(), "cap of one is full");
        assert_eq!(held.into_inner(), 7, "into_inner returns the payload");
        assert_eq!(limit.live(), 0, "and releases the slot");
        assert!(limit.admit(9, true).is_some(), "capacity is back");
        assert_eq!(limit.dropped(), 1);
    }

    /// Sinks are independent (a saturated stream must not shed another
    /// stream's traffic) but share one drop counter.
    #[test]
    fn sinks_are_independent_but_share_the_counter() {
        let dropped = Arc::new(AtomicU64::new(0));
        let obs = InflightLimit::new(1, dropped.clone());
        let hb = InflightLimit::new(1, dropped.clone());
        let _held = obs.admit(1, true).expect("obs slot");
        assert!(obs.admit(2, true).is_none());
        assert!(hb.admit(3, true).is_some(), "a separate sink is unaffected");
        assert_eq!(dropped.load(Ordering::Relaxed), 1);
        assert_eq!(hb.dropped(), 1, "one counter, every sink");
    }
}
