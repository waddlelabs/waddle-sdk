//! Lock-free data paths: SPSC sample rings (rtrb) and a wait-free
//! latest-value slot for "current snapshot" consumers.

use arc_swap::ArcSwapOption;
use std::sync::Arc;

/// A wait-free latest-value slot. Writers replace, readers snapshot; there
/// is no queue and no backpressure — latest wins, by design (a late
/// observation is a wrong observation).
#[derive(Debug, Default)]
pub struct LatestSlot<T> {
    slot: ArcSwapOption<T>,
}

impl<T> LatestSlot<T> {
    #[must_use]
    pub fn new() -> Self {
        Self {
            slot: ArcSwapOption::empty(),
        }
    }

    pub fn publish(&self, value: T) {
        self.slot.store(Some(Arc::new(value)));
    }

    #[must_use]
    pub fn latest(&self) -> Option<Arc<T>> {
        self.slot.load_full()
    }
}

/// A wait-free SPSC ring for per-source sample paths (gate records, teleop
/// actions). Thin re-export of rtrb with a capacity constructor.
#[must_use]
pub fn sample_ring<T>(capacity: usize) -> (rtrb::Producer<T>, rtrb::Consumer<T>) {
    rtrb::RingBuffer::new(capacity)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn latest_slot_is_latest_wins() {
        let slot = LatestSlot::new();
        assert!(slot.latest().is_none());
        slot.publish(1u32);
        slot.publish(2u32);
        assert_eq!(*slot.latest().unwrap(), 2);
    }

    #[test]
    fn ring_is_spsc_fifo_and_bounded() {
        let (mut tx, mut rx) = sample_ring::<u64>(4);
        for i in 0..4 {
            tx.push(i).unwrap();
        }
        assert!(tx.push(9).is_err(), "full ring rejects, never blocks");
        assert_eq!(rx.pop().unwrap(), 0);
        assert_eq!(rx.pop().unwrap(), 1);
    }
}
