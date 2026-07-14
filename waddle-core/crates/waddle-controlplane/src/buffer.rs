//! Bounded offline buffering: drop-oldest with a loud high-water mark,
//! replayed strictly in order on reconnect (FSM.md §8).

use std::collections::VecDeque;

#[derive(Debug)]
pub struct OfflineBuffer<T> {
    items: VecDeque<T>,
    capacity: usize,
    dropped: u64,
}

impl<T> OfflineBuffer<T> {
    #[must_use]
    pub fn new(capacity: usize) -> Self {
        Self {
            items: VecDeque::with_capacity(capacity.min(1024)),
            capacity: capacity.max(1),
            dropped: 0,
        }
    }

    /// Push, dropping the OLDEST item when full (the newest state is the
    /// most valuable during recovery). Returns true when the push dropped.
    pub fn push(&mut self, item: T) -> bool {
        let mut dropped = false;
        if self.items.len() == self.capacity {
            self.items.pop_front();
            self.dropped += 1;
            dropped = true;
        }
        self.items.push_back(item);
        dropped
    }

    /// Drain in arrival order.
    pub fn drain(&mut self) -> impl Iterator<Item = T> + '_ {
        self.items.drain(..)
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.items.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    #[must_use]
    pub fn dropped(&self) -> u64 {
        self.dropped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drops_oldest_and_replays_in_order() {
        let mut b = OfflineBuffer::new(3);
        for i in 0..5 {
            b.push(i);
        }
        assert_eq!(b.dropped(), 2);
        let out: Vec<i32> = b.drain().collect();
        assert_eq!(out, vec![2, 3, 4]);
    }
}
