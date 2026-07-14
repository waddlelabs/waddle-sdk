//! Gate-tick statistics: a lock-free reservoir of inter-tick deltas feeding
//! the N11 heartbeat proxy signals (gate-tick jitter is a direct read on the
//! integrator's loop health for advisory-lease integrations).

use std::sync::atomic::{AtomicI64, AtomicU64, Ordering};

use waddle_types::MonoNs;

const SLOTS: usize = 128;

#[derive(Debug)]
pub struct TickStats {
    last_tick_ns: AtomicI64,
    deltas: [AtomicI64; SLOTS],
    count: AtomicU64,
}

impl Default for TickStats {
    fn default() -> Self {
        Self::new()
    }
}

/// A percentile snapshot of recent inter-tick deltas.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TickSnapshot {
    pub p50_ns: i64,
    pub p95_ns: i64,
    pub max_ns: i64,
    pub samples: u32,
}

impl TickStats {
    #[must_use]
    pub fn new() -> Self {
        Self {
            last_tick_ns: AtomicI64::new(0),
            deltas: [const { AtomicI64::new(0) }; SLOTS],
            count: AtomicU64::new(0),
        }
    }

    /// Record a tick (called from the gate fast path: two atomics, wait-free).
    pub fn record(&self, now: MonoNs) {
        let prev = self.last_tick_ns.swap(now.0, Ordering::Relaxed);
        if prev != 0 {
            let delta = now.0 - prev;
            let n = self.count.fetch_add(1, Ordering::Relaxed);
            #[allow(clippy::cast_possible_truncation)]
            let slot = (n as usize) % SLOTS;
            self.deltas[slot].store(delta, Ordering::Relaxed);
        }
    }

    #[must_use]
    pub fn last_tick(&self) -> Option<MonoNs> {
        match self.last_tick_ns.load(Ordering::Relaxed) {
            0 => None,
            t => Some(MonoNs(t)),
        }
    }

    /// Percentiles over the recent reservoir (heartbeat-time, not hot path).
    #[must_use]
    pub fn snapshot(&self) -> TickSnapshot {
        let n = self.count.load(Ordering::Relaxed);
        #[allow(clippy::cast_possible_truncation)]
        let filled = (n.min(SLOTS as u64)) as usize;
        if filled == 0 {
            return TickSnapshot::default();
        }
        let mut vals: Vec<i64> = self.deltas[..filled]
            .iter()
            .map(|d| d.load(Ordering::Relaxed))
            .collect();
        vals.sort_unstable();
        #[allow(clippy::cast_possible_truncation)]
        TickSnapshot {
            p50_ns: vals[filled / 2],
            p95_ns: vals[(filled * 95) / 100],
            max_ns: *vals.last().expect("non-empty"),
            samples: filled as u32,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snapshot_reports_percentiles() {
        let stats = TickStats::new();
        for i in 0..100 {
            stats.record(MonoNs(i * 1_000));
        }
        let snap = stats.snapshot();
        assert_eq!(snap.p50_ns, 1_000);
        assert_eq!(snap.max_ns, 1_000);
        assert!(snap.samples > 0);
    }
}
