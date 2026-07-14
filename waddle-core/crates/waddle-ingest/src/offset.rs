//! Per-source clock-offset estimation: maps a foreign source clock (camera
//! driver timestamps, teleop client clocks) onto the session monotonic
//! timeline.
//!
//! Estimator: a min-filter over `(host_receive_mono - source_ts)` in a
//! sliding window. The minimum is the sample with the least network/queueing
//! delay, which is the best offset estimate under one-way jitter; the window
//! bounds drift.

use std::collections::VecDeque;

use waddle_types::MonoNs;

#[derive(Debug)]
pub struct SourceOffsetEstimator {
    window: usize,
    /// (host_recv_mono, delta = host_recv_mono - source_ts)
    deltas: VecDeque<i64>,
}

impl SourceOffsetEstimator {
    #[must_use]
    pub fn new(window: usize) -> Self {
        Self {
            window: window.max(1),
            deltas: VecDeque::new(),
        }
    }

    pub fn observe(&mut self, source_ts_ns: i64, host_recv: MonoNs) {
        if self.deltas.len() == self.window {
            self.deltas.pop_front();
        }
        self.deltas.push_back(host_recv.0 - source_ts_ns);
    }

    /// The current offset estimate (add to a source timestamp to land on the
    /// session timeline). `None` until at least one observation.
    #[must_use]
    pub fn offset_ns(&self) -> Option<i64> {
        self.deltas.iter().copied().min()
    }

    /// Map a source timestamp onto the session timeline.
    #[must_use]
    pub fn map(&self, source_ts_ns: i64) -> Option<MonoNs> {
        Some(MonoNs(source_ts_ns + self.offset_ns()?))
    }
}

impl Default for SourceOffsetEstimator {
    fn default() -> Self {
        Self::new(256)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// Under bounded one-way jitter, the estimate converges to within
        /// the jitter bound of the true offset.
        #[test]
        fn converges_under_bounded_jitter(
            true_offset in 1_000_000i64..1_000_000_000,
            jitters in proptest::collection::vec(0i64..5_000_000, 32..256),
        ) {
            let mut est = SourceOffsetEstimator::new(512);
            let mut source_ts = 0i64;
            for j in &jitters {
                source_ts += 10_000_000; // 100 Hz source
                let host_recv = MonoNs(source_ts + true_offset + j);
                est.observe(source_ts, host_recv);
            }
            let got = est.offset_ns().unwrap();
            prop_assert!(got >= true_offset);
            prop_assert!(got <= true_offset + 5_000_000);
        }
    }

    #[test]
    fn min_filter_prefers_least_delayed_sample() {
        let mut est = SourceOffsetEstimator::new(8);
        est.observe(0, MonoNs(100));
        est.observe(10, MonoNs(150)); // more delayed
        est.observe(20, MonoNs(121)); // barely delayed
        assert_eq!(est.offset_ns(), Some(100));
        assert_eq!(est.map(50), Some(MonoNs(150)));
    }
}
