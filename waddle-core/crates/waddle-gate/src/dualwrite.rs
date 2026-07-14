//! Runtime dual-write detection (N14).
//!
//! During any bypass on an advisory-lease integration, the intervenor knows
//! exactly what it commanded; sustained divergence between the commanded
//! trajectory and proprioception means either the envelope clamped it or
//! someone else is writing. Detection converts silent corruption into a
//! loud, diagnosable incident — the most an advisory lease can honestly
//! offer.
//!
//! Pure: feed (commanded, observed) pairs with timestamps; get a verdict
//! when divergence is sustained for the whole window.

use waddle_types::MonoNs;

#[derive(Debug, Clone, PartialEq)]
pub struct DualWriteVerdict {
    /// Mean L2 divergence over the sustained window.
    pub divergence_metric: f64,
    pub window_ns: i64,
}

#[derive(Debug)]
pub struct DivergenceDetector {
    threshold: f64,
    window_ns: i64,
    /// Start of the current sustained-divergence run.
    run_start: Option<MonoNs>,
    run_sum: f64,
    run_samples: u32,
    fired: bool,
}

impl DivergenceDetector {
    #[must_use]
    pub fn new(threshold: f64, window_ns: i64) -> Self {
        Self {
            threshold,
            window_ns: window_ns.max(1),
            run_start: None,
            run_sum: 0.0,
            run_samples: 0,
            fired: false,
        }
    }

    /// Feed one (commanded, observed) pair. Returns a verdict once
    /// divergence has been sustained for the full window (fires once; reset
    /// with [`Self::reset`]).
    pub fn feed(
        &mut self,
        commanded: &[f64],
        observed: &[f64],
        at: MonoNs,
    ) -> Option<DualWriteVerdict> {
        if self.fired {
            return None;
        }
        let n = commanded.len().min(observed.len());
        if n == 0 {
            return None;
        }
        let l2: f64 = commanded[..n]
            .iter()
            .zip(&observed[..n])
            .map(|(c, o)| (c - o) * (c - o))
            .sum::<f64>()
            .sqrt();

        if l2 <= self.threshold {
            self.run_start = None;
            self.run_sum = 0.0;
            self.run_samples = 0;
            return None;
        }

        let start = *self.run_start.get_or_insert(at);
        self.run_sum += l2;
        self.run_samples += 1;
        if at.0 - start.0 >= self.window_ns {
            self.fired = true;
            return Some(DualWriteVerdict {
                divergence_metric: self.run_sum / f64::from(self.run_samples),
                window_ns: at.0 - start.0,
            });
        }
        None
    }

    /// Re-arm after the incident is handled.
    pub fn reset(&mut self) {
        self.fired = false;
        self.run_start = None;
        self.run_sum = 0.0;
        self.run_samples = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transient_divergence_does_not_fire() {
        let mut d = DivergenceDetector::new(0.05, 100_000_000);
        assert!(d.feed(&[1.0], &[2.0], MonoNs(0)).is_none());
        // Converges again: run resets.
        assert!(d.feed(&[1.0], &[1.01], MonoNs(50_000_000)).is_none());
        assert!(d.feed(&[1.0], &[2.0], MonoNs(100_000_000)).is_none());
        assert!(d.feed(&[1.0], &[2.0], MonoNs(150_000_000)).is_none());
    }

    #[test]
    fn sustained_divergence_fires_once_with_trace_stats() {
        let mut d = DivergenceDetector::new(0.05, 100_000_000);
        assert!(d.feed(&[1.0], &[1.5], MonoNs(0)).is_none());
        assert!(d.feed(&[1.0], &[1.6], MonoNs(50_000_000)).is_none());
        let verdict = d.feed(&[1.0], &[1.7], MonoNs(100_000_000)).unwrap();
        assert!(verdict.divergence_metric > 0.4);
        assert_eq!(verdict.window_ns, 100_000_000);
        // Fires once until reset.
        assert!(d.feed(&[1.0], &[9.0], MonoNs(200_000_000)).is_none());
        d.reset();
        assert!(d.feed(&[1.0], &[9.0], MonoNs(300_000_000)).is_none()); // new run starts
    }
}
