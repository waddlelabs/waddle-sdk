//! The session clock: the sole production minter of [`Stamp`]s.

use std::sync::Arc;
use std::sync::atomic::{AtomicI64, Ordering};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use waddle_types::time::{Clock, ClockAnchor, EpochNs, MonoNs, Stamp};

/// The production session clock. The session timeline starts at 0 when the
/// clock is captured; the anchor pairs that origin with the wall clock.
#[derive(Debug, Clone)]
pub struct SessionClock {
    origin: Instant,
    anchor: ClockAnchor,
}

impl SessionClock {
    /// Capture the monotonic↔wall anchor "atomically": several paired reads,
    /// keeping the pair with the minimum inter-read delta, so a scheduler
    /// preemption between the two reads cannot skew the anchor.
    #[must_use]
    pub fn capture() -> Self {
        let mut best: Option<(Instant, i64, u128)> = None;
        for _ in 0..5 {
            let m0 = Instant::now();
            let wall = SystemTime::now();
            let m1 = Instant::now();
            let delta = m1.duration_since(m0).as_nanos();
            let unix_ns = wall
                .duration_since(UNIX_EPOCH)
                .expect("system clock before the unix epoch")
                .as_nanos();
            #[allow(clippy::cast_possible_truncation)]
            let unix_ns = unix_ns as i64;
            if best.is_none_or(|(_, _, d)| delta < d) {
                best = Some((m0, unix_ns, delta));
            }
        }
        let (origin, unix_ns, _) = best.expect("at least one read");
        Self {
            origin,
            anchor: ClockAnchor {
                monotonic_ns: MonoNs(0),
                unix_ns: EpochNs(unix_ns),
            },
        }
    }

    #[must_use]
    pub fn anchor(&self) -> ClockAnchor {
        self.anchor
    }

    #[must_use]
    pub fn now(&self) -> MonoNs {
        #[allow(clippy::cast_possible_truncation)]
        MonoNs(self.origin.elapsed().as_nanos() as i64)
    }
}

impl Clock for SessionClock {
    fn stamp_now(&self) -> Stamp {
        let mono = self.now();
        Stamp::from_parts_unchecked(mono, self.anchor.locate(mono))
    }
}

/// A manually advanced clock for tests and conformance runs. Deterministic:
/// never touches the OS.
#[derive(Debug, Clone)]
pub struct FakeClock {
    mono: Arc<AtomicI64>,
    anchor: ClockAnchor,
}

impl FakeClock {
    #[must_use]
    pub fn new(start: MonoNs, unix_at_zero: EpochNs) -> Self {
        Self {
            mono: Arc::new(AtomicI64::new(start.0)),
            anchor: ClockAnchor {
                monotonic_ns: MonoNs(0),
                unix_ns: unix_at_zero,
            },
        }
    }

    pub fn advance(&self, ns: i64) {
        self.mono.fetch_add(ns, Ordering::SeqCst);
    }

    pub fn set(&self, t: MonoNs) {
        self.mono.store(t.0, Ordering::SeqCst);
    }

    #[must_use]
    pub fn anchor(&self) -> ClockAnchor {
        self.anchor
    }

    #[must_use]
    pub fn now(&self) -> MonoNs {
        MonoNs(self.mono.load(Ordering::SeqCst))
    }
}

impl Default for FakeClock {
    fn default() -> Self {
        Self::new(MonoNs(0), EpochNs(1_780_000_000_000_000_000))
    }
}

impl Clock for FakeClock {
    fn stamp_now(&self) -> Stamp {
        let mono = self.now();
        Stamp::from_parts_unchecked(mono, self.anchor.locate(mono))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn session_clock_is_monotone_and_anchored() {
        let clock = SessionClock::capture();
        let a = clock.stamp_now();
        let b = clock.stamp_now();
        assert!(b.mono_ns() >= a.mono_ns());
        // Epoch twin equals anchor-located mono by construction.
        assert_eq!(a.epoch_ns(), clock.anchor().locate(a.mono_ns()));
    }

    #[test]
    fn fake_clock_is_deterministic() {
        let clock = FakeClock::default();
        clock.advance(1_000);
        let s = clock.stamp_now();
        assert_eq!(s.mono_ns(), MonoNs(1_000));
        assert_eq!(s.epoch_ns(), EpochNs(1_780_000_000_000_001_000));
    }
}
