//! The two-clock discipline, as types.
//!
//! Every stream timestamp is session-monotonic nanoseconds ([`MonoNs`]).
//! Wall-clock location comes from a [`ClockAnchor`] captured atomically; where
//! a record needs both clocks, the epoch twin is captured AT STAMP TIME as a
//! [`Stamp`] — never derived later (a host suspend between stamp and
//! conversion silently corrupts offsets; this cost a production postmortem).
//!
//! Only `waddle-ingest::SessionClock` mints [`Stamp`]s in production code;
//! tests use `waddle-ingest::FakeClock`. The workspace clippy configuration
//! (`disallowed-methods`) enforces this.

/// Session-monotonic nanoseconds.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct MonoNs(pub i64);

/// Unix-epoch nanoseconds.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct EpochNs(pub i64);

impl MonoNs {
    #[must_use]
    pub fn saturating_add(self, ns: i64) -> Self {
        Self(self.0.saturating_add(ns))
    }

    #[must_use]
    pub fn saturating_sub_ns(self, other: Self) -> i64 {
        self.0.saturating_sub(other.0)
    }
}

/// A dual-clock timestamp: the monotonic stamp and its epoch twin, captured
/// together at stamp time. Fields are private on purpose — construction goes
/// through a [`Clock`] implementation, not ad-hoc pairing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Stamp {
    mono: MonoNs,
    epoch: EpochNs,
}

impl Stamp {
    #[must_use]
    pub fn mono_ns(self) -> MonoNs {
        self.mono
    }

    #[must_use]
    pub fn epoch_ns(self) -> EpochNs {
        self.epoch
    }

    /// Construct a `Stamp` from raw parts, bypassing the clock discipline.
    ///
    /// Reserved for `waddle-ingest`'s clock implementations and test
    /// fixtures; banned everywhere else via clippy `disallowed-methods`.
    #[doc(hidden)]
    #[must_use]
    pub fn from_parts_unchecked(mono: MonoNs, epoch: EpochNs) -> Self {
        Self { mono, epoch }
    }
}

/// Pairs the session-monotonic clock with the wall clock, captured atomically
/// (paired reads, minimum inter-read delta) at session start and at every
/// recording-file open.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ClockAnchor {
    pub monotonic_ns: MonoNs,
    pub unix_ns: EpochNs,
}

impl ClockAnchor {
    /// Locate a monotonic stamp on the wall clock via this anchor.
    ///
    /// For RECORDING a dual-clock value, capture a [`Stamp`] at stamp time
    /// instead — this method is for read-side location of historical data.
    #[must_use]
    pub fn locate(&self, t: MonoNs) -> EpochNs {
        EpochNs(t.0 + (self.unix_ns.0 - self.monotonic_ns.0))
    }

    pub fn to_pb(self) -> crate::pb::v0::ClockAnchor {
        crate::pb::v0::ClockAnchor {
            monotonic_ns: self.monotonic_ns.0,
            unix_ns: self.unix_ns.0,
        }
    }

    pub fn from_pb(pb: &crate::pb::v0::ClockAnchor) -> Self {
        Self {
            monotonic_ns: MonoNs(pb.monotonic_ns),
            unix_ns: EpochNs(pb.unix_ns),
        }
    }
}

/// The stamp minter. Implemented by `waddle-ingest::SessionClock` (production,
/// reads OS clocks) and `waddle-ingest::FakeClock` (tests, manually advanced).
/// Hot-path consumers are generic over `C: Clock` so the discipline is
/// monomorphized away.
pub trait Clock: Send + Sync + 'static {
    fn stamp_now(&self) -> Stamp;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn anchor_locates_monotonic_on_wall_clock() {
        let anchor = ClockAnchor {
            monotonic_ns: MonoNs(1_000),
            unix_ns: EpochNs(1_780_000_000_000_000_000),
        };
        assert_eq!(
            anchor.locate(MonoNs(2_000)),
            EpochNs(1_780_000_000_000_001_000)
        );
    }

    #[cfg(feature = "serde")]
    #[test]
    #[allow(clippy::disallowed_methods)] // test fixture construction
    fn stamp_serde_round_trip() {
        let s = Stamp::from_parts_unchecked(MonoNs(42), EpochNs(1_780_000_000_000_000_042));
        let json = serde_json::to_string(&s).unwrap();
        let back: Stamp = serde_json::from_str(&json).unwrap();
        assert_eq!(s, back);
    }
}
