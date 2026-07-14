//! waddle-ingest — where observations enter the system and where ALL
//! timestamps are minted.
//!
//! This is the only crate in the workspace allowed to read OS clocks (the
//! workspace clippy `disallowed-methods` configuration enforces it). The
//! session timeline is monotonic nanoseconds from session start; the
//! [`SessionClock`] captures the wall-clock anchor atomically and mints
//! dual-clock [`Stamp`]s at stamp time — never derived later.

// The trusted crate: OS clock reads and Stamp construction live here, and
// only here. Every use is confined to `clock.rs`.
#![allow(clippy::disallowed_methods)]

pub mod clock;
pub mod hub;
pub mod offset;
pub mod ring;

pub use clock::{FakeClock, SessionClock};
pub use hub::{IngestHub, ObsFrame};
pub use offset::SourceOffsetEstimator;
pub use ring::{LatestSlot, sample_ring};

pub use waddle_types::time::{Clock, ClockAnchor, EpochNs, MonoNs, Stamp};
