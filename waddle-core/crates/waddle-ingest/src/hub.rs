//! The ingest hub: registers sources, stamps everything entering the system
//! onto the session timeline, and exposes latest-value snapshots.

use std::collections::HashMap;
use std::sync::Arc;

use parking_lot::{Mutex, RwLock};
use smallvec::SmallVec;
use waddle_types::time::{Clock, Stamp};
use waddle_types::{MonoNs, SourceId};

use crate::offset::SourceOffsetEstimator;
use crate::ring::LatestSlot;

/// A stamped observation frame: proprioception or a generic series sample.
/// Media payloads take the media plane; the hub carries the low-rate
/// numeric state the tripwires and the gate need.
#[derive(Debug, Clone)]
pub struct ObsFrame {
    pub source: SourceId,
    pub stamp: Stamp,
    pub values: SmallVec<[f64; 16]>,
}

#[derive(Debug)]
struct SourceEntry {
    offset: Mutex<SourceOffsetEstimator>,
    latest: LatestSlot<ObsFrame>,
}

/// Owns all timestamps: every sample entering through the hub is stamped by
/// the session clock (or mapped from its source clock via the per-source
/// offset estimator) at ingest time.
#[derive(Debug)]
pub struct IngestHub<C: Clock> {
    clock: C,
    sources: RwLock<HashMap<SourceId, Arc<SourceEntry>>>,
}

impl<C: Clock> IngestHub<C> {
    pub fn new(clock: C) -> Self {
        Self {
            clock,
            sources: RwLock::new(HashMap::new()),
        }
    }

    pub fn clock(&self) -> &C {
        &self.clock
    }

    pub fn register(&self, source: SourceId) {
        self.sources.write().entry(source).or_insert_with(|| {
            Arc::new(SourceEntry {
                offset: Mutex::new(SourceOffsetEstimator::default()),
                latest: LatestSlot::new(),
            })
        });
    }

    fn entry(&self, source: &SourceId) -> Option<Arc<SourceEntry>> {
        self.sources.read().get(source).cloned()
    }

    /// Ingest a sample stamped by the host at arrival ("the tap returned").
    pub fn push_now(&self, source: &SourceId, values: SmallVec<[f64; 16]>) -> Option<Stamp> {
        let entry = self.entry(source)?;
        let stamp = self.clock.stamp_now();
        entry.latest.publish(ObsFrame {
            source: source.clone(),
            stamp,
            values,
        });
        Some(stamp)
    }

    /// Ingest a sample carrying a foreign source timestamp: fold it into the
    /// offset estimator and map onto the session timeline. Falls back to the
    /// arrival stamp until the estimator warms up.
    pub fn push_with_source_ts(
        &self,
        source: &SourceId,
        source_ts_ns: i64,
        values: SmallVec<[f64; 16]>,
    ) -> Option<Stamp> {
        let entry = self.entry(source)?;
        let arrival = self.clock.stamp_now();
        let mapped: Option<MonoNs> = {
            let mut est = entry.offset.lock();
            est.observe(source_ts_ns, arrival.mono_ns());
            est.map(source_ts_ns)
        };
        // The epoch twin is captured at stamp time from the arrival stamp's
        // anchor relation — mapped mono, arrival-consistent epoch.
        let stamp = match mapped {
            Some(mono) => Stamp::from_parts_unchecked(
                mono,
                waddle_types::EpochNs(arrival.epoch_ns().0 - (arrival.mono_ns().0 - mono.0)),
            ),
            None => arrival,
        };
        entry.latest.publish(ObsFrame {
            source: source.clone(),
            stamp,
            values,
        });
        Some(stamp)
    }

    #[must_use]
    pub fn latest(&self, source: &SourceId) -> Option<Arc<ObsFrame>> {
        self.entry(source)?.latest.latest()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::FakeClock;
    use smallvec::smallvec;

    #[test]
    fn hub_stamps_at_ingest_and_serves_latest() {
        let clock = FakeClock::default();
        let hub = IngestHub::new(clock.clone());
        let src = SourceId::new("/robot/joints");
        hub.register(src.clone());

        clock.advance(5_000);
        hub.push_now(&src, smallvec![1.0, 2.0]);
        clock.advance(5_000);
        hub.push_now(&src, smallvec![3.0, 4.0]);

        let latest = hub.latest(&src).unwrap();
        assert_eq!(latest.stamp.mono_ns(), MonoNs(10_000));
        assert_eq!(latest.values.as_slice(), &[3.0, 4.0]);
    }

    #[test]
    fn source_timestamps_map_onto_session_timeline() {
        let clock = FakeClock::default();
        let hub = IngestHub::new(clock.clone());
        let src = SourceId::new("cam");
        hub.register(src.clone());

        // Source clock runs 1_000_000ns "behind" arrival.
        for i in 0..10i64 {
            clock.set(MonoNs(i * 10_000 + 1_000_000));
            hub.push_with_source_ts(&src, i * 10_000, smallvec![0.0]);
        }
        let latest = hub.latest(&src).unwrap();
        assert_eq!(latest.stamp.mono_ns(), MonoNs(9 * 10_000 + 1_000_000));
    }
}
