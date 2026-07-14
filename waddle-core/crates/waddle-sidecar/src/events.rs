//! The bounded in-memory event ring and incident persistence.
//!
//! The ring holds the most recent episode events so an incident handler can
//! persist a window around a fault ("what led up to this") without keeping
//! the whole stream in memory.

use std::collections::VecDeque;

use parking_lot::Mutex;
use waddle_types::pb::v0 as pb;

use crate::error::SidecarError;

/// A bounded FIFO of episode events; pushing onto a full ring drops the
/// oldest. Thread-safe (interior mutex): producers push, the incident path
/// drains.
#[derive(Debug)]
pub struct EventRing {
    capacity: usize,
    inner: Mutex<VecDeque<pb::EpisodeEvent>>,
}

impl EventRing {
    /// `capacity` is clamped to at least 1.
    #[must_use]
    pub fn new(capacity: usize) -> Self {
        let capacity = capacity.max(1);
        Self {
            capacity,
            inner: Mutex::new(VecDeque::with_capacity(capacity)),
        }
    }

    pub fn push(&self, event: pb::EpisodeEvent) {
        let mut q = self.inner.lock();
        if q.len() == self.capacity {
            q.pop_front();
        }
        q.push_back(event);
    }

    /// Take everything currently buffered, oldest first.
    #[must_use]
    pub fn drain(&self) -> Vec<pb::EpisodeEvent> {
        self.inner.lock().drain(..).collect()
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.inner.lock().len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.inner.lock().is_empty()
    }
}

/// Persists an incident window `(t_start_ns, t_end_ns)` of buffered data
/// and returns the [`pb::ArchiveRef`] to record in the sidecar's
/// `incident_clips`.
pub trait IncidentPersist {
    fn persist(&self, range: (i64, i64), cause: &str) -> Result<pb::ArchiveRef, SidecarError>;
}

/// The no-op default: persists nothing and returns a ref whose resolver is
/// `"none"`. The time range and cause are still recorded, so the sidecar
/// says honestly that an incident happened and its bytes were not kept.
#[derive(Debug, Clone, Copy, Default)]
pub struct NullIncidentPersist;

impl IncidentPersist for NullIncidentPersist {
    fn persist(&self, range: (i64, i64), cause: &str) -> Result<pb::ArchiveRef, SidecarError> {
        Ok(pb::ArchiveRef {
            ref_id: format!("incident-{cause}-{}", range.0),
            stream_id: String::new(),
            t_start_ns: range.0,
            t_end_ns: range.1,
            content_hash: String::new(),
            resolver: "none".to_owned(),
            uri_hint: String::new(),
            media_type: String::new(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn event(t_ns: i64) -> pb::EpisodeEvent {
        pb::EpisodeEvent {
            t_ns,
            ..Default::default()
        }
    }

    #[test]
    fn ring_drops_oldest_when_full() {
        let ring = EventRing::new(3);
        for t in 0..5 {
            ring.push(event(t));
        }
        assert_eq!(ring.len(), 3);
        let drained = ring.drain();
        assert_eq!(
            drained.iter().map(|e| e.t_ns).collect::<Vec<_>>(),
            [2, 3, 4]
        );
        assert!(ring.is_empty());
    }

    #[test]
    fn null_persist_returns_a_none_resolver_ref() {
        let r = NullIncidentPersist
            .persist((100, 200), "dual_write")
            .unwrap();
        assert_eq!(r.resolver, "none");
        assert_eq!((r.t_start_ns, r.t_end_ns), (100, 200));
        assert!(r.ref_id.contains("dual_write"));
    }
}
