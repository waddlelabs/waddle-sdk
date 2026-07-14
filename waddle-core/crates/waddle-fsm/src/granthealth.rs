//! Live grant health (FSM.md §7; N6/N11).
//!
//! Health is inferred from proxy signals, never from live verb invocation.
//! Signal-driven demotion is DEFERRED to the next planning boundary (episode
//! end) — it never interrupts an active lease. Partition-driven demotion is
//! immediate (the plane is gone; the grant is unusable, not merely slow).
//! Re-promotion requires sustained recovery below the hysteresis band, or a
//! server-directed change.

use waddle_types::{Grant, GrantStatus, Verb};

use crate::event::ProxySample;

/// What one proxy sample did to a grant's health.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthEvent {
    /// Demotion decided; deferred to the next planning boundary.
    DemotePending,
    /// Sustained recovery below the hysteresis band re-promoted the grant.
    Repromoted,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct GrantHealthEntry {
    pub verb: Verb,
    pub status: GrantStatus,
    /// Demotion decided but deferred to the next planning boundary.
    pub pending_demote: Option<String>,
    pub declared_bound_ns: Option<i64>,
    /// Set when the demotion came from a partition (auto re-promotes on
    /// reconnect).
    pub partition_demoted: bool,
    bad_streak: u32,
    good_streak: u32,
}

impl GrantHealthEntry {
    #[must_use]
    pub fn from_grant(g: &Grant) -> Self {
        Self {
            verb: g.verb,
            status: GrantStatus::Active,
            pending_demote: None,
            declared_bound_ns: g.declared_latency_bound_ns,
            partition_demoted: false,
            bad_streak: 0,
            good_streak: 0,
        }
    }

    /// Feed one proxy sample.
    pub fn observe(
        &mut self,
        sample: &ProxySample,
        demote_after: u32,
        promote_after: u32,
        hysteresis_ratio: f64,
    ) -> Option<HealthEvent> {
        let bound = self.declared_bound_ns?;
        let observed = sample.gate_tick_p95_ns.max(sample.callback_dispatch_p95_ns);
        #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
        let good_band = (bound as f64 * hysteresis_ratio) as i64;

        if observed > bound {
            self.bad_streak += 1;
            self.good_streak = 0;
            if self.status == GrantStatus::Active
                && self.pending_demote.is_none()
                && self.bad_streak >= demote_after
            {
                self.pending_demote = Some(format!(
                    "proxy signal {observed}ns exceeded declared bound {bound}ns"
                ));
                return Some(HealthEvent::DemotePending);
            }
        } else if observed < good_band {
            // Below the hysteresis band: counts toward recovery.
            self.good_streak += 1;
            self.bad_streak = 0;
            self.pending_demote = None;
            if self.status == GrantStatus::Demoted
                && !self.partition_demoted
                && self.good_streak >= promote_after
            {
                self.status = GrantStatus::Active;
                self.good_streak = 0;
                return Some(HealthEvent::Repromoted);
            }
        } else {
            // Hovering at the bound: neither bad enough to demote further
            // nor good enough to recover — hysteresis holds the line.
            self.bad_streak = 0;
            self.good_streak = 0;
        }
        None
    }

    /// Apply a pending demotion at a planning boundary. Returns the reason
    /// when a demotion was applied.
    pub fn apply_pending(&mut self) -> Option<String> {
        let reason = self.pending_demote.take()?;
        self.status = GrantStatus::Demoted;
        self.good_streak = 0;
        Some(reason)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(bound: i64) -> GrantHealthEntry {
        GrantHealthEntry::from_grant(&Grant {
            verb: Verb::Hold,
            send_interfaces: vec![],
            declared_latency_bound_ns: Some(bound),
            hardware: false,
        })
    }

    fn sample(ns: i64) -> ProxySample {
        ProxySample {
            gate_tick_p95_ns: ns,
            ..Default::default()
        }
    }

    #[test]
    fn demotion_is_deferred_not_immediate() {
        let mut e = entry(1_000_000);
        let ev = e.observe(&sample(2_000_000), 1, 3, 0.8);
        assert_eq!(ev, Some(HealthEvent::DemotePending));
        assert_eq!(e.status, GrantStatus::Active); // not yet applied
        assert!(e.apply_pending().is_some());
        assert_eq!(e.status, GrantStatus::Demoted);
    }

    #[test]
    fn hovering_at_the_bound_does_not_flap() {
        let mut e = entry(1_000_000);
        e.observe(&sample(2_000_000), 1, 3, 0.8);
        e.apply_pending();
        // 0.9 * bound: inside the hysteresis band — no recovery credit.
        for _ in 0..10 {
            assert_eq!(e.observe(&sample(900_000), 1, 3, 0.8), None);
        }
        assert_eq!(e.status, GrantStatus::Demoted);
        // Well below the band, three consecutive samples: recovers.
        assert_eq!(e.observe(&sample(100_000), 1, 3, 0.8), None);
        assert_eq!(e.observe(&sample(100_000), 1, 3, 0.8), None);
        assert_eq!(
            e.observe(&sample(100_000), 1, 3, 0.8),
            Some(HealthEvent::Repromoted)
        );
        assert_eq!(e.status, GrantStatus::Active);
    }
}
