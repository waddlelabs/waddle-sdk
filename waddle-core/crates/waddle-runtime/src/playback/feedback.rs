//! Measured reference adaptation and encoder stability on the local worker.

use std::collections::VecDeque;

use waddle_types::Step;

use super::{Completion, Feedback, PlaybackError, PlaybackPolicy};

/// Unselected coordinates approach their unchanged targets as a group. Limits
/// are caller-supplied position errors, before the 90% execution headroom.
#[derive(Clone, Debug)]
pub struct HeldJointGroup {
    pub indices: Vec<usize>,
    pub position_error: Vec<f64>,
    pub period_ns: i64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum VelocityHints {
    #[default]
    Curve,
    TowardReference,
    /// Once measured position crosses the goal in the hinted direction, that
    /// coordinate's hint stays zero for the rest of this buffer.
    TowardGoal,
}

/// Encoder drift replaces instantaneous velocity as the stationarity measure.
/// The span is `PlaybackPolicy.settle_ns`; at least three fresh samples must
/// cover it. Optional pre-endpoint history is useful for stationary misses.
#[derive(Clone, Debug)]
pub struct PositionWindow {
    pub max_drift: Vec<f64>,
    pub max_gap_ns: Option<i64>,
    pub during_motion: bool,
    pub consecutive_samples: usize,
}

#[derive(Clone, Debug, Default)]
pub struct FeedbackPolicy {
    pub held_groups: Vec<HeldJointGroup>,
    /// Copy measured coordinates, e.g. a jaw already held by its local driver.
    /// This neither enables force control nor establishes contact.
    pub measured_indices: Vec<usize>,
    pub velocity_hints: VelocityHints,
    pub position_window: Option<PositionWindow>,
}

impl FeedbackPolicy {
    pub(super) fn validate(
        &self,
        policy: &PlaybackPolicy,
        width: usize,
    ) -> Result<(), PlaybackError> {
        let invalid = PlaybackError::Invalid;
        let mut used = policy.indices.clone();
        for (indices, limits) in self
            .held_groups
            .iter()
            .map(|g| (&g.indices, Some(g)))
            .chain(std::iter::once((&self.measured_indices, None)))
        {
            if let Some(group) = limits
                && (indices.is_empty()
                    || group.position_error.len() != indices.len()
                    || group
                        .position_error
                        .iter()
                        .any(|x| !x.is_finite() || *x <= 0.0)
                    || !(100_000..=1_000_000_000).contains(&group.period_ns))
            {
                return Err(invalid(
                    "held groups require positive row limits and a bounded period",
                ));
            }
            for index in indices {
                if *index >= width || used.contains(index) {
                    return Err(invalid(
                        "feedback rows must be unique and outside the moving rows",
                    ));
                }
                used.push(*index);
            }
        }
        if let Some(window) = &self.position_window
            && (window.max_drift.len() != policy.indices.len()
                || window.max_drift.iter().any(|x| !x.is_finite() || *x <= 0.0)
                || window
                    .max_gap_ns
                    .is_some_and(|gap| gap <= 0 || gap > policy.timeout_ns)
                || !(1..=16_385).contains(&window.consecutive_samples))
        {
            return Err(invalid(
                "position window requires positive drift, gap and sample bounds",
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct SettlingEvidence {
    pub samples: usize,
    pub span_ns: i64,
    pub joint_spans: Vec<f64>,
    pub raw_velocity_peak: f64,
    pub consecutive_samples: usize,
}

#[derive(Debug)]
pub(super) struct FeedbackState {
    stopped_hints: Vec<bool>,
    rows: VecDeque<Feedback>,
    consecutive: usize,
    pub evidence: Option<SettlingEvidence>,
    arrived_since: Option<i64>,
}

impl FeedbackState {
    pub fn reset_window(&mut self) {
        self.rows.clear();
        self.consecutive = 0;
        self.arrived_since = None;
        self.evidence = None;
    }

    pub fn new(width: usize) -> Self {
        Self {
            stopped_hints: vec![false; width],
            rows: VecDeque::new(),
            consecutive: 0,
            evidence: None,
            arrived_since: None,
        }
    }

    pub fn adapt(
        &mut self,
        step: &mut Step,
        measured: &Feedback,
        policy: &PlaybackPolicy,
        goal: &[f64],
    ) {
        let Some(spec) = &policy.feedback else { return };
        let mut values = step.values.to_vec();
        for i in &spec.measured_indices {
            values[*i] = measured.positions[*i];
        }
        for group in &spec.held_groups {
            let period_s = group.period_ns as f64 * 1e-9;
            let delta: Vec<_> = group
                .indices
                .iter()
                .map(|i| values[*i] - measured.positions[*i])
                .collect();
            let caps: Vec<_> = group
                .indices
                .iter()
                .zip(&group.position_error)
                .zip(&delta)
                .map(|((i, cap), delta)| {
                    let reserve = if delta * measured.velocities[*i] < 0.0 {
                        measured.velocities[*i].abs() * period_s
                    } else {
                        0.0
                    };
                    (0.9 * cap - reserve).max(0.0)
                })
                .collect();
            if delta.iter().zip(&caps).all(|(d, cap)| d.abs() <= *cap) {
                continue; // Retain exact target bits, including the endpoint.
            }
            let scale = delta
                .iter()
                .zip(&caps)
                .filter(|(d, _)| d.abs() > 1e-12)
                .map(|(d, cap)| cap / d.abs())
                .fold(1.0_f64, f64::min);
            for (i, delta) in group.indices.iter().zip(delta) {
                values[*i] = measured.positions[*i] + scale * delta;
            }
        }
        if let Some(hints) = &step.velocity_feedforward {
            let mut hints = hints.to_vec();
            for i in &policy.indices {
                match spec.velocity_hints {
                    VelocityHints::Curve => {}
                    VelocityHints::TowardReference => {
                        if (values[*i] - measured.positions[*i]) * hints[*i] <= 0.0 {
                            hints[*i] = 0.0;
                        }
                    }
                    VelocityHints::TowardGoal => {
                        self.stopped_hints[*i] |= hints[*i] != 0.0
                            && (goal[*i] - measured.positions[*i]) * hints[*i] <= 0.0;
                        if self.stopped_hints[*i] {
                            hints[*i] = 0.0;
                        }
                    }
                }
            }
            step.velocity_feedforward = Some(hints.into());
        }
        step.values = values.into();
    }

    /// Called once per fresh observed sample. A repeated sample resets evidence;
    /// decreasing timestamps are already a terminal error in playback intake.
    pub fn observe(
        &mut self,
        measured: &Feedback,
        policy: &PlaybackPolicy,
        arrived: bool,
        finished: bool,
        fresh: bool,
    ) -> Result<Option<Completion>, PlaybackError> {
        let Some(window) = policy
            .feedback
            .as_ref()
            .and_then(|s| s.position_window.as_ref())
        else {
            return Ok(None);
        };
        let eligible = (finished || window.during_motion) && (arrived || policy.allow_settled);
        if !fresh
            || !eligible
            || self.rows.back().is_some_and(|last| {
                window
                    .max_gap_ns
                    .is_some_and(|gap| measured.observed_ns - last.observed_ns > gap)
            })
        {
            self.reset_window();
        }
        if fresh && eligible {
            if arrived {
                self.arrived_since.get_or_insert(measured.observed_ns);
            } else {
                self.arrived_since = None;
            }
            self.rows.push_back(measured.clone());
            while self.rows.len() > 3
                && self.rows[1].observed_ns <= measured.observed_ns - policy.settle_ns
            {
                self.rows.pop_front();
            }
            if self.rows.len() > 16_385 {
                return Err(PlaybackError::InvalidFeedback);
            }
        }
        let spans: Vec<_> = policy
            .indices
            .iter()
            .map(|i| {
                let (low, high) = self
                    .rows
                    .iter()
                    .map(|row| row.positions[*i])
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), q| {
                        (lo.min(q), hi.max(q))
                    });
                if self.rows.is_empty() {
                    0.0
                } else {
                    high - low
                }
            })
            .collect();
        let span_ns = self
            .rows
            .front()
            .zip(self.rows.back())
            .map_or(0, |(first, last)| last.observed_ns - first.observed_ns);
        let stable = self.rows.len() >= 3
            && span_ns >= policy.settle_ns
            && spans
                .iter()
                .zip(&window.max_drift)
                .all(|(span, max)| span <= max);
        self.consecutive = if finished && eligible && fresh && stable {
            self.consecutive + 1
        } else {
            0
        };
        self.evidence = Some(SettlingEvidence {
            samples: self.rows.len(),
            span_ns,
            joint_spans: spans,
            raw_velocity_peak: self
                .rows
                .iter()
                .flat_map(|row| policy.indices.iter().map(|i| row.velocities[*i].abs()))
                .fold(0.0_f64, f64::max),
            consecutive_samples: self.consecutive,
        });
        Ok((self.consecutive >= window.consecutive_samples).then_some(
            if arrived
                && self
                    .arrived_since
                    .is_some_and(|since| measured.observed_ns - since >= policy.settle_ns)
            {
                Completion::Arrived
            } else {
                Completion::Settled
            },
        ))
    }
}
