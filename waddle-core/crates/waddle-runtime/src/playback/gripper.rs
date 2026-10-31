//! Locally paced jaw references and explicit measured completion conditions.

use std::collections::VecDeque;

use super::{Completion, Feedback, PlaybackError, PlaybackPolicy};

#[derive(Clone, Debug)]
pub struct MeasuredForce {
    pub force_n: f64,
    pub source: String,
    pub estimated: bool,
}

#[derive(Clone, Debug)]
pub struct ContactCondition {
    pub minimum_opening_m: f64,
    pub minimum_progress_m: f64,
    pub target_shortfall_m: f64,
    pub commanded_shortfall_m: f64,
    pub max_drift_m: f64,
    pub window_ns: i64,
}

#[derive(Clone, Debug)]
pub struct ForceCondition {
    /// Zero explicitly releases an existing force mode; completion then uses
    /// ordinary positional/contact conditions. Positive force needs feedback.
    pub target_n: f64,
    pub tolerance_n: f64,
    pub closed_tolerance_m: f64,
}

#[derive(Clone, Debug)]
pub struct GripperPolicy {
    pub period_ns: i64,
    pub position_error: f64,
    pub tracking_tolerance: f64,
    pub closed_position: f64,
    pub open_position: f64,
    pub closed_m: f64,
    pub open_m: f64,
    pub contact: Option<ContactCondition>,
    pub force: Option<ForceCondition>,
}

impl GripperPolicy {
    pub fn opening(&self, q: f64) -> f64 {
        self.closed_m
            + (q - self.closed_position) / (self.open_position - self.closed_position)
                * (self.open_m - self.closed_m)
    }

    pub fn force_grasp(&self) -> bool {
        self.force
            .as_ref()
            .is_some_and(|force| force.target_n > 0.0)
    }

    pub(super) fn validate(
        &self,
        policy: &PlaybackPolicy,
        start: f64,
        goal: f64,
    ) -> Result<(), PlaybackError> {
        let invalid = PlaybackError::Invalid;
        if policy.indices.len() != 1
            || policy.paced_curve.is_some()
            || policy.allow_settled
            || policy
                .feedback
                .as_ref()
                .is_some_and(|f| f.position_window.is_some())
            || !(100_000..=1_000_000_000).contains(&self.period_ns)
            || [self.position_error, self.tracking_tolerance]
                .iter()
                .any(|x| !x.is_finite() || *x <= 0.0)
            || [
                self.closed_position,
                self.open_position,
                self.closed_m,
                self.open_m,
            ]
            .iter()
            .any(|x| !x.is_finite())
            || self.closed_position == self.open_position
            || self.closed_m < 0.0
            || self.open_m <= self.closed_m
        {
            return Err(invalid(
                "gripper playback requires one mapped jaw, positive limits and no curve/window/miss completion",
            ));
        }
        if let Some(contact) = &self.contact
            && (self.opening(goal) >= self.opening(start)
                || self.force_grasp()
                || [
                    contact.minimum_opening_m,
                    contact.minimum_progress_m,
                    contact.target_shortfall_m,
                    contact.commanded_shortfall_m,
                    contact.max_drift_m,
                ]
                .iter()
                .any(|x| !x.is_finite() || *x <= 0.0)
                || contact.window_ns <= 0
                || contact.window_ns >= policy.timeout_ns)
        {
            return Err(invalid(
                "contact completion requires closing travel and positive bounded measurements",
            ));
        }
        if let Some(force) = &self.force
            && (!force.target_n.is_finite()
                || force.target_n < 0.0
                || !force.tolerance_n.is_finite()
                || force.tolerance_n < 0.0
                || (force.target_n > 0.0 && force.tolerance_n >= force.target_n)
                || !force.closed_tolerance_m.is_finite()
                || force.closed_tolerance_m <= 0.0
                || force.closed_tolerance_m >= self.open_m - self.closed_m)
        {
            return Err(invalid(
                "force completion requires a finite target, tolerance and empty-jaw exclusion",
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct GripperEvidence {
    pub target_opening_m: f64,
    pub measured_opening_m: f64,
    pub commanded_opening_m: Option<f64>,
    pub progress_m: f64,
    pub target_shortfall_m: f64,
    pub commanded_shortfall_m: Option<f64>,
    pub window_ns: i64,
    pub drift_m: f64,
    pub force_target_n: Option<f64>,
    pub force_samples: usize,
    pub arrival_samples: usize,
}

#[derive(Debug, Default)]
pub(super) struct GripperState {
    pub last_command: Option<f64>,
    pub nominal_complete: bool,
    samples: VecDeque<(i64, f64)>,
    force_samples: usize,
    arrival_samples: usize,
    pub evidence: Option<GripperEvidence>,
}

impl GripperState {
    pub fn reset_samples(&mut self) {
        self.samples.clear();
        self.force_samples = 0;
        self.arrival_samples = 0;
    }

    pub fn observe(
        &mut self,
        measured: &Feedback,
        policy: &PlaybackPolicy,
        start: f64,
        goal: f64,
        fresh: bool,
    ) -> Result<Option<Completion>, PlaybackError> {
        let spec = policy.gripper.as_ref().expect("gripper mode");
        let index = policy.indices[0];
        let q = measured.positions[index];
        let opening = spec.opening(q);
        let commanded = self.last_command.map(|q| spec.opening(q));
        if !opening.is_finite() || commanded.is_some_and(|x| !x.is_finite()) {
            return Err(PlaybackError::InvalidFeedback);
        }
        if !fresh {
            self.samples.clear();
            self.force_samples = 0;
            self.arrival_samples = 0;
        }
        let mut outcome = None;
        let mut failure = None;
        let stationary = measured.velocities[index].abs() < policy.stationary_velocity[0];
        let mut tracking_exempt = false;
        if let Some(force) = spec.force.as_ref().filter(|_| spec.force_grasp()) {
            if let Some(actual) = &measured.gripper_force {
                // A motor's effort at the mechanical stop is never a grasp.
                if opening <= spec.closed_m + force.closed_tolerance_m {
                    failure = Some(PlaybackError::EmptyGripper);
                } else {
                    self.force_samples = if fresh
                        && self.last_command.is_some()
                        && stationary
                        && actual.force_n >= force.target_n - force.tolerance_n
                    {
                        self.force_samples + 1
                    } else {
                        0
                    };
                    if self.force_samples >= 3 {
                        outcome = Some(Completion::ForceReached);
                    }
                    tracking_exempt = true;
                }
            } else {
                failure = Some(PlaybackError::ForceUnavailable);
            }
        }
        let progress = spec.opening(start) - opening;
        let shortfall = opening - spec.opening(goal);
        let demand = commanded.map(|commanded| opening - commanded);
        let mut span = 0;
        let mut drift = 0.0;
        if let Some(contact) = &spec.contact {
            if fresh && commanded.is_some() {
                self.samples.push_back((measured.observed_ns, opening));
                while self.samples.len() > 2
                    && self.samples[1].0 <= measured.observed_ns - contact.window_ns
                {
                    self.samples.pop_front();
                }
                if self.samples.len() > 16_385 {
                    return Err(PlaybackError::InvalidFeedback);
                }
            }
            if let (Some(first), Some(last)) = (self.samples.front(), self.samples.back()) {
                span = last.0 - first.0;
                let (low, high) = self
                    .samples
                    .iter()
                    .map(|row| row.1)
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), q| {
                        (lo.min(q), hi.max(q))
                    });
                drift = high - low;
            }
            tracking_exempt = opening >= contact.minimum_opening_m
                && progress >= contact.minimum_progress_m
                && shortfall > contact.target_shortfall_m
                && demand.is_some_and(|d| d >= contact.commanded_shortfall_m);
            if fresh && tracking_exempt && span >= contact.window_ns && drift <= contact.max_drift_m
            {
                outcome = Some(Completion::ContactStall);
            }
        }
        self.arrival_samples = if fresh
            && !spec.force_grasp()
            && self.nominal_complete
            && stationary
            && (q - goal).abs() <= policy.position_tolerance[0]
        {
            self.arrival_samples + 1
        } else {
            0
        };
        if self.arrival_samples >= 3 {
            outcome = Some(Completion::Arrived);
        }
        self.evidence = Some(GripperEvidence {
            target_opening_m: spec.opening(goal),
            measured_opening_m: opening,
            commanded_opening_m: commanded,
            progress_m: progress,
            target_shortfall_m: shortfall,
            commanded_shortfall_m: demand,
            window_ns: span,
            drift_m: drift,
            force_target_n: spec.force.as_ref().map(|f| f.target_n),
            force_samples: self.force_samples,
            arrival_samples: self.arrival_samples,
        });
        if let Some(error) = failure {
            return Err(error);
        }
        if !tracking_exempt
            && self
                .last_command
                .is_some_and(|last| (q - last).abs() > spec.tracking_tolerance)
        {
            return Err(PlaybackError::Tracking);
        }
        Ok(outcome)
    }

    pub fn reference(&self, measured: &Feedback, policy: &PlaybackPolicy, target: f64) -> f64 {
        let spec = policy.gripper.as_ref().expect("gripper mode");
        let index = policy.indices[0];
        let q = measured.positions[index];
        let delta = target - q;
        let reserve = if delta * measured.velocities[index] < 0.0 {
            measured.velocities[index].abs() * spec.period_ns as f64 * 1e-9
        } else {
            0.0
        };
        let headroom = if spec.force_grasp() { 0.5 } else { 0.9 };
        let cap = (headroom * spec.position_error - reserve).max(0.0);
        if delta.abs() <= cap {
            target
        } else {
            q + cap.copysign(delta)
        }
    }
}
