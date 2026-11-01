//! Measured completion of a supplied goal, independent of the commanded endpoint.

use super::{Feedback, PlaybackError, PlaybackPolicy};

/// The SDK owner's TCP, in its declared base frame, from the same joint sample.
#[derive(Clone, Debug)]
pub struct Pose {
    pub position_m: [f64; 3],
    pub quaternion_wxyz: [f64; 4],
    pub frame_id: String,
}

impl Pose {
    pub fn valid(&self) -> bool {
        !self.frame_id.is_empty()
            && self.position_m.iter().all(|v| v.is_finite())
            && self.quaternion_wxyz.iter().all(|v| v.is_finite())
            && (self.quaternion_wxyz.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() <= 2.1e-6
    }
}

/// Exactly one original joint or Cartesian goal; this performs no planning.
#[derive(Clone, Debug)]
pub struct PlaybackGoal {
    pub joint_positions: Option<Vec<f64>>,
    pub pose: Option<Pose>,
    pub position_tolerance_m: f64,
    /// None means position-only completion.
    pub orientation_tolerance_rad: Option<f64>,
}

impl PlaybackGoal {
    pub(super) fn validate(&self, policy: &PlaybackPolicy) -> Result<(), PlaybackError> {
        let valid = match (&self.joint_positions, &self.pose) {
            (Some(q), None) => q.len() == policy.indices.len() && q.iter().all(|v| v.is_finite()),
            (None, Some(pose)) => {
                pose.valid()
                    && self.position_tolerance_m.is_finite()
                    && self.position_tolerance_m > 0.0
                    && self
                        .orientation_tolerance_rad
                        .is_none_or(|v| v.is_finite() && v > 0.0)
            }
            _ => false,
        };
        if !valid || policy.reference_only || policy.gripper.is_some() {
            return Err(PlaybackError::Invalid(
                "measured completion requires exactly one valid joint or TCP goal",
            ));
        }
        Ok(())
    }

    pub(super) fn reached(
        &self,
        measured: &Feedback,
        policy: &PlaybackPolicy,
    ) -> Result<bool, PlaybackError> {
        if let Some(target) = &self.joint_positions {
            return Ok(policy
                .indices
                .iter()
                .zip(target)
                .zip(&policy.position_tolerance)
                .all(|((i, q), tolerance)| (measured.positions[*i] - q).abs() <= *tolerance));
        }
        let target = self.pose.as_ref().expect("validated goal");
        let actual = measured
            .pose
            .as_ref()
            .ok_or(PlaybackError::InvalidFeedback)?;
        if !actual.valid() || actual.frame_id != target.frame_id {
            return Err(PlaybackError::InvalidFeedback);
        }
        let distance = actual
            .position_m
            .iter()
            .zip(&target.position_m)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt();
        let orientation_ok = self.orientation_tolerance_rad.is_none_or(|limit| {
            let norm = (actual.quaternion_wxyz.iter().map(|v| v * v).sum::<f64>()
                * target.quaternion_wxyz.iter().map(|v| v * v).sum::<f64>())
            .sqrt();
            let dot = actual
                .quaternion_wxyz
                .iter()
                .zip(&target.quaternion_wxyz)
                .map(|(a, b)| a * b)
                .sum::<f64>()
                / norm;
            let dot = dot.abs().min(1.0);
            2.0 * dot.acos() <= limit
        });
        Ok(distance <= self.position_tolerance_m && orientation_ok)
    }
}
