//! Evaluation and acknowledged progression of a caller-supplied checked curve.
//! No path search, fitting, geometry policy or device authority lives here.

use super::{Feedback, PlaybackError};

/// Piecewise polynomials in normalized segment time, increasing power order.
/// Rows describe only the parent policy's selected coordinates. Exact boundary
/// positions are retained separately to avoid rounding an admitted endpoint.
#[derive(Clone, Debug)]
pub struct PacedCurve {
    pub times_s: Vec<f64>,
    pub positions: Vec<Vec<f64>>,
    /// segment → polynomial power (zero through five) → selected coordinate.
    pub coefficients: Vec<Vec<Vec<f64>>>,
    pub period_ns: i64,
    pub position_error: Vec<f64>,
    pub tracking_tolerance: Vec<f64>,
    /// Optional exact nominal samples, including every required waypoint.
    /// Without a grid, each acknowledgement advances by at most one period.
    pub sample_times_s: Option<Vec<f64>>,
}

#[derive(Clone, Debug, Default)]
pub struct CurveProgress {
    pub acknowledged_curve_time_s: Option<f64>,
    pub next_curve_time_s: Option<f64>,
    pub endpoint_acknowledged: bool,
    pub limited_ticks: usize,
    pub partial_ticks: usize,
    pub held_ticks: usize,
}

impl PacedCurve {
    pub(crate) fn validate(&self, width: usize) -> Result<(), PlaybackError> {
        let invalid = PlaybackError::Invalid;
        if !(2..=16_385).contains(&self.times_s.len())
            || self.positions.len() != self.times_s.len()
            || self.coefficients.len() + 1 != self.times_s.len()
            || self.times_s[0] != 0.0
            || self.times_s.iter().any(|t| !t.is_finite())
            || self.times_s.windows(2).any(|t| t[1] <= t[0])
            || self.duration_s() > 120.0
            || !(100_000..=1_000_000_000).contains(&self.period_ns)
        {
            return Err(invalid("invalid bounded polynomial timing"));
        }
        for rows in [&self.position_error, &self.tracking_tolerance] {
            if rows.len() != width || rows.iter().any(|x| !x.is_finite() || *x <= 0.0) {
                return Err(invalid("requires positive per-coordinate pacing limits"));
            }
        }
        if self
            .positions
            .iter()
            .any(|q| q.len() != width || q.iter().any(|x| !x.is_finite()))
            || self.coefficients.iter().any(|segment| {
                segment.len() != 6
                    || segment
                        .iter()
                        .any(|q| q.len() != width || q.iter().any(|x| !x.is_finite()))
            })
        {
            return Err(invalid("invalid polynomial dimensions or values"));
        }
        for (index, coefficients) in self.coefficients.iter().enumerate() {
            for coordinate in 0..width {
                let start = coefficients[0][coordinate];
                let end = coefficients.iter().map(|c| c[coordinate]).sum::<f64>();
                if !end.is_finite()
                    || (start - self.positions[index][coordinate]).abs() > 1e-9
                    || (end - self.positions[index + 1][coordinate]).abs() > 1e-9
                {
                    return Err(invalid("polynomial does not meet its declared boundary"));
                }
            }
        }
        if let Some(grid) = &self.sample_times_s {
            if !(2..=16_385).contains(&grid.len())
                || grid[0] != 0.0
                || grid.last().copied() != Some(self.duration_s())
                || grid.iter().any(|t| !t.is_finite())
                || grid
                    .windows(2)
                    .any(|t| t[1] <= t[0] || t[1] - t[0] > self.period_ns as f64 * 1e-9 + 1e-12)
            {
                return Err(invalid("invalid exact polynomial sample grid"));
            }
            // Bisection on a nominal grid must stay inside one monotone segment.
            // Its independent coordinates may have different signs; each
            // normalized quintic is the same rest-to-rest scalar progression.
            if self
                .times_s
                .iter()
                .any(|t| grid.binary_search_by(|v| v.total_cmp(t)).is_err())
            {
                return Err(invalid("sample grid must retain every curve boundary"));
            }
            for (index, c) in self.coefficients.iter().enumerate() {
                for coordinate in 0..width {
                    let delta =
                        self.positions[index + 1][coordinate] - self.positions[index][coordinate];
                    let expected = [
                        self.positions[index][coordinate],
                        0.0,
                        0.0,
                        10.0 * delta,
                        -15.0 * delta,
                        6.0 * delta,
                    ];
                    if c.iter()
                        .zip(expected)
                        .any(|(row, value)| row[coordinate] != value)
                    {
                        return Err(invalid(
                            "sample-grid curves require exact rest-to-rest quintics",
                        ));
                    }
                }
            }
        }
        // Every finite curve ends at rest; never hide a moving final state by
        // extending it as a stationary position command.
        let (_, velocity) = self.polynomial(self.coefficients.len() - 1, 1.0);
        if velocity.iter().any(|v| !v.is_finite() || v.abs() > 1e-9) {
            return Err(invalid("paced curve must end at rest"));
        }
        Ok(())
    }

    pub(crate) fn duration_s(&self) -> f64 {
        *self.times_s.last().expect("validated curve")
    }

    fn polynomial(&self, segment: usize, u: f64) -> (Vec<f64>, Vec<f64>) {
        let c = &self.coefficients[segment];
        let duration = self.times_s[segment + 1] - self.times_s[segment];
        let width = c[0].len();
        let mut position = c[5].clone();
        let mut velocity = vec![0.0; width];
        for power in (0..5).rev() {
            for coordinate in 0..width {
                position[coordinate] = position[coordinate] * u + c[power][coordinate];
            }
        }
        for power in (1..6).rev() {
            for coordinate in 0..width {
                velocity[coordinate] =
                    velocity[coordinate] * u + power as f64 * c[power][coordinate] / duration;
            }
        }
        (position, velocity)
    }

    pub(crate) fn sample(&self, time_s: f64) -> (Vec<f64>, Vec<f64>) {
        if time_s >= self.duration_s() {
            return (
                self.positions.last().unwrap().clone(),
                vec![0.0; self.positions[0].len()],
            );
        }
        let segment = self
            .times_s
            .partition_point(|t| *t <= time_s)
            .saturating_sub(1);
        let u =
            (time_s - self.times_s[segment]) / (self.times_s[segment + 1] - self.times_s[segment]);
        let (mut q, v) = self.polynomial(segment, u);
        if u == 0.0 {
            q.clone_from(&self.positions[segment]);
        }
        (q, v)
    }
}

#[derive(Debug)]
pub(crate) struct CurveCursor {
    pub spec: PacedCurve,
    pub progress: CurveProgress,
    pub proposed_s: f64,
    grid_index: usize,
}

impl CurveCursor {
    pub fn check_tracking(
        &self,
        feedback: &Feedback,
        indices: &[usize],
    ) -> Result<(), PlaybackError> {
        let previous = self
            .spec
            .sample(self.progress.acknowledged_curve_time_s.unwrap_or(0.0));
        if indices
            .iter()
            .zip(&previous.0)
            .zip(&self.spec.tracking_tolerance)
            .any(|((i, q), limit)| (feedback.positions[*i] - q).abs() > *limit)
        {
            return Err(PlaybackError::Tracking);
        }
        Ok(())
    }
    pub fn new(spec: PacedCurve) -> Self {
        Self {
            spec,
            progress: CurveProgress {
                next_curve_time_s: Some(0.0),
                ..Default::default()
            },
            proposed_s: 0.0,
            grid_index: 0,
        }
    }

    pub fn propose(
        &mut self,
        feedback: &Feedback,
        indices: &[usize],
    ) -> Result<(Vec<f64>, Vec<f64>), PlaybackError> {
        let previous_s = self.progress.acknowledged_curve_time_s.unwrap_or(0.0);
        let pending_s = self.progress.next_curve_time_s.unwrap_or(previous_s);
        let measured: Vec<_> = indices.iter().map(|i| feedback.positions[*i]).collect();
        let previous = self.spec.sample(previous_s);
        let fits = |q: &[f64], margin: f64| {
            q.iter()
                .zip(&measured)
                .zip(&self.spec.position_error)
                .all(|((q, m), cap)| (q - m).abs() <= cap * margin + 1e-12)
        };
        let mut candidate = self.spec.sample(pending_s);
        self.proposed_s = pending_s;
        if !fits(&candidate.0, 0.9) {
            self.progress.limited_ticks += 1;
            if !fits(&previous.0, 1.0) {
                return Err(PlaybackError::Tracking);
            }
            self.proposed_s = previous_s;
            candidate = previous;
            if fits(&candidate.0, 0.9) {
                let (mut low, mut high) = (previous_s, pending_s);
                for _ in 0..24 {
                    let middle = (low + high) / 2.0;
                    let sample = self.spec.sample(middle);
                    if fits(&sample.0, 0.9) {
                        low = middle;
                        candidate = sample;
                    } else {
                        high = middle;
                    }
                }
                self.proposed_s = low;
            }
            if self.proposed_s - previous_s <= (self.spec.period_ns as f64 * 1e-15).max(1e-9) {
                self.progress.held_ticks += 1;
                self.proposed_s = previous_s;
                candidate = self.spec.sample(previous_s);
            } else {
                self.progress.partial_ticks += 1;
            }
        }
        // Hint follows actual curve progress, not elapsed wall time. A repeated
        // reference has zero feedforward; a late callback cannot accelerate it.
        let scale = (self.proposed_s - previous_s) / (self.spec.period_ns as f64 * 1e-9);
        for velocity in &mut candidate.1 {
            *velocity *= scale;
        }
        if candidate
            .0
            .iter()
            .chain(&candidate.1)
            .any(|v| !v.is_finite())
        {
            return Err(PlaybackError::Invalid("polynomial evaluation is nonfinite"));
        }
        Ok(candidate)
    }

    pub fn acknowledge(&mut self) {
        self.progress.acknowledged_curve_time_s = Some(self.proposed_s);
        if self.proposed_s >= self.spec.duration_s() {
            self.progress.endpoint_acknowledged = true;
            self.progress.next_curve_time_s = None;
        } else if let Some(grid) = &self.spec.sample_times_s {
            if self.proposed_s == grid[self.grid_index] {
                self.grid_index += 1;
            }
            self.progress.next_curve_time_s = Some(grid[self.grid_index]);
        } else {
            self.progress.next_curve_time_s = Some(
                (self.proposed_s + self.spec.period_ns as f64 * 1e-9).min(self.spec.duration_s()),
            );
        }
    }
}
