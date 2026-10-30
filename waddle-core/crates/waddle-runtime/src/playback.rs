//! Lossless finite joint-reference accounting for a local execution loop.
//!
//! This reuses the ordinary action chunk, without the intervention jitter
//! buffer's latest-wins shedding. Every reference must be dispatched through
//! the existing episode gate and owner envelope and acknowledged exactly once.
//! The caller supplies session-monotonic times and fresh measured feedback;
//! neither a receipt nor exhaustion of the buffer establishes physical arrival.
//! This module grants no authority, opens no device and performs no planning.

use waddle_types::{ActionChunk, Step};

/// Bounded completion/timing choices supplied with a finite reference buffer.
/// `indices` selects the moving rows for start/completion measurements; untouched
/// rows still cross the complete ordinary owner envelope on every dispatch.
#[derive(Clone, Debug)]
pub struct PlaybackPolicy {
    pub indices: Vec<usize>,
    pub start_tolerance: Vec<f64>,
    pub position_tolerance: Vec<f64>,
    pub stationary_velocity: Vec<f64>,
    pub settle_ns: i64,
    pub max_lateness_ns: i64,
    pub timeout_ns: i64,
    /// Permit a separately reported stationary miss, never an arrival claim.
    pub allow_settled: bool,
}

/// Measured feedback stamped on the same session clock as playback.
#[derive(Clone, Debug)]
pub struct Feedback {
    pub observed_ns: i64,
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Completion {
    Arrived,
    Settled,
}

#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum PlaybackError {
    #[error("invalid finite reference buffer: {0}")]
    Invalid(&'static str),
    #[error("measured start differs from the admitted first reference")]
    StaleStart,
    #[error("reference {index} is {lateness_ns} ns late")]
    LateReference { index: usize, lateness_ns: i64 },
    #[error("finite reference deadline expired")]
    Deadline,
    #[error("a reference has an unacknowledged dispatch; do not replay it")]
    DispatchPending,
    #[error("dispatch receipt does not match the outstanding reference")]
    InvalidAcknowledgement,
    #[error("invalid or non-monotonic measured feedback")]
    InvalidFeedback,
    #[error("finite execution cancelled: {0}")]
    Cancelled(String),
}

/// One immutable, fully received finite buffer. Positions are held between
/// references, without fitting, interpolation, retiming or reference shedding.
/// The action chunk must first have been validated against its joint-position
/// declaration by the ordinary action-space intake.
#[derive(Clone, Debug)]
pub struct FiniteReferences {
    chunk: ActionChunk,
    policy: PlaybackPolicy,
}

impl FiniteReferences {
    pub fn new(chunk: ActionChunk, policy: PlaybackPolicy) -> Result<Self, PlaybackError> {
        let invalid = PlaybackError::Invalid;
        if !(1..=16_385).contains(&chunk.steps.len()) {
            return Err(invalid("requires 1..=16385 references"));
        }
        let first = &chunk.steps[0];
        let width = first.values.len();
        if !(1..=128).contains(&width) || first.offset_ns != 0 {
            return Err(invalid("requires 1..=128 joints and a zero-offset start"));
        }
        let mut previous = -1;
        for step in &chunk.steps {
            if step.values.len() != width
                || step.values.iter().any(|v| !v.is_finite())
                || step.gripper.is_some()
                || step.part != first.part
                || step.offset_ns <= previous
                || step.offset_ns > 120_000_000_000
                || step.velocity_feedforward.as_ref().is_some_and(|velocity| {
                    velocity.len() != width || velocity.iter().any(|v| !v.is_finite())
                })
            {
                return Err(invalid(
                    "requires ordered finite joint references for one scope",
                ));
            }
            previous = step.offset_ns;
        }
        if chunk.horizon_ns != previous {
            return Err(invalid("horizon must equal the final reference offset"));
        }
        if chunk.steps.last().is_some_and(|step| {
            step.velocity_feedforward
                .as_ref()
                .is_some_and(|v| v.iter().any(|x| *x != 0.0))
        }) {
            return Err(invalid("final reference must end at rest"));
        }
        if policy.indices.is_empty()
            || policy.indices.iter().any(|i| *i >= width)
            || policy
                .indices
                .iter()
                .enumerate()
                .any(|(i, value)| policy.indices[..i].contains(value))
        {
            return Err(invalid("measurement indices must be unique declared rows"));
        }
        for values in [
            &policy.start_tolerance,
            &policy.position_tolerance,
            &policy.stationary_velocity,
        ] {
            if values.len() != policy.indices.len()
                || values.iter().any(|v| !v.is_finite() || *v <= 0.0)
            {
                return Err(invalid("requires positive per-joint tolerances"));
            }
        }
        if policy.settle_ns <= 0
            || policy.timeout_ns <= chunk.horizon_ns
            || policy.timeout_ns > 180_000_000_000
            || policy.settle_ns >= policy.timeout_ns - chunk.horizon_ns
            || !(0..=1_000_000_000).contains(&policy.max_lateness_ns)
        {
            return Err(invalid("invalid bounded settling, lateness or deadline"));
        }
        Ok(Self { chunk, policy })
    }

    #[must_use]
    pub fn reference(&self, index: usize) -> Option<&Step> {
        self.chunk.steps.get(index)
    }
}

/// The next local-loop action. `Dispatch` is permission to *attempt* the normal
/// gate/envelope path, not permission to write a device or proof of dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlaybackPoll {
    Dispatch(usize),
    WaitUntil(i64),
    Observe,
    Complete(Completion),
}

/// Per-buffer accounting. Instances cannot be resumed after a terminal error.
/// Outstanding uncertain writes stay uncertain; constructing another instance
/// does not establish permission to retry an old physical command.
#[derive(Debug)]
pub struct Playback {
    references: FiniteReferences,
    started_ns: i64,
    now_ns: i64,
    next: usize,
    pending: bool,
    acknowledged_ns: Option<i64>,
    observed_ns: Option<i64>,
    settled_since: Option<i64>,
    arrived_since: Option<i64>,
    completion: Option<Completion>,
    failure: Option<PlaybackError>,
}

impl Playback {
    #[must_use]
    pub fn new(references: FiniteReferences, started_ns: i64) -> Self {
        Self {
            references,
            started_ns,
            now_ns: started_ns,
            next: 0,
            pending: false,
            acknowledged_ns: None,
            observed_ns: None,
            settled_since: None,
            arrived_since: None,
            completion: None,
            failure: None,
        }
    }

    #[must_use]
    pub fn reference(&self, index: usize) -> Option<&Step> {
        self.references.reference(index)
    }

    #[must_use]
    pub fn dispatched(&self) -> usize {
        self.next
    }

    #[must_use]
    pub fn completion(&self) -> Option<Completion> {
        self.completion
    }

    pub fn cancel(&mut self, reason: &str) {
        if self.completion.is_none() && self.failure.is_none() {
            self.failure = Some(PlaybackError::Cancelled(reason.to_owned()));
        }
    }

    fn fail<T>(&mut self, error: PlaybackError) -> Result<T, PlaybackError> {
        self.failure = Some(error.clone());
        Err(error)
    }

    pub fn poll(
        &mut self,
        now_ns: i64,
        measured: &Feedback,
    ) -> Result<PlaybackPoll, PlaybackError> {
        if let Some(error) = &self.failure {
            return Err(error.clone());
        }
        if let Some(outcome) = self.completion {
            return Ok(PlaybackPoll::Complete(outcome));
        }
        if self.pending {
            return self.fail(PlaybackError::DispatchPending);
        }
        let width = self.references.chunk.steps[0].values.len();
        if now_ns < self.now_ns
            || measured.observed_ns > now_ns
            || measured.observed_ns < self.started_ns
            || self
                .observed_ns
                .is_some_and(|previous| measured.observed_ns < previous)
            || measured.positions.len() != width
            || measured.velocities.len() != width
            || measured
                .positions
                .iter()
                .chain(&measured.velocities)
                .any(|v| !v.is_finite())
        {
            return self.fail(PlaybackError::InvalidFeedback);
        }
        self.now_ns = now_ns;
        if now_ns.saturating_sub(self.started_ns) >= self.references.policy.timeout_ns {
            return self.fail(PlaybackError::Deadline);
        }
        let fresh = self.observed_ns != Some(measured.observed_ns);
        self.observed_ns = Some(measured.observed_ns);
        if self.next == 0
            && !within(
                &measured.positions,
                &self.references.chunk.steps[0].values,
                &self.references.policy.start_tolerance,
                &self.references.policy.indices,
            )
        {
            return self.fail(PlaybackError::StaleStart);
        }
        if let Some(step) = self.references.chunk.steps.get(self.next) {
            let due = self.started_ns.saturating_add(step.offset_ns);
            if now_ns < due {
                return Ok(PlaybackPoll::WaitUntil(due));
            }
            let lateness_ns = now_ns.saturating_sub(due);
            if lateness_ns > self.references.policy.max_lateness_ns {
                return self.fail(PlaybackError::LateReference {
                    index: self.next,
                    lateness_ns,
                });
            }
            if self
                .acknowledged_ns
                .is_some_and(|at| measured.observed_ns <= at)
            {
                return Ok(PlaybackPoll::Observe);
            }
            self.pending = true;
            return Ok(PlaybackPoll::Dispatch(self.next));
        }
        // Only measurements acquired after the final dispatch can settle it.
        if !fresh
            || self
                .acknowledged_ns
                .is_none_or(|at| measured.observed_ns <= at)
        {
            return Ok(PlaybackPoll::Observe);
        }
        let stationary = self
            .references
            .policy
            .indices
            .iter()
            .zip(&self.references.policy.stationary_velocity)
            .all(|(i, limit)| measured.velocities[*i].abs() <= *limit);
        let arrived = within(
            &measured.positions,
            &self
                .references
                .chunk
                .steps
                .last()
                .expect("validated nonempty buffer")
                .values,
            &self.references.policy.position_tolerance,
            &self.references.policy.indices,
        );
        if !stationary || (!arrived && !self.references.policy.allow_settled) {
            self.settled_since = None;
            self.arrived_since = None;
            return Ok(PlaybackPoll::Observe);
        }
        if arrived {
            self.arrived_since.get_or_insert(measured.observed_ns);
        } else {
            self.arrived_since = None;
        }
        let since = *self.settled_since.get_or_insert(measured.observed_ns);
        if measured.observed_ns - since < self.references.policy.settle_ns {
            return Ok(PlaybackPoll::Observe);
        }
        let outcome = if self
            .arrived_since
            .is_some_and(|since| measured.observed_ns - since >= self.references.policy.settle_ns)
        {
            Completion::Arrived
        } else {
            Completion::Settled
        };
        self.completion = Some(outcome);
        Ok(PlaybackPoll::Complete(outcome))
    }

    /// Record a confirmed dispatch only. On rejection or unknown outcome the
    /// owner stops playback and retains its original fault; it must not ack.
    pub fn acknowledge(&mut self, index: usize, now_ns: i64) -> Result<(), PlaybackError> {
        if let Some(error) = &self.failure {
            return Err(error.clone());
        }
        if self.completion.is_some() {
            return Err(PlaybackError::InvalidAcknowledgement);
        }
        if !self.pending || index != self.next || now_ns < self.now_ns {
            return self.fail(PlaybackError::InvalidAcknowledgement);
        }
        self.next += 1;
        self.pending = false;
        self.now_ns = now_ns;
        self.acknowledged_ns = Some(now_ns);
        Ok(())
    }
}

fn within(actual: &[f64], target: &[f64], tolerances: &[f64], indices: &[usize]) -> bool {
    indices
        .iter()
        .zip(tolerances)
        .all(|(i, tolerance)| (actual[*i] - target[*i]).abs() <= *tolerance)
}
