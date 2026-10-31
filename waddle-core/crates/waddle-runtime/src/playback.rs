//! Lossless finite joint-reference accounting for a local execution loop.
//!
//! This reuses the ordinary action chunk, without the intervention jitter
//! buffer's latest-wins shedding. Fixed offsets retain every input reference;
//! optional paced curves evaluate the supplied polynomial on acknowledgements.
//! Every dispatch crosses the existing episode gate and owner envelope.
//! The caller supplies session-monotonic times and fresh measured feedback;
//! neither a receipt nor exhaustion of the buffer establishes physical arrival.
//! This module grants no authority, opens no device and performs no planning.

use waddle_types::{ActionChunk, Step};

mod curve;
use curve::CurveCursor;
pub use curve::{CurveProgress, CurveUpdate, PacedCurve};
mod feedback;
use feedback::FeedbackState;
pub use feedback::{
    FeedbackPolicy, HeldJointGroup, PositionWindow, SettlingEvidence, VelocityHints,
};
mod gripper;
use gripper::GripperState;
pub use gripper::{
    ContactCondition, ForceCondition, GripperEvidence, GripperPolicy, MeasuredForce,
};
pub(crate) mod execution;
pub use execution::{
    CurveUpdateReceipt, ExecutionFailure, ExecutionHandle, ExecutionResult, ExecutionSnapshot,
    PlaybackIo, PlaybackIoError, ReferenceReceipt,
};

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
    /// Fresh eligible observations required in addition to the settling span.
    pub minimum_settle_samples: usize,
    pub max_lateness_ns: i64,
    pub timeout_ns: i64,
    /// Permit a separately reported stationary miss, never an arrival claim.
    pub allow_settled: bool,
    /// Finish after all references are acknowledged and fresh feedback arrives.
    /// This reports dispatch completion, never measured arrival or stationarity.
    pub reference_only: bool,
    /// Retain the first fresh post-acknowledgement feedback at these offsets.
    /// These are evidence markers, not arrival conditions or authority changes.
    pub checkpoint_offsets_ns: Vec<i64>,
    /// Permit revision-checked future suffixes on a reference-only paced curve.
    pub allow_curve_updates: bool,
    /// Credit only completed moving reference ticks; false keeps a wall deadline.
    pub credit_pacing_delay: bool,
    /// Optional exact polynomial evaluation with acknowledgement-paced progress.
    /// Omission retains fixed offsets and the original lateness guard.
    pub paced_curve: Option<PacedCurve>,
    /// Optional measured holding, velocity-hint direction and encoder settling.
    pub feedback: Option<FeedbackPolicy>,
    pub gripper: Option<GripperPolicy>,
}

/// Measured feedback stamped on the same session clock as playback.
#[derive(Clone, Debug)]
pub struct Feedback {
    pub observed_ns: i64,
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
    pub gripper_force: Option<MeasuredForce>,
}

/// Measured evidence after an acknowledged reference crosses a requested offset.
/// Several markers can share one sample; none certifies intermediate arrival.
#[derive(Clone, Debug)]
pub struct CheckpointEvidence {
    pub offset_ns: i64,
    pub acknowledged_offset_ns: i64,
    pub acknowledged_ns: i64,
    pub elapsed_ns: i64,
    pub feedback: Feedback,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Completion {
    ReferencesDispatched,
    Arrived,
    Settled,
    ContactStall,
    ForceReached,
}

#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum PlaybackError {
    #[error("another local execution or safety request is still active")]
    Busy,
    #[error("invalid finite reference buffer: {0}")]
    Invalid(&'static str),
    #[error("finite reference action-space intake refused: {0}")]
    InvalidAction(String),
    #[error("measured start differs from the admitted first reference")]
    StaleStart,
    #[error("measured joints left the admitted reference tracking window")]
    Tracking,
    #[error("gripper force feedback is unavailable")]
    ForceUnavailable,
    #[error("the jaw reached its closed stop without a qualified force grasp")]
    EmptyGripper,
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
    pub(crate) fn scope(&self) -> Option<&str> {
        self.chunk.steps[0].part.as_deref()
    }

    pub(crate) fn len(&self) -> usize {
        self.chunk.steps.len()
    }

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
        if policy.credit_pacing_delay
            && (!policy.reference_only || policy.paced_curve.is_none() || policy.gripper.is_some())
        {
            return Err(invalid(
                "paced deadlines require reference-only polynomial playback",
            ));
        }
        if policy.allow_curve_updates
            && (!policy.reference_only
                || policy.gripper.is_some()
                || !policy.checkpoint_offsets_ns.is_empty()
                || policy
                    .paced_curve
                    .as_ref()
                    .is_none_or(|c| c.sample_times_s.is_some())
                || policy
                    .feedback
                    .as_ref()
                    .is_some_and(|f| f.velocity_hints == VelocityHints::TowardGoal))
        {
            return Err(invalid(
                "curve updates require reference-only paced playback without checkpoint or goal-hint policies",
            ));
        }
        if policy.checkpoint_offsets_ns.len() > 256
            || policy
                .checkpoint_offsets_ns
                .iter()
                .any(|t| *t < 0 || *t > previous)
            || policy
                .checkpoint_offsets_ns
                .windows(2)
                .any(|t| t[0] >= t[1])
            || (policy.gripper.is_some() && !policy.checkpoint_offsets_ns.is_empty())
        {
            return Err(invalid(
                "requires at most 256 ordered arm checkpoint offsets within the buffer",
            ));
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
            || !(1..=16_385).contains(&policy.minimum_settle_samples)
            || (policy.gripper.is_some() && policy.minimum_settle_samples != 1)
            || (policy.gripper.is_some() && policy.reference_only)
            || policy.timeout_ns <= chunk.horizon_ns
            || policy.timeout_ns > 180_000_000_000
            || policy.settle_ns >= policy.timeout_ns - chunk.horizon_ns
            || !(0..=1_000_000_000).contains(&policy.max_lateness_ns)
        {
            return Err(invalid("invalid bounded settling, lateness or deadline"));
        }
        if let Some(curve) = &policy.paced_curve {
            curve.validate(policy.indices.len())?;
            if (curve.duration_s() * 1e9 - chunk.horizon_ns as f64).abs() > 1.0 {
                return Err(invalid(
                    "polynomial duration differs from the admitted buffer",
                ));
            }
            for step in &chunk.steps {
                let (q, velocity) = curve.sample(step.offset_ns as f64 * 1e-9);
                if policy
                    .indices
                    .iter()
                    .zip(q)
                    .any(|(i, q)| (step.values[*i] - q).abs() > 1e-8)
                {
                    return Err(invalid("polynomial differs from admitted joint references"));
                }
                if (0..width)
                    .any(|i| !policy.indices.contains(&i) && step.values[i] != first.values[i])
                {
                    return Err(invalid("paced playback must retain unselected coordinates"));
                }
                if step.velocity_feedforward.is_some() != first.velocity_feedforward.is_some() {
                    return Err(invalid("paced velocity hints must be consistently enabled"));
                }
                if let Some(hints) = &step.velocity_feedforward
                    && (policy
                        .indices
                        .iter()
                        .zip(velocity)
                        .any(|(i, v)| (hints[*i] - v).abs() > 1e-8)
                        || (0..width).any(|i| !policy.indices.contains(&i) && hints[i] != 0.0))
                {
                    return Err(invalid("paced velocity hints must match the polynomial"));
                }
            }
        }
        if let Some(feedback) = &policy.feedback {
            feedback.validate(&policy, width)?;
        }
        if policy.feedback.is_some() || policy.gripper.is_some() {
            for step in &chunk.steps {
                if (0..width).any(|i| {
                    !policy.indices.contains(&i)
                        && (step.values[i] != first.values[i]
                            || step
                                .velocity_feedforward
                                .as_ref()
                                .is_some_and(|h| h[i] != 0.0))
                }) {
                    return Err(invalid(
                        "feedback playback requires unchanged unselected targets and zero hints",
                    ));
                }
            }
        }
        if let Some(gripper) = &policy.gripper {
            let index = policy.indices[0];
            gripper.validate(
                &policy,
                first.values[index],
                chunk.steps.last().unwrap().values[index],
            )?;
            let low = gripper.closed_position.min(gripper.open_position);
            let high = gripper.closed_position.max(gripper.open_position);
            if chunk.steps.iter().any(|step| {
                !(low..=high).contains(&step.values[index])
                    || step
                        .velocity_feedforward
                        .as_ref()
                        .is_some_and(|v| v.iter().any(|x| *x != 0.0))
            }) {
                return Err(invalid(
                    "gripper references require mapped positions and zero velocity hints",
                ));
            }
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
    settled_samples: usize,
    arrived_samples: usize,
    completion: Option<Completion>,
    failure: Option<PlaybackError>,
    curve: Option<CurveCursor>,
    proposed: Option<Step>,
    dispatched_ns: Option<i64>,
    feedback: FeedbackState,
    gripper: GripperState,
    checkpoints: Vec<CheckpointEvidence>,
}

impl Playback {
    /// Withdraw a future suffix without changing the incumbent or its deadline.
    /// An already dispatched handoff must be acknowledged, never retracted.
    pub fn withdraw_curve(&mut self, expected_revision: u64) -> Result<(), PlaybackError> {
        if !self.references.policy.allow_curve_updates
            || self.completion.is_some()
            || self.failure.is_some()
        {
            return Err(PlaybackError::Invalid(
                "execution does not admit curve withdrawal",
            ));
        }
        if self.pending {
            return Err(PlaybackError::DispatchPending);
        }
        self.curve
            .as_mut()
            .expect("validated revisable curve")
            .withdraw(expected_revision)
    }

    /// Refusal leaves the incumbent intact; it is not a motion failure.
    pub fn update_curve(&mut self, update: CurveUpdate) -> Result<(), PlaybackError> {
        if !self.references.policy.allow_curve_updates
            || self.completion.is_some()
            || self.failure.is_some()
        {
            return Err(PlaybackError::Invalid(
                "execution does not admit curve updates",
            ));
        }
        if self.pending {
            return Err(PlaybackError::DispatchPending);
        }
        self.curve
            .as_mut()
            .expect("validated revisable curve")
            .schedule(update)
    }

    #[must_use]
    pub fn new(references: FiniteReferences, started_ns: i64) -> Self {
        let curve = references
            .policy
            .paced_curve
            .clone()
            .map(|spec| CurveCursor::new(spec, started_ns));
        let feedback = FeedbackState::new(references.chunk.steps[0].values.len());
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
            settled_samples: 0,
            arrived_samples: 0,
            completion: None,
            failure: None,
            curve,
            proposed: None,
            dispatched_ns: None,
            feedback,
            gripper: GripperState::default(),
            checkpoints: Vec::new(),
        }
    }

    #[must_use]
    pub fn reference(&self, index: usize) -> Option<&Step> {
        if self.curve.is_some()
            || self.references.policy.feedback.is_some()
            || self.references.policy.gripper.is_some()
        {
            self.proposed.as_ref().filter(|_| index == self.next)
        } else {
            self.references.reference(index)
        }
    }

    #[must_use]
    pub fn curve_progress(&self) -> Option<CurveProgress> {
        self.curve.as_ref().map(|curve| curve.progress.clone())
    }

    #[must_use]
    pub fn settling_evidence(&self) -> Option<SettlingEvidence> {
        self.feedback.evidence.clone()
    }

    #[must_use]
    pub fn gripper_evidence(&self) -> Option<GripperEvidence> {
        self.gripper.evidence.clone()
    }

    #[must_use]
    pub fn checkpoints(&self) -> &[CheckpointEvidence] {
        &self.checkpoints
    }

    fn record_checkpoints(&mut self, measured: &Feedback) {
        let Some(acknowledged_ns) = self.acknowledged_ns else {
            return;
        };
        if measured.observed_ns <= acknowledged_ns {
            return;
        }
        let offset_ns = self.curve.as_ref().map_or_else(
            || self.references.chunk.steps[self.next - 1].offset_ns,
            |curve| (curve.progress.acknowledged_curve_time_s.unwrap_or(0.0) * 1e9).round() as i64,
        );
        for offset in &self.references.policy.checkpoint_offsets_ns[self.checkpoints.len()..] {
            if *offset > offset_ns {
                break;
            }
            self.checkpoints.push(CheckpointEvidence {
                offset_ns: *offset,
                acknowledged_offset_ns: offset_ns,
                acknowledged_ns,
                elapsed_ns: measured.observed_ns - self.started_ns,
                feedback: measured.clone(),
            });
        }
    }

    pub(crate) fn force_n(&self) -> Option<f64> {
        self.references
            .policy
            .gripper
            .as_ref()
            .and_then(|g| g.force.as_ref())
            .map(|f| f.target_n)
    }

    #[must_use]
    pub fn dispatched(&self) -> usize {
        self.next
    }

    #[must_use]
    pub fn completion(&self) -> Option<Completion> {
        self.completion
    }

    /// The worker must revisit accounting by this time even when the declared
    /// feedback cadence is slower. This changes no reference or tolerance.
    pub(crate) fn check_by_ns(&self) -> i64 {
        let deadline = self.started_ns.saturating_add(self.timeout_ns());
        if self.curve.is_some() || self.references.policy.gripper.is_some() {
            return deadline;
        }
        self.references
            .chunk
            .steps
            .get(self.next)
            .map_or(deadline, |step| {
                deadline.min(
                    self.started_ns
                        .saturating_add(step.offset_ns)
                        .saturating_add(self.references.policy.max_lateness_ns)
                        .saturating_add(1),
                )
            })
    }

    fn timeout_ns(&self) -> i64 {
        self.references.policy.timeout_ns.saturating_add(
            self.curve
                .as_ref()
                .map_or(0, |c| c.progress.pacing_extension_ns),
        )
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
            || measured
                .gripper_force
                .as_ref()
                .is_some_and(|f| !f.force_n.is_finite() || f.force_n < 0.0 || f.source.is_empty())
        {
            return self.fail(PlaybackError::InvalidFeedback);
        }
        self.now_ns = now_ns;
        if now_ns.saturating_sub(self.started_ns) >= self.timeout_ns() {
            return self.fail(PlaybackError::Deadline);
        }
        let fresh = self.observed_ns != Some(measured.observed_ns);
        if !fresh {
            self.feedback.reset_window();
        }
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
        if self.references.policy.gripper.is_some() {
            return self.poll_gripper(now_ns, measured, fresh);
        }
        if let Some(curve) = &self.curve {
            if self
                .acknowledged_ns
                .is_some_and(|at| measured.observed_ns <= at)
            {
                return Ok(PlaybackPoll::Observe);
            }
            if let Err(error) = curve.check_tracking(measured, &self.references.policy.indices) {
                return self.fail(error);
            }
        }
        if fresh {
            self.record_checkpoints(measured);
        }
        let finished = self
            .curve
            .as_ref()
            .map_or(self.next == self.references.chunk.steps.len(), |curve| {
                curve.progress.endpoint_acknowledged
            });
        let goal = &self
            .references
            .chunk
            .steps
            .last()
            .expect("validated buffer")
            .values;
        let arrived = within(
            &measured.positions,
            goal,
            &self.references.policy.position_tolerance,
            &self.references.policy.indices,
        );
        let after_ack = self
            .acknowledged_ns
            .is_some_and(|at| measured.observed_ns > at);
        if self.references.policy.reference_only {
            if finished && after_ack && fresh {
                self.completion = Some(Completion::ReferencesDispatched);
                return Ok(PlaybackPoll::Complete(Completion::ReferencesDispatched));
            }
            if finished {
                return Ok(PlaybackPoll::Observe);
            }
        }
        match self.feedback.observe(
            measured,
            &self.references.policy,
            arrived,
            finished && after_ack,
            fresh,
        ) {
            Ok(Some(outcome)) => {
                self.completion = Some(outcome);
                return Ok(PlaybackPoll::Complete(outcome));
            }
            Err(error) => return self.fail(error),
            Ok(None) => {}
        }
        if let Some(curve) = self
            .curve
            .as_mut()
            .filter(|c| !c.progress.endpoint_acknowledged)
        {
            let due = self.dispatched_ns.map_or(self.started_ns, |at| {
                at.saturating_add(curve.spec.period_ns)
            });
            if now_ns < due {
                return Ok(PlaybackPoll::WaitUntil(due));
            }
            let (positions, velocity) =
                match curve.propose(measured, &self.references.policy.indices) {
                    Ok(state) => state,
                    Err(error) => return self.fail(error),
                };
            let mut step = self.references.chunk.steps[0].clone();
            let mut values = step.values.to_vec();
            let mut hints = vec![0.0; values.len()];
            for ((index, q), v) in self
                .references
                .policy
                .indices
                .iter()
                .zip(positions)
                .zip(velocity)
            {
                values[*index] = q;
                hints[*index] = v;
            }
            step.values = values.into();
            step.offset_ns = (curve.proposed_s * 1e9).round() as i64;
            if step.velocity_feedforward.is_some() {
                step.velocity_feedforward = Some(hints.into());
            }
            self.feedback
                .adapt(&mut step, measured, &self.references.policy, goal);
            self.proposed = Some(step);
            self.pending = true;
            return Ok(PlaybackPoll::Dispatch(self.next));
        }
        if let Some(step) = self
            .references
            .chunk
            .steps
            .get(self.next)
            .filter(|_| self.curve.is_none())
        {
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
            if self.references.policy.feedback.is_some() {
                let mut step = step.clone();
                self.feedback
                    .adapt(&mut step, measured, &self.references.policy, goal);
                self.proposed = Some(step);
            }
            return Ok(PlaybackPoll::Dispatch(self.next));
        }
        if self
            .references
            .policy
            .feedback
            .as_ref()
            .is_some_and(|s| s.position_window.is_some())
        {
            return Ok(PlaybackPoll::Observe);
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
            self.settled_samples = 0;
            self.arrived_samples = 0;
            return Ok(PlaybackPoll::Observe);
        }
        if arrived {
            self.arrived_since.get_or_insert(measured.observed_ns);
            self.arrived_samples += 1;
        } else {
            self.arrived_since = None;
            self.arrived_samples = 0;
        }
        self.settled_samples += 1;
        let since = *self.settled_since.get_or_insert(measured.observed_ns);
        if measured.observed_ns - since < self.references.policy.settle_ns
            || self.settled_samples < self.references.policy.minimum_settle_samples
        {
            return Ok(PlaybackPoll::Observe);
        }
        let outcome = if self
            .arrived_since
            .is_some_and(|since| measured.observed_ns - since >= self.references.policy.settle_ns)
            && self.arrived_samples >= self.references.policy.minimum_settle_samples
        {
            Completion::Arrived
        } else {
            Completion::Settled
        };
        self.completion = Some(outcome);
        Ok(PlaybackPoll::Complete(outcome))
    }

    fn poll_gripper(
        &mut self,
        now_ns: i64,
        measured: &Feedback,
        fresh: bool,
    ) -> Result<PlaybackPoll, PlaybackError> {
        let policy = &self.references.policy;
        let spec = policy.gripper.as_ref().expect("gripper mode");
        let due = self
            .acknowledged_ns
            .map_or(self.started_ns, |at| at.saturating_add(spec.period_ns));
        if now_ns < due {
            return Ok(PlaybackPoll::WaitUntil(due));
        }
        if self
            .acknowledged_ns
            .is_some_and(|at| measured.observed_ns <= at)
            || !fresh
        {
            self.gripper.reset_samples();
            return Ok(PlaybackPoll::Observe);
        }
        let index = policy.indices[0];
        let first = &self.references.chunk.steps[0];
        let last = self
            .references
            .chunk
            .steps
            .last()
            .expect("validated buffer");
        match self.gripper.observe(
            measured,
            policy,
            first.values[index],
            last.values[index],
            fresh,
        ) {
            Ok(Some(result)) => {
                self.completion = Some(result);
                return Ok(PlaybackPoll::Complete(result));
            }
            Err(error) => return self.fail(error),
            Ok(None) => {}
        }
        let nominal = if spec.force_grasp() {
            last
        } else {
            &self.references.chunk.steps[self.next.min(self.references.chunk.steps.len() - 1)]
        };
        let mut step = nominal.clone();
        let mut positions = step.values.to_vec();
        positions[index] = self
            .gripper
            .reference(measured, policy, nominal.values[index]);
        step.values = positions.into();
        self.feedback
            .adapt(&mut step, measured, policy, &last.values);
        self.proposed = Some(step);
        self.pending = true;
        Ok(PlaybackPoll::Dispatch(self.next))
    }

    /// Record a confirmed dispatch only. On rejection or unknown outcome the
    /// owner stops playback and retains its original fault; it must not ack.
    pub fn acknowledge(&mut self, index: usize, now_ns: i64) -> Result<(), PlaybackError> {
        self.acknowledge_at(index, now_ns, now_ns)
    }

    /// Exact local submission start when provided by the owner adapter. Without
    /// it, callers conservatively use acknowledgement time for minimum spacing.
    pub fn acknowledge_at(
        &mut self,
        index: usize,
        started_ns: i64,
        now_ns: i64,
    ) -> Result<(), PlaybackError> {
        if let Some(error) = &self.failure {
            return Err(error.clone());
        }
        if self.completion.is_some() {
            return Err(PlaybackError::InvalidAcknowledgement);
        }
        if !self.pending || index != self.next || started_ns < self.now_ns || now_ns < started_ns {
            return self.fail(PlaybackError::InvalidAcknowledgement);
        }
        self.next += 1;
        self.pending = false;
        self.now_ns = now_ns;
        self.acknowledged_ns = Some(now_ns);
        self.dispatched_ns = Some(started_ns);
        if self.references.policy.gripper.is_some() {
            self.gripper.last_command = Some(
                self.proposed
                    .as_ref()
                    .expect("proposed gripper reference")
                    .values[self.references.policy.indices[0]],
            );
            self.gripper.nominal_complete = self.next >= self.references.chunk.steps.len();
        }
        if let Some(curve) = &mut self.curve {
            curve.acknowledge(now_ns, self.references.policy.credit_pacing_delay);
        }
        Ok(())
    }
}

fn within(actual: &[f64], target: &[f64], tolerances: &[f64], indices: &[usize]) -> bool {
    indices
        .iter()
        .zip(tolerances)
        .all(|(i, tolerance)| (actual[*i] - target[*i]).abs() <= *tolerance)
}
