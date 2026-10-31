//! Marshalling for the native finite executor. No frontend timing or authority.

use std::sync::Arc;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyDict;
use waddle_runtime::playback::{
    Completion, ContactCondition, CurveUpdate, ExecutionFailure, ExecutionHandle,
    ExecutionSnapshot, Feedback, FeedbackPolicy, ForceCondition, GripperPolicy, HeldJointGroup,
    MeasuredForce, PacedCurve, PlaybackError, PlaybackIo, PlaybackIoError, PlaybackPolicy,
    PositionWindow, ReferenceReceipt, VelocityHints,
};
use waddle_types::Step;

pyo3::create_exception!(_core, PlaybackBusyError, PyRuntimeError);

#[derive(FromPyObject)]
pub(crate) struct Policy {
    indices: Vec<usize>,
    start_tolerance: Vec<f64>,
    position_tolerance: Vec<f64>,
    stationary_velocity: Vec<f64>,
    settle_ns: i64,
    minimum_settle_samples: usize,
    max_lateness_ns: i64,
    timeout_ns: i64,
    allow_settled: bool,
    reference_only: bool,
    checkpoint_offsets_ns: Vec<i64>,
    allow_curve_updates: bool,
    paced_curve: Option<Curve>,
    feedback: Option<FeedbackOptions>,
    gripper: Option<Gripper>,
}

#[derive(FromPyObject)]
struct Gripper {
    period_ns: i64,
    position_error: f64,
    tracking_tolerance: f64,
    closed_position: f64,
    open_position: f64,
    closed_m: f64,
    open_m: f64,
    contact: Option<Contact>,
    force: Option<Force>,
}

#[derive(FromPyObject)]
struct Contact {
    minimum_opening_m: f64,
    minimum_progress_m: f64,
    target_shortfall_m: f64,
    commanded_shortfall_m: f64,
    max_drift_m: f64,
    window_ns: i64,
}

#[derive(FromPyObject)]
struct Force {
    target_n: f64,
    tolerance_n: f64,
    closed_tolerance_m: f64,
}

#[derive(FromPyObject)]
struct ForceFeedback {
    force_n: f64,
    source: String,
    estimated: bool,
}

#[derive(FromPyObject)]
struct FeedbackOptions {
    held_groups: Vec<HeldGroup>,
    measured_indices: Vec<usize>,
    velocity_hints: String,
    position_window: Option<Window>,
}

#[derive(FromPyObject)]
struct HeldGroup {
    indices: Vec<usize>,
    position_error: Vec<f64>,
    period_ns: i64,
}

#[derive(FromPyObject)]
struct Window {
    max_drift: Vec<f64>,
    max_gap_ns: Option<i64>,
    during_motion: bool,
    consecutive_samples: usize,
}

#[derive(FromPyObject)]
struct Curve {
    times_s: Vec<f64>,
    positions: Vec<Vec<f64>>,
    coefficients: Vec<Vec<Vec<f64>>>,
    period_ns: i64,
    position_error: Vec<f64>,
    tracking_tolerance: Vec<f64>,
    sample_times_s: Option<Vec<f64>>,
}

impl From<Curve> for PacedCurve {
    fn from(c: Curve) -> Self {
        Self {
            times_s: c.times_s,
            positions: c.positions,
            coefficients: c.coefficients,
            period_ns: c.period_ns,
            position_error: c.position_error,
            tracking_tolerance: c.tracking_tolerance,
            sample_times_s: c.sample_times_s,
        }
    }
}

#[derive(FromPyObject)]
struct Update {
    expected_revision: u64,
    switch_ns: i64,
    curve: Curve,
}

impl TryFrom<Policy> for PlaybackPolicy {
    type Error = PyErr;

    fn try_from(p: Policy) -> PyResult<Self> {
        let feedback = p
            .feedback
            .map(|f| -> PyResult<FeedbackPolicy> {
                let velocity_hints = match f.velocity_hints.as_str() {
                    "curve" => VelocityHints::Curve,
                    "toward_reference" => VelocityHints::TowardReference,
                    "toward_goal" => VelocityHints::TowardGoal,
                    _ => return Err(PyValueError::new_err("unknown playback velocity hint mode")),
                };
                Ok(FeedbackPolicy {
                    held_groups: f
                        .held_groups
                        .into_iter()
                        .map(|g| HeldJointGroup {
                            indices: g.indices,
                            position_error: g.position_error,
                            period_ns: g.period_ns,
                        })
                        .collect(),
                    measured_indices: f.measured_indices,
                    velocity_hints,
                    position_window: f.position_window.map(|w| PositionWindow {
                        max_drift: w.max_drift,
                        max_gap_ns: w.max_gap_ns,
                        during_motion: w.during_motion,
                        consecutive_samples: w.consecutive_samples,
                    }),
                })
            })
            .transpose()?;
        Ok(Self {
            indices: p.indices,
            start_tolerance: p.start_tolerance,
            position_tolerance: p.position_tolerance,
            stationary_velocity: p.stationary_velocity,
            settle_ns: p.settle_ns,
            minimum_settle_samples: p.minimum_settle_samples,
            max_lateness_ns: p.max_lateness_ns,
            timeout_ns: p.timeout_ns,
            allow_settled: p.allow_settled,
            reference_only: p.reference_only,
            checkpoint_offsets_ns: p.checkpoint_offsets_ns,
            allow_curve_updates: p.allow_curve_updates,
            paced_curve: p.paced_curve.map(Into::into),
            feedback,
            gripper: p.gripper.map(|g| GripperPolicy {
                period_ns: g.period_ns,
                position_error: g.position_error,
                tracking_tolerance: g.tracking_tolerance,
                closed_position: g.closed_position,
                open_position: g.open_position,
                closed_m: g.closed_m,
                open_m: g.open_m,
                contact: g.contact.map(|c| ContactCondition {
                    minimum_opening_m: c.minimum_opening_m,
                    minimum_progress_m: c.minimum_progress_m,
                    target_shortfall_m: c.target_shortfall_m,
                    commanded_shortfall_m: c.commanded_shortfall_m,
                    max_drift_m: c.max_drift_m,
                    window_ns: c.window_ns,
                }),
                force: g.force.map(|f| ForceCondition {
                    target_n: f.target_n,
                    tolerance_n: f.tolerance_n,
                    closed_tolerance_m: f.closed_tolerance_m,
                }),
            }),
        })
    }
}

/// Callbacks return this for a known owner failure, retaining opaque evidence.
#[pyclass(name = "PlaybackIoError", frozen)]
pub(crate) struct PyIoError(PlaybackIoError);

#[pymethods]
impl PyIoError {
    #[new]
    fn new(detail: String, evidence: String, outcome_unknown: bool) -> Self {
        Self(PlaybackIoError {
            detail,
            evidence,
            outcome_unknown,
        })
    }
}

#[derive(FromPyObject)]
#[pyo3(from_item_all)]
struct Measured {
    observed_ns: i64,
    positions: Vec<f64>,
    velocities: Vec<f64>,
    gripper_force: Option<ForceFeedback>,
}

#[derive(FromPyObject)]
#[pyo3(from_item_all)]
struct Receipt {
    dispatched: bool,
    detail: String,
    started_ns: i64,
}

pub(crate) struct PyIo {
    pub observe: Py<PyAny>,
    pub dispatch: Py<PyAny>,
}

fn callback<T>(
    unknown: bool,
    call: impl FnOnce(Python<'_>) -> PyResult<Result<T, PlaybackIoError>>,
) -> Result<T, PlaybackIoError> {
    Python::try_attach(|py| {
        call(py).unwrap_or_else(|error| {
            Err(PlaybackIoError {
                detail: error
                    .value(py)
                    .repr()
                    .map(|v| v.to_string())
                    .unwrap_or_else(|_| error.to_string()),
                evidence: String::new(),
                outcome_unknown: unknown,
            })
        })
    })
    .unwrap_or_else(|| {
        Err(PlaybackIoError {
            detail: "Python interpreter unavailable".into(),
            evidence: String::new(),
            outcome_unknown: unknown,
        })
    })
}

fn feedback(py: Python<'_>, value: &Feedback) -> PyResult<Py<PyDict>> {
    let out = PyDict::new(py);
    out.set_item("observed_ns", value.observed_ns)?;
    out.set_item("positions", &value.positions)?;
    out.set_item("velocities", &value.velocities)?;
    let force = value
        .gripper_force
        .as_ref()
        .map(|f| -> PyResult<Py<PyDict>> {
            let result = PyDict::new(py);
            result.set_item("force_n", f.force_n)?;
            result.set_item("source", &f.source)?;
            result.set_item("estimated", f.estimated)?;
            Ok(result.unbind())
        })
        .transpose()?;
    out.set_item("gripper_force", force)?;
    Ok(out.unbind())
}

fn reference(py: Python<'_>, value: &Step) -> PyResult<Py<PyDict>> {
    let step = PyDict::new(py);
    step.set_item("positions", value.values.as_ref())?;
    step.set_item(
        "velocity_feedforward",
        value.velocity_feedforward.as_deref(),
    )?;
    step.set_item("part", value.part.as_deref())?;
    step.set_item("offset_ns", value.offset_ns)?;
    Ok(step.unbind())
}

impl PlaybackIo for PyIo {
    fn observe(&self) -> Result<Feedback, PlaybackIoError> {
        callback(false, |py| {
            let value = self.observe.bind(py).call0()?;
            if let Ok(error) = value.extract::<PyRef<'_, PyIoError>>() {
                return Ok(Err(error.0.clone()));
            }
            let measured: Measured = value.extract()?;
            Ok(Ok(Feedback {
                observed_ns: measured.observed_ns,
                positions: measured.positions,
                velocities: measured.velocities,
                gripper_force: measured.gripper_force.map(|f| MeasuredForce {
                    force_n: f.force_n,
                    source: f.source,
                    estimated: f.estimated,
                }),
            }))
        })
    }

    fn dispatch(
        &self,
        step: &Step,
        measured: &Feedback,
        force_n: Option<f64>,
    ) -> Result<ReferenceReceipt, PlaybackIoError> {
        callback(true, |py| {
            let value = self.dispatch.bind(py).call1((
                reference(py, step)?,
                feedback(py, measured)?,
                force_n,
            ))?;
            if let Ok(error) = value.extract::<PyRef<'_, PyIoError>>() {
                return Ok(Err(error.0.clone()));
            }
            let receipt: Receipt = value.extract()?;
            Ok(Ok(if receipt.dispatched {
                ReferenceReceipt::DispatchedAt(receipt.started_ns)
            } else {
                ReferenceReceipt::Superseded(receipt.detail)
            }))
        })
    }
}

pub(crate) fn start(
    session: &waddle_runtime::Session,
    id: &waddle_types::EpisodeId,
    chunk_json: &str,
    policy: Policy,
    observe: Py<PyAny>,
    dispatch: Py<PyAny>,
) -> PyResult<PyExecutionHandle> {
    let chunk = waddle_sidecar::json::message_from_json::<waddle_types::pb::v0::ActionChunk>(
        "waddle.v0.ActionChunk",
        chunk_json,
    )
    .map_err(|e| PyValueError::new_err(e.to_string()))?;
    session
        .start_playback(
            id,
            &chunk,
            policy.try_into()?,
            Arc::new(PyIo { observe, dispatch }),
        )
        .map(PyExecutionHandle)
        .map_err(|error| match error {
            PlaybackError::Busy => PlaybackBusyError::new_err(error.to_string()),
            _ => PyValueError::new_err(error.to_string()),
        })
}

#[pyclass(name = "ExecutionHandle", frozen)]
pub(crate) struct PyExecutionHandle(ExecutionHandle);

#[pymethods]
impl PyExecutionHandle {
    fn snapshot(&self, py: Python<'_>) -> PyResult<Py<PyDict>> {
        snapshot(py, self.0.snapshot())
    }
    fn wait(&self, py: Python<'_>) -> PyResult<Py<PyDict>> {
        snapshot(py, py.detach(|| self.0.wait()))
    }
    fn cancel(&self, reason: &str) {
        self.0.cancel(reason);
    }
    fn update_curve(&self, update: Update) -> PyResult<u64> {
        CurveUpdate::new(
            update.expected_revision,
            update.switch_ns,
            update.curve.into(),
        )
        .and_then(|update| self.0.update_curve(update))
        .map_err(|error| match error {
            PlaybackError::Busy => PlaybackBusyError::new_err(error.to_string()),
            _ => PyValueError::new_err(error.to_string()),
        })
    }
}

fn snapshot(py: Python<'_>, value: ExecutionSnapshot) -> PyResult<Py<PyDict>> {
    let out = PyDict::new(py);
    out.set_item("id", value.id)?;
    out.set_item("episode_id", value.episode_id.to_string())?;
    out.set_item("dispatched", value.dispatched)?;
    out.set_item("references", value.references)?;
    out.set_item("last_acknowledged_ns", value.last_acknowledged_ns)?;
    let update = value
        .curve_update
        .map(|receipt| -> PyResult<Py<PyDict>> {
            let result = PyDict::new(py);
            result.set_item("request_id", receipt.request_id)?;
            result.set_item(
                "status",
                match &receipt.result {
                    None => "pending",
                    Some(Ok(())) => "accepted",
                    Some(Err(_)) => "rejected",
                },
            )?;
            result.set_item(
                "error",
                receipt.result.and_then(Result::err).map(|e| e.to_string()),
            )?;
            Ok(result.unbind())
        })
        .transpose()?;
    out.set_item("curve_update", update)?;
    let checkpoints = value
        .checkpoints
        .iter()
        .map(|sample| -> PyResult<Py<PyDict>> {
            let item = PyDict::new(py);
            item.set_item("offset_ns", sample.offset_ns)?;
            item.set_item("acknowledged_offset_ns", sample.acknowledged_offset_ns)?;
            item.set_item("acknowledged_ns", sample.acknowledged_ns)?;
            item.set_item("elapsed_ns", sample.elapsed_ns)?;
            item.set_item("feedback", feedback(py, &sample.feedback)?)?;
            Ok(item.unbind())
        })
        .collect::<PyResult<Vec<_>>>()?;
    out.set_item("checkpoints", checkpoints)?;
    out.set_item(
        "feedback_reference",
        value
            .feedback_reference
            .as_ref()
            .map(|r| reference(py, r))
            .transpose()?,
    )?;
    out.set_item(
        "last_reference",
        value
            .last_reference
            .as_ref()
            .map(|r| reference(py, r))
            .transpose()?,
    )?;
    let progress = value
        .curve_progress
        .map(|progress| -> PyResult<Py<PyDict>> {
            let result = PyDict::new(py);
            result.set_item("revision", progress.revision)?;
            result.set_item("pending_revision", progress.pending_revision)?;
            result.set_item(
                "acknowledged_curve_time_s",
                progress.acknowledged_curve_time_s,
            )?;
            result.set_item("next_curve_time_s", progress.next_curve_time_s)?;
            result.set_item("endpoint_acknowledged", progress.endpoint_acknowledged)?;
            result.set_item("limited_ticks", progress.limited_ticks)?;
            result.set_item("partial_ticks", progress.partial_ticks)?;
            result.set_item("held_ticks", progress.held_ticks)?;
            Ok(result.unbind())
        })
        .transpose()?;
    out.set_item("curve_progress", progress)?;
    let settling = value
        .settling
        .map(|e| -> PyResult<Py<PyDict>> {
            let result = PyDict::new(py);
            result.set_item("samples", e.samples)?;
            result.set_item("span_ns", e.span_ns)?;
            result.set_item("joint_spans", e.joint_spans)?;
            result.set_item("raw_velocity_peak", e.raw_velocity_peak)?;
            result.set_item("consecutive_samples", e.consecutive_samples)?;
            Ok(result.unbind())
        })
        .transpose()?;
    out.set_item("settling", settling)?;
    let gripper = value
        .gripper
        .map(|e| -> PyResult<Py<PyDict>> {
            let result = PyDict::new(py);
            result.set_item("target_opening_m", e.target_opening_m)?;
            result.set_item("measured_opening_m", e.measured_opening_m)?;
            result.set_item("commanded_opening_m", e.commanded_opening_m)?;
            result.set_item("progress_m", e.progress_m)?;
            result.set_item("target_shortfall_m", e.target_shortfall_m)?;
            result.set_item("commanded_shortfall_m", e.commanded_shortfall_m)?;
            result.set_item("window_ns", e.window_ns)?;
            result.set_item("drift_m", e.drift_m)?;
            result.set_item("force_target_n", e.force_target_n)?;
            result.set_item("force_samples", e.force_samples)?;
            result.set_item("arrival_samples", e.arrival_samples)?;
            Ok(result.unbind())
        })
        .transpose()?;
    out.set_item("gripper", gripper)?;
    let cleanup = value
        .cleanup
        .map(|receipt| -> PyResult<Py<PyDict>> {
            let result = PyDict::new(py);
            result.set_item("request", format!("{:?}", receipt.verb).to_lowercase())?;
            result.set_item(
                "outcome",
                if receipt.result.is_ok() {
                    "acknowledged"
                } else {
                    "unknown"
                },
            )?;
            result.set_item("error", receipt.result.err().map(|e| e.to_string()))?;
            result.set_item("completed_ns", receipt.at.0)?;
            result.set_item("measured_stationary", false)?;
            Ok(result.unbind())
        })
        .transpose()?;
    out.set_item("cleanup", cleanup)?;
    out.set_item(
        "feedback",
        value
            .feedback
            .as_ref()
            .map(|v| feedback(py, v))
            .transpose()?,
    )?;
    let status = match value.result.as_ref() {
        None => "running",
        Some(Ok(Completion::ReferencesDispatched)) => "references_dispatched",
        Some(Ok(Completion::Arrived)) => "arrived",
        Some(Ok(Completion::Settled)) => "settled",
        Some(Ok(Completion::ContactStall)) => "contact_stall",
        Some(Ok(Completion::ForceReached)) => "force_reached",
        Some(Err(_)) => "failed",
    };
    out.set_item("status", status)?;
    let failure = if let Some(Err(error)) = value.result {
        Some(failure(py, error)?)
    } else {
        None
    };
    out.set_item("failure", failure)?;
    Ok(out.unbind())
}

fn failure(py: Python<'_>, error: ExecutionFailure) -> PyResult<Py<PyDict>> {
    let out = PyDict::new(py);
    let mut unknown = false;
    let mut evidence = String::new();
    let (code, detail) = match error {
        ExecutionFailure::Playback(ref error) => {
            let code = match error {
                PlaybackError::Busy => "busy",
                PlaybackError::Invalid(_) | PlaybackError::InvalidAction(_) => "invalid",
                PlaybackError::StaleStart => "stale_start",
                PlaybackError::Tracking => "tracking",
                PlaybackError::ForceUnavailable => "gripper_force_unavailable",
                PlaybackError::EmptyGripper => "gripper_empty",
                PlaybackError::LateReference { index, lateness_ns } => {
                    out.set_item("reference_index", index)?;
                    out.set_item("lateness_ns", lateness_ns)?;
                    "late_reference"
                }
                PlaybackError::Deadline => "deadline",
                PlaybackError::DispatchPending => {
                    unknown = true;
                    "dispatch_pending"
                }
                PlaybackError::InvalidAcknowledgement => {
                    unknown = true;
                    "invalid_acknowledgement"
                }
                PlaybackError::InvalidFeedback => "invalid_feedback",
                PlaybackError::Cancelled(_) => "cancelled",
            };
            (code, error.to_string())
        }
        ExecutionFailure::Observation(error) => {
            unknown = error.outcome_unknown;
            evidence = error.evidence;
            ("observation", error.detail)
        }
        ExecutionFailure::Dispatch(error) => {
            unknown = error.outcome_unknown;
            evidence = error.evidence;
            ("dispatch", error.detail)
        }
        ExecutionFailure::Superseded(detail) => ("superseded", detail),
        ExecutionFailure::Panicked => {
            unknown = true;
            ("panicked", "local callback panicked".into())
        }
    };
    out.set_item("code", code)?;
    out.set_item("detail", detail)?;
    out.set_item("outcome_unknown", unknown)?;
    out.set_item("evidence", evidence)?;
    Ok(out.unbind())
}
