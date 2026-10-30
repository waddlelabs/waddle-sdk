//! Marshalling for the native finite executor. No frontend timing or authority.

use std::sync::Arc;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyDict;
use waddle_runtime::playback::{
    Completion, ExecutionFailure, ExecutionHandle, ExecutionSnapshot, Feedback, PlaybackError,
    PlaybackIo, PlaybackIoError, PlaybackPolicy, ReferenceReceipt,
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
    max_lateness_ns: i64,
    timeout_ns: i64,
    allow_settled: bool,
}

impl From<Policy> for PlaybackPolicy {
    fn from(p: Policy) -> Self {
        Self {
            indices: p.indices,
            start_tolerance: p.start_tolerance,
            position_tolerance: p.position_tolerance,
            stationary_velocity: p.stationary_velocity,
            settle_ns: p.settle_ns,
            max_lateness_ns: p.max_lateness_ns,
            timeout_ns: p.timeout_ns,
            allow_settled: p.allow_settled,
        }
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
}

#[derive(FromPyObject)]
#[pyo3(from_item_all)]
struct Receipt {
    dispatched: bool,
    detail: String,
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
    Ok(out.unbind())
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
            }))
        })
    }

    fn dispatch(
        &self,
        reference: &Step,
        measured: &Feedback,
    ) -> Result<ReferenceReceipt, PlaybackIoError> {
        callback(true, |py| {
            let step = PyDict::new(py);
            step.set_item("positions", reference.values.as_ref())?;
            step.set_item(
                "velocity_feedforward",
                reference.velocity_feedforward.as_deref(),
            )?;
            step.set_item("part", reference.part.as_deref())?;
            step.set_item("offset_ns", reference.offset_ns)?;
            let value = self
                .dispatch
                .bind(py)
                .call1((step, feedback(py, measured)?))?;
            if let Ok(error) = value.extract::<PyRef<'_, PyIoError>>() {
                return Ok(Err(error.0.clone()));
            }
            let receipt: Receipt = value.extract()?;
            Ok(Ok(if receipt.dispatched {
                ReferenceReceipt::Dispatched
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
            policy.into(),
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
}

fn snapshot(py: Python<'_>, value: ExecutionSnapshot) -> PyResult<Py<PyDict>> {
    let out = PyDict::new(py);
    out.set_item("id", value.id)?;
    out.set_item("episode_id", value.episode_id.to_string())?;
    out.set_item("dispatched", value.dispatched)?;
    out.set_item("references", value.references)?;
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
        Some(Ok(Completion::Arrived)) => "arrived",
        Some(Ok(Completion::Settled)) => "settled",
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
