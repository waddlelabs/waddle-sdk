//! Marshalling helpers: ndarray/sequence extraction, enum parsing, error
//! mapping. Pure conversion — no policy.

use numpy::{PyArray1, PyArrayMethods, PyReadonlyArray1};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use waddle_runtime::RuntimeError;
use waddle_types::pb::v0 as pb;
use waddle_types::{HandoffPolicy, LeaseEnforcement, TerminalOutcome};

/// A borrowed-or-owned float64 row extracted from a Python object:
/// zero-copy for contiguous float64 ndarrays, an owned copy for lists and
/// other dtypes (the documented slow path).
pub(crate) enum F64s<'py> {
    Array(PyReadonlyArray1<'py, f64>),
    Owned(Vec<f64>),
}

impl F64s<'_> {
    pub(crate) fn as_slice(&self) -> &[f64] {
        match self {
            // Contiguity was checked at construction.
            Self::Array(a) => a.as_slice().expect("checked contiguous"),
            Self::Owned(v) => v,
        }
    }
}

pub(crate) fn extract_f64s<'py>(obj: &Bound<'py, PyAny>) -> PyResult<F64s<'py>> {
    if let Ok(arr) = obj.cast::<PyArray1<f64>>() {
        let readonly = arr.readonly();
        if readonly.as_slice().is_ok() {
            return Ok(F64s::Array(readonly));
        }
    }
    Ok(F64s::Owned(obj.extract::<Vec<f64>>()?))
}

pub(crate) fn parse_robot_json(json: &str) -> PyResult<pb::RobotDescription> {
    waddle_sidecar::json::message_from_json::<pb::RobotDescription>(
        "waddle.v0.RobotDescription",
        json,
    )
    .map_err(|e| PyValueError::new_err(e.to_string()))
}

pub(crate) fn parse_handoff(kind: &str, ns: i64) -> PyResult<HandoffPolicy> {
    match kind {
        "hold_first" => Ok(HandoffPolicy::HoldFirst),
        "immediate" => Ok(HandoffPolicy::Immediate { blend_ns: ns }),
        "chunk_boundary" => Ok(HandoffPolicy::ChunkBoundary { max_wait_ns: ns }),
        other => Err(PyValueError::new_err(format!(
            "unknown handoff kind {other:?}"
        ))),
    }
}

pub(crate) fn parse_enforcement(value: &str) -> PyResult<LeaseEnforcement> {
    match value.to_ascii_lowercase().as_str() {
        "advisory" => Ok(LeaseEnforcement::Advisory),
        "enforced" => Ok(LeaseEnforcement::Enforced),
        other => Err(PyValueError::new_err(format!(
            "lease_enforcement={other:?}: expected \"advisory\" or \"enforced\""
        ))),
    }
}

pub(crate) fn parse_outcome(value: &str) -> PyResult<TerminalOutcome> {
    match value.to_ascii_lowercase().as_str() {
        "success" => Ok(TerminalOutcome::Success),
        "failure" => Ok(TerminalOutcome::Failure),
        "abort" => Ok(TerminalOutcome::Abort),
        other => Err(PyValueError::new_err(format!(
            "outcome={other:?}: expected \"success\", \"failure\" or \"abort\""
        ))),
    }
}

pub(crate) fn outcome_str(outcome: TerminalOutcome) -> &'static str {
    match outcome {
        TerminalOutcome::Success => "success",
        TerminalOutcome::Failure => "failure",
        TerminalOutcome::Abort => "abort",
        TerminalOutcome::AbortedRetake => "aborted_retake",
    }
}

pub(crate) fn runtime_err(e: RuntimeError) -> PyErr {
    match e {
        RuntimeError::MissingRobot | RuntimeError::Types(_) => PyValueError::new_err(e.to_string()),
        _ => PyRuntimeError::new_err(e.to_string()),
    }
}
