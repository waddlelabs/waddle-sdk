//! Marshalling helpers: ndarray/sequence extraction, enum parsing, error
//! mapping. Pure conversion — no policy.

use std::sync::Arc;

use numpy::{PyArray1, PyArrayMethods, PyReadonlyArray1};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use waddle_runtime::{ResetHook, ResetSpec, RuntimeError};
use waddle_types::pb::v0 as pb;
use waddle_types::{
    ActorKind, HandoffPolicy, LeaseEnforcement, ResetVerificationMode, TerminalOutcome,
};

use crate::verbs::PyResetHook;

/// A borrowed-or-owned float64 row extracted from a Python object:
/// zero-copy for contiguous float64 ndarrays, an owned copy for lists and
/// other dtypes (the documented slow path).
///
/// The zero-copy borrow is held across the core gate call. Sound because
/// the caller keeps the GIL for the whole call (gate never detaches) on a
/// GIL build (abi3): no Python thread can mutate the buffer mid-call. A
/// free-threaded (nogil) build would invalidate this reasoning — revisit
/// before shipping one.
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

/// Build one reset phase's [`ResetSpec`] from its four kwargs (all sharing
/// one `label` — `"pre_reset"` or `"post_reset"` — for error messages).
/// `"none"` disables the phase (`None`); `"hook"` wraps `hook` as a
/// [`PyResetHook`] (requires `hook` to be set); `"teleop"`/`"agent"` build a
/// [`ResetSpec::Remote`] window for that actor. Pure kwarg-to-type mapping —
/// zero reset/claim logic of its own.
pub(crate) fn parse_reset_spec(
    label: &str,
    kind: &str,
    hook: Option<Py<PyAny>>,
    prompt: Option<&str>,
    timeout_ns: i64,
) -> PyResult<Option<ResetSpec>> {
    match kind {
        "none" => Ok(None),
        "hook" => {
            let cb = hook.ok_or_else(|| {
                PyValueError::new_err(format!(
                    "{label}_kind=\"hook\" requires {label}_hook to be set"
                ))
            })?;
            let hook = Arc::new(PyResetHook { cb });
            let reset_hook: ResetHook = Arc::new(move |task: &str| hook.call(task));
            Ok(Some(ResetSpec::Hook(reset_hook)))
        }
        "teleop" => Ok(Some(ResetSpec::Remote {
            actor: ActorKind::Teleoperator,
            prompt: prompt.unwrap_or_default().to_owned(),
            timeout_ns,
        })),
        "agent" => Ok(Some(ResetSpec::Remote {
            actor: ActorKind::Agent,
            prompt: prompt.unwrap_or_default().to_owned(),
            timeout_ns,
        })),
        other => Err(PyValueError::new_err(format!(
            "{label}_kind={other:?}: expected \"none\", \"hook\", \"teleop\", or \"agent\""
        ))),
    }
}

pub(crate) fn parse_verification_mode(value: &str) -> PyResult<ResetVerificationMode> {
    match value {
        "blocking" => Ok(ResetVerificationMode::Blocking),
        "optimistic" => Ok(ResetVerificationMode::OptimisticAsync),
        other => Err(PyValueError::new_err(format!(
            "reset_verification={other:?}: expected \"blocking\" or \"optimistic\""
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
