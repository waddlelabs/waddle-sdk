//! Marshalling helpers: ndarray/sequence extraction, enum parsing, error
//! mapping. Pure conversion — no policy.

use std::sync::Arc;

use numpy::{PyArray1, PyArrayMethods, PyReadonlyArray1};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyDict;
use waddle_runtime::{ResetHook, ResetSpec, RuntimeError};
use waddle_types::pb::v0 as pb;
use waddle_types::{
    ActionSpace, ActorKind, HandoffPolicy, LeaseEnforcement, ResetVerificationMode, SpaceSpec,
    TerminalOutcome,
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

/// The declared `Composite` layout: each part's name and width, in
/// declaration order — which IS the layout of the concatenated action
/// vector, so slicing a whole-robot row into parts is arithmetic over the
/// customer's own declaration and never an invention.
///
/// Built once per session and shared (`Arc`) by every payload path that can
/// carry an intervention into Python: the dispatched-chunk steps
/// ([`crate::verbs::PySend`]) and the gate's Substitute/Blend returns
/// ([`crate::episode::PyEpisode`]). Absent — and therefore the flat
/// float64-ndarray surface, unchanged — for every declaration that is not a
/// `Composite` of parts with known widths.
pub(crate) struct PartsLayout {
    parts: Vec<(String, usize)>,
    /// The declared space's own width: the parts' widths summed.
    total: usize,
}

/// The declared action space, parsed once per session.
///
/// `None` when the declaration has none or cannot be parsed — `build()` is
/// about to refuse it with the real reason, and a second, worse-worded copy
/// of that refusal here would only get in the way.
pub(crate) fn declared_space(robot: &pb::RobotDescription) -> Option<Arc<ActionSpace>> {
    Some(Arc::new(
        ActionSpace::from_pb(robot.action_space.as_ref()?).ok()?,
    ))
}

impl PartsLayout {
    /// The layout of a declared space, or `None` if it declares no parts.
    ///
    /// A `Composite` part with no declared width (an opaque one) yields
    /// `None` too: such a space is not executable at all
    /// (`ActionSpace::dims` is `None`, and the core's own chunk intake
    /// refuses it), so there is no payload for a layout to key.
    pub(crate) fn of(space: &ActionSpace) -> Option<Arc<Self>> {
        let SpaceSpec::Composite { parts } = &space.spec else {
            return None;
        };
        let mut layout = Vec::with_capacity(parts.len());
        let mut total = 0usize;
        for (name, part) in parts {
            let dims = part.dims()?;
            total += dims;
            layout.push((name.clone(), dims));
        }
        Some(Arc::new(Self {
            parts: layout,
            total,
        }))
    }

    /// One whole-robot row cut into its declared parts, in declaration
    /// order.
    pub(crate) fn split<'a>(&self, values: &'a [f64]) -> PyResult<Vec<(&str, &'a [f64])>> {
        if values.len() != self.total {
            return Err(PyValueError::new_err(format!(
                "a whole-robot action carries the declared space's {} rows (its parts \
                 concatenated in declaration order); got {}",
                self.total,
                values.len()
            )));
        }
        let mut out = Vec::with_capacity(self.parts.len());
        let mut at = 0usize;
        for (name, width) in &self.parts {
            out.push((name.as_str(), &values[at..at + width]));
            at += width;
        }
        Ok(out)
    }

    /// One intervention payload keyed by part — the ONE rule a Composite
    /// session's Python surface follows, wherever the payload crosses:
    ///
    /// * a part-scoped action (`part = Some`) is that part alone, at that
    ///   part's width. The parts it does not name carry no command at all —
    ///   "move this part, hold the rest" (docs/FSM.md §4), and an absent key
    ///   is how that is said;
    /// * a whole-robot action is every declared part, sliced;
    /// * a gripper-only action — "hold the arm, move the gripper" — has no
    ///   rows to carry, so its parts map to EMPTY arrays: the key set still
    ///   says which arms are being held, and the gripper rides the step's
    ///   own `gripper` slot as it always has.
    pub(crate) fn by_part<'py>(
        &self,
        py: Python<'py>,
        values: &[f64],
        part: Option<&str>,
    ) -> PyResult<Bound<'py, PyDict>> {
        let out = PyDict::new(py);
        match part {
            Some(name) => out.set_item(name, PyArray1::from_slice(py, values))?,
            None if values.is_empty() => {
                for (name, _) in &self.parts {
                    out.set_item(name.as_str(), PyArray1::<f64>::from_slice(py, &[]))?;
                }
            }
            None => {
                for (name, rows) in self.split(values)? {
                    out.set_item(name, PyArray1::from_slice(py, rows))?;
                }
            }
        }
        Ok(out)
    }
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
        RuntimeError::MissingRobot
        | RuntimeError::Types(_)
        | RuntimeError::InvalidTaskMetadata(_) => PyValueError::new_err(e.to_string()),
        _ => PyRuntimeError::new_err(e.to_string()),
    }
}
