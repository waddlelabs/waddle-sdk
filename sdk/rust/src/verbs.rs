//! Python verb callables wrapped as core verb traits. Invoked only on the
//! core's serialized `waddle-verbs` dispatch thread (already
//! catch_unwind-wrapped). An exception becomes `VerbError::Failed(repr)`
//! feeding the existing outcome pump — a raising `hold()` is a failed verb,
//! never a poisoned core. `Python::try_attach` (not `attach`) guards the
//! interpreter-finalization window: after Python begins shutdown, verbs
//! fail cleanly instead of aborting.

use std::sync::Arc;

use numpy::PyArray1;
use pyo3::prelude::*;
use pyo3::types::PyList;
use waddle_runtime::{SendVerb, UnitVerb, VerbError};
use waddle_types::ActionChunk;

use crate::convert::PartsLayout;

fn py_err_repr(py: Python<'_>, err: &PyErr) -> String {
    let value = err.value(py);
    match value.repr() {
        Ok(repr) => repr.to_string(),
        Err(_) => err.to_string(),
    }
}

fn unavailable() -> Result<(), VerbError> {
    Err(VerbError::Failed("Python interpreter unavailable".into()))
}

/// A no-argument verb callable (`hold`, `resume`, `home`, `estop`).
pub(crate) struct PyUnit {
    pub cb: Py<PyAny>,
}

/// A pre/post-reset hook callable, wrapped for `ResetSpec::Hook`
/// (`Arc<dyn Fn(&str) -> (bool, Option<bool>) + Send + Sync>`): invoked
/// with the episode's task string, on whichever thread the hook naturally
/// runs on (the caller's own thread for an inline pre-reset, the
/// `waddle-reset-hooks` pump thread otherwise — see `waddle-runtime`'s
/// `ResetSpec` doc). Same GIL/shutdown shape as [`PyUnit`]: `try_attach`
/// so an interpreter-finalization window degrades to `(false, None)`
/// instead of blocking or aborting.
pub(crate) struct PyResetHook {
    pub cb: Py<PyAny>,
}

impl PyResetHook {
    /// Call the hook with `task`; MUST NOT panic or unwind into Rust — this
    /// runs on a core-owned thread. Normalizes every outcome defensively:
    /// a raised exception, or a return value that is neither `bool` nor
    /// `(bool, Optional[bool])`, is reported via `PyErr::write_unraisable`
    /// (the same "log, don't propagate" mechanism CPython uses for
    /// background-thread/destructor callbacks — there is no result channel
    /// back to the caller here, unlike verb dispatch's `VerbError`) and
    /// normalized to `(false, None)`.
    pub(crate) fn call(&self, task: &str) -> (bool, Option<bool>) {
        Python::try_attach(|py| match self.cb.bind(py).call1((task,)) {
            Ok(value) => normalize_hook_result(py, &value),
            Err(err) => {
                err.write_unraisable(py, Some(self.cb.bind(py)));
                (false, None)
            }
        })
        .unwrap_or((false, None))
    }
}

/// `bool` -> `(bool, Some(bool))`: a hook that only reports success is read
/// as also vouching for it — the same "no distinct verification opinion"
/// default the no-spec pipeline already uses (`(true, Some(true))`), and
/// the only reading under which a bare-`True` hook reaches READY by itself
/// under the default `Blocking` verification mode (which requires
/// `verified = Some(true)`) rather than hanging in RESETTING forever.
/// `(bool, Optional[bool])` -> as-is. Anything else is reported as
/// unraisable and normalized to `(false, None)`.
fn normalize_hook_result(py: Python<'_>, value: &Bound<'_, PyAny>) -> (bool, Option<bool>) {
    if let Ok(ok) = value.extract::<bool>() {
        return (ok, Some(ok));
    }
    if let Ok((ok, verified)) = value.extract::<(bool, Option<bool>)>() {
        return (ok, verified);
    }
    let repr = value
        .repr()
        .map(|r| r.to_string())
        .unwrap_or_else(|_| "<unrepr-able>".to_owned());
    let err = pyo3::exceptions::PyTypeError::new_err(format!(
        "reset hook must return bool or (bool, Optional[bool]); got {repr}"
    ));
    err.write_unraisable(py, Some(value));
    (false, None)
}

impl UnitVerb for PyUnit {
    fn call(&self) -> Result<(), VerbError> {
        Python::try_attach(|py| {
            self.cb
                .bind(py)
                .call0()
                .map(|_| ())
                .map_err(|e| VerbError::Failed(py_err_repr(py, &e)))
        })
        .unwrap_or_else(unavailable)
    }
}

/// One dispatched chunk crossing into Python: `steps` is a list of
/// `(values, gripper, offset_ns)` tuples.
///
/// `values` is a float64 ndarray of the declared action space's width — with
/// one shape change, and only on a `Composite` declaration, where it is
/// instead a `dict` of that step's rows keyed by declared part (see
/// [`PartsLayout::by_part`]). A robot with named parts is a robot whose
/// intervenor may address one of them, and a bare 7-row array out of a
/// 14-row cell cannot say which arm it commands.
#[pyclass(name = "Chunk", frozen)]
pub(crate) struct PyChunk {
    #[pyo3(get)]
    steps: Py<PyList>,
    #[pyo3(get)]
    provenance: String,
    #[pyo3(get)]
    seq: u64,
}

/// The `send` verb callable: receives a [`PyChunk`].
pub(crate) struct PySend {
    pub cb: Py<PyAny>,
    /// The declared parts layout, `Some` iff this session declared a
    /// `Composite` space — the ONE thing that decides a step's values shape.
    /// Computed once at `create_session` from the declaration itself, never
    /// per step and never per tick.
    pub parts: Option<Arc<PartsLayout>>,
}

impl SendVerb for PySend {
    fn send(&self, chunk: &ActionChunk) -> Result<(), VerbError> {
        Python::try_attach(|py| {
            let build_and_call = || -> PyResult<()> {
                let steps = PyList::empty(py);
                for step in &chunk.steps {
                    let values: Bound<'_, PyAny> = match &self.parts {
                        Some(layout) => layout
                            .by_part(py, &step.values, step.part.as_deref())?
                            .into_any(),
                        None => PyArray1::from_slice(py, &step.values).into_any(),
                    };
                    steps.append((values, step.gripper, step.offset_ns))?;
                }
                let pychunk = Bound::new(
                    py,
                    PyChunk {
                        steps: steps.unbind(),
                        provenance: chunk.provenance.provenance.to_string(),
                        seq: chunk.seq,
                    },
                )?;
                self.cb.bind(py).call1((pychunk,)).map(|_| ())
            };
            build_and_call().map_err(|e| VerbError::Failed(py_err_repr(py, &e)))
        })
        .unwrap_or_else(unavailable)
    }
}
