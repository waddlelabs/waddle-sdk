//! Python verb callables wrapped as core verb traits. Invoked only on the
//! core's serialized `waddle-verbs` dispatch thread (already
//! catch_unwind-wrapped). An exception becomes `VerbError::Failed(repr)`
//! feeding the existing outcome pump — a raising `hold()` is a failed verb,
//! never a poisoned core. `Python::try_attach` (not `attach`) guards the
//! interpreter-finalization window: after Python begins shutdown, verbs
//! fail cleanly instead of aborting.

use numpy::PyArray1;
use pyo3::prelude::*;
use pyo3::types::PyList;
use waddle_runtime::{SendVerb, UnitVerb, VerbError};
use waddle_types::ActionChunk;

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
/// `(float64 ndarray, gripper, offset_ns)` tuples.
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
}

impl SendVerb for PySend {
    fn send(&self, chunk: &ActionChunk) -> Result<(), VerbError> {
        Python::try_attach(|py| {
            let build_and_call = || -> PyResult<()> {
                let steps = PyList::empty(py);
                for step in &chunk.steps {
                    let values = PyArray1::from_slice(py, &step.values);
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
