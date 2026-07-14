//! The episode surface. `waddle_runtime::Episode` is Send but !Sync and
//! `gate()` needs `&mut`, so it lives behind a `parking_lot::Mutex` inside a
//! frozen pyclass: `Mutex<T: Send>` is Sync (satisfying pyo3) and cross-
//! thread touches from Python (GC finalizers, watchdog threads reading
//! `ep.done`) stay sound instead of hard-erroring. The uncontended lock is
//! ~20 ns against the ~1 µs Python-call floor; the sacred core `Gate::gate`
//! fast path is untouched.

use numpy::PyArray1;
use parking_lot::Mutex;
use pyo3::prelude::*;
use waddle_gate::gate::GateOutput;

use crate::convert::{extract_f64s, outcome_str, parse_outcome};

/// What the last `gate()` call decided (marshalled from the core decision —
/// Python never computes any of this).
#[pyclass(name = "GateInfo", frozen, skip_from_py_object)]
#[derive(Clone)]
pub(crate) struct GateInfo {
    /// "pass" | "substitute" | "blend" | "noop" | "hold"
    #[pyo3(get)]
    kind: String,
    /// Core-computed provenance, verbatim ("policy", "teleop", ...); None
    /// on Hold (no provenance attaches to a held tick).
    #[pyo3(get)]
    provenance: Option<String>,
    /// Blend progress in [0, 1]; None outside a blend window.
    #[pyo3(get)]
    progress: Option<f32>,
    /// The substitute/blend action's gripper command, when one rode along.
    #[pyo3(get)]
    gripper: Option<f64>,
}

#[pymethods]
impl GateInfo {
    fn __repr__(&self) -> String {
        format!(
            "GateInfo(kind={:?}, provenance={:?}, progress={:?}, gripper={:?})",
            self.kind, self.provenance, self.progress, self.gripper
        )
    }
}

#[pyclass(name = "Episode", frozen)]
pub(crate) struct PyEpisode {
    inner: Mutex<waddle_runtime::Episode>,
    last: Mutex<Option<GateInfo>>,
}

impl PyEpisode {
    pub(crate) fn new(episode: waddle_runtime::Episode) -> Self {
        Self {
            inner: Mutex::new(episode),
            last: Mutex::new(None),
        }
    }
}

#[pymethods]
impl PyEpisode {
    #[getter]
    fn id(&self) -> String {
        self.inner.lock().id().to_string()
    }

    /// The gate: returns what you should send — your own `action` object on
    /// Pass (identity-preserved), a fresh float64 ndarray on
    /// Substitute/Blend, or `None` when you must not send (Noop/Hold).
    /// Synchronous and fast; keeps the GIL (it never blocks).
    #[pyo3(signature = (action, obs=None, gripper=None))]
    fn gate(
        &self,
        py: Python<'_>,
        action: &Bound<'_, PyAny>,
        obs: Option<&Bound<'_, PyAny>>,
        gripper: Option<f64>,
    ) -> PyResult<Py<PyAny>> {
        let action_row = extract_f64s(action)?;
        let obs_row = obs.map(extract_f64s).transpose()?;
        let output = self.inner.lock().gate(
            action_row.as_slice(),
            gripper,
            obs_row.as_ref().map(crate::convert::F64s::as_slice),
        );

        let (result, info) = match output {
            GateOutput::Pass { provenance } => (
                action.clone().unbind(),
                GateInfo {
                    kind: "pass".into(),
                    provenance: Some(provenance.provenance.to_string()),
                    progress: None,
                    gripper: None,
                },
            ),
            GateOutput::Substitute { action, provenance } => (
                PyArray1::from_slice(py, &action.values).into_any().unbind(),
                GateInfo {
                    kind: "substitute".into(),
                    provenance: Some(provenance.provenance.to_string()),
                    progress: None,
                    gripper: action.gripper,
                },
            ),
            GateOutput::Blend {
                action,
                progress,
                provenance,
            } => (
                PyArray1::from_slice(py, &action.values).into_any().unbind(),
                GateInfo {
                    kind: "blend".into(),
                    provenance: Some(provenance.provenance.to_string()),
                    progress: Some(progress),
                    gripper: action.gripper,
                },
            ),
            GateOutput::Noop { provenance } => (
                py.None(),
                GateInfo {
                    kind: "noop".into(),
                    provenance: Some(provenance.provenance.to_string()),
                    progress: None,
                    gripper: None,
                },
            ),
            GateOutput::Hold => (
                py.None(),
                GateInfo {
                    kind: "hold".into(),
                    provenance: None,
                    progress: None,
                    gripper: None,
                },
            ),
        };
        *self.last.lock() = Some(info);
        Ok(result)
    }

    /// What the last `gate()` call decided; `None` before the first call.
    #[getter]
    fn last_gate(&self) -> Option<GateInfo> {
        self.last.lock().clone()
    }

    /// True once the episode reached a terminal outcome (single read of the
    /// core mirror).
    #[getter]
    fn done(&self) -> bool {
        self.inner.lock().done()
    }

    /// The terminal outcome as a string, or `None` while running.
    #[getter]
    fn outcome(&self) -> Option<&'static str> {
        self.inner.lock().outcome().map(outcome_str)
    }

    /// Terminate with an outcome ("success" | "failure" | "abort"); blocks
    /// until the core confirms the terminal state (GIL released).
    #[pyo3(signature = (outcome = "abort", reason = ""))]
    fn terminate(&self, py: Python<'_>, outcome: &str, reason: &str) -> PyResult<()> {
        let outcome = parse_outcome(outcome)?;
        py.detach(|| self.inner.lock().terminate(outcome, reason));
        Ok(())
    }
}
