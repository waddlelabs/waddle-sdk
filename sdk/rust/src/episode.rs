//! The episode surface. `waddle_runtime::Episode` is Send but !Sync and
//! `gate()` needs `&mut`, so it lives behind a `parking_lot::Mutex` inside a
//! frozen pyclass: `Mutex<T: Send>` is Sync (satisfying pyo3) and cross-
//! thread touches from Python (GC finalizers, watchdog threads reading
//! `ep.done`) stay sound instead of hard-erroring. The uncontended lock is
//! ~20 ns against the ~1 µs Python-call floor; the sacred core `Gate::gate`
//! fast path is untouched.

use std::sync::Arc;

use numpy::PyArray1;
use parking_lot::Mutex;
use pyo3::prelude::*;
use waddle_gate::gate::GateOutput;

use crate::convert::{PartsLayout, extract_f64s, outcome_str, parse_outcome};

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
    /// The declared PART a substitute/blend action addressed, when it
    /// addressed one (`Action.part`, flag `waddle.v0.parts`): the returned
    /// array is that part's rows, in that part's order — not the whole
    /// declared space. `None` means the whole robot, which is every action
    /// on a non-Composite declaration.
    ///
    /// Without this, a 7-row command for one arm of a 14-row bimanual cell
    /// arrives indistinguishable from a whole-robot command — and the core
    /// declares `waddle.v0.parts` for every Composite declaration, so a
    /// plane may send one.
    #[pyo3(get)]
    part: Option<String>,
}

#[pymethods]
impl GateInfo {
    fn __repr__(&self) -> String {
        format!(
            "GateInfo(kind={:?}, provenance={:?}, progress={:?}, gripper={:?}, part={:?})",
            self.kind, self.provenance, self.progress, self.gripper, self.part
        )
    }
}

#[pyclass(name = "Episode", frozen)]
pub(crate) struct PyEpisode {
    inner: Mutex<waddle_runtime::Episode>,
    /// Session handle + id copies so `done`/`outcome`/`terminate` never
    /// take the `inner` mutex: a blocking `terminate` (GIL released) must
    /// not freeze other Python threads calling `gate`/`done` (which hold
    /// the GIL while waiting on `inner`).
    session: waddle_runtime::Session,
    id: waddle_types::EpisodeId,
    last: Mutex<Option<GateInfo>>,
    /// The session's declared parts layout (`Some` iff `Composite`), shared
    /// with the `send` verb so both payload paths key by part identically.
    parts: Option<Arc<PartsLayout>>,
}

impl PyEpisode {
    pub(crate) fn new(episode: waddle_runtime::Episode, parts: Option<Arc<PartsLayout>>) -> Self {
        Self {
            session: episode.session().clone(),
            id: episode.id().clone(),
            inner: Mutex::new(episode),
            last: Mutex::new(None),
            parts,
        }
    }

    /// One Substitute/Blend action as Python sees it: a flat float64 ndarray,
    /// or — on a `Composite` declaration — the same rows keyed by the part
    /// they command.
    fn payload(
        &self,
        py: Python<'_>,
        action: &waddle_gate::gate::OwnedAction,
    ) -> PyResult<Py<PyAny>> {
        Ok(match &self.parts {
            Some(layout) => layout
                .by_part(py, &action.values, action.part.as_deref())?
                .into_any()
                .unbind(),
            None => PyArray1::from_slice(py, &action.values).into_any().unbind(),
        })
    }
}

#[pymethods]
impl PyEpisode {
    #[getter]
    fn id(&self) -> String {
        self.id.to_string()
    }

    /// The gate: returns what you should send — your own `action` object on
    /// Pass (identity-preserved), a fresh float64 ndarray on
    /// Substitute/Blend, or `None` when you must not send (Noop/Hold).
    /// Synchronous and fast; keeps the GIL (it never blocks).
    ///
    /// A Substitute/Blend array is the declared action space's width, with
    /// one exception named by `last_gate`: a gripper-only action — "hold the
    /// arm, move the gripper" — is an EMPTY array with `info.gripper` set
    /// (command the gripper, leave the arm target where it was).
    ///
    /// On a `Composite` declaration a Substitute/Blend is instead a **dict
    /// keyed by declared part** — `{"right": ndarray}` for an action
    /// addressing one arm ("move this part, hold the rest": the parts absent
    /// from the dict are commanded nothing), every declared part for a
    /// whole-robot one. `info.part` names the addressed part either way
    /// (`None` = the whole robot). Pass returns your own object and
    /// Noop/Hold return `None` on every declaration, unchanged.
    ///
    /// The record captures the values at call time; mutating the array
    /// afterwards (before your `send`) makes the dispatched action diverge
    /// from the recorded one.
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
                    part: None,
                },
            ),
            GateOutput::Substitute { action, provenance } => (
                self.payload(py, &action)?,
                GateInfo {
                    kind: "substitute".into(),
                    provenance: Some(provenance.provenance.to_string()),
                    progress: None,
                    gripper: action.gripper,
                    part: action.part.as_ref().map(ToString::to_string),
                },
            ),
            GateOutput::Blend {
                action,
                progress,
                provenance,
            } => (
                self.payload(py, &action)?,
                GateInfo {
                    kind: "blend".into(),
                    provenance: Some(provenance.provenance.to_string()),
                    progress: Some(progress),
                    gripper: action.gripper,
                    part: action.part.as_ref().map(ToString::to_string),
                },
            ),
            GateOutput::Noop { provenance } => (
                py.None(),
                GateInfo {
                    kind: "noop".into(),
                    provenance: Some(provenance.provenance.to_string()),
                    progress: None,
                    gripper: None,
                    part: None,
                },
            ),
            GateOutput::Hold => (
                py.None(),
                GateInfo {
                    kind: "hold".into(),
                    provenance: None,
                    progress: None,
                    gripper: None,
                    part: None,
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

    /// True once the episode ended: a terminal outcome, POST_RESET entry
    /// (the terminal outcome is already pinned there — E14; only the scene
    /// cleanup, which self-resolves via a hook, a remote window, or its
    /// timeout, is still running — so `terminate()` becomes a no-op from
    /// this point on), a successor replaced it, or the session shut down
    /// (single read of the core mirror).
    #[getter]
    fn done(&self) -> bool {
        self.session.episode_done(&self.id)
    }

    /// The terminal outcome as a string, or `None` while running. Reads the
    /// pinned outcome once `done` flips true at POST_RESET entry (before
    /// `Phase::Terminal` itself) — the two are always the same value
    /// (FSM.md E15-E17 carry the pinned outcome to Terminal unchanged), so
    /// a `while not done: ...` caller sees a real outcome the instant
    /// `done` does, never a spurious `None`.
    #[getter]
    fn outcome(&self) -> Option<&'static str> {
        let s = self.session.status();
        s.outcome.or(s.pinned_outcome).map(outcome_str)
    }

    /// PERMANENT once set: the post-reset cleanup failed (a `False`/invalid
    /// hook result, exhausted retries, or an estop during POST_RESET —
    /// FSM.md E16/E17). Never alters the (already pinned) outcome.
    #[getter]
    fn post_reset_failed(&self) -> bool {
        self.session.status().post_reset_failed
    }

    /// Gate records dropped because the recording fell behind the loop.
    /// Nonzero means training-data loss.
    #[getter]
    fn records_dropped(&self) -> u64 {
        self.inner.lock().records_dropped()
    }

    /// Terminate with an outcome ("success" | "failure" | "abort"); blocks
    /// until the core confirms the terminal state (GIL released). A no-op
    /// when this episode is no longer the live one — it never terminates a
    /// successor.
    #[pyo3(signature = (outcome = "abort", reason = ""))]
    fn terminate(&self, py: Python<'_>, outcome: &str, reason: &str) -> PyResult<()> {
        let outcome = parse_outcome(outcome)?;
        py.detach(|| self.session.terminate_episode(&self.id, outcome, reason));
        Ok(())
    }
}
