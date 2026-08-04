//! The session surface + `create_session`.
//!
//! GIL discipline (blanket rule: every core call that can block detaches):
//!
//! | shim call                          | blocks on                       | GIL |
//! |------------------------------------|---------------------------------|-----|
//! | `Session::start_episode`           | reset pipeline + mirror condvar | detach (mandatory) |
//! | `Episode::terminate`               | mirror condvar                  | detach (mandatory) |
//! | `Session::shutdown`                | joins core threads; the verb thread is waited on transitively (the outcome pump only exits once verb dispatch drops its sender) | detach (mandatory — the verb thread may be inside `Python::try_attach` waiting for the GIL we hold: classic deadlock) |
//! | `Episode::gate`, `done`, `outcome` | nothing                         | keep GIL |
//! | `Session::agent`                   | a whole agent-driven episode    | detach in short slices (see [`PySession::agent`]) |

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{RecvTimeoutError, channel};
use std::time::Duration;

use bytes::Bytes;
use numpy::{PyArray3, PyArrayMethods, PyUntypedArrayMethods};
use parking_lot::Mutex;
use pyo3::exceptions::{PyRuntimeError, PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyBytes;
use waddle_media::{DataTopic, LoopbackFarEnd, LoopbackMedia};
use waddle_runtime::{
    AgentOutcome, ControlRegistry, EePose, EpisodeOptions, EstopDecl, FrameData, ProprioReport,
    Session,
};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActorKind, TerminalOutcome};

use crate::convert::{
    PartsLayout, extract_f64s, outcome_str, parse_enforcement, parse_handoff, parse_outcome,
    parse_reset_spec, parse_robot_json, parse_verification_mode, runtime_err,
};
use crate::episode::PyEpisode;
use crate::verbs::{PySend, PyUnit};

/// Default reset-window deadline (10 minutes) — matches the design's
/// `TeleopReset`/`AgentReset` default, used whenever a caller declares a
/// remote reset phase without an explicit timeout.
const DEFAULT_RESET_TIMEOUT_NS: i64 = 600_000_000_000;

/// How long each GIL-released slice of [`PySession::agent`]'s wait lasts
/// before the calling thread reattaches to run Python's pending signal
/// handlers. Short enough that Ctrl-C reads as immediate, long enough that
/// an episode running for minutes costs nothing to wait on.
const AGENT_POLL_SLICE: Duration = Duration::from_millis(50);

/// The result of one agent-driven episode — a verbatim marshalling of
/// `waddle_runtime::AgentOutcome` (flag `waddle.v0.agent`). Every field is
/// core's word: this class computes nothing.
#[pyclass(name = "AgentResult", frozen, skip_from_py_object)]
pub(crate) struct AgentResult {
    /// "success" | "failure" | "abort" | "aborted_retake" — the same
    /// spelling `Episode.outcome` uses.
    #[pyo3(get)]
    outcome: &'static str,
    #[pyo3(get)]
    episode_id: String,
    /// The opaque Waddle-side recording reference from the plane's
    /// `AgentTaskUpdate{COMPLETED}`, when one arrived for this episode.
    #[pyo3(get)]
    recording_ref: Option<String>,
    /// The plane's last `AgentTaskUpdate.detail` for this episode (a
    /// DENIED's reason, or COMPLETED's summary); empty when none arrived.
    #[pyo3(get)]
    detail: String,
}

impl From<AgentOutcome> for AgentResult {
    fn from(outcome: AgentOutcome) -> Self {
        Self {
            outcome: outcome_str(outcome.outcome),
            episode_id: outcome.episode_id.to_string(),
            recording_ref: outcome.recording_ref,
            detail: outcome.detail,
        }
    }
}

#[pymethods]
impl AgentResult {
    fn __repr__(&self) -> String {
        format!(
            "AgentResult(outcome={:?}, episode_id={:?}, recording_ref={:?}, detail={:?})",
            self.outcome, self.episode_id, self.recording_ref, self.detail
        )
    }
}

/// Build one episode's [`EpisodeOptions`] from the eight per-phase reset
/// kwargs `start_episode` and `agent` share. `None` for a `*_kind` inherits
/// the session default for that phase (see [`PySession::start_episode`]);
/// pure kwarg-to-type mapping, zero reset logic.
#[allow(clippy::too_many_arguments)]
fn episode_options(
    pre_reset_kind: Option<&str>,
    pre_reset_hook: Option<Py<PyAny>>,
    pre_reset_prompt: Option<&str>,
    pre_reset_timeout_ns: i64,
    post_reset_kind: Option<&str>,
    post_reset_hook: Option<Py<PyAny>>,
    post_reset_prompt: Option<&str>,
    post_reset_timeout_ns: i64,
) -> PyResult<EpisodeOptions> {
    let pre_reset = pre_reset_kind
        .map(|kind| {
            parse_reset_spec(
                "pre_reset",
                kind,
                pre_reset_hook,
                pre_reset_prompt,
                pre_reset_timeout_ns,
            )
        })
        .transpose()?;
    let post_reset = post_reset_kind
        .map(|kind| {
            parse_reset_spec(
                "post_reset",
                kind,
                post_reset_hook,
                post_reset_prompt,
                post_reset_timeout_ns,
            )
        })
        .transpose()?;
    Ok(EpisodeOptions {
        pre_reset,
        post_reset,
        // Append-proof against runtime-side EpisodeOptions growth.
        // `agent_invite` is NOT set here: `Session::run_agent` owns it (it
        // is the invite, not an episode option a caller composes).
        ..EpisodeOptions::default()
    })
}

#[pyclass(name = "Session", frozen)]
pub(crate) struct PySession {
    /// `waddle_runtime::Session` is Clone + Send + Sync (Arc inner) —
    /// `frozen` fits.
    inner: Session,
    closed: AtomicBool,
    /// The loopback far end, present iff `testing_loopback` (the private
    /// `waddle._testing` surface drives it).
    testing_far: Option<Mutex<LoopbackFarEnd>>,
    teleop_seq: AtomicU64,
    /// [`PySession::_testing_push_chunk`]'s own stream sequence — monotone
    /// per stream, and a different stream from the teleop packets above.
    chunk_seq: AtomicU64,
    /// The declared parts layout (`Some` iff `Composite`), handed to every
    /// episode so a gate return is keyed by part exactly as a dispatched
    /// chunk's steps are.
    parts: Option<Arc<PartsLayout>>,
}

/// Dropping an un-shutdown session must not run the core's blocking
/// teardown (thread joins, transitively the verb thread) while holding the
/// GIL — a verb callback mid-`try_attach` would deadlock. Dealloc happens
/// with the GIL held, so detach around the join. The blessed
/// `waddle.init`/`waddle.shutdown` path never reaches this (atexit calls
/// shutdown first); this covers direct `_core.create_session` users.
impl Drop for PySession {
    fn drop(&mut self) {
        if self.closed.swap(true, Ordering::SeqCst) {
            return;
        }
        let session = self.inner.clone();
        if Python::try_attach(|py| py.detach(|| session.shutdown())).is_none() {
            // Interpreter unavailable (finalization): verb callbacks can no
            // longer block on the GIL, so joining directly is safe.
            self.inner.clone().shutdown();
        }
    }
}

impl PySession {
    fn testing_far(&self) -> PyResult<&Mutex<LoopbackFarEnd>> {
        self.testing_far.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err(
                "test hooks require waddle.init(_testing=True) (loopback media plane)",
            )
        })
    }

    /// Request an abort of the live agent-invited episode — what a Ctrl-C
    /// during [`PySession::agent`] means. The single mirror snapshot only
    /// NAMES the episode this call is waiting on (both fields come from the
    /// same read, and `agent_invited` stays readable through TERMINAL); the
    /// abort itself is the core operation `Episode.terminate()` already
    /// exposes, and the core makes it a no-op once that episode is no
    /// longer the live one or its outcome is already pinned.
    fn abort_live_agent_episode(&self, py: Python<'_>) {
        let status = self.inner.status();
        let Some(id) = status.episode_id.filter(|_| status.agent_invited) else {
            return;
        };
        py.detach(|| {
            self.inner
                .terminate_episode(&id, TerminalOutcome::Abort, "keyboard interrupt")
        });
    }
}

#[pymethods]
impl PySession {
    /// Open an episode; blocks through the reset pipeline (GIL released).
    /// Every `*_reset_kind` kwarg defaults to `None` (inherit the session's
    /// declared default for that phase, exactly as plain `start_episode`
    /// did before these existed); passing one overrides that phase for
    /// this episode only — `"none"` disables it, `"hook"`/`"teleop"`/
    /// `"agent"` mirror `create_session`'s own kinds. See
    /// `waddle_runtime::EpisodeOptions` for the inherit/disable contract.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (
        task,
        pre_reset_kind=None,
        pre_reset_hook=None,
        pre_reset_prompt=None,
        pre_reset_timeout_ns=DEFAULT_RESET_TIMEOUT_NS,
        post_reset_kind=None,
        post_reset_hook=None,
        post_reset_prompt=None,
        post_reset_timeout_ns=DEFAULT_RESET_TIMEOUT_NS,
    ))]
    fn start_episode(
        &self,
        py: Python<'_>,
        task: &str,
        pre_reset_kind: Option<&str>,
        pre_reset_hook: Option<Py<PyAny>>,
        pre_reset_prompt: Option<&str>,
        pre_reset_timeout_ns: i64,
        post_reset_kind: Option<&str>,
        post_reset_hook: Option<Py<PyAny>>,
        post_reset_prompt: Option<&str>,
        post_reset_timeout_ns: i64,
    ) -> PyResult<PyEpisode> {
        let opts = episode_options(
            pre_reset_kind,
            pre_reset_hook,
            pre_reset_prompt,
            pre_reset_timeout_ns,
            post_reset_kind,
            post_reset_hook,
            post_reset_prompt,
            post_reset_timeout_ns,
        )?;
        let session = self.inner.clone();
        let task = task.to_owned();
        let episode = py
            .detach(move || session.start_episode_with(&task, opts))
            .map_err(runtime_err)?;
        Ok(PyEpisode::new(episode, self.parts.clone()))
    }

    /// Ask Waddle to drive one episode (flag `waddle.v0.agent`): opens an
    /// agent-invited episode (`prompt` is both the invite prompt and the
    /// episode task) and BLOCKS until it reaches a terminal outcome,
    /// returning an [`AgentResult`]. `timeout_ns` is the invite deadline —
    /// no agent claim engaged in time aborts the episode (FSM.md E25), as
    /// does a plane DENIED before engage (E26). The reset kwargs are
    /// `start_episode`'s, unchanged.
    ///
    /// While this blocks, the caller's thread is not ticking `gate()` —
    /// that is the point: the invited agent claims through the EXISTING
    /// intervention machinery and the core's bypass pump drives the
    /// registered `send`. Everything about that lives in core; this method
    /// marshals a prompt in and an outcome out.
    ///
    /// # Interruption
    ///
    /// `Session::run_agent` blocks for the whole episode (minutes), so it
    /// runs on a dedicated `waddle-py-agent` thread while THIS thread waits
    /// in short GIL-released slices, running Python's signal handlers
    /// between them. A Ctrl-C therefore does not sit unheard until the
    /// deadline: it asks the core to end the live agent-invited episode
    /// (the same abort `Episode.terminate()` requests — the shim decides
    /// nothing about the timeline, and the core no-ops if that episode is
    /// no longer live), and keeps asking on every later slice until the run
    /// reports finished. Re-asking is what makes the promise hold at the
    /// edges: a signal consumed before the run thread has published its
    /// episode — or while a predecessor's POST_RESET still owns the mirror —
    /// would otherwise abort nothing at all and never be reconsidered,
    /// leaving the caller blocked to the invite deadline with the interrupt
    /// already latched. The call still returns only once the core reports
    /// the run finished, so the robot is never left driven by an agent whose
    /// caller has walked away, and the thread is never orphaned;
    /// `KeyboardInterrupt` is raised then.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (
        prompt,
        timeout_ns,
        pre_reset_kind=None,
        pre_reset_hook=None,
        pre_reset_prompt=None,
        pre_reset_timeout_ns=DEFAULT_RESET_TIMEOUT_NS,
        post_reset_kind=None,
        post_reset_hook=None,
        post_reset_prompt=None,
        post_reset_timeout_ns=DEFAULT_RESET_TIMEOUT_NS,
    ))]
    fn agent(
        &self,
        py: Python<'_>,
        prompt: &str,
        timeout_ns: i64,
        pre_reset_kind: Option<&str>,
        pre_reset_hook: Option<Py<PyAny>>,
        pre_reset_prompt: Option<&str>,
        pre_reset_timeout_ns: i64,
        post_reset_kind: Option<&str>,
        post_reset_hook: Option<Py<PyAny>>,
        post_reset_prompt: Option<&str>,
        post_reset_timeout_ns: i64,
    ) -> PyResult<AgentResult> {
        let opts = episode_options(
            pre_reset_kind,
            pre_reset_hook,
            pre_reset_prompt,
            pre_reset_timeout_ns,
            post_reset_kind,
            post_reset_hook,
            post_reset_prompt,
            post_reset_timeout_ns,
        )?;
        let session = self.inner.clone();
        let prompt = prompt.to_owned();
        let (tx, rx) = channel();
        // Detached, not joined: the thread's last act is this send, so it
        // is already unwinding by the time we read it — and joining while
        // holding the GIL is exactly the deadlock shape `shutdown` warns
        // about.
        std::thread::Builder::new()
            .name("waddle-py-agent".to_owned())
            .spawn(move || {
                let _ = tx.send(session.run_agent(&prompt, timeout_ns, opts));
            })
            .map_err(|e| PyRuntimeError::new_err(format!("failed to spawn the agent run: {e}")))?;

        // The receiver is used only by this thread; the mutex is what makes
        // it shareable into the detached wait at all (`Receiver` is Send,
        // not Sync), never contention.
        let rx = Mutex::new(rx);
        let mut interrupt: Option<PyErr> = None;
        loop {
            match py.detach(|| rx.lock().recv_timeout(AGENT_POLL_SLICE)) {
                Ok(result) => {
                    // An interrupt that already landed wins over whatever
                    // the (aborted) run reported — the caller asked to stop.
                    return match interrupt {
                        Some(err) => Err(err),
                        None => result.map(AgentResult::from).map_err(runtime_err),
                    };
                }
                Err(RecvTimeoutError::Timeout) => {
                    if let Err(err) = py.check_signals() {
                        interrupt.get_or_insert(err);
                    }
                    // Ask on the slice that consumed the signal AND on every
                    // slice after it: the first ask can land in a window
                    // where there is no live agent-invited episode yet (the
                    // run thread has not opened one, or is still waiting out
                    // a predecessor's POST_RESET), and a one-shot ask would
                    // then abort nothing while the caller stays blocked to
                    // the invite deadline. `terminate_episode` is idempotent
                    // — it returns immediately for an episode already done —
                    // so re-asking costs a mirror read per slice and the ask
                    // that finds the episode blocks until it is terminal.
                    if interrupt.is_some() {
                        self.abort_live_agent_episode(py);
                    }
                }
                // The run thread died without reporting (a panic): there is
                // no outcome to wait for.
                Err(RecvTimeoutError::Disconnected) => {
                    return Err(interrupt.unwrap_or_else(|| {
                        PyRuntimeError::new_err("the agent run ended without an outcome")
                    }));
                }
            }
        }
    }

    /// Join all core threads and flush recorders. Idempotent; blocks with
    /// the GIL released (verb dispatch may need it to finish a callback).
    fn shutdown(&self, py: Python<'_>) {
        if !self.closed.swap(true, Ordering::SeqCst) {
            let session = self.inner.clone();
            py.detach(move || session.shutdown());
        }
    }

    /// Publish one raw video frame for a declared camera. `frame` must be a
    /// numpy `uint8` ndarray shaped `(height, width, 3)` (packed row-major
    /// RGB8 — the only layout this SDK supports today); a C-contiguous
    /// array is copied once into the frame the core queues, never mutated.
    /// Cheap: the declared uplink fps throttle, the lazy `publish_track`,
    /// and the actual encode all run in core, never here (hollow frontend —
    /// this method only marshals a name and an array).
    ///
    /// Raises `TypeError` for a wrong dtype/rank/shape or an array that
    /// isn't C-contiguous (row-major) — `numpy`'s own "contiguous" also
    /// accepts Fortran/column-major order, which would silently transpose
    /// this method's row-major byte assumption instead of raising, so this
    /// checks C-order specifically rather than reusing that broader
    /// definition; `RuntimeError` for an undeclared camera name or a
    /// `frame` whose (height, width) disagrees with that camera's
    /// declaration (both mapped from the core's `RuntimeError`).
    fn publish_frame(&self, camera: &str, frame: &Bound<'_, PyAny>) -> PyResult<()> {
        let arr = frame.cast::<PyArray3<u8>>().map_err(|_| {
            PyTypeError::new_err("frame must be a numpy uint8 ndarray shaped (height, width, 3)")
        })?;
        let ro = arr.readonly();
        let shape = ro.shape();
        if shape.len() != 3 || shape[2] != 3 {
            return Err(PyTypeError::new_err(format!(
                "frame must be shaped (height, width, 3); got {shape:?}"
            )));
        }
        if !ro.is_c_contiguous() {
            return Err(PyTypeError::new_err(
                "frame must be a C-contiguous (row-major) numpy array — a Fortran-ordered \
                 array passes numpy's own (layout-agnostic) contiguity check but would \
                 transpose the pixel bytes this method assumes; call \
                 numpy.ascontiguousarray(frame) first",
            ));
        }
        let slice = ro
            .as_slice()
            .map_err(|_| PyTypeError::new_err("frame must be a contiguous numpy array"))?;
        // numpy image convention: shape is (height, width, channels).
        let (height, width) = (shape[0] as u32, shape[1] as u32);
        let data = FrameData::rgb8(width, height, Bytes::copy_from_slice(slice));
        self.inner.publish_frame(camera, data).map_err(runtime_err)
    }

    /// Report a richer proprioceptive sample than the bare `joint_pos`
    /// every `gate(action, obs)` call already records. Every argument
    /// PATCHES the core's latest known sample — omit one (or pass `None`)
    /// to leave its previously reported value in place (there is no way to
    /// clear one in v0). The merged sample lands in every subsequent
    /// gate-tick's recorded `/waddle/observations` entry (Local recording)
    /// and in the periodic `StreamObservations` uplink whenever a
    /// supervision plane is connected.
    ///
    /// `joint_vel` and `ee_pose` accept a numpy `float64` ndarray or a plain
    /// list (numpy is zero-copy; anything else is a one-time owned copy —
    /// same convention as `gate(action, obs)`). `ee_pose`, when given, must
    /// have exactly 7 values: xyz position followed by a wxyz unit
    /// quaternion (w first — this protocol's pinned convention), expressed
    /// in `ee_pose_frame` (default `"ee"`) — every reported pose must name
    /// its frame (there is no frame-less default at the wire level: an
    /// untagged pose is exactly how misaligned data corrupts a corpus
    /// silently). Raises `ValueError` for a wrong `ee_pose` length or an
    /// empty `ee_pose_frame`.
    #[pyo3(signature = (joint_vel=None, ee_pose=None, ee_pose_frame="ee", gripper=None))]
    fn report_proprio(
        &self,
        joint_vel: Option<&Bound<'_, PyAny>>,
        ee_pose: Option<&Bound<'_, PyAny>>,
        ee_pose_frame: &str,
        gripper: Option<f64>,
    ) -> PyResult<()> {
        let joint_vel = joint_vel
            .map(extract_f64s)
            .transpose()?
            .map(|row| row.as_slice().to_vec());
        let ee_pose = ee_pose
            .map(|obj| {
                let row = extract_f64s(obj)?;
                let values = row.as_slice();
                if values.len() != 7 {
                    return Err(PyValueError::new_err(format!(
                        "ee_pose must have exactly 7 values (xyz position + wxyz \
                         orientation); got {}",
                        values.len()
                    )));
                }
                EePose::new(
                    [values[0], values[1], values[2]],
                    [values[3], values[4], values[5], values[6]],
                    ee_pose_frame,
                )
                .map_err(|e| PyValueError::new_err(e.to_string()))
            })
            .transpose()?;
        // `part`/`joint_pos` are not kwargs on this surface yet: every
        // report from here describes the robot as declared (`part: ""`, the
        // sole/default part), which the core accepts unconditionally. The
        // refusal is still mapped rather than unwrapped — it is a
        // caller-facing validation error the moment `part=` is exposed.
        self.inner
            .report_proprio(ProprioReport {
                joint_vel,
                ee_pose,
                gripper,
                ..Default::default()
            })
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// PRIVATE/UNSTABLE: every raw frame payload the loopback media plane's
    /// far end has received for `camera`, in publish order — lets pytest
    /// observe `publish_frame` without a real transport.
    fn _testing_frames(&self, py: Python<'_>, camera: &str) -> PyResult<Vec<Py<PyAny>>> {
        let far = self.testing_far()?;
        Ok(far
            .lock()
            .frames()
            .into_iter()
            .filter(|(name, _)| name == camera)
            .map(|(_, f)| PyBytes::new(py, &f.data).into_any().unbind())
            .collect())
    }

    /// PRIVATE/UNSTABLE: grant + engage a local claim (what a control-plane
    /// directive would do).
    fn _testing_engage(&self, claim_id: &str, source: &str) -> PyResult<()> {
        self.testing_far()?;
        let actor = match source {
            "teleop" => ActorKind::Teleoperator,
            "agent" => ActorKind::Agent,
            _ => ActorKind::Custom,
        };
        waddle_runtime::grant_and_engage(&self.inner, claim_id, source, actor);
        Ok(())
    }

    /// PRIVATE/UNSTABLE: release the claim.
    fn _testing_release(&self, claim_id: &str) -> PyResult<()> {
        self.testing_far()?;
        waddle_runtime::release_claim(&self.inner, claim_id);
        Ok(())
    }

    /// PRIVATE/UNSTABLE: push one teleop stream packet through the loopback
    /// media plane — the same `DataTopic::TeleopPose` wire packet a
    /// teleoperator console would send (values become a Twist: first three
    /// linear, next three angular).
    #[pyo3(signature = (values, gripper=None))]
    fn _testing_push_teleop(&self, values: Vec<f64>, gripper: Option<f64>) -> PyResult<()> {
        let far = self.testing_far()?;
        let seq = self.teleop_seq.fetch_add(1, Ordering::SeqCst);
        let mut axes = [0.0f64; 6];
        if values.len() > 6 {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "push_teleop takes at most 6 values (linear xyz + angular xyz)",
            ));
        }
        axes[..values.len()].copy_from_slice(&values);
        let packet = pb::TeleopStreamPacket {
            t_client_ns: i64::try_from(seq).unwrap_or(0),
            seq,
            targets: vec![pb::PartTarget {
                part: String::new(),
                target: Some(pb::part_target::Target::Twist(pb::Twist {
                    linear: Some(pb::Vec3 {
                        x: axes[0],
                        y: axes[1],
                        z: axes[2],
                    }),
                    angular: Some(pb::Vec3 {
                        x: axes[3],
                        y: axes[4],
                        z: axes[5],
                    }),
                })),
                gripper,
            }],
            clutch_engaged: true,
            inputs: None,
        };
        far.lock()
            .push(DataTopic::TeleopPose, &packet)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }

    /// PRIVATE/UNSTABLE: push ONE intervention step into the session's
    /// intervention stream — the local counterpart of a plane
    /// `intervention_chunk`, exactly as `_testing_engage` is the local
    /// counterpart of a claim directive. It runs the core's own intake
    /// (`waddle_runtime::push_intervention_chunk`): same validation, same
    /// once-per-claim-window refusals on the episode timeline, and the
    /// jitter buffer still decides when — and whether — the step plays out.
    ///
    /// `part` addresses one declared part by name (`Action.part`, flag
    /// `waddle.v0.parts`). `None` is the whole robot, which on a `Composite`
    /// declaration is marshalled into the `CompositeAction` naming every
    /// part that the wire requires — the split is the declared layout's own
    /// arithmetic, the same one a dispatched step comes back keyed by.
    ///
    /// The target is always the joint-position arm: it is what the
    /// part-addressed declarations declare, and a space that declares
    /// another one refuses this at the intake, which is the intake doing its
    /// job rather than this hook second-guessing it.
    #[pyo3(signature = (values, part=None, gripper=None, offset_ns=0))]
    fn _testing_push_chunk(
        &self,
        values: Vec<f64>,
        part: Option<&str>,
        gripper: Option<f64>,
        offset_ns: i64,
    ) -> PyResult<()> {
        self.testing_far()?;
        let joints = |values: Vec<f64>| {
            Some(pb::action::Target::JointPosition(pb::JointVector {
                values,
            }))
        };
        let target = match (part, &self.parts) {
            // A part-scoped action carries that part's own space directly.
            (Some(_), _) => joints(values),
            // A whole-robot action on a robot with declared parts must name
            // every one of them (`CompositeAction`); nothing is invented,
            // the row is cut where the declaration says.
            (None, Some(layout)) => Some(pb::action::Target::Composite(pb::CompositeAction {
                parts: layout
                    .split(&values)?
                    .into_iter()
                    .map(|(name, rows)| pb::composite_action::PartAction {
                        name: name.to_owned(),
                        action: Some(pb::Action {
                            target: joints(rows.to_vec()),
                            ..Default::default()
                        }),
                    })
                    .collect(),
            })),
            (None, None) => joints(values),
        };
        waddle_runtime::push_intervention_chunk(
            &self.inner,
            pb::ActionChunk {
                actions: vec![pb::Action {
                    target,
                    gripper: gripper.map(|position| pb::GripperCommand {
                        position,
                        effort: None,
                    }),
                    t_offset_ns: offset_ns,
                    part: part.unwrap_or_default().to_owned(),
                }],
                seq: self.chunk_seq.fetch_add(1, Ordering::SeqCst),
                source_id: "waddle.testing".to_owned(),
                ..Default::default()
            },
        );
        Ok(())
    }

    /// PRIVATE/UNSTABLE: engage an already-open reset window — injects
    /// `ClaimGranted` then `ResetWindowEngage` (the same sequence a plane
    /// ENGAGE directive produces), so pytest can drive a remote reset
    /// window without a control-plane transport. `actor` follows
    /// `_testing_engage`'s own convention (`"teleop"` / `"agent"`, else
    /// `Custom`) — pick the one the window under test actually expects
    /// (C6), or the FSM rejects the claim.
    fn _testing_reset_window_engage(&self, claim_id: &str, actor: &str) -> PyResult<()> {
        self.testing_far()?;
        let actor_kind = match actor {
            "teleop" => ActorKind::Teleoperator,
            "agent" => ActorKind::Agent,
            _ => ActorKind::Custom,
        };
        waddle_runtime::reset_window_engage(&self.inner, claim_id, actor, actor_kind);
        Ok(())
    }

    /// PRIVATE/UNSTABLE: end the live episode the way a plane
    /// `EpisodeDirective{MARK_DONE}` does — the directive's runtime-side
    /// half is `SessionEvent::Terminate`, which is exactly what
    /// `terminate_episode` injects (`pumps::forward_server_msg`'s
    /// `Msg::Episode` arm). The live episode is read from the same mirror
    /// snapshot that names it, because a `waddle.agent()` caller holds no
    /// episode handle to terminate through — that is the whole point of a
    /// plane-driven episode, and it is why this seam exists at all.
    /// Blocks through the terminal (and any post-reset) with the GIL
    /// released, like every other terminating call.
    #[pyo3(signature = (outcome="success", reason=""))]
    fn _testing_mark_done(&self, py: Python<'_>, outcome: &str, reason: &str) -> PyResult<()> {
        self.testing_far()?;
        let outcome = parse_outcome(outcome)?;
        let id =
            self.inner.status().episode_id.ok_or_else(|| {
                PyRuntimeError::new_err("no live episode for a MARK_DONE to address")
            })?;
        py.detach(|| self.inner.terminate_episode(&id, outcome, reason));
        Ok(())
    }

    /// PRIVATE/UNSTABLE: complete an engaged reset window — injects
    /// `ResetWindowComplete{claim_id, ok, verified}` (the runtime-side half
    /// of a plane COMPLETE directive).
    #[pyo3(signature = (claim_id, ok, verified=None))]
    fn _testing_reset_window_complete(
        &self,
        claim_id: &str,
        ok: bool,
        verified: Option<bool>,
    ) -> PyResult<()> {
        self.testing_far()?;
        waddle_runtime::reset_window_complete(&self.inner, claim_id, ok, verified);
        Ok(())
    }
}

/// Build the session. Every argument is plain data; the callables cross as
/// opaque objects invoked only on the core's verb-dispatch thread.
///
/// `transport_url`/`media_url` (with their tokens — the plane mints both,
/// this SDK never does) wire the CONNECTED build: the tonic control
/// transport (`grpc` feature) and the LiveKit media plane (`livekit`). Both
/// are compiled out by default; naming one a build does not carry raises
/// rather than degrading to a silent offline session — losing supervision
/// quietly is the one failure mode a supervision layer must not have.
/// `_core.FEATURES` is how the Python layer sees which are present.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
#[pyo3(signature = (
    project,
    robot_json,
    send=None,
    hold=None,
    resume=None,
    home=None,
    estop=None,
    estop_hardware=false,
    estop_latency_bound_ns=None,
    recording_dir=None,
    handoff_kind="hold_first",
    handoff_ns=0,
    lease_enforcement="advisory",
    testing_loopback=false,
    pre_reset_kind="none",
    pre_reset_hook=None,
    pre_reset_prompt=None,
    pre_reset_timeout_ns=DEFAULT_RESET_TIMEOUT_NS,
    post_reset_kind="none",
    post_reset_hook=None,
    post_reset_prompt=None,
    post_reset_timeout_ns=DEFAULT_RESET_TIMEOUT_NS,
    reset_verification="blocking",
    transport_url=None,
    transport_token=None,
    media_url=None,
    media_token=None,
))]
pub(crate) fn create_session(
    py: Python<'_>,
    project: &str,
    robot_json: &str,
    send: Option<Py<PyAny>>,
    hold: Option<Py<PyAny>>,
    resume: Option<Py<PyAny>>,
    home: Option<Py<PyAny>>,
    estop: Option<Py<PyAny>>,
    estop_hardware: bool,
    estop_latency_bound_ns: Option<i64>,
    recording_dir: Option<&str>,
    handoff_kind: &str,
    handoff_ns: i64,
    lease_enforcement: &str,
    testing_loopback: bool,
    pre_reset_kind: &str,
    pre_reset_hook: Option<Py<PyAny>>,
    pre_reset_prompt: Option<&str>,
    pre_reset_timeout_ns: i64,
    post_reset_kind: &str,
    post_reset_hook: Option<Py<PyAny>>,
    post_reset_prompt: Option<&str>,
    post_reset_timeout_ns: i64,
    reset_verification: &str,
    transport_url: Option<&str>,
    transport_token: Option<&str>,
    media_url: Option<&str>,
    media_token: Option<&str>,
) -> PyResult<PySession> {
    let robot = parse_robot_json(robot_json)?;
    let handoff = parse_handoff(handoff_kind, handoff_ns)?;
    let enforcement = parse_enforcement(lease_enforcement)?;
    let pre_reset = parse_reset_spec(
        "pre_reset",
        pre_reset_kind,
        pre_reset_hook,
        pre_reset_prompt,
        pre_reset_timeout_ns,
    )?;
    let post_reset = parse_reset_spec(
        "post_reset",
        post_reset_kind,
        post_reset_hook,
        post_reset_prompt,
        post_reset_timeout_ns,
    )?;
    let verification = parse_verification_mode(reset_verification)?;

    // Connected transports. The "this build has no such transport" refusal
    // comes first: it is the actionable one, and it must never be reached
    // by accident — a supervision session that silently ran offline because
    // a URL was ignored is the failure mode this whole layer exists to
    // prevent. (Python raises its own friendlier version before ever
    // getting here; this is defence in depth for `_core` callers.)
    #[cfg(not(feature = "grpc"))]
    if transport_url.is_some() {
        return Err(PyRuntimeError::new_err(
            "transport_url requires a waddle-sdk built with the `grpc` feature; this build has \
             none (see waddle._native.FEATURES)",
        ));
    }
    #[cfg(not(feature = "livekit"))]
    if media_url.is_some() {
        return Err(PyRuntimeError::new_err(
            "media_url requires a waddle-sdk built with the `livekit` feature; this build has \
             none — install the teleop extra: pip install 'waddle-sdk[teleop]' (see \
             waddle._native.FEATURES)",
        ));
    }
    if transport_token.is_some() && transport_url.is_none() {
        return Err(PyValueError::new_err(
            "transport_token was given without transport_url",
        ));
    }
    if media_token.is_some() && media_url.is_none() {
        return Err(PyValueError::new_err(
            "media_token was given without media_url",
        ));
    }
    if media_url.is_some() && media_token.is_none() {
        return Err(PyValueError::new_err(
            "media_url requires media_token: the supervision plane mints the room token, this \
             SDK never does",
        ));
    }
    if media_url.is_some() && testing_loopback {
        return Err(PyValueError::new_err(
            "media_url cannot be combined with testing_loopback: a session has exactly one media \
             plane, and the loopback one replaces the real transport",
        ));
    }
    // A LiveKit track publishes at ONE declared resolution and every
    // frame disagreeing with it is dropped, so every declared camera must
    // reach the media plane at the resolution the robot declared. The
    // mapping is core's (`with_robot_cameras`, tested there); this is the
    // marshalling.
    #[cfg(feature = "livekit")]
    let media_config = media_url.map(|url| {
        waddle_media::livekit::LiveKitConfig::new(
            url.to_owned(),
            // Checked present above.
            media_token.unwrap_or_default().to_owned(),
        )
        .with_robot_cameras(&robot)
    });

    // The declared parts layout, computed once here and shared by every
    // path an intervention payload crosses on (the `send` verb's steps, the
    // gate's Substitute/Blend returns). Absent for every declaration without
    // parts, which is what keeps those sessions on the flat ndarray surface.
    let parts = PartsLayout::of(&robot);

    let mut registry = ControlRegistry::default();
    if let Some(cb) = send {
        registry.send = Some(Arc::new(PySend {
            cb,
            parts: parts.clone(),
        }));
    }
    if let Some(cb) = hold {
        registry.hold = Some(Arc::new(PyUnit { cb }));
    }
    if let Some(cb) = resume {
        registry.resume = Some(Arc::new(PyUnit { cb }));
    }
    if let Some(cb) = home {
        registry.home = Some(Arc::new(PyUnit { cb }));
    }
    if let Some(cb) = estop {
        registry.estop = Some((
            Arc::new(PyUnit { cb }),
            EstopDecl {
                hardware: estop_hardware,
                declared_latency_bound_ns: estop_latency_bound_ns,
            },
        ));
    }

    let mut builder = Session::builder(project)
        .robot(robot)
        .control(registry)
        .handoff(handoff)
        .lease_enforcement(enforcement)
        .verification_mode(verification);
    if let Some(dir) = recording_dir {
        builder = builder.recording_dir(dir);
    }
    if let Some(spec) = pre_reset {
        builder = builder.pre_reset(spec);
    }
    if let Some(spec) = post_reset {
        builder = builder.post_reset(spec);
    }
    let mut testing_far = None;
    if testing_loopback {
        let (media, far) = LoopbackMedia::new();
        builder = builder.media(media);
        testing_far = Some(Mutex::new(far));
    }
    // Constructing the transport dials nothing (the core client owns
    // connect/backoff/replay on its own thread), so this stays out of the
    // detached region.
    #[cfg(feature = "grpc")]
    if let Some(url) = transport_url {
        let mut config = waddle_controlplane::grpc::GrpcConfig::new(url);
        if let Some(token) = transport_token {
            config = config.with_token(token);
        }
        builder = builder.transport(waddle_controlplane::grpc::connect(config));
    }

    let session = py
        .detach(move || -> Result<Session, waddle_runtime::RuntimeError> {
            #[cfg_attr(not(feature = "livekit"), allow(unused_mut))]
            let mut builder = builder;
            // `LiveKitMedia::connect` blocks on the signal handshake, so it
            // belongs inside this GIL-released region with the core build,
            // never before it.
            #[cfg(feature = "livekit")]
            if let Some(config) = media_config {
                builder = builder.media(waddle_media::livekit::LiveKitMedia::connect(config)?);
            }
            builder.build()
        })
        .map_err(runtime_err)?;
    Ok(PySession {
        inner: session,
        closed: AtomicBool::new(false),
        testing_far,
        teleop_seq: AtomicU64::new(1),
        chunk_seq: AtomicU64::new(1),
        parts,
    })
}
