//! The session surface + `create_session`.
//!
//! GIL discipline (blanket rule: every core call that can block detaches):
//!
//! | shim call                          | blocks on                       | GIL |
//! |------------------------------------|---------------------------------|-----|
//! | `Session::start_episode`           | reset pipeline + mirror condvar | detach (mandatory) |
//! | `Episode::terminate`               | mirror condvar                  | detach (mandatory) |
//! | `Session::shutdown`                | joins all threads incl. verb dispatch | detach (mandatory — the verb thread may be inside `Python::try_attach` waiting for the GIL we hold: classic deadlock) |
//! | `Episode::gate`, `done`, `outcome` | nothing                         | keep GIL |

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use parking_lot::Mutex;
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use waddle_media::{DataTopic, LoopbackFarEnd, LoopbackMedia};
use waddle_runtime::{ControlRegistry, EstopDecl, Session};
use waddle_types::ActorKind;
use waddle_types::pb::v0 as pb;

use crate::convert::{parse_enforcement, parse_handoff, parse_robot_json, runtime_err};
use crate::episode::PyEpisode;
use crate::verbs::{PySend, PyUnit};

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
}

impl PySession {
    fn testing_far(&self) -> PyResult<&Mutex<LoopbackFarEnd>> {
        self.testing_far.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err(
                "test hooks require waddle.init(_testing=True) (loopback media plane)",
            )
        })
    }
}

#[pymethods]
impl PySession {
    /// Open an episode; blocks through the reset pipeline (GIL released).
    fn start_episode(&self, py: Python<'_>, task: &str) -> PyResult<PyEpisode> {
        let session = self.inner.clone();
        let task = task.to_owned();
        let episode = py
            .detach(move || session.start_episode(&task))
            .map_err(runtime_err)?;
        Ok(PyEpisode::new(episode))
    }

    /// Join all core threads and flush recorders. Idempotent; blocks with
    /// the GIL released (verb dispatch may need it to finish a callback).
    fn shutdown(&self, py: Python<'_>) {
        if !self.closed.swap(true, Ordering::SeqCst) {
            let session = self.inner.clone();
            py.detach(move || session.shutdown());
        }
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
}

/// Build the session. Every argument is plain data; the callables cross as
/// opaque objects invoked only on the core's verb-dispatch thread.
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
) -> PyResult<PySession> {
    let robot = parse_robot_json(robot_json)?;
    let handoff = parse_handoff(handoff_kind, handoff_ns)?;
    let enforcement = parse_enforcement(lease_enforcement)?;

    let mut registry = ControlRegistry::default();
    if let Some(cb) = send {
        registry.send = Some(Arc::new(PySend { cb }));
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
        .lease_enforcement(enforcement);
    if let Some(dir) = recording_dir {
        builder = builder.recording_dir(dir);
    }
    let mut testing_far = None;
    if testing_loopback {
        let (media, far) = LoopbackMedia::new();
        builder = builder.media(media);
        testing_far = Some(Mutex::new(far));
    }

    let session = py.detach(move || builder.build()).map_err(runtime_err)?;
    Ok(PySession {
        inner: session,
        closed: AtomicBool::new(false),
        testing_far,
        teleop_seq: AtomicU64::new(1),
    })
}
