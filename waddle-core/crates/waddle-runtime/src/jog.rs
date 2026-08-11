//! Core-owned local site-operator jog mechanics.
//!
//! The browser supplies only intent (axis, direction, configured increment)
//! and deadman heartbeats. This module resolves the declared action space,
//! derives absolute joint targets from the freshest local proprio sample,
//! and owns the one-second claim deadline. It never clamps: the owner's
//! `Control.send` envelope remains the final whole-command refusal.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use parking_lot::{Condvar, Mutex, RwLock};
use waddle_ingest::SessionClock;
use waddle_types::pb::v0 as pb;
use waddle_types::time::Clock;
use waddle_types::{ActionSpace, MonoNs, SpaceSpec, unflatten_action};

pub(crate) const LOCAL_JOG_SOURCE: &str = "local-ui-jog";
pub(crate) const DEADMAN_TIMEOUT_NS: i64 = 1_000_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JogAxis {
    Joint(usize),
    Linear(usize),
    Angular(usize),
}

#[derive(Clone, Debug, PartialEq)]
pub struct JogRequest {
    /// Named composite part, or `None` for a non-composite declaration.
    pub part: Option<String>,
    pub axis: JogAxis,
    /// Exactly -1 or +1.
    pub direction: i8,
    /// Positive, finite increment in the units implied by `axis`.
    pub step: f64,
}

#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum JogRefusal {
    #[error("no episode is running")]
    NoRunningEpisode,
    #[error("agent-invited episodes admit the invited host only")]
    AgentInvitedEpisode,
    #[error("another intervention claim is active")]
    ConflictingClaim,
    #[error("the local jog claim was refused or did not engage")]
    EngageRefused,
    #[error("a composite action space requires a declared part")]
    PartRequired,
    #[error("unknown action-space part {0:?}")]
    UnknownPart(String),
    #[error("jog is unsupported for declared action space {0}")]
    UnsupportedSpace(&'static str),
    #[error("no local joint-position observation is available for part {0:?}")]
    MissingProprio(String),
    #[error("latest joint-position observation has {got} values; declaration requires {want}")]
    ProprioDimension { got: usize, want: usize },
    #[error("latest joint-position observation contains a non-finite value")]
    InvalidProprio,
    #[error("invalid jog request: {0}")]
    InvalidRequest(&'static str),
    #[error("the local jog deadman is not active")]
    DeadmanInactive,
    #[error("the session is shutting down")]
    ShuttingDown,
}

impl JogRefusal {
    /// Stable machine-readable refusal code for thin bindings and local UI.
    #[must_use]
    pub fn code(&self) -> &'static str {
        match self {
            Self::NoRunningEpisode => "no_running_episode",
            Self::AgentInvitedEpisode => "agent_invited_episode",
            Self::ConflictingClaim => "conflicting_claim",
            Self::EngageRefused => "engage_refused",
            Self::PartRequired => "part_required",
            Self::UnknownPart(_) => "unknown_part",
            Self::UnsupportedSpace(_) => "unsupported_space",
            Self::MissingProprio(_) => "missing_proprio",
            Self::ProprioDimension { .. } => "proprio_dimension",
            Self::InvalidProprio => "invalid_proprio",
            Self::InvalidRequest(_) => "invalid_request",
            Self::DeadmanInactive => "deadman_inactive",
            Self::ShuttingDown => "shutting_down",
        }
    }
}

/// Timestamped joint positions used only to derive a local absolute jog
/// target. Timestamp comparison keeps an older reducer-drained gate record
/// from overwriting a newer explicit `report_proprio` call.
#[derive(Debug, Default)]
pub(crate) struct LatestJoints {
    values: RwLock<BTreeMap<String, (MonoNs, Vec<f64>)>>,
}

impl LatestJoints {
    pub(crate) fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    pub(crate) fn publish(&self, part: &str, at: MonoNs, values: &[f64]) {
        let mut latest = self.values.write();
        let replace = latest.get(part).is_none_or(|(seen, _)| at >= *seen);
        if replace {
            latest.insert(part.to_owned(), (at, values.to_vec()));
        }
    }

    fn read(&self, part: &str) -> Option<Vec<f64>> {
        self.values
            .read()
            .get(part)
            .map(|(_, values)| values.clone())
    }
}

fn addressed_space<'a>(
    root: &'a ActionSpace,
    part: Option<&str>,
) -> Result<(&'a ActionSpace, Option<&'a str>), JogRefusal> {
    match (&root.spec, part.filter(|part| !part.is_empty())) {
        (SpaceSpec::Composite { parts }, Some(name)) => parts
            .iter()
            .find(|(declared, _)| declared == name)
            .map(|(declared, space)| (space, Some(declared.as_str())))
            .ok_or_else(|| JogRefusal::UnknownPart(name.to_owned())),
        (SpaceSpec::Composite { .. }, None) => Err(JogRefusal::PartRequired),
        (_, Some(name)) => Err(JogRefusal::UnknownPart(name.to_owned())),
        (_, None) => Ok((root, None)),
    }
}

pub(crate) fn build_action(
    request: &JogRequest,
    root: &ActionSpace,
    latest: &LatestJoints,
) -> Result<pb::Action, JogRefusal> {
    if request.direction != -1 && request.direction != 1 {
        return Err(JogRefusal::InvalidRequest("direction must be -1 or +1"));
    }
    if !request.step.is_finite() || request.step <= 0.0 {
        return Err(JogRefusal::InvalidRequest(
            "step must be finite and positive",
        ));
    }
    let (space, part) = addressed_space(root, request.part.as_deref())?;
    let signed = f64::from(request.direction) * request.step;
    let values = match (&request.axis, &space.spec) {
        (JogAxis::Joint(index), SpaceSpec::JointPosition { joints }) => {
            if *index >= joints.len() {
                return Err(JogRefusal::InvalidRequest("joint index is out of range"));
            }
            let key = part.unwrap_or_default();
            let mut values = latest
                .read(key)
                .ok_or_else(|| JogRefusal::MissingProprio(key.to_owned()))?;
            if values.len() != joints.len() {
                return Err(JogRefusal::ProprioDimension {
                    got: values.len(),
                    want: joints.len(),
                });
            }
            if values.iter().any(|value| !value.is_finite()) {
                return Err(JogRefusal::InvalidProprio);
            }
            values[*index] += signed;
            values
        }
        (JogAxis::Linear(axis), SpaceSpec::EePoseDelta { .. }) if *axis < 3 => {
            let mut values = vec![0.0; 6];
            values[*axis] = signed;
            values
        }
        (JogAxis::Angular(axis), SpaceSpec::EePoseDelta { .. }) if *axis < 3 => {
            let mut values = vec![0.0; 6];
            values[3 + *axis] = signed;
            values
        }
        (JogAxis::Linear(_) | JogAxis::Angular(_), SpaceSpec::EePoseDelta { .. }) => {
            return Err(JogRefusal::InvalidRequest("Cartesian axis is out of range"));
        }
        (_, SpaceSpec::JointPosition { .. }) => {
            return Err(JogRefusal::UnsupportedSpace(
                "joint_position for this jog axis",
            ));
        }
        (_, SpaceSpec::JointVelocity { .. }) => {
            return Err(JogRefusal::UnsupportedSpace("joint_velocity"));
        }
        (_, SpaceSpec::EePoseDelta { .. }) => {
            return Err(JogRefusal::UnsupportedSpace(
                "ee_pose_delta for this jog axis",
            ));
        }
        (_, SpaceSpec::EePoseAbs { .. }) => {
            return Err(JogRefusal::UnsupportedSpace("ee_pose_absolute"));
        }
        (_, SpaceSpec::BaseTwist { .. }) => {
            return Err(JogRefusal::UnsupportedSpace("base_twist"));
        }
        (_, SpaceSpec::Composite { .. }) => unreachable!("addressed_space returns a leaf"),
        (_, SpaceSpec::Opaque { .. }) => return Err(JogRefusal::UnsupportedSpace("opaque")),
    };
    unflatten_action(&values, None, part, root).map_err(|_| JogRefusal::EngageRefused)
}

#[derive(Debug, Default)]
struct DeadmanState {
    claim_id: Option<String>,
    deadline: MonoNs,
    shutdown: bool,
}

impl DeadmanState {
    fn arm(&mut self, claim_id: &str, now: MonoNs) {
        self.claim_id = Some(claim_id.to_owned());
        self.deadline = now.saturating_add(DEADMAN_TIMEOUT_NS);
    }

    fn heartbeat(&mut self, now: MonoNs) -> Result<(), JogRefusal> {
        if self.claim_id.is_none() {
            return Err(JogRefusal::DeadmanInactive);
        }
        self.deadline = now.saturating_add(DEADMAN_TIMEOUT_NS);
        Ok(())
    }

    fn expire(&mut self, now: MonoNs) -> Option<String> {
        if now < self.deadline {
            return None;
        }
        self.claim_id.take()
    }

    fn disarm(&mut self) -> Option<String> {
        self.claim_id.take()
    }
}

pub(crate) struct JogDeadman {
    state: Arc<Mutex<DeadmanState>>,
    changed: Arc<Condvar>,
}

impl std::fmt::Debug for JogDeadman {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("JogDeadman").finish_non_exhaustive()
    }
}

impl JogDeadman {
    pub(crate) fn spawn(
        clock: SessionClock,
        release: Arc<dyn Fn(String) + Send + Sync>,
    ) -> (Arc<Self>, JoinHandle<()>) {
        let state = Arc::new(Mutex::new(DeadmanState::default()));
        let changed = Arc::new(Condvar::new());
        let deadman = Arc::new(Self {
            state: state.clone(),
            changed: changed.clone(),
        });
        let thread = std::thread::Builder::new()
            .name("waddle-local-jog-deadman".into())
            .spawn(move || {
                loop {
                    let expired = {
                        let mut state = state.lock();
                        if state.shutdown {
                            state.disarm()
                        } else {
                            let expired = state.expire(clock.stamp_now().mono_ns());
                            if expired.is_none() {
                                changed.wait_for(&mut state, Duration::from_millis(50));
                            }
                            expired
                        }
                    };
                    if let Some(claim_id) = expired {
                        release(claim_id);
                    }
                    if state.lock().shutdown {
                        return;
                    }
                }
            })
            .expect("spawn local jog deadman");
        (deadman, thread)
    }

    pub(crate) fn arm(&self, claim_id: &str, now: MonoNs) {
        self.state.lock().arm(claim_id, now);
        self.changed.notify_all();
    }

    pub(crate) fn heartbeat(&self, now: MonoNs) -> Result<(), JogRefusal> {
        let result = self.state.lock().heartbeat(now);
        self.changed.notify_all();
        result
    }

    pub(crate) fn disarm(&self) -> Option<String> {
        let claim = self.state.lock().disarm();
        self.changed.notify_all();
        claim
    }

    pub(crate) fn close(&self) {
        self.state.lock().shutdown = true;
        self.changed.notify_all();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn joints(names: &[&str]) -> Vec<waddle_types::JointDescriptor> {
        names
            .iter()
            .map(|name| waddle_types::JointDescriptor {
                name: (*name).into(),
                min_position: None,
                max_position: None,
                max_velocity: None,
                max_effort: None,
            })
            .collect()
    }

    fn joint_position(names: &[&str]) -> ActionSpace {
        ActionSpace {
            spec: SpaceSpec::JointPosition {
                joints: joints(names),
            },
            rate_hz: 50.0,
            chunking: Default::default(),
            gripper: None,
        }
    }

    #[test]
    fn deadman_heartbeat_extends_and_timeout_releases_without_wall_clock() {
        let mut state = DeadmanState::default();
        state.arm("claim", MonoNs(10));
        assert_eq!(state.expire(MonoNs(DEADMAN_TIMEOUT_NS + 9)), None);
        state.heartbeat(MonoNs(500)).unwrap();
        assert_eq!(state.expire(MonoNs(DEADMAN_TIMEOUT_NS + 499)), None);
        assert_eq!(
            state.expire(MonoNs(DEADMAN_TIMEOUT_NS + 500)),
            Some("claim".into())
        );
        assert_eq!(state.expire(MonoNs(i64::MAX)), None);
    }

    #[test]
    fn joint_jog_derives_one_absolute_target_and_never_clamps() {
        let latest = LatestJoints::new();
        latest.publish("", MonoNs(1), &[0.95, -0.2]);
        let space = joint_position(&["j0", "j1"]);
        let action = build_action(
            &JogRequest {
                part: None,
                axis: JogAxis::Joint(0),
                direction: 1,
                step: 0.1,
            },
            &space,
            &latest,
        )
        .unwrap();
        let Some(pb::action::Target::JointPosition(target)) = action.target else {
            panic!("joint-position target")
        };
        assert_eq!(target.values, vec![1.05, -0.2]);
    }

    #[test]
    fn missing_proprio_and_unsupported_spaces_are_typed_refusals() {
        let latest = LatestJoints::new();
        let request = JogRequest {
            part: None,
            axis: JogAxis::Joint(0),
            direction: 1,
            step: 0.01,
        };
        assert_eq!(
            build_action(&request, &joint_position(&["j0", "j1"]), &latest),
            Err(JogRefusal::MissingProprio(String::new()))
        );

        let velocity = ActionSpace {
            spec: SpaceSpec::JointVelocity {
                joints: joints(&["j0", "j1"]),
            },
            rate_hz: 50.0,
            chunking: Default::default(),
            gripper: None,
        };
        assert_eq!(
            build_action(&request, &velocity, &latest),
            Err(JogRefusal::UnsupportedSpace("joint_velocity"))
        );
    }

    #[test]
    fn composite_jog_requires_and_routes_to_exactly_one_declared_part() {
        let latest = LatestJoints::new();
        latest.publish("right", MonoNs(3), &[0.2, -0.4]);
        let composite = ActionSpace {
            spec: SpaceSpec::Composite {
                parts: vec![
                    ("left".into(), joint_position(&["l0", "l1"])),
                    ("right".into(), joint_position(&["r0", "r1"])),
                ],
            },
            rate_hz: 50.0,
            chunking: Default::default(),
            gripper: None,
        };
        let without_part = JogRequest {
            part: None,
            axis: JogAxis::Joint(1),
            direction: 1,
            step: 0.05,
        };
        assert_eq!(
            build_action(&without_part, &composite, &latest),
            Err(JogRefusal::PartRequired)
        );

        let action = build_action(
            &JogRequest {
                part: Some("right".into()),
                ..without_part
            },
            &composite,
            &latest,
        )
        .unwrap();
        assert_eq!(action.part, "right");
        let Some(pb::action::Target::JointPosition(target)) = action.target else {
            panic!("part-scoped joint-position target")
        };
        assert_eq!(target.values[0], 0.2);
        assert!((target.values[1] + 0.35).abs() < f64::EPSILON);
    }
}
