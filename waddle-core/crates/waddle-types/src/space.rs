//! The validated action-space layer: the closed enum of canonical spaces with
//! complete execution semantics (design doc §2.2). Conversions from `pb` are
//! where "must declare, never guess" is enforced.

use crate::error::TypesError;
use crate::grants::Grant;
use crate::ids::{CellId, FrameId, RobotId};
use crate::pb::v0 as pb;

/// A reference to a canonical space type (parallel to the `ActionSpace`
/// oneof).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SpaceKind {
    JointPosition,
    JointVelocity,
    EePoseDelta,
    EePoseAbs,
    BaseTwist,
    Composite,
    Opaque,
}

impl SpaceKind {
    pub fn from_pb(value: i32) -> Result<Self, TypesError> {
        match pb::SpaceKind::try_from(value) {
            Ok(pb::SpaceKind::JointPosition) => Ok(Self::JointPosition),
            Ok(pb::SpaceKind::JointVelocity) => Ok(Self::JointVelocity),
            Ok(pb::SpaceKind::EePoseDelta) => Ok(Self::EePoseDelta),
            Ok(pb::SpaceKind::EePoseAbs) => Ok(Self::EePoseAbs),
            Ok(pb::SpaceKind::BaseTwist) => Ok(Self::BaseTwist),
            Ok(pb::SpaceKind::Composite) => Ok(Self::Composite),
            Ok(pb::SpaceKind::Opaque) => Ok(Self::Opaque),
            Ok(pb::SpaceKind::Unspecified) | Err(_) => Err(TypesError::InvalidEnum {
                field: "SpaceKind",
                value,
            }),
        }
    }

    #[must_use]
    pub fn to_pb(self) -> pb::SpaceKind {
        match self {
            Self::JointPosition => pb::SpaceKind::JointPosition,
            Self::JointVelocity => pb::SpaceKind::JointVelocity,
            Self::EePoseDelta => pb::SpaceKind::EePoseDelta,
            Self::EePoseAbs => pb::SpaceKind::EePoseAbs,
            Self::BaseTwist => pb::SpaceKind::BaseTwist,
            Self::Composite => pb::SpaceKind::Composite,
            Self::Opaque => pb::SpaceKind::Opaque,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JointDescriptor {
    pub name: String,
    pub min_position: Option<f64>,
    pub max_position: Option<f64>,
    pub max_velocity: Option<f64>,
    pub max_effort: Option<f64>,
}

impl JointDescriptor {
    fn from_pb(j: &pb::JointDescriptor) -> Self {
        Self {
            name: j.name.clone(),
            min_position: j.min_position,
            max_position: j.max_position,
            max_velocity: j.max_velocity,
            max_effort: j.max_effort,
        }
    }
}

/// How the three angular numbers of a Twist are interpreted. No default:
/// the platform must be told, never guess.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum RotationEncoding {
    AxisAngle,
    RotVec,
    EulerRpy,
    EulerXyz,
    QuatXyzw,
    QuatWxyz,
}

impl RotationEncoding {
    fn from_pb(value: i32) -> Result<Self, TypesError> {
        match pb::RotationEncoding::try_from(value) {
            Ok(pb::RotationEncoding::AxisAngle) => Ok(Self::AxisAngle),
            Ok(pb::RotationEncoding::Rotvec) => Ok(Self::RotVec),
            Ok(pb::RotationEncoding::EulerRpy) => Ok(Self::EulerRpy),
            Ok(pb::RotationEncoding::EulerXyz) => Ok(Self::EulerXyz),
            Ok(pb::RotationEncoding::QuatXyzw) => Ok(Self::QuatXyzw),
            Ok(pb::RotationEncoding::QuatWxyz) => Ok(Self::QuatWxyz),
            Ok(pb::RotationEncoding::Unspecified) => Err(TypesError::MustDeclare {
                field: "rotation_encoding",
            }),
            Err(_) => Err(TypesError::InvalidEnum {
                field: "rotation_encoding",
                value,
            }),
        }
    }
}

/// How a delta composes with the current pose. No default.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum DeltaFrame {
    /// Pre-multiply: `new = delta * current`.
    Base,
    /// Post-multiply: `new = current * delta`.
    Body,
}

impl DeltaFrame {
    fn from_pb(value: i32) -> Result<Self, TypesError> {
        match pb::DeltaFrame::try_from(value) {
            Ok(pb::DeltaFrame::Base) => Ok(Self::Base),
            Ok(pb::DeltaFrame::Body) => Ok(Self::Body),
            Ok(pb::DeltaFrame::Unspecified) => Err(TypesError::MustDeclare {
                field: "delta_frame",
            }),
            Err(_) => Err(TypesError::InvalidEnum {
                field: "delta_frame",
                value,
            }),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ReplanPolicy {
    Immediate,
    ChunkBoundary,
    Blend,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Interp {
    Hold,
    Linear,
    Cubic,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Chunking {
    pub horizon_steps: u32,
    pub replan: ReplanPolicy,
    pub interp: Interp,
}

impl Default for Chunking {
    /// Single-step, replace-immediately, zero-order hold: the semantics of a
    /// policy that emits one action at a time.
    fn default() -> Self {
        Self {
            horizon_steps: 1,
            replan: ReplanPolicy::Immediate,
            interp: Interp::Hold,
        }
    }
}

impl Chunking {
    fn from_pb(c: &pb::ChunkingSemantics) -> Result<Self, TypesError> {
        let replan = match pb::ReplanPolicy::try_from(c.replan) {
            Ok(pb::ReplanPolicy::Immediate) | Ok(pb::ReplanPolicy::Unspecified) => {
                ReplanPolicy::Immediate
            }
            Ok(pb::ReplanPolicy::ChunkBoundary) => ReplanPolicy::ChunkBoundary,
            Ok(pb::ReplanPolicy::Blend) => ReplanPolicy::Blend,
            Err(_) => {
                return Err(TypesError::InvalidEnum {
                    field: "replan",
                    value: c.replan,
                });
            }
        };
        let interp = match pb::Interpolation::try_from(c.interpolation) {
            Ok(pb::Interpolation::Hold) | Ok(pb::Interpolation::Unspecified) => Interp::Hold,
            Ok(pb::Interpolation::Linear) => Interp::Linear,
            Ok(pb::Interpolation::Cubic) => Interp::Cubic,
            Err(_) => {
                return Err(TypesError::InvalidEnum {
                    field: "interpolation",
                    value: c.interpolation,
                });
            }
        };
        Ok(Self {
            horizon_steps: c.horizon_steps.max(1),
            replan,
            interp,
        })
    }
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GripperKind {
    Parallel {
        open_value: f64,
        closed_value: f64,
        /// Index of the gripper channel within the part's action vector;
        /// -1 means the last element.
        action_dim: i32,
    },
    Suction,
    Dexterous {
        joints: Vec<JointDescriptor>,
    },
}

impl GripperKind {
    fn from_pb(g: &pb::GripperSpec) -> Result<Self, TypesError> {
        match g.kind.as_ref() {
            Some(pb::gripper_spec::Kind::Parallel(p)) => Ok(Self::Parallel {
                open_value: p.open_value,
                closed_value: p.closed_value,
                action_dim: p.action_dim,
            }),
            Some(pb::gripper_spec::Kind::Suction(_)) => Ok(Self::Suction),
            Some(pb::gripper_spec::Kind::Dexterous(d)) => Ok(Self::Dexterous {
                joints: d.joints.iter().map(JointDescriptor::from_pb).collect(),
            }),
            None => Err(TypesError::MissingField("GripperSpec.kind")),
        }
    }

    /// Map a teleop gripper command — normalized `0..1` where `1` is fully
    /// open (the media-plane convention) — through this declared spec's own
    /// actuator convention.
    ///
    /// - `Parallel`: linearly onto `[closed_value, open_value]`.
    /// - `Suction`: the proto declares no continuous open/closed values (a
    ///   bare on/off channel), so this thresholds at 0.5 into `{0.0, 1.0}`.
    /// - `Dexterous`: no single-scalar convention is declared for a
    ///   multi-joint hand here; passes the command through unchanged.
    #[must_use]
    pub fn map_normalized(&self, g: f64) -> f64 {
        match self {
            Self::Parallel {
                open_value,
                closed_value,
                ..
            } => closed_value + g * (open_value - closed_value),
            Self::Suction => {
                if g >= 0.5 {
                    1.0
                } else {
                    0.0
                }
            }
            Self::Dexterous { .. } => g,
        }
    }
}

/// The closed set of canonical space shapes.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SpaceSpec {
    JointPosition {
        joints: Vec<JointDescriptor>,
    },
    JointVelocity {
        joints: Vec<JointDescriptor>,
    },
    EePoseDelta {
        frame: FrameId,
        rotation: RotationEncoding,
        delta_frame: DeltaFrame,
        max_linear_step_m: Option<f64>,
        max_angular_step_rad: Option<f64>,
    },
    EePoseAbs {
        frame: FrameId,
        rotation: RotationEncoding,
    },
    BaseTwist {
        frame: FrameId,
        max_linear_mps: Option<f64>,
        max_angular_radps: Option<f64>,
    },
    /// Ordered named parts; order defines the concatenated wire-vector
    /// layout. Depth pinned to 1 in v0.
    Composite {
        parts: Vec<(String, ActionSpace)>,
    },
    /// Monitor-only escape hatch.
    Opaque {
        format_hint: String,
        dim: Option<u32>,
    },
}

/// A canonical action space with complete execution semantics.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActionSpace {
    pub spec: SpaceSpec,
    pub rate_hz: f64,
    pub chunking: Chunking,
    pub gripper: Option<GripperKind>,
}

impl ActionSpace {
    /// Width of the flattened action vector, when the space is executable.
    /// `None` for opaque spaces without a declared dim.
    #[must_use]
    pub fn dims(&self) -> Option<usize> {
        match &self.spec {
            SpaceSpec::JointPosition { joints } | SpaceSpec::JointVelocity { joints } => {
                Some(joints.len())
            }
            SpaceSpec::EePoseDelta { .. } | SpaceSpec::BaseTwist { .. } => Some(6),
            // 3 position + 4 quaternion (wxyz).
            SpaceSpec::EePoseAbs { .. } => Some(7),
            SpaceSpec::Composite { parts } => {
                let mut total = 0usize;
                for (_, space) in parts {
                    total += space.dims()?;
                }
                Some(total)
            }
            SpaceSpec::Opaque { dim, .. } => dim.map(|d| d as usize),
        }
    }

    #[must_use]
    pub fn kind(&self) -> SpaceKind {
        match &self.spec {
            SpaceSpec::JointPosition { .. } => SpaceKind::JointPosition,
            SpaceSpec::JointVelocity { .. } => SpaceKind::JointVelocity,
            SpaceSpec::EePoseDelta { .. } => SpaceKind::EePoseDelta,
            SpaceSpec::EePoseAbs { .. } => SpaceKind::EePoseAbs,
            SpaceSpec::BaseTwist { .. } => SpaceKind::BaseTwist,
            SpaceSpec::Composite { .. } => SpaceKind::Composite,
            SpaceSpec::Opaque { .. } => SpaceKind::Opaque,
        }
    }

    /// Delta spaces compose against the chunk-start snapshot and therefore
    /// refuse mid-chunk splice entry (see FSM.md §5).
    #[must_use]
    pub fn contains_delta(&self) -> bool {
        match &self.spec {
            SpaceSpec::EePoseDelta { .. } => true,
            SpaceSpec::Composite { parts } => parts.iter().any(|(_, s)| s.contains_delta()),
            _ => false,
        }
    }

    pub fn from_pb(space: &pb::ActionSpace) -> Result<Self, TypesError> {
        Self::from_pb_at_depth(space, 0)
    }

    fn from_pb_at_depth(space: &pb::ActionSpace, depth: u8) -> Result<Self, TypesError> {
        let spec = match space.space.as_ref() {
            None => return Err(TypesError::MissingField("ActionSpace.space")),
            Some(pb::action_space::Space::JointPosition(j)) => SpaceSpec::JointPosition {
                joints: j.joints.iter().map(JointDescriptor::from_pb).collect(),
            },
            Some(pb::action_space::Space::JointVelocity(j)) => SpaceSpec::JointVelocity {
                joints: j.joints.iter().map(JointDescriptor::from_pb).collect(),
            },
            Some(pb::action_space::Space::EeDelta(d)) => SpaceSpec::EePoseDelta {
                frame: FrameId::new(&d.frame_id)?,
                rotation: RotationEncoding::from_pb(d.rotation_encoding)?,
                delta_frame: DeltaFrame::from_pb(d.delta_frame)?,
                max_linear_step_m: d.max_linear_step_m,
                max_angular_step_rad: d.max_angular_step_rad,
            },
            Some(pb::action_space::Space::EeAbsolute(a)) => SpaceSpec::EePoseAbs {
                frame: FrameId::new(&a.frame_id)?,
                rotation: RotationEncoding::from_pb(a.rotation_encoding)?,
            },
            Some(pb::action_space::Space::BaseTwist(b)) => SpaceSpec::BaseTwist {
                frame: FrameId::new(&b.frame_id)?,
                max_linear_mps: b.max_linear_mps,
                max_angular_radps: b.max_angular_radps,
            },
            Some(pb::action_space::Space::Composite(c)) => {
                if depth > 0 {
                    return Err(TypesError::CompositeDepth {
                        part: "<nested>".to_owned(),
                    });
                }
                let mut parts = Vec::with_capacity(c.parts.len());
                for part in &c.parts {
                    if part.name.is_empty() {
                        return Err(TypesError::MissingField("Composite.Part.name"));
                    }
                    if parts.iter().any(|(n, _)| n == &part.name) {
                        return Err(TypesError::DuplicatePart(part.name.clone()));
                    }
                    let inner = part
                        .space
                        .as_ref()
                        .ok_or(TypesError::MissingField("Composite.Part.space"))?;
                    if matches!(
                        inner.space.as_ref(),
                        Some(pb::action_space::Space::Composite(_))
                    ) {
                        return Err(TypesError::CompositeDepth {
                            part: part.name.clone(),
                        });
                    }
                    parts.push((part.name.clone(), Self::from_pb_at_depth(inner, depth + 1)?));
                }
                if parts.is_empty() {
                    return Err(TypesError::MissingField("Composite.parts"));
                }
                SpaceSpec::Composite { parts }
            }
            Some(pb::action_space::Space::Opaque(o)) => SpaceSpec::Opaque {
                format_hint: o.format_hint.clone(),
                dim: o.dim,
            },
        };

        if depth == 0 && space.rate_hz <= 0.0 {
            return Err(TypesError::InvalidValue {
                field: "rate_hz",
                reason: "must be > 0 at the top level",
            });
        }

        Ok(Self {
            spec,
            rate_hz: space.rate_hz,
            chunking: space
                .chunking
                .as_ref()
                .map(Chunking::from_pb)
                .transpose()?
                .unwrap_or_default(),
            gripper: space
                .gripper
                .as_ref()
                .map(GripperKind::from_pb)
                .transpose()?,
        })
    }
}

/// The validated robot declaration. Cameras, series, URDF, and frame graph
/// stay on the `pb::RobotDescription` (they are configuration for capture and
/// the closed side, not core execution semantics).
#[derive(Debug, Clone, PartialEq)]
pub struct RobotDescription {
    pub name: String,
    pub robot_id: RobotId,
    pub cell_id: CellId,
    pub action_space: ActionSpace,
    pub grants: Vec<Grant>,
}

impl TryFrom<&pb::RobotDescription> for RobotDescription {
    type Error = TypesError;

    fn try_from(r: &pb::RobotDescription) -> Result<Self, Self::Error> {
        let space = r
            .action_space
            .as_ref()
            .ok_or(TypesError::MissingField("RobotDescription.action_space"))?;
        Ok(Self {
            name: r.name.clone(),
            robot_id: RobotId::new(&r.robot_id),
            cell_id: CellId::new(&r.cell_id),
            action_space: ActionSpace::from_pb(space)?,
            grants: r
                .grants
                .iter()
                .map(Grant::try_from)
                .collect::<Result<_, _>>()?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ee_delta_must_declare_rotation_and_delta_frame() {
        let space = pb::ActionSpace {
            space: Some(pb::action_space::Space::EeDelta(pb::EePoseDelta {
                frame_id: "base".into(),
                rotation_encoding: 0, // UNSPECIFIED
                delta_frame: pb::DeltaFrame::Base as i32,
                max_linear_step_m: None,
                max_angular_step_rad: None,
            })),
            rate_hz: 30.0,
            chunking: None,
            gripper: None,
        };
        assert!(matches!(
            ActionSpace::from_pb(&space),
            Err(TypesError::MustDeclare {
                field: "rotation_encoding"
            })
        ));
    }

    #[test]
    fn composite_depth_is_pinned_to_one() {
        let inner_composite = pb::ActionSpace {
            space: Some(pb::action_space::Space::Composite(pb::Composite {
                parts: vec![],
            })),
            rate_hz: 0.0,
            chunking: None,
            gripper: None,
        };
        let space = pb::ActionSpace {
            space: Some(pb::action_space::Space::Composite(pb::Composite {
                parts: vec![pb::composite::Part {
                    name: "nested".into(),
                    space: Some(inner_composite),
                }],
            })),
            rate_hz: 50.0,
            chunking: None,
            gripper: None,
        };
        assert!(matches!(
            ActionSpace::from_pb(&space),
            Err(TypesError::CompositeDepth { part }) if part == "nested"
        ));
    }

    #[test]
    fn parallel_gripper_maps_normalized_open_to_declared_open_value() {
        let spec = GripperKind::Parallel {
            open_value: 0.04,
            closed_value: 0.0,
            action_dim: -1,
        };
        // 1.0 is fully open in the media-plane convention.
        assert!((spec.map_normalized(1.0) - 0.04).abs() < 1e-12);
        // 0.0 is fully closed.
        assert!((spec.map_normalized(0.0) - 0.0).abs() < 1e-12);
        // Linear in between.
        assert!((spec.map_normalized(0.5) - 0.02).abs() < 1e-12);
    }

    #[test]
    fn suction_gripper_thresholds_at_one_half() {
        let spec = GripperKind::Suction;
        assert_eq!(spec.map_normalized(1.0), 1.0);
        assert_eq!(spec.map_normalized(0.6), 1.0);
        assert_eq!(spec.map_normalized(0.5), 1.0);
        assert_eq!(spec.map_normalized(0.49), 0.0);
        assert_eq!(spec.map_normalized(0.0), 0.0);
    }

    #[test]
    fn empty_frame_is_rejected() {
        let space = pb::ActionSpace {
            space: Some(pb::action_space::Space::EeAbsolute(pb::EePoseAbs {
                frame_id: String::new(),
                rotation_encoding: pb::RotationEncoding::QuatWxyz as i32,
            })),
            rate_hz: 30.0,
            chunking: None,
            gripper: None,
        };
        assert!(matches!(
            ActionSpace::from_pb(&space),
            Err(TypesError::EmptyFrame)
        ));
    }
}
