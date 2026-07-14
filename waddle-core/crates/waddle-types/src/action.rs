//! Executable actions: wire chunks flattened into numeric vectors per the
//! declared action-space layout. This is the single place where wire shapes
//! (oneofs, wxyz quaternions, composite part order) become flat `f64` rows.

use smallvec::SmallVec;

use crate::error::TypesError;
use crate::ids::SourceId;
use crate::pb::v0 as pb;
use crate::provenance::ProvenanceTag;
use crate::space::{ActionSpace, SpaceSpec};

/// Inline capacity 16 covers a 14-dof bimanual plus grippers without heap
/// allocation on the gate fast path.
pub type ActionValues = SmallVec<[f64; 16]>;

/// Inline capacity 32 covers bimanual proprio (14 pos + 14 vel + 2 grippers
/// = 30) without heap allocation on the gate fast path. Wider observations
/// spill to the heap: correct but no longer allocation-free — a documented
/// degradation, never truncation.
pub type ObsValues = SmallVec<[f64; 32]>;

/// One flattened action step.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Step {
    /// Playback offset within the chunk; 0 = now.
    pub offset_ns: i64,
    /// Flattened values, laid out per the declared space (composite parts in
    /// declaration order; poses as `[x, y, z, qw, qx, qy, qz]`).
    pub values: ActionValues,
    /// Gripper command in declared units, when present.
    pub gripper: Option<f64>,
}

/// A validated, flattened action chunk.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActionChunk {
    pub steps: Vec<Step>,
    pub dims: usize,
    pub horizon_ns: i64,
    pub t_emitted_ns: i64,
    /// Timestamp of the observation this chunk was computed from
    /// (time-aligned splice entry; staleness accounting).
    pub t_obs_ns: i64,
    /// Monotone per stream.
    pub seq: u64,
    pub source: SourceId,
    pub provenance: ProvenanceTag,
}

impl ActionChunk {
    /// Validate and flatten a wire chunk against the declared space.
    pub fn from_pb(chunk: &pb::ActionChunk, space: &ActionSpace) -> Result<Self, TypesError> {
        let dims = space.dims().ok_or(TypesError::OpaqueNotExecutable)?;
        let provenance = chunk
            .provenance
            .as_ref()
            .map(ProvenanceTag::try_from)
            .transpose()?
            .unwrap_or_else(ProvenanceTag::policy);

        let mut steps = Vec::with_capacity(chunk.actions.len());
        for action in &chunk.actions {
            steps.push(flatten_action(action, space)?);
        }

        Ok(Self {
            steps,
            dims,
            horizon_ns: chunk.horizon_ns,
            t_emitted_ns: chunk.t_emitted_ns,
            t_obs_ns: chunk.t_obs_ns,
            seq: chunk.seq,
            source: SourceId::new(&chunk.source_id),
            provenance,
        })
    }
}

/// Flatten one wire action against the declared space.
pub fn flatten_action(action: &pb::Action, space: &ActionSpace) -> Result<Step, TypesError> {
    let mut values = ActionValues::new();
    flatten_target(action, space, &mut values)?;

    let expected = space.dims().ok_or(TypesError::OpaqueNotExecutable)?;
    if values.len() != expected {
        return Err(TypesError::DimensionMismatch {
            expected,
            got: values.len(),
        });
    }

    Ok(Step {
        offset_ns: action.t_offset_ns,
        values,
        gripper: action.gripper.as_ref().map(|g| g.position),
    })
}

fn flatten_target(
    action: &pb::Action,
    space: &ActionSpace,
    out: &mut ActionValues,
) -> Result<(), TypesError> {
    let target = action
        .target
        .as_ref()
        .ok_or(TypesError::MissingField("Action.target"))?;

    match (&space.spec, target) {
        (SpaceSpec::JointPosition { joints }, pb::action::Target::JointPosition(v))
        | (SpaceSpec::JointVelocity { joints }, pb::action::Target::JointVelocity(v)) => {
            if v.values.len() != joints.len() {
                return Err(TypesError::DimensionMismatch {
                    expected: joints.len(),
                    got: v.values.len(),
                });
            }
            out.extend_from_slice(&v.values);
            Ok(())
        }
        (SpaceSpec::EePoseDelta { .. }, pb::action::Target::EeDelta(t))
        | (SpaceSpec::BaseTwist { .. }, pb::action::Target::BaseTwist(t)) => {
            let lin = t
                .linear
                .as_ref()
                .ok_or(TypesError::MissingField("Twist.linear"))?;
            let ang = t
                .angular
                .as_ref()
                .ok_or(TypesError::MissingField("Twist.angular"))?;
            out.extend_from_slice(&[lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]);
            Ok(())
        }
        (SpaceSpec::EePoseAbs { frame, .. }, pb::action::Target::EeAbsolute(p)) => {
            if p.frame_id != frame.as_str() {
                return Err(TypesError::InvalidValue {
                    field: "Pose.frame_id",
                    reason: "does not match the declared EE frame",
                });
            }
            let pos = p
                .position
                .as_ref()
                .ok_or(TypesError::MissingField("Pose.position"))?;
            let rot = p
                .rotation
                .as_ref()
                .ok_or(TypesError::MissingField("Pose.rotation"))?;
            // wxyz on the wire, wxyz in the flat layout: the ONE place this
            // convention is interpreted.
            out.extend_from_slice(&[pos.x, pos.y, pos.z, rot.w, rot.x, rot.y, rot.z]);
            Ok(())
        }
        (SpaceSpec::Composite { parts }, pb::action::Target::Composite(c)) => {
            for pa in &c.parts {
                if !parts.iter().any(|(name, _)| name == &pa.name) {
                    return Err(TypesError::UnknownPart(pa.name.clone()));
                }
            }
            // Output layout is DECLARATION order, regardless of message order.
            for (name, part_space) in parts {
                let pa = c
                    .parts
                    .iter()
                    .find(|pa| &pa.name == name)
                    .ok_or_else(|| TypesError::MissingPart(name.clone()))?;
                let inner = pa
                    .action
                    .as_ref()
                    .ok_or(TypesError::MissingField("PartAction.action"))?;
                flatten_target(inner, part_space, out)?;
            }
            Ok(())
        }
        (_, pb::action::Target::Opaque(_)) => Err(TypesError::OpaqueNotExecutable),
        (_, pb::action::Target::Noop(_)) => Err(TypesError::NoopNotExecutable),
        _ => Err(TypesError::InvalidValue {
            field: "Action.target",
            reason: "target arm does not match the declared action space",
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::space::ActionSpace;

    fn joint_space(n: usize) -> pb::ActionSpace {
        pb::ActionSpace {
            space: Some(pb::action_space::Space::JointPosition(pb::JointPosition {
                joints: (0..n)
                    .map(|i| pb::JointDescriptor {
                        name: format!("j{i}"),
                        ..Default::default()
                    })
                    .collect(),
            })),
            rate_hz: 50.0,
            chunking: None,
            gripper: None,
        }
    }

    fn bimanual_space() -> ActionSpace {
        let space = pb::ActionSpace {
            space: Some(pb::action_space::Space::Composite(pb::Composite {
                parts: vec![
                    pb::composite::Part {
                        name: "left".into(),
                        space: Some(joint_space(7)),
                    },
                    pb::composite::Part {
                        name: "right".into(),
                        space: Some(joint_space(7)),
                    },
                ],
            })),
            rate_hz: 50.0,
            chunking: None,
            gripper: None,
        };
        ActionSpace::from_pb(&space).unwrap()
    }

    fn part_action(values: Vec<f64>) -> pb::Action {
        pb::Action {
            target: Some(pb::action::Target::JointPosition(pb::JointVector {
                values,
            })),
            ..Default::default()
        }
    }

    #[test]
    fn composite_flattens_in_declaration_order_regardless_of_message_order() {
        let space = bimanual_space();
        let action = pb::Action {
            target: Some(pb::action::Target::Composite(pb::CompositeAction {
                parts: vec![
                    // Message order: right first — output must still be left-first.
                    pb::composite_action::PartAction {
                        name: "right".into(),
                        action: Some(part_action(vec![2.0; 7])),
                    },
                    pb::composite_action::PartAction {
                        name: "left".into(),
                        action: Some(part_action(vec![1.0; 7])),
                    },
                ],
            })),
            ..Default::default()
        };
        let step = flatten_action(&action, &space).unwrap();
        assert_eq!(step.values.len(), 14);
        assert!(step.values[..7].iter().all(|v| *v == 1.0));
        assert!(step.values[7..].iter().all(|v| *v == 2.0));
    }

    #[test]
    fn missing_part_is_rejected() {
        let space = bimanual_space();
        let action = pb::Action {
            target: Some(pb::action::Target::Composite(pb::CompositeAction {
                parts: vec![pb::composite_action::PartAction {
                    name: "left".into(),
                    action: Some(part_action(vec![1.0; 7])),
                }],
            })),
            ..Default::default()
        };
        assert!(matches!(
            flatten_action(&action, &space),
            Err(TypesError::MissingPart(p)) if p == "right"
        ));
    }

    #[test]
    fn wrong_arity_is_rejected() {
        let space = ActionSpace::from_pb(&joint_space(7)).unwrap();
        let action = part_action(vec![0.0; 6]);
        assert!(matches!(
            flatten_action(&action, &space),
            Err(TypesError::DimensionMismatch {
                expected: 7,
                got: 6
            })
        ));
    }

    #[test]
    fn ee_absolute_flattens_wxyz() {
        let space = ActionSpace::from_pb(&pb::ActionSpace {
            space: Some(pb::action_space::Space::EeAbsolute(pb::EePoseAbs {
                frame_id: "base".into(),
                rotation_encoding: pb::RotationEncoding::QuatWxyz as i32,
            })),
            rate_hz: 30.0,
            chunking: None,
            gripper: None,
        })
        .unwrap();
        let action = pb::Action {
            target: Some(pb::action::Target::EeAbsolute(pb::Pose {
                position: Some(pb::Vec3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                }),
                rotation: Some(pb::Quat {
                    w: 0.5,
                    x: 0.6,
                    y: 0.7,
                    z: 0.8,
                }),
                frame_id: "base".into(),
            })),
            ..Default::default()
        };
        let step = flatten_action(&action, &space).unwrap();
        assert_eq!(step.values.as_slice(), &[1.0, 2.0, 3.0, 0.5, 0.6, 0.7, 0.8]);
    }

    #[test]
    fn noop_is_not_executable() {
        let space = ActionSpace::from_pb(&joint_space(3)).unwrap();
        let action = pb::Action {
            target: Some(pb::action::Target::Noop(pb::NoopMarker {
                reason: pb::NoopReason::BypassActive as i32,
            })),
            ..Default::default()
        };
        assert!(matches!(
            flatten_action(&action, &space),
            Err(TypesError::NoopNotExecutable)
        ));
    }
}
