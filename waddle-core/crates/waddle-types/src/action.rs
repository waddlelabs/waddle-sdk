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
    ///
    /// EMPTY means a gripper-only step — "hold the arm, move the gripper"
    /// (see [`flatten_action`]). The arm keeps whatever it was commanded
    /// last; only the gripper channel is written. Every other step carries
    /// exactly the declared space's width.
    pub values: ActionValues,
    /// Gripper command in declared units, when present.
    pub gripper: Option<f64>,
}

impl Step {
    /// A step that commands the gripper alone, leaving the arm to hold.
    #[must_use]
    pub fn is_gripper_only(&self) -> bool {
        self.values.is_empty() && self.gripper.is_some()
    }
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

/// What one wire chunk flattened into: the executable steps, plus the
/// steps that carried nothing to execute and were left out. See
/// [`ActionChunk::from_pb`].
#[derive(Debug, Clone, PartialEq)]
pub struct FlattenedChunk {
    pub chunk: ActionChunk,
    /// Indices into `pb::ActionChunk.actions` of the INERT steps — a
    /// `NoopMarker` target with no gripper command riding along, i.e. "this
    /// tick commands nothing". Legal wire with nothing to dispatch. They are
    /// reported rather than silently swallowed so the intake can say so on
    /// the episode's own timeline.
    pub inert: Vec<usize>,
}

impl ActionChunk {
    /// Validate and flatten a wire chunk against the declared space.
    ///
    /// Two failure shapes, deliberately different:
    ///
    /// * An INERT step (a `NoopMarker` carrying no gripper command) is
    ///   skipped and reported in [`FlattenedChunk::inert`]; the rest of the
    ///   chunk still executes. One step with nothing in it must never cost
    ///   the sender the waypoints around it.
    /// * Anything else — a target arm the declared space doesn't have, a
    ///   missing field, a width that doesn't match — means this chunk isn't
    ///   speaking the declared space, and the WHOLE chunk is refused
    ///   (`Err`). A partial trajectory from a sender that disagrees about
    ///   the space is not a degraded-but-safe thing to actuate.
    pub fn from_pb(
        chunk: &pb::ActionChunk,
        space: &ActionSpace,
    ) -> Result<FlattenedChunk, TypesError> {
        let dims = space.dims().ok_or(TypesError::OpaqueNotExecutable)?;
        let provenance = chunk
            .provenance
            .as_ref()
            .map(ProvenanceTag::try_from)
            .transpose()?
            .unwrap_or_else(ProvenanceTag::policy);

        let mut steps = Vec::with_capacity(chunk.actions.len());
        let mut inert = Vec::new();
        for (index, action) in chunk.actions.iter().enumerate() {
            match flatten_action(action, space) {
                Ok(step) => steps.push(step),
                Err(TypesError::NoopNotExecutable) => inert.push(index),
                Err(err) => return Err(err),
            }
        }

        Ok(FlattenedChunk {
            chunk: Self {
                steps,
                dims,
                horizon_ns: chunk.horizon_ns,
                t_emitted_ns: chunk.t_emitted_ns,
                t_obs_ns: chunk.t_obs_ns,
                seq: chunk.seq,
                source: SourceId::new(&chunk.source_id),
                provenance,
            },
            inert,
        })
    }
}

/// Flatten one wire action against the declared space.
///
/// A `NoopMarker` target carrying a gripper command is EXECUTABLE: control.proto
/// has the gripper "ride alongside the target in one logical tick", and noop is
/// a target arm like any other, so the pair says "hold the arm, move the
/// gripper". It flattens to a step with no values (see [`Step::values`]).
/// A noop with no gripper carries nothing to dispatch and stays
/// [`TypesError::NoopNotExecutable`].
pub fn flatten_action(action: &pb::Action, space: &ActionSpace) -> Result<Step, TypesError> {
    let expected = space.dims().ok_or(TypesError::OpaqueNotExecutable)?;
    let gripper = action.gripper.as_ref().map(|g| g.position);

    if matches!(&action.target, Some(pb::action::Target::Noop(_))) {
        return match gripper {
            Some(_) => Ok(Step {
                offset_ns: action.t_offset_ns,
                values: ActionValues::new(),
                gripper,
            }),
            None => Err(TypesError::NoopNotExecutable),
        };
    }

    let mut values = ActionValues::new();
    flatten_target(action, space, &mut values)?;

    if values.len() != expected {
        return Err(TypesError::DimensionMismatch {
            expected,
            got: values.len(),
        });
    }

    Ok(Step {
        offset_ns: action.t_offset_ns,
        values,
        gripper,
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

/// The exact inverse of the flattening above: rebuild a wire action from a
/// flat row against the declared space. This is the ONE place flat rows
/// become wire shapes (composite parts in declaration order; poses as
/// `[x, y, z, qw, qx, qy, qz]`, wxyz on the wire).
///
/// An empty row with a gripper is the gripper-only step [`flatten_action`]
/// produces, and rebuilds the wire shape it came from: a `NoopMarker` with
/// the gripper riding alongside. Without this a dispatched gripper-only
/// action would have no wire form and would vanish from the recording.
pub fn unflatten_action(
    values: &[f64],
    gripper: Option<f64>,
    space: &ActionSpace,
) -> Result<pb::Action, TypesError> {
    let expected = space.dims().ok_or(TypesError::OpaqueNotExecutable)?;
    if values.is_empty() && gripper.is_some() {
        return Ok(pb::Action {
            target: Some(pb::action::Target::Noop(pb::NoopMarker::default())),
            gripper: gripper.map(|position| pb::GripperCommand {
                position,
                effort: None,
            }),
            ..Default::default()
        });
    }
    if values.len() != expected {
        return Err(TypesError::DimensionMismatch {
            expected,
            got: values.len(),
        });
    }
    let mut cursor = values;
    let mut action = unflatten_target(&mut cursor, space)?;
    debug_assert!(cursor.is_empty(), "dims() must equal the consumed width");
    action.gripper = gripper.map(|position| pb::GripperCommand {
        position,
        effort: None,
    });
    Ok(action)
}

fn unflatten_target(cursor: &mut &[f64], space: &ActionSpace) -> Result<pb::Action, TypesError> {
    // The top-level arity check against dims() guarantees enough values for
    // every leaf; a short take here would be an internal inconsistency.
    fn take<'a>(cursor: &mut &'a [f64], n: usize) -> Result<&'a [f64], TypesError> {
        if cursor.len() < n {
            return Err(TypesError::DimensionMismatch {
                expected: n,
                got: cursor.len(),
            });
        }
        let (head, tail) = cursor.split_at(n);
        *cursor = tail;
        Ok(head)
    }
    fn twist(v: &[f64]) -> pb::Twist {
        pb::Twist {
            linear: Some(pb::Vec3 {
                x: v[0],
                y: v[1],
                z: v[2],
            }),
            angular: Some(pb::Vec3 {
                x: v[3],
                y: v[4],
                z: v[5],
            }),
        }
    }

    let target = match &space.spec {
        SpaceSpec::JointPosition { joints } => pb::action::Target::JointPosition(pb::JointVector {
            values: take(cursor, joints.len())?.to_vec(),
        }),
        SpaceSpec::JointVelocity { joints } => pb::action::Target::JointVelocity(pb::JointVector {
            values: take(cursor, joints.len())?.to_vec(),
        }),
        SpaceSpec::EePoseDelta { .. } => pb::action::Target::EeDelta(twist(take(cursor, 6)?)),
        SpaceSpec::BaseTwist { .. } => pb::action::Target::BaseTwist(twist(take(cursor, 6)?)),
        SpaceSpec::EePoseAbs { frame, .. } => {
            let v = take(cursor, 7)?;
            pb::action::Target::EeAbsolute(pb::Pose {
                position: Some(pb::Vec3 {
                    x: v[0],
                    y: v[1],
                    z: v[2],
                }),
                // wxyz in the flat layout, wxyz on the wire.
                rotation: Some(pb::Quat {
                    w: v[3],
                    x: v[4],
                    y: v[5],
                    z: v[6],
                }),
                frame_id: frame.as_str().to_owned(),
            })
        }
        SpaceSpec::Composite { parts } => {
            let mut out = Vec::with_capacity(parts.len());
            for (name, part_space) in parts {
                out.push(pb::composite_action::PartAction {
                    name: name.clone(),
                    action: Some(unflatten_target(cursor, part_space)?),
                });
            }
            pb::action::Target::Composite(pb::CompositeAction { parts: out })
        }
        SpaceSpec::Opaque { .. } => return Err(TypesError::OpaqueNotExecutable),
    };
    Ok(pb::Action {
        target: Some(target),
        ..Default::default()
    })
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

    /// Round-trip property: for every executable space shape,
    /// `flatten_action(unflatten_action(row)) == row` (and the wire message
    /// survives a second unflatten identically).
    #[test]
    fn unflatten_is_the_exact_inverse_of_flatten() {
        let ee_abs = ActionSpace::from_pb(&pb::ActionSpace {
            space: Some(pb::action_space::Space::EeAbsolute(pb::EePoseAbs {
                frame_id: "base".into(),
                rotation_encoding: pb::RotationEncoding::QuatWxyz as i32,
            })),
            rate_hz: 30.0,
            chunking: None,
            gripper: None,
        })
        .unwrap();
        let ee_delta = ActionSpace::from_pb(&pb::ActionSpace {
            space: Some(pb::action_space::Space::EeDelta(pb::EePoseDelta {
                frame_id: "base".into(),
                rotation_encoding: pb::RotationEncoding::Rotvec as i32,
                delta_frame: pb::DeltaFrame::Base as i32,
                ..Default::default()
            })),
            rate_hz: 30.0,
            chunking: None,
            gripper: None,
        })
        .unwrap();
        let cases: Vec<(ActionSpace, Vec<f64>)> = vec![
            (
                ActionSpace::from_pb(&joint_space(7)).unwrap(),
                (0..7).map(f64::from).collect(),
            ),
            (ee_delta, vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.6]),
            (ee_abs, vec![1.0, 2.0, 3.0, 0.5, 0.6, 0.7, 0.8]),
            (bimanual_space(), (0..14).map(f64::from).collect()),
        ];
        for (space, row) in cases {
            for gripper in [None, Some(0.25)] {
                let action = unflatten_action(&row, gripper, &space).unwrap();
                let step = flatten_action(&action, &space).unwrap();
                assert_eq!(step.values.as_slice(), row.as_slice(), "{space:?}");
                assert_eq!(step.gripper, gripper);
                // The wire message itself is stable under a second pass.
                assert_eq!(
                    unflatten_action(step.values.as_slice(), step.gripper, &space).unwrap(),
                    action
                );
            }
        }
    }

    #[test]
    fn unflatten_composite_emits_parts_in_declaration_order() {
        let space = bimanual_space();
        let row: Vec<f64> = [[1.0; 7], [2.0; 7]].concat();
        let action = unflatten_action(&row, None, &space).unwrap();
        let Some(pb::action::Target::Composite(c)) = &action.target else {
            panic!("expected composite target, got {action:?}");
        };
        assert_eq!(c.parts[0].name, "left");
        assert_eq!(c.parts[1].name, "right");
    }

    #[test]
    fn unflatten_rejects_dims_mismatch_and_opaque() {
        let space = ActionSpace::from_pb(&joint_space(7)).unwrap();
        assert!(matches!(
            unflatten_action(&[0.0; 6], None, &space),
            Err(TypesError::DimensionMismatch {
                expected: 7,
                got: 6
            })
        ));

        let opaque = ActionSpace::from_pb(&pb::ActionSpace {
            space: Some(pb::action_space::Space::Opaque(pb::Opaque {
                format_hint: "vendor".into(),
                dim: Some(4),
            })),
            rate_hz: 10.0,
            chunking: None,
            gripper: None,
        })
        .unwrap();
        assert!(matches!(
            unflatten_action(&[0.0; 4], None, &opaque),
            Err(TypesError::OpaqueNotExecutable)
        ));
    }

    #[test]
    fn noop_without_a_gripper_is_not_executable() {
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

    /// "Hold the arm, move the gripper": the gripper rides ALONGSIDE the
    /// target (control.proto), so a noop target with a gripper command is a
    /// legal executable action — the shape a supervision plane sends for a
    /// gripper-only command, since its own command carries no arm target to
    /// put beside it.
    #[test]
    fn noop_carrying_a_gripper_is_a_gripper_only_step() {
        let space = ActionSpace::from_pb(&joint_space(3)).unwrap();
        let action = pb::Action {
            target: Some(pb::action::Target::Noop(pb::NoopMarker::default())),
            gripper: Some(pb::GripperCommand {
                position: 0.04,
                effort: None,
            }),
            t_offset_ns: 120_000_000,
            ..Default::default()
        };
        let step = flatten_action(&action, &space).unwrap();
        assert!(step.is_gripper_only());
        assert!(step.values.is_empty(), "the arm holds: no values to write");
        assert_eq!(step.gripper, Some(0.04), "declared units, unmapped");
        assert_eq!(step.offset_ns, 120_000_000);

        // Round-trips through the wire shape it came from.
        assert_eq!(
            unflatten_action(step.values.as_slice(), step.gripper, &space).unwrap(),
            action_without_offset(&action)
        );
    }

    fn action_without_offset(action: &pb::Action) -> pb::Action {
        pb::Action {
            t_offset_ns: 0,
            ..action.clone()
        }
    }

    /// One inert step must never cost the sender the waypoints around it:
    /// it is skipped and reported, the rest of the chunk still executes.
    #[test]
    fn from_pb_skips_the_inert_step_and_keeps_the_rest() {
        let space = ActionSpace::from_pb(&joint_space(3)).unwrap();
        let chunk = pb::ActionChunk {
            actions: vec![
                part_action(vec![1.0; 3]),
                pb::Action {
                    target: Some(pb::action::Target::Noop(pb::NoopMarker::default())),
                    ..Default::default()
                },
                part_action(vec![3.0; 3]),
            ],
            seq: 7,
            ..Default::default()
        };
        let flattened = ActionChunk::from_pb(&chunk, &space).unwrap();
        assert_eq!(flattened.inert, vec![1]);
        assert_eq!(flattened.chunk.steps.len(), 2);
        assert_eq!(flattened.chunk.steps[0].values.as_slice(), &[1.0; 3]);
        assert_eq!(flattened.chunk.steps[1].values.as_slice(), &[3.0; 3]);
    }

    /// A step that isn't speaking the declared space at all is a different
    /// fact: the whole chunk is refused, never partially actuated.
    #[test]
    fn from_pb_refuses_the_whole_chunk_on_a_space_mismatch() {
        let space = ActionSpace::from_pb(&joint_space(3)).unwrap();
        let chunk = pb::ActionChunk {
            actions: vec![part_action(vec![1.0; 3]), part_action(vec![0.0; 2])],
            ..Default::default()
        };
        assert!(matches!(
            ActionChunk::from_pb(&chunk, &space),
            Err(TypesError::DimensionMismatch {
                expected: 3,
                got: 2
            })
        ));
    }
}
