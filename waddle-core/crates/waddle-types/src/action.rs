//! Executable actions: wire chunks flattened into numeric vectors per the
//! declared action-space layout. This is the single place where wire shapes
//! (oneofs, wxyz quaternions, composite part order) become flat `f64` rows.

use std::sync::Arc;

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
    /// The one declared part this step addresses (`Action.part`), or `None`
    /// for the whole declared space. A part-tagged step's `values` carry
    /// THAT part's width, and the parts it does not address carry no command
    /// at all — "move this part, hold the rest" (docs/FSM.md §4).
    ///
    /// `Arc<str>` because the tag is minted once per wire action at the
    /// intake and then rides the gate's fast path: every later clone is an
    /// atomic increment, never a malloc.
    pub part: Option<Arc<str>>,
}

impl Step {
    /// A step that commands the gripper alone, leaving the arm to hold.
    #[must_use]
    pub fn is_gripper_only(&self) -> bool {
        self.values.is_empty() && self.gripper.is_some()
    }
}

/// Whether an intake honors `Action.part` — the wire field by which one
/// action addresses a single declared part of a `Composite` space instead of
/// carrying a `CompositeAction` that names every part.
///
/// Honoring it is gated on the `waddle.v0.parts` feature flag
/// (docs/VERSIONING.md §3), so the caller that negotiated the connection
/// picks; nothing below this line reads a flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PartPolicy {
    /// Flatten and validate a part-scoped action against **that declared
    /// part's** own space and dims, and tag the step with the part.
    Honor,
    /// Ignore `Action.part` — the pre-flag meaning, in which every action is
    /// read against the whole declared space (so a part-scoped one is
    /// refused on any real multi-part robot, deterministically).
    Ignore,
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
    ///
    /// `parts` decides whether a step that addresses one declared part by
    /// name is honored or read against the whole space; see [`PartPolicy`].
    /// `dims` is the DECLARED space's width either way — a part-scoped step
    /// is narrower than the chunk it rides in, by construction.
    pub fn from_pb(
        chunk: &pb::ActionChunk,
        space: &ActionSpace,
        parts: PartPolicy,
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
            match flatten_action(action, space, parts) {
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
///
/// Under [`PartPolicy::Honor`] an action naming a declared part
/// (`Action.part`) is read against **that part's** space instead: its width
/// is the part's width, and the step comes out tagged
/// ([`Step::part`]). The part is resolved BEFORE anything is decoded, so a
/// name the declaration does not have is [`TypesError::UnknownPart`] — a
/// different fact from a width that doesn't fit, and reported as one
/// (docs/FSM.md §4). `Action.part == ""` is the sole/default part: core, not
/// part-addressed, and identical under either policy.
pub fn flatten_action(
    action: &pb::Action,
    space: &ActionSpace,
    parts: PartPolicy,
) -> Result<Step, TypesError> {
    let part = addressed_part(action, parts);
    let space = match part {
        Some(name) => declared_part(space, name)?,
        None => space,
    };
    let expected = space.dims().ok_or(TypesError::OpaqueNotExecutable)?;
    let gripper = action.gripper.as_ref().map(|g| g.position);
    // Minted once here, at the intake, off the caller's real-time thread.
    let part = part.map(Arc::from);

    if matches!(&action.target, Some(pb::action::Target::Noop(_))) {
        return match gripper {
            Some(_) => Ok(Step {
                offset_ns: action.t_offset_ns,
                values: ActionValues::new(),
                gripper,
                part,
            }),
            None => Err(TypesError::NoopNotExecutable),
        };
    }

    // A part-scoped action addresses ONE part; a CompositeAction inside it
    // would be a second level of part naming, and v0 pins nesting to depth 1
    // (descriptors.proto). Named here so the refusal says which of the two
    // ways of addressing parts the sender mixed, rather than the generic
    // "target arm does not match" it would otherwise fall into.
    if part.is_some() && matches!(&action.target, Some(pb::action::Target::Composite(_))) {
        return Err(TypesError::InvalidValue {
            field: "Action.target",
            reason: "a part-scoped action may not carry a CompositeAction (nesting is depth 1)",
        });
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
        part,
    })
}

/// The declared part this action addresses, or `None` for the whole space.
///
/// `""` is the sole/default part (control.proto `Action.part`) — the whole
/// space by another name, and already core, so it is never a tag.
fn addressed_part(action: &pb::Action, parts: PartPolicy) -> Option<&str> {
    match parts {
        PartPolicy::Ignore => None,
        PartPolicy::Honor => Some(action.part.as_str()).filter(|name| !name.is_empty()),
    }
}

/// Resolve a declared part by name. A space that declares no parts at all
/// has no part by this name either, so both misses are the same refusal:
/// this action does not fit the declared space.
fn declared_part<'a>(space: &'a ActionSpace, name: &str) -> Result<&'a ActionSpace, TypesError> {
    match &space.spec {
        SpaceSpec::Composite { parts } => parts
            .iter()
            .find(|(declared, _)| declared == name)
            .map(|(_, part_space)| part_space)
            .ok_or_else(|| TypesError::UnknownPart(name.to_owned())),
        _ => Err(TypesError::UnknownPart(name.to_owned())),
    }
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
///
/// `part` is the row's [`Step::part`] tag: a part-tagged row is a part-width
/// row, so it is rebuilt against that part's space and comes back out as an
/// `Action` carrying `part`. Without this a part-scoped dispatch would fail
/// to decode against the whole space and would land in `/waddle/actions` as
/// an empty action list — the recording would show that a tick commanded
/// nothing when it commanded one arm.
pub fn unflatten_action(
    values: &[f64],
    gripper: Option<f64>,
    part: Option<&str>,
    space: &ActionSpace,
) -> Result<pb::Action, TypesError> {
    let part = part.filter(|name| !name.is_empty());
    let space = match part {
        Some(name) => declared_part(space, name)?,
        None => space,
    };
    let expected = space.dims().ok_or(TypesError::OpaqueNotExecutable)?;
    let part = part.unwrap_or_default().to_owned();
    if values.is_empty() && gripper.is_some() {
        return Ok(pb::Action {
            target: Some(pb::action::Target::Noop(pb::NoopMarker::default())),
            gripper: gripper.map(|position| pb::GripperCommand {
                position,
                effort: None,
            }),
            part,
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
    action.part = part;
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
        let step = flatten_action(&action, &space, PartPolicy::Ignore).unwrap();
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
            flatten_action(&action, &space, PartPolicy::Ignore),
            Err(TypesError::MissingPart(p)) if p == "right"
        ));
    }

    /// `Action.part` addresses ONE declared part, so the action is read
    /// against that part's space: 7 values against a 14-wide bimanual is the
    /// left arm's width, not a mismatch. This is what lets a sender
    /// intervene on one arm without inventing values for the other.
    #[test]
    fn part_scoped_action_flattens_against_the_parts_space() {
        let space = bimanual_space();
        for (name, value) in [("left", 1.0), ("right", 2.0)] {
            let action = pb::Action {
                part: name.into(),
                t_offset_ns: 20_000_000,
                ..part_action(vec![value; 7])
            };
            let step = flatten_action(&action, &space, PartPolicy::Honor).unwrap();
            assert_eq!(step.values.as_slice(), &[value; 7]);
            assert_eq!(step.part.as_deref(), Some(name));
            assert_eq!(step.offset_ns, 20_000_000);
        }
    }

    /// The width a part-scoped action is measured against is the PART's, so
    /// the mismatch report names the part's width — the number the sender
    /// has to fix, not the composite's.
    #[test]
    fn part_scoped_dims_mismatch_names_the_parts_width() {
        let space = bimanual_space();
        let action = pb::Action {
            part: "left".into(),
            ..part_action(vec![0.0; 6])
        };
        assert!(matches!(
            flatten_action(&action, &space, PartPolicy::Honor),
            Err(TypesError::DimensionMismatch {
                expected: 7,
                got: 6
            })
        ));
    }

    /// A part the declaration does not have is a DIFFERENT fact from a width
    /// that does not fit — "the sender disagrees about which parts exist" —
    /// and it is resolved before anything is decoded, so a well-formed
    /// 7-value action still refuses by name. A space that declares no parts
    /// at all answers the same way: it has no part by that name either.
    #[test]
    fn part_scoped_unknown_part_is_refused() {
        let space = bimanual_space();
        let action = pb::Action {
            part: "waist".into(),
            ..part_action(vec![0.0; 7])
        };
        assert!(matches!(
            flatten_action(&action, &space, PartPolicy::Honor),
            Err(TypesError::UnknownPart(p)) if p == "waist"
        ));

        let single = ActionSpace::from_pb(&joint_space(7)).unwrap();
        let action = pb::Action {
            part: "left".into(),
            ..part_action(vec![0.0; 7])
        };
        assert!(matches!(
            flatten_action(&action, &single, PartPolicy::Honor),
            Err(TypesError::UnknownPart(p)) if p == "left"
        ));

        // The same refusal on the way back out: a row tagged with a part the
        // declaration does not have has no wire form.
        assert!(matches!(
            unflatten_action(&[0.0; 7], None, Some("waist"), &space),
            Err(TypesError::UnknownPart(p)) if p == "waist"
        ));
    }

    /// The two ways of addressing parts do not nest: v0 pins Composite depth
    /// to 1, so a part-scoped action carrying a `CompositeAction` is refused
    /// in its own words rather than decoded one level deeper.
    ///
    /// The WORDS are the whole point of the guard — without it this action
    /// still refuses, but as the generic "target arm does not match the
    /// declared action space" the fall-through arm produces, which tells a
    /// sender that mixed the two addressing modes nothing about what it
    /// mixed. The reason string is what reaches that sender (it becomes
    /// `RejectReason::NotExecutable(err.to_string())` at the intake), so it
    /// is asserted, not `..`-ignored.
    #[test]
    fn part_scoped_composite_target_is_refused() {
        let space = bimanual_space();
        let action = pb::Action {
            part: "left".into(),
            target: Some(pb::action::Target::Composite(pb::CompositeAction {
                parts: vec![pb::composite_action::PartAction {
                    name: "left".into(),
                    action: Some(part_action(vec![1.0; 7])),
                }],
            })),
            ..Default::default()
        };
        let refused = flatten_action(&action, &space, PartPolicy::Honor);
        let Err(TypesError::InvalidValue { field, reason }) = refused else {
            panic!("expected a refusal, got {refused:?}");
        };
        assert_eq!(field, "Action.target");
        assert!(
            reason.contains("CompositeAction") && reason.contains("depth 1"),
            "the refusal must name the nesting the sender attempted, not fall \
             through to the generic arm-mismatch reason: {reason:?}"
        );
    }

    /// "Move this part, hold the rest" generalizes "hold the arm, move the
    /// gripper": a part-scoped noop carrying a gripper is that part's
    /// gripper-only step, and it keeps its tag.
    #[test]
    fn part_scoped_noop_with_a_gripper_is_a_part_tagged_gripper_only_step() {
        let space = bimanual_space();
        let action = pb::Action {
            target: Some(pb::action::Target::Noop(pb::NoopMarker::default())),
            gripper: Some(pb::GripperCommand {
                position: 0.04,
                effort: None,
            }),
            part: "right".into(),
            ..Default::default()
        };
        let step = flatten_action(&action, &space, PartPolicy::Honor).unwrap();
        assert!(step.is_gripper_only());
        assert_eq!(step.part.as_deref(), Some("right"));
        assert_eq!(
            unflatten_action(
                step.values.as_slice(),
                step.gripper,
                step.part.as_deref(),
                &space
            )
            .unwrap(),
            action
        );
    }

    /// Pre-flag behavior, pinned: on a connection that did not negotiate
    /// `waddle.v0.parts` the field is not read at all, so a part-scoped
    /// action means exactly what it meant before the flag existed — read
    /// against the whole declared space, hence refused on a real multi-part
    /// robot. And an action that addresses no part (`""`, the sole/default
    /// part) is identical under both policies: the pre-flag shape stays
    /// byte-compatible under the flag.
    #[test]
    fn part_ignored_policy_keeps_todays_behavior() {
        let space = bimanual_space();
        let scoped = pb::Action {
            part: "left".into(),
            ..part_action(vec![1.0; 7])
        };
        let untagged = pb::Action {
            part: String::new(),
            ..scoped.clone()
        };
        assert_eq!(
            flatten_action(&scoped, &space, PartPolicy::Ignore),
            flatten_action(&untagged, &space, PartPolicy::Ignore),
            "IGNORE reads the action as if the field were not set"
        );
        assert!(
            matches!(
                flatten_action(&scoped, &space, PartPolicy::Ignore),
                Err(TypesError::InvalidValue {
                    field: "Action.target",
                    ..
                })
            ),
            "one arm's joint vector is not the 14-wide composite's target arm"
        );

        let whole = pb::Action {
            target: Some(pb::action::Target::Composite(pb::CompositeAction {
                parts: vec![
                    pb::composite_action::PartAction {
                        name: "left".into(),
                        action: Some(part_action(vec![1.0; 7])),
                    },
                    pb::composite_action::PartAction {
                        name: "right".into(),
                        action: Some(part_action(vec![2.0; 7])),
                    },
                ],
            })),
            ..Default::default()
        };
        assert_eq!(
            flatten_action(&whole, &space, PartPolicy::Honor).unwrap(),
            flatten_action(&whole, &space, PartPolicy::Ignore).unwrap(),
        );
        assert_eq!(
            flatten_action(&whole, &space, PartPolicy::Honor)
                .unwrap()
                .part,
            None,
            "a whole-robot action carries no part tag under either policy"
        );
    }

    /// The wire↔row seam is symmetric for part-scoped actions too: without
    /// this a part-width row would fail to decode against the whole space
    /// and the recording would show a tick that commanded nothing.
    #[test]
    fn part_scoped_round_trips_through_unflatten() {
        let space = bimanual_space();
        let row: Vec<f64> = (0..7).map(f64::from).collect();
        for gripper in [None, Some(0.25)] {
            let action = unflatten_action(&row, gripper, Some("right"), &space).unwrap();
            assert_eq!(action.part, "right");
            assert!(
                matches!(&action.target, Some(pb::action::Target::JointPosition(v)) if v.values == row),
                "rebuilt against the PART's space, not the composite: {action:?}"
            );

            let step = flatten_action(&action, &space, PartPolicy::Honor).unwrap();
            assert_eq!(step.values.as_slice(), row.as_slice());
            assert_eq!(step.gripper, gripper);
            assert_eq!(step.part.as_deref(), Some("right"));
            assert_eq!(
                unflatten_action(
                    step.values.as_slice(),
                    step.gripper,
                    step.part.as_deref(),
                    &space
                )
                .unwrap(),
                action
            );
        }
    }

    /// Addressing is per STEP, not per chunk: one chunk may move the left
    /// arm, then the right, then both.
    #[test]
    fn from_pb_tags_each_step_with_the_part_it_addresses() {
        let space = bimanual_space();
        let chunk = pb::ActionChunk {
            actions: vec![
                pb::Action {
                    part: "left".into(),
                    ..part_action(vec![1.0; 7])
                },
                pb::Action {
                    part: "right".into(),
                    ..part_action(vec![2.0; 7])
                },
                pb::Action {
                    target: Some(pb::action::Target::Composite(pb::CompositeAction {
                        parts: vec![
                            pb::composite_action::PartAction {
                                name: "left".into(),
                                action: Some(part_action(vec![3.0; 7])),
                            },
                            pb::composite_action::PartAction {
                                name: "right".into(),
                                action: Some(part_action(vec![4.0; 7])),
                            },
                        ],
                    })),
                    ..Default::default()
                },
            ],
            ..Default::default()
        };
        let flattened = ActionChunk::from_pb(&chunk, &space, PartPolicy::Honor).unwrap();
        let steps = &flattened.chunk.steps;
        assert_eq!(steps.len(), 3);
        assert_eq!(steps[0].part.as_deref(), Some("left"));
        assert_eq!(steps[0].values.len(), 7);
        assert_eq!(steps[1].part.as_deref(), Some("right"));
        assert_eq!(steps[2].part, None);
        assert_eq!(steps[2].values.len(), 14);
        assert_eq!(
            flattened.chunk.dims, 14,
            "the chunk's dims stay the DECLARED width; the steps are narrower"
        );

        // Under IGNORE the same chunk is refused whole: partial trajectories
        // from a sender that disagrees about the space are never actuated.
        assert!(ActionChunk::from_pb(&chunk, &space, PartPolicy::Ignore).is_err());
    }

    #[test]
    fn wrong_arity_is_rejected() {
        let space = ActionSpace::from_pb(&joint_space(7)).unwrap();
        let action = part_action(vec![0.0; 6]);
        assert!(matches!(
            flatten_action(&action, &space, PartPolicy::Ignore),
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
        let step = flatten_action(&action, &space, PartPolicy::Ignore).unwrap();
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
                let action = unflatten_action(&row, gripper, None, &space).unwrap();
                let step = flatten_action(&action, &space, PartPolicy::Ignore).unwrap();
                assert_eq!(step.values.as_slice(), row.as_slice(), "{space:?}");
                assert_eq!(step.gripper, gripper);
                // The wire message itself is stable under a second pass.
                assert_eq!(
                    unflatten_action(step.values.as_slice(), step.gripper, None, &space).unwrap(),
                    action
                );
            }
        }
    }

    #[test]
    fn unflatten_composite_emits_parts_in_declaration_order() {
        let space = bimanual_space();
        let row: Vec<f64> = [[1.0; 7], [2.0; 7]].concat();
        let action = unflatten_action(&row, None, None, &space).unwrap();
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
            unflatten_action(&[0.0; 6], None, None, &space),
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
            unflatten_action(&[0.0; 4], None, None, &opaque),
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
            flatten_action(&action, &space, PartPolicy::Ignore),
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
        let step = flatten_action(&action, &space, PartPolicy::Ignore).unwrap();
        assert!(step.is_gripper_only());
        assert!(step.values.is_empty(), "the arm holds: no values to write");
        assert_eq!(step.gripper, Some(0.04), "declared units, unmapped");
        assert_eq!(step.offset_ns, 120_000_000);

        // Round-trips through the wire shape it came from.
        assert_eq!(
            unflatten_action(step.values.as_slice(), step.gripper, None, &space).unwrap(),
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
        let flattened = ActionChunk::from_pb(&chunk, &space, PartPolicy::Ignore).unwrap();
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
            ActionChunk::from_pb(&chunk, &space, PartPolicy::Ignore),
            Err(TypesError::DimensionMismatch {
                expected: 3,
                got: 2
            })
        ));
    }
}
