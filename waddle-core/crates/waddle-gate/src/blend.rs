//! Pure handoff blending math. Cross-fades between the last commanded point
//! and the intervention stream per the declared interpolation rule.

use waddle_types::Interp;

use crate::gate::OwnedAction;

/// Interpolation weight for progress `t` in [0, 1].
#[must_use]
fn weight(interp: Interp, t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    match interp {
        // Zero-order hold: switch at the end of the window.
        Interp::Hold => {
            if t >= 1.0 {
                1.0
            } else {
                0.0
            }
        }
        Interp::Linear => t,
        // Smoothstep: C1-continuous ease for cubic-declared spaces.
        Interp::Cubic => t * t * (3.0 - 2.0 * t),
    }
}

/// Blend one step: `from * (1-w) + to * w` componentwise, or `None` when the
/// two are not endpoints of the same cross-fade — callers fall back to
/// `Hold`, which for the gate means the action is consumed and dropped, not
/// deferred (docs/FSM.md §5).
///
/// `from` is the last action that left the gate, or `None` when none has yet
/// — a caller that never ticked (an agent-invited episode, FSM.md E24) or a
/// bypass pump. With no point to fade from, a whole-robot action is its own
/// endpoint and crosses the window unchanged; a part-scoped one has no
/// endpoint at all, and manufacturing one out of the target itself would
/// fade it in as if it commanded the whole robot.
///
/// Two shapes are not endpoints for each other:
///
/// - **Mismatched widths.** Action-space validation lives at the intakes
///   (`spawn_media_intake`, the intervention-chunk arm), so a mismatch here
///   should never happen in practice; this is defense in depth, not the
///   primary check. It refuses rather than zip-truncating — a truncated
///   action is a meaningless one, never a degraded-but-safe one. It is also
///   what holds a part-scoped action out of a whole-robot cross-fade
///   (FSM.md §5): a part-width action is not an endpoint for the parts it
///   does not address.
/// - **Two DIFFERENT declared parts** ([`OwnedAction::part`]). Their widths
///   match whenever the parts are symmetric — two 7-dof arms — so the width
///   check cannot see this pair, and blending it would interpolate one arm's
///   last setpoint into the other arm's target: a trajectory no sender
///   issued, dispatched and recorded under the sender's provenance. An
///   UNTAGGED action commands the whole declared space, hence whatever part
///   the other side names, so only two distinct names disqualify a pair on
///   scope.
///
/// The two rules are independent, and neither stands in for the other. What
/// holds a whole-robot anchor out of a PROPER part's cross-fade is the width
/// rule, not the scope one: the gate carries no part layout and cannot slice
/// a 14-wide anchor down to the left arm's rows, so the pair refuses on
/// width. They agree only where the part's width IS the full width — the
/// sole part of a one-part `Composite`, the degenerate case FSM.md §5 says
/// does cross-fade.
///
/// One `to` shape is legitimately shorter and must not read as a mismatch:
/// a gripper-only action ("hold the arm, move the gripper" —
/// `waddle_types::Step`) carries no arm row at all. It stays gripper-only
/// through the window, with the gripper channel cross-faded; holding
/// instead would silently drop a commanded grip. That exempts it from the
/// WIDTH check only — it still takes the anchor's gripper as its starting
/// point, so the scope rule binds it on the same terms as an arm row: two
/// distinct names refuse here too.
///
/// What does not carry over is the width rule's incidental reach. A
/// part-tagged gripper-only action DOES fade out of an untagged anchor of
/// any width, where a part-tagged arm row would have been refused on width.
/// That is the rule landing where it should, not a hole in it: the anchor
/// commands every part's grip, an `OwnedAction` carries ONE gripper scalar
/// (v0's sidechannel is per action, never per part — `flatten_action` reads
/// `Action.gripper` before it even resolves the part), and there are no rows
/// to slice, so nothing is fabricated. Refusing would drop a commanded grip
/// for the length of the window, which is what the exemption exists to
/// prevent. Should the sidechannel ever become per-part (the deferred
/// media-plane part-routing work), that premise dies and this pair becomes a
/// scope question again.
///
/// `from` is only the anchor: what leaves the gate commands what `to`
/// commands, so the blended action carries `to`'s part tag.
#[must_use]
pub fn blend_step(
    from: Option<&OwnedAction>,
    to: &OwnedAction,
    t: f32,
    interp: Interp,
) -> Option<OwnedAction> {
    let gripper_only = to.values.is_empty() && to.gripper.is_some();
    let Some(from) = from else {
        return (gripper_only || to.part.is_none()).then(|| to.clone());
    };
    if crosses_parts(from, to) || (!gripper_only && from.values.len() != to.values.len()) {
        return None;
    }
    let w = f64::from(weight(interp, t));
    let values = from
        .values
        .iter()
        .zip(to.values.iter())
        .map(|(a, b)| a * (1.0 - w) + b * w)
        .collect();
    Some(OwnedAction {
        values,
        gripper: match (from.gripper, to.gripper) {
            (Some(a), Some(b)) => Some(a * (1.0 - w) + b * w),
            (_, b @ Some(_)) => b,
            (a, None) => a,
        },
        part: to.part.clone(),
    })
}

/// Do these two actions address two DIFFERENT declared parts? See
/// [`blend_step`] for why that disqualifies them as endpoints, and why an
/// untagged action is compatible with either.
fn crosses_parts(from: &OwnedAction, to: &OwnedAction) -> bool {
    matches!(
        (from.part.as_deref(), to.part.as_deref()),
        (Some(anchor), Some(target)) if anchor != target
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use smallvec::SmallVec;

    fn action(vals: &[f64]) -> OwnedAction {
        OwnedAction {
            values: SmallVec::from_slice(vals),
            gripper: None,
            part: None,
        }
    }

    /// The same action, addressed to one declared part.
    fn part_action(part: &str, vals: &[f64]) -> OwnedAction {
        OwnedAction {
            part: Some(std::sync::Arc::from(part)),
            ..action(vals)
        }
    }

    proptest! {
        /// Endpoints are exact and the linear blend never overshoots the
        /// per-component envelope.
        #[test]
        fn linear_blend_is_bounded_with_exact_endpoints(
            a in proptest::collection::vec(-10.0f64..10.0, 1..16),
            b_offsets in proptest::collection::vec(-10.0f64..10.0, 1..16),
            t in 0.0f32..=1.0,
        ) {
            let n = a.len().min(b_offsets.len());
            let a = &a[..n];
            let b: Vec<f64> = a.iter().zip(&b_offsets[..n]).map(|(x, d)| x + d).collect();
            let from = action(a);
            let to = action(&b);

            let at0 = blend_step(Some(&from), &to, 0.0, Interp::Linear).unwrap();
            let at1 = blend_step(Some(&from), &to, 1.0, Interp::Linear).unwrap();
            prop_assert_eq!(at0.values.as_slice(), from.values.as_slice());
            prop_assert_eq!(at1.values.as_slice(), to.values.as_slice());

            let mid = blend_step(Some(&from), &to, t, Interp::Linear).unwrap();
            for ((m, x), y) in mid.values.iter().zip(a).zip(&b) {
                let (lo, hi) = if x <= y { (x, y) } else { (y, x) };
                prop_assert!(*m >= lo - 1e-9 && *m <= hi + 1e-9);
            }
        }

        /// Monotone progress never reverses direction under Linear.
        #[test]
        fn linear_blend_is_monotone(
            t1 in 0.0f32..=1.0,
            t2 in 0.0f32..=1.0,
        ) {
            let from = action(&[0.0]);
            let to = action(&[1.0]);
            let (lo, hi) = if t1 <= t2 { (t1, t2) } else { (t2, t1) };
            let a = blend_step(Some(&from), &to, lo, Interp::Linear).unwrap();
            let b = blend_step(Some(&from), &to, hi, Interp::Linear).unwrap();
            prop_assert!(a.values[0] <= b.values[0] + 1e-9);
        }
    }

    #[test]
    fn hold_switches_only_at_the_end() {
        let from = action(&[0.0]);
        let to = action(&[1.0]);
        assert_eq!(
            blend_step(Some(&from), &to, 0.99, Interp::Hold)
                .unwrap()
                .values[0],
            0.0
        );
        assert_eq!(
            blend_step(Some(&from), &to, 1.0, Interp::Hold)
                .unwrap()
                .values[0],
            1.0
        );
    }

    /// Dims-validation defense in depth: a dims mismatch must never zip-truncate
    /// silently. Intake validation should keep this from happening in
    /// practice, but the blend step itself must refuse rather than produce
    /// a truncated, meaningless action.
    #[test]
    fn mismatched_dims_return_none_instead_of_truncating() {
        let from = action(&[0.0, 0.0, 0.0]);
        let to = action(&[1.0, 1.0]); // shorter: would silently truncate today
        assert!(blend_step(Some(&from), &to, 0.5, Interp::Linear).is_none());
    }

    /// A part-scoped action is not an endpoint for a whole-robot cross-fade:
    /// its width is the part's, so splicing it in would either truncate the
    /// target or fabricate values for the parts it does not address (FSM.md
    /// §4 bans both, §5 holds instead).
    #[test]
    fn a_part_scoped_action_is_not_an_endpoint_for_a_whole_robot_anchor() {
        let from = action(&[0.0; 14]); // last commanded whole-robot point
        let to = part_action("left", &[1.0; 7]);
        assert!(blend_step(Some(&from), &to, 0.5, Interp::Linear).is_none());
    }

    /// The one part-scoped pair whose widths MATCH: two symmetric arms. The
    /// width check cannot see this, and blending anyway would interpolate one
    /// arm's last setpoint into the other arm's target — a trajectory the
    /// sender never issued, recorded under the sender's provenance.
    #[test]
    fn two_different_parts_are_never_endpoints_for_each_other() {
        let from = part_action("left", &[0.0; 7]);
        let to = part_action("right", &[1.0; 7]);
        assert!(
            blend_step(Some(&from), &to, 0.5, Interp::Linear).is_none(),
            "one arm's anchor must never fade into the other arm's target"
        );
    }

    /// `from` is only the anchor; what leaves the gate commands what `to`
    /// commands. Shown on the one-part `Composite` — the degenerate case
    /// FSM.md §5 says DOES cross-fade, because the part's width is the full
    /// width — where losing the tag would silently widen a part command into
    /// a whole-robot one.
    #[test]
    fn a_blend_commands_the_part_its_target_commands() {
        let from = action(&[0.0; 7]);
        let to = part_action("arm", &[1.0; 7]);
        let mid = blend_step(Some(&from), &to, 0.5, Interp::Linear)
            .expect("the sole part's width IS the full width");
        assert_eq!(
            mid.part.as_deref(),
            Some("arm"),
            "the blended action commands the part its target commanded"
        );
        assert!((mid.values[0] - 0.5).abs() < 1e-12);
    }

    /// No anchor at all (nothing has left the gate yet). A whole-robot
    /// action is its own endpoint and crosses the window unchanged — the
    /// behavior before parts existed, when the gate anchored a missing
    /// `from` on the target itself.
    #[test]
    fn with_nothing_commanded_yet_a_whole_robot_action_crosses_the_window() {
        let to = action(&[1.0, 2.0, 3.0]);
        let mid = blend_step(None, &to, 0.5, Interp::Linear).expect("its own endpoint");
        assert_eq!(mid.values.as_slice(), to.values.as_slice());
        assert_eq!(mid.part, None);
    }

    /// The same, part-scoped: a part-width action is not an endpoint for the
    /// parts it does not address, and anchoring it on itself would fade it in
    /// as if it commanded the whole robot — and leave the OTHER part's next
    /// action fading out of this part's setpoint.
    #[test]
    fn with_nothing_commanded_yet_a_part_scoped_action_has_no_endpoint() {
        let to = part_action("left", &[1.0; 7]);
        assert!(blend_step(None, &to, 0.5, Interp::Linear).is_none());
    }

    /// A gripper-only action with no anchor is the deliberate asymmetry: it
    /// commands ONE scalar and fabricates no arm row for anyone, so with
    /// nothing to fade from it is its own endpoint whether or not it names a
    /// part. Narrowing the rule to "no anchor and no tag" would silently
    /// drop a commanded grip in exactly the episodes that have no anchor —
    /// an agent-invited one whose caller only ever got `Noop`s (FSM.md E24).
    #[test]
    fn with_nothing_commanded_yet_a_part_scoped_gripper_is_its_own_endpoint() {
        let to = OwnedAction {
            values: SmallVec::new(),
            gripper: Some(0.04),
            part: Some(std::sync::Arc::from("left")),
        };
        let mid = blend_step(None, &to, 0.5, Interp::Linear)
            .expect("one scalar, nothing to fade from, nothing fabricated");
        assert_eq!(mid.gripper, Some(0.04));
        assert_eq!(mid.part.as_deref(), Some("left"));
    }

    /// A gripper-only action has no arm row by construction, not by
    /// truncation: it must survive the cross-fade window as itself rather
    /// than being refused as a dims mismatch, which would hold the gate and
    /// silently drop the commanded grip.
    #[test]
    fn a_gripper_only_action_survives_the_blend_window() {
        let from = OwnedAction {
            values: SmallVec::from_slice(&[0.0, 0.0, 0.0]),
            gripper: Some(0.0),
            part: None,
        };
        let to = OwnedAction {
            values: SmallVec::new(),
            gripper: Some(0.04),
            part: None,
        };
        let mid = blend_step(Some(&from), &to, 0.5, Interp::Linear)
            .expect("gripper-only is not a dims mismatch");
        assert!(mid.values.is_empty(), "the arm holds: no values to write");
        assert!((mid.gripper.unwrap() - 0.02).abs() < 1e-12);

        let end = blend_step(Some(&from), &to, 1.0, Interp::Linear).unwrap();
        assert_eq!(end.gripper, Some(0.04));
    }

    /// The pair the width rule does NOT backstop, decided rather than
    /// stumbled into: an UNTAGGED anchor of any width against a part-tagged
    /// gripper-only target. The anchor commands every part (FSM.md §4),
    /// including the ONE gripper channel an `OwnedAction` carries, so it
    /// does command the target's scope; and with no arm rows to line up
    /// there is nothing for the width rule to refuse. The same-width arm-row
    /// pair below IS refused — on width, because the gate holds no part
    /// layout and cannot slice a whole-robot anchor down to one part's rows.
    #[test]
    fn a_part_tagged_gripper_only_action_fades_out_of_the_whole_robots_grip() {
        let from = OwnedAction {
            gripper: Some(0.0),
            ..action(&[0.0; 14])
        };
        let to = OwnedAction {
            values: SmallVec::new(),
            gripper: Some(0.04),
            part: Some(std::sync::Arc::from("left")),
        };
        let mid = blend_step(Some(&from), &to, 0.5, Interp::Linear)
            .expect("a whole-robot point commands the part's grip too");
        assert!(mid.values.is_empty(), "the arm holds: no values to write");
        assert!((mid.gripper.unwrap() - 0.02).abs() < 1e-12);
        assert_eq!(mid.part.as_deref(), Some("left"));

        let arm_row = part_action("left", &[1.0; 7]);
        assert!(
            blend_step(Some(&from), &arm_row, 0.5, Interp::Linear).is_none(),
            "the same anchor cannot be sliced down to the left arm's rows"
        );
    }

    /// Surviving the window does not exempt a gripper-only action from the
    /// scope rule. It takes no values from the anchor, but it DOES take the
    /// anchor's gripper as its starting point — so one part's gripper faded
    /// into another part's grip target is the same fabricated trajectory the
    /// arm rows are refused for. `waddle_types::flatten_action` builds
    /// exactly this shape from a part-scoped noop + gripper.
    #[test]
    fn a_gripper_only_action_never_cross_fades_out_of_another_parts_gripper() {
        let from = OwnedAction {
            gripper: Some(0.0),
            ..part_action("left", &[0.0; 7])
        };
        let to = OwnedAction {
            values: SmallVec::new(),
            gripper: Some(0.04),
            part: Some(std::sync::Arc::from("right")),
        };
        assert!(
            blend_step(Some(&from), &to, 0.5, Interp::Linear).is_none(),
            "the right gripper was commanded a trajectory starting at the LEFT gripper's \
             last commanded position"
        );
    }
}
