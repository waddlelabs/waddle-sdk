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

/// Blend one step: `from * (1-w) + to * w` componentwise. Lengths must
/// match (validated upstream by the flattener).
#[must_use]
pub fn blend_step(from: &OwnedAction, to: &OwnedAction, t: f32, interp: Interp) -> OwnedAction {
    let w = f64::from(weight(interp, t));
    let values = from
        .values
        .iter()
        .zip(to.values.iter())
        .map(|(a, b)| a * (1.0 - w) + b * w)
        .collect();
    OwnedAction {
        values,
        gripper: match (from.gripper, to.gripper) {
            (Some(a), Some(b)) => Some(a * (1.0 - w) + b * w),
            (_, b @ Some(_)) => b,
            (a, None) => a,
        },
    }
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

            let at0 = blend_step(&from, &to, 0.0, Interp::Linear);
            let at1 = blend_step(&from, &to, 1.0, Interp::Linear);
            prop_assert_eq!(at0.values.as_slice(), from.values.as_slice());
            prop_assert_eq!(at1.values.as_slice(), to.values.as_slice());

            let mid = blend_step(&from, &to, t, Interp::Linear);
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
            let a = blend_step(&from, &to, lo, Interp::Linear);
            let b = blend_step(&from, &to, hi, Interp::Linear);
            prop_assert!(a.values[0] <= b.values[0] + 1e-9);
        }
    }

    #[test]
    fn hold_switches_only_at_the_end() {
        let from = action(&[0.0]);
        let to = action(&[1.0]);
        assert_eq!(blend_step(&from, &to, 0.99, Interp::Hold).values[0], 0.0);
        assert_eq!(blend_step(&from, &to, 1.0, Interp::Hold).values[0], 1.0);
    }
}
