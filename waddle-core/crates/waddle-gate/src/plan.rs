//! The gate plan: what the gate does with the next action. Written only by
//! the runtime's FSM reducer (on `Effect::SetGateMode`); read wait-free on
//! every tick.

use waddle_types::{Interp, MonoNs, ProvenanceTag};

/// A cross-fade window (HandoffPolicy::Immediate).
#[derive(Debug, Clone, PartialEq)]
pub struct BlendSchedule {
    pub start: MonoNs,
    pub blend_ns: i64,
    pub interp: Interp,
}

impl BlendSchedule {
    /// Blend progress in [0, 1] at time `now`.
    #[must_use]
    pub fn progress(&self, now: MonoNs) -> f32 {
        if self.blend_ns <= 0 {
            return 1.0;
        }
        #[allow(clippy::cast_precision_loss)]
        let p = (now.0 - self.start.0) as f64 / self.blend_ns as f64;
        p.clamp(0.0, 1.0) as f32
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum PlanMode {
    /// Nominal: the action passes through unmodified.
    Passthrough,
    /// A claim holds the episode: the gate substitutes intervention actions,
    /// optionally cross-fading in over `blend`.
    Claimed {
        provenance: ProvenanceTag,
        blend: Option<BlendSchedule>,
    },
    /// Claimed-while-stalled: the runtime pump drives `send` directly; the
    /// integrator's loop receives NOOP markers and MUST NOT dispatch them.
    Bypass { provenance: ProvenanceTag },
    /// A remote actor is performing a scene reset through the SDK (flag
    /// `waddle.v0.reset.remote`, gate mode RESET): the reset claimant holds
    /// the lease and the SDK drives `send` on its behalf from its own
    /// thread. A caller ticking `gate()` on this now-stale handle is a
    /// spectator (the stale-handle contract) — same shape as `Bypass`, distinct so the
    /// reducer can render `NoopReason::RESET_ACTIVE` instead of
    /// `BYPASS_ACTIVE`.
    Reset { provenance: ProvenanceTag },
    /// A hold is active (HOLD_FIRST engage, tripwire hold).
    Held,
}

#[derive(Debug, Clone, PartialEq)]
pub struct GatePlan {
    pub mode: PlanMode,
    pub since: MonoNs,
}

impl GatePlan {
    #[must_use]
    pub fn passthrough(since: MonoNs) -> Self {
        Self {
            mode: PlanMode::Passthrough,
            since,
        }
    }
}
