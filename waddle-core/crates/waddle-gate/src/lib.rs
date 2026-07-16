//! waddle-gate — the single point where Waddle touches the integrator's
//! loop.
//!
//! `Gate::gate()` is synchronous and, in passthrough, wait-free and
//! allocation-free: one plan load (arc-swap), one clock read, one ring push.
//! Latency added to the integrator's loop is a bug of the highest severity.
//!
//! Structure:
//! - the *plan* (rare writes) lives in [`GateShared`] and is written only by
//!   the runtime's FSM reducer;
//! - the *stream* (intervention actions, high rate) arrives on an SPSC ring
//!   fed by the media intake, reordered by the [`jitter`] buffer;
//! - every returned action is provenance-tagged at write time and recorded
//!   onto the gate-record ring for the sidecar.

pub mod blend;
pub mod dualwrite;
pub mod gate;
pub mod jitter;
pub mod plan;
pub mod record;
pub mod stats;

pub use dualwrite::{DivergenceDetector, DualWriteVerdict};
pub use gate::{Gate, GateOutput, OwnedAction};
pub use jitter::{ChunkMeta, JitterBuffer, StreamChannel, TimedAction};
pub use plan::{BlendSchedule, GatePlan, PlanMode};
pub use record::{GateDecision, GateRecord};
pub use stats::TickStats;
