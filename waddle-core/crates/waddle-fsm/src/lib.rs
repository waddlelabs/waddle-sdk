//! waddle-fsm — the episode/claim/lease state machines as pure transition
//! functions. **This crate is the behavioral conformance target**: the
//! guard tables in `waddle-protocol/docs/FSM.md` are implemented here, and
//! the scenarios in `waddle-protocol/fixtures/behaviors/` pin the behavior.
//!
//! Purity rules (structural, not aspirational):
//! - no I/O, no clocks (time arrives on events; timers are effects),
//! - no randomness or token minting (lease tokens and successor ids arrive on
//!   events; the FSM asks for them via effects),
//! - `step(state, event)` is deterministic and side-effect-free.
//!
//! The runtime (or the conformance runner) interprets [`Effect`]s and feeds
//! resulting completions back in as events — e.g. `Effect::MintLeaseToken`
//! is answered by [`SessionEvent::LeaseTokenMinted`].

pub mod claim;
pub mod config;
pub mod effect;
pub mod emit;
pub mod episode;
pub mod event;
pub mod granthealth;
pub mod lease;
pub mod session;

pub use claim::ActiveClaim;
pub use config::SessionConfig;
pub use effect::{AfterLease, Effect, HandbackThen, PendingLeaseOp};
pub use episode::{EpisodeState, Phase, ResetWindowState};
pub use event::{
    AgentInvite, GrantChangeDirective, MarkKind, ProxySample, RejectReason, SessionEvent, TimerId,
    WindowSpec,
};
pub use granthealth::GrantHealthEntry;
pub use lease::{LeaseCmd, LeaseOutcome, LeaseState};
pub use session::{Rejected, SessionFsm, Step, step};
