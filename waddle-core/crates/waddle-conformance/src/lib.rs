//! waddle-conformance — the behavioral-scenario runner.
//!
//! Implements exactly the schema in
//! `waddle-protocol/conformance/scenario-format.md` and drives the scenarios
//! in `waddle-protocol/fixtures/behaviors/` against the reference
//! implementation (`waddle-fsm` composed with `waddle-gate` for the gate
//! target). If this runner and scenario-format.md disagree, the document
//! wins and this runner is wrong.
//!
//! Determinism: time is virtual (`advance_ns`), lease tokens are minted as
//! `lease-1`, `lease-2`, … in effect order, and no code path here reads an
//! OS clock or randomness.

pub mod emissions;
pub mod matching;
pub mod runner;
pub mod scenario;
pub mod target;

pub use emissions::{Codec, EmissionEntry};
pub use runner::{Report, run_dir, run_scenario_file};
pub use scenario::Scenario;

/// Errors raised while loading or driving a scenario (distinct from a
/// scenario *failing*, which is reported via [`Report`]).
#[derive(Debug, thiserror::Error)]
pub enum ConformanceError {
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("json: {0}")]
    Json(#[from] serde_json::Error),
    #[error("types: {0}")]
    Types(#[from] waddle_types::TypesError),
    #[error("scenario: {0}")]
    Scenario(String),
}

pub(crate) fn scenario_err(msg: impl Into<String>) -> ConformanceError {
    ConformanceError::Scenario(msg.into())
}
