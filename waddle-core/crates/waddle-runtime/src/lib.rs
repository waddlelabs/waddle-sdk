//! waddle-runtime — the composition root (stub while lower crates land).
//!
//! The Session object and thread wiring land in M7; the verb-dispatch layer
//! is here already.

pub mod verbs;

pub use verbs::{ControlRegistry, EstopDecl, SendVerb, UnitVerb, VerbDispatch, VerbError};
