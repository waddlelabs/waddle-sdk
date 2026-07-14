//! Verb-invocation requests flowing toward the integrator's declared verbs.

use std::sync::Arc;

use crate::action::ActionChunk;
use crate::grants::Verb;

/// A request to invoke one of the five declared verbs. Tripwires, the FSM,
/// and the control plane all speak this; only the runtime's verb-dispatch
/// thread ever executes it.
#[derive(Debug, Clone)]
pub enum VerbRequest {
    Hold,
    Resume,
    Home,
    Estop,
    Send { chunk: Arc<ActionChunk> },
}

impl VerbRequest {
    #[must_use]
    pub fn verb(&self) -> Verb {
        match self {
            Self::Hold => Verb::Hold,
            Self::Resume => Verb::Resume,
            Self::Home => Verb::Home,
            Self::Estop => Verb::Estop,
            Self::Send { .. } => Verb::Send,
        }
    }
}
