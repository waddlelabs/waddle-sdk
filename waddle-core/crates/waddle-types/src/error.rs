//! The waddle-types error taxonomy: every `TryFrom<pb::_>` failure names what
//! was wrong and where.

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum TypesError {
    #[error("missing required field: {0}")]
    MissingField(&'static str),

    #[error("invalid enum value {value} for {field}")]
    InvalidEnum { field: &'static str, value: i32 },

    #[error("frame_id must be non-empty (untagged geometry is rejected, never defaulted)")]
    EmptyFrame,

    #[error("{field} must be declared, never guessed (is UNSPECIFIED)")]
    MustDeclare { field: &'static str },

    #[error("dimension mismatch: expected {expected}, got {got}")]
    DimensionMismatch { expected: usize, got: usize },

    #[error("Composite nesting depth is pinned to 1 in v0 (part {part:?} is itself composite)")]
    CompositeDepth { part: String },

    #[error("duplicate composite part name {0:?}")]
    DuplicatePart(String),

    #[error("action references unknown part {0:?}")]
    UnknownPart(String),

    #[error("part {0:?} missing from composite action")]
    MissingPart(String),

    #[error("opaque action spaces are monitor-only and cannot be flattened for execution")]
    OpaqueNotExecutable,

    #[error("NOOP markers are gate outputs, not executable actions")]
    NoopNotExecutable,

    #[error("invalid {field}: {reason}")]
    InvalidValue {
        field: &'static str,
        reason: &'static str,
    },
}
