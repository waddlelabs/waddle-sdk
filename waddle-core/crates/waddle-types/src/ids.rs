//! Opaque string identifiers.
//!
//! Ids are cheap-to-clone (`Arc<str>`) and deliberately opaque: internal id
//! schemes (e.g. `int-<hex>` episode ids) flow through unmodified.

use std::sync::Arc;

macro_rules! string_id {
    ($(#[$meta:meta])* $name:ident) => {
        $(#[$meta])*
        #[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        #[cfg_attr(feature = "serde", serde(transparent))]
        pub struct $name(Arc<str>);

        impl $name {
            #[must_use]
            pub fn new(s: impl AsRef<str>) -> Self {
                Self(Arc::from(s.as_ref()))
            }

            #[must_use]
            pub fn as_str(&self) -> &str {
                &self.0
            }

            #[must_use]
            pub fn is_empty(&self) -> bool {
                self.0.is_empty()
            }
        }

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str(&self.0)
            }
        }

        impl From<&str> for $name {
            fn from(s: &str) -> Self {
                Self::new(s)
            }
        }

        impl From<String> for $name {
            fn from(s: String) -> Self {
                Self(Arc::from(s))
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self(Arc::from(""))
            }
        }
    };
}

string_id!(
    /// One rollout attempt (the unit of the sidecar).
    EpisodeId
);
string_id!(SessionId);
string_id!(
    /// Orchestration-level assignment of an episode to an actor.
    ClaimId
);
string_id!(
    /// The actuation-level single-writer token. Freshly minted per
    /// grant/handoff; minted by the runtime, never by the FSM.
    LeaseId
);
string_id!(
    /// A stable writer identity (the lease is held by a client).
    ClientId
);
string_id!(RobotId);
string_id!(CellId);
string_id!(
    /// An action/observation stream source (policy identity, teleop rig, …).
    SourceId
);

/// A named coordinate frame. Non-empty by construction: untagged geometry is
/// how misaligned data corrupts a corpus silently.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct FrameId(Arc<str>);

impl FrameId {
    pub fn new(s: impl AsRef<str>) -> Result<Self, crate::TypesError> {
        let s = s.as_ref();
        if s.is_empty() {
            return Err(crate::TypesError::EmptyFrame);
        }
        Ok(Self(Arc::from(s)))
    }

    #[must_use]
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl std::fmt::Display for FrameId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
