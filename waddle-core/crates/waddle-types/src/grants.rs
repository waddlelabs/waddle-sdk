//! Grants: the permissions an integrator extends to Waddle, with guarantees.

use crate::error::TypesError;
use crate::pb::v0 as pb;
use crate::space::SpaceKind;

/// The five-verb control contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Verb {
    Send,
    Hold,
    Resume,
    Home,
    Estop,
}

impl Verb {
    pub fn from_pb(value: i32) -> Result<Self, TypesError> {
        match pb::Verb::try_from(value) {
            Ok(pb::Verb::Send) => Ok(Self::Send),
            Ok(pb::Verb::Hold) => Ok(Self::Hold),
            Ok(pb::Verb::Resume) => Ok(Self::Resume),
            Ok(pb::Verb::Home) => Ok(Self::Home),
            Ok(pb::Verb::Estop) => Ok(Self::Estop),
            Ok(pb::Verb::Unspecified) | Err(_) => Err(TypesError::InvalidEnum {
                field: "Verb",
                value,
            }),
        }
    }

    #[must_use]
    pub fn to_pb(self) -> pb::Verb {
        match self {
            Self::Send => pb::Verb::Send,
            Self::Hold => pb::Verb::Hold,
            Self::Resume => pb::Verb::Resume,
            Self::Home => pb::Verb::Home,
            Self::Estop => pb::Verb::Estop,
        }
    }
}

/// Where the single-writer lease is enforced (N7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum LeaseEnforcement {
    /// A broker/mux/proxy physically owns the only write path.
    Enforced,
    /// In-process callables: nothing physically stops the integrator's loop
    /// from writing during a takeover. Planner prefers HOLD_FIRST; dual-write
    /// detection runs during bypass (N14).
    Advisory,
}

impl LeaseEnforcement {
    pub fn from_pb(value: i32) -> Result<Self, TypesError> {
        match pb::LeaseEnforcement::try_from(value) {
            Ok(pb::LeaseEnforcement::Enforced) => Ok(Self::Enforced),
            Ok(pb::LeaseEnforcement::Advisory) => Ok(Self::Advisory),
            Ok(pb::LeaseEnforcement::Unspecified) | Err(_) => Err(TypesError::InvalidEnum {
                field: "LeaseEnforcement",
                value,
            }),
        }
    }
}

/// Live grant status (N6/N11). Demotion never interrupts an active lease.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GrantStatus {
    Active,
    Demoted,
    Revoked,
}

/// A declared permission with its guarantees.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Grant {
    pub verb: Verb,
    /// Populated only for `Verb::Send`.
    pub send_interfaces: Vec<SpaceKind>,
    pub declared_latency_bound_ns: Option<i64>,
    /// Meaningful for `Verb::Estop`: hardware chain vs software stop.
    pub hardware: bool,
}

impl TryFrom<&pb::Grant> for Grant {
    type Error = TypesError;

    fn try_from(g: &pb::Grant) -> Result<Self, Self::Error> {
        Ok(Self {
            verb: Verb::from_pb(g.verb)?,
            send_interfaces: g
                .send_interfaces
                .iter()
                .map(|v| SpaceKind::from_pb(*v))
                .collect::<Result<_, _>>()?,
            declared_latency_bound_ns: g.declared_latency_bound_ns,
            hardware: g.hardware,
        })
    }
}
