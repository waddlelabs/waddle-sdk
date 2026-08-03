//! Provenance: per-action origin, written at gate time.

use crate::error::TypesError;
use crate::pb::v0 as pb;

/// Protocol-wide actor vocabulary (N17): TELEOPERATOR is a Waddle work-plane
/// human; SITE_OPERATOR is a customer-side human at the cell. Unqualified
/// "operator" is banned in normative text.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ActorKind {
    Teleoperator,
    SiteOperator,
    Agent,
    Policy,
    System,
    Custom,
}

impl ActorKind {
    #[must_use]
    pub fn to_pb(self) -> pb::ActorKind {
        match self {
            Self::Teleoperator => pb::ActorKind::Teleoperator,
            Self::SiteOperator => pb::ActorKind::SiteOperator,
            Self::Agent => pb::ActorKind::Agent,
            Self::Policy => pb::ActorKind::Policy,
            Self::System => pb::ActorKind::System,
            Self::Custom => pb::ActorKind::Custom,
        }
    }

    pub fn from_pb(value: i32) -> Result<Self, TypesError> {
        match pb::ActorKind::try_from(value) {
            Ok(pb::ActorKind::Teleoperator) => Ok(Self::Teleoperator),
            Ok(pb::ActorKind::SiteOperator) => Ok(Self::SiteOperator),
            Ok(pb::ActorKind::Agent) => Ok(Self::Agent),
            Ok(pb::ActorKind::Policy) => Ok(Self::Policy),
            Ok(pb::ActorKind::System) => Ok(Self::System),
            Ok(pb::ActorKind::Custom) => Ok(Self::Custom),
            Ok(pb::ActorKind::Unspecified) | Err(_) => Err(TypesError::InvalidEnum {
                field: "ActorKind",
                value,
            }),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActorRef {
    pub kind: ActorKind,
    /// Stable identity (for humans: a server-stamped participant id, never a
    /// client-forgeable string).
    pub id: String,
    pub display_name: String,
}

impl ActorRef {
    /// An actor known only by kind: no server-stamped id, no display name.
    /// The shape a LOCAL actor has — a leader-arm clutch, a test's direct
    /// grant — where nothing upstream minted an identity. Remote actors
    /// always arrive as a full `pb::ActorRef` and must be carried whole
    /// (`TryFrom<&pb::ActorRef>`); this is not a substitute for that.
    #[must_use]
    pub fn of_kind(kind: ActorKind) -> Self {
        Self {
            kind,
            id: String::new(),
            display_name: String::new(),
        }
    }

    #[must_use]
    pub fn to_pb(&self) -> pb::ActorRef {
        pb::ActorRef {
            kind: self.kind.to_pb() as i32,
            id: self.id.clone(),
            display_name: self.display_name.clone(),
        }
    }
}

impl TryFrom<&pb::ActorRef> for ActorRef {
    type Error = TypesError;

    fn try_from(a: &pb::ActorRef) -> Result<Self, Self::Error> {
        Ok(Self {
            kind: ActorKind::from_pb(a.kind)?,
            id: a.id.clone(),
            display_name: a.display_name.clone(),
        })
    }
}

/// Per-action origin: `policy | teleop | agent | custom:<name>`.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Provenance {
    Policy,
    Teleop,
    Agent,
    Custom(String),
}

impl Provenance {
    /// The provenance of an action driven by a claim held by `actor`. THE
    /// mapping — every producer of a claimed/bypass-window provenance tag
    /// (the gate plan the reducer projects, the sidecar's provenance spans,
    /// the conformance target) derives it here, so a recording's spans and
    /// its per-action tags can never disagree about who was driving.
    /// `source` is the claim's registered intervention-source name, used
    /// only for the actor kinds this vocabulary has no dedicated value for.
    #[must_use]
    pub fn for_claim(actor: ActorKind, source: &str) -> Self {
        match actor {
            ActorKind::Teleoperator => Self::Teleop,
            ActorKind::Agent => Self::Agent,
            ActorKind::Policy => Self::Policy,
            // SITE_OPERATOR is deliberately NOT teleop: a customer-side
            // human at the cell is a different actor from a Waddle
            // work-plane teleoperator (N17), and the corpus must be able to
            // tell them apart.
            ActorKind::SiteOperator | ActorKind::System | ActorKind::Custom => {
                Self::Custom(source.to_owned())
            }
        }
    }
}

impl std::fmt::Display for Provenance {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Policy => f.write_str("policy"),
            Self::Teleop => f.write_str("teleop"),
            Self::Agent => f.write_str("agent"),
            Self::Custom(name) => write!(f, "custom:{name}"),
        }
    }
}

/// The full origin tag. `bypass_approval` generalizes the production
/// operator-initiated stamp: a directly-initiated human action may bypass a
/// motion-approval gate, but NEVER the envelope, the lease, or the e-stop.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ProvenanceTag {
    pub provenance: Provenance,
    pub actor: Option<ActorRef>,
    pub bypass_approval: bool,
}

impl ProvenanceTag {
    #[must_use]
    pub fn policy() -> Self {
        Self {
            provenance: Provenance::Policy,
            actor: None,
            bypass_approval: false,
        }
    }

    /// The exact inverse of `TryFrom<&pb::ProvenanceTag>`.
    #[must_use]
    pub fn to_pb(&self) -> pb::ProvenanceTag {
        let (kind, custom_name) = match &self.provenance {
            Provenance::Policy => (pb::ProvenanceKind::Policy, String::new()),
            Provenance::Teleop => (pb::ProvenanceKind::Teleop, String::new()),
            Provenance::Agent => (pb::ProvenanceKind::Agent, String::new()),
            Provenance::Custom(name) => (pb::ProvenanceKind::Custom, name.clone()),
        };
        pb::ProvenanceTag {
            kind: kind as i32,
            custom_name,
            actor: self.actor.as_ref().map(ActorRef::to_pb),
            bypass_approval: self.bypass_approval,
        }
    }
}

impl TryFrom<&pb::ProvenanceTag> for ProvenanceTag {
    type Error = TypesError;

    fn try_from(t: &pb::ProvenanceTag) -> Result<Self, Self::Error> {
        let provenance = match pb::ProvenanceKind::try_from(t.kind) {
            Ok(pb::ProvenanceKind::Policy) => Provenance::Policy,
            Ok(pb::ProvenanceKind::Teleop) => Provenance::Teleop,
            Ok(pb::ProvenanceKind::Agent) => Provenance::Agent,
            Ok(pb::ProvenanceKind::Custom) => {
                if t.custom_name.is_empty() {
                    return Err(TypesError::MissingField("ProvenanceTag.custom_name"));
                }
                Provenance::Custom(t.custom_name.clone())
            }
            Ok(pb::ProvenanceKind::Unspecified) | Err(_) => {
                return Err(TypesError::InvalidEnum {
                    field: "ProvenanceKind",
                    value: t.kind,
                });
            }
        };
        Ok(Self {
            provenance,
            actor: t.actor.as_ref().map(ActorRef::try_from).transpose()?,
            bypass_approval: t.bypass_approval,
        })
    }
}
