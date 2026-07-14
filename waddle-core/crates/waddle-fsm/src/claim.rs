//! The active claim (FSM.md §2). One active claim in v0 (one episode per
//! session, N18). A claim SURVIVES retake (row C5): the successor episode is
//! born claimed under it.

use waddle_types::{ActorKind, ClaimId, ClientId};

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct ActiveClaim {
    pub id: ClaimId,
    /// Registered intervention-source name (e.g. "teleop", "leader_arm").
    pub source: String,
    pub actor: ActorKind,
    /// Engagement-initiated (clutch) claims: requested and granted in one
    /// step; the platform records the intervention rather than fighting it.
    pub self_initiated: bool,
}

impl ActiveClaim {
    /// The actuation identity the intervenor's lease is held under.
    #[must_use]
    pub fn client(&self) -> ClientId {
        ClientId::new(&self.source)
    }
}
