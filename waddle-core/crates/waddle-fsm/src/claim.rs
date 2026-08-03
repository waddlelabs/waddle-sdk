//! The active claim (FSM.md §2). One active claim in v0 (one episode per
//! session, N18). A claim SURVIVES retake (row C5): the successor episode is
//! born claimed under it.

use waddle_types::{ActorRef, ClaimId, ClientId, Provenance, ProvenanceTag};

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct ActiveClaim {
    pub id: ClaimId,
    /// Registered intervention-source name (e.g. "teleop", "leader_arm").
    pub source: String,
    /// WHO holds the claim, whole: kind AND the identity the granting side
    /// stamped (`ActorRef::of_kind` for a local grant that has none). Carried
    /// verbatim onto every claim emission and every provenance tag minted
    /// under this claim — a recording that cannot name its driver cannot be
    /// audited, judged, or trained on with any confidence about who acted.
    pub actor: ActorRef,
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

    /// The provenance every action driven under this claim carries: the
    /// actor's kind decides the vocabulary ([`Provenance::for_claim`]), the
    /// actor itself rides along, and a self-initiated (clutch) claim carries
    /// the `bypass_approval` stamp. The ONE source of claimed-window
    /// provenance — the reducer's gate plan and the conformance target both
    /// call this rather than re-deriving it.
    #[must_use]
    pub fn provenance(&self) -> ProvenanceTag {
        ProvenanceTag {
            provenance: Provenance::for_claim(self.actor.kind, &self.source),
            actor: Some(self.actor.clone()),
            bypass_approval: self.self_initiated,
        }
    }
}
