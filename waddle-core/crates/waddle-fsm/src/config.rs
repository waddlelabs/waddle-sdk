//! Static session configuration the machines plan against.

use waddle_types::{
    ActorKind, CellId, ClientId, EpisodeId, Grant, HandoffPolicy, LeaseEnforcement, RobotId,
    SessionId,
};

#[derive(Debug, Clone)]
pub struct SessionConfig {
    pub session_id: SessionId,
    pub robot_id: RobotId,
    pub cell_id: CellId,
    /// The integrator's loop identity — the nominal lease holder.
    pub loop_client: ClientId,
    pub enforcement: LeaseEnforcement,
    pub handoff: HandoffPolicy,
    pub grants: Vec<Grant>,
    /// Whether the declared action space contains a delta space: engage
    /// under `Immediate` degrades to `HoldFirst` (FSM.md §5, delta-space
    /// restriction).
    pub space_contains_delta: bool,
    /// Source name recorded for clutch-initiated (self-initiated) claims.
    pub clutch_source: String,
    /// Actor kind recorded for clutch-initiated (self-initiated) claims
    /// (N17). The FSM's own default (`SiteOperator`) is fixture-stable and
    /// deliberately not the honest one — `waddle-runtime`'s `SessionBuilder`
    /// overrides it to `Teleoperator` at build time, since a clutch edge on
    /// the media plane is our teleoperators' takeover path.
    pub clutch_actor: ActorKind,
    /// Heartbeat staleness window after a partition before the local
    /// tripwire requests HOLD (FSM.md §8).
    pub heartbeat_timeout_ns: i64,
    /// How long an engage may sit incomplete before retake becomes legal
    /// from ENGAGE (FSM.md §4, I2).
    pub engage_timeout_ns: i64,
    /// Consecutive bad proxy samples before a grant demotes (N11).
    pub demote_after: u32,
    /// Consecutive good proxy samples (below the hysteresis band) before a
    /// demoted grant re-promotes without a safe-window measurement (N11).
    pub promote_after: u32,
    /// A "good" sample must be below `bound * hysteresis_ratio` — a signal
    /// hovering at the bound must not flap the planner (N11).
    pub hysteresis_ratio: f64,
}

impl SessionConfig {
    /// A minimal config for tests and scenario setup.
    #[must_use]
    pub fn minimal(
        loop_client: &str,
        handoff: HandoffPolicy,
        enforcement: LeaseEnforcement,
    ) -> Self {
        Self {
            session_id: SessionId::new("session-1"),
            robot_id: RobotId::new("robot-1"),
            cell_id: CellId::new("cell-1"),
            loop_client: ClientId::new(loop_client),
            enforcement,
            handoff,
            grants: Vec::new(),
            space_contains_delta: false,
            clutch_source: "custom".to_owned(),
            clutch_actor: ActorKind::SiteOperator,
            heartbeat_timeout_ns: 2_000_000_000,
            engage_timeout_ns: 10_000_000_000,
            demote_after: 1,
            promote_after: 3,
            hysteresis_ratio: 0.8,
        }
    }
}

/// Successor-episode parameters produced by a retake (interpreted by the
/// runtime into a subsequent `EpisodeOpen`).
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct SuccessorSpec {
    pub predecessor: EpisodeId,
    pub successor: EpisodeId,
}
