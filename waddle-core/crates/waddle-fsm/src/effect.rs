//! Effects: what the pure machine asks its runtime to do. The runtime (or
//! conformance target) interprets these and feeds completions back in as
//! events.

use waddle_types::{
    ClaimId, ClientId, EpisodeId, GateMode, LeaseId, MonoNs, ResetVerificationMode, Verb,
    pb::v0 as pb,
};

use crate::event::TimerId;

/// What a minted lease token is for, and what completes when it applies.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct PendingLeaseOp {
    pub op: LeaseOpKind,
    pub then: AfterLease,
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub enum LeaseOpKind {
    Acquire { client: ClientId },
    Handoff { from: LeaseId, to: ClientId },
}

/// What the session does once the pending lease operation applies.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AfterLease {
    /// The integrator loop's initial acquire.
    InitialAcquire,
    /// Engage completes: gate flips to INTERVENTION, phase → SETTLE.
    EngageComplete,
    /// Release completes: gate flips to PASSTHROUGH, phase → RUNNING,
    /// claim released.
    ReleaseComplete,
}

#[derive(Debug)]
pub enum Effect {
    /// Consumed by the gate (runtime stores it into the gate plan).
    SetGateMode(GateMode),
    /// Invoke a declared verb (via the runtime's verb-dispatch thread).
    /// The FSM only ever requests non-send verbs.
    RequestVerb(Verb),
    ArmTimer {
        id: TimerId,
        deadline: MonoNs,
    },
    CancelTimer {
        id: TimerId,
    },
    /// Retake: open the successor episode under the still-held claim
    /// (N2/N12/N18). The runtime answers with `SessionEvent::EpisodeOpen`.
    OpenSuccessor {
        predecessor: EpisodeId,
        successor: EpisodeId,
        claim: ClaimId,
        born_claimed: bool,
        mode: ResetVerificationMode,
    },
    /// Mint a fresh lease token (the FSM never mints). The runtime answers
    /// with `SessionEvent::LeaseTokenMinted`.
    MintLeaseToken(PendingLeaseOp),
    /// Release re-primes the policy on fresh observations before pass
    /// resumes (FSM.md §5).
    ReprimePolicy,
    /// The permanent `reset_unverified` flag was set retroactively (N12).
    SetResetUnverified {
        episode: EpisodeId,
    },
    /// An `EpisodeEvent` for the session event stream (sidecar, recorder,
    /// control plane).
    Emit(pb::EpisodeEvent),
}
