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
    /// A remote reset claimant's lease routed in (E20): gate flips to RESET,
    /// the window is marked ENGAGED.
    ResetEngageComplete,
    /// The reset window's lease handed back to the loop client (E21/E22).
    /// Deferred-apply: only after the lease is back does the gate drop to
    /// PASSTHROUGH, the claim release (C7), and `then` apply the result — so
    /// the next `EpisodeOpen` finds the lease non-vacant is impossible.
    ResetHandback { then: HandbackThen },
}

/// What applies after a reset-window lease handback completes (E21/E22).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum HandbackThen {
    /// PRE window: apply the reset result as the pre-reset pipeline (E2–E5).
    ApplyPreResult { ok: bool, verified: Option<bool> },
    /// POST window: apply the reset result as the post-reset pipeline
    /// (E15/E16).
    ApplyPostResult { ok: bool },
    /// The window timed out (E22): abort (pre) / pinned + failed (post).
    TimeoutClose,
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
    /// The permanent `post_reset_failed` flag was set (E16/E17): post-reset
    /// cleanup failed or was estopped. NEVER alters the pinned outcome.
    SetPostResetFailed {
        episode: EpisodeId,
    },
    /// Run the declared post-reset hook pipeline (E14, hook variant). The
    /// runtime answers with `SessionEvent::PostResetResult`.
    RunPostReset {
        episode: EpisodeId,
    },
    /// An `EpisodeEvent` for the session event stream (sidecar, recorder,
    /// control plane).
    Emit(Box<pb::EpisodeEvent>),
}
