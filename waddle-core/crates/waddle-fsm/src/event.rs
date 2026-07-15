//! Inputs to the session machine. Time arrives here (`at`), never from a
//! clock. The variants mirror the scenario-format inject kinds plus the
//! completions the runtime feeds back for effects.

use waddle_types::{
    ActorKind, ClaimId, EpisodeId, LeaseId, MonoNs, ResetVerificationMode, TerminalOutcome, Verb,
    pb::v0 as pb,
};

/// Deterministic timer identities (armed/cancelled via effects).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TimerId {
    EngageTimeout,
    ChunkBoundaryCap,
    HeartbeatStale,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MarkKind {
    Start,
    EndSuccess,
    EndFailure,
    EndAbort,
    Retake,
}

impl MarkKind {
    #[must_use]
    pub fn to_pb(self) -> pb::MarkKind {
        match self {
            Self::Start => pb::MarkKind::Start,
            Self::EndSuccess => pb::MarkKind::EndSuccess,
            Self::EndFailure => pb::MarkKind::EndFailure,
            Self::EndAbort => pb::MarkKind::EndAbort,
            Self::Retake => pb::MarkKind::Retake,
        }
    }

    #[must_use]
    pub fn terminal_outcome(self) -> Option<TerminalOutcome> {
        match self {
            Self::EndSuccess => Some(TerminalOutcome::Success),
            Self::EndFailure => Some(TerminalOutcome::Failure),
            Self::EndAbort => Some(TerminalOutcome::Abort),
            Self::Start | Self::Retake => None,
        }
    }
}

/// Safely-measurable proxy signals (N11) as consumed by grant health.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ProxySample {
    pub control_rtt_ns: i64,
    pub gate_tick_p95_ns: i64,
    pub callback_dispatch_p95_ns: i64,
    pub host_load_1m: f64,
}

/// A server-directed grant change (rides `HeartbeatAck.grant_changes`).
#[derive(Debug, Clone, PartialEq)]
pub struct GrantChangeDirective {
    pub verb: Verb,
    pub to: waddle_types::GrantStatus,
    pub reason: String,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SessionEvent {
    EpisodeOpen {
        id: EpisodeId,
        verification: ResetVerificationMode,
        born_claimed: bool,
        parent: Option<EpisodeId>,
        at: MonoNs,
    },
    /// The reset pipeline reported. `verified` carries an inline
    /// verification outcome when the pipeline ran one.
    ResetResult {
        ok: bool,
        verified: Option<bool>,
        at: MonoNs,
    },
    /// A (possibly late/async) reset verification (N12).
    VerificationResult {
        verified: bool,
        invalidated_async: bool,
        at: MonoNs,
    },
    /// READY → RUNNING without modeling gate ticks.
    Start {
        at: MonoNs,
    },
    /// A caller-loop tick was observed (first one drives READY → RUNNING).
    GateTick {
        at: MonoNs,
    },
    ClaimRequested {
        id: ClaimId,
        source: String,
        actor: ActorKind,
        self_initiated: bool,
        at: MonoNs,
    },
    ClaimGranted {
        id: ClaimId,
        source: String,
        actor: ActorKind,
        self_initiated: bool,
        at: MonoNs,
    },
    ClaimReleased {
        id: ClaimId,
        at: MonoNs,
    },
    Engage {
        claim: ClaimId,
        at: MonoNs,
    },
    /// The executing chunk finished (CHUNK_BOUNDARY engage path).
    ChunkBoundaryReached {
        at: MonoNs,
    },
    Release {
        claim: ClaimId,
        at: MonoNs,
    },
    Retake {
        claim: ClaimId,
        initiator: ActorKind,
        successor: EpisodeId,
        at: MonoNs,
    },
    /// Local-source clutch edge (self-initiated claims).
    Clutch {
        engaged: bool,
        at: MonoNs,
    },
    VerbResult {
        verb: Verb,
        ok: bool,
        fault: Option<pb::FaultKind>,
        at: MonoNs,
    },
    /// Completion of `Effect::MintLeaseToken`.
    LeaseTokenMinted {
        minted: LeaseId,
        at: MonoNs,
    },
    Estop {
        at: MonoNs,
    },
    Terminate {
        outcome: TerminalOutcome,
        reason: String,
        at: MonoNs,
    },
    /// An episode judgment arrived (attach-only; termination is driven by
    /// `Terminate` / marks / directives).
    JudgeResult {
        judge_id: String,
        passed: Option<bool>,
        at: MonoNs,
    },
    Mark {
        kind: MarkKind,
        at: MonoNs,
    },
    ProxySignals {
        sample: ProxySample,
        at: MonoNs,
    },
    HeartbeatAck {
        grant_changes: Vec<GrantChangeDirective>,
        at: MonoNs,
    },
    PartitionStart {
        at: MonoNs,
    },
    PartitionEnd {
        at: MonoNs,
    },
    TimerFired {
        id: TimerId,
        at: MonoNs,
    },
    /// The integrator's loop stalled while a claim is active: no gate tick
    /// within the stall threshold. Detected by the runtime pump / gate
    /// harness; the FSM owns the resulting BYPASS transition (FSM.md §6).
    StallDetected {
        at: MonoNs,
    },
    /// Caller ticks resumed during BYPASS.
    TicksResumed {
        at: MonoNs,
    },
    /// Dual-write divergence detected by the gate during advisory-lease
    /// bypass (N14).
    DualWrite {
        divergence_metric: f64,
        window_ns: i64,
        /// Reference to the persisted divergence trace (sidecar incident
        /// clip id).
        trace_ref: String,
        at: MonoNs,
    },
    /// Media intake dropped a teleop action whose flattened width didn't
    /// match the declared action space (Bug 2: action-space validation).
    /// A diagnostic emission — records `Fault{VALIDATION_ERROR}` — never a
    /// state transition; the intake thread already deduplicates this to
    /// once per claim window before injecting it.
    InterventionRejected {
        dims_got: usize,
        dims_want: usize,
        at: MonoNs,
    },
}

impl SessionEvent {
    /// The timestamp carried on the event.
    #[must_use]
    pub fn at(&self) -> MonoNs {
        match self {
            Self::EpisodeOpen { at, .. }
            | Self::ResetResult { at, .. }
            | Self::VerificationResult { at, .. }
            | Self::Start { at }
            | Self::GateTick { at }
            | Self::ClaimRequested { at, .. }
            | Self::ClaimGranted { at, .. }
            | Self::ClaimReleased { at, .. }
            | Self::Engage { at, .. }
            | Self::ChunkBoundaryReached { at }
            | Self::Release { at, .. }
            | Self::Retake { at, .. }
            | Self::Clutch { at, .. }
            | Self::VerbResult { at, .. }
            | Self::LeaseTokenMinted { at, .. }
            | Self::Estop { at }
            | Self::Terminate { at, .. }
            | Self::JudgeResult { at, .. }
            | Self::Mark { at, .. }
            | Self::ProxySignals { at, .. }
            | Self::HeartbeatAck { at, .. }
            | Self::PartitionStart { at }
            | Self::PartitionEnd { at }
            | Self::TimerFired { at, .. }
            | Self::StallDetected { at }
            | Self::TicksResumed { at }
            | Self::DualWrite { at, .. }
            | Self::InterventionRejected { at, .. } => *at,
        }
    }
}
