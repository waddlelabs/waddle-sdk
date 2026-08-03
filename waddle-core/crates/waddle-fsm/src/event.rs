//! Inputs to the session machine. Time arrives here (`at`), never from a
//! clock. The variants mirror the scenario-format inject kinds plus the
//! completions the runtime feeds back for effects.

use waddle_types::{
    ActorKind, ActorRef, ClaimId, EpisodeId, LeaseId, MonoNs, ResetVerificationMode,
    TerminalOutcome, Verb, pb::v0 as pb,
};

/// Deterministic timer identities (armed/cancelled via effects).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TimerId {
    EngageTimeout,
    ChunkBoundaryCap,
    HeartbeatStale,
    /// A remote reset window's deadline (flag `waddle.v0.reset.remote`, E19);
    /// firing drives E22 (window not completed in time).
    ResetWindowTimeout,
    /// An agent invite's deadline (flag `waddle.v0.agent`, E23); firing
    /// drives E25 (no agent claim engaged in time). A stale expiry racing
    /// the cancellation is discarded (FSM.md §1.5).
    AgentInviteTimeout,
}

/// A declared remote reset window (flag `waddle.v0.reset.remote`). Carried on
/// `EpisodeOpen` for the PRE window (opened immediately) and stashed for the
/// POST window (opened at E14).
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct WindowSpec {
    /// The actor the plane expects to perform the reset (C6 admission).
    pub expected: ActorKind,
    pub prompt: String,
    /// Deadline offset from window open; arms `ResetWindowTimeout`.
    pub timeout_ns: i64,
}

/// A declared agent invite (flag `waddle.v0.agent`), carried on
/// `EpisodeOpen` (E23): the customer asked Waddle to drive this episode. The
/// invite is emitted to the plane and arms `AgentInviteTimeout`; the invited
/// agent then claims with the ordinary `Claim`/`Lease` machinery (C8
/// restricts admission, nothing else — FSM.md §1.5).
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct AgentInvite {
    /// The natural-language task for the invited agent.
    pub prompt: String,
    /// Deadline offset from episode open; arms `AgentInviteTimeout` (E25).
    pub timeout_ns: i64,
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
        /// A post-reset pipeline runs before TERMINAL (flag
        /// `waddle.v0.reset.phases`). `post_window` being set also implies a
        /// declared post-reset (a remote one); `post_reset` alone means a
        /// hook.
        post_reset: bool,
        /// A remote PRE reset window to open on entry to RESETTING (E19).
        pre_window: Option<WindowSpec>,
        /// A remote POST reset window, stashed for E14 to open.
        post_window: Option<WindowSpec>,
        /// An agent invite (flag `waddle.v0.agent`, E23): emitted at open,
        /// arming `AgentInviteTimeout`.
        agent_invite: Option<AgentInvite>,
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
        /// The claimant, whole (kind AND the granting side's stamped
        /// identity) — carried onto the claim emission verbatim. Use
        /// [`ActorRef::of_kind`] when the grant is local and has no identity.
        actor: ActorRef,
        self_initiated: bool,
        at: MonoNs,
    },
    ClaimGranted {
        id: ClaimId,
        source: String,
        /// The claimant, whole — see [`Self::ClaimRequested`].
        actor: ActorRef,
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
    /// An intake producer dropped an action whose flattened width didn't
    /// match the declared action space (the dims-validation contract,
    /// shared by the media-intake teleop path and the plane pump's
    /// `InterventionChunk` agent-chunk path). A
    /// diagnostic emission — records `Fault{VALIDATION_ERROR}` — never a
    /// state transition; the intake thread already deduplicates this to
    /// once per claim window before injecting it. `source` names the
    /// producer (e.g. "media-intake", "agent-chunk") so the emitted fault
    /// never misattributes which producer/space actually mismatched.
    InterventionRejected {
        dims_got: usize,
        dims_want: usize,
        source: &'static str,
        at: MonoNs,
    },
    /// The post-reset pipeline reported (flag `waddle.v0.reset.phases`, rows
    /// E15/E16). Legal only in POST_RESET.
    PostResetResult {
        ok: bool,
        detail: String,
        at: MonoNs,
    },
    /// A granted reset claim engages the open window (flag
    /// `waddle.v0.reset.remote`, E20): the lease routes to the claimant and
    /// the gate flips to RESET.
    ResetWindowEngage {
        claim: ClaimId,
        at: MonoNs,
    },
    /// The remote actor finished (flag `waddle.v0.reset.remote`, E21): lease
    /// hands back, the window closes, and the result applies as if from the
    /// pipeline.
    ResetWindowComplete {
        claim: ClaimId,
        ok: bool,
        verified: Option<bool>,
        at: MonoNs,
    },
    /// The plane denied the agent task (flag `waddle.v0.agent`, E26/E26b) —
    /// injected by the runtime when `AgentTaskUpdate{DENIED}` arrives.
    /// QUEUED/COMPLETED updates are runtime-side information, never FSM
    /// events (FSM.md §1.5).
    AgentTaskDenied {
        detail: String,
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
            | Self::InterventionRejected { at, .. }
            | Self::PostResetResult { at, .. }
            | Self::ResetWindowEngage { at, .. }
            | Self::ResetWindowComplete { at, .. }
            | Self::AgentTaskDenied { at, .. } => *at,
        }
    }
}
