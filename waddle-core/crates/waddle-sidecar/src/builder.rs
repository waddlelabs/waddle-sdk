//! [`SidecarBuilder`]: accumulates one episode's event stream and derives
//! the sidecar's span tables incrementally.
//!
//! Span derivation rules (all timestamps are event `t_ns`, session-monotonic):
//!
//! - `ClaimEvent` GRANTED opens a [`pb::ClaimSpan`]; RELEASED closes the
//!   matching one. A claim still open at close stays open (`t_end_ns == 0`)
//!   — that is the ABORTED_RETAKE shape, where the claim is carried to the
//!   born-claimed successor.
//! - `LeaseEvent` ACQUIRED/HANDED_OFF closes the previous [`pb::LeaseSpan`]
//!   and opens a new one; RELEASED/REVOKED_ALL closes without opening.
//! - `InterventionEvent` ENGAGE opens an [`pb::InterventionSpan`]; each
//!   subsequent phase closes the previous [`pb::intervention_span::PhaseSpan`]
//!   and opens the next; the `StateTransition` OUT of
//!   `EPISODE_STATE_INTERVENTION` closes the open phase and the span.
//! - `GateModeChange` drives [`pb::ProvenanceSpan`]s: PASSTHROUGH means the
//!   policy is writing, INTERVENTION and BYPASS mean the claim-holding human
//!   is (during bypass the SDK's thread writes on the intervenor's behalf).
//!   Teleop spans carry the open claim's actor when one exists.
//!
//! Two-clock discipline: `open_bounds`/`close_bounds` take [`Stamp`]s and
//! copy BOTH twins from them. The epoch twin is never derived from the
//! [`ClockAnchor`] at close — that post-hoc derivation silently corrupts
//! offsets across host suspends and is the postmortem behind the `Stamp`
//! type.

use waddle_types::pb::v0 as pb;
use waddle_types::time::{ClockAnchor, Stamp};
use waddle_types::{CellId, EpisodeId, RobotId, SessionId, TerminalOutcome};

use crate::error::SidecarError;

#[derive(Debug)]
struct OpenIntervention {
    t_start_ns: i64,
    claim_id: String,
    phases: Vec<pb::intervention_span::PhaseSpan>,
    open_phase: Option<(i32, i64)>,
}

impl OpenIntervention {
    fn close_phase(&mut self, t_ns: i64) {
        if let Some((phase, t_start_ns)) = self.open_phase.take() {
            self.phases.push(pb::intervention_span::PhaseSpan {
                phase,
                span: Some(span(t_start_ns, t_ns)),
            });
        }
    }

    fn into_span(mut self, t_end_ns: i64) -> pb::InterventionSpan {
        self.close_phase(t_end_ns);
        pb::InterventionSpan {
            span: Some(span(self.t_start_ns, t_end_ns)),
            claim_id: self.claim_id,
            phases: self.phases,
        }
    }
}

/// Builds one [`pb::Sidecar`] from an episode's identity, its event stream,
/// and explicit terminal facts. See the module docs for derivation rules.
#[derive(Debug)]
pub struct SidecarBuilder {
    episode_id: EpisodeId,
    project: String,
    session_id: SessionId,
    robot_id: RobotId,
    cell_id: CellId,
    task: String,
    anchor: ClockAnchor,
    recording_mode: pb::RecordingMode,

    bounds_start: Option<Stamp>,
    bounds_end: Option<Stamp>,

    outcome: i32,
    outcome_detail: String,
    born_claimed: bool,
    retake: Option<pb::RetakeLink>,
    reset_verification: Option<pb::ResetVerification>,
    reset_unverified: bool,
    post_reset_declared: bool,
    post_reset_failed: bool,
    post_reset_result: Option<pb::ResetResult>,
    /// Bounds of the post-reset phase (flag `waddle.v0.reset.phases`): opens
    /// at the →POST_RESET state transition, closes at →TERMINAL. Both ends
    /// are event `t_ns` (session-monotonic), same as every other span.
    post_reset_start_ns: Option<i64>,
    post_reset_end_ns: Option<i64>,

    events: Vec<pb::EpisodeEvent>,
    judgments: Vec<pb::Judgment>,
    refs: Vec<pb::ArchiveRef>,
    incident_clips: Vec<pb::ArchiveRef>,
    audit: Option<pb::AuditRecord>,

    claims: Vec<pb::ClaimSpan>,
    open_claims: Vec<(i64, pb::Claim)>,
    leases: Vec<pb::LeaseSpan>,
    open_lease: Option<(i64, pb::Lease)>,
    provenance: Vec<pb::ProvenanceSpan>,
    open_provenance: Option<(i64, pb::ProvenanceTag)>,
    interventions: Vec<pb::InterventionSpan>,
    open_intervention: Option<OpenIntervention>,
}

impl SidecarBuilder {
    #[expect(clippy::too_many_arguments, reason = "episode identity is wide")]
    #[must_use]
    pub fn new(
        episode_id: EpisodeId,
        project: impl Into<String>,
        session_id: SessionId,
        robot_id: RobotId,
        cell_id: CellId,
        task: impl Into<String>,
        anchor: ClockAnchor,
        recording_mode: pb::RecordingMode,
    ) -> Self {
        Self {
            episode_id,
            project: project.into(),
            session_id,
            robot_id,
            cell_id,
            task: task.into(),
            anchor,
            recording_mode,
            bounds_start: None,
            bounds_end: None,
            outcome: pb::TerminalOutcome::Unspecified as i32,
            outcome_detail: String::new(),
            born_claimed: false,
            retake: None,
            reset_verification: None,
            reset_unverified: false,
            post_reset_declared: false,
            post_reset_failed: false,
            post_reset_result: None,
            post_reset_start_ns: None,
            post_reset_end_ns: None,
            events: Vec::new(),
            judgments: Vec::new(),
            refs: Vec::new(),
            incident_clips: Vec::new(),
            audit: None,
            claims: Vec::new(),
            open_claims: Vec::new(),
            leases: Vec::new(),
            open_lease: None,
            provenance: Vec::new(),
            open_provenance: None,
            interventions: Vec::new(),
            open_intervention: None,
        }
    }

    /// Open the episode bounds. Both twins (`bounds.t_start_ns` and
    /// `t_start_unix_ns`) come from this stamp, captured at stamp time.
    /// Also opens the initial policy provenance span.
    pub fn open_bounds(&mut self, stamp: Stamp) {
        self.bounds_start = Some(stamp);
        if self.open_provenance.is_none() {
            self.open_provenance = Some((stamp.mono_ns().0, policy_tag()));
        }
    }

    /// Close the episode bounds. Both twins come from this stamp — NEVER
    /// derived from the clock anchor at close. Trailing open lease and
    /// provenance spans close here; a still-held claim (and its
    /// intervention, if any) stays open per Span semantics.
    pub fn close_bounds(&mut self, stamp: Stamp) {
        self.bounds_end = Some(stamp);
        let t = stamp.mono_ns().0;
        if let Some((t_start, lease)) = self.open_lease.take() {
            self.leases.push(pb::LeaseSpan {
                span: Some(span(t_start, t)),
                lease: Some(lease),
            });
        }
        if let Some((t_start, tag)) = self.open_provenance.take() {
            self.provenance.push(pb::ProvenanceSpan {
                span: Some(span(t_start, t)),
                tag: Some(tag),
            });
        }
    }

    /// Append one event to the sidecar's event log, deriving spans
    /// incrementally (see module docs).
    pub fn push_event(&mut self, event: pb::EpisodeEvent) {
        let t = event.t_ns;
        match &event.event {
            Some(pb::episode_event::Event::Claim(c)) => self.on_claim(t, c),
            Some(pb::episode_event::Event::Lease(l)) => self.on_lease(t, l),
            Some(pb::episode_event::Event::Intervention(i)) => self.on_intervention(t, i),
            Some(pb::episode_event::Event::Gate(g)) => self.on_gate(t, g),
            Some(pb::episode_event::Event::State(s)) => self.on_state(t, s),
            // The post-reset pipeline reported (E15/E16): keep the latest
            // result payload; `pinned_outcome` already lives on the sidecar's
            // own `outcome` field (pinned at E14, carried to TERMINAL).
            Some(pb::episode_event::Event::PostReset(pr)) => {
                self.post_reset_result.clone_from(&pr.result);
            }
            _ => {}
        }
        self.events.push(event);
    }

    fn on_claim(&mut self, t: i64, e: &pb::ClaimEvent) {
        match pb::ClaimEventKind::try_from(e.kind) {
            Ok(pb::ClaimEventKind::Granted) => {
                if let Some(claim) = &e.claim {
                    self.open_claims.push((t, claim.clone()));
                }
            }
            Ok(pb::ClaimEventKind::Released) => {
                let claim_id = e.claim.as_ref().map(|c| c.claim_id.as_str());
                let idx = self
                    .open_claims
                    .iter()
                    .position(|(_, c)| Some(c.claim_id.as_str()) == claim_id);
                if let Some(idx) = idx {
                    let (t_start, claim) = self.open_claims.remove(idx);
                    self.claims.push(pb::ClaimSpan {
                        span: Some(span(t_start, t)),
                        claim: Some(claim),
                    });
                }
            }
            _ => {}
        }
    }

    fn on_lease(&mut self, t: i64, e: &pb::LeaseEvent) {
        let kind = pb::LeaseEventKind::try_from(e.kind);
        let closes = matches!(
            kind,
            Ok(pb::LeaseEventKind::Acquired
                | pb::LeaseEventKind::HandedOff
                | pb::LeaseEventKind::Released
                | pb::LeaseEventKind::RevokedAll)
        );
        let opens = matches!(
            kind,
            Ok(pb::LeaseEventKind::Acquired | pb::LeaseEventKind::HandedOff)
        );
        if closes && let Some((t_start, lease)) = self.open_lease.take() {
            self.leases.push(pb::LeaseSpan {
                span: Some(span(t_start, t)),
                lease: Some(lease),
            });
        }
        if opens && let Some(lease) = &e.lease {
            self.open_lease = Some((t, lease.clone()));
        }
    }

    fn on_intervention(&mut self, t: i64, e: &pb::InterventionEvent) {
        if pb::InterventionPhase::try_from(e.phase) == Ok(pb::InterventionPhase::Engage) {
            // A dangling previous intervention (no state transition seen)
            // closes at the new engage rather than leaking open forever.
            if let Some(open) = self.open_intervention.take() {
                self.interventions.push(open.into_span(t));
            }
            self.open_intervention = Some(OpenIntervention {
                t_start_ns: t,
                claim_id: e.claim_id.clone(),
                phases: Vec::new(),
                open_phase: Some((e.phase, t)),
            });
        } else if let Some(open) = &mut self.open_intervention {
            open.close_phase(t);
            open.open_phase = Some((e.phase, t));
        }
    }

    fn on_gate(&mut self, t: i64, e: &pb::GateModeChange) {
        let tag = match pb::GateMode::try_from(e.to) {
            Ok(pb::GateMode::Passthrough) => policy_tag(),
            // During BYPASS the SDK's thread writes on the claim-holding
            // intervenor's behalf; provenance is theirs, same as INTERVENTION.
            Ok(pb::GateMode::Intervention | pb::GateMode::Bypass) => {
                let actor = self.open_claims.last().and_then(|(_, c)| c.actor.clone());
                pb::ProvenanceTag {
                    kind: pb::ProvenanceKind::Teleop as i32,
                    custom_name: String::new(),
                    actor,
                    bypass_approval: false,
                }
            }
            _ => return,
        };
        if let Some((t_start, prev)) = self.open_provenance.take() {
            self.provenance.push(pb::ProvenanceSpan {
                span: Some(span(t_start, t)),
                tag: Some(prev),
            });
        }
        self.open_provenance = Some((t, tag));
    }

    fn on_state(&mut self, t: i64, e: &pb::StateTransition) {
        let leaving_intervention = e.from == pb::EpisodeState::Intervention as i32
            && e.to != pb::EpisodeState::Intervention as i32;
        if leaving_intervention && let Some(open) = self.open_intervention.take() {
            self.interventions.push(open.into_span(t));
        }
        // Post-reset bounds: open at →POST_RESET (E14), close at →TERMINAL
        // (E15/E16/E17 all transition there). An episode force-finalized
        // mid-POST_RESET leaves them open (t_end_ns == 0), like every span.
        if e.to == pb::EpisodeState::PostReset as i32 {
            self.post_reset_start_ns = Some(t);
        }
        if e.to == pb::EpisodeState::Terminal as i32 && self.post_reset_start_ns.is_some() {
            self.post_reset_end_ns = Some(t);
        }
    }

    /// Record the terminal outcome (domain enum — the wire cannot carry
    /// `UNSPECIFIED` through this path).
    pub fn set_outcome(&mut self, outcome: TerminalOutcome, detail: impl Into<String>) {
        self.outcome = outcome.to_pb() as i32;
        self.outcome_detail = detail.into();
    }

    /// Link this episode into a retake pair (set on BOTH sides of the pair).
    pub fn set_retake(&mut self, parent: &EpisodeId, successor: &EpisodeId) {
        self.retake = Some(pb::RetakeLink {
            parent_episode_id: parent.as_str().to_owned(),
            successor_episode_id: successor.as_str().to_owned(),
        });
    }

    /// Mark the episode born-claimed (a retake successor opened under a
    /// still-held claim, N18). Sets the metrics class accordingly:
    /// born-claimed episodes are their own metrics class, excluded from
    /// mean-time-to-intervention.
    pub fn set_born_claimed(&mut self, born_claimed: bool) {
        self.born_claimed = born_claimed;
    }

    /// Attach the reset-verification record. A failed or retroactively
    /// invalidated verification permanently sets `reset_unverified` (N2/N12)
    /// — the flag is never cleared, even by a later successful record.
    pub fn set_reset_verification(&mut self, v: pb::ResetVerification) {
        if !v.verified || v.invalidated_async {
            self.reset_unverified = true;
        }
        self.reset_verification = Some(v);
    }

    /// Permanently mark this episode as having run without a verified reset
    /// (e.g. a retake successor that skipped verification). One-way.
    pub fn mark_reset_unverified(&mut self) {
        self.reset_unverified = true;
    }

    /// Mark that this episode declared a post-reset pipeline (flag
    /// `waddle.v0.reset.phases`). Stamped explicitly by the runtime from
    /// `EpisodeOpen` — a session event, not an emission, so it cannot be
    /// derived from the pushed event stream.
    pub fn set_post_reset_declared(&mut self, declared: bool) {
        self.post_reset_declared = declared;
    }

    /// Permanently mark the post-reset cleanup as failed or estopped
    /// (E16/E17). One-way, like `mark_reset_unverified`; NEVER alters the
    /// outcome — the pinned outcome from before POST_RESET entry stands.
    pub fn mark_post_reset_failed(&mut self) {
        self.post_reset_failed = true;
    }

    pub fn add_judgment(&mut self, j: pb::Judgment) {
        self.judgments.push(j);
    }

    /// Add a Reference-mode bulk pointer.
    pub fn add_ref(&mut self, r: pb::ArchiveRef) {
        self.refs.push(r);
    }

    /// Add a ring-buffer incident clip reference.
    pub fn add_incident_clip(&mut self, r: pb::ArchiveRef) {
        self.incident_clips.push(r);
    }

    /// Set the audit-slice record (N13).
    pub fn set_audit(&mut self, audit: pb::AuditRecord) {
        self.audit = Some(audit);
    }

    /// Assemble the sidecar (`sidecar_version = 1`). Spans still open —
    /// a held claim, an unterminated intervention — are emitted with
    /// `t_end_ns == 0` ("open when the record was written").
    pub fn finish(mut self, robot_description_digest: &str) -> Result<pb::Sidecar, SidecarError> {
        let Some(start) = self.bounds_start else {
            return Err(SidecarError::Invalid(
                "SidecarBuilder::finish called before open_bounds".to_owned(),
            ));
        };

        // Anything left open closes as an open span (t_end_ns == 0).
        for (t_start, claim) in self.open_claims.drain(..) {
            self.claims.push(pb::ClaimSpan {
                span: Some(span(t_start, 0)),
                claim: Some(claim),
            });
        }
        if let Some((t_start, lease)) = self.open_lease.take() {
            self.leases.push(pb::LeaseSpan {
                span: Some(span(t_start, 0)),
                lease: Some(lease),
            });
        }
        if let Some((t_start, tag)) = self.open_provenance.take() {
            self.provenance.push(pb::ProvenanceSpan {
                span: Some(span(t_start, 0)),
                tag: Some(tag),
            });
        }
        if let Some(open) = self.open_intervention.take() {
            self.interventions.push(open.into_span(0));
        }

        let metrics_class = if self.born_claimed {
            pb::MetricsClass::BornClaimed
        } else {
            pb::MetricsClass::Standard
        };

        Ok(pb::Sidecar {
            sidecar_version: 1,
            episode_id: self.episode_id.as_str().to_owned(),
            project: self.project,
            session_id: self.session_id.as_str().to_owned(),
            robot_id: self.robot_id.as_str().to_owned(),
            cell_id: self.cell_id.as_str().to_owned(),
            task: self.task,
            task_metadata: Default::default(),
            clock_anchor: Some(self.anchor.to_pb()),
            bounds: Some(span(
                start.mono_ns().0,
                self.bounds_end.map_or(0, |s| s.mono_ns().0),
            )),
            t_start_unix_ns: start.epoch_ns().0,
            t_end_unix_ns: self.bounds_end.map_or(0, |s| s.epoch_ns().0),
            outcome: self.outcome,
            outcome_detail: self.outcome_detail,
            born_claimed: self.born_claimed,
            retake: self.retake,
            metrics_class: metrics_class as i32,
            claims: self.claims,
            leases: self.leases,
            provenance: self.provenance,
            interventions: self.interventions,
            events: self.events,
            judgments: self.judgments,
            reset_verification: self.reset_verification,
            reset_unverified: self.reset_unverified,
            recording_mode: self.recording_mode as i32,
            refs: self.refs,
            incident_clips: self.incident_clips,
            audit: self.audit,
            robot_description_digest: robot_description_digest.to_owned(),
            vendor: Default::default(),
            post_reset_declared: self.post_reset_declared,
            post_reset_failed: self.post_reset_failed,
            post_reset_result: self.post_reset_result,
            post_reset_bounds: self
                .post_reset_start_ns
                .map(|t_start| span(t_start, self.post_reset_end_ns.unwrap_or(0))),
        })
    }
}

fn span(t_start_ns: i64, t_end_ns: i64) -> pb::Span {
    pb::Span {
        t_start_ns,
        t_end_ns,
    }
}

fn policy_tag() -> pb::ProvenanceTag {
    pb::ProvenanceTag {
        kind: pb::ProvenanceKind::Policy as i32,
        custom_name: String::new(),
        actor: None,
        bypass_approval: false,
    }
}
