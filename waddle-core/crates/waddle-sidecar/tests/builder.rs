//! Builder span derivation + the canonical-JSON golden snapshot.

use waddle_sidecar::{SidecarBuilder, sidecar_to_json};
use waddle_types::pb::v0 as pb;
use waddle_types::time::{ClockAnchor, EpochNs, MonoNs, Stamp};
use waddle_types::{CellId, EpisodeId, RobotId, SessionId, TerminalOutcome};

const ANCHOR_MONO: i64 = 3_600_000_000_000;
const ANCHOR_UNIX: i64 = 1_784_000_000_000_000_000;

/// FakeClock-style fixed stamps for tests. waddle-sidecar must not depend
/// on waddle-ingest (layering), so tests mint stamps from parts directly —
/// the one sanctioned use of the escape hatch outside waddle-ingest.
#[allow(clippy::disallowed_methods)]
fn stamp(mono_ns: i64) -> Stamp {
    Stamp::from_parts_unchecked(
        MonoNs(mono_ns),
        EpochNs(ANCHOR_UNIX + (mono_ns - ANCHOR_MONO)),
    )
}

fn anchor() -> ClockAnchor {
    ClockAnchor {
        monotonic_ns: MonoNs(ANCHOR_MONO),
        unix_ns: EpochNs(ANCHOR_UNIX),
    }
}

fn builder(episode_id: &str) -> SidecarBuilder {
    SidecarBuilder::new(
        EpisodeId::new(episode_id),
        "towel-folding-pilot",
        SessionId::new("sess-7d41f0"),
        RobotId::new("yam-01"),
        CellId::new("cell-a"),
        "fold_towel_half",
        anchor(),
        pb::RecordingMode::Local,
    )
}

fn event(t_ns: i64, episode_id: &str, e: pb::episode_event::Event) -> pb::EpisodeEvent {
    pb::EpisodeEvent {
        t_ns,
        episode_id: episode_id.into(),
        event: Some(e),
    }
}

fn state(from: pb::EpisodeState, to: pb::EpisodeState, reason: &str) -> pb::episode_event::Event {
    pb::episode_event::Event::State(pb::StateTransition {
        from: from as i32,
        to: to as i32,
        reason: reason.into(),
        outcome: pb::TerminalOutcome::Unspecified as i32,
    })
}

fn lease_event(
    kind: pb::LeaseEventKind,
    lease_id: &str,
    holder: &str,
    t: i64,
) -> pb::episode_event::Event {
    pb::episode_event::Event::Lease(pb::LeaseEvent {
        kind: kind as i32,
        lease: Some(pb::Lease {
            lease_id: lease_id.into(),
            holder_client_id: holder.into(),
            enforcement: pb::LeaseEnforcement::Enforced as i32,
            t_acquired_ns: t,
        }),
        detail: String::new(),
    })
}

fn claim(claim_id: &str, episode_id: &str, t_granted_ns: i64) -> pb::Claim {
    pb::Claim {
        claim_id: claim_id.into(),
        episode_id: episode_id.into(),
        actor: Some(pb::ActorRef {
            kind: pb::ActorKind::Teleoperator as i32,
            id: "top-347".into(),
            display_name: "Waddle teleoperator 347".into(),
        }),
        source_name: "teleop".into(),
        self_initiated: false,
        t_granted_ns,
        handoff: Some(pb::HandoffPolicy {
            policy: Some(pb::handoff_policy::Policy::Immediate(
                pb::handoff_policy::Immediate {
                    blend_ns: 300_000_000,
                },
            )),
        }),
        spaces: vec![pb::SpaceKind::EePoseDelta as i32],
    }
}

fn intervention(phase: pb::InterventionPhase, claim_id: &str) -> pb::episode_event::Event {
    pb::episode_event::Event::Intervention(pb::InterventionEvent {
        phase: phase as i32,
        claim_id: claim_id.into(),
    })
}

fn gate(from: pb::GateMode, to: pb::GateMode, reason: &str) -> pb::episode_event::Event {
    pb::episode_event::Event::Gate(pb::GateModeChange {
        from: from as i32,
        to: to as i32,
        reason: reason.into(),
    })
}

/// A nominal episode with one teleop intervention-and-release cycle:
/// policy → teleop → policy.
fn nominal_episode() -> pb::Sidecar {
    use pb::EpisodeState as Es;
    let ep = "int-builder-01";
    let mut b = builder(ep);
    b.open_bounds(stamp(3_700_000_000_000));

    b.push_event(event(
        3_700_000_000_000,
        ep,
        state(Es::Unspecified, Es::Resetting, "episode_open"),
    ));
    b.push_event(event(
        3_700_000_000_000,
        ep,
        lease_event(
            pb::LeaseEventKind::Acquired,
            "lease-a1f3",
            "customer-loop",
            3_700_000_000_000,
        ),
    ));
    b.push_event(event(
        3_703_000_000_000,
        ep,
        state(Es::Resetting, Es::Ready, "reset verified (blocking)"),
    ));
    b.push_event(event(
        3_704_000_000_000,
        ep,
        state(Es::Ready, Es::Running, "first gated action"),
    ));

    // Intervention cycle: claim, engage, handoff, settle, release, handoff
    // back, un-claim.
    b.push_event(event(
        3_721_800_000_000,
        ep,
        pb::episode_event::Event::Claim(pb::ClaimEvent {
            kind: pb::ClaimEventKind::Granted as i32,
            claim: Some(claim("claim-01a", ep, 3_721_800_000_000)),
            detail: String::new(),
        }),
    ));
    b.push_event(event(
        3_722_000_000_000,
        ep,
        intervention(pb::InterventionPhase::Engage, "claim-01a"),
    ));
    b.push_event(event(
        3_722_100_000_000,
        ep,
        state(Es::Running, Es::Intervention, "engage"),
    ));
    b.push_event(event(
        3_722_500_000_000,
        ep,
        lease_event(
            pb::LeaseEventKind::HandedOff,
            "lease-b27c",
            "teleop-console-7",
            3_722_500_000_000,
        ),
    ));
    b.push_event(event(
        3_722_500_000_000,
        ep,
        gate(
            pb::GateMode::Passthrough,
            pb::GateMode::Intervention,
            "engage complete",
        ),
    ));
    b.push_event(event(
        3_722_600_000_000,
        ep,
        intervention(pb::InterventionPhase::Settle, "claim-01a"),
    ));
    b.push_event(event(
        3_740_500_000_000,
        ep,
        intervention(pb::InterventionPhase::Release, "claim-01a"),
    ));
    b.push_event(event(
        3_741_000_000_000,
        ep,
        lease_event(
            pb::LeaseEventKind::HandedOff,
            "lease-c390",
            "customer-loop",
            3_741_000_000_000,
        ),
    ));
    b.push_event(event(
        3_741_000_000_000,
        ep,
        gate(
            pb::GateMode::Intervention,
            pb::GateMode::Passthrough,
            "release complete",
        ),
    ));
    b.push_event(event(
        3_741_400_000_000,
        ep,
        pb::episode_event::Event::Claim(pb::ClaimEvent {
            kind: pb::ClaimEventKind::Released as i32,
            claim: Some(claim("claim-01a", ep, 3_721_800_000_000)),
            detail: String::new(),
        }),
    ));
    b.push_event(event(
        3_741_500_000_000,
        ep,
        state(Es::Intervention, Es::Running, "release"),
    ));
    b.push_event(event(
        3_760_000_000_000,
        ep,
        state(Es::Running, Es::Terminal, "judge_result: done"),
    ));

    b.set_outcome(TerminalOutcome::Success, "judge_result: done");
    b.set_reset_verification(pb::ResetVerification {
        mode: pb::ResetVerificationMode::Blocking as i32,
        verified: true,
        invalidated_async: false,
        judge_id: "reset-scene-v1".into(),
        judge_version: "2026.05".into(),
        score: Some(0.95),
        t_ns: 3_702_800_000_000,
    });
    b.set_audit(pb::AuditRecord {
        state: pb::AuditState::NotRetained as i32,
        random_quota_sample: false,
        audit_labels: vec![],
    });
    b.close_bounds(stamp(3_760_000_000_000));
    b.finish("sha256:8c1d29f4a7b3e6d05f92c84ab1e7d3960f45ac28b91d7e30c6a54f18d2e9b07c")
        .unwrap()
}

#[test]
fn nominal_episode_derives_spans() {
    let s = nominal_episode();

    // Bounds: both twins copied from the stamps, never derived at close.
    let bounds = s.bounds.as_ref().unwrap();
    assert_eq!(bounds.t_start_ns, 3_700_000_000_000);
    assert_eq!(bounds.t_end_ns, 3_760_000_000_000);
    assert_eq!(s.t_start_unix_ns, 1_784_000_100_000_000_000);
    assert_eq!(s.t_end_unix_ns, 1_784_000_160_000_000_000);

    // Three lease spans: policy → teleop → policy, each with its own token.
    assert_eq!(s.leases.len(), 3);
    let ids: Vec<&str> = s
        .leases
        .iter()
        .map(|l| l.lease.as_ref().unwrap().lease_id.as_str())
        .collect();
    assert_eq!(ids, ["lease-a1f3", "lease-b27c", "lease-c390"]);
    let spans: Vec<(i64, i64)> = s
        .leases
        .iter()
        .map(|l| {
            let sp = l.span.as_ref().unwrap();
            (sp.t_start_ns, sp.t_end_ns)
        })
        .collect();
    assert_eq!(
        spans,
        [
            (3_700_000_000_000, 3_722_500_000_000),
            (3_722_500_000_000, 3_741_000_000_000),
            (3_741_000_000_000, 3_760_000_000_000),
        ]
    );

    // One closed claim span.
    assert_eq!(s.claims.len(), 1);
    let cs = &s.claims[0];
    assert_eq!(cs.span.as_ref().unwrap().t_start_ns, 3_721_800_000_000);
    assert_eq!(cs.span.as_ref().unwrap().t_end_ns, 3_741_400_000_000);
    assert_eq!(cs.claim.as_ref().unwrap().claim_id, "claim-01a");

    // Provenance follows the gate: policy → teleop (with the claim's
    // actor) → policy, aligned to the gate-mode changes.
    assert_eq!(s.provenance.len(), 3);
    let kinds: Vec<i32> = s
        .provenance
        .iter()
        .map(|p| p.tag.as_ref().unwrap().kind)
        .collect();
    assert_eq!(
        kinds,
        [
            pb::ProvenanceKind::Policy as i32,
            pb::ProvenanceKind::Teleop as i32,
            pb::ProvenanceKind::Policy as i32,
        ]
    );
    let teleop = &s.provenance[1];
    assert_eq!(
        teleop.tag.as_ref().unwrap().actor.as_ref().unwrap().id,
        "top-347"
    );
    assert_eq!(teleop.span.as_ref().unwrap().t_start_ns, 3_722_500_000_000);
    assert_eq!(teleop.span.as_ref().unwrap().t_end_ns, 3_741_000_000_000);

    // One intervention span, engage → settle → release, closed by the
    // state transition out of INTERVENTION.
    assert_eq!(s.interventions.len(), 1);
    let iv = &s.interventions[0];
    assert_eq!(iv.claim_id, "claim-01a");
    assert_eq!(iv.span.as_ref().unwrap().t_start_ns, 3_722_000_000_000);
    assert_eq!(iv.span.as_ref().unwrap().t_end_ns, 3_741_500_000_000);
    let phases: Vec<(i32, i64, i64)> = iv
        .phases
        .iter()
        .map(|p| {
            let sp = p.span.as_ref().unwrap();
            (p.phase, sp.t_start_ns, sp.t_end_ns)
        })
        .collect();
    assert_eq!(
        phases,
        [
            (
                pb::InterventionPhase::Engage as i32,
                3_722_000_000_000,
                3_722_600_000_000
            ),
            (
                pb::InterventionPhase::Settle as i32,
                3_722_600_000_000,
                3_740_500_000_000
            ),
            (
                pb::InterventionPhase::Release as i32,
                3_740_500_000_000,
                3_741_500_000_000
            ),
        ]
    );

    // The raw event log is preserved verbatim.
    assert_eq!(s.events.len(), 16);
    assert_eq!(s.outcome(), pb::TerminalOutcome::Success);
    assert_eq!(s.metrics_class(), pb::MetricsClass::Standard);
    assert_eq!(s.recording_mode(), pb::RecordingMode::Local);
    assert!(!s.reset_unverified);
}

#[test]
fn born_claimed_retake_successor_keeps_claim_open() {
    let ep = "int-successor";
    let mut b = builder(ep);
    b.open_bounds(stamp(3_800_000_000_000));
    // Born claimed: the claim from the parent episode is still held.
    b.push_event(event(
        3_800_000_000_000,
        ep,
        pb::episode_event::Event::Claim(pb::ClaimEvent {
            kind: pb::ClaimEventKind::Granted as i32,
            claim: Some(claim("claim-r1", ep, 3_790_000_000_000)),
            detail: "carried from parent".into(),
        }),
    ));
    b.set_born_claimed(true);
    b.set_retake(&EpisodeId::new("int-parent"), &EpisodeId::new(ep));
    // Successor skipped verification: permanently flagged.
    b.mark_reset_unverified();
    b.set_outcome(TerminalOutcome::Success, "");
    b.close_bounds(stamp(3_830_000_000_000));
    let s = b.finish("sha256:digest").unwrap();

    assert!(s.born_claimed);
    assert_eq!(s.metrics_class(), pb::MetricsClass::BornClaimed);
    assert!(s.reset_unverified);
    let retake = s.retake.as_ref().unwrap();
    assert_eq!(retake.parent_episode_id, "int-parent");
    assert_eq!(retake.successor_episode_id, ep);
    // The still-held claim is an OPEN span (t_end_ns == 0).
    assert_eq!(s.claims.len(), 1);
    assert_eq!(s.claims[0].span.as_ref().unwrap().t_end_ns, 0);
}

#[test]
fn failed_reset_verification_permanently_flags() {
    let mut b = builder("int-flagged");
    b.open_bounds(stamp(3_900_000_000_000));
    b.set_reset_verification(pb::ResetVerification {
        mode: pb::ResetVerificationMode::OptimisticAsync as i32,
        verified: false,
        invalidated_async: true,
        judge_id: "reset-scene-v1".into(),
        judge_version: "2026.05".into(),
        score: Some(0.31),
        t_ns: 3_901_000_000_000,
    });
    // A later green record must NOT clear the flag.
    b.set_reset_verification(pb::ResetVerification {
        mode: pb::ResetVerificationMode::OptimisticAsync as i32,
        verified: true,
        invalidated_async: false,
        judge_id: "reset-scene-v1".into(),
        judge_version: "2026.05".into(),
        score: Some(0.99),
        t_ns: 3_902_000_000_000,
    });
    b.close_bounds(stamp(3_930_000_000_000));
    let s = b.finish("sha256:digest").unwrap();
    assert!(s.reset_unverified, "reset_unverified is never cleared");
}

#[test]
fn finish_before_open_bounds_is_an_error() {
    assert!(builder("int-unopened").finish("sha256:digest").is_err());
}

/// Post-reset record (flag `waddle.v0.reset.phases`): the declared flag is
/// stamped explicitly (it comes from `EpisodeOpen`, which is a session
/// event, not an emission); the `PostResetResult` payload is derived from
/// the pushed event; the bounds open at the →POST_RESET transition and
/// close at →TERMINAL (task duration = post_reset_bounds.start -
/// bounds.start, per the proto comment).
#[test]
fn post_reset_record_derives_result_and_bounds() {
    use pb::EpisodeState as Es;
    let ep = "int-post-reset";
    let mut b = builder(ep);
    b.open_bounds(stamp(4_000_000_000_000));
    b.set_post_reset_declared(true);
    b.push_event(event(
        4_000_000_000_000,
        ep,
        state(Es::Unspecified, Es::Resetting, "episode_open"),
    ));
    b.push_event(event(
        4_001_000_000_000,
        ep,
        state(Es::Resetting, Es::Ready, "reset ok"),
    ));
    b.push_event(event(
        4_002_000_000_000,
        ep,
        state(Es::Ready, Es::Running, "first gated action"),
    ));
    b.push_event(event(
        4_010_000_000_000,
        ep,
        state(Es::Running, Es::PostReset, "terminate: post-reset declared"),
    ));
    b.push_event(event(
        4_012_000_000_000,
        ep,
        pb::episode_event::Event::PostReset(pb::PostResetResult {
            result: Some(pb::ResetResult {
                ok: true,
                detail: "scene cleared".into(),
                ..Default::default()
            }),
            pinned_outcome: pb::TerminalOutcome::Success as i32,
        }),
    ));
    b.push_event(event(
        4_012_000_000_000,
        ep,
        state(Es::PostReset, Es::Terminal, "post-reset ok"),
    ));
    b.set_outcome(TerminalOutcome::Success, "");
    b.close_bounds(stamp(4_012_000_000_000));
    let s = b.finish("sha256:digest").unwrap();

    assert!(s.post_reset_declared);
    assert!(!s.post_reset_failed);
    let result = s.post_reset_result.as_ref().unwrap();
    assert!(result.ok);
    assert_eq!(result.detail, "scene cleared");
    let prb = s.post_reset_bounds.as_ref().unwrap();
    assert_eq!(prb.t_start_ns, 4_010_000_000_000);
    assert_eq!(prb.t_end_ns, 4_012_000_000_000);
}

/// E16/E17: `post_reset_failed` is permanent once set and never alters the
/// pinned outcome (field 13 keeps the outcome fixed at POST_RESET entry).
/// A failed cleanup's result payload is still recorded.
#[test]
fn post_reset_failed_flag_is_permanent_and_never_alters_outcome() {
    use pb::EpisodeState as Es;
    let ep = "int-post-reset-failed";
    let mut b = builder(ep);
    b.open_bounds(stamp(4_100_000_000_000));
    b.set_post_reset_declared(true);
    b.push_event(event(
        4_110_000_000_000,
        ep,
        state(Es::Running, Es::PostReset, "terminate: post-reset declared"),
    ));
    b.push_event(event(
        4_112_000_000_000,
        ep,
        pb::episode_event::Event::PostReset(pb::PostResetResult {
            result: Some(pb::ResetResult {
                ok: false,
                detail: "bin jammed".into(),
                ..Default::default()
            }),
            pinned_outcome: pb::TerminalOutcome::Success as i32,
        }),
    ));
    b.mark_post_reset_failed();
    b.push_event(event(
        4_112_000_000_000,
        ep,
        state(Es::PostReset, Es::Terminal, "post-reset failed"),
    ));
    // The outcome was pinned at POST_RESET entry — the failed cleanup never
    // rewrites it.
    b.set_outcome(TerminalOutcome::Success, "");
    b.close_bounds(stamp(4_112_000_000_000));
    let s = b.finish("sha256:digest").unwrap();

    assert!(s.post_reset_failed);
    assert_eq!(s.outcome(), pb::TerminalOutcome::Success);
    assert!(!s.post_reset_result.as_ref().unwrap().ok);
    let prb = s.post_reset_bounds.as_ref().unwrap();
    assert_eq!(prb.t_start_ns, 4_110_000_000_000);
    assert_eq!(prb.t_end_ns, 4_112_000_000_000);
}

/// An episode force-finalized mid-POST_RESET (session shutdown) leaves the
/// post-reset bounds open (`t_end_ns == 0`), the same "open when the record
/// was written" shape every other span uses.
#[test]
fn post_reset_bounds_left_open_when_never_terminal() {
    use pb::EpisodeState as Es;
    let ep = "int-post-reset-open";
    let mut b = builder(ep);
    b.open_bounds(stamp(4_200_000_000_000));
    b.set_post_reset_declared(true);
    b.push_event(event(
        4_210_000_000_000,
        ep,
        state(Es::Running, Es::PostReset, "terminate: post-reset declared"),
    ));
    b.close_bounds(stamp(4_211_000_000_000));
    let s = b.finish("sha256:digest").unwrap();

    let prb = s.post_reset_bounds.as_ref().unwrap();
    assert_eq!(prb.t_start_ns, 4_210_000_000_000);
    assert_eq!(prb.t_end_ns, 0, "never reached TERMINAL: open span");
    assert!(s.post_reset_result.is_none());
}

#[test]
fn canonical_json_golden_snapshot() {
    // Fixed stamps end to end, so the canonical JSON is deterministic.
    let json = sidecar_to_json(&nominal_episode()).unwrap();
    let value: serde_json::Value = serde_json::from_str(&json).unwrap();
    insta::assert_json_snapshot!(value);
}

// --- Provenance spans follow the claim's ACTOR, not an assumption ---------

/// One claimed span, derived from a claim held by `actor`. Returns the
/// INTERVENTION span's tag (index 1: policy → claimed → …).
fn claimed_span_tag(
    ep: &str,
    actor: Option<pb::ActorRef>,
    source_name: &str,
    self_initiated: bool,
) -> pb::ProvenanceTag {
    let mut b = builder(ep);
    b.open_bounds(stamp(4_300_000_000_000));
    b.push_event(event(
        4_301_000_000_000,
        ep,
        pb::episode_event::Event::Claim(pb::ClaimEvent {
            kind: pb::ClaimEventKind::Granted as i32,
            claim: Some(pb::Claim {
                claim_id: "claim-prov".into(),
                episode_id: ep.into(),
                actor,
                source_name: source_name.into(),
                self_initiated,
                ..Default::default()
            }),
            detail: String::new(),
        }),
    ));
    b.push_event(event(
        4_302_000_000_000,
        ep,
        gate(
            pb::GateMode::Passthrough,
            pb::GateMode::Intervention,
            "engage complete",
        ),
    ));
    b.close_bounds(stamp(4_310_000_000_000));
    let s = b.finish("sha256:digest").unwrap();
    assert_eq!(s.provenance.len(), 2, "policy then claimed");
    s.provenance[1].tag.clone().unwrap()
}

fn actor(kind: pb::ActorKind, id: &str) -> Option<pb::ActorRef> {
    Some(pb::ActorRef {
        kind: kind as i32,
        id: id.into(),
        display_name: String::new(),
    })
}

/// The bug this pins: a claimed span was minted as TELEOP unconditionally,
/// so every span of every agent-driven episode claimed a teleoperator drove
/// it. The claim's actor decides — and the span names them.
#[test]
fn an_agent_claim_makes_an_agent_span() {
    let tag = claimed_span_tag(
        "int-prov-agent",
        actor(pb::ActorKind::Agent, "agent:ws-1@plane"),
        "waddle-agent",
        false,
    );
    assert_eq!(tag.kind, pb::ProvenanceKind::Agent as i32);
    assert_eq!(tag.actor.as_ref().unwrap().id, "agent:ws-1@plane");
}

/// The flow that already worked must keep working byte for byte: a
/// teleoperator's claim is still TELEOP, still carrying the teleoperator.
#[test]
fn a_teleoperator_claim_still_makes_a_teleop_span() {
    let tag = claimed_span_tag(
        "int-prov-teleop",
        actor(pb::ActorKind::Teleoperator, "top-347"),
        "teleop",
        false,
    );
    assert_eq!(tag.kind, pb::ProvenanceKind::Teleop as i32);
    assert_eq!(tag.actor.as_ref().unwrap().id, "top-347");
}

/// A customer-side human at the cell is NOT a Waddle work-plane
/// teleoperator (N17): the corpus must be able to tell them apart, so the
/// span is CUSTOM named for the claim's source rather than folded into
/// TELEOP. A clutch claim also carries its `bypass_approval` stamp, the
/// same one the per-action tags carry.
#[test]
fn a_site_operator_clutch_claim_is_custom_and_keeps_its_bypass_stamp() {
    let tag = claimed_span_tag(
        "int-prov-site",
        actor(pb::ActorKind::SiteOperator, ""),
        "leader_arm",
        true,
    );
    assert_eq!(tag.kind, pb::ProvenanceKind::Custom as i32);
    assert_eq!(tag.custom_name, "leader_arm");
    assert!(tag.bypass_approval);
}

/// A claim event that reached the sidecar with no actor at all (a peer that
/// predates the actor being carried) is attributed to its source name — the
/// builder never invents an actor kind it was not told.
#[test]
fn a_claim_with_no_actor_is_attributed_to_its_source() {
    let tag = claimed_span_tag("int-prov-none", None, "leader_arm", false);
    assert_eq!(tag.kind, pb::ProvenanceKind::Custom as i32);
    assert_eq!(tag.custom_name, "leader_arm");
    assert!(tag.actor.is_none());
}
