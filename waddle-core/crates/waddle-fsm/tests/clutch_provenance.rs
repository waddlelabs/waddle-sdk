//! Bug: clutch-initiated (self-initiated) claims hardcoded
//! `ActorKind::SiteOperator`, so interventions performed by our
//! teleoperators through the leader-arm/console-clutch takeover path were
//! recorded as NOT teleop — provenance-labeled training data (DAgger pairs)
//! silently mislabeled (N17 actor vocabulary violated).
//!
//! `SessionConfig` gains `clutch_actor` alongside `clutch_source`; the
//! clutch handler uses it instead of a hardcoded actor. The FSM-level
//! defaults stay exactly as they were (`SiteOperator` / "custom") — pinned
//! by the second test below — so existing conformance fixtures and FSM
//! unit tests exercising clutch with default config keep identical traces;
//! only `waddle-runtime`'s `SessionBuilder` sets the honest default.

use waddle_fsm::{ActiveClaim, SessionConfig, SessionEvent, SessionFsm, step};
use waddle_types::{ActorKind, EpisodeId, HandoffPolicy, LeaseEnforcement, MonoNs};

/// Steps a fresh session through EPISODE_OPEN → RESET_RESULT(ok, verified)
/// → START, landing in RUNNING with no active claim — the precondition for
/// a clutch engage to self-initiate a claim (FSM.md §2/§5).
fn running_fsm(cfg: &SessionConfig) -> SessionFsm {
    let state = SessionFsm::new(cfg);
    let stepped = step(
        cfg,
        &state,
        &SessionEvent::EpisodeOpen {
            id: EpisodeId::new("ep-1"),
            verification: waddle_types::ResetVerificationMode::Blocking,
            born_claimed: false,
            parent: None,
            post_reset: false,
            pre_window: None,
            post_window: None,
            at: MonoNs(0),
        },
    )
    .expect("episode_open is legal from a fresh session");

    let stepped = step(
        cfg,
        &stepped.next,
        &SessionEvent::ResetResult {
            ok: true,
            verified: Some(true),
            at: MonoNs(1_000),
        },
    )
    .expect("reset_result is legal in RESETTING");

    let stepped = step(
        cfg,
        &stepped.next,
        &SessionEvent::Start { at: MonoNs(2_000) },
    )
    .expect("start is legal in READY");

    stepped.next
}

fn clutch_engaged_claim(cfg: &SessionConfig) -> ActiveClaim {
    let state = running_fsm(cfg);
    let stepped = step(
        cfg,
        &state,
        &SessionEvent::Clutch {
            engaged: true,
            at: MonoNs(3_000),
        },
    )
    .expect("clutch engage is legal in RUNNING with no active claim");
    stepped
        .next
        .claim
        .expect("clutch engage self-initiates a claim")
}

#[test]
fn clutch_engage_uses_the_configured_actor_and_source() {
    let mut cfg = SessionConfig::minimal(
        "loop-client",
        HandoffPolicy::HoldFirst,
        LeaseEnforcement::Advisory,
    );
    cfg.clutch_actor = ActorKind::Teleoperator;
    cfg.clutch_source = "teleop-clutch".to_owned();

    let claim = clutch_engaged_claim(&cfg);
    assert_eq!(claim.actor, ActorKind::Teleoperator);
    assert_eq!(claim.source, "teleop-clutch");
    assert!(claim.self_initiated);
}

#[test]
fn clutch_engage_with_default_config_keeps_legacy_actor_and_source() {
    // Pins fixture-stability: the FSM-level defaults must never move, only
    // the runtime layer's chosen defaults change.
    let cfg = SessionConfig::minimal(
        "loop-client",
        HandoffPolicy::HoldFirst,
        LeaseEnforcement::Advisory,
    );

    let claim = clutch_engaged_claim(&cfg);
    assert_eq!(claim.actor, ActorKind::SiteOperator);
    assert_eq!(claim.source, "custom");
    assert!(claim.self_initiated);
}
