//! Tracks the gate's per-tick cost (target: sub-microsecond nominal) on the
//! passthrough path AND under a plane-granted claim — the claimed arms carry
//! a provenance tag the gate clones twice per tick, so they are the ones a
//! tag that grew a heap field would regress. Reported, not asserted — the
//! binding numbers for a deployment come from `waddle doctor` on the actual
//! rig (N16).

use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;
use std::sync::Arc;
use waddle_fsm::ActiveClaim;
use waddle_gate::plan::PlanMode;
use waddle_gate::{Gate, GatePlan, gate::GateShared};
use waddle_ingest::SessionClock;
use waddle_types::{ActorKind, ActorRef, ClaimId, MonoNs, ReplanPolicy};

fn bench_passthrough(c: &mut Criterion) {
    let clock = SessionClock::capture();
    let (shared, _stream_tx) = GateShared::new(
        GatePlan::passthrough(MonoNs(0)),
        64,
        0,
        ReplanPolicy::Immediate,
    );
    let (mut gate, mut records_rx) = Gate::new(shared, clock, 8192);
    let action = [0.1f64; 14];

    c.bench_function("gate_passthrough_14dof", |b| {
        b.iter(|| {
            let out = gate.gate(black_box(&action), Some(0.5), None);
            // Keep the record ring from filling (a drop is a different path).
            while records_rx.pop().is_ok() {}
            black_box(out)
        });
    });

    let obs = [0.1f64; 30];
    c.bench_function("gate_passthrough_14dof_obs30", |b| {
        b.iter(|| {
            let out = gate.gate(black_box(&action), Some(0.5), Some(black_box(&obs)));
            while records_rx.pop().is_ok() {}
            black_box(out)
        });
    });
}

/// The BYPASS arm under a claim the plane granted (a stamped `ActorRef`, a
/// display name): one record push, one marker return, and two clones of the
/// claim's provenance tag. Same cost class as passthrough — that is the
/// property this tracks.
fn bench_claimed(c: &mut Criterion) {
    let clock = SessionClock::capture();
    let (shared, _stream_tx) = GateShared::new(
        GatePlan::passthrough(MonoNs(0)),
        64,
        0,
        ReplanPolicy::Immediate,
    );
    let (mut gate, mut records_rx) = Gate::new(shared.clone(), clock, 8192);
    let action = [0.1f64; 14];
    let obs = [0.1f64; 30];

    let claim = ActiveClaim {
        id: ClaimId::new("claim-bench"),
        source: "agent".to_owned(),
        actor: Arc::new(ActorRef {
            kind: ActorKind::Agent,
            id: "agent:ws-1@plane".to_owned(),
            display_name: "Waddle agent 347".to_owned(),
        }),
        self_initiated: false,
    };
    shared.store_plan(GatePlan {
        mode: PlanMode::Bypass {
            provenance: claim.provenance(),
        },
        since: MonoNs(0),
    });

    c.bench_function("gate_bypass_plane_claim_14dof_obs30", |b| {
        b.iter(|| {
            let out = gate.gate(black_box(&action), Some(0.5), Some(black_box(&obs)));
            while records_rx.pop().is_ok() {}
            black_box(out)
        });
    });
}

criterion_group!(benches, bench_passthrough, bench_claimed);
criterion_main!(benches);
