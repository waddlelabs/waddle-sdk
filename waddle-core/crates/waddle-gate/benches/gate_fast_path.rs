//! Tracks the passthrough fast path (target: sub-microsecond nominal).
//! Reported, not asserted — the binding numbers for a deployment come from
//! `waddle doctor` on the actual rig (N16).

use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;
use waddle_gate::{Gate, GatePlan, gate::GateShared};
use waddle_ingest::SessionClock;
use waddle_types::MonoNs;

fn bench_passthrough(c: &mut Criterion) {
    let clock = SessionClock::capture();
    let (shared, _stream_tx) = GateShared::new(GatePlan::passthrough(MonoNs(0)), 64, 0);
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

criterion_group!(benches, bench_passthrough);
criterion_main!(benches);
