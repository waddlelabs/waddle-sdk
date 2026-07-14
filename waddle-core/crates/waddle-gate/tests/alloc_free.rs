//! The passthrough fast path must be allocation-free (≤ 16 dims) and fast.
//! A counting allocator proves the former; a generous wall-clock smoke bound
//! (p50 < 5 µs over 1M calls) guards against gross regressions without CI
//! flake.

// The counting allocator requires implementing GlobalAlloc (unsafe by
// nature); confined to this test binary.
#![allow(unsafe_code)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicU64, Ordering};

struct CountingAlloc;

static ALLOCATIONS: AtomicU64 = AtomicU64::new(0);

// SAFETY: delegates directly to the system allocator; the counter is a
// relaxed atomic with no other side effects.
unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        // SAFETY: same contract as the caller's.
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: same contract as the caller's.
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: CountingAlloc = CountingAlloc;

#[test]
fn passthrough_is_allocation_free_and_fast() {
    use waddle_gate::{Gate, GateOutput, GatePlan, gate::GateShared};
    use waddle_ingest::FakeClock;
    use waddle_types::MonoNs;

    let clock = FakeClock::default();
    let (shared, _stream_tx) = GateShared::new(GatePlan::passthrough(MonoNs(0)), 64, 0);
    let (mut gate, mut records_rx) = Gate::new(shared, clock.clone(), 4096);
    let action = [0.25f64; 14];

    // Warm up (first call may lazily initialize).
    for _ in 0..1_000 {
        clock.advance(1_000);
        let _ = gate.gate(&action, Some(0.5));
    }
    while records_rx.pop().is_ok() {}

    const CALLS: u64 = 1_000_000;
    let before = ALLOCATIONS.load(Ordering::Relaxed);
    #[allow(clippy::disallowed_methods)] // test-only wall measurement
    let start = std::time::Instant::now();
    for i in 0..CALLS {
        clock.advance(1_000);
        let out = gate.gate(&action, Some(0.5));
        assert!(matches!(out, GateOutput::Pass { .. }));
        // Drain periodically so the ring never fills (drop path ≠ fast path).
        if i % 1024 == 0 {
            while records_rx.pop().is_ok() {}
        }
    }
    let elapsed = start.elapsed();
    let after = ALLOCATIONS.load(Ordering::Relaxed);

    assert_eq!(
        after - before,
        0,
        "the passthrough fast path must not allocate"
    );

    let mean_ns = elapsed.as_nanos() / u128::from(CALLS);
    assert!(
        mean_ns < 5_000,
        "passthrough mean {mean_ns}ns exceeded the 5µs smoke bound"
    );
}
