//! The fast path must be allocation-free (≤ 16 action dims, ≤ 32 obs dims)
//! and fast. A counting allocator proves the former; a generous wall-clock
//! smoke bound (p50 < 5 µs over 1M calls) guards against gross regressions
//! without CI flake.
//!
//! Allocation-freedom is proved for PASSTHROUGH (the nominal tick) *and*
//! for every other plan arm the gate can tick under, because a claim is not
//! an exceptional state: a supervised session spends whole windows in
//! CLAIMED/BYPASS, and those ticks run on the same customer real-time
//! thread as the passthrough ones.

// The counting allocator requires implementing GlobalAlloc (unsafe by
// nature); confined to this test binary.
#![allow(unsafe_code)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

struct CountingAlloc;

thread_local! {
    /// Counted PER THREAD, not process-wide: the test harness runs the tests
    /// in this binary concurrently, and a shared counter would charge one
    /// test's setup allocations to another test's measured loop (which is
    /// exactly what a process-wide counter here did once a second measuring
    /// test existed). Const-initialized and `Copy`, so reading it never
    /// allocates or registers a destructor.
    static ALLOCATIONS: Cell<u64> = const { Cell::new(0) };
}

/// This thread's allocation count so far.
fn allocations() -> u64 {
    ALLOCATIONS.with(Cell::get)
}

// SAFETY: delegates directly to the system allocator; the counter is a
// thread-local `Cell` with no other side effects.
unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        // `try_with`: an allocation during TLS teardown must not panic.
        let _ = ALLOCATIONS.try_with(|c| c.set(c.get() + 1));
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
    use waddle_types::{MonoNs, ReplanPolicy};

    let clock = FakeClock::default();
    let (shared, _stream_tx) = GateShared::new(
        GatePlan::passthrough(MonoNs(0)),
        64,
        0,
        ReplanPolicy::Immediate,
    );
    let (mut gate, mut records_rx) = Gate::new(shared, clock.clone(), 4096);
    let action = [0.25f64; 14];
    let obs = [0.5f64; 30];

    // Warm up (first call may lazily initialize).
    for _ in 0..1_000 {
        clock.advance(1_000);
        let _ = gate.gate(&action, Some(0.5), Some(&obs));
    }
    while records_rx.pop().is_ok() {}

    const CALLS: u64 = 1_000_000;
    let before = allocations();
    #[allow(clippy::disallowed_methods)] // test-only wall measurement
    let start = std::time::Instant::now();
    for i in 0..CALLS {
        clock.advance(1_000);
        let out = gate.gate(&action, Some(0.5), Some(&obs));
        assert!(matches!(out, GateOutput::Pass { .. }));
        // Drain periodically so the ring never fills (drop path ≠ fast path).
        if i % 1024 == 0 {
            while records_rx.pop().is_ok() {}
        }
    }
    let elapsed = start.elapsed();
    let after = allocations();

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

/// Every OTHER plan arm is allocation-free too, under a claim shaped the way
/// the plane actually grants one.
///
/// The gate clones the active `ProvenanceTag` twice on every tick (once into
/// the record ring, once into the returned `GateOutput`), so anything the
/// tag OWNS is a malloc pair per clone on the customer's real-time thread.
/// A plane-granted claim carries a stamped `ActorRef{id, display_name}` and
/// a site-operator claim carries a `custom:<source>` provenance name — the
/// two variable-length fields — and both are exercised here. The passthrough
/// proof above cannot see this: its tag is `ProvenanceTag::policy()`, which
/// carries neither.
///
/// CLAIMED is covered with an EMPTY intervention ring. A claimed tick with a
/// pending action additionally drains the jitter buffer, whose per-channel
/// reorder map (a `BTreeMap`) allocates as it grows — a property of that
/// buffer's reordering, not of the tick, and unchanged by this test. It
/// clones the very same tag the same number of times, so the arm is covered
/// for what this test is about.
#[test]
fn every_plan_arm_is_allocation_free() {
    use std::sync::Arc;
    use waddle_fsm::ActiveClaim;
    use waddle_gate::plan::PlanMode;
    use waddle_gate::record::GateDecision;
    use waddle_gate::{Gate, GatePlan, gate::GateShared};
    use waddle_ingest::FakeClock;
    use waddle_types::{ActorKind, ActorRef, ClaimId, MonoNs, ReplanPolicy};

    // A claim as the PLANE grants it: a server-stamped identity, not a bare
    // kind. `ActorRef::of_kind` (the local clutch shape) has empty strings
    // and would hide exactly the defect this pins.
    fn plane_claim(kind: ActorKind, source: &str) -> ActiveClaim {
        ActiveClaim {
            id: ClaimId::new("claim-alloc"),
            source: source.to_owned(),
            actor: Arc::new(ActorRef {
                kind,
                id: "agent:ws-1@plane".to_owned(),
                display_name: "Waddle agent 347".to_owned(),
            }),
            self_initiated: false,
        }
    }

    // Minted the one way production mints it (`ActiveClaim::provenance`),
    // so this covers the whole path from claim state to per-tick tag.
    let agent = plane_claim(ActorKind::Agent, "agent").provenance();
    let site_operator = plane_claim(ActorKind::SiteOperator, "leader_arm").provenance();

    // Each arm carries the decision it must produce, checked before the
    // measurement below: a plan that never took effect would otherwise
    // measure PASSTHROUGH seven times and prove nothing.
    let arms: Vec<(&str, PlanMode, GateDecision)> = vec![
        ("passthrough", PlanMode::Passthrough, GateDecision::Pass),
        ("held", PlanMode::Held, GateDecision::Hold),
        (
            "bypass",
            PlanMode::Bypass {
                provenance: agent.clone(),
            },
            GateDecision::Noop,
        ),
        (
            "reset",
            PlanMode::Reset {
                provenance: agent.clone(),
            },
            GateDecision::ResetActive,
        ),
        (
            "agent_episode",
            PlanMode::AgentEpisode {
                provenance: agent.clone(),
            },
            GateDecision::AgentEpisode,
        ),
        // Claimed with an empty intervention ring: HOLD, and the same two
        // tag clones as any other claimed tick.
        (
            "claimed",
            PlanMode::Claimed {
                provenance: agent,
                blend: None,
            },
            GateDecision::Hold,
        ),
        (
            "claimed_custom_provenance",
            PlanMode::Claimed {
                provenance: site_operator,
                blend: None,
            },
            GateDecision::Hold,
        ),
    ];

    let clock = FakeClock::default();
    let (shared, _stream_tx) = GateShared::new(
        GatePlan::passthrough(MonoNs(0)),
        64,
        0,
        ReplanPolicy::Immediate,
    );
    let (mut gate, mut records_rx) = Gate::new(shared.clone(), clock.clone(), 4096);
    let action = [0.25f64; 14];
    let obs = [0.5f64; 30];

    const CALLS: u64 = 10_000;
    for (name, mode, decision) in arms {
        // Neither the plan swap nor a ring drain is the fast path.
        shared.store_plan(GatePlan {
            mode,
            since: MonoNs(0),
        });
        for _ in 0..64 {
            clock.advance(1_000);
            let _ = gate.gate(&action, Some(0.5), Some(&obs));
        }
        while records_rx.pop().is_ok() {}
        clock.advance(1_000);
        let _ = gate.gate(&action, Some(0.5), Some(&obs));
        assert_eq!(
            records_rx.pop().ok().map(|rec| rec.decision),
            Some(decision),
            "the {name} plan is not the plan the gate ticked under"
        );

        let before = allocations();
        for i in 0..CALLS {
            clock.advance(1_000);
            let out = gate.gate(&action, Some(0.5), Some(&obs));
            std::hint::black_box(&out);
            // Drain periodically so the ring never fills (drop path ≠ fast
            // path).
            if i % 1024 == 0 {
                while records_rx.pop().is_ok() {}
            }
        }
        let counted = allocations() - before;

        assert_eq!(
            counted, 0,
            "the {name} plan arm allocated {counted} times over {CALLS} ticks"
        );
    }
}

/// Observations wider than the 32-dim inline bound spill to the heap:
/// correct (never truncated) but no longer allocation-free. This test
/// documents the spill by round-tripping a 64-dim obs into the record.
#[test]
fn wide_obs_spills_but_round_trips() {
    use waddle_gate::{Gate, GateOutput, GatePlan, gate::GateShared};
    use waddle_ingest::FakeClock;
    use waddle_types::{MonoNs, ReplanPolicy};

    let clock = FakeClock::default();
    let (shared, _stream_tx) = GateShared::new(
        GatePlan::passthrough(MonoNs(0)),
        64,
        0,
        ReplanPolicy::Immediate,
    );
    let (mut gate, mut records_rx) = Gate::new(shared, clock.clone(), 64);
    let action = [0.25f64; 14];
    let obs: Vec<f64> = (0..64).map(f64::from).collect();

    clock.advance(1_000);
    let out = gate.gate(&action, None, Some(&obs));
    assert!(matches!(out, GateOutput::Pass { .. }));
    let rec = records_rx.pop().unwrap();
    assert_eq!(rec.obs.unwrap().as_slice(), obs.as_slice());
}
