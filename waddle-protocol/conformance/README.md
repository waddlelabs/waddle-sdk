# Conformance

"Speaks Waddle" is a testable claim, made in three tiers. Tiers 1 and 2
certify **logic**; tier 3 covers **timing** — fixtures verify logic, not
physics (N3), and a frontend can pass every fixture while missing every
deadline. For any *deployment*, the only binding conformance statement is
`waddle doctor` run on the actual rig.

Vocabulary per `../docs/GLOSSARY.md`; evolution rules per
`../docs/VERSIONING.md` (goldens are append-only; changing one is breaking).

## Tier 1 — wire and golden fixtures

Inputs: `../fixtures/wire/` (schema-canonical messages) and
`../fixtures/sidecars/` (complete golden `Sidecar` records). Envelope format
and canonical JSON conventions: `../fixtures/README.md`.

- An implementation MUST parse each fixture's `message` into its own typed
  binding and compare **semantically — never textually** (parse → message →
  field-by-field compare). Byte- or string-comparing JSON is non-conforming.
- Round-trip requirement: parse → serialize → parse MUST be a fixed point.
- What this tier certifies: schema bindings, the pinned conventions (wxyz
  quaternions, `int64` nanosecond times, normative `Composite` part order),
  and reserved-field discipline.

## Tier 2 — behavioral scenarios

Inputs: `../fixtures/behaviors/`, whose JSON schema is
[`scenario-format.md`](scenario-format.md) (normative — if a runner and that
document drift, the document wins and the runner is wrong).

Scenarios are declarative inject/expect scripts against the episode FSM and
the gate. They are transport-free: no sockets, no threads, no real clocks —
time advances only by injection.

**The runner contract.** Any implementation is runnable if it exposes three
operations over its FSM/gate under test:

| Operation | Meaning |
|---|---|
| **inject(input)** | deliver one scripted input: an action or chunk, a verb result, a claim/episode directive, a tripwire condition, a clock advance |
| **snapshot()** | return the currently observable state: `EpisodeState`, `GateMode`, active claim and lease, grant statuses |
| **drain-emissions()** | return the ordered `EpisodeEvent`s and outbound messages emitted since the last drain |

`waddle-conformance` (a crate in the `waddle-core` workspace) is the
**reference runner**: point it at an implementation of this contract and get
a pass/fail report per scenario. Independent implementations MAY reimplement
the runner from `scenario-format.md` instead of linking Rust.

What this tier certifies: the semantics that make frontends interchangeable —
claimed-while-stalled bypass, mid-chunk handoff per each `HandoffPolicy` arm,
backend-partition degradation, reset-verification failure and the permanent
`reset_unverified` flag, retake and born-claimed successor accounting.

## Tier 3 — timing and soak benches (N3, N16)

Fixtures verify logic, not physics. Tier 3 measures each frontend against the
bench dimensions defined in [`timing-envelopes.md`](timing-envelopes.md):
gate passthrough latency, engage-to-first-intervention-action, blend-window
adherence, deadman/staleness cutoff, and hold round-trip — run as sustained
soaks with jitter percentiles, not one-shot microbenchmarks.

Published envelopes are **observed bench measurements under stated
conditions, with explicit non-warranty language** (N16). Safety-adjacent
numbers (hold, e-stop) are binding only as measured per deployment by
`waddle doctor` — never from a brochure. Doctor exercises the declared
integration end-to-end: no-op chunk round-trip, `hold()` latency, clock-skew
and timestamp-monotonicity checks, URDF-vs-declaration validation, and the
NOOP-compliance test for advisory-lease integrations (N7), with runtime
dual-write detection (N14) covering what a doctor-time test cannot.

## Certifying a third-party implementation or adapter in CI

1. **Pin the standard.** Vendor or submodule `waddle-protocol` at a tagged
   release and compile the schemas in your build — no generated code ships in
   this repository.
2. **Tier 1.** For every file in `../fixtures/wire/` and
   `../fixtures/sidecars/`: parse, round-trip, semantic-compare. Enumerate
   the directories at test time — fixtures are append-only, and a hand-kept
   list silently rots.
3. **Tier 2.** Implement the runner contract (inject / snapshot /
   drain-emissions) over your FSM/gate, or embed the reference
   `waddle-conformance` runner if you can link Rust. Run every scenario in
   `../fixtures/behaviors/`.
4. **Tier 3** (frontends that own a gate loop). Run the benches of
   `timing-envelopes.md` on representative hardware; publish your envelope
   table with conditions, dates, and the mandatory non-warranty language.
   For adapters (a camera tap, a robot transport), tiers 1–2 plus a
   `waddle doctor` run on the target rig is the certification bar.
5. **State the claim precisely.** A certification claim names the
   `waddle-protocol` release, the feature flags exercised, and the tiers
   passed. "Passes conformance" without a tier list is not a conformance
   claim.

This is how third parties extend the open surface without the control plane
ever seeing a malformed stream: adapters certify against the goldens in CI,
and every new behavior lands with a guard-table row in `../docs/FSM.md` plus
an asserting scenario (see `../docs/VERSIONING.md` §6).
