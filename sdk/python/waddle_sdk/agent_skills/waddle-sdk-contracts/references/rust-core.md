# Rust core and protocol contracts

## Normative sources

In a source checkout, use this precedence:

1. `waddle-protocol/docs/GLOSSARY.md` for words and ownership.
2. `waddle-protocol/docs/FSM.md` for lifecycle guards and transitions.
3. `waddle-protocol/docs/VERSIONING.md` for compatibility and feature flags.
4. Versioned protobuf comments, conformance format, and append-only fixtures.
5. Public design rationale only where the normative documents are silent.

Protocol evolution is append-only within `waddle.v0`. Do not renumber or reuse removed fields or enum values. Reserve both number and name. Unknown enum values mean a newer peer and require the field's conservative fallback.

## Crate responsibilities

- `waddle-types`: protocol-domain types; no I/O, clocks, threads, or async runtime.
- `waddle-fsm`: the authoritative episode, claim, lease, handoff, and intervention state machine.
- `waddle-gate`: synchronous action selection and provenance tagging on the real-time path.
- `waddle-ingest`: the sole production source of paired session/wall timestamps.
- `waddle-tripwire`: watchdogs that request declared holds; never the owner envelope.
- `waddle-sidecar` and `waddle-codecs`: durable semantic records and independent encodings.
- `waddle-controlplane` and `waddle-media`: transport implementations behind features.
- `waddle-runtime`: lifecycle composition and thread ownership.
- `waddle-ffi`: the narrow native binding surface.
- `waddle-conformance`: executable behavioral and wire compatibility contracts.

Keep low-level crates free of I/O and async dependencies. Confine Tokio to transport implementations. Do not expose Tokio types through public core signatures.

## Gate constraints

`Gate::gate()` is synchronous and allocation-free on supported inline dimensions, including claimed/bypass paths. Do not add locks, syscalls, heap-owned strings, or transport work to it. Variable provenance and part identifiers must be shared values minted off the gate thread.

Every action source crosses the same gate and owner envelope. No language frontend, media path, hardware adapter, or helper may create an alternate dispatch route.

The runtime's `playback` module accounts for finite joint-reference buffers using
the existing `ActionChunk` types. It distinguishes dispatch acknowledgement from
measured arrival and refuses stale starts, missing acknowledgements, late steps
and cancellation without retry. `Session.start_playback` validates the declared
scope and runs local observation/gate/owner callbacks on the existing serialized
control thread. Hold/e-stop, episode closure and shutdown drain the reference loop;
original faults and stop outcomes stay separate. It creates no authority or device
owner. Python `Run.start_playback` uses these callbacks through the ordinary local
owner; connector integration is still pending. See
`docs/core/finite-reference-playback.md` in a source checkout for the contract.

Optional `PacedCurve` evaluation uses that same worker: exact supplied polynomial,
acknowledged progress, bounded measured headroom and minimum local submission
spacing. Fixed offsets retain their original lateness guard. Keep trajectory
validation outside this evaluator and preserve absolute deadlines, native gates
and original unknown outcomes.

Optional `FeedbackPolicy` uses that same worker for bounded measured holding,
measured-row passthrough, velocity-hint direction and encoder-window settling.
Keep selected curve coordinates unchanged and distinct arrival/stationary-miss
outcomes. Original owner admission and deadline apply to every adapted reference.
Settling evidence retains sample spans and raw velocity without inferring contact.

## Conformance change protocol

When behavior changes:

1. Amend the governing normative FSM prose or guard row.
2. Add an asserting scenario under the append-only behavior fixtures.
3. Update the reference implementation.
4. Run the complete conformance suite and relevant feature-gated builds.

Do not modify an existing golden to make a new implementation pass. Add a superseding fixture and mark the old case deprecated when a contradiction is proven.
