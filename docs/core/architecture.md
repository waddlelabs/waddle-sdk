# Rust core architecture

`waddle-core` is the reference implementation of the public protocol. Pure decision
logic is separated from clocks, I/O, and threads so the same behavior can be tested
without a device or network.

| Crate | Responsibility |
|---|---|
| `waddle-types` | Generated wire types and validated domain types |
| `waddle-fsm` | Pure episode, claim, and lease transition functions; the behavioral conformance target |
| `waddle-gate` | Synchronous action gate, substitution, hold/no-op decisions, and bypass |
| `waddle-tripwire` | Local watchdog evaluation that requests declared safety verbs |
| `waddle-ingest` | Session clock, paired timestamps, source offsets, and bounded sample storage |
| `waddle-codecs` | Independently versioned dialect descriptors and wire codecs |
| `waddle-sidecar` | Semantic sidecar and local MCAP recording |
| `waddle-media` | Media transport traits and optional LiveKit implementation |
| `waddle-controlplane` | Registration, negotiation, reconnect, heartbeats, and optional gRPC transport |
| `waddle-runtime` | Composition root and owner of core worker threads |
| `waddle-ffi` | Unstable C ABI used by language bindings |
| `waddle-conformance` | Runner for normative behavioral scenarios and golden fixtures |

## Load-bearing boundaries

- `waddle-types`, `waddle-fsm`, `waddle-gate`, and `waddle-codecs` contain no OS
  clocks, threads, or I/O.
- Only `waddle-ingest` reads OS clocks in production code.
- Tokio is confined to the optional gRPC and LiveKit transport implementations; no
  Tokio type appears in a public signature.
- `Gate::gate()` stays synchronous and wait-free in passthrough. Its common action and
  observation widths are allocation-free.
- `waddle-runtime` owns thread lifecycles. A language binding creates one runtime and
  shuts it down deterministically; it does not grow a second authority machine.

Control-plane reconnect backoff uses the owning session's monotonic `Clock` through
`ControlPlaneClient::spawn_with_clock`. The legacy `spawn` convenience captures an
ingest `SessionClock`. Delayed/coalesced wakeups count toward the actual reconnect
deadline rather than accumulating requested sleep slices. Commands are still
classified into the bounded offline buffer on every wakeup; registration,
per-connection negotiation and authority checks remain unchanged. OS-clock reads
remain inside ingest.

## Build and conformance

From `waddle-core/`:

The pinned build toolchain is Rust 1.98.1, including the upstream Mach-O
LINKEDIT alignment correction for modern macOS loaders. The language MSRV
declared in the workspace stays separate from the qualified build toolchain.

```console
cargo test --workspace --locked
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo fmt --check
```

Transport feature passes are separate because featureless builds intentionally remain
free of their dependencies. The repository's contributor instructions list the full
required matrix.
