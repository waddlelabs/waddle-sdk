# waddle-core

The Rust reference implementation of [waddle-protocol](../waddle-protocol/):
episode/claim/lease state machines, the gate, tripwires, sidecar + MCAP
recording, codecs, the control-plane client, and the `libwaddle` C ABI.

No system `protoc` needed (protox compiles the schemas at build time) and no
async runtime anywhere — dedicated named threads and channels.

```bash
cargo test --workspace                 # includes the conformance suite
cargo run -p xtask -- gen-header       # emits target/include/waddle.h
cargo bench -p waddle-gate             # gate fast-path tracking
```

Crate map (design doc §3.2): `waddle-types` (wire + validated domain layer),
`waddle-fsm` (pure machines — THE behavioral conformance target),
`waddle-gate` (the fast path), `waddle-tripwire`, `waddle-ingest` (the only
OS-clock reader), `waddle-media` / `waddle-controlplane` (transport seams
with in-memory implementations; LiveKit/tonic deferred behind features),
`waddle-sidecar`, `waddle-codecs` (independently versioned),
`waddle-runtime` (composition root), `waddle-ffi` (→ `libwaddle`, ABI
unstable per N5), plus `waddle-conformance` (the scenario runner) and
`xtask` (tooling).
