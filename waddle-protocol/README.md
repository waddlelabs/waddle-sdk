# waddle-protocol

The Waddle standard: protobuf schemas, the episode/claim/lease FSM
specification, the sidecar schema, and conformance fixtures for a supervision
layer over real-world robot policy rollouts.

**This package depends on nothing, and everything depends on it.** It is
implementable without `waddle-core` — that is the point. An independent
implementation that passes the fixtures in [`fixtures/`](fixtures/) speaks
Waddle.

## The six files

| File | Owns |
|---|---|
| [`proto/waddle/v0/descriptors.proto`](proto/waddle/v0/descriptors.proto) | Declarations: robot, action spaces, cameras, series, grants, verbs — plus the base types (geometry, time, actors) |
| [`proto/waddle/v0/control.proto`](proto/waddle/v0/control.proto) | The write path: actions, chunks, handoff policy, provenance, faults, verb invocation |
| [`proto/waddle/v0/episode.proto`](proto/waddle/v0/episode.proto) | The episode FSM, claims, leases, intervention lifecycle, reset verification, judgments, the event stream |
| [`proto/waddle/v0/sidecar.proto`](proto/waddle/v0/sidecar.proto) | The per-episode semantic record and recording modes |
| [`proto/waddle/v0/services.proto`](proto/waddle/v0/services.proto) | The gRPC control plane (Register, Negotiate, GateActions, Heartbeat, …) |
| [`proto/waddle/v0/media.proto`](proto/waddle/v0/media.proto) | Media-plane data-topic payloads (teleop stream, clutch, marks, telemetry) |

## Normative documents

- [`docs/GLOSSARY.md`](docs/GLOSSARY.md) — the single vocabulary, internal and
  external. Frozen; amended only by reviewed PR.
- [`docs/FSM.md`](docs/FSM.md) — state diagrams and transition-guard tables.
- [`docs/VERSIONING.md`](docs/VERSIONING.md) — feature-flag negotiation and
  what "breaking" means here.
- [`conformance/scenario-format.md`](conformance/scenario-format.md) — the
  behavioral-scenario schema that `fixtures/behaviors/` files follow.

The design rationale (including its adversarial stress-test history) is
preserved unchanged at
[`docs/rationale/waddle_api_design_doc.md`](docs/rationale/waddle_api_design_doc.md).
Where rationale and normative docs diverge, the normative docs win.

## Consuming the schemas

No generated code is checked in here — ever. Compile the `.proto` files in
your build:

- **Rust**: `protox` + `prost-build` (what `waddle-core` does; no system
  `protoc` needed).
- **Anything else**: `protoc`/`buf` per your ecosystem. `buf.yaml` configures
  lint (STANDARD) and breaking-change checks (FILE) for CI.

## Conformance

Three tiers (see [`conformance/README.md`](conformance/README.md)):

1. **Wire fixtures** (`fixtures/wire/`, `fixtures/sidecars/`) — golden
   messages in canonical proto3 JSON, compared semantically.
2. **Behavioral scenarios** (`fixtures/behaviors/`) — declarative
   inject/expect scripts against the FSM and gate.
3. **Timing/soak benches** — defined in
   [`conformance/timing-envelopes.md`](conformance/timing-envelopes.md);
   published numbers are non-warranty bench observations. The only binding
   conformance statement for a deployment is `waddle doctor` on the actual
   rig.

## License

Apache-2.0.
