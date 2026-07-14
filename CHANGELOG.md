# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Added
- **waddle-core (obs logging)**: `gate()` now takes the observation the
  caller computed its action from (`obs: Option<&[f64]>`) and records it on
  every decision arm — Pass records are the training pairs;
  Substitute/Blend records are pre-labeled DAgger pairs. New
  `waddle_types::ObsValues` (inline to 32 dims; wider observations spill to
  the heap, never truncate); `GateRecord.obs`. The alloc-free proof now
  covers a 30-dim obs; new `gate_passthrough_14dof_obs30` bench.
- **waddle-core M2 (`waddle-fsm`)**: pure Mealy session machine (episode ×
  claim × lease × grant health) implementing every FSM.md guard row — reset
  verification modes (N12), retake → born-claimed successor under the
  surviving claim (N2/N18), per-policy handoff sub-protocol with delta-space
  degradation, clutch self-initiated claims, estop revoke-all, grant liveness
  with hysteresis and never-mid-lease demotion (N11), dual-write hold
  requests (N14), FSM-owned bypass transitions. 256-case random-walk proptest
  holding eight invariants.
- **waddle-core M3 (`waddle-ingest`, `waddle-gate`, `waddle-tripwire`)**:
  SessionClock (sole OS-clock reader) + FakeClock + per-source offset
  estimation; the gate fast path (ArcSwap plan + SPSC stream + jitter buffer
  + blend math + NOOP bypass + DivergenceDetector), proven allocation-free
  over 1M passthrough calls; tripwire evaluator + heartbeat watchdog on
  dedicated threads, edge-triggered, verbs requested never enforced.
- **waddle-core M4–M5 (`waddle-codecs`, `waddle-sidecar`)**: codec
  trait/registry with version pinning, mandatory round-trip certification and
  signing seam (N4/N15) + lerobot-async/openpi dialects; sidecar records as
  wire-exact canonical JSON (prost-reflect over the embedded descriptor set),
  span derivation from the event stream, atomic writes + manifest, Local-mode
  MCAP recorder with clock-anchor metadata, Reference-mode resolver seam.
- **waddle-core M6 (`waddle-controlplane`, `waddle-media`)**: control-plane
  client thread (backoff reconnect, in-order offline replay, N11 heartbeat
  proxy signals, N7/N13 negotiation) over a scriptable in-memory transport;
  MediaPlane trait + loopback with the media.proto topic table.
- **waddle-core M7 (`waddle-conformance`, `waddle-runtime`)**: the
  behavioral-scenario runner implementing `conformance/scenario-format.md`
  exactly (canonical-JSON matching via prost-reflect, virtual time, FSM and
  gate targets with a reference bypass pump) — **all 12 protocol scenarios
  pass with zero changes to waddle-fsm/waddle-gate**, plus mutation tests
  proving the runner detects wrong values/order/forbidden emissions; the
  runtime Session/Episode API — five-verb dispatch thread (serialized,
  catch_unwind, estop priority path), single-writer FSM reducer interpreting
  effects, per-episode sidecar + MCAP finalization, blocking-through-reset
  episode open, bypass pump, media intake, plane pump, ordered shutdown.
  e2e: nominal recording, teleop engage/substitute/release, and the
  claimed-while-stalled NOOP-spectator contract.
- **waddle-core M8 (`waddle-ffi` → libwaddle, `xtask`)**: the C ABI — opaque
  handles, pb-bytes configuration, five-verb C callbacks invoked only on the
  dispatch thread, status codes + thread-local `waddle_last_error`,
  panic-proof entry points; `cargo run -p xtask -- gen-header` emits
  `target/include/waddle.h` (marked `WADDLE_ABI_UNSTABLE` per N5); verified
  by Rust round-trip tests and a real C caller compiled with gcc against the
  generated header and linked to `libwaddle.so`.

### Changed
- **Signatures (pre-1.0 / ABI unstable per N5)**: `Gate::gate`,
  `Episode::gate`, and the C ABI `waddle_gate` gained the obs parameter
  (`obs`/`obs_len` on the ABI; NULL or 0 = no observation). Header
  regenerated.
- Six behavioral fixtures aligned to implementation emission order where
  FSM.md deliberately does not pin intra-step order (each documented in its
  fixture description); `backend_partition_degradation` now asserts buffer
  counts + reconnect re-promotion (transport replay is waddle-controlplane's
  tested contract).
- **waddle-protocol v0**: the six schemas (`descriptors`, `control`,
  `episode`, `sidecar`, `services`, `media` under `proto/waddle/v0/`) with
  amendments N1–N18 applied; normative docs (GLOSSARY.md, FSM.md with
  transition-guard tables, VERSIONING.md); conformance tier docs and the
  normative behavioral-scenario format (`conformance/scenario-format.md`);
  buf configs. Design doc archived unchanged at `docs/rationale/`.
- **waddle-core M0–M1**: cargo workspace (11 crates + conformance runner +
  xtask; edition 2024; clippy `disallowed-methods` enforcing the clock
  discipline) and `waddle-types` — protox+prost build (no system protoc),
  embedded FileDescriptorSet, and the validated domain layer: `Stamp`
  dual-clock type, opaque ids, action-space validation (must-declare
  rotation/delta conventions, composite depth pin, frame tagging), wire→flat
  action flattening (declaration-order composites, wxyz quaternions),
  grants/handoff/provenance/outcome domain enums.
- Monorepo bootstrap: git repository, Apache-2.0 license, agent bootstrap
  (`CLAUDE.md`), changelog discipline (this file + `docs/changelogs/`).

## Stowed changelogs

_None yet. On first release, the released section moves to
`docs/changelogs/CHANGELOG-<artifact>-<version>.md` and is linked here._
