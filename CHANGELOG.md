# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Added
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
- **waddle-core M7 (partial, `waddle-runtime`)**: five-verb dispatch thread —
  serialized integrator callables, catch_unwind isolation, latency sampling
  for heartbeat proxies, estop priority path that never queues.
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
