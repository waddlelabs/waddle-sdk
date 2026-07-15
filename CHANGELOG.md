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
- **waddle-core (gate-record persistence)**: the reducer now drains the
  per-episode gate-record ring every wake and persists it to the Local-mode
  MCAP — the obs as `ObservationUpdate` on the new `/waddle/observations`
  topic, the decision as a single-step `ActionChunk` on `/waddle/actions`
  (Noop/Hold write `NoopMarker` actions, making the topic the complete
  per-tick trace). "The reducer owns all recording" is now structural. New
  `waddle_types::unflatten_action` (exact inverse of the flattening path)
  and `ProvenanceTag::to_pb`; `McapEpisodeWriter::write_observation`.
- **`sdk/` — the Python `waddle-sdk` frontend (Tier-1 minimum)**: the
  six-line tutorial loop against a real customer loop with Local recording,
  fully offline (no control plane, no relay). PyO3 0.29 shim binding
  `waddle-runtime` directly (abi3-py310, own cargo workspace with path-deps
  into waddle-core), `uv`/maturin packaging. Public surface: `init` /
  `rollout` / `shutdown`, `Control` (five verb callables → derived grants),
  `Handoff`, `Outcome`, and pure-Python descriptor sugar (`Robot`,
  `JointSpace`, `EEDelta`, `Composite`, `Opaque`, `Camera`, `Chunking`,
  `Gripper`) compiling to canonical proto3 JSON. `ep.gate(action, obs)`
  returns "what you should send, or None if you must not send" (Pass
  returns the caller's exact object; Noop/Hold return None); exiting
  `rollout()` non-terminal aborts, never succeeds. Private `waddle._testing`
  hooks (engage/release/push_teleop over the loopback media plane) drive the
  intervention pytest; the nominal pytest reads the episode MCAP back as the
  Python-side proof of obs logging + gate-record persistence. New core
  helper `waddle_runtime::release_claim` (counterpart of
  `grant_and_engage`).
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

### Removed
- `Episode::drain_records` and the episode-held record consumer: gate
  records now flow to the reducer (via an internal hand-off slot) and land
  in the episode MCAP; callers no longer see the ring.

### Fixed
- The task passed to `Session::start_episode` now reaches the episode
  sidecar (it was previously dropped after the reset hook; sidecars always
  recorded an empty task).
- Episode-lifecycle hardening (from an adversarial review of this series):
  `start_episode` while an episode is live now returns
  `RuntimeError::EpisodeActive` instead of destroying the live episode's
  recording and blocking forever; a stale `Episode` handle can no longer
  write records into a later episode's MCAP (the fresh ring reaches the
  reducer before the open event; stale leftovers are discarded, while
  retake successors still inherit the caller's ring) or terminate a later
  episode (`terminate` is a no-op unless the episode is still live);
  `Episode::done` now also flips on session shutdown, so the tutorial loop
  cannot spin forever after `waddle.shutdown()`. New
  `Episode::records_dropped()` surfaces ring overflow (training-data
  loss). A gated action that does not fit the declared space (raw teleop
  stream ahead of closed-side retargeting) now records an action-less
  chunk instead of silently skipping the tick, keeping `/waddle/actions`
  obs-aligned. The Python shim's `Session` also shuts the core down safely
  when dropped without `shutdown()`, and `terminate` no longer holds the
  episode lock across its blocking wait (other threads' `gate`/`done`
  stay responsive).
- **Media intake — stale-backlog replay**: intake now pushes a teleop pose
  into the intervention ring only while a claim is active; previously every
  pose was queued regardless of claim state, so up to a whole ring's worth
  of pre-claim poses could all become "due" the instant a claim engaged and
  replay as stale motion while fresh packets were dropped by the full ring.
- **Media intake — no action-space validation on injected teleop actions**:
  a flattened teleop action whose width doesn't match the session's
  declared action space is now dropped at intake instead of substituted
  verbatim, with a `Fault{VALIDATION_ERROR}` recorded once per claim window
  (not once per packet at 60-90 Hz) via a new
  `SessionEvent::InterventionRejected`. `waddle-gate`'s blend step no longer
  zip-truncates a dims mismatch between the blend anchor and the
  intervention target (a false "validated upstream" comment is now true in
  practice, and a real defense-in-depth guard on the rare mismatch that
  still reaches it); it now returns no blend and the gate falls back to
  Hold.
- **GripperSpec never applied**: the teleop gripper command (normalized
  0..1, 1 = open — the media-plane convention) is now mapped through the
  session's declared `GripperSpec` at intake — linearly onto
  `[closed_value, open_value]` for `Parallel`, thresholded at 0.5 for
  `Suction` — instead of being copied onto the wire verbatim. No declared
  spec still passes the command through unchanged.

## Stowed changelogs

_None yet. On first release, the released section moves to
`docs/changelogs/CHANGELOG-<artifact>-<version>.md` and is linked here._
