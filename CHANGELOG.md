# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Added
- **waddle-runtime (reset-window actuation + plane directives)**: the
  bypass pump (`pumps::spawn_bypass_pump`) gains a RESET arm — while the
  mirror shows `GateMode::Reset` with an active claim, due intervention
  actions (teleop via the existing media intake, agent chunks via the new
  plane arm below) are driven straight to `send`, identical mechanics to
  the BYPASS arm (provenance from the mirror, same chunk shape), no stall
  detection. `forward_server_msg` now handles
  `GateServerMessage.reset_window` (flag `waddle.v0.reset.remote`): ENGAGE
  injects `ClaimGranted` (from the directive's claim) then
  `ResetWindowEngage`; COMPLETE injects `ResetWindowComplete{ok, verified}`
  from the attached result; CANCEL injects `ResetWindowComplete{ok:false}`
  (no dedicated FSM event exists for a plane-initiated cancel — it is
  observably the same as a failed completion). `forward_server_msg` also
  handles `GateServerMessage.intervention_chunk` while the mirror shows
  `GateMode::Reset`: the chunk's steps (dims-validated via
  `ActionChunk::from_pb`) join the intervention ring as timed actions,
  keyed off this arrival plus each step's declared offset; every other
  gate mode still drops this arm silently (the general Claimed-mode chunk
  intake — jitter horizon, `ReplanPolicy` — remains a later milestone).
  The intervention ring's single write end is now Mutex-shared
  (`StreamProducer`) between the media-intake thread and the plane pump,
  since `rtrb` is strictly SPSC and both now need to push. Proven
  end-to-end over a real `ControlPlaneClient` + `InMemoryTransport` script
  (not direct FSM injection): a remote PRE window engaging over teleop and
  completing to READY; a remote POST window engaging an agent, dispatching
  an `intervention_chunk`, and completing to `Terminal`; a POST window
  that is never engaged, timing out for real (short `timeout_ns`, no
  `TimerFired` shortcut) to `Terminal{pinned}` + `post_reset_failed`; the
  same window-timer slot reused correctly across a PRE-then-POST window
  pair inside one episode; and a born-claimed retake successor confirmed
  to never open a remote pre-window even when the session default is
  `Remote`. An MCAP read-back confirms the actuation lands on
  `/waddle/actions` (as the caller-tick's `RESET_ACTIVE` `NoopMarker`,
  tagged with the claimant's provenance — the gate's per-tick record
  remains the only writer onto that topic; the bypass pump's direct `send`
  dispatch is a separate verb call, not itself an MCAP record, unchanged
  from BYPASS).
- **waddle-runtime (reset pump + post-reset recording)**: a new core thread,
  `waddle-reset-hooks` (`pumps::spawn_reset_pump`, mirror-watch like the
  bypass pump), is the single scripted-hook invocation site for resets: a
  LIVE episode in RESETTING that no `start_episode_with` call is driving
  inline gets the effective PRE hook run there (session/per-episode config;
  the trivial `(true, Some(true))` default when none) and its `ResetResult`
  injected, and a LIVE episode in POST_RESET whose effective POST spec is a
  `Hook` gets that hook run there and its `PostResetResult` injected
  (E15/E16) — so blocking `terminate` now completes on post-reset-declared
  episodes. `Remote` specs are untouched by the pump: the FSM's window
  machinery owns them, including the timeout. `start_episode_with` publishes
  the episode's resolved specs (a new internal slot, written before
  `EpisodeOpen`) so per-episode overrides are honored by the pump; hooks run
  off the caller thread (the `ResetHook` type already requires
  `Send + Sync`) and must return — shutdown joins the pump. The mirror
  `Status` gains `pinned_outcome` and `post_reset_failed`, and the sidecar
  now carries the full post-reset record: `post_reset_declared` (stamped
  from `EpisodeOpen`), `post_reset_failed`
  (`Effect::SetPostResetFailed`, permanent, never alters the outcome),
  `post_reset_result` (derived from the emitted `PostResetResult` event),
  and `post_reset_bounds` (opens at the →POST_RESET transition, closes at
  →TERMINAL; left open if force-finalized mid-cleanup).
  `Effect::RunPostReset` is a documented reducer no-op — the mirror-watch
  pump sees the same transition, and user hooks must never run on the
  reducer thread. A Reset-mode gate tick's RESET_ACTIVE `NoopMarker` on
  `/waddle/actions` (wired earlier, untested) is now pinned by an
  end-to-end remote-post-window test.
- **waddle-runtime (reset config surface)**: the first runtime seam for
  reset phases (`waddle-core/crates/waddle-runtime`) — `ResetSpec { Hook(ResetHook) |
  Remote { actor, prompt, timeout_ns } }`; `SessionBuilder::pre_reset`/
  `post_reset` (declaring `post_reset` at all — either variant — is what
  makes an episode detour through `Phase::PostReset`, FSM.md row E14) and
  the previously-missing `verification_mode` setter; `reset_hook` stays as
  an alias for `pre_reset(ResetSpec::Hook(hook))`, now `#[deprecated]` since
  no internal caller exists anywhere in the workspace that would need
  migrating first. `EpisodeOptions {
  pre_reset: Option<Option<ResetSpec>>, post_reset: Option<Option<ResetSpec>> }`
  (outer `None` inherits the session default, inner `None` disables that
  phase for this episode only) plus `Session::start_episode_with`, with
  `start_episode` now a thin default-options delegate. `start_episode_with`
  resolves the effective pre/post specs and injects them onto
  `EpisodeOpen`; a `Hook` (or no spec at all) runs inline on the caller
  thread exactly as before; a `Remote` pre-spec skips the hook/`ResetResult`
  injection entirely and lets the FSM's window machinery (rows E19–E22)
  drive RESETTING to READY or Terminal on its own — no runtime-side
  timeout is added, the FSM window timer owns it. New
  `inline_reset_owner: Mutex<Option<EpisodeId>>` on `SessionInner`, set
  before `EpisodeOpen` for every inline pre-reset path and cleared when the
  call returns, for the reset pump (a later task) to consult so it never
  double-services an episode `start_episode_with` already handled. New
  guard: a predecessor episode that has reached `Phase::PostReset` (its own
  cleanup, past the pinned outcome) is waited out to Terminal and opened
  over instead of erroring `EpisodeActive` — POST_RESET self-resolves, so
  back-to-back rollouts started without an explicit `terminate` + wait no
  longer race the guard. The `Register` feature-flag declaration always
  includes `waddle.v0.reset` (alongside the existing unconditional
  `waddle.v0.core`) and adds `waddle.v0.reset.phases`/`.remote` whenever the
  session-level config declares a matching spec; per-episode `Remote`
  overrides can only narrow what the session already declared, never widen
  it (documented on `EpisodeOptions`, not runtime-enforced — the simpler of
  the two options the brief offered). The reset pump (the actual hook
  invocation for post-reset, and the successor-episode fix for
  reducer-opened retakes), the RESET bypass-pump arm, and
  `forward_server_msg` window handling are explicitly out of scope here —
  reducer/mirror fields are untouched.
- **waddle-protocol (reset-phases vocabulary, inert)**: two new feature
  flags, `waddle.v0.reset.phases` and `waddle.v0.reset.remote` (registered
  in `VERSIONING.md`), gate the wire vocabulary for pre/post-reset phases
  and remote reset windows: `EPISODE_STATE_POST_RESET`, `GATE_MODE_RESET`,
  `ResetKind`, `PostResetResult`, `ResetWindowEvent`/`ResetWindowEventKind`,
  `ResetWindowDirective`/`ResetWindowDirectiveKind`, `EpisodeEvent` arms 16
  (`post_reset`) and 17 (`reset_window`), `GateServerMessage.reset_window`
  (arm 6), `Sidecar` fields 32-35 (`post_reset_declared`,
  `post_reset_failed`, `post_reset_result`, `post_reset_bounds`), and
  `NOOP_REASON_RESET_ACTIVE` — purely additive on the wire; nothing emits or
  reads any of it yet. `waddle-types` mirrors: `EpisodeStateKind::PostReset`,
  `GateMode::Reset`, new `ResetKind { Pre, Post }`, with pb round-trip
  conversions and a unit test per new enum. Every exhaustive match this
  touched across `waddle-fsm`/`waddle-runtime`/`waddle-conformance`/
  `waddle-sidecar` gained an inert arm (behavior unchanged; the FSM/gate/
  runtime behavior for these flags lands in a later change on this branch).
- **waddle-fsm (POST_RESET phase + remote reset windows)**: the FSM now
  implements FSM.md rows E14–E22 and C6/C7 behind the reset-phases flags. An
  episode that declares a post-reset runs a cleanup pipeline INSIDE the
  finishing episode: the terminal outcome is pinned at POST_RESET entry (E14)
  and never changes — a post-reset failure only sets the permanent
  `post_reset_failed` flag (E16), and an estop during cleanup keeps the pinned
  outcome rather than flipping an earned SUCCESS to ABORT (E17). A late
  terminate is rejected and a late END_* mark records the mark without
  transitioning (E14b). Remote reset windows (E19–E22) let a plane-directed
  actor perform a scene reset through the SDK: a window opens in RESETTING
  (pre) or POST_RESET (post), a reset claim is admitted with an actor check
  (C6: a TELEOPERATOR window also admits SITE_OPERATOR, an AGENT window admits
  AGENT only), the claimant engages (lease → claimant, gate → RESET, E20), and
  on completion the lease hands back to the loop client BEFORE the pipeline
  result applies (E21, the deferred-apply invariant), releasing the reset
  claim (C7); a deadline aborts (pre) or pins + flags (post) (E22). The
  central run-closing block is factored into `close_run` (shared by terminal
  and post-reset entry, byte-identical for undeclared episodes) and the E10
  trigger set routes through `request_terminal`, which detours to POST_RESET
  only when declared. New `SessionEvent`s (`PostResetResult`,
  `ResetWindowEngage`, `ResetWindowComplete`), `EpisodeOpen` fields
  (`post_reset`, `pre_window`/`post_window`), `TimerId::ResetWindowTimeout`,
  `AfterLease::{ResetEngageComplete, ResetHandback}`, and
  `Effect::{SetPostResetFailed, RunPostReset}`; the reducer-side handling of
  the new effects stays inert (runtime reset seams land in a later change).
  Undeclared episodes behave exactly per E1–E13 (the additive guarantee); all
  13 conformance fixtures stay byte-identical green.
- **waddle-fsm proptests (I9–I14) + waddle-gate `PlanMode::Reset`**: the
  random-walk harness (`tests/properties.rs`) now drives POST_RESET and
  remote reset windows too — `Cmd` gained `OpenPostReset` (varying pre/post
  window declarations), `PostResetOk`/`PostResetFail`, `WindowEngage`,
  `WindowComplete{ok}` — and checks six new invariants: I9 (PostReset ⇒
  declared), I10 (`pinned_outcome` set-once; PostReset is followed only by
  TERMINAL{pinned}, including via estop), I11 (estop from PostReset ⇒
  TERMINAL ∧ lease Vacant), I12 (`post_reset_failed` monotone; false at
  TERMINAL ⇒ the last post-reset result was ok), I13 (gate RESET ⇒ an active
  claim ∧ phase ∈ {RESETTING, POST_RESET}), I14 (retake acceptance ⇒
  TERMINAL{ABORTED_RETAKE} with no intervening POST_RESET). `Cmd` also gained
  `GateTick`, directly proptesting D7 edge 3: a gate tick landing in
  RESETTING/POST_RESET must never transition the phase (only the READY→RUNNING
  first-gated-action trigger, E6, may). A new deterministic smoke test drives
  a full remote POST-window lifecycle, asserting E21's deferred-apply
  emission order. `waddle-gate` gains
  `PlanMode::Reset { provenance }` (mirroring `Bypass`): `Gate::gate()`
  returns `Noop` and records the new `GateDecision::ResetActive`, same cost
  class as the existing NOOP paths (no locks/syscalls/allocations); the
  runtime reducer wires `GateMode::Reset` to it and renders
  `NoopReason::RESET_ACTIVE` distinctly from `BYPASS_ACTIVE` — the D7 edge 3
  stale-handle protection (a caller ticking `gate()` while a remote actor
  resets dispatches nothing).
- **Bug fix (found by proptest I13) — FSM.md row E19b**: `reset_result` /
  `post_reset_result` are now rejected while a remote reset window is open
  (`waddle-fsm`). Previously the pipeline-hook completion path (E2–E5 /
  E15–E16) could land while a window was OPEN or ENGAGED, abandoning the
  window/claim/lease bookkeeping and leaving `gate_mode == RESET` stuck
  alongside a phase that had already moved past RESETTING/POST_RESET. Not
  reachable through a config-correct runtime (`ResetSpec` is `Hook` XOR
  `Remote`; the reset pump skips hook injection for `Remote`, D4) but guarded
  in `waddle-fsm` anyway per the hollow-frontend rule. Two regression tests
  pin it in `tests/remote_reset_windows.rs`; `docs/FSM.md` §1.4 gains the
  E19b row. New conformance fixture `remote_window_owns_pipeline_result`
  (`fixtures/behaviors/`) asserts the guard on both the pre- and post-window
  path; it is currently runner-skipped (needs `waddle.v0.reset.phases` +
  `waddle.v0.reset.remote`, neither implemented by `waddle-conformance`
  yet — a D6 conformance-runner task) and will activate once that lands.
- **Conformance coverage for reset phases + remote reset windows**
  (`waddle-conformance`): the runner now implements scenario-format.md's
  reset-phase vocabulary — `SUPPORTED_FEATURES` gains
  `waddle.v0.reset.phases`/`waddle.v0.reset.remote`; `episode_open` parses
  the optional `post_reset`/`pre_reset_window`/`post_reset_window` keys; the
  new inject kinds `post_reset_result`, `reset_window_engage`,
  `reset_window_complete`; the state-snapshot document gains
  `episode.post_reset_declared`/`post_reset_failed`/`pinned_outcome` and the
  top-level `reset_window` document; `GateMode::Reset` now maps to
  `waddle-gate`'s real `PlanMode::Reset` instead of a Passthrough
  placeholder. Activating the flags also brought the previously
  runner-skipped `remote_window_owns_pipeline_result` fixture (added above)
  online; running it for the first time surfaced an emission-cursor
  authoring gap (three legitimate transitions never explicitly consumed via
  `expect_emission`, so a later `expect_no_emission` tripped on one of them)
  — fixed by adding the missing assertions, with no change to the guard it
  pins. Nine new fixtures added (`fixtures/behaviors/`), covering FSM.md
  rows E14–E22 and C6/C7: `post_reset_happy`, `post_reset_failure_flags`,
  `post_reset_skipped_when_undeclared` (core-only, the additive guarantee),
  `estop_during_post_reset` (E17), `retake_skips_post_reset` (E18),
  `post_reset_from_intervention` (E14 from INTERVENTION),
  `remote_pre_reset_claim_engage_complete` (the full E19→C6→E20→E21 flow,
  emission order asserted), `remote_post_reset_timeout` (E22), and
  `remote_reset_wrong_actor_denied` (C6). 23/23 scenarios pass (13 original +
  the newly-activated `remote_window_owns_pipeline_result` + these 9).
- New conformance fixture `teleop_dims_mismatch_holds` (`waddle-conformance`,
  gate target): pins the media-intake action-space-validation contract as
  gate-observable — a teleop injection whose flattened width doesn't match
  the declared action space is never dispatched, `gate_tick` returns hold
  however many mismatched packets arrive in the blend window, exactly one
  `Fault{FAULT_KIND_VALIDATION_ERROR}` fires per claim window, and a
  subsequent dims-correct packet still substitutes normally. FSM.md §5
  (IMMEDIATE{blend_ns}) now states the dims-mismatch contract explicitly.
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
- **`Session::episode_done` / `Episode::done` flip at `Phase::PostReset`**,
  not only at Terminal: the terminal outcome is pinned at POST_RESET entry
  (E14), so the rollout is over from the caller's view while only the scene
  cleanup (which self-resolves) is still running. Consequences:
  `terminate_episode`/`Episode::terminate` are now no-ops during POST_RESET
  (a teardown path — e.g. a context-manager `__exit__` racing a plane
  directive — can no longer inject a second Terminate against a pinned
  outcome), and `Episode::outcome()` returns the pinned outcome while the
  cleanup runs (the same value the eventual →TERMINAL carries). A terminate
  that itself detours through POST_RESET still blocks to Terminal,
  unchanged.
- **`FSM.md`** gains §1.3 "Post-reset" (flag `waddle.v0.reset.phases`,
  guard rows E14-E18 + E14b) and §1.4 "Remote reset windows" (flag
  `waddle.v0.reset.remote`, guard rows E19-E22), plus claim-lifecycle rows
  C6/C7 (§2) and two gate-mode-table rows (PASSTHROUGH↔RESET). No FSM
  behavior changes: these rows are prose/normative only in this change: the
  9 fixtures that pin them, and the FSM implementation itself, land together
  in a later change on this branch (the repo rule that guard rows +
  fixtures + a green runner land in one change is satisfied at the
  branch level, not this commit). **`conformance/scenario-format.md`**
  gains the `post_reset_result` / `reset_window_engage` /
  `reset_window_complete` inject kinds, `episode_open`'s optional
  `post_reset?`/`pre_reset_window?`/`post_reset_window?` keys, state-
  snapshot additions (`episode.post_reset_declared`,
  `episode.post_reset_failed`, `episode.pinned_outcome`, top-level
  `reset_window`), and effects-vocabulary additions (`GATE_MODE_RESET`,
  `set_flag{post_reset_failed}`, `arm_timer{reset_window_timeout}`) — all
  gated by the same two flags, documenting what the conformance runner will
  implement in a later task on this branch.
- **Golden fixture amendment (pre-release; no tagged versions exist) —
  `handoff_immediate_mid_chunk`**: its teleop packets now carry Pose
  targets for both parts (7+7=14 values) to stay dims-consistent with the
  declared bimanual composite, instead of single-part Twist packets. The
  fixture previously reached its `blend` expectation only via the
  since-fixed silent dim-truncation defect (see the media-intake fix
  below) — it pinned the defect, not intended behavior. This is a
  deliberate, documented exception to the append-only-goldens rule, made
  possible only because no version has shipped yet; the other 11 existing
  fixtures are untouched.
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
- **Reducer-opened retake successors hung in RESETTING forever**: only
  `start_episode`'s inline path ever ran the pre-reset pipeline, and a
  retake successor is opened by the reducer (`Effect::OpenSuccessor`) with
  no blocked caller — so nothing injected its `ResetResult` and the
  born-claimed successor never reached READY. The reset pump now services
  it (regression-tested by driving a retake through the runtime and
  asserting the successor passes through reset to READY, with the
  session's PRE hook run exactly once for it).
- **Verb-registration validation at session build**: `SessionBuilder::build`
  now fails fast with a new `RuntimeError::MissingVerb` instead of letting a
  missing callable surface only at first dispatch. Previously, the default
  handoff policy (HOLD_FIRST) issues `Verb::Hold` on every engage; with a
  media plane wired but no `hold` callable registered, dispatch failed
  `NotRegistered` silently and the engage fail-closed only at the 10s engage
  timeout — the teleoperator's clutch did nothing, with no diagnosable
  error. `hold` is now required at build time whenever the handoff policy is
  HOLD_FIRST and the session has a live engage path; `send` is now required
  under that same condition, independent of handoff policy (the bypass pump
  can drive `Verb::Send` directly once a claimed loop stalls). A live engage
  path is a wired media plane **or** `hold`/`send` registered in `Control`
  directly — `grant_and_engage` (the local-intervention convenience,
  exported from the crate root and used by "tests and local intervention
  sources") injects `ClaimGranted`/`Engage` with zero dependency on
  `self.media`, so a session that registers `send` for local intervention
  without ever calling `.media(...)` is exactly as live an engage path as
  one wired to a media plane, and is now checked the same way. Both errors
  name the fix directly (e.g. "handoff HOLD_FIRST requires a registered
  `hold` verb — register one in your Control, or choose a different handoff
  policy"). Sessions built with no Control and no media plane (the
  descriptors-only / minimal-local case, including the PyO3 shim's
  all-None-verbs `create_session`) are unaffected and stay buildable — that
  shape has no build-time-visible engage path at all; `grant_and_engage`'s
  own doc comment now carries an explicit safety note that direct callers
  outside that shape are still responsible for registering `hold`/`send`
  themselves. A missing `estop` is deliberately never build-fatal, but the
  degradation is now recorded on the status mirror
  (`Status::estop_unregistered`) so it stays observable. The `hold` check
  reasons about the *effective* handoff policy, not the raw declared enum
  variant: `waddle_fsm::begin_engage` silently degrades a declared
  `HandoffPolicy::Immediate` to HOLD_FIRST on the very first engage whenever
  the robot's action space contains a delta component (FSM.md §5 — delta
  spaces refuse mid-chunk splice entry), so `build()` now applies that same
  degrade before checking `hold`, closing a gap where a declared-IMMEDIATE
  session over an `EePoseDelta`/composite-with-delta space built clean and
  then stalled at the first engage the same way the undegraded bug did.
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
- **Conformance runner — `teleop_action` injection only read the first part
  target**: `waddle-conformance`'s scripted intervention-stream flattening
  now concatenates every part target in packet order (pose → 7 values
  wxyz, twist → 6), matching production `flatten_packet` semantics.
  `scenario-format.md`'s `teleop_action` payload never pinned "first target
  only"; the narrower reading was a runner defect, surfaced by the
  media-intake dims-validation fix above.
- **GripperSpec never applied**: the teleop gripper command (normalized
  0..1, 1 = open — the media-plane convention) is now mapped through the
  session's declared `GripperSpec` at intake — linearly onto
  `[closed_value, open_value]` for `Parallel`, thresholded at 0.5 for
  `Suction` — instead of being copied onto the wire verbatim. No declared
  spec still passes the command through unchanged.
- **Clutch claim provenance mislabeled as non-teleop**: a clutch edge on the
  media plane (the leader-arm/console-clutch takeover path) self-initiates a
  claim; `waddle-runtime`'s `SessionBuilder` now defaults that claim's actor
  to `ActorKind::Teleoperator` (source `"teleop-clutch"`) instead of
  inheriting `waddle-fsm`'s `SiteOperator`/"custom" default, so the
  reducer's provenance mapping records these interventions as teleop —
  provenance-labeled training data (DAgger pairs) was silently mislabeled,
  and the N17 actor vocabulary was violated. `waddle-fsm`'s own default is
  unchanged (fixture stability); `SessionConfig` gains `clutch_actor`
  (alongside `clutch_source`), and the new `SessionBuilder::clutch_identity`
  setter lets integrators override both.
- **Jitter buffer — one shared reorder cursor for two independent
  producers**: the intervention ring's `JitterBuffer` kept a single
  session-wide `last_popped_seq` watermark, but two producers write into
  it — the media-intake thread (teleop, seq = wire
  `TeleopStreamPacket.seq`) and the plane pump's reset-window
  `intervention_chunk` arm (agent chunks, seq = a fresh pump-local counter
  starting at 0). An ordinary teleop claim earlier in the session (nothing
  to do with any reset window) would advance that one shared cursor well
  past 1, so the first agent-chunk step of a *later* reset window — the
  exact `pre_reset=TeleopReset`/`post_reset=AgentReset` shape the design
  suggests as normal — would look "late" and be silently, permanently
  dropped, with the window then just timing out and no diagnostic trail
  (`dropped_late` has no readers). `JitterBuffer` now keeps one reorder
  cursor per `TimedAction::channel` (`StreamChannel::Teleop` /
  `StreamChannel::AgentChunk`), so neither producer's activity can starve
  or drop the other's arrivals. Regression-tested by driving an ordinary
  teleop claim to completion (advancing the teleop channel's cursor well
  past a small number) and then confirming a later Remote POST window's
  agent chunk still dispatches.
- **`intervention_chunk` during a reset window — malformed chunks dropped
  with zero signal**: a wire chunk that fails `ActionChunk::from_pb`
  (dims mismatch, wrong target variant, an Opaque space, …) during a
  Reset-mode window was silently ignored, unlike the parallel teleop path
  (which raises `SessionEvent::InterventionRejected` on a dims mismatch).
  Since this is the only actuation channel for an Agent-kind reset window
  (no teleop fallback), `forward_server_msg` now logs a `tracing::warn!`
  naming the rejection instead of dropping it with no trace; behaviorally
  verified (no dispatch, no corruption, the window still resolves
  normally on the plane's COMPLETE).

## Stowed changelogs

_None yet. On first release, the released section moves to
`docs/changelogs/CHANGELOG-<artifact>-<version>.md` and is linked here._
