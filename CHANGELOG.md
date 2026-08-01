# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Added
- **waddle-protocol/waddle-core (agent-invited episodes, new feature flag
  `waddle.v0.agent`)**: a customer can now ask Waddle to drive an episode
  rather than driving it themselves — `Session::run_agent(prompt, timeout_ns,
  opts)` opens an *agent-invited* episode and blocks until it terminates,
  returning `AgentOutcome { outcome, episode_id, recording_ref, detail }`.
  **The invite adds no authority concepts.** It is one `EpisodeEvent`
  (`AgentInviteEvent { prompt, timeout_ns }`, arm 18) forwarded to the plane
  like any other emission; the hosted agent then claims the episode with the
  EXISTING intervention machinery (`ClaimDirective{GRANT, ACTOR_KIND_AGENT}`
  → engage), streams chunks on the EXISTING `intervention_chunk` arm, and
  finishes with the EXISTING `EpisodeDirective{MARK_DONE}` +
  `ClaimDirective{RELEASE}`. Everything else about the episode — E7 engage,
  chunk handoff, E10 termination, both reset phases — applies verbatim.
  - **Protocol**: `AgentInviteEvent` (episode.proto);
    `AgentTaskUpdate { episode_id, kind, detail, recording_ref,
    directive_id? }` with `AgentTaskUpdateKind`
    (UNSPECIFIED/QUEUED/DENIED/COMPLETED) as `GateServerMessage` arm 7 — the
    plane's status channel for the ask itself, distinct from the episode's
    own timeline; `NOOP_REASON_AGENT_EPISODE = 4` (control.proto). All
    append-only; registry row in VERSIONING.md.
  - **FSM.md §1.5** (guard rows E23–E26/E26b, C8): E23 opens the episode,
    emits the invite, and arms `agent_invite_timeout`; **E24** — the caller's
    own `gate()` ticks NEVER dispatch while no claim is engaged (plan Noop,
    reason `NOOP_REASON_AGENT_EPISODE`; no fault, no state change), which is
    what makes "you asked, Waddle drives" honest rather than a race between
    two writers; **E25/E26** (deadline elapsed, or a plane `DENIED` while the
    invite is open) are declared **members of E10's trigger set** with the
    outcome fixed to ABORT, so E14 routes them through the episode's normal
    termination (TERMINAL{ABORT}, or POST_RESET{ABORT pinned} when
    post-reset is declared) instead of around it; **E26b** records a late
    `DENIED` as an event only, so it can never disturb a pinned outcome.
    **C8** admits `ACTOR_KIND_AGENT` claims only, and records the refusal of
    any other actor as `claim{DENIED}` — a declared reset window on an
    agent-invited episode still admits its teleoperator under C6, which
    C8 does not touch. Two latches are emission-invisible state:
    `agent_engaged` (set by the first agent ENGAGE; never re-arms the invite
    timer on a release/re-engage) and `invite_aborted` (set by E25/E26 and
    nothing else, so an embedder can tell "the ask went unanswered" from
    "the episode broke for unrelated reasons" without parsing reasons).
    The invite closes on the first of an agent ENGAGE or any exit from
    {RESETTING, READY, RUNNING}; every closing row cancels the timer, and a
    stale expiry after close is discarded.
  - **Conformance**: eight new scenarios in `fixtures/behaviors/`
    (`agent_invite_happy`, `agent_caller_tick_noop`, `agent_invite_timeout`,
    `agent_invite_timeout_post_reset`, `agent_invite_denied`,
    `agent_invite_denied_after_engage`, `agent_invite_wrong_actor_denied`,
    `agent_invite_denied_in_post_reset`), all listing `waddle.v0.agent` in
    `requires_features`; a ninth, `agent_invite_retake_successor`, arrives
    with the E24 re-projection fix below. `scenario-format.md` gains the `episode_open`
    `agent_invite` key, the `agent_task_update` inject kind (the update
    nests as a canonical `waddle.v0.AgentTaskUpdate` under `update`, the
    shape `reset_result` already uses — the message's own `kind` field
    cannot ride flat beside the inject dispatcher's `kind`), the
    `episode.agent_invited` / `episode.agent_engaged` snapshot paths, and
    the `agent_invite_timeout` timer id. The runner now captures a Noop's
    reason from the plan mode that produced the tick instead of hardcoding
    `BYPASS_ACTIVE`, which also fixed a latent runner-vs-E20 drift.
  - **Runtime**: `EpisodeOptions.agent_invite` (waddle-fsm's own
    `AgentInvite`, re-exported — the frontend stays hollow) opens without
    blocking; `run_agent` is the blocking convenience and fails loudly up
    front when the engage-path verbs the invite needs are unwired. The plane
    pump retains every `AgentTaskUpdate` on the mirror
    (`Status.agent_task`); only a DENIED addressed to the ACTIVE episode is
    dispatched to the FSM, which alone picks E26 vs E26b — QUEUED/COMPLETED
    never touch it, and COMPLETED's `recording_ref`/`detail` feed the
    outcome. The mirror publishes `agent_invited` / `agent_engaged` /
    `agent_invite_aborted`. The flag is declared at Register
    unconditionally whenever a transport is configured: the SDK always
    supports being agent-driven, and a plane that did not accept it simply
    never routes an invite (the deadline then closes the episode via E25).
- **waddle-protocol/waddle-runtime (control-plane stills, new feature flag
  `waddle.v0.obs.stills`)**: a hosted agent needs to SEE the scene, and until
  now `publish_frame` fed only the media plane — an agent-only session with no
  LiveKit anywhere had no frame path to the control plane at all. A camera
  declaring `StreamPolicy.still_fps > 0` (descriptors.proto field 3; 0/absent
  means no stills) now has each published frame teed into a latest-wins
  per-camera slot, which the existing `waddle-media-uplink` pump samples at
  the declared rate, JPEG-encodes (never on the customer's thread, never on
  the gate path), and sends as `ObservationUpdate{ still: FrameStill }`
  (`FrameStill { camera, frame_seq, encoding, width, height, data }`,
  services.proto payload arm 5). **Bounded by declaration — never a video
  path; LiveKit media remains the only video transport**, and the file-header
  and StreamPolicy comments asserting that nothing high-bandwidth rides these
  RPCs now name this bounded exception explicitly.
  - The intake grew a second, independent leg rather than a branch: the media
    leg keeps its own fps throttle and bounded drop-oldest queue, the stills
    leg its own frame-timeline throttle and capacity-one slot; neither
    rate-limits the other. `CameraUplink` is built whenever EITHER leg has
    somewhere to go, so a stills-only camera works with no media plane, while
    a session with neither keeps `publish_frame`'s cheap no-op early return
    and declared-uplink validation stays scoped to cameras that would
    actually be wired.
  - The throttle reads the frame's own `SessionClock` stamp, never a
    pump-side clock, so the sampled rate is a property of the frames and the
    sampling is deterministic under test; a not-yet-due frame is kept rather
    than discarded, so a publisher slower than its declared rate still gets
    every frame sampled. `frame_seq` is minted once per validated
    `publish_frame`, before either throttle — it numbers the camera's frames,
    not the subset a policy admitted, and is THE per-camera `FrameNotice`
    counter for whatever emits `FrameNotice` later. Stills stay out of
    `camera_frames_dropped`, which keeps meaning exactly media-uplink loss.
  - The flag is declared at Register iff some camera asks for stills
    (declaring it otherwise would claim a behavior the session cannot
    produce), and stills are emitted only while the CURRENT connection
    accepted it (VERSIONING §3), refreshed on every registration by the plane
    pump — the emitting thread is not the one that sees the response.
    Control-plane stills are also the first *droppable* history-free message
    class: see the `waddle-controlplane` entry under Fixed for how they are
    shed rather than buffered while a plane is unreachable or stalled.
- **waddle-protocol (fixture `remote_reset_caller_tick_noop`)**: pins FSM.md
  E20's caller-tick marker, previously asserted by no golden — a `gate_tick`
  during an ENGAGED remote reset window returns
  `Noop{NOOP_REASON_RESET_ACTIVE}` and causes no episode transition (the
  stale-handle contract), and the first tick after `reset_window_complete`
  passes through and drives E6 READY→RUNNING, pinning the marker as
  window-scoped rather than sticky. Before this fixture the conformance
  runner's `(Noop, PlanMode::Reset)` marker-translation arm could be
  reverted to `BYPASS_ACTIVE` with the whole suite staying green; the E20
  row's Fixture column now lists it.
- **waddle-runtime (`ServerMsg::ResetProgress` handling)**: the
  plane-executed reset completion path (`RequestReset`/`ResetProgress`,
  `waddle.v0.reset`) is no longer dropped — every message updates a new
  `Status.reset_progress` mirror field (observational only; `episode.proto`
  doesn't model this as an `EpisodeEvent`), and `ResetProgress{DONE, result}`
  injects `SessionEvent::ResetResult` exactly like the inline/pump paths
  already do, completing the pipeline. No episode-id filtering (the message
  carries none — session-scoped, like `HeartbeatAck`); the FSM's own E19b
  guard (`ResetResult` requires `Phase::Resetting` with no open remote
  window) makes a stray or out-of-order DONE harmless. **Closes a
  long-documented gap**: a retake successor under a session-level
  `Remote` PRE spec is born-claimed, so its pre-reset window never opens
  (D7 edge 5); nothing else in the runtime could ever complete that
  successor's RESETTING. `RequestReset` issuance (the outbound half) stays
  unimplemented — no `ResetSpec` variant models "the plane executes this
  reset automatically," so there is no clean trigger to fire it from — a
  known open item.
- **waddle-runtime (`Session::report_proprio` + `StreamObservations`
  uplink)**: `report_proprio(ProprioReport { joint_vel, ee_pose, gripper })`
  reports a richer proprioceptive sample than the bare `joint_pos` every
  `gate(obs=...)` call already records; the reducer merges it with the
  latest gate-tick `joint_pos` into every recorded `ProprioSample` (Local
  mode, `/waddle/observations`) and into a periodic `ClientMsg::Observation`
  uplink sent whenever a transport is configured (10 Hz conservative
  default — no declared per-robot rate exists on this control-plane RPC to
  key off; see `Reducer::DEFAULT_OBSERVATION_UPLINK_HZ`'s doc). Every field
  PATCHES the reducer's latest known sample; `None` leaves a previously
  reported value in place (no way to clear one in v0). `ee_pose` is a
  frame-tagged `EePose` (position + wxyz orientation + a non-empty
  `frame_id`, per `descriptors.proto`'s `Pose` invariant) rather than the
  design sketch's bare `[f64; 7]`, since an untagged pose is exactly the
  silent-corruption failure mode that invariant exists to prevent.
- **sdk (Python `session.report_proprio`)**: `session.report_proprio(
  joint_vel=..., ee_pose=..., ee_pose_frame="ee", gripper=...)` — numpy
  `float64` ndarray or plain list accepted for `joint_vel`/`ee_pose` (same
  zero-copy-when-possible convention as `gate(action, obs)`); `ee_pose`
  raises `ValueError` unless it has exactly 7 values (xyz + wxyz).
  `ee_pose_frame` (default `"ee"`) names the frame the pose is expressed
  in — deliberately one kwarg wider than a bare 7-value pose signature,
  for the same frame-tagging reason as the Rust `EePose` type.
  Dev-only new dependency: `mcap-protobuf-support` (pulls in `protobuf`),
  used by the extended `test_nominal_episode` MCAP read-back test to
  decode `/waddle/observations` messages via the channel's own embedded
  `FileDescriptorSet` schema and assert the merged field values, not just
  topic/message counts. Not a runtime dependency of the SDK itself.
- **waddle-protocol/waddle-runtime (directive acks, new feature flag
  `waddle.v0.plane.acks`)**: plane→SDK directives are no longer blind
  fire-and-forget — an FSM rejection is now observable to the plane.
  services.proto gains an optional `directive_id` on `ClaimDirective` (field
  3), `EpisodeDirective` (field 5), and `ResetWindowDirective` (field 5),
  plus `DirectiveAck { directive_id, accepted, reason }` as
  `GateClientMessage` arm 4 (append-only). When a directive carries a
  `directive_id` AND the connection negotiated the flag, the SDK answers
  with exactly one ack per directive: `accepted=true` when the session FSM
  applied every event the directive decoded into, `accepted=false` with the
  FSM's rejection reason in guard-row language (e.g. "engage outside RUNNING
  (E7)", "terminate rejected in POST_RESET (E14b)", the C6 reset-claim
  admission reason) when any was rejected — a directive that decodes into
  two events (claim GRANT, reset-window ENGAGE) acks once, with the first
  rejection's reason. Zero guard-semantics changes: the FSM accepts and
  rejects exactly what it did before; acks are a runtime/plane behavior and
  never appear on the `EpisodeEvent` stream, in sidecars, or in fixtures.
  Directives without an id stay fire-and-forget; the flag is always declared
  at Register when a transport is configured (safe — emission still requires
  the id). Registry row in VERSIONING.md; normative ack paragraph in FSM.md
  §8.
- **waddle-gate/waddle-runtime (Claimed-mode agent-chunk intake + jitter
  horizon + `ReplanPolicy`)**: cloud-agent interventions are now real
  outside a reset window too. `forward_server_msg`'s `InterventionChunk` arm
  (previously Reset-mode-only intake) now accepts a chunk whenever a
  claim is active — the same `claim_active`-alone gate `spawn_media_intake`'s
  teleop path already uses, so a chunk arriving during the ENGAGE handoff
  sub-phase still buffers correctly and is ready the instant the handoff
  completes.
  - `waddle-gate::jitter::JitterBuffer` is chunk-aware on the `AgentChunk`
    channel: each arrival carries the wire chunk's `ChunkMeta`
    (`seq`/`t_emitted_ns`); a chunk boundary (a step from a different chunk
    than the channel's currently-executing one) decides stale-vs-supersede —
    `chunk_seq` (the one field `control.proto` normatively requires to be
    monotone per stream) is the primary staleness signal, so a chunk whose
    `seq` is not strictly newer is rejected wholesale (`dropped_stale_chunks`);
    `t_emitted_ns` is consulted only as an additional rejection when BOTH the
    executing and candidate chunk declare a nonzero value and the new one
    isn't strictly newer, so a wire-legal producer that leaves it at the
    proto3 default 0 (or ties it) is never wrongly locked out (a fixed review
    finding: the original `chunk_seq` **AND** `t_emitted_ns` rule rejected
    every subsequent chunk of a claim window forever the moment a producer
    left the timestamp unset). A genuinely newer chunk applies the declared
    `descriptors.proto` `ChunkingSemantics.replan`:
    `REPLAN_POLICY_IMMEDIATE`/`REPLAN_POLICY_BLEND` drop the executing
    chunk's still-pending steps (BLEND has no declared blend duration/curve
    for a chunk-to-chunk splice and its own comment steers away from it, so
    it maps onto the same replace-remaining behavior as IMMEDIATE — a
    documented simplification); `REPLAN_POLICY_CHUNK_BOUNDARY` lets them finish
    first. `clear_pending` (the existing claim/window-teardown discard) also
    forgets the executing-chunk pointer, so a brand-new claim's first chunk
    is never wrongly rejected as stale against an unrelated prior claim's
    last one. `GateShared::new`/`JitterBuffer::new` take the declared
    `ReplanPolicy` (from `ActionSpace.chunking.replan`) as a new parameter.
  - Playout scheduling stays session-receive-time + each step's
    `t_offset_ns` (unchanged from the Reset-mode intake) — chunk
    `seq`/`t_emitted_ns` are
    used only for the boundary/staleness decision, never as the playout
    anchor.
  - Dims validation: a chunk whose flattened width doesn't match the
    declared action space now raises `SessionEvent::InterventionRejected`
    (once per claim window, chunk dropped) — the same event/fault the
    teleop path already uses. The event gained a `source` field
    (`"media-intake"` / `"agent-chunk"`) so the emitted fault names the
    actual rejecting producer instead of always saying "teleop action" /
    "media-intake"; every other wire-validation error (missing field, wrong
    target arm, Opaque space, …) still only gets a `tracing::warn!` (Task
    10's reasoning: a dims-only event would misreport those).
  - New runtime e2e tests (`claimed_chunk_intake.rs`, `InMemoryTransport`):
    a 5-step Claimed-mode chunk substitutes in order via the caller's own
    `gate()`, tagged `Provenance::Agent`, with MCAP read-back; a superseding
    chunk mid-horizon under `IMMEDIATE` drops the executing chunk's
    remaining steps; a dims-mismatched chunk faults once per claim window
    and drops, a subsequent correct one still substitutes.
- **waddle-controlplane (real tonic gRPC `ControlTransport`, feature
  `tonic-transport`)**: the `tonic-transport` feature is no longer an empty
  stub — `waddle_controlplane::grpc::{GrpcConfig, GrpcTransport}` implements
  the same `ControlTransport` trait the in-memory transport does, over the
  eight `ControlPlane` RPCs of services.proto.
  - Mapping: `Register`/`Negotiate`/`ClaimEpisode`/`HandoffLease` are unary;
    `GateActions` + `Heartbeat` are eager long-lived bidi streams (the
    plane's directive/demotion down-paths); `StreamObservations` opens
    lazily on the first observation (acks are drained); `RequestReset`
    progress funnels back through the single ordered rx. Any transport-level
    error severs the connection channels, handing recovery to the client's
    existing backoff/replay machinery — the transport duplicates none of it.
  - Tokio confinement (the Task-14 pattern): one dedicated
    `waddle-controlplane-grpc` thread per live connection owns a private
    current-thread runtime (plus a `waddle-controlplane-grpc-tx` forwarder
    thread); the trait surface stays sync/channel-based and featureless
    builds stay tokio-free (`cargo tree` verified).
  - Auth per services.proto: `GrpcConfig { url, token }` sends
    `authorization: Bearer <token>` metadata on every RPC (`Debug` redacts
    the token). `https://` URLs use rustls with the platform's native roots.
  - Codegen stays protoc-free: `waddle-controlplane/build.rs` feeds a
    protox-compiled descriptor set to `tonic-prost-build` with `extern_path`
    mapping every message back to `waddle_types::pb::v0`, so only service
    glue is generated and exactly one copy of the wire types exists.
  - In-process integration tests (generated tonic server as the test plane):
    connect → auto-Register with bearer metadata, gate round-trip both ways,
    hard server kill → `Disconnected`, restart on the same port →
    re-register + in-order replay of messages buffered while offline.
  - Deps (all optional, behind the feature): tonic 0.14.6 + tonic-prost
    0.14.6 (the prost-0.14 pairing), tokio (rt/sync/macros), tokio-stream;
    build-deps tonic-prost-build + protox. tonic's `server`/`router`
    features ride the same feature solely for the in-process test plane
    (cargo cannot feature-gate dev-dependencies) — compile-time cost only.
- **waddle-runtime (`Session::publish_frame` — cameras are live) + tripwires
  evaluate real observations + `session.publish_frame` (Python)**: the
  biggest Milestone-A gap closes — declared cameras and tripwires actually
  do something.
  - `Session::publish_frame(camera, FrameData)` (`FrameData::rgb8(width,
    height, bytes)`, RGB8 only for now — typed as an enum so a future
    `Depth16` variant can land without breaking the constructor): validates
    `camera` against the robot's declared `cameras` (unknown → `Err`;
    declared but no media plane wired → a cheap `Ok(())` no-op — Local mode
    still records no video in v0), applies the declared
    `StreamPolicy.uplink` fps throttle (a wait-free atomic-timestamp check;
    a throttled frame is silently dropped, never an error, never counted),
    and enqueues onto a small (4-deep) per-camera bounded queue that
    drops the OLDEST frame under backpressure — counted, and surfaced via
    the new `Session::camera_frames_dropped(camera)`. Everything past the
    queue (the lazy, once-per-camera `publish_track` call; encode — raw
    passthrough for `RGB8`/`BGR8`/`JPEG`, the declared encoding being
    bandwidth-intent for the track rather than a literal wire format (see
    the Fixed entry below); `push_frame`) runs on one new dedicated
    `waddle-media-uplink` pump thread, never the customer's own thread. A
    declared `CAMERA_ENCODING_H264` uplink policy is a build-time error
    (`RuntimeError::UnsupportedCameraEncoding`) for any camera a wired
    media plane will actually publish — never a silent per-frame failure
    later.
  - `waddle-tripwire`'s `ObsSource` is no longer wired to an always-`None`
    stub: the reducer now publishes every gate tick's `obs` (the
    customer's `gate(obs=...)` argument) onto a wait-free `LatestSlot`
    (`waddle_ingest::LatestSlot`) as it drains the gate-record ring —
    unconditionally, whether or not local MCAP recording is on, and never
    touching `Gate::gate()`'s fast path. The flat customer vector maps onto
    `ObsSnapshot::joint_pos` verbatim, so a declared `JointLimitMargin` or
    `Staleness` tripwire now genuinely fires a HOLD (or whatever verb it
    declares) through dispatch; `WorkspaceAabb`/`ForceThreshold` still need
    a capture integration publishing structured `ee_pos`/`force_n`.
  - `session.publish_frame(camera, frame)` (PyO3): accepts a numpy `uint8`
    ndarray shaped `(height, width, 3)` (packed row-major RGB8); a
    contiguous array is copied once into the frame the core queues. A
    wrong dtype/rank/shape (or a non-contiguous array) raises `TypeError`;
    an unknown camera or a resolution mismatch raises `RuntimeError` (from
    the core). `waddle.init(..., media=waddle.LiveKit(url, token))`
    exposes the config shape for a real WebRTC-backed media plane, but
    this SDK build does not compile the heavy `livekit` Cargo feature
    (~700 MB webrtc-sys download, tokio) in by default — passing it raises
    a clean, actionable `RuntimeError` naming the gap, exactly like the
    deferred `grpc` transport; `_testing=True` (the in-process loopback)
    is unaffected and is how `publish_frame` is exercised end-to-end today
    (`waddle._testing.frames(session, camera)` observes the far end).
- **waddle-media (real LiveKit `MediaPlane` behind the `livekit` feature)**:
  `livekit::LiveKitMedia` is the first real transport.
  `LiveKitMedia::connect(LiveKitConfig { url, token, track_resolutions })`
  spawns ONE dedicated thread (`waddle-media-livekit`) owning a private
  current-thread tokio runtime; all `MediaPlane` methods stay synchronous
  and forward over channels, so **tokio stays confined to this feature** —
  no tokio type crosses the public API and featureless builds have no
  tokio in the tree at all. `DataTopic` maps to LiveKit data-channel
  publishes on the normative `media.proto` topic strings with the
  normative reliability classes (TeleopPose/Telemetry lossy latest-wins,
  TeleopClutch/TeleopMark reliable ordered); inbound packets route by
  topic into the existing `DataRx` seam. `publish_track` publishes a
  native video track at the camera's declared resolution (default
  640x480); because LiveKit video sources consume RAW frames (libwebrtc
  encodes uplink itself, no pre-encoded JPEG accepted), `push_frame`
  accepts RGB8 (converted via `rgb8_to_i420`) or already-planar I420 —
  the JPEG encoder is for the data-channel/recording path. Feature-gated
  tests: a CI-safe unreachable-server test plus an `#[ignore]`d live
  end-to-end test driven by `WADDLE_LIVEKIT_URL`/`WADDLE_LIVEKIT_TOKEN`.
  Build note: with `--features livekit`, `webrtc-sys` downloads a
  prebuilt libwebrtc at build time (network on cold builds, ~690 MB
  extracted per target dir, ~30 s cold check); default builds unaffected.
- **waddle-media (real JPEG `VideoEncoder` + RGB8→I420 conversion)**:
  `JpegEncoder` (Motion JPEG over RGB8 via the pure-Rust `jpeg-encoder`
  crate; every frame a keyframe) joins `PassthroughEncoder` behind a new
  `VideoEncoding` selector (`make_encoder(encoding, width, height)`).
  `VideoEncoding::H264` stays a typed TODO — requesting it returns
  `MediaError::Unimplemented`, never a silent fallback. `rgb8_to_i420`
  converts RGB8 frames to planar I420 (BT.601 studio swing, 2x2
  block-averaged chroma, odd dims round chroma up) for raw-frame WebRTC
  video sources. `MediaError` gains `BadFrame`/`Encode`/`Transport`
  variants and `Unimplemented` now names what is deferred.
- **sdk (Python reset API: `TeleopReset`/`AgentReset`, `init`/`rollout`
  kwargs)**: the headline user-facing surface for the reset-phases branch.
  `waddle.TeleopReset(prompt, *, timeout_s=600.0)` and
  `waddle.AgentReset(prompt, *, timeout_s=600.0)` are small frozen,
  repr-friendly dataclasses declaring a remote reset window for a
  teleoperator/agent respectively (their docstrings name the production
  caveat: this open-source runtime has no supervision-plane transport
  wired yet, so today a window can only be driven end-to-end via the
  private `waddle._testing` reset-window hooks). `waddle.init` gains
  `pre_reset=None`, `post_reset=None` (`None` | callable | `TeleopReset` |
  `AgentReset`) and `reset_verification="blocking"` (`"blocking"` |
  `"optimistic"`); `waddle.rollout(task, *, pre_reset=_UNSET,
  post_reset=_UNSET)` gains the same two kwargs with a module-level
  `_UNSET` sentinel distinguishing "inherit `init()`'s declaration"
  (`_UNSET`, the default) from "disable this phase for this one episode
  only" (explicit `None`) from "override it" (a fresh marker/callable).
  Callables are normalized **in Python** (`_normalize_reset_hook`) so the
  `_core` FFI always receives `(bool, Optional[bool])`: a bare `bool`
  return vouches for its own verification (`(ok, ok)`, matching the
  existing FFI-level default, now pure defense-in-depth underneath this);
  anything else — wrong arity, a non-bool first element, a second element
  that is neither `bool` nor `None` — raises `TypeError` naming the
  contract. That `TypeError` is diagnostic only: `PyResetHook::call`
  (Rust) catches every exception from the hook callable, including this
  wrapper's own, and reports it solely via `sys.unraisablehook` before
  normalizing to `(False, None)` — the `rollout()` caller sees the same
  generic `RuntimeError: reset failed` as a hook that legitimately
  returns `False`, and cannot tell the two apart from that exception
  alone. `waddle._testing` gains `reset_window_engage`/
  `reset_window_complete` thin wrappers alongside the existing
  `engage`/`release`/`push_teleop` (they deserved the same wrapper
  treatment). `rollout`'s docstring now
  documents the post-reset exit contract: `ep.done` flips to `True` at
  POST_RESET entry (before cleanup finishes); the ordinary
  `ep.terminate(...)` call already blocks the `with`-exit through it
  (unchanged); a `with` block that exits some other way while
  POST_RESET is still running finds `__exit__` already a no-op (it never
  aborts, or otherwise touches, an in-flight post-reset); a failed
  post-reset never changes the pinned outcome, only
  `ep.post_reset_failed`. `init`'s docstring documents
  `reset_verification` and the remote-window build-time-negotiation
  narrowing rule. `sdk/README.md`'s hollow-frontend checklist gains a
  Reset API bullet: markers/callables are pure type dispatch and
  input-shape validation, never reset decisions — every actual behavior
  stays in waddle-core.
- **sdk (PyO3 shim: reset kwargs, `PyResetHook`, testing hooks)**: the
  `_core` module surface now exposes the full reset-config vocabulary.
  `create_session` gains `{pre,post}_reset_kind` (`"none"`|`"hook"`|
  `"teleop"`|`"agent"`), `{pre,post}_reset_hook`, `{pre,post}_reset_prompt`,
  `{pre,post}_reset_timeout_ns` (default 600s), and `reset_verification`
  (`"blocking"`|`"optimistic"`) — all defaulted for full back-compat,
  mapping onto `SessionBuilder::pre_reset`/`post_reset`/`verification_mode`.
  `PySession::start_episode` gains the same eight kwargs as per-episode
  overrides (`None` = inherit the session default) → `start_episode_with`.
  A Python callable crosses as a `PyResetHook` (`sdk/rust/src/verbs.rs`,
  the `PyUnit` GIL/shutdown pattern): it normalizes a bare `bool` return to
  `(bool, Some(bool))` (a hook with no separate verification opinion is
  read as vouching for its own `ok` — otherwise a bare `True` would hang
  forever in RESETTING under the default Blocking verification mode,
  which requires `verified = Some(true)`), passes an explicit
  `(bool, Optional[bool])` tuple through as-is, and — for anything else
  (a raised exception, or a return value of neither shape) — reports it
  via `PyErr::write_unraisable` (CPython's "log, don't propagate" hook for
  background-thread callbacks) and normalizes to `(false, None)`; the hook
  never panics or unwinds into Rust. `PyEpisode` gains a `post_reset_failed`
  getter (mirror read); `done`'s docstring documents the POST_RESET flip;
  `outcome` now reads `status().outcome.or(status().pinned_outcome)` so it
  returns the pinned value (not `None`) once `done` flips true at
  POST_RESET entry, matching `waddle_runtime::Episode::outcome()`'s own
  contract without touching the episode's inner mutex.
  Two new `_testing`-gated hooks (`testing_loopback=True` only, following
  the existing `_testing_engage`/`_testing_push_teleop` pattern):
  `_testing_reset_window_engage(claim_id, actor)` and
  `_testing_reset_window_complete(claim_id, ok, verified=None)` inject the
  window `SessionEvent`s directly (mirroring the exact `ClaimGranted` +
  `ResetWindowEngage` / `ResetWindowComplete` sequences
  `forward_server_msg`'s plane ENGAGE/COMPLETE arms produce), backed by two
  new `waddle-runtime` convenience functions (`reset_window_engage`,
  `reset_window_complete`, alongside the existing `grant_and_engage`/
  `release_claim`) so the shim never mints its own clock stamps. Verified
  (not changed): the reset pump's shutdown ordering — it checks
  `mirror.status.shutdown` at the top of its loop exactly like the bypass
  pump, `Session::shutdown()` sets that flag before joining any thread, and
  `PySession::shutdown`/`Drop` already run the blocking join with the GIL
  detached — so a `PyResetHook`'s `Python::try_attach` on the pump thread
  can never deadlock against a Python caller holding the GIL during
  interpreter teardown.
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
  it (documented on `EpisodeOptions`, not runtime-enforced — the simpler
  sound option). The reset pump (the actual hook
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
- **sdk (descriptors: intrinsics, stream policy, URDF, frame graph, joint
  limits)**: `sdk/python/waddle/descriptors.py` widens to cover the rest of
  `descriptors.proto`'s declaration surface (shape only — the hollow-frontend
  rule: no new semantic validation, `RobotDescription::try_from` remains the
  one semantic validator). `Camera` gains optional `intrinsics: Intrinsics`
  (`fx, fy, cx, cy`, `distortion_model` — short names, defaults to
  `"unspecified"` — `distortion: tuple[float, ...]`, `depth_scale_mm`),
  optional `stream_policy: StreamPolicy` (`local_full_rate: bool`, optional
  `uplink: Uplink(fps, encoding, max_kbps)`, compiling to the `stream` wire
  field), and `vendor: dict[str, str]`. `Robot` gains optional
  `kinematics_urdf: bytes | str | Path` (`bytes` passes through as-is;
  `str`/`Path` is read from disk **at compile time** — pick one, document
  it, no silent XML-vs-path guessing), `frames: tuple[FrameTransform, ...]`
  (new dataclass: `parent`, `child`, `position` (x, y, z), `quaternion`
  — **wxyz**, pinned by a dedicated non-symmetric test — compiling to a
  `FrameGraph`; the nested `Pose.frame_id` is filled from `parent`, the
  frame the transform's numbers are expressed in), and `series: dict[str,
  TimeSeries]` (`dtype`, `shape`, `units`, `frame_id`, `rate_hz`).
  `JointSpace.joints` and the new `Gripper.dexterous(joints)` both now
  accept either a bare name (names-only form, unchanged) or a new `Joint`
  dataclass (`min_position`, `max_position`, `max_velocity`, `max_effort`)
  via a shared `_compile_joint` helper. Validation stays minimal and
  objective: `min_position <= max_position`, `max_velocity`/`max_effort`
  `>= 0`, `fps > 0`, `max_kbps > 0`, `depth_scale_mm > 0` — everything else
  is shape-only and deferred to waddle-core. Back-compat is a golden assert:
  descriptors that set none of the new fields compile to the exact same
  dict as before.

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
- **`waddle-fsm` — the E24 agent-episode gate plan is now re-projected
  whenever its inputs move, not only when the gate MODE does**: the gate plan
  is derived state, and plan derivers (the runtime reducer, the conformance
  target) re-derive it only when they see `Effect::SetGateMode`. E24's Noop
  plan also depends on the EPISODE (agent-invited, phase), so any row that
  moved that without touching the mode left every deriver holding a stale
  plan. **The reachable failure**: an agent-invited episode closed by a
  **retake** (C5 — the claim survives, so the shared run-closing block skips
  the re-projection it does on a claim-releasing close) handed its
  born-claimed successor, a NORMAL episode, the predecessor's Noop plan; the
  customer's own `gate()` ticks then returned
  `Noop{NOOP_REASON_AGENT_EPISODE}` forever, with no fault — a control loop
  that silently stops actuating. Retake is plane-reachable from the engage
  timeout, and no fixture covered retake on an agent-invited episode, so
  every gate stayed green. The mode-unchanged re-projection now happens
  centrally, in the one place every row funnels through (`Ctx::finish`,
  keyed on the FSM-owned plan inputs `(gate_mode, agent_episode_noop())`),
  replacing the two per-row pushes that covered only episode open and
  claim-releasing closes; new session invariant **I20** asserts it for every
  step of the random walk, and the new fixture
  `agent_invite_retake_successor` pins the retake path end to end. E24's
  scope also gained INTERVENTION — the *engage window*, where the handoff is
  in flight, the gate is still PASSTHROUGH and nothing is engaged yet, so
  E24's own guard ("no engaged claim") holds: the predicate said otherwise
  while the installed plan noop'd, and the plan was right. **FSM.md**'s E24
  row stated `RUNNING` alone in its From column while the implementation (and
  the fixtures) also noop'd in RESETTING and READY; it now states the full
  set, and §1.5 says outright that the plan is scoped to the episode it was
  derived for and must be re-derived when the episode state behind it moves
  — a second implementation can no longer dispatch the caller's actions
  inside an agent-invited episode's reset, or keep noop'ing after it, and
  still pass the suite.
- **The bypass pump exempted a never-ticked gate from stall detection, so an
  engaged claim in a session whose gate never ticks would have gone undriven
  forever**: `spawn_bypass_pump` only reported a stall when a previous
  `gate_tick` existed and was older than the threshold, so a `None` last tick
  was silently exempt. FSM.md §6's condition is "no `gate_tick` within the
  stall threshold", which holds *vacuously* when there has never been one —
  and that is exactly the shape of an agent-invited episode driven through
  `Session::run_agent`, which reaches RUNNING with the caller's thread
  blocked and therefore never ticks at all. A `None` last tick now counts as
  stalled (the `Some` threshold contract is unchanged); the FSM's own
  `StallDetected` guard still decides whether anything follows, so no guard
  row changed — the pump only reports.
- **`waddle-controlplane` — droppable messages can no longer queue without
  bound while the plane is unreachable or stalled**: `ClientMsg` now answers
  "is this perception/liveness, or history?" in exactly one place
  (`is_droppable`; `buffer_when_offline` is its negation), and BOTH moments a
  message can be shed honour it. (1) While connect attempts fail, the client
  thread now drains its command channel into the bounded offline buffer on
  every backoff slice (`backoff_draining`) instead of only before and after
  the sleep: an unreachable plane used to let the unbounded command channel
  grow for a whole backoff plateau (16 s in production) with the drop-oldest
  bound and its loud `BufferOverflowed` never applying, and every message
  parked there — including droppable ones — was handed to the plane the
  moment it came up, so a partition's worth of stale pictures replayed as if
  fresh. (2) The gRPC transport meters every outbound stream with its own
  `InflightLimit` (new `inflight` module; cap 4 per stream, shed count on
  `GrpcTransport::droppable_dropped`): a plane that accepts
  `StreamObservations` and then stops reading it never errors, so no
  `Disconnected` is ever raised and the offline classification never runs —
  the stills piled up in the transport's internal channels behind a stream
  h2 had stopped polling, unbounded, until OOM. History is never shed by
  either mechanism; only heartbeats and control-plane stills are droppable.
  The `ControlTransport` trait now states the contract: a transport that
  buffers internally must bound what it holds for droppable messages.
- **`Session::run_agent` no longer masks a genuine pre-reset failure (E5)
  as a normal-looking agent ABORT**: the recovery arm that turns a
  `ResetFailed` from the start path into an `AgentOutcome` exists for
  closes the invite machinery itself produces while the caller is still
  blocked in RESETTING (E25's deadline expiry, E26's pre-engage DENIED),
  but it keyed on `agent_invited` alone — the mirror carried no "why", so
  a failing pre-reset hook on an agent-invited episode returned
  `Ok(AgentOutcome{ABORT, detail: ""})` (indistinguishable from "no agent
  engaged") instead of the `RuntimeError::ResetFailed` every other start
  path surfaces, and retry loops would grind against broken reset hardware
  with no error ever raised. The FSM now latches `episode.invite_aborted`
  on exactly E25/E26 (documented in FSM.md §1.5 alongside the
  `agent_engaged` latch; pinned by session-invariant I19), the mirror
  publishes it as `Status.agent_invite_aborted`, and the recovery arm keys
  on that: E25/E26-during-RESETTING still return the ABORT outcome (new
  test drives a real invite timeout under a slow pre-reset hook), while an
  E5 reset failure surfaces as `ResetFailed` (new test). Also pinned by
  test: the unconditional `waddle.v0.agent` Register advertisement
  (deleting it previously kept the whole suite green while silently
  severing real-plane invite routing).
- **`waddle-fsm` — a wrong-actor grant on an agent-invited episode now
  records `claim{DENIED}` (FSM.md C8) instead of being silently dropped**:
  C8 specifies "any other actor's grant is rejected, `claim{DENIED}`" — the
  plane already sent GRANT, so the SDK's refusal must go on the timeline —
  but the reference FSM returned a bare rejection and emitted nothing, and
  the `agent_invite_wrong_actor_denied` fixture asserted only the absence of
  a GRANTED emission, so a spec-following implementation (emitting DENIED)
  and a silently-dropping one both passed conformance. The refusal now
  emits `ClaimEvent{DENIED, detail}` with no state change (same shape as the
  stale reset-engage mint's `lease{DENIED}`; the first production emitter of
  `CLAIM_EVENT_KIND_DENIED`), the fixture asserts the emission, and the FSM
  lifecycle smoke test walks it. C6's wrong-actor rejection stays silent —
  its row never specified a DENIED record and its released golden pins that.
- **`waddle-fsm` — a `reset_window_complete` racing an in-flight engage
  lease mint panicked the reducer thread and hung every blocked caller**:
  E20's lease routing is asynchronous (the runtime answers
  `Effect::MintLeaseToken` via the tail of its single event queue), so a
  plane sending ENGAGE and COMPLETE back-to-back gets the COMPLETE processed
  before the engage's mint answer. The COMPLETE handler had no
  engage-in-flight guard: it saw the window un-engaged, closed it, released
  the reset claim, and (PRE, ok) went READY with the engage's
  `pending_lease` still populated — the stale mint answer then handed the
  lease to the released claimant and panicked (`expect("reset claim
  held")`), killing the reducer (no catch_unwind) so `start_episode*` /
  `terminate_episode` waits hung forever. Two-part fix, pinned as normative
  prose in FSM.md §1.4 ("Engage atomicity"): (1) a COMPLETE arriving while
  an engage mint is in flight is **rejected** — a window that never
  observably ENGAGED has nothing to honorably complete; the plane retries
  after it sees `reset_window{ENGAGED}`; (2) a minted engage lease whose
  reset claim (or window) is gone by the time it applies — e.g. a legal
  `claim_released` raced the answer — is discarded (`lease{DENIED}`, lease
  unmoved, window still serviceable) instead of panicking: the FSM never
  panics on a legal event ordering. The invisibility root cause was that
  every existing harness (the FSM test drivers and the conformance runner)
  answered mints synchronously, so the interleaving was inexpressible;
  the FSM test driver and the property-test alphabet now support deferred
  mint answers (`DeferMints`/`AnswerMint` random-walk commands run all 14
  session invariants over these interleavings), four deferred-mint FSM
  regression tests pin rejection/degradation/benign-overwrite/timeout
  behavior, and two runtime tests drive ENGAGE+COMPLETE back-to-back
  through the production plane-directive path and assert the session
  always resolves with the reducer alive.
- **`waddle-controlplane`'s `tonic-transport` test build (pre-existing
  break)**: `grpc_transport.rs`'s two
  `ClaimDirective` struct literals predated the directive-acks feature
  (`waddle.v0.plane.acks`) that added its `directive_id` field and were
  never updated for it — a compile break invisible to `cargo test
  --workspace` (featureless) and never caught because nothing since had
  re-run this crate's feature-gated tests (they are now part of the
  standing pre-commit gates in CLAUDE.md). Fixed with `directive_id: None`
  (no production code touched).
- **`Session::publish_frame` — a declared `CAMERA_ENCODING_JPEG` uplink
  policy would fail every frame against a real LiveKit-backed session**:
  the previous behavior ran a declared JPEG uplink through the real
  `JpegEncoder` (Motion JPEG bytes) before handing it to `MediaPlane::
  push_frame`, but a WebRTC video track (the only real transport wired,
  `LiveKitMedia`) ingests raw RGB8/I420 only — libwebrtc encodes the
  uplink itself, and a still-image byte stream is not a track format at
  all. Neither `media.proto` nor `descriptors.proto`'s `StreamPolicy`/
  `UplinkPolicy` comments promise JPEG-on-the-wire for tracks, so the fix
  reconciles the encoding contract instead of the transport: a declared
  `UplinkPolicy.encoding` is now bandwidth-intent for the customer, not a
  literal byte format — `RGB8`, `BGR8`, and `JPEG` all resolve to raw
  passthrough on the track path and publish identically (the transport
  converts to whatever the track needs; `LiveKitMedia::push_frame` already
  did this conversion, `rgb8_to_i420`, for RGB8 — it now also receives
  correctly-shaped bytes for a JPEG-declared camera instead of a mismatched
  compressed buffer). `CAMERA_ENCODING_H264` is unchanged: still the one
  genuinely unsupported encoding, still a build-time
  `RuntimeError::UnsupportedCameraEncoding`, never a silent per-frame
  failure. `waddle-media`'s real `JpegEncoder` is untouched and
  remains available for a genuine still-image byte stream path (e.g. a
  future data-channel/recording snapshot) — nothing on the track path
  calls it today. Regression-tested with a LiveKit-shaped `MediaPlane`
  test double (validates the same RGB8-or-I420 track shape `LiveKitMedia`
  does, without the `livekit` feature or a live server): RGB8 and JPEG
  declarations both publish a raw frame through to the track with zero
  drops; H264 stays a clear build-time error.
- **`sdk/tests/test_e2e.py::test_intervention`'s pre-existing flake**: the
  test declared a 3-joint robot but pushed teleop `Twist` packets, which
  `pumps::flatten_packet` always flattens to exactly 6 values (linear xyz +
  angular xyz) — media intake's dims validation (already landed)
  correctly rejected every packet as a dims mismatch (3 declared vs. 6
  incoming), so the intervention stream never reached the gate and the
  test's 5s wait for a substitution always timed out. This was a stale test
  fixture, not a timing race or a core regression (confirmed deterministic
  across repeated runs, and identical on the commit immediately before this
  change) — fixed by giving the test's robot a 6-joint action space to
  match the raw twist width it actually exercises.
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
- **Intervention ring — a released claim's leftover, not-yet-due actions
  could outlive it and dispatch under a LATER, unrelated claim's
  provenance**: per-channel reorder cursors (above) stop the wrong-channel
  seq collision, but not this — an arrival pushed but not yet due when its
  claim releases or its reset window closes sat in that channel's pending
  map with nothing left to drain it (the caller stopped ticking `Claimed`,
  and the bypass pump only polls while `Bypass`/`Reset` is active). It
  resurfaced the next time anything popped that same channel, which could
  be a much later, entirely unrelated claim or reset window, dispatched
  tagged with THAT claimant's mirror provenance — corrupting the
  provenance-tagged actuation record during a reset window, a scene-reset-
  sensitive context. With a 20ms playout delay and typical teleop packet
  rates this triggered routinely (at least one in-flight packet pending at
  essentially every claim release), not as a rare race. `waddle-gate`'s
  `JitterBuffer::clear_pending`/`StreamIntake::clear` now discard every
  channel's pending, not-yet-due arrivals (cursors untouched); the reducer
  (`Effect::SetGateMode`) calls it on every transition back to
  `GateMode::Passthrough` — the one point every claim/reset-window
  teardown funnels through, while `Bypass`<->`Intervention` toggling for
  the SAME live claim never passes through it, so nothing still
  legitimately in flight is discarded. Regression-tested at the
  `JitterBuffer` level and end-to-end (an ordinary teleop claim releases
  with in-flight packets still pending, then a later Remote POST window's
  agent chunk dispatches with zero teleop residue reaching `send`, checked
  on the dispatched values rather than provenance alone).
- **Retake successors never inherited the session's `post_reset` config**:
  `Effect::OpenSuccessor` hardcoded `post_reset: false` (with a stale
  comment claiming a runtime start path applied the config — no such path
  runs for a reducer-opened episode), so a retaken episode's own
  termination skipped straight to `Terminal` with no cleanup at all, even
  when the session declared one. The reducer now carries the session-level
  `post_reset` default and resolves it the same way `start_episode_with`
  does (`Hook` → `post_reset: true`; `Remote` → the declared `post_window`
  too) when answering `OpenSuccessor`; a `Remote` post-reset opens the
  successor's own POST window exactly as it would for any other episode —
  the born-claimed suppression (D7 edge 5) is a PRE-window-only guard and
  never applied to POST. A predecessor's per-episode `post_reset` override
  still does not carry across a retake (documented on `EpisodeOptions`):
  the successor only ever sees the session-level default, matching the PRE
  side's existing behavior. `pre_reset` on successors is unchanged (the
  reset pump already fell back to the session default for the PRE phase;
  a declared `Remote` PRE spec on a successor remains the known gap noted
  above, pending the closed-side retake/hand-reset flow).

## Stowed changelogs

_None yet. On first release, the released section moves to
`docs/changelogs/CHANGELOG-<artifact>-<version>.md` and is linked here._
