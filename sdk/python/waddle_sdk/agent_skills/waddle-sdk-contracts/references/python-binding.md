# Python binding contract

The optional `waddle_sdk_relay.device.Device` relays public SDK operations over
an application-authenticated socket. It owns no listener, login or planner and
preserves the existing Site/Run/native gate. Keep reserved Hold/e-stop capacity,
run-bound playback receipts, exact camera evidence and uncertain-drain ownership.
`waddle_sdk.workspace.files` owns the shared POSIX confined file implementation;
relay transfer excludes hidden components and recordings. See
`docs/python/device-relay.md` for platform limits and the application boundary.

`waddle_sdk.media.PreviewPublisher` consumes copied RGB8 bytes in a separately
owned presentation process. It requires the matching optional media core, explicit
camera grids and bounded demand-driven RGB-only settings. It opens no site, device,
run or control transport. The caller owns sampling and process/resource isolation;
the media core enforces the frame-rate ceiling on original sample timestamps.

`waddle_sdk_relay.connection` shares private key/configuration file parsing without
loading the SDK runtime. Applications select paths and authorize peers; retain
private-file checks, bounded JSON, TLS and explicit numeric-loopback exceptions.
Parsing a saved launcher never executes it or opens hardware.

## Hollow frontend

The Python package describes hardware and composes lifecycle resources. It must not reproduce Rust decisions about claims, leases, handoffs, feature negotiation, or episode timelines. If Python grows an `if` about those concepts, move the behavior to the core and expose only the narrow result needed by the binding.

The extension shim grows declarations and keyword arguments, not policy logic. Select optional native features through the package's declared feature set rather than probing imports.

## Public lifecycle

The root package intentionally keeps a small surface: `Site`, `SiteSession`, `Run`, `load_site`, transport declarations, outcomes, and typed manifest/connector errors. Driver extensions live under `waddle_sdk.robots` and `waddle_sdk.cameras`. Structural runtime DTOs and optional support, FK, and geometry ports live in `waddle_sdk.runtime`.

`load_site(path)` validates and confines configuration without opening adapters. `Site.open(...)` returns an unopened session context. The enter operation resolves factories, opens resources, registers the exact declarations and verbs, and starts owned pumps. Exit finalizes recording before hardware close and cleans half-open resources.

## Declarations and support

Pure-Python descriptors compile to canonical protocol JSON. They validate shape; the core validates protocol semantics. A declaration's joint order and widths must exactly match driver reads and writes. Composite declaration order defines flattened action layout.

After open, the SDK derives `waddle.sdk.support/v1` from actual registered verbs, declarations, and optional opened facets. Adapters do not author this matrix. Missing optional FK, body geometry, intrinsics, or velocity feedforward removes only dependent behavior unless the owner configured an envelope rule that requires it.

## Extension seams

Optional robot model sources are non-opening immutable bundles under
`waddle_sdk.robots.models`. For reference simulators,
`waddle_sdk.simulators.articulated_model_sources` preserves moving hand geometry,
linkages and native actuators; `reference_model_sources` deliberately freezes the
hand for planning. Retain each bundle's licenses and stated physical limitations.

A manifest robot factory accepts `PartConfig`, returns a declaration-only `Rig`, and defers bus/thread creation to `Rig.build_arms`. A manifest-loaded part factory returns one bare `Arm`; site composition assigns its manifest part name. A camera factory accepts `CameraConfig` and is called only during site open. A shared simulator may instead declare a `WorldConfig` factory returning `SimulationBackend`; the SDK opens that world once after authorization, advances it once per composite tick, and adapts its optional part and camera facets into the same runtime contracts.

Use structural protocols rather than inheritance. Keep vendor imports lazy. Connection credentials stay in manifest secret references and are resolved only for opening. Package facts with provenance and test them against the pinned vendor artifact where possible.

Use public `robots.metadata` and `cameras.metadata` helpers to interpret descriptions; preserve per-scope missing metadata. The SDK validates physical conversions and camera intrinsics, while applications own orchestration. See the public hardware-backends guide for these contracts and the optional shared `robots.models.ModelSourceProvider` contract.

Site context entry acquires SDK-owned per-user site-ID exclusivity before factories
run. `SiteOwnershipError.code` distinguishes contention and uncertain teardown;
uncertain teardown retains ownership. The optional `MediaRuntimePort.media_tracks()`
returns native publisher identities and last local attempt status, not proof of
viewer delivery. See the public site ownership/media guide for scope and limits.

## Named-part operations

Optional `NamedPartsObservationPort.observe_parts()` retains healthy measurements
and exact per-part faults. `NamedPartsRunPort.step_parts()` submits named
`JointPositionCommand` values through the native gate and returns per-part receipts.
Check `observation.named_parts` / `action.named_parts` support facts. These are
independent submissions under one site lease, not an atomic multi-arm transaction
or a promise of hardware fault containment. Configured cross-part envelope checks
remain mandatory. See `docs/python/site.md` for completion and failure semantics.
SDK-owned receipts include local monotonic bounds for the dispatch call after it
acquires the shared dispatch boundary; applications may use the start bound for
cadence without counting unrelated lock wait as driver execution.

## Finite joint playback

`FinitePlaybackRunPort.start_playback(FiniteJointMotion)` admits one complete,
bounded buffer for a named part when `execution.finite_joint_playback` is present.
`JointReference` carries an exact position command and local offset;
Binding API 16 adds optional `PlaybackPolicy.reference_only`, gated by
`execution.reference_joint_playback`. It ends with `references_dispatched` only
after every reference is acknowledged and fresh post-endpoint feedback passes
normal tracking checks. It makes no arrival, stationarity or grasp claim and
cannot be combined with a gripper outcome policy. Start, gate, pacing, tracking,
deadline and stop behavior are unchanged. Snapshots retain
`last_acknowledged_ns` on the SDK session clock so callers can budget subsequent
goal checks from device acknowledgement rather than remote reply receipt.
No new thread, writer or authority is created; ordinary policies remain unchanged.

Binding API 15 adds `minimum_settle_samples` (default one), separately advertised
as `execution.sampled_joint_playback`. Fresh eligible samples must satisfy both
the count and existing dwell; duplicates do not count and velocity/goal loss
resets the velocity-settling count. Encoder windows retain at least the requested
count and three samples. Gripper policies retain their separate outcome rules.

`PlaybackPolicy` supplies selected-row start/completion tolerances and bounded
settling, lateness and timeout. Core validates and executes it on the existing
control thread. Python adapts observation and the same gate/owner envelope used
by `Run.step`; never add a frontend timer, replay loop or remote per-step call.

The handle's `snapshot` and `wait` retain identity, dispatch count, measured state,
status and original failure evidence. `cancel` requests native drain/Hold. A
terminal handle cannot cancel a successor. Direct steps return Busy while playback
is active, and run/site teardown drains before hardware parking/close. Binding API
14 prevents stale native wheels from opening hardware through this surface.
Snapshots retain the completed stop callback in `cleanup`, separately from the
original motion failure. Missing or failed cleanup is not confirmation, and a
queued Hold/e-stop request cannot replace it. Callback success is not measured
physical stopping; preserve its session-clock timestamp and original error.
Per-reference force modifiers, dynamic reference updates and coupled multi-part execution remain
unsupported. See `docs/core/finite-reference-playback.md` in a source checkout.

An optional `PacedJointCurve` in the policy requires the separate
`execution.paced_joint_playback` fact. Supply an already checked polynomial;
native code owns bounded evaluation, acknowledged progress, measured headroom
and minimum submission spacing. A late read never skips curve phase. This does
not add planning or measured grasp completion.

Optional `PlaybackFeedbackPolicy` requires `execution.feedback_joint_playback`.
It adapts unselected held groups and measured rows locally, filters velocity hints
by measured error, and optionally uses `PositionWindow` encoder stability in place
of instantaneous velocity. Preserve disjoint rows, exact selected curves, owner
admission, original timeout and distinct arrival/miss evidence. `during_motion`
may retain eligible stability history, but completion still needs the endpoint
acknowledgement and fresh feedback. No Python execution loop or force/contact
classification is added; see the same normative playback page.

`GripperPlaybackPolicy` separately requires `execution.gripper_playback` and
the Site's declared jaw index and physical mapping. Native code owns bounded
position correction, acknowledgement-completion spacing and measured arrival,
contact-stall or force completion. Optional constant force requires both force
control and sourced feedback; zero releases force. A closed empty jaw cannot
qualify as a force grasp. Preserve source/estimate evidence, original failure and
normal stop/drain; successful force completion keeps the driver's mode active.
This measures generic jaw conditions, not object identity or calibrated contact.

## Imported simulation worlds

Reference MuJoCo configuration can select an explicit `native_scene` with
portable asset digests, robot prefixes, camera bindings and an initial keyframe.
Use the ordinary SDK world/part/camera lifecycle; keep trusted reset and ground
truth outside participant observations. The SDK simulation guide documents the
configuration and compiled camera/kinematic checks. Imported assets do not
establish physical calibration or contact fidelity.


## Reviewed site setup

The optional `setup` extra supplies `waddle_sdk.workspace.tui.run_initializer`.
Discovery and reviewed site publication never open an arm; explicit camera preview
uses the camera inspection lifecycle. `workspace.initialization.build_site` accepts
reviewed rows or an explicit source manifest, preserving site identity and bounded
source/calibration snapshots. Its trusted configure callback runs before atomic
publication and must not open hardware. Linux absent-destination rename is the
qualified filesystem boundary. Normal SDK imports do not require the terminal UI.
