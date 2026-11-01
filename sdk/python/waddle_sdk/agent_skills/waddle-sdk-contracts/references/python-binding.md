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

Binding API 40 exposes `execution.transit_playback`: an admitted same-arm
successor accepting `references_dispatched` may waive intermediate settling.
Require the native terminal result and retain its `transit_successor` ID. Cancelled,
expired, gripper or other-arm successors grant no waiver; required goal tolerance
and the child's fresh-start check still apply. Reference crossing is not arrival.

Binding API 39 permits revisable local settling without a separate measured goal:
require `execution.revisable_settled_playback`, set `allow_settled=True`, and
submit checked finite curves. Completion uses the accepted curve endpoint and
local stationarity; `settled` remains distinct from `arrived`. Preserve original
deadlines, cancellation and unknown-reply handling. No new worker is introduced.

Binding API 38 allows explicit `PlaybackGoal.allow_updates` with pose and dwell.
`update_goal(PlaybackGoalUpdate(...))` queues a target on the same worker; inspect
`goal_update` for acceptance and use `goal_target` paired with retained feedback.
Unknown responses never authorize replay. Original policy/deadlines and terminal
results remain binding. Explicit-goal curves may reuse the native motion-pacing
allowance; target updates never earn credit. See the finite-reference-playback guide.

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

Binding API 42 exposes `InputPlaybackRunPort.start_playback_input(timeout_ns)`
and `PlaybackInputPort` under `execution.input_watchdog_playback`. Bind the input
to checked work with `Run.start_playback(..., input=control)`. Forward newly
accepted human input with increasing sequences independently of planning/status;
never renew automatically. Release and expiry cannot be revived. The device
relay accepts renewals outside ordinary I/O admission and releases through the
reserved Stop lane. Release alone is not completed Hold or physical stopping.
See `docs/core/finite-reference-playback.md#input-arrival-watchdog`.

Binding API 41 adds optional `PlaybackPolicy.expected_start` under
`execution.expected_start_playback`. It separates measured start admission from
the first command without changing tolerances or owner limits. See
`docs/core/finite-reference-playback.md#measured-starting-position`.

Binding API 37 adds optional `PlaybackGoalDwell` under
`execution.dwell_goal_playback`. Use SDK observation-clock evidence, preserve the
nominal/outer tolerance bands and independent stationarity, and never renew a
deadline or infer a moving-goal update from dwell alone. See the normative
finite-reference playback guide.

Binding API 36 adds opt-in `complete_in_resting_tail` for revisable paced goals.
It lets the worker settle during an acknowledged constant tail, after checkpoints
and any accepted future motion are resolved. Default false preserves requested
rest duration; see the normative finite-playback page for result semantics.

Binding API 35 adds `execution.playback_timing`. Use snapshot `elapsed_ns` for
local time since activation at the retained feedback sample; queued wait is
excluded and an unstarted execution has no elapsed duration. The normative
finite-playback page defines deadline and delayed-sample semantics.

Binding API 31 supports `PlaybackGoal` and local endpoint settling allowances.
Use SDK-owned measured outcomes for finite trajectories; original joint or
declared-TCP goals are independent of corrected reference endpoints. Pose goals
require local FK and the exact SDK base frame. For a selected custom TCP,
`execution.kinematic_goal_playback` accepts numeric `PlaybackKinematics` on the
goal; the normative finite-playback page defines its joint/frame binding. It
changes completion measurement only, never owner safety geometry. A delayed status read cannot renew
a deadline or change a retained result. See the normative finite-playback page.

Binding API 30 adds `DeferredPlaybackHandlePort.finish_references` with
`execution.deferred_reference_completion`. Opt-in finite, fixed reference-only
curves keep their local sampler and owner reservation after endpoint acknowledgement,
without more writes. Require a terminal snapshot after finish; acceptance alone
does not confirm completion. Preserve deadlines, final drift/gap diagnostics,
cancellation and ordinary Hold/drain. See the normative finite-playback page.

Binding API 26 supports explicit reference streams with `timeout_ns=None` and
`execution.streamed_joint_playback`. Require a watchdog and revisable reference
curves ending at zero velocity/acceleration. The local worker retains supervised
rest until continuation, stop or expiry. Finite deadlines remain unchanged; no
caller polling or curve update renews input liveness.

Binding API 25 adds optional `PlaybackWatchdog` and `WatchdogPlaybackHandlePort`
under `execution.watchdog_playback`. Use acquired SDK feedback time and reconcile
the native renewal receipt; delayed requests cannot revive expiry. Preserve
ordinary Hold/cleanup and the application input policy. See the normative
finite-reference-playback page for timing and blocked-callback limits.

`BoundedPlaybackRunPort.set_playback_bounds` optionally installs run-wide
`JointPositionBounds` under `execution.joint_playback_bounds`. The existing local
worker checks fresh bounded-part feedback and each reference before dispatch;
a shared bound/read failure requests global Hold even for scoped-cleanup buffers.
The caller owns geometry proof and serialized, acknowledged configuration. Empty
bounds remove the envelope. Direct commands and supervision substitutions check
the final gate-selected target; explicit controls retain takeover/drain semantics.
Bounds do not widen site limits or prove a physical stop.

Binding API 24 adds opt-in `PlaybackPolicy.part_hold`, requiring the separate
`execution.part_hold_playback` fact on a named scope. Cancellation/failure uses
that owner's existing Hold while siblings continue. Preserve scoped receipts,
original faults and unknown cleanup. Whole-site/group buffers refuse this policy;
explicit global controls and closure still drain all buffers. Geometry clearance
and uncertain-operation ownership remain application responsibilities.

Binding API 23 adds `execution.parallel_joint_playback`: distinct named buffers
interleave on the existing worker, with separate completion and unchanged owner
checks. Whole-site or overlapping scopes conflict. Site-wide Hold/e-stop/failure
cleanup drains active and queued siblings; never infer selectively scoped stop
or application collision clearance from this capability. Named direct commands
may address idle parts. See the normative finite-playback page for limits.

Binding API 22 adds `CoordinatedJointMotion` for every declared Site part in exact
declaration order. Require `execution.coordinated_joint_playback` on every part;
flatten reference/policy/curve rows in that same order. One native worker/clock
uses all-part owner preflight and normal site Hold/drain. Driver writes remain
sequential, so preserve partial-write uncertainty. Scalar group gripper policies
are refused. See the normative finite-playback page for the complete contract.

Binding API 21 adds `PlaybackPolicy.credit_pacing_delay` under
`execution.paced_playback_deadline`. It is opt-in for reference-only polynomial
execution: complete acknowledged moving ticks credit only their excess local
wall time. Exact stationary tails and stalled phase cannot renew the budget.
Use native `curve_progress.pacing_extension_ns`; never infer credit from cloud
receipt delays. Default false retains an absolute timeout. Updates/withdrawals
preserve accrued credit without granting more. See the finite-playback page.

Binding API 20 adds optional `WithdrawablePlaybackHandlePort.withdraw_curve`
under `execution.withdrawable_joint_playback`. Bind the accepted update ID and
incumbent revision; wait for the new receipt's accepted result before treating
the suffix as removed. Late withdrawal cannot rewind an acknowledged handoff,
and a delayed old request cannot remove a replacement. The incumbent continues
with its original deadline. This is not a stop or geometry proof; never replay
after an uncertain reply. See the normative finite-playback page.

Binding API 19 adds `feedback_reference` and
`execution.playback_feedback_reference`. This pairs each snapshot's feedback with
the preceding acknowledged command; a later `last_reference` cannot relabel that
observation. It is absent before the first acknowledgement. The offset is global
phase for paced playback, not elapsed wall time or proof of physical arrival.

Binding API 18 adds `PlaybackCurveUpdate` and the optional
`RevisablePlaybackHandlePort` under `execution.revisable_joint_playback`. Opt-in
reference-only curves accept one revision-checked future suffix while local
playback continues. Preserve handoff position/velocity/acceleration, original
limits and deadline; reconcile request receipts and acknowledged revisions
without replaying uncertain updates. See the normative finite-playback page for
admission restrictions, clock semantics and the separate application checks.

Binding API 33 additionally advertises `execution.revisable_goal_playback`.
Checked corrections can use that same update API while retaining one measured
goal, execution identity and original deadline. The SDK finishes locally and
advances admitted dependents without caller handoff. Original checkpoints must
be observed before replacement and are never repeated. See the finite-playback
page for resting continuations, pending suffixes and completion policies.

Binding API 17 optionally records bounded `PlaybackPolicy.checkpoint_offsets_ns`
under `execution.checkpointed_joint_playback`. Retained snapshots bind fresh local
feedback to acknowledged reference phase; markers do not imply arrival or motor
acquisition timing. Delayed polling must use the retained samples, not final
feedback as a substitute. See `docs/core/finite-reference-playback.md` for limits
and clock semantics.

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
terminal handle cannot cancel a successor. Direct steps return Busy on scopes
that overlap active playback, and run/site teardown drains before hardware parking/close. Binding API
14 prevents stale native wheels from opening hardware through this surface.
Snapshots retain the completed stop callback in `cleanup`, separately from the
original motion failure. Missing or failed cleanup is not confirmation, and a
queued Hold/e-stop request cannot replace it. Callback success is not measured
physical stopping; preserve its session-clock timestamp and original error.
Per-reference force modifiers remain unsupported. See `docs/core/finite-reference-playback.md` in a source checkout.

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
