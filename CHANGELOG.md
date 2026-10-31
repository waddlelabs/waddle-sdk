# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

- Confirm failed Site startup cleanup before clearing the device relay's session,
  so ordinary disconnect does not hang on a session already closed by entry.
  Uncertain teardown retains ownership and the original error chain.

- Prepare coordinated playback for ordered subsets of declared joint parts
  (binding API 27), with native member reservations, ordinary gate/owner preflight,
  and unchanged global group cleanup. A separate support fact gates selection.

- Add explicit watchdog-limited reference streams through `timeout_ns=None`
  (binding API 26), with separately declared support. Bounded curves remain at
  supervised rest until a checked continuation, cancellation or watchdog expiry;
  finite motion deadlines and completion retain their existing semantics.

- Add optional local playback watchdogs on the existing native worker (binding
  API 25), with session-clock feedback renewal and retained receipt evidence.
  Expiry keeps the original failure and ordinary Hold/cleanup; delayed or repeated
  messages cannot revive or extend expired execution. No frontend timing loop.

- Distinguish direct playback-bound refusals from synchronous owner-envelope
  cleanup: their global Hold is queued, not yet a completed stop receipt.

- Add optional run-wide joint envelopes for buffered execution, with a separately
  declared capability and active-run relay binding. The existing local worker
  validates fresh required-part feedback and each reference, requesting global
  Hold on shared bounds/read failure while retaining original faults. Bounds are
  copied atomically; geometry proof remains the caller's responsibility.

- Add opt-in per-part Hold cleanup for named finite buffers (binding API 24).
  Preserve sibling execution, original errors and structured cleanup evidence;
  global Hold/e-stop still drain all buffers. Require separately declared support
  and reject whole-site/group scope before local I/O. Defaults remain unchanged.

- Admit independent named finite buffers on one serialized worker (binding API 23).
  Preserve per-buffer clocks/completion, reject overlapping or whole-site scopes,
  and permit named direct commands only on idle scopes. Site-wide stops and failure
  cleanup drain every active/queued buffer, retain original faults and fence new
  admission until cleanup completes; no extra device owner or writer is created.

- Add coordinated whole-site joint buffers on the existing native playback worker
  (binding API 22). Preserve declared part order, one shared curve/acknowledgement
  clock, the slowest command cadence, all-part envelope preflight and normal
  Hold/drain. Public Python and relay values require separate coordinated support;
  sequential driver writes retain partial-failure uncertainty.

- Add opt-in SDK-local pacing credit for reference-only polynomial playback
  (binding API 21). Complete acknowledged moving ticks retain the original finish
  allowance; stationary tails and stalled progress cannot renew it. Default
  absolute deadlines stay unchanged, and snapshots retain the accumulated credit
  independently of remote response timing or curve updates/withdrawals.

- Add explicit withdrawal of a named, accepted future curve suffix (binding
  API 20). The existing native worker confirms removal before handoff, refuses
  stale/late withdrawals and preserves the incumbent, pacing and deadline.
  Public handles and the optional device relay retain request identity and
  pending/accepted/rejected receipts; withdrawal never implies stopping.

- Pair each native playback feedback sample with its preceding acknowledged
  reference (binding API 19, `execution.playback_feedback_reference`). Later
  dispatch and delayed status replies retain that causal pair; no extra writer,
  command, authority or timing assumption is introduced.

- Add opt-in revision-checked future curve suffixes on the existing native worker
  (binding API 18), with nonblocking update admission and retained receipts through
  the public handle and relay. Preserve handoff position/velocity/acceleration,
  local pacing, original deadlines and ordinary stop/drain behavior. Refused
  updates leave the incumbent intact; accepted updates do not imply arrival.

- Retain optional bounded checkpoint measurements inside local finite playback
  (binding API 17). Delayed polling preserves intermediate feedback and its actual
  acknowledgement/observation times without adding intermediate arrival gates.

- Add optional guided site setup with shared hardware profiles, explicit camera
  preview and atomic source/calibration copying. Trusted application configuration
  runs in the private stage; failed or changed inputs never publish a new site.

- Share runtime-independent connection configuration and private credential-file
  reading with relay consumers. Retain bounded versioned JSON, private regular
  files, explicit paths, WSS validation and numeric-loopback development opt-in.

- Add an optional application-authenticated device relay and shared confined
  workspace files. The relay preserves native ownership, reserved Hold/e-stop
  admission, exact bounded SDK values, run-bound playback and uncertain cleanup.
  Hidden paths and recordings stay out of transfer. No planner, login, listener
  or extra dependency is added to ordinary SDK imports.

- Add separately advertised reference-only completion and native acknowledgement
  timestamps (binding API 16). A finished buffer reports dispatch evidence,
  never arrival or stationarity; original tracking, deadline and stop gates remain.

- Add separately advertised minimum settling-sample counts to native finite
  playback (binding API 15), in addition to its existing dwell requirement.
  Slow or duplicate observations cannot stand in for fresh stationary samples;
  velocity/goal loss resets the count. Existing default policies stay unchanged.

- Retain the completed Hold/e-stop callback receipt in native playback snapshots
  (binding API 14), separately from the original motion failure. Failed or absent
  cleanup cannot be inferred successful from a terminal loop or queued stop
  request. This records callback completion, not measured physical stopping.

- Gate native scene-replacement tests on the optional MuJoCo capability, matching
  the remaining simulation suite and preserving the base installation footprint.

- Reacquire reference simulation camera state after a transient snapshot-transfer
  delay, with at most three attempts under the unchanged 50 ms pairing bound.
  Retain replacement model identity and original persistent errors; never
  restamp or render a rejected snapshot.

### Added

- Optional native gripper playback, with binding API 13 and an independent support
  fact. Preserve the declared jaw mapping, local acknowledgement spacing and
  ordinary owner gates. Distinguish three-sample measured arrival, sustained
  closing contact and sourced force completion; reject empty-jaw effort at the
  closed stop. Constant force is capability-gated, zero releases it, and original
  failures/cancellation retain normal Hold/drain. Snapshots retain jaw evidence.
  Native and fake-owner tests do not qualify physical contact or calibrated force.

- Optional measured playback policies on the existing native worker, with binding
  API 12 and a separate support fact: held-coordinate correction, measured-row
  passthrough, directional velocity hints and bounded encoder-window settling.
  Preserve exact moving curves, native owner admission, original timeout and
  distinct arrival/stationary-miss outcomes. Snapshots retain settling evidence;
  no force/contact or physical performance qualification is implied.

- Optional exact polynomial playback on the existing native worker, with a
  separate paced-playback support fact and binding API 11. Progress advances on
  acknowledgements within measured headroom, never from late wall time; partial
  steps retain the curve and scale velocity hints. Preserve minimum command
  spacing, tracking errors, absolute timeout and original lifecycle drain. Status
  includes acknowledged curve progress and the last confirmed reference. Fixed
  reference offsets and lateness semantics remain unchanged.

- Public `Run.start_playback(FiniteJointMotion)` and optional structural execution
  ports, using native timing and the ordinary local gate/owner envelope. Add a
  per-part support fact, measured status/fault snapshots, cancellation, and drain
  before run/site teardown. Preserve exact references and velocity hints; reject
  concurrent direct writes and unsupported force modifiers. Python/native binding
  API 11 rejects stale extensions before hardware opens. Connector integration
  and physical qualification remain pending.

- Native session execution of finite reference buffers on the existing serialized
  control thread. Validate the declared scope before local I/O; retain measured
  completion, original faults and separate Hold/e-stop outcomes. Cancel/drain on
  control takeover, episode closure and shutdown without replaying references or
  letting stale handles stop successors. Slow declared feedback cannot postpone
  reference-lateness checks or the buffer deadline. Connector transport remains
  pending; no installed runtime or physical owner is activated.

- Native finite joint-reference accounting using existing action chunks. Separate
  admission/dispatch from measured arrival and optional settled misses; refuse
  stale starts, late or unacknowledged references, invalid feedback and cancellation
  without replay. Connector integration remains pending.

- Optional private compiled MuJoCo model reuse for verified native scenes. Fresh
  model/data instances preserve independent state; source/native-version identity
  and binary checksums prevent stale/corrupt cache reuse. Existing source errors,
  simulation ownership, camera capture and control gates remain unchanged.

- Optional finite two-YAM reference assemblies in authored scenes: handover,
  movable container, peg/socket and loaded tray, with bounded repeatable XY
  variations, native contacts and ordinary editable extra props.

- Optional trusted scene replacement for prepared native MuJoCo environments.
  Retain the opened site, robot/camera ports and physics/render processes;
  preserve robot/camera contracts, reset changed objects and fence old images.
  Unsupported backends retain ordinary close/open behavior.

- Compile bounded editable tabletop scenes with native reference robots, cameras,
  immutable reset state and an easier contact profile. Preserve actual force,
  missed-grasp and release outcomes; all six robot/arm configurations retain their
  declared camera capabilities.

### Fixed

- Use the native gate's selected scope for neighbor collision checks and velocity
  hints when a named supervision command replaces a whole-site caller. Preserve
  the selected stream, original owner refusals and ordinary playback cleanup.

- Apply authored scenes' finger-contact material to unnamed two-YAM fixture
  collision shapes, including compound children. Existing compiled scenes stay
  immutable; both-arm peg retention/release and ordinary-material drop controls
  exercise actual native contact outcomes.

- Serialize generated YAM source poses at twelve decimal places with canonical
  zero. Equivalent BLAS results on different hosts now produce matching model
  bytes, while exact vendor asset checks and saved model identities remain intact.

- Prioritize queued robot feedback/control over reference simulation camera and
  viewer snapshots, including the independent renderer's state acquisition.
  Preserve FIFO within each priority and finish the active pipe transaction.

- Reduce excessive retention in editable scenes: moderate sliding friction, a
  1 mm torsional patch and three NoSlip iterations retain centered grasps while
  letting weak/edge grasps fall. Preserve native actuation, force limits and release.

- Use bounded native motor effort for MuJoCo YAM force hold under unequal and
  changing finger loads. Retain the drive's damping as a passive transmission
  load; restore the original position servo and limits on release, Hold or reset.
  Preserve force ceilings, native contact dynamics and measured feedback. Cover
  stationary force acquisition, a bilateral-contact bottle lift and gravity release
  on the unchanged shampoo scene.

- Declare zero distortion for default reference-simulation pinhole cameras so
  metric image consumers receive explicit rectified intrinsics. Preserve
  supplied camera profiles and their declared optical properties.

- Isolate reference MuJoCo rendering from physics/control with bounded camera work,
  at most two inherited rendering CPUs, acquisition-time robot feedback and reset-generation
  fencing. Keep measured motion outcomes and image freshness checks unchanged.

- Read completed immutable YAM feedback publications without waiting on the CAN
  transaction lock for the verified pinned I2RT implementation. Preserve original
  motor-loop behavior and feedback liveness/stall checks; unverified revisions
  retain vendor observation locking.

- Require complete Orbbec color/depth framesets before alignment; report absent
  filter output as an Orbbec RGB-D error instead of a Python attribute error.

- Preserve active RealSense RGB distortion identity, including append-only wire values for modified and inverse Brown–Conrady. Do not mislabel these images as rectified or ROS plumb_bob.

- Add `waddle_sdk.media.PreviewPublisher` for camera-only RGB publication from a
  separate presentation process, without Site/device/run/control ownership.
  Preserve source dimensions/timestamps and native demand/rate/bitrate bounds.
  Python/native binding API advances to 9; rebuild the matching base/media pair.

- Demand-driven previews skip conversion/encoding when the room has no peers,
  independently of SFU layer-pause timing. Disconnect/worker exit clear demand.
  Previews capped at <=2 fps reduce idle video polling from 200 Hz to 10 Hz,
  bounding added presentation/shutdown polling latency to 100 ms.

- Presentation-only LiveKit can exclude intervention data topics and teleoperation
  intake. Video lifecycle polling avoids the controller snapshot lock; native
  media metadata releases the Python interpreter lock. Keep short, bounded frame
  admission without an extra per-frame interpreter handoff.

- Prepare matching SDK and media wheels as 0.1.13; the optional preview bindings
  require API 9, which was not included in the published 0.1.12 pair.

- Add optional LiveKit preview width, fps and per-track bitrate ceilings, RGB-only
  presentation and demand-driven publisher dynacast. Preserve original RGB-D
  samples and independent agent still rates; skip unused depth colorization.
- Bound pending video to the latest frame and separate its native worker from
  control-plane stills. Lower Linux media worker priority and bound publication
  waits. Demand-driven preview signaling starts/retries asynchronously, without
  delaying session startup. Python/native binding API advances to 9; rebuild both extensions.

## Released changelogs

- [`0.1.12` — 2026-10-10](docs/changelogs/CHANGELOG-0.1.12.md)
- [`0.1.11` — 2026-08-30](docs/changelogs/CHANGELOG-0.1.11.md)
- [`0.1.10` — 2026-08-29](docs/changelogs/CHANGELOG-0.1.10.md)
- [`0.1.9` — 2026-08-28](docs/changelogs/CHANGELOG-0.1.9.md)
- [`0.1.8` — 2026-08-28](docs/changelogs/CHANGELOG-0.1.8.md)
- [`0.1.7` — 2026-08-28 (not published)](docs/changelogs/CHANGELOG-0.1.7.md)
- [`0.1.6` — 2026-08-28 (not published)](docs/changelogs/CHANGELOG-0.1.6.md)
- [`0.1.5` — 2026-08-27](docs/changelogs/CHANGELOG-0.1.5.md)
- [`0.1.4` — 2026-08-27 (withdrawn)](docs/changelogs/CHANGELOG-0.1.4.md)
- [`0.1.3` — 2026-08-27](docs/changelogs/CHANGELOG-0.1.3.md)
- [`0.1.2` — 2026-08-25](docs/changelogs/CHANGELOG-0.1.2.md)
- [`0.1.1` — 2026-08-24](docs/changelogs/CHANGELOG-0.1.1.md)
- [`0.1.0` — 2026-08-23](docs/changelogs/CHANGELOG-0.1.0.md)
