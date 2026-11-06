## 2026-11-02 — Curved hollow tabletop bowls

- Give newly authored bowls smooth curved sides, a deep open interior and a
  small flat foot, with separate convex wall contacts that support nesting.
- Preserve finite mass, native contact/release and canonical reset. Replace
  quadratic contact-pair bookkeeping without changing pair materials.

# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Added

- Ship reproducible default object geometry for all seven tabletop object kinds
  and both contact policies. Bind names, poses and colors from private copies;
  custom dimensions and stale or absent bundles retain ordinary compilation.

### Changed

- Reuse bounded parsed object/contact fragments across authored additions with
  different prop names and poses. Preserve exact geometry, pair values, external
  bindings and full-source fallback. Copy during parent attachment so contact
  references to robot geometry survive; restore private templates after failure.
  Retain native binding handles once; verify seven object kinds against every
  native array and ten physics steps, with 58 focused cases passing.

- Preserve complete native source validation on Python 3.10 with bounded-memory
  streaming SHA-256 when hashlib.file_digest is unavailable.

- Reject the resident native preparation experiment after whole-addition timing
  fails to improve. Restore the qualified SDK runtime/API and retain all evidence.
  All 62 restoration checks and changed-source lint pass with exact baseline parity.

- Add bounded trusted authoring templates for final prop translations, preserving
  exact native/reset bytes, independent models and consumer source validation.
  Unsupported dependencies and other scene changes retain ordinary authoring.

- Reuse a bounded private whole-scene template for isolated free-object
  translations, recomputing native constants and preserving source validation,
  independent models and full compilation for other changes/dependencies.
  All 104 native checks and 44 edits across 11 exact published presets pass,
  preserving live state/history, source XML and affected-object comparisons.
  All tested translations reuse the model; additions remain above one second
  when prepared after placement; parent editor prewarming is separate. Preserve
  native contact layout by fully parsing reordered unchanged contact sets.
  Retain both failing regressions, the disk-full attempt and verified retry.

- Verify exact retired contact references and remove authored bodies through native
  cascading deletion. Reject retained invalid references through the full parser.
  All 86 focused native and 1,349 default SDK checks, strict docs and all 44 edits
  across 11 published presets pass, preserving source, live state and history.
  Paired bowl deletion/image means fall from 6.000 to 5.488 s; addition remains
  above one second. Retain all samples, the failing baseline and verified archives.

- Carry immutable authored body comparisons through the existing private native
  model handoff, retaining source/binding checks, byte bounds and parsing fallback.
  All 44 edits across 11 published presets preserve live state/history and exact
  XML; source/native/default SDK checks and strict docs pass. Local addition remains
  above one second, with no general speedup established. Retain complete timings
  and original diagnostics with verified closure and archive evidence.

- Carry exact authored robot-contract comparisons with bounded private model
  handoffs, avoiding repeated XML parsing after fresh source validation. Preserve
  ordinary comparison fallback, selection checks and live-state transfer.
  Qualify source/state invariants and all 44 edits across 11 published presets;
  SDK commit work falls, but overall addition latency remains above the target.

- Extend recognized Playground contact preparations using only new geometry
  pairs, retaining exact serialized rows and normalized comparison records.
  Preserve complete source validation, native compilation and live-state transfer;
  unrecognized layouts keep the full builder. All 44 edits across 11 published
  presets pass with byte-identical XML, preserved state/history and no freshness
  retries. Paired bowl add/image means fall from 3.148 to 2.673 s; simple-scene
  timings are mixed and the sub-second target remains unmet.

- Reuse prepared Playground contact sections and serialized fragments, keyed by
  exact base contacts and ordered geometry names. Keep every contact pair and
  parameter, detached trees, fresh asset serialization and canonical reset keys.

- Prepare native authored-source comparisons from a detached copy of the exact
  tree just serialized, avoiding a redundant XML parse. Preserve byte-based
  callers, fresh asset hashes, native fallback and independent runtime models.

- Reuse bounded XML reference analysis by freshly verified content digest during
  native scene validation. Continue to read/hash every asset and check current
  path confinement, manifest membership and configuration; analyze the exact XML
  snapshot whose digest was checked, without reopening it during the same pass.
  Trusted authoring prepares reference analysis from its exact serialized tree.

- Retain private authored XML comparison records with prepared native scenes,
  avoiding repeated parsing and contact-key construction for the old revision.
  Continue to compare every new source and freshly hashed asset, compile an
  independent model and discard records with their specification on failure,
  eviction or fork.

- Store repeated Playground finger-contact parameters in one named XML default,
  retaining every native pair and its original contact settings. Reuse parsed
  preparations with unchanged pair-only defaults; expand those settings for
  attached additions and recompile changed or unsupported defaults normally.
- Preserve parsed robot/body/contact preparation when resizing same-topology
  inline bowl meshes. Recreate vertex/face-only native meshes in order, with fresh
  native collision bounds and mass properties; retain full-source fallback for
  unsupported mesh changes and independent live-state transfer.

- Reuse bounded reference XML and unchanged authored object fragments during
  Playground edits. Reread and hash every source asset; invalidate on reference,
  configuration or fixture-placement changes. Keep independent compilation,
  final source validation, canonical reset and ordinary live-state transfer.

- Reuse private authored MuJoCo specifications for supported object additions,
  removals, pose changes and primitive geometry/material edits. Compare full XML
  and freshly hashed assets, copy the preparation and parse only appended
  bodies/assets/contact pairs before compiling an independent model. Bound
  reuse lineages; keep full parsing for unsupported differences and failed reuse.
  Preserve canonical reset and ordinary live-state transfer.

- Parse site/scene YAML with the optional safe C parser, retaining safe Python
  fallback, fresh reads and ordinary manifest validation.

- Compile verified scenes through explicit virtual files on newer MuJoCo bindings.
  With the optional cache enabled, reuse bounded unchanged asset buffers across
  edits while freshly hashing source bytes and compiling independent models.
  Keep the original loader for older bindings and release buffers on failures.

- Transfer current simulation models to independent renderers through negotiated
  sealed descriptors, preserving fresh state/pixels and independent native models.
  Retain copied-buffer compatibility and close handles on partial-transfer faults.

- Retain the loaded robot/camera contract for live scene comparisons and bundle
  reference-worker post-edit state with its acknowledgement. New publications
  remain validated; legacy backends and multi-world sites retain final readback.


- Reuse compatible RGB/depth rendering contexts across native scene revisions.
  Rebuild for changed uploaded assets, planes or visual settings; continue to
  acquire fresh pixels from the current model/state with generation fencing.

- Let resident Playground authoring prefer a retained sealed model over a second
  disk-cache export. Keep ordinary source files and disk caching when sealing or
  bounded handoff retention is unavailable; general authoring still fills disk cache.


- Reuse resolved asset directories within each native source validation pass.
  Every asset still receives a fresh byte hash; no path or digest result survives
  between checks. Preserve portable-path, symlink and manifest rejection.


- Transfer matching authoring preparations to resident MuJoCo owners through
  immutable Linux file descriptors, retaining ordinary source checks and separate
  model/state ownership. Bound the private handoff cache to two models and
  256 MiB; retain the existing loader on unsupported systems.

- Retain a single tabletop scene preparation for authoring/model handoffs, with
  identical portable geometry and canonical reset arrays. Compare and compile
  native replacements from one verified byte snapshot, preserving source
  validation, cache corruption fallback and independent runtime state.

- Advertise prepared-playback relay support. Return state and configuration with
  observations/completion; check a submitted configuration revision and numerical
  joint preconditions locally before native admission. No planner or native API
  change; existing start, cancellation and completion policies remain authoritative.

- Add an advertised read-only relay observation bundle with one retained frame,
  description, support and bounded workspace reads; preserve per-file errors.

- Bind native scene replacement to robot-referenced assets rather than tabletop
  prop meshes. Preserve genuine robot/camera refusal and image fencing. Give
  administrative scene loading its own 60-second wait; control deadlines remain
  unchanged. One-/two-arm block and bowl replacement/removal regressions pass.

- Add native `drawer_cabinet` and `cleanup_station` authored fixtures for one/two
  YAMs, with a passive retrieval drawer and a rigid brush/loose-piece/dustpan scene.
  Preserve bounded variation, exact reset, contact dynamics and robot contracts.
- Keep qualified single-arm robot bases out of fixture-contact renaming, and
  classify native passive joint types using Python integers for MuJoCo enum
  compatibility. Existing scenes and supported joint kinds remain unchanged.

- Start authored bowl walls above their solid floor, removing coplanar wall
  soles that multiply table contact constraints. Preserve the open rim, wall
  pieces, total mass, materials and solver settings; newly compiled scene geometry
  and inertia change, while existing immutable scenes remain unchanged.


- Include local feedback acquisition inside paced motion intervals instead of adding its cost after every cadence wait.

- Preserve bent-path, IO cadence, headroom and gripper budget regressions at native playback boundaries.

- Add optional bounded playback call history with native session/epoch stamps,
  pending calls, original faults and explicit supersession; keep history reads
  separate from execution status and preserve the existing native worker. Actual
  SDK host-monotonic samples additionally support same-host camera diagnostics;
  native paired timestamps remain unchanged.


### Fixed

- Advertise existing scoped playback Hold on single-arm named sites as well as
  composite sites, without widening action or Hold grants.

- Preserve declared mechanical body contact exclusions in reference planner
  exports and assembled physical YAM sources, including opposing slide fingers.
  Existing published bundles and simulation contact physics remain unchanged.

- Keep live playback input across an individual buffer cancellation only after
  successful scoped Hold; faults and global controls still fence input.

- Sample fresh local feedback between slow paced-curve commands so accepted input
  can renew its watchdog. Preserve write cadence and expiry without input.

- Retain local gripper contact sample counts and compatibility evidence on failed
  attempts, without changing execution or cleanup.

- Preserve structured callback diagnostics in site-wide Hold/e-stop cleanup and
  completed-stop receipts. Attempt every declared part and retain all failures.

### Added

- Add optional trusted live scene edits preserving the running simulator state
  and native processes. Existing replacement/reset semantics remain unchanged.

- Add a native run-bound playback input watchdog, renewed on SDK input arrival
  independently of planning and status polling. Preserve ordered renewal,
  irreversible release/expiry, same-run binding and ordinary worker cleanup.
  Binding API 42 exposes public Python input handles and device-relay operations;
  renewals bypass ordinary I/O admission and release uses reserved Stop capacity.
  Cloud application integration and actual WAN qualification remain separate.

- Preserve execution deadline-during-feedback regression coverage on the native
  worker using explicit callback barriers; no runtime or policy change.

- Separate the expected measured start from the first playback command with
  optional `PlaybackPolicy.expected_start` (binding API 41). Preserve the original
  start tolerance and all owner command limits; reject stale local feedback.

- Permit explicit transit handoffs to admitted successors on the existing worker
  (binding API 40). Preserve required goal tolerance, completed references,
  cancellation/expiry gates, fresh-start checks and the distinction from arrival.

- Allow revised finite curves to complete with SDK-local settling even without
  an explicit goal (binding API 39). Score the accepted endpoint and preserve
  stationary misses, original deadlines and cancellation on the existing worker.

- Add opt-in same-frame pose updates for dwelled goals on the existing worker
  (binding API 38). Retain accepted target/revision with feedback and terminal
  results; reject stale updates without changing trajectories, policy or deadlines.
  Explicit-goal polynomial playback may reuse native motion-pacing credit;
  stationary tails, target messages and dwell do not increase the allowance.

- Add optional `PlaybackGoalDwell` on the existing worker for independent goal
  dwell with a wider reset band. Current nominal tolerance and stationarity
  still gate completion; snapshots retain observation-clock dwell evidence.
  Original deadlines, cancellation and default settling behavior are unchanged.

- Add opt-in local goal completion within a constant resting tail (binding API
  36, `execution.resting_goal_playback`). Preserve pending future motion,
  checkpoint observations, local settling, original deadlines and cancellation;
  callers no longer need to shorten a resting curve just to report arrival.

- Evaluate selected custom TCP goals from numeric `PlaybackKinematics` on the
  existing worker (`execution.kinematic_goal_playback`). Support hinge/slide
  chains and affine joint coordinates without a planner/model dependency, while
  retaining the owner's safety geometry, local settling and original goal result.

- Expose SDK-local execution elapsed time (`execution.playback_timing`, binding
  API 35), bound to retained feedback. Queued work starts its motion budget at
  activation; cloud planning can use the same elapsed duration without counting
  dependency wait or synchronizing clocks. Existing deadlines remain enforced.

- Expose nonterminal stationary-goal misses from the existing local settling
  checks (`execution.goal_progress`, binding API 34). Metal can plan a checked
  correction on the same execution without reconstructing a window from cloud
  samples; original goal completion, deadlines and cleanup remain SDK-owned.

- Keep one measured-goal execution across checked curve corrections (binding
  API 33). Reuse the existing worker and revision/withdrawal API, preserve the
  original goal, total deadline and checkpoint evidence, and let queued work
  depend on final local completion. Delayed updates cannot revive expired
  endpoint or motion allowances, including after blocked observation callbacks.

- Add dependent playback on the existing SDK worker (binding API 32). Reserve
  queued parts, start only after explicitly accepted predecessor results, and
  optionally expire a waiting plan on the SDK clock. Queued cancellation leaves
  a running predecessor alone; site-wide Stop still drains the whole queue.
  Public Run and device relay preserve retained identities without cloud handoff.

- Add SDK-local completion against an original joint or declared TCP goal,
  with local tolerance, settling and endpoint-timeout evaluation (binding API 31).
  Terminal outcomes require no caller polling or explicit finish request.
  Entering goal tolerance starts the full local settling interval; earlier
  stationary samples outside tolerance cannot end that attempt as a miss.

- Add opt-in deferred reference completion (binding API 30): retain the local
  sampler and owner reservation after a fixed curve ends, without repeating
  references. Explicit finish returns fresh final diagnostics under the original
  deadline and stop rules. Keep reference dispatch distinct from goal arrival.

- Add sample-bound encoder-window diagnostics for reference-only playback
  (binding API 29). Include the final fresh sample and its drift/gap resets,
  while preserving reference completion semantics and existing local execution.

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

- Add optional completed Hold/e-stop receipts through `SiteSession.stop` and the remote safety lane (binding API 28), including stopping after terminal playback without changing its result.

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
