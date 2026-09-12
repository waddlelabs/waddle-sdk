# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Added

- Optional YAM `arm_gains` configures independent six-joint KP/KD values while
  preserving hand gains and recovery behavior. Paired raw/SDK benchmarks apply
  the same settings and distinguish requested gains from predicted MIT encoding.
  Explicit and scaled gains are validated before CAN startup; unrepresentable
  requests now fail instead of being silently clamped by the vendor codec.

- Real-camera RGB/depth publication and reconnect tests through the native SDK
  and configured LiveKit service, using isolated camera ownership without opening
  physical arms. Reports distinguish source dimensions from adaptive video sizes.

- Opt-in paired YAM benchmarks run pristine I2RT and SDK processes with matching
  controller settings and sampled trajectories. Reports preserve measured joint
  and TCP arrival, timing, vendor diagnostics, failures and cleanup; explicit
  comparison margins enforce accuracy and settling-time noninferiority.
  Optional reviewed rest targets return healthy trials to supported parking before
  backend hand-off. Parking remains separate measured evidence; parking failures
  block the next backend, and original motion/background faults prohibit further
  trajectory commands.
  Both backends now require observable post-start CAN feedback before timing a
  trajectory, preserving startup evidence separately and refusing stale startup
  within a bounded deadline without moving or changing envelope checks.
  Comparison rejects nonzero child exits, missing or unequal manifest hashes,
  and missing, duplicated or reordered phases even when recorded arrivals pass.

- Unify software and live behavior tests under `sdk/tests`; `pytest --live` selects
  viable camera/robot checks from metadata and optional motion profiles, reports
  missing prerequisites and stays disabled in CI/releases.
- Explicit `SiteSession.close(torque_release_authorized=True)` for authorized
  headless teardown, sharing context-exit resource and ownership handling.

- Shared site-ID process ownership in `Site.open()`, acquired before adapter
  construction and retained after uncertain teardown. Applications using the
  same site ID now coordinate through the same SDK lock.
- Optional `MediaRuntimePort` and `SiteSession.media_tracks()` expose native
  publisher identities, per-stream last-attempt status and drop counts without
  reconstructing track facts from observations.

- Shared optional `ModelSourceProvider` and immutable `ModelSources` bundle for
  complete source geometry, assets, named bindings and attribution. Any robot
  adapter can expose the same non-opening module extension without registry edits;
  absent support and selected failures remain distinct.

- Public non-opening robot/camera metadata helpers: named part action spaces,
  validated physical gripper mappings with reversed action ranges, effective
  camera declarations and intrinsics with explicit optional depth scaling.
- The YAM `model_sources` extension provides verified source geometry, attribution,
  TCP attachment and coupled finger travel from the exact optional I2RT pin,
  without opening hardware or selecting a planner.

- Export public `waddle_sdk.FEATURES` as the selected native core's feature set,
  allowing applications to check media support without importing SDK internals.
- Add opt-in public SDK camera/LiveKit acceptance using scoped grants, actual RGB
  and depth-display frames, native signaling reconnect with the same SiteSession
  and Run, and independent viewer rejoin. Document matching media dependencies
  and the explicit test lifecycle without changing hardware/media lifecycle behavior.

- Add manifest-selected shared simulation worlds through the public
  `WorldConfig`/`SimulationBackend` contract. World-backed parts and cameras reuse the
  ordinary SDK runtime, support matrix, owner envelope, observations, and RGB-D sample
  path consumed by applications.
- Add a shared-world MuJoCo reference backend with one physics state for all attached
  parts and cameras, exactly-once world stepping, episode reset, derived camera
  intrinsics, and aligned metric Z16 rendering.
- Add the strict `waddle.scene/v1` portable URDF scene contract, non-overwriting
  `waddle-sdk sim backends|init|validate|compile|run` workflow, seeded value resolution,
  replayable file-hash evidence, and a checked scene/wrist RGB-D example. The MuJoCo
  compiler covers robot placement, reviewed joint limits, scene geometry, camera
  optical poses, lights, materials, visual/physical coatings, and link-local safety
  spheres while emitting an ordinary `site.yaml`.
- Add installed simulator extension points through the
  `waddle_sdk.simulation_backends` and `waddle_sdk.simulation_compilers` entry-point
  groups while retaining explicit `module:callable` targets.
- Add a lazy ROS 2 shared-world backend for simulator graphs. It maps reviewed joint
  declarations to ros2_control or `JointState` position topics, pairs RGB/depth by
  acquisition time, derives active intrinsics from `CameraInfo`, and supports an
  optional `std_srvs/Empty` reset service. This provides the ordinary SDK boundary for
  ROS-connected Gazebo and Isaac Sim processes.

### Changed

- Owner-envelope refusals preserve their originating safety fault, exact reason,
  part, measured/commanded values and bounds in the submission receipt and event.
  Independent hold failures remain separate diagnostics.

- Native binding API 4 adds publisher snapshots; rebuild base and media extensions
  together. Camera/arm/world teardown failures now prevent site ownership release.

- YAM source loading implements the shared adapter contract and returns complete
  articulated MJCF, preserving physical slide coordinates and coupling. Remove
  the unpublished YAM-specific bundle fields from the public source boundary.
- Document and regression-test xArm G2's physical millimetre API boundary. Vendor
  code owns nonlinear motor-pulse conversion; parallel opening mappings stay linear.

- Camera depth resolution shares metadata validation and refuses malformed
  distortion coefficients before calling a vendor resolver.
- Correct the documented root-export regression to include the existing public
  `FEATURES` export.

- Make `AGENTS.md` the canonical guide and `CLAUDE.md` its relative symlink; keep
  build and public-contract guidance together, with publication-confirmed changelog
  archiving. Clarify public runtime documentation, examples, diagnostic messages,
  and historical notes without changing SDK APIs or execution behavior.
- Keep public rationale and amendment history while removing unrelated
  implementation and naming roadmaps; clarify its informative status.
- Document the optional MuJoCo prerequisite for the complete scene-compiler test
  suite and headless Linux rendering configuration.

- Set YAM absolute arm gravity factors to `[1.0, 1.1, 1.2, 1.3, 1.0, 1.0]`, using
  rounded two-arm bench calibration for joints 3/4. A validated, declaration-frozen
  `gravity_comp_factor` option overrides all six values per arm. Pass it into I2RT
  before its servo starts; retain the vendor gripper factor, PD gains, and friction.

- Expand the YAM tabletop initializer preset and example to workspace bounds
  `[-0.7, -0.7, 0.0]` through `[0.7, 0.7, 1.0]` metres in each arm base frame.
  Existing site declarations remain authoritative and are not rewritten.

- Preserve the existing `waddle_sdk.robots.mujoco:arm` factory as a compatible
  private-world adapter while documenting `waddle_sdk.robots.mujoco:backend` for
  multipart and RGB-D scenes.
- Require MuJoCo 3.5 or newer for the portable `MjSpec` scene compiler and renderer;
  the existing runtime remains lazy behind the `[mujoco]` extra.

### Fixed

- Replace a scheduler-dependent per-part uplink test with controlled reducer
  admission times, verifying independent 10 Hz budgets and nonstarvation without
  requiring unrelated streams to have identical phases.

- Preserve original runtime exception text, JSON metadata and nested causes through
  `RuntimeFault.from_exception`; existing typed faults pass through unchanged and
  only credential material is redacted. Teardown/reporting failures no longer
  replace the primary live motion failure.

- Refuse YAM reads and commands when its CAN/server writer has stopped or real
  feedback cache updates stall, even if vendor observation timestamps advance.

- Accept optional grasp metadata alongside the standard YAM physical gripper
  mapping when resolving model sources; retain rejection of changed jaw/action
  mappings and all source geometry/provenance validation.

- Keep MuJoCo renderer creation and destruction on its owning camera-pump thread so
  EGL/OpenGL RGB-D capture closes without cross-thread context failures.
- Transform link-local conservative collision-sphere offsets with each simulated
  body's pose instead of treating every sphere as body-origin centered.

## Released changelogs

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
