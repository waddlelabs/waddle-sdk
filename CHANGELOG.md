# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Added

- Reference scene `render_quality` presets (`fast`, `standard`, `high`) using
  native renderer settings without changing robot, physics, or camera contracts.
  Existing scene files retain standard rendering.

- Replace primitive simulation robots with pinned manufacturer YAM/xArm7 URDF
  assemblies, actual visual meshes, masses, COMs and full inertia tensors. Match
  the live YAM linear hand and xArm G2 gripper with native coupled joints,
  nonlinear jaw mapping, shared convex hand collisions and mesh-derived
  planning bounds. Reuse the optional trajectory-velocity driver port. Include
  licenses, source hashes and an offline vendoring tool.

- Reference YAM and xArm7 workspaces for MuJoCo, Isaac Sim, and SAPIEN, with shared
  scene/wrist RGB-D profiles, normalized grippers, two cubes, a threaded cap, and a drawer.
  Optional engines run in isolated workers behind the existing shared-world lifecycle.

- Add manifest-selected shared simulation worlds through the public
  `WorldConfig`/`SimulationBackend` contract. World-backed parts and cameras reuse the
  ordinary SDK runtime, support matrix, owner envelope, observations, and RGB-D sample
  path.
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

- Batch reference-world 2 ms physics substeps between state reports at twice the
  declared command rate, with a 100 Hz floor. Carry fractional substeps forward
  instead of advancing extra physics time when reporting intervals do not divide
  the native timestep.
- Use ManiSkill's native PD mimic-controller pattern for SAPIEN jaws, sharing the
  original gain, force and inertia budget across coupled drives. Retain the xArm
  passive linkage closures and unchanged contact settings.
- Add a bundled CC0 Poly Haven table texture and native material/key/fill lighting
  to MuJoCo and SAPIEN reference scenes, without changing geometry or RGB-D units.

- Base the reference simulation implementations on maintained 2026 I2RT,
  Menagerie, mjlab, ManiSkill and Isaac Lab sources. Use native URDF importers,
  xArm linkage closures and threaded-cap constraints; remove custom servo-target
  clipping, custom solver overrides and cap force callbacks. Reuse the SDK
  shared-world physics clock instead of running an independent worker clock. Derive the LINEAR_4310 pinch
  offset from the manufacturer's corrected grasp site without changing SDK FK.

- Preserve the existing `waddle_sdk.robots.mujoco:arm` factory as a compatible
  private-world adapter while documenting `waddle_sdk.robots.mujoco:backend` for
  multipart and RGB-D scenes.
- Require MuJoCo 3.5 or newer for the portable `MjSpec` scene compiler and renderer;
  the existing runtime remains lazy behind the `[mujoco]` extra.

### Fixed

- Prioritize queued simulation state/control requests ahead of camera captures,
  preserving one serialized native transaction. Real-time SDK pump ticks no
  longer send redundant clock requests; native reads, writes and captures
  already integrate elapsed time. This reduces camera-induced command latency.
- Decompose concave manufacturer arm and housing collision meshes through the
  existing CoACD build step, alongside fingers. Preserve physical recesses that
  a single convex hull filled, causing false YAM forearm/hand collisions during
  wrist motion. Convex source meshes and visual geometry remain unchanged.
- Build conservative planning bounds per physical link, independent of how its
  collision mesh is partitioned. This avoids redundant overlapping spheres and
  keeps native convex decomposition from multiplying planning work.
- Balance MuJoCo hand actuation with Menagerie's native fixed tendon, preserving
  the single motor's total force, gain and reflected inertia. This prevents the
  YAM jaws from diverging under handle contact and displacing the TCP.
- Use MuJoCo's documented elliptic friction cone and impedance ratio 10 across
  reference scenes to reduce slow contact creep. Native xArm handle-grasp
  retention is now checked alongside joint motion and RGB-D conformance.
- Apply Menagerie's published finger-pad contact response to the original
  manufacturer collision meshes. Prevent excessive cube penetration and YAM
  jaw oscillation; add a native grasp-width and settling regression.
- Give the MuJoCo reference cap native dry thread resistance and check released
  axial retention, instead of letting gravity back-drive a frictionless helix.
- Give the reference drawer explicit passive joint damping, preventing an
  undamped pull from coasting into its limit and bouncing back after release.
  Preserve the passive joint and normal gripper contacts across engine adapters.

- Advance reference physics by elapsed monotonic time before reading state or
  changing targets, so rendering delays do not silently slow declared joint
  velocities or replay new targets into the past. Retain fixed engine substeps
  and explicit rollout stepping through `worlds.cell.options.real_time: false`.
  Use ManiSkill's standard 100 Hz SAPIEN physics cadence; SDK command and camera
  rates remain independent. New scene control defaults come from the physical
  SDK embodiment declarations instead of a simulator-wide 50 Hz / 0.5 rad/s.

- Place the reference YAM at a tabletop working pose with its TCP near
  `(0.36, 0, 0.14)` m and the hand pitched 45 degrees down. The prior pose lay
  near the inner boundary for horizontal approaches. The model and owner limits
  are unchanged; physical adapters retain their measured configuration. Place
  the bottle and cabinet clear of the starting hand and check native startup
  contacts for both robot models in every reference scene.

- Resume robot-pump cadence after a scheduling delay instead of replaying missed
  ticks under a newly issued target. Catch-up bursts after planning could advance
  simulated arms past their target, causing owner-envelope refusals and timeouts.

- Skip missed camera capture slots after slow rendering/publication instead of
  issuing catch-up bursts. Those bursts starved shared simulation physics at full
  RGB-D resolution and caused ordinary SAPIEN joint/orientation motion to time out.
  Preserve native physics timesteps, owner limits, and every acquired frame's
  normal publication/recording path.

- Preserve site-selected robot-part, base-frame, and wrist-camera owner names
  in reference simulation worlds, allowing the same program scopes as physical
  hardware. Reject binding two logical parts to the single reference robot.

- Correct MuJoCo principal-point signs so RGB pixels and axial depth agree
  with the declared optical intrinsics for off-center cameras. Native engine
  acceptance now checks projected RGB geometry and depth with unequal focal
  lengths, an off-center principal point, and submillimetre depth units.

- Preserve reference simulation state across control runs by default. Starting
  a new run after jog release no longer teleports the robot or rearranges props
  underneath an already observed command. Shared-world initialization owns the
  home pose; per-arm episode hooks do not home again. Rollouts can explicitly
  select `worlds.cell.options.reset_on_episode: true` for native scene reset.
- Reject invalid portable-scene robot base frames and unsafe URDF joint-limit
  widening before loading the optional MuJoCo compiler dependency.
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
