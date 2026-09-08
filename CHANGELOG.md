# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Added

- Reference YAM and xArm7 workspaces for MuJoCo, Isaac Sim, and SAPIEN, with shared
  scene/wrist RGB-D profiles, normalized grippers, two cubes, a threaded cap, and a drawer.
  Optional engines run in isolated workers behind the existing shared-world lifecycle.

- Add manifest-selected shared simulation worlds through the public
  `WorldConfig`/`SimulationBackend` contract. World-backed parts and cameras reuse the
  ordinary SDK runtime, support matrix, owner envelope, observations, and RGB-D sample
  path consumed by Metal.
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

- Preserve the existing `waddle_sdk.robots.mujoco:arm` factory as a compatible
  private-world adapter while documenting `waddle_sdk.robots.mujoco:backend` for
  multipart and RGB-D scenes.
- Require MuJoCo 3.5 or newer for the portable `MjSpec` scene compiler and renderer;
  the existing runtime remains lazy behind the `[mujoco]` extra.

### Fixed

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
