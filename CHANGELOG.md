# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Fixed

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
