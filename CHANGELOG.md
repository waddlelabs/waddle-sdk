# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Added

- Add a separate attended OpenArm single-motor commissioning owner, with explicit
  whole-bus support/clearance and cutoff inputs, silent watchdog qualification,
  bounded raw-coordinate out-and-back references, decoded MIT effort checks,
  fresh peer/status/temperature guards and independent disable on failure.
  Restore original RAM mode/TIMEOUT only after confirmed disable, retain original
  and cleanup errors, and keep the ordinary monitor adapter read-only. Include
  fake-wire cancellation/error/cleanup checks and an explicit opt-in live profile
  with complete wire evidence; physical qualification remains separately measured.
  Clear only the deliberately qualified watchdog fault, with fresh disabled
  confirmation; support separately requested, retained-evidence recovery without
  enable/MIT. Distinguish observed displacement, exact arrival and hardware faults.
- Add the monitor-only `waddle_sdk.robots.openarm:arm` adapter for seven arm
  encoder coordinates and a raw gripper-motor radian coordinate. Bind exact USB
  adapter serials and CAN IDs, query current encoding spans independently of
  explicit site limits, retain complete fresh feedback and scoped motor/bus
  failures, and fence duplicate local query owners. All lifecycle paths remain
  read-only; motion postures, physical stops, FK and jaw metres are unavailable.
  Include fake-wire and ordinary Site lifecycle checks; hardware motion is not
  qualified by this addition.
  Maintain a capability-gated live two-bus monitor case with guarded read-only
  wire capture, named receipt-freshness checks and ordinary close evidence.
- Support explicit passive-driver lifecycle facts (`actuation_supported`,
  `estop_supported`, `close_releases_torque`) without claiming simulation or a
  physical stop. Preserve legacy drivers; withhold unsupported actuation facts
  and global stop registration, report missing Site stop implementations, and
  bypass parking only when close explicitly cannot release torque.

### Fixed

- Fix Orbbec startup and alignment: skip incomplete startup output, cast aligned
  frames to a frameset, require paired RGB/depth and own captured pixels. Support
  exact serial selection, preserve active color intrinsics/distortion and metric
  depth scale, reject changes during streaming, and close partial starts.

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
