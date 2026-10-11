# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

- Prepare matching SDK and media wheels as 0.1.13; the optional preview bindings
  require API 6, which was not included in the published 0.1.12 pair.

- Add optional LiveKit preview width, fps and per-track bitrate ceilings, RGB-only
  presentation and demand-driven publisher dynacast. Preserve original RGB-D
  samples and independent agent still rates; skip unused depth colorization.
- Bound pending video to the latest frame and separate its native worker from
  control-plane stills. Lower Linux media worker priority and bound publication
  waits. Demand-driven preview signaling starts/retries asynchronously, without
  delaying session startup. Python/native binding API advances to 6; rebuild both extensions.

### Changed

- Detach the native core after confirmed managed-rig teardown so native reset
  callbacks cannot retain closed rig/camera lifecycles. Uncertain cleanup keeps
  the native reference and site ownership intact.

- Retain phase and measured caller duration in USB submission deadline errors,
  preserving the original budget and terminal fence. Live startup reports retained
  vendor control faults before generic stopped-worker evidence.
- Refuse unsupported USB selection in the pristine raw-vendor live comparator
  before vendor imports or site ownership, preserving its SocketCAN comparison.

- Pin build toolchains to Rust 1.98.1 for the upstream Mach-O binary-alignment
  correction. Verify real media wheel imports and unavailable-service startup in
  CI. Preserve broken companion import faults on explicit media requests instead
  of reporting only that media was not compiled; optional absence stays usable.
- Set media wheel deployment floors in both Darwin targets' source configuration
  to macOS 13.0, covering ordinary builds and CI without an external release-only
  setting. Whole-release wheel tags cannot accurately represent ScreenCaptureKit's
  12.3 minimum; check the installed companion's actual wheel tags.

- Extend SDK/core CI to native arm64 macOS 15 while retaining Ubuntu. Exercise
  optional simulation/USB-CAN software behavior without activating live hardware.

### Added

- Add explicit exact-device USB-CAN selection to the shared YAM factories and
  Site connection mappings. Freeze device-free configuration, verify the reviewed
  runtime before imports/opening, reject conflicting or duplicate selections and
  retain candidate transport/source evidence. Preserve the vendor controller and
  existing SDK limits/supervision; physical qualification remains open.

- Add a source-verifying builder for an explicitly versioned I2RT transport
  runtime candidate. Accept its unchanged YAM model assets only after complete
  SDK-approved source/asset, installed-record and metadata/license verification.
  Preserve the pristine Git source path and physical qualification boundaries.

- Add a reviewed macOS arm64 RealSense packaging recipe and source-verifying
  wheel assembler. Retain original vendor sources, shared libusb source/notices,
  build identity and unqualified hardware status. Include native software-device
  pixel/progress and cleanup ownership checks. Correct upstream cleanup transfer
  assumptions under libc++ through an explicit source patch; retain the separate
  native alignment GIL patch. Camera privileges and live timing remain separate.

- Add an optional classic gs_usb transport candidate with exact-device selection,
  bounded I/O/queueing, nonblocking receive, terminal fault fencing and scoped
  cleanup. Include a source-pinned I2RT transport-injection patch and software
  qualification tests. Explicit YAM selection requires the reviewed derived
  runtime; hardware qualification remains separate.
  The vendor candidate also retains original control-thread faults, confirms
  first-iteration readiness, accepts a cadence-wait provider and drains before
  transport release. YAM honors retained faults and explicit bounded drains while
  preserving compatibility with the pristine vendor.

- Add a scoped periodic-thread scheduling request for macOS, restoring the prior
  Mach policy on normal/exceptional exit and leaving other platforms unchanged.
  Keep scheduling failures structured and preserve caller timing supervision.
  Robot/camera/inspection pumps request cadence only while waiting. Preserve
  shared publishing faults through site events/observations while individual arm
  failures retain their existing independent behavior.

- Discover macOS USB-CAN, serial and RealSense evidence through IORegistry without
  opening drivers. Preserve the distinction between raw USB serial evidence and
  a librealsense-resolved camera selector; Linux continues to use sysfs.

### Fixed

- Retain LiveKit/WebRTC Objective-C categories in final macOS targets and media
  wheels. Native simulation acceptance selects CGL on macOS and EGL on Linux,
  including isolated worker processes.

- Remove a timing race from malformed teleop acceptance: a FIFO packet barrier
  proves consumption, and both caller and bypass dispatches reject invalid widths
  without relying on nominal sleep duration to keep the caller healthy.
  Reset-progress acceptance similarly holds DONE on a transport-server channel
  until intermediate-state inspection completes.

- Count actual elapsed monotonic time during control-plane reconnect backoff.
  Late/coalesced wakeups no longer multiply nominal sleep slices. Share the
  owning session's ingest clock and preserve buffering, registration and authority
  fences; keep the existing reconnect delay values.

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
