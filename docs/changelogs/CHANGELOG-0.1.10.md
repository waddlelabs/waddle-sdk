# Changelog: 0.1.10

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

This file preserves the coordinated `0.1.10` release of the repository.

## [0.1.10] - 2026-08-29

### Fixed

- Restore the pinned I2RT starvation-safe SocketCAN receive path before a live
  YAM opens. It uses one kernel wait plus a final non-blocking drain so a healthy
  reply queued during a Python scheduling stall cannot poison the next motor
  transaction as a cascade of false timeouts; vendor signature drift fails closed.
- Open passive cameras before supervised robot drivers so USB enumeration and
  first-frame setup cannot starve an already-energized YAM control loop. A camera
  open failure now leaves every arm unopened.

### Added

- Added a reusable, fail-closed SocketCAN link helper. An opted-in robot adapter can
  activate only its exact declared interface at its exact bitrate before opening
  hardware; already-up mismatched links and non-CAN interfaces are refused without
  mutation, and missing privilege reports the exact bounded command to authorize.

### Changed

- The YAM adapter accepts `configure_can` plus `can_bitrate`. SDK callers remain
  opt-in, while workspace frontends can make the choice explicit in `site.yaml`.
- Updated both Python wheel projects and the exact media-companion pin to version 0.1.10.
