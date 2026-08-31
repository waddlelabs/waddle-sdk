# Changelog: 0.1.11

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

This file preserves the coordinated `0.1.11` release of the repository.

## [0.1.11] - 2026-08-30

### Fixed

- Publish pinned-I2RT YAM position/velocity commands atomically under its server
  lock, preventing a transient all-zero PD command (gravity-only tick) during jog
  and planned motion while retaining I2RT gravity and friction compensation.

### Changed

- Updated both Python wheel projects and the exact media-companion pin to version 0.1.11.
