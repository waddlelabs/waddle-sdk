# Changelog: 0.1.9

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

This file preserves the coordinated `0.1.9` release of the repository.

## [0.1.9] - 2026-08-28

### Added

- Added the explicit `waddle_sdk.cameras.inspect_cameras()` lifecycle for local
  multi-camera identification. Discovery and context construction remain non-opening;
  context entry opens cameras only, retains one latest immutable frame per camera, and
  closes drivers before bounded capture-thread joins.

### Changed

- Updated both Python wheel projects and the exact media-companion pin to version 0.1.9.
- Made the native `BINDING_API_VERSION` the strict media-companion compatibility
  contract. A package-version mismatch now warns but uses a compatible media core;
  only an actual binding-API mismatch disables the companion.
