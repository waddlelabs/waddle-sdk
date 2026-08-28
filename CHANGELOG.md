# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Changed

- Rename the LiveKit packaging surface from `teleop` to `media`: customers install
  `waddle-sdk[media]`, which selects the exact-version `waddle-sdk-media` companion
  without implying motion authority. The old extra and companion are removed.
- Build and native-import the media companion on Linux x86-64/ARM64, macOS
  Intel/Apple Silicon, and Windows x64 before either SDK distribution may publish.
  macOS media wheels declare the ScreenCaptureKit-compatible 12.3+ floor; Windows
  ARM64 remains unadvertised until both SDK distributions have native coverage.

## Released changelogs

- [`0.1.3` — 2026-08-27](docs/changelogs/CHANGELOG-0.1.3.md)
- [`0.1.2` — 2026-08-25](docs/changelogs/CHANGELOG-0.1.2.md)
- [`0.1.1` — 2026-08-24](docs/changelogs/CHANGELOG-0.1.1.md)
- [`0.1.0` — 2026-08-23](docs/changelogs/CHANGELOG-0.1.0.md)
