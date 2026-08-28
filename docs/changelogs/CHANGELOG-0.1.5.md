# Changelog — 0.1.5

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

This file preserves the coordinated `0.1.5` release of the repository. It supersedes
the withdrawn, unpublished `0.1.4` release attempt.

## [0.1.5] - 2026-08-27

### Changed

- Rename the LiveKit packaging surface from `teleop` to `media`: customers install
  `waddle-sdk[media]`, which selects the exact-version `waddle-sdk-media` companion
  without implying motion authority. The old extra and companion are removed.
- Build and native-import the media companion on Linux x86-64/ARM64, macOS
  Intel/Apple Silicon, and Windows x64 before either SDK distribution may publish.
  macOS media wheels declare the ScreenCaptureKit-compatible 12.3+ floor; Windows
  ARM64 remains unadvertised until both SDK distributions have native coverage.
- Match Rust and C++ to LiveKit's static MSVC runtime in the Windows media wheel,
  preventing `/MT` and `/MD` objects from entering the same extension.
