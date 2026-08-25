# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Changed

- SDK runtime faults now carry transport-safe structured cause chains. Public
  `SiteSession`/`Run` operations classify untyped driver and native failures with the
  failed operation, affected scope, and exception category while retaining the raw
  exception only as the local Python cause; calibration refusal events use the same
  concise schema instead of interpolating arbitrary vendor exception text. Hosted
  bootstrap refusals retain their validated fault code and HTTP status, while hardware
  discovery, safety-preset, and collision-provider warnings no longer expose arbitrary
  extension exception strings.

## Released changelogs

- [`0.1.1` — 2026-08-24](docs/changelogs/CHANGELOG-0.1.1.md)
- [`0.1.0` — 2026-08-23](docs/changelogs/CHANGELOG-0.1.0.md)
