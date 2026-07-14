# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Added
- **waddle-protocol v0**: the six schemas (`descriptors`, `control`,
  `episode`, `sidecar`, `services`, `media` under `proto/waddle/v0/`) with
  amendments N1–N18 applied; normative docs (GLOSSARY.md, FSM.md with
  transition-guard tables, VERSIONING.md); conformance tier docs and the
  normative behavioral-scenario format (`conformance/scenario-format.md`);
  buf configs. Design doc archived unchanged at `docs/rationale/`.
- **waddle-core M0–M1**: cargo workspace (11 crates + conformance runner +
  xtask; edition 2024; clippy `disallowed-methods` enforcing the clock
  discipline) and `waddle-types` — protox+prost build (no system protoc),
  embedded FileDescriptorSet, and the validated domain layer: `Stamp`
  dual-clock type, opaque ids, action-space validation (must-declare
  rotation/delta conventions, composite depth pin, frame tagging), wire→flat
  action flattening (declaration-order composites, wxyz quaternions),
  grants/handoff/provenance/outcome domain enums.
- Monorepo bootstrap: git repository, Apache-2.0 license, agent bootstrap
  (`CLAUDE.md`), changelog discipline (this file + `docs/changelogs/`).

## Stowed changelogs

_None yet. On first release, the released section moves to
`docs/changelogs/CHANGELOG-<artifact>-<version>.md` and is linked here._
