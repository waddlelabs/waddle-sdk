# Changelog — 0.1.6

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

This file preserves the coordinated `0.1.6` release of the repository.

## [0.1.6] - 2026-08-28

### Added

- Added a strict, versionable MkDocs Material site for SDK concepts, protocol and
  Rust-core contracts, the Python frontend, and custom robot/camera porting, together
  with Read the Docs configuration, pull-request/version setup guidance, static
  Python API generation, warning-clean Rust API generation, and version-matched
  rendering of the normative glossary, FSM, and versioning rules.
- Added the portable `waddle-sdk-contracts` and `port-waddle-hardware` Agent Skills,
  including non-opening adapter scaffold/validation helpers, and bundled them as
  Python package data.
- Added `waddle-sdk skills list [--json]` and
  `waddle-sdk skills export <name> --output <directory>` with installed-version
  reporting and no-overwrite export semantics.
- Added CI gates for the hash-locked documentation build and real-wheel skill
  list/export verification, plus release-workflow support for a deterministic,
  checksummed skills archive gated alongside all wheels before either publisher.

### Changed

- Made documentation maintenance a standing agent obligation: behavior, public API,
  protocol, configuration, extension, test, workflow, and layout changes must review
  and update the affected docs, examples, generated-API inputs, and shipped skills in
  the same commit.
- Rewrote the hardware-porting and lease-lifecycle guides for the current
  `Site`/`SiteSession`/`Run`, `PartConfig`, `Rig`, structural driver, support-fact,
  and fixed hold-first contracts.
- Corrected Rust and generated-protobuf documentation links so the full core API can
  build with Rustdoc warnings denied.

### Fixed

- Removed internal vocabulary policy from the published normative glossary and
  eliminated its restricted internal process names from generated public Rust and
  protocol API documentation.
- Made Agent Skill metadata validation ignore ordinary non-skill package directories
  such as Python's generated `__pycache__`, matching the runtime skill discovery
  contract in clean Python 3.10 CI environments.
