# Stowed changelogs

When a version of any artifact in this monorepo is released, the finished section of
the root `CHANGELOG.md` is copied here as `CHANGELOG-<artifact>-<version>.md`
(e.g. `CHANGELOG-waddle-protocol-0.1.0.md`) and the root file is reset to
`[Unreleased]` plus a pointer list.

History is never deleted — it only moves into this directory.
