# Changelog — 0.1.3

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

This file preserves the coordinated `0.1.3` release of the repository.

## [0.1.3] - 2026-08-27

### Added

- Add first-class site topology declarations for robot base frames and scene/wrist camera
  mounts, pass them through public adapter configs, and reject drivers or references that
  contradict the declared physical frames.
