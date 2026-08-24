# Changelog — 0.1.1

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

This file preserves the coordinated `0.1.1` release of the repository.

## [0.1.1] - 2026-08-24

### Fixed

- Reuse each arm's sampled joint vector when deriving its end-effector pose for
  composite observations and proprioception reports, avoiding a second vendor
  observation RPC per arm and reducing the cold-start latency of hosted jog.
