# Rust API reference

This page is the workspace landing for generated crate and symbol documentation. Read
the [architecture overview](architecture.md) first: Rust API pages describe
implementation surfaces, while the protocol documents remain normative for behavior.

- [`waddle-types`](../reference/rust/waddle_types/index.html): wire and validated domain types
- [`waddle-fsm`](../reference/rust/waddle_fsm/index.html): episode, claim, and lease machines
- [`waddle-gate`](../reference/rust/waddle_gate/index.html): synchronous action gate
- [`waddle-tripwire`](../reference/rust/waddle_tripwire/index.html): local watchdog evaluation
- [`waddle-ingest`](../reference/rust/waddle_ingest/index.html): clocks and bounded sample ingestion
- [`waddle-codecs`](../reference/rust/waddle_codecs/index.html): dialect and wire codecs
- [`waddle-sidecar`](../reference/rust/waddle_sidecar/index.html): semantic and MCAP recording
- [`waddle-media`](../reference/rust/waddle_media/index.html): media transport seam
- [`waddle-controlplane`](../reference/rust/waddle_controlplane/index.html): control-plane client and transport seam
- [`waddle-runtime`](../reference/rust/waddle_runtime/index.html): composition root
- [`waddle-conformance`](../reference/rust/waddle_conformance/index.html): normative scenario runner
- [`waddle`](../reference/rust/waddle/index.html): unstable `libwaddle` C ABI

The hosted documentation build generates this reference from the exact source revision
with:

```console
RUSTDOCFLAGS="-Dwarnings" cargo doc --workspace --exclude xtask --no-deps --locked
```

Dependencies are excluded so the reference remains focused on Waddle's public crates.
