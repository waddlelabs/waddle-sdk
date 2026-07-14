# CLAUDE.md — waddle-sdk monorepo

This file bootstraps every agent that works in this repository. Read it fully before
touching anything.

## Standing obligations (non-negotiable, for every agent, every session)

1. **Keep this file up to date.** `waddle-sdk/CLAUDE.md` must always reflect the actual
   state of the repo: layout, build commands, conventions, and gotchas. If your change
   makes any statement in this file stale, updating this file is part of that change —
   not a follow-up. If you discover this file is already stale, fix it before you build
   on top of the stale assumption.
2. **Maintain `waddle-sdk/CHANGELOG.md`.** A comprehensive, human-readable changelog
   lives at the repo root (Keep-a-Changelog style: `[Unreleased]` on top, newest release
   first; entries grouped Added/Changed/Fixed/Removed). Every substantive change lands
   with a changelog entry in the same commit series. When a version is released
   (tagged), **stow the finished changelog**: copy the released content to
   `docs/changelogs/CHANGELOG-<version>.md`, then reset root `CHANGELOG.md` to carry
   `[Unreleased]` plus a pointer list to the stowed files. Never delete changelog
   history; it only moves into `docs/changelogs/`.
3. **Git lives here.** The repository root is `waddle-sdk/`. Never `git init` or write
   artifacts in the parent directory (`api-dev/`). Scratch work goes in your session
   scratchpad, not the repo.

## What this repo is

The open half of Waddle — a supervision layer for real-world robot policy rollouts
(watch / intervene / reset / judge / improve). The normative design rationale is
`waddle-protocol/docs/rationale/waddle_api_design_doc.md` (v0.9). Amendments **N1–N18
are applied** in protocol v0; N19 (full spec consolidation) is deliberately deferred —
the fresh normative docs are `waddle-protocol/docs/{GLOSSARY,FSM,VERSIONING}.md`, and
they win over the rationale doc wherever the two diverge.

The closed side (control plane, teleop network, judges, relay) lives in a separate
internal repo (`waddle`, the "cell" codebase) and is NOT here. Nothing in this repo may
depend on it.

## Repo map

```
waddle-sdk/
  CLAUDE.md, CHANGELOG.md, README.md, LICENSE (Apache-2.0), .gitignore
  docs/changelogs/           # stowed changelogs of released versions
  waddle-protocol/           # THE STANDARD: schemas + fixtures + normative docs
    proto/waddle/v0/         # descriptors, control, episode, sidecar, services, media
    fixtures/                # wire/ sidecars/ behaviors/ (JSON, semantic-compared)
    conformance/             # scenario-format.md (normative), tiers, timing envelopes
    docs/                    # GLOSSARY.md FSM.md VERSIONING.md + rationale/
  waddle-core/               # Rust workspace: the reference implementation
    crates/waddle-{types,fsm,gate,tripwire,ingest,media,controlplane,
                   sidecar,codecs,runtime,ffi,conformance}
    xtask/                   # cbindgen header gen etc. (publish = false)
```

Future artifacts (Python `waddle-sdk` frontend via PyO3/maturin, `waddle-proxy`,
`waddle-cpp`, `waddle_ros`) will live in new top-level dirs; they are not built yet.

## Build & test

- Toolchain: Rust (see `waddle-core/rust-toolchain.toml`). **No system `protoc` or
  `buf` is required or assumed** — proto compilation happens in
  `crates/waddle-types/build.rs` via `protox` (pure Rust). Generated code is never
  checked in, in either repo.
- Everything Rust runs from `waddle-core/`:
  - `cargo build --workspace` / `cargo test --workspace` (includes the
    conformance suite: `cargo test -p waddle-conformance -- --nocapture`
    prints per-scenario PASS/FAIL)
  - `cargo clippy --workspace --all-targets -- -D warnings` and `cargo fmt --check`
    must be clean before any commit.
  - Feature-gated stubs: `cargo check -p waddle-runtime --features grpc,livekit`
    must keep compiling (both are typed stubs until those integrations land).
  - `cargo run -p xtask -- gen-header` emits the libwaddle C header to
    `target/include/waddle.h` (a CI artifact, never checked in).
  - `cargo bench -p waddle-gate` tracks the gate fast path (sub-µs target;
    an alloc-free proof also runs as a normal test).
- Quick proto syntax check without cargo:
  `uv run --with grpcio-tools python -m grpc_tools.protoc --descriptor_set_out=/dev/null -Iwaddle-protocol/proto waddle-protocol/proto/waddle/v0/*.proto`

## Load-bearing invariants (violating these is a bug, not a style choice)

- **Hollow-frontend rule.** All claim/lease/handoff/timeline logic lives in
  `waddle-core` exactly once (`waddle-fsm` is the behavioral conformance target). If a
  binding or frontend grows an `if` about claims, leases, handoffs, or timelines, that
  is a defect.
- **Vocabulary discipline.** `waddle-protocol/docs/GLOSSARY.md` is normative for every
  word in code, comments, and docs: grant (permission), claim (orchestration), lease
  (actuation single-writer), envelope (owner's hard safety — Waddle never provides
  it), tripwire (Waddle-side, requests holds via declared verbs), capability (robot
  skills ONLY), feature flag (protocol evolution). Reserved words never used in public
  artifacts: **"bridge"**, **"broker"** (internal process names), and **"agent"** as a
  component name. Say *teleoperator* (Waddle work-plane human) or *site operator*
  (customer-side human); unqualified "operator" is banned in normative text (N17).
- **Two-clock discipline.** Stream timestamps are session-monotonic ns; wall-clock
  location comes from a `ClockAnchor` captured atomically; epoch twins are captured at
  stamp time, never derived later. In Rust this is the `Stamp` type — only
  `waddle_ingest::SessionClock` mints it in production code (clippy
  `disallowed-methods` enforces; `FakeClock` for tests). Do not weaken this: it exists
  because of a production data-corruption postmortem.
- **Proto evolution is append-only.** Never reuse or renumber a field or enum value;
  removed fields become `reserved` (number AND name). Times/durations are `int64`
  nanoseconds (`_ns`; wall twins `_unix_ns`; operator-clock `_client_ns`, never
  recorded as session time). Quaternions are **wxyz** on this wire. Breaking changes
  mean a new package (`waddle.v1`), never in-place edits — see
  `waddle-protocol/docs/VERSIONING.md`.
- **Conformance fixtures pin behavior.** Golden fixtures are append-only; changing an
  existing golden IS a breaking change. New FSM/gate behavior requires (a) a guard-table
  row in `docs/FSM.md`, (b) at least one asserting scenario in `fixtures/behaviors/`,
  (c) green `waddle-conformance` run. The scenario JSON schema is
  `waddle-protocol/conformance/scenario-format.md`; `waddle-conformance` implements
  exactly that schema — if they drift, the .md wins and the runner is wrong.
- **Crate layering.** `waddle-types`/`-fsm`/`-gate`/`-codecs` must stay free of tokio,
  threads, I/O, clocks, and randomness. Only `waddle-ingest` reads OS clocks. Threads
  are owned by `waddle-runtime` (plus the thread harnesses in waddle-tripwire and the
  client threads in waddle-controlplane, whose lifecycles runtime owns). There is
  deliberately **no async runtime anywhere yet** — everything is dedicated named
  threads + channels; tokio arrives only if/when the tonic (`grpc`) or LiveKit
  (`livekit`) integrations land and stays confined to those transports.
  `waddle-codecs` is independently versioned and may depend only on `waddle-types` +
  serde (N4).
- **The gate fast path is sacred.** `Gate::gate()` must remain synchronous, wait-free
  in passthrough, and allocation-free. Benchmarks in `waddle-gate` track this; don't
  add locks, syscalls, or allocations to that path.

## Working conventions

- Commit style: small, coherent commits; imperative subject; body explains why. End
  commit messages with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- Prefer editing existing structures over adding parallel ones; no one-off fixes where
  an abstraction exists (or should).
- Tests are the spec: FSM invariants are proptests, behavior is fixture scenarios;
  when fixing a bug, first add the failing scenario/property, then fix.
- When you finish a work session, verify: workspace tests green, clippy/fmt clean,
  CHANGELOG.md updated, this file still accurate.
