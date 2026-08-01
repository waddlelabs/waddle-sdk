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
    conformance/             # scenario-format.md (normative), README.md (the
                             #   three tiers), timing-envelopes.md
    docs/                    # GLOSSARY.md FSM.md VERSIONING.md + rationale/
  waddle-core/               # Rust workspace: the reference implementation
    crates/waddle-{types,fsm,gate,tripwire,ingest,media,controlplane,
                   sidecar,codecs,runtime,ffi,conformance}
    xtask/                   # cbindgen header gen etc. (publish = false)
  sdk/                       # the Python `waddle-sdk` frontend (PyO3 + maturin)
    pyproject.toml           # maturin backend; module waddle._core; uv-managed
    rust/                    # the shim: its OWN cargo workspace (see build notes)
    python/waddle/           # pure-Python surface: init/rollout/Control + descriptors
    tests/                   # pytest: descriptors + e2e (incl. MCAP read-back)
```

Future artifacts (`waddle-proxy`, `waddle-cpp`, `waddle_ros`) will live in new
top-level dirs; they are not built yet.

## Build & test

- Toolchain: Rust (see `waddle-core/rust-toolchain.toml`). **No system `protoc` or
  `buf` is required or assumed** — proto compilation happens in
  `crates/waddle-types/build.rs` via `protox` (pure Rust); the gRPC service
  codegen in `crates/waddle-controlplane/build.rs` (`tonic-transport` feature
  only) is also protox-based. Generated code is never checked in, in either repo.
- Everything Rust runs from `waddle-core/`:
  - `cargo build --workspace` / `cargo test --workspace` (includes the
    conformance suite: `cargo test -p waddle-conformance -- --nocapture`
    prints per-scenario PASS/FAIL)
  - Must be clean before any commit (the featureless workspace run alone is
    NOT enough — it has provably missed feature-gated compile breaks):
    - `cargo clippy --workspace --all-targets -- -D warnings` and
      `cargo fmt --check`
    - the feature-gated test suites:
      `cargo test -p waddle-controlplane --features tonic-transport` and
      `cargo test -p waddle-media --features livekit`
    - a feature-enabled clippy pass:
      `cargo clippy -p waddle-runtime --features grpc,livekit --all-targets -- -D warnings`
  - Both transport features are REAL transports: `grpc` is the tonic
    `ControlTransport` (`waddle-controlplane/src/grpc.rs`, feature
    `tonic-transport`), `livekit` is the LiveKit media plane
    (`waddle-media/src/livekit.rs`). Build cost: the `webrtc-sys` build
    script downloads a prebuilt libwebrtc (~hundreds of MB compressed, ~690 MB
    extracted into the target dir) on the first build per target dir — network
    required on cold builds, ~30 s wall on a fast machine, warm re-checks are
    seconds; the tonic stack is ordinary crates.io deps (~1 min cold, no
    special downloads). Featureless builds are unaffected (no tokio, tonic,
    or livekit in the tree).
  - `cargo run -p xtask -- gen-header` emits the libwaddle C header to
    `target/include/waddle.h` (a CI artifact, never checked in).
  - `cargo bench -p waddle-gate` tracks the gate fast path (sub-µs target;
    an alloc-free proof also runs as a normal test).
- Quick proto syntax check without cargo:
  `uv run --with grpcio-tools python -m grpc_tools.protoc --descriptor_set_out=/dev/null -Iwaddle-protocol/proto waddle-protocol/proto/waddle/v0/*.proto`
- The Python SDK runs from `sdk/` (Python 3.10+, `uv` on PATH):
  - `uv sync --dev && uv run pytest` — full build (maturin backend into `.venv`)
    + the pytest suite. Iterate on the Rust shim with
    `uv run maturin develop --uv && uv run --no-sync pytest`.
  - `cargo clippy --manifest-path rust/Cargo.toml --all-targets -- -D warnings`
    and `cargo fmt --manifest-path rust/Cargo.toml --check` must be clean
    (this works because pyo3's `extension-module` feature lives only in
    `[tool.maturin].features`, never in Cargo.toml).
  - `sdk/rust` is deliberately its **own cargo workspace** with path-deps into
    `../waddle-core/crates/*` — do NOT add it to the waddle-core workspace
    (extension-module would make plain `cargo test` unlinkable there and drag
    a Python-interpreter probe into core CI). Cost: a second lockfile/target
    dir. Path deps always build the working-tree core, so no version skew.
  - The built extension (`python/waddle/_core*.so`) is a build artifact,
    never checked in.

## Load-bearing invariants (violating these is a bug, not a style choice)

- **Hollow-frontend rule.** All claim/lease/handoff/timeline logic lives in
  `waddle-core` exactly once (`waddle-fsm` is the behavioral conformance target). If a
  binding or frontend grows an `if` about claims, leases, handoffs, or timelines, that
  is a defect. The Python-specific review checklist lives in `sdk/README.md`.
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
  are owned by `waddle-runtime` (plus the thread harnesses in waddle-tripwire, the
  client threads in waddle-controlplane, whose lifecycles runtime owns, and
  waddle-media's LiveKit worker + data-tx forwarder threads behind the `livekit`
  feature). **Tokio is confined to transports**: it exists ONLY inside
  waddle-media's `livekit` feature (one dedicated `waddle-media-livekit` thread)
  and waddle-controlplane's `tonic-transport` feature (one dedicated
  `waddle-controlplane-grpc` thread per live connection, plus its
  `-tx` forwarder thread) — in both, the thread owns a private current-thread
  runtime, no tokio type appears in any public signature, and default builds
  are tokio-free (`cargo tree -p waddle-media` / `-p waddle-controlplane` must
  show no tokio without the features). Everything else stays dedicated named
  threads + channels. `waddle-codecs` is independently versioned and may depend
  only on `waddle-types` + serde (N4).
- **The gate fast path is sacred.** `Gate::gate()` must remain synchronous, wait-free
  in passthrough, and allocation-free up to 16 action dims and 32 obs dims (wider
  observations spill to the heap — a documented degradation, never truncation).
  Benchmarks in `waddle-gate` track this; don't add locks, syscalls, or allocations
  to that path.
- **The control plane carries no bandwidth.** Media rides the media plane; the local
  recorder keeps the full-rate archive. There is exactly ONE declared exception —
  `FrameStill` observations behind `waddle.v0.obs.stills`, bounded by the camera's
  declared `StreamPolicy.still_fps` — and it is not a precedent: anything else
  high-bandwidth needs its own flag and its own bound, or it doesn't ride these RPCs.
  Messages that may be shed answer `ClientMsg::is_droppable` in waddle-controlplane
  (the ONE place that classifies), and every point a droppable message can queue —
  offline buffer and in-flight transport alike — must honor it; history is never
  shed.

## Working conventions

- Commit style: small, coherent commits; imperative subject; body explains why. End
  commit messages with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- Prefer editing existing structures over adding parallel ones; no one-off fixes where
  an abstraction exists (or should).
- Tests are the spec: FSM invariants are proptests, behavior is fixture scenarios;
  when fixing a bug, first add the failing scenario/property, then fix.
- When you finish a work session, verify: workspace tests green, clippy/fmt clean,
  CHANGELOG.md updated, this file still accurate.
