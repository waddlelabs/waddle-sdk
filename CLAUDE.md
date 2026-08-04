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
    python/waddle/           # pure-Python surface: init/rollout/Control/agent +
                             #   descriptors; _native.py picks the compiled core
      robots/                # opt-in robot modules (NOT imported by `import
                             #   waddle`): base.py is the vendor-neutral half
                             #   (Driver protocol, SimDriver twin, the Arm
                             #   envelope seam, console recovery, RobotPump,
                             #   Rig); a vendor module is facts + driver +
                             #   factory on top of it
        yam.py, yam_data/    # the I2RT YAM: constants-with-provenance, and
                             #   the vendor's own MIT model (URDF text, no
                             #   meshes) shipped beside them so
                             #   tests/test_yam_facts.py gates every number
                             #   the model states — directional (a declared
                             #   limit may only be TIGHTER). The convention
                             #   the next vendor module inherits: ship the
                             #   source that can gate a fact, and where none
                             #   can (here the MJCF tightenings and both hand
                             #   facts) the comment names the pinned model it
                             #   came from — an unsourced number is one
                             #   nothing checks. Vendored data ships to every
                             #   installer, comments included: it may name
                             #   only what a wheel-holder can open (gated)
    teleop/                  # the `waddle-sdk-teleop` companion distribution:
                             #   same rust/Cargo.toml, + the livekit feature
    examples/                # toy_robot.py: the runnable customer program
                             #   (simulated 6-dof arm; offline, connected,
                             #   and agent modes) + its README
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
    `[tool.maturin].features`, never in Cargo.toml). Clippy must be clean in
    **three passes** — featureless, `--features grpc`, `--features
    grpc,livekit` — since no single pass compiles all the cfg'd code; the
    featureless pass is the baseline that must stay tokio- and
    libwebrtc-free, and is not what either wheel ships.
  - **Two distributions from one source tree** (the psycopg / psycopg-binary
    shape). Both are one build of `rust/Cargo.toml`, so they can never
    disagree on a version:
    - `waddle-sdk` (`sdk/pyproject.toml`, module `waddle._core`):
      `[tool.maturin] features = ["pyo3/extension-module", "grpc"]`. The gRPC
      control transport is in the DEFAULT build — a supervision layer whose
      default install cannot reach the supervision plane would be a strange
      thing to ship — and therefore in `uv sync` / `maturin develop` too, so
      the dev extension is connected, not offline.
    - `waddle-sdk-teleop` (`sdk/teleop/pyproject.toml`, module
      `waddle_teleop._core`): the same manifest with `livekit` added.
      Installed as the extra, never by name: `pip install
      'waddle-sdk[teleop]'`. It is separate because libwebrtc is ~690 MB of
      build that an install which only supervises a policy should not pay
      for; measured on this tree, 3.7 MB wheel / 9.8 MB `.so` against
      16.7 MB / 45.1 MB — ~4.5x.
    - `python/waddle/_native.py` is the ONE place that picks a core: the
      bundled one unless a version-matched `waddle_teleop._core` is installed
      and `WADDLE_NO_TELEOP != "1"` (a mismatch warns and falls back).
      `_native.FEATURES` (a frozenset re-exported from the SELECTED core) is
      the only feature detection the Python layer may do — never a
      try-import, and never `_core.FEATURES`, which on a `[teleop]` install
      describes the bundled core the process is not using.
    - **Release checklist**: `teleop = ["waddle-sdk-teleop==X"]` in
      `sdk/pyproject.toml` is the ONE version maturin cannot derive from the
      manifest (PEP 621 has no dynamic optional-dependencies), so a version
      bump must edit it — otherwise the extra resolves to the previous
      release and `_native` silently falls back to a core with no LiveKit.
      `tests/test_features.py` fails until the pin equals
      `waddle.__version__`. Build and publish the two wheels together.
  - Build the wheels with `uv build --wheel -o dist .` and `uv build --wheel
    -o dist teleop` (`dist/` is git-ignored). Both `[tool.maturin]` blocks
    carry `exclude = ["python/**/__pycache__/**"]`: `python-source` is the
    working tree, so without it a build after a test run ships that
    interpreter's bytecode and a build on a clean checkout does not.
  - A build without a feature REFUSES the matching `create_session` kwarg
    (`transport_url`/`transport_token`, `media_url`/`media_token`) rather
    than degrading to a silent offline session; the LiveKit refusal names the
    `[teleop]` extra. The shim grows kwargs, never logic. `grpc` adds a
    direct optional `waddle-controlplane` dep (runtime takes an
    `Arc<dyn ControlTransport>` and does not re-export `grpc::connect`).
    Build the companion's flavour in place with
    `uv run maturin develop --uv --features grpc,livekit`.
  - `sdk/rust` is deliberately its **own cargo workspace** with path-deps into
    `../waddle-core/crates/*` — do NOT add it to the waddle-core workspace
    (extension-module would make plain `cargo test` unlinkable there and drag
    a Python-interpreter probe into core CI). Cost: a second lockfile/target
    dir. Path deps always build the working-tree core, so no version skew.
    That second lockfile is why `sdk/rust/Cargo.lock` pins the livekit
    crates to the set `waddle-core/Cargo.lock` resolves (livekit 0.7.52,
    -api 0.5.5, -protocol 0.7.10, -common 0.1.0, -data-stream 0.1.0,
    -datatrack 0.1.11): the newest published set does not compile
    (livekit-api 0.5.6 against livekit-protocol 0.7.12). Pin in BOTH locks,
    or the shim's `livekit` feature breaks while core's stays green.
  - The built extensions (`python/waddle/_core*.so`,
    `teleop/python/waddle_teleop/_core*.so`) and `dist/` are build artifacts,
    never checked in.

## Load-bearing invariants (violating these is a bug, not a style choice)

- **Hollow-frontend rule.** All claim/lease/handoff/timeline logic lives in
  `waddle-core` exactly once (`waddle-fsm` is the behavioral conformance target). If a
  binding or frontend grows an `if` about claims, leases, handoffs, or timelines, that
  is a defect. The Python-specific review checklist lives in `sdk/README.md`.
  `python/waddle/robots/` is owner-side code that ships in the frontend, and the same
  rule binds it: it enforces the OWNER's envelope (limits arithmetic on the owner's own
  numbers, refusing whole and never clamping) and asks nothing about who may command
  what. The part an action addresses is the core's answer — indexed, never validated.
  A posture (`monitor`/`supervised`) maps to which `Control` verbs are registered and
  to nothing else.
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
  existing golden IS a breaking change. New FSM/gate behavior requires (a) normative
  text in `docs/FSM.md` — a guard-table row when a transition or guard moves, the
  governing section's prose when none does (intake validation, dispatch shape, and the
  blend/gripper/part contracts of §4-§5 are prose, not rows) — (b) at least one
  asserting scenario in `fixtures/behaviors/`, (c) green `waddle-conformance` run.
  (a) and (b) are enforced against each other:
  `every_behavior_fixture_is_named_in_fsm_md` fails until FSM.md names the fixture,
  so a scenario can never pin a behavior the standard does not claim.
  The scenario JSON schema is `waddle-protocol/conformance/scenario-format.md`;
  `waddle-conformance` implements exactly that schema — if they drift, the .md wins
  and the runner is wrong. A scenario's `requires_features` is the NEGOTIATION the
  runner models, not only a skip filter: where a flag changes how a message is read
  the runner reads it there and nowhere else, never inferring it from the robot, so
  a registry row's pre-flag behavior is an expressible scenario. Fixture directories
  are enumerated at test time, never listed by hand (`behaviors/` in
  `tests/behaviors.rs`, `wire/` in `tests/wire_fixtures.rs`, `sidecars/` in
  waddle-sidecar's `tests/fixtures.rs`).
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
  to that path. This holds on **every plan arm**, not just PASSTHROUGH — a
  supervised session spends whole windows CLAIMED/BYPASS on the same real-time
  thread, and `tests/alloc_free.rs` proves all of them. In particular the gate
  clones the active `ProvenanceTag` twice per tick, so **nothing owned may live on
  that tag**: its variable-length fields (`Provenance::Custom`'s name, the
  `ActorRef`) are `Arc`-shared and minted once per claim, off that thread. Adding an
  owned `String` to it costs a malloc pair per field per tick and the featureless
  `alloc_free` proof is what catches it. The same rule binds the action the gate
  dispatches: a claimed tick clones `OwnedAction` twice (record ring, blend anchor;
  three times when it blends), so its part tag is an `Arc<str>` minted once per wire
  action at the intake — nothing owned belongs on that struct either.
- **The control plane carries no bandwidth.** Media rides the media plane; the local
  recorder keeps the full-rate archive. There is exactly ONE declared exception —
  `FrameStill` observations behind `waddle.v0.obs.stills`, bounded by the camera's
  declared `StreamPolicy.still_fps` — and it is not a precedent: anything else
  high-bandwidth needs its own flag and its own bound, or it doesn't ride these RPCs.
  A flag that MULTIPLIES an existing low-rate send answers the same rule and must
  say so in its registry row: `waddle.v0.parts` makes the `StreamObservations`
  proprio cadence per part, so a flagged connection carries the declared part count
  (plus the sole part) times the unflagged rate — bounded by the declaration, which
  is fixed for the session and visible to the plane before it accepts.
  Messages that may be shed answer `ClientMsg::is_droppable` in waddle-controlplane
  (the ONE place that classifies), and every point a droppable message can queue —
  offline buffer and in-flight transport alike — must honor it; history is never
  shed.
- **A negotiated flag belongs to one connection.** Feature flags are re-negotiated
  at every Register and the client re-registers on every reconnect, so acceptance
  never outlives the connection that gave it (VERSIONING §3). Two halves, both
  required: producers read the current answer off `Status.*_negotiated`, which the
  plane pump clears at every connection boundary; and
  `ClientMsg::connection_scoped_flag` (waddle-controlplane, the ONE place that
  classifies this) keeps a flag-scoped message off any connection that did not
  accept its flag — filtered on the way out, and never buffered offline, since the
  offline buffer replays onto the NEXT connection before it has said what it
  accepts. Withholding such a message is not shedding history: the local recorder
  keeps the full-rate archive.

## Working conventions

- Commit style: small, coherent commits; imperative subject; body explains why. End
  commit messages with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- Prefer editing existing structures over adding parallel ones; no one-off fixes where
  an abstraction exists (or should).
- Tests are the spec: FSM invariants are proptests, behavior is fixture scenarios;
  when fixing a bug, first add the failing scenario/property, then fix.
- No test may depend on winning a wall-clock race. A window a test needs open (the
  plane unreachable, a claim un-engaged) is held open explicitly — e.g.
  `InMemoryTransport::refuse_connections` until the test heals the partition — and
  closed on an observable happens-before, never on a sleep or a backoff step chosen
  to be "long enough". A test that fails under load fails a gate this repo requires
  clean before every commit, and its assertions usually cannot tell the racing
  outcome from the intended one anyway.
- When you finish a work session, verify: workspace tests green, clippy/fmt clean,
  CHANGELOG.md updated, this file still accurate.
