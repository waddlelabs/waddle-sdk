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
  .github/workflows/
    release.yml              # the ONLY workflow: tag v* (or dispatch) builds
                             #   both wheels and publishes them to PyPI
                             #   (Trusted Publishing, no secrets). There is
                             #   no test/CI workflow yet.
  docs/                      # repo-level customer-facing docs:
    RELEASING.md             #   the release checklist + the one-time PyPI
                             #   account setup (pending trusted publishers)
    lease-lifecycle.md       #   the session/lease lifecycle from the
                             #   customer's side (grant/claim/lease/envelope,
                             #   who holds the robot in every phase); cites
                             #   FSM.md rather than restating it
    hardware-backends.md     #   the customer porting contract: minimal
                             #   Robot/Rig/Driver/camera surface, optional
                             #   SDK FK/geometry facets, support/grant fallback,
                             #   and per-scope embodiment identity
    changelogs/              #   stowed changelogs of released versions
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
    pyproject.toml           # maturin backend; module waddle_sdk._core; uv-managed
    rust/                    # the shim: its OWN cargo workspace (see build notes)
    python/waddle_sdk/       # THE IMPORTABLE PACKAGE IS `waddle_sdk`, not
                             #   `waddle`: the closed backend owns that name.
                             #   `site.py` is the primary Site/SiteSession/Run
                             #   lifecycle; `runtime.py` owns the structural
                             #   SDK port/DTOs consumed by Metal, including an
                             #   immutable `waddle.sdk.support/v1` matrix and
                             #   independent optional support/FK/body-geometry
                             #   ports. SiteSession derives those facts from
                             #   opened Arm/camera implementations, publishes
                             #   the exact registered action space and grants,
                             #   gives the complete site and each robot/camera
                             #   scope separate stable public embodiment digests
                             #   (robot/composite identity includes each opened
                             #   part's declared base frame),
                             #   and never infers persistent depth support from
                             #   one transient RGB-D sample. It also carries
                             #   JointPositionCommand's optional known-trajectory
                             #   velocity feedforward; an open describe() adds
                             #   the canonical robot action descriptor so Metal can map
                             #   named parts; schemas/
                             #   carries the strict `waddle.site/v1` schema.
                             #   A part-level `gripper` record is driver-neutral
                             #   public metadata mapping physical jaw metres to
                             #   one declared action row; the open runtime's
                             #   action descriptor carries that row's actual
                             #   max velocity, not the arm-joint value, so Metal
                             #   can rate-bound gripper streams without knowing
                             #   the hardware vendor. Its optional complete
                             #   grasp-geometry fact set declares the TCP-frame
                             #   closing axis, pinch offset, and canonical
                             #   pointing-down wxyz orientation for generic
                             #   Metal skills; unlike `options`, none of this is
                             #   forwarded to the adapter factory.
                             #   Root exports exactly Site/SiteSession/Run, load_site,
                             #   Grpc/LiveKit, Outcome, manifest errors, and the
                             #   typed connector compatibility warning/refusal.
                             #   transport.py holds pure transport declarations;
                             #   _session.py is the private, non-global builder with
                             #   fixed hold-first/enforced core wiring. There is no
                             #   init/rollout/agent/shutdown or _testing module.
                             #   cli.py owns the `waddle-sdk connect` process;
                             #   it reads workspace identity from site.metadata.id
                             #   and combines that with customer/project provenance
                             #   resolved from WADDLE_API_KEY before hardware opens.
                             #   After the complete site opens it reuses the key once
                             #   against the hosted HTTP invitation endpoint and prints the
                             #   derived short-lived UI URL. The key itself never
                             #   enters a URL/browser, the request never retries,
                             #   and invitation failure leaves the site connected.
                             #   Driver-extension APIs stay in descriptors/,
                             #   robots/, and cameras/.
                             #   Hardware opens only in SiteSession.__enter__; a
                             #   bound Grpc connector first completes an
                             #   authorization-only waddle.v0 registration
                             #   and must negotiate connector.binding. Every RPC
                             #   carries its exact binding and a fresh connection
                             #   nonce; Register is a barrier before traffic. A
                             #   successful pre-deadline Register lets the SDK render
                             #   only allow-listed fields from bounded compatibility
                             #   JSON; upgrade_required remains a typed, hold-first
                             #   refusal and no customer software is changed remotely.
                             #   After runnable registration, the native runtime emits
                             #   500 ms v0 heartbeats; key revocation tears down
                             #   the connection and reaches the core-owned hold.
                             #   all commands cross the core gate and the
                             #   owner envelope. Raw RGB-D depth stays
                             #   process-local for calibration/perception,
                             #   while a deterministic colorized preview of
                             #   every captured depth plane rides the paired
                             #   `<camera>/depth` LiveKit track.
                             #   `waddle.v0.motion.feedforward` preserves a
                             #   same-shape optional joint-velocity hint through
                             #   remote intake, gate substitution/bypass, raw
                             #   recording and Python dispatch. `robots` exports
                             #   the structural PositionVelocityDriver protocol;
                             #   position-only drivers retain the same target.
                             #   Workspace bounds cover both the TCP and every
                             #   adapter-supplied conservative body sphere.
                             #   Static box/sphere keep-outs and named-body
                             #   self/cross-part collision checks are SDK-owned:
                             #   adapters supply conservative CollisionSphere
                             #   geometry in one declared frame; missing or
                             #   incompatible configured geometry fails closed.
                             #   `_native.py` picks the compiled core.
                             #   The SDK-local UI and hosted task/artifact
                             #   facades are deleted; those product surfaces
                             #   live in closed Waddle through Metal.
      cameras/               # structural CameraDriver plus immutable paired-
                             #   timestamp RGB/RGB-D samples; latest raw aligned
                             #   depth stays local for click deprojection and
                             #   metric perception. CameraPump publishes RGB
                             #   on the legacy camera track and a stable
                             #   colorized preview on `<camera>/depth`.
                             #   CameraCalibrationDriver is an OPTIONAL
                             #   extension: a live driver may report the
                             #   intrinsics of its active aligned stream, which
                             #   RigSession folds into the declaration before
                             #   core registration. Explicit site.yaml values
                             #   win; cameras without the extension are valid.
                             #   CameraFrame/CameraSample may also carry an
                             #   optional LOCAL-ONLY vendor point resolver;
                             #   RealSense uses librealsense for distorted
                             #   aligned pixels, while the generic pinhole
                             #   fallback refuses non-zero distortion.
                             #   Orbbec/RealSense adapters import vendor SDKs
                             #   lazily behind [orbbec]/[realsense]; USB does
                             #   the same for OpenCV behind [usb]. [cameras]
                             #   composes all three; mock is dependency-free.
                             #   RealSense owns one process-lifetime context,
                             #   proves frame flow, resets one wedged device
                             #   once, rebuilds after a later timeout, and
                             #   reports the active color-grid intrinsics plus
                             #   depth scale through that optional extension
                             #   and attaches its matching deprojection path.
                             #   Built-in vendor adapters normalize validated
                             #   whole-number YAML rates such as 30.0 to the
                             #   integer form their APIs require.
      discovery.py           # non-opening configuration evidence for CAN,
                             #   serial and camera devices; immutable rows,
                             #   isolated custom scanners via the
                             #   waddle_sdk.hardware_discovery entry-point
                             #   group. RealSense selectors use non-streaming
                             #   librealsense enumeration when its optional
                             #   package is present; raw sysfs USB serials are
                             #   non-executable evidence because they may not
                             #   be valid enable_device identities. Never
                             #   constructs a driver or guesses which robot is
                             #   attached to a generic bus.
      robots/                # opt-in driver-extension modules; Site resolves
                             #   their declared module:factory targets lazily.
                             #   base.py is the vendor-neutral Driver/SimDriver,
                             #   plus an optional PositionVelocityDriver
                             #   extension that degrades to Driver.write,
                             #   Arm envelope (including CollisionSphere-backed
                             #   body-workspace, keep-out and self-collision
                             #   arithmetic), recovery,
                             #   pump, Rig and RigSession
                             #   layer. SiteSession composes RigSession._open
                             #   with the internal native-session builder;
                             #   driver factory calls open no device. A half-open
                             #   rig closes everything it opened and context
                             #   exit finalizes recording before hardware close.
                             #   safety.py defines immutable, non-opening owner-
                             #   envelope presets discovered beside each driver
                             #   target. Initializers copy selected values into
                             #   site.yaml; preset choice never reaches runtime
                             #   or replaces explicit site review. Custom adapter
                             #   modules may publish the same optional
                             #   safety_presets(factory=, options=) extension.
                             #   The bar that keeps base.py vendor-neutral is a test:
                             #   tests/test_robots_base.py builds a whole toy
                             #   vendor module (facts + SimDriver + factory,
                             #   ~50 lines) and drives it end to end with
                             #   nothing vendor-specific in base to help it.
                             #   Those exact lines (between the two --8<--
                             #   markers) are also the template sdk/README.md
                             #   publishes, and the copy is held to them by a
                             #   test in the same file — edit the test, paste
                             #   what it prints. sdk/README.md is where
                             #   this subpackage is documented OUTWARD (the
                             #   layering, the envelope-ownership doctrine,
                             #   the posture table), and the root README carries the Site quickstart and
                             #   the I2RT install command
        mock.py              # dependency-free manifest-native simulated arm;
                             # strict configurable limits/rate/home, planar FK,
                             # conservative CollisionSphere geometry, a derived
                             # planar-reach safety preset, and
                             # Site/static-keepout tests.
        xarm.py              # UFactory xArm 6/7: lazy xarm-python-sdk,
                             #   position-mode joint + G2 action rows,
                             #   controller-native safety setup, monitor,
                             #   local e-stop/re-enable, and fake-vendor tests.
        alicia.py            # Synria Alicia-M: lazy alicia-m-sdk, one PV
                             #   frame for six joints + gripper, torque-off
                             #   monitor/e-stop, model-derived limits.
        alicia_d.py          # Synria Alicia-D: lazy alicia-d-sdk, blocking
                             #   calls behind Driver, nonblocking commands,
                             #   torque-off monitor/e-stop, model-derived
                             #   limits. Alicia extras require Python 3.11+.
        mujoco.py            # manifest-native joint-target simulation:
                             #   lazy [mujoco], confined MJCF path, explicit
                             #   scalar joint/actuator mapping, scratch-state
                             #   FK/body spheres, local e-stop
        yam.py, yam_data/    # the I2RT YAM: constants-with-provenance, and
                             #   the vendor's own MIT model (URDF text, no
                             #   meshes) shipped beside them so
                             #   tests/test_yam_facts.py gates every number
                             #   the model states — directional (a declared
                             #   limit may only be TIGHTER). That gate binds
                             #   what the WHEEL SHIPS; a rig states what its
                             #   own machine has via `joint_limits=` on the
                             #   factories (the owner's envelope, and the
                             #   same table the declaration carries), which
                             #   may widen and says so at every start. The convention
                             #   the next vendor module inherits: ship the
                             #   source that can gate a fact, and where none
                             #   can (here the MJCF tightenings and both hand
                             #   facts) the comment names the pinned model it
                             #   came from — an unsourced number is one
                             #   nothing checks. Vendored data ships to every
                             #   installer, comments included: it may name
                             #   only what a wheel-holder can open (gated).
                             #   Also the LiveDriver + the bimanual()/arm()
                             #   factories. Their declaration publishes separate
                             #   arm and gripper velocity rows matching the same
                             #   profile the owner envelope enforces. GOTCHA:
                             #   driving metal needs the
                             #   vendor package, which is NOT a dependency and
                             #   cannot be an extra (not on PyPI; direct refs
                             #   are rejected there) — it is the documented
                             #   `I2RT_INSTALL` command, BUILT from I2RT_PIN
                             #   so the two cannot drift (the root README
                             #   quotes it verbatim and a test holds that
                             #   copy to it), and imported lazily inside
                             #   LiveDriver.__init__ so importing the
                             #   module never needs it. `yam.declaration()` is
                             #   public and byte-equal to what the factories
                             #   register (golden test vs the rig's own
                             #   customer program). LiveDriver uses I2RT
                             #   command_joint_state for a caller's
                             #   known bounded arm velocity (never one derived
                             #   from measurements/IK), zeros the hand velocity,
                             #   and falls back to command_joint_pos when the
                             #   pinned vendor surface lacks that capability.
                             #   Per-unit gripper motor limits are an optional
                             #   override; when absent, hardware open delegates
                             #   to I2RT's jaw-moving auto-range. Simulation
                             #   needs no motor measurement.
                             #   LiveDriver also works
                             #   around the pinned vendor close race by joining
                             #   its unretained CAN writer before the vendor
                             #   closes python-can's SocketCAN descriptor.
                             #   It also publishes the SDK example's tabletop
                             #   workspace as a non-opening initializer preset;
                             #   mounting/table/tool clearance still requires
                             #   explicit site review.
    teleop/                  # the `waddle-sdk-teleop` companion distribution:
                             #   same rust/Cargo.toml, + the livekit feature
    examples/                # one strict simulated Site program:
                             #   site.yaml + run_site.py + README. The program is
                             #   subprocess-tested and exercises load/open/run/close.
    tests/                   # pytest: Site/runtime contracts, descriptors, native
                             #   transport/FSM behavior, camera lifecycle, owner
                             #   envelope, YAM facts/factories, lazy adapters,
                             #   packaging, and the shipped Site program. Legacy
                             #   module-global API tests were deleted at cutover.
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
    - `waddle-sdk` (`sdk/pyproject.toml`, module `waddle_sdk._core`):
      `[tool.maturin] features = ["pyo3/extension-module", "grpc"]`. The gRPC
      control transport is in the DEFAULT build — a supervision layer whose
      default install cannot reach the supervision plane would be a strange
      thing to ship — and therefore in `uv sync` / `maturin develop` too, so
      the dev extension is connected, not offline.
      Camera support is independently lazy: `[orbbec]` installs
      `pyorbbecsdk2`, `[realsense]` installs `pyrealsense2`, and
      `[usb]` installs OpenCV; `[cameras]` composes all three. Robot support is
      likewise lazy: `[xarm]`, `[alicia]`, and `[alicia-d]` install their
      vendor packages, while `[robots]` composes them. Alicia vendor packages
      require Python 3.11+, expressed as dependency markers without narrowing
      the base wheel's Python 3.10+ range; the explicit pre-release
      `synria-robocore` constraint is necessary for deterministic resolution.
      MuJoCo is separate behind `[mujoco]` and loads its MJCF only at Site.open().
      The base wheel imports no vendor SDK, and the extras compose with
      `[teleop]`. Metadata/install
      combinations are held by `tests/test_clean_installs.py`;
      hardware construction remains opt-in because importing an adapter never
      opens a device.
    - `waddle-sdk-teleop` (`sdk/teleop/pyproject.toml`, module
      `waddle_teleop._core`): the same manifest with `livekit` added.
      Installed as the extra, never by name: `pip install
      'waddle-sdk[teleop]'`. It is separate because libwebrtc is ~690 MB of
      build that an install which only supervises a policy should not pay
      for; measured on this tree, 3.7 MB wheel / 9.8 MB `.so` against
      16.7 MB / 45.1 MB — ~4.5x.
    - `python/waddle_sdk/_native.py` is the ONE place that picks a core: the
      bundled one unless a version- and `BINDING_API_VERSION`-matched
      `waddle_teleop._core` is installed
      and `WADDLE_NO_TELEOP != "1"` (a mismatch warns and falls back).
      A stale bundled core fails at import before hardware opens; a stale
      teleop core warns and falls back to the bundled core. `_native.FEATURES`
      (a frozenset re-exported from the SELECTED core) is
      the only feature detection the Python layer may do — never a
      try-import, and never `_core.FEATURES`, which on a `[teleop]` install
      describes the bundled core the process is not using.
    - **Release checklist**: `teleop = ["waddle-sdk-teleop==X"]` in
      `sdk/pyproject.toml` is the ONE version maturin cannot derive from the
      manifest (PEP 621 has no dynamic optional-dependencies), so a version
      bump must edit it — otherwise the extra resolves to the previous
      release and `_native` silently falls back to a core with no LiveKit.
      `tests/test_features.py` fails until the pin equals
      `waddle_sdk.__version__`. Build and publish the two wheels together — in CI,
      not by hand: see **Release** below and `docs/RELEASING.md`.
  - Build the wheels with `uv build --wheel -o dist .` and `uv build --wheel
    -o dist teleop` (`dist/` is git-ignored). Both `[tool.maturin]` blocks
    carry `exclude = ["python/**/__pycache__/**"]`: `python-source` is the
    working tree, so without it a build after a test run ships that
    interpreter's bytecode and a build on a clean checkout does not.
    Non-Python PACKAGE DATA under `python-source` (today
    `waddle_sdk/robots/yam_data/`: 16.1 KB of URDF text + licence + README, 5.6 KB
    of it once deflated into the wheel — sdk/README.md quotes that same 16 KB
    to a customer, so the two move together) ships with no pyproject edit at
    all, and the code reads it through `importlib.resources` so a wheel, an
    editable install and a checkout all work. Adding or moving such data is
    the one time to build a wheel and list it — that it landed, and that no
    bytecode or mesh came with it.
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
  - The built extensions (`python/waddle_sdk/_core*.so`,
    `teleop/python/waddle_teleop/_core*.so`) and `dist/` are build artifacts,
    never checked in.

## Release

`docs/RELEASING.md` is the checklist (including the one-time PyPI account setup);
`.github/workflows/release.yml` is the pipeline, and the only workflow this repo has —
there is no test/CI workflow yet, so the local gates above are still the gate.

- **The trigger is a `v*` tag** (`workflow_dispatch` is the manual escape hatch).
  Pushing `main` builds nothing.
- **Wheels only, never an sdist** — for either project. Both `[tool.maturin]
  manifest-path`s point at `sdk/rust/Cargo.toml`, whose path deps into
  `../../waddle-core/crates/*` escape both pyproject directories, so an sdist would be
  an archive nobody can build. pyo3's `abi3-py310` makes that one wheel per platform,
  good for 3.10+ (free-threaded interpreters are not abi3 and are not built).
- **What each matrix leg covers**: `waddle-sdk` on linux x86_64 + aarch64 (manylinux
  containers, both native so each leg imports the wheel it just built), macOS arm64 +
  x86_64, Windows x64. `waddle-sdk-teleop` on **linux x86_64 alone** until libwebrtc's
  wheel is audited elsewhere — so `pip install 'waddle-sdk[teleop]'` resolves there and
  nowhere else, and that is the honest thing to say in release notes. Every wheel is
  installed and imported before it is uploaded, asserting its `FEATURES`.
- **No leg may be given `continue-on-error`.** A failing default leg blocks
  `publish-sdk`, which needs the whole matrix. A failing teleop build blocks
  `publish-teleop` only — the split below means the release then ships default-only on
  its own, which must be said in the release notes (add `teleop-wheel` to
  `publish-sdk`'s `needs` if a release should be all-or-nothing instead).
- **Publishing is Trusted Publishing (OIDC)**, no token or secret in this repo — and it
  is **two jobs, two GitHub environments**: `publish-sdk` → `pypi`, `publish-teleop` →
  `pypi-teleop`. PyPI keys a pending trusted publisher on (owner, repo, workflow,
  environment) and refuses that tuple twice, so the two projects cannot share one; a job
  carries exactly one environment, hence one job each, over artifacts named
  `sdk-wheels-*` / `teleop-wheels-*` so neither job can upload the other's wheel.
  A version bump edits **two** files (`sdk/rust/Cargo.toml` and the teleop pin in
  `sdk/pyproject.toml`); each publish job re-checks its wheels against the tag before
  anything reaches PyPI.

## Load-bearing invariants (violating these is a bug, not a style choice)

- **Hollow-frontend rule.** All claim/lease/handoff/timeline logic lives in
  `waddle-core` exactly once (`waddle-fsm` is the behavioral conformance target). If a
  binding or frontend grows an `if` about claims, leases, handoffs, or timelines, that
  is a defect. The Python-specific review checklist lives in `sdk/README.md`.
  `python/waddle_sdk/robots/` is owner-side code that ships in the frontend, and the same
  rule binds it: it enforces the OWNER's envelope (limits arithmetic on the owner's own
  numbers, refusing whole and never clamping) and asks nothing about who may command
  what. The part an action addresses is the core's answer — indexed, never validated.
  A posture (`monitor`/`supervised`) maps to which `Control` verbs are registered, and
  in a vendor module to how the driver is CONSTRUCTED where the vendor has a compliant
  mode (`monitor` opens a YAM in zero gravity, and that driver then refuses to write —
  so "nothing may command it" is a property of the object, not of a flag somebody
  remembered to check). It maps to no authority decision, ever: who may command a
  robot, when, and under what claim is waddle-core's and is identical under both.
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
  A composite SDK `Observation` takes its outer `Stamp` after snapshotting parts
  and camera samples, so a concurrently published camera frame cannot appear to
  come from the observation envelope's future.
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
  offline buffer would otherwise replay onto the NEXT connection under a different
  answer. Register is a barrier and history replays only after its response; scoped
  messages still never cross the boundary because they belong to the old answer.
  Withholding such a message is not shedding history: the local recorder
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
