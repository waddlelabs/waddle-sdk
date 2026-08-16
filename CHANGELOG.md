# Changelog

All notable changes to the waddle-sdk monorepo are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
artifact they describe (waddle-protocol and waddle-core version independently;
waddle-codecs versions independently of waddle-core per amendment N4).

Released changelogs are stowed in [`docs/changelogs/`](docs/changelogs/) when a version
ships; this root file always carries `[Unreleased]` plus pointers.

## [Unreleased]

### Fixed

- Made `RuntimeFault` a normal mutable exception so Python traceback and
  context-manager machinery can attach `__traceback__` while preserving its
  typed fault fields.
- Serialize the frozen top-level site manifest correctly from `Site.describe()` instead of passing `mappingproxy` to the JSON encoder.
- Accept append-only legacy `waddle.fixture/v0` wire and `waddle.behavior/v0` scenario envelope spellings in conformance readers while new goldens use the current `waddle_sdk.*/v0` spellings.

### Changed

- **BREAKING:** make `Site`, `SiteSession`, `Run`, `load_site`, transport selection, outcomes, and manifest errors the only root API. Site sessions use a private non-global builder with fixed hold-first, core-enforced safety wiring.
- Set the SDK and native shim release version to 0.1.0 and keep the teleop companion pin exact.
- **BREAKING (pre-release): the importable package is now `waddle_sdk`, not
  `waddle`.** `pip install waddle-sdk` then `import waddle_sdk`; the
  distribution name is unchanged. The extension module is `waddle_sdk._core`.

  WHY. The closed backend's own Python package is also named `waddle`, and the
  two must share one environment — `waddle` depends on `waddle-metal`, which
  imports this SDK (`from waddle_sdk.robots.base import Rig, CrossArm, ...`).
  Two distributions cannot both own a top-level `waddle/`. Co-installed, the
  result was not an error but something worse: `import waddle` resolved to
  whichever landed in `site-packages` while submodules still resolved through
  the other's editable finder — a silent HYBRID, masked whenever the cwd
  happened to be a repo root. It cost an evening to find.

  The SDK yields rather than the backend because the blast radius is 26 import
  sites against 481, and because nothing is released: this is `v0.0.0` with an
  empty `docs/changelogs/`, so no installed user is broken.

  What did NOT move, deliberately:
  - `waddle.execution.v1` did not move during the package rename because it was a string contract, not an import. The 0.1.0 Site cutover below subsequently removes that upward-discovery contract.
  - `waddle.v0` / a future `waddle.v1` — the wire protocol namespace.
  - MCAP topic names (`/waddle/observations`, `/waddle/actions`) and the
    `waddle.testing` source id. A recorded name that changes with a package
    rename splits one stream in two across the rename boundary.
  - `waddle-sdk-teleop` and its `waddle_teleop._core` module, already distinct.

### Removed

- Physically delete module-global `init`, `rollout`, `agent`, and
  `shutdown`, public `Control`/handoff/reset declarations, the private
  global-session test helper, their legacy-only tests, and the old
  `toy_robot.py`/`yam_bimanual.py` programs. A subprocess-tested simulated
  `site.yaml` example now exercises the sole Site lifecycle.
- Delete the SDK-local authenticated web UI, its bundled assets, and the hosted task/artifact/execution facades. Guided calibration and product task state now live exclusively in closed Waddle and Metal; SDK retains only local RGB-D measurement plumbing.

- Remove lease-enforcement and handoff choice from the primary Site API; both remain native implementation details.

### Added

- Open `SiteSession.describe()` responses now include the canonical robot action descriptor so Metal can map named part commands onto the complete SDK action vector using public data.
- Add the canonical `waddle-sdk connect` process and `waddle.v0.connector.binding` registration. An authorization-only probe authenticates the exact customer/project/workspace tuple and must negotiate the binding feature before SiteSession invokes any arm or camera builder; reconnect clears and re-establishes that authorization, while authorization probes cannot negotiate hosted runs. Registered runtimes emit 500 ms v0 heartbeats so API-key revocation becomes a transport partition and requests the existing core-owned hold/abort path.

- Add strict Draft 2020-12 `waddle.site/v1` loading with confined relative paths, named secret references, lazy driver construction, deterministic half-open cleanup, local RGB-D measurements, and an SDK-owned structural runtime port shared by local and remote Metal adapters.
- Add deterministic SDK-owned static hard safety: strict box/sphere keep-outs, named conservative `CollisionSphere` body geometry from driver adapters, within-arm and cross-part self-collision with explicit adjacent-body exclusions, shared-frame validation, and reject-whole/fail-closed dispatch before any driver write.
- Extract the dependency-free deterministic mock RGB-D camera and lazy OpenCV USB/UVC adapter into the SDK, including BGR-to-RGB conversion, half-open cleanup, idempotent close, fake-vendor tests, a `usb` extra, and USB composition in `cameras`.
- Add a dependency-free manifest-native mock/sim arm with configurable limits, rate, step caps and home, planar forward kinematics, conservative body geometry, and full Site/keep-out lifecycle coverage.
- Extract lazy manifest-native UFactory xArm 6/7, Synria Alicia-M, and Synria Alicia-D drivers with model-derived joint limits, unified joint/gripper actions, monitor posture, half-open cleanup, controller-local stop/re-enable behavior, controller-native xArm safety configuration, fake-vendor lifecycle tests, and isolated optional-dependency install coverage. Alicia extras are correctly limited to the vendor SDKs' Python 3.11+ support without narrowing the base SDK's Python 3.10+ range.
- Extract a lazy manifest-native MuJoCo joint-target driver with confined model paths, explicit joint/actuator mappings, compiled-model limit checks, deterministic stepping, local e-stop, and scratch-model FK/conservative body geometry for SDK hard-safety preflight. The isolated `mujoco` extra is covered by a fake-runtime install and lifecycle suite.
- Add `waddle.v0.hosted.runs` with append-only protobuf arms, connection-scoped negotiation, bounded session-lifetime request idempotency, ordinary core episode creation, structured admission status, timeout/disconnect HOLD+ABORT behavior, and no reconnect motion replay.
- Add whole-command owner-envelope preflight so a multi-part command cannot move an earlier part before a later part refuses.
- **managed rigs and lazy RGB-D cameras**: `waddle_sdk.init()` now accepts a
  mutually exclusive `rig=` form while preserving the existing
  `robot`/`control` API. The shared `RigSession` opens arms and cameras
  inside the lifecycle, starts reporting and latest-only capture pumps, closes
  every partially opened resource on failure, and lets `waddle_sdk.shutdown()`
  deterministically finalize the recording and hardware. A structural
  `CameraDriver` returns immutable RGB or pixel-aligned RGB-D
  `CameraFrame` values; the rig pairs each capture with one
  `Session.stamp()`, publishes RGB through the existing bounded paths, and
  retains depth locally for pinhole deprojection. Lazy adapters and extras are
  `[orbbec]`, `[realsense]`, and `[cameras]`; the base wheel stays free of
  vendor camera dependencies and every extra composes with `[teleop]`.
- **durable task, calibration, artifact, and execution facades**: three
  append-only connection-scoped protocol flags add named hosted-task sessions,
  locally resolved calibration measurements, and reviewed workspace-delivery
  metadata. Task history is durable across reconnects and plane restarts;
  clients page it explicitly with a session-global cursor and a bounded
  completion marker. Task events expose only bounded user/assistant text plus
  public-safe lifecycle; calibration sends a correlated 3-D measurement but
  no RGB/depth bytes; GateActions carries artifact metadata and a one-time
  reference but never archive bytes. The Python SDK provides typed facades and
  the authenticated UI adds named sessions, live history, interjection,
  interrupt, cameras, calibration, recordings, and Hosted/Local selection.
  That local-runtime discovery was subsequently removed by the Site cutover; Metal now consumes the SDK-owned `SdkRuntimePort` directly.
- **exclusive remote-to-local control handoff**: waddle-core now owns the
  operation that releases an active remote claim through normative E8, waits
  for lease handback and RUNNING/no-claim mirrors, and only then permits a
  normal local site-operator jog. The UI requires that handoff before every
  gesture; local commands still traverse the ordinary claim, action intake,
  deadman, and customer `Control.send` path, so a frontend never becomes a
  second authority and the owner's reject-whole envelope remains final.
- **`waddle_sdk.ui()` and remote invited-host chat**: an active SDK session can now
  open one dependency-free browser application on an OS-selected
  `127.0.0.1` port and receives a `UIHandle` with `url`, `close()` and context
  management. A per-run 256-bit fragment token, exact Host/Origin checks,
  custom JSON headers, no CORS, bounded bodies, no-store/CSP/referrer policy
  and automatic pre-core shutdown protect the loopback surface. Native core
  remains authoritative for rendered status, the priority local e-stop request,
  and site-operator jog/deadman claims (250 ms browser heartbeat, one-second
  expiry, injected-clock coverage); accepted jogs are ordinary one-step chunks
  through `Control.send`, leaving the owner's whole-command envelope final.
  The shim retains only latest proprio and one raw RGB frame per declared
  camera, while recording browsing resolves manifest-named sidecar/MCAP files
  beneath `recording_dir`. The additive `waddle.v0.agent.chat` flag adds bounded
  correlated ChatRequest/ChatEvent arms to `GateActions`; requests are
  connection-scoped, never buffered offline, and a bounded event ring exposes
  only accepted/text/done/unavailable/error lifecycle to the page. Local state,
  e-stop, jog, cameras and recordings are independent of chat availability.
  There is deliberately no `waddle ui` command.
- **generic episode task context and paired session stamps**: `waddle_sdk.rollout()`
  and `waddle_sdk.agent()` accept bounded string `task_metadata`, persisted in every
  sidecar and forwarded unchanged on `AgentInviteEvent.task_metadata` (append-only
  field 3) without participating in authority. `Session.stamp()` returns an
  immutable `SessionStamp(session_ns, unix_ns)` minted by the existing core
  session clock in one paired read, preserving the two-clock discipline for
  external evidence. Fresh conformance, Rust, and Python tests pin invite-wire
  propagation and metadata retention across outcomes, reset failure, and retake.
- **`joint_limits=` on the YAM factories and `yam.declaration()`**: the intervals a
  rig accepts are the OWNER's to state, and the shipped model's `JOINT_LIMITS` are
  the default rather than a ceiling. One table reaches both readers — the envelope
  `base.Arm` enforces and the `Joint.min_position`/`max_position` the declaration
  carries to the plane — so a teleoperator or a Waddle-hosted agent is shown the
  range the rig really has. Rows wider than the shipped model's are reported by
  name and by how far, at every start, so nobody inherits a widened envelope
  silently; malformed tables are refused by argument name, in sim as on metal.
  Found on live metal: a motor zeroed ~3 mrad off rests just outside a theoretical
  range whose lower limit is exactly 0, so a hold of its own measured pose was a
  command the envelope refused forever (1800 of 1800 in one warm-up rollout) — a
  correct refusal of a number that did not describe that machine. The directional
  fact gate is unchanged and still binds what the wheel SHIPS (a declared limit may
  only be tighter than the vendor's model); what a particular rig accepts is a
  different statement, made by its owner.
- **release pipeline (`.github/workflows/release.yml`, `docs/RELEASING.md`)**: the
  first CI this repo has — a tag of the form `v*` (or a manual dispatch) builds
  both distributions and publishes them to PyPI. `waddle-sdk` is built on five
  platforms (linux x86_64 + aarch64 in the manylinux containers `maturin-action`
  selects, both NATIVE so each leg can import what it just built; macOS arm64 and
  x86_64; Windows x64), one abi3 wheel per platform since pyo3 is built
  `abi3-py310`; `waddle-sdk-teleop` is built on linux x86_64 alone until the
  libwebrtc side of that build is audited elsewhere, and until then `pip install
  'waddle-sdk[teleop]'` resolves on that platform only — a loud install-time
  failure everywhere else, which beats a session that quietly has no media plane.
  Every wheel is imported before it is uploaded, and asserts the features the
  build was supposed to carry (`['grpc']` for the default,
  `['grpc', 'livekit']` for the companion) through the one feature-detection
  surface the Python layer may use. Publishing is PyPI Trusted Publishing —
  `id-token: write` and a GitHub environment, no token or secret in the repo —
  and that is also how both project names get claimed: a *pending* publisher per
  name, converted by the first successful run, with no placeholder upload. It is
  **two publish jobs over two environments** (`publish-sdk` → `pypi`,
  `publish-teleop` → `pypi-teleop`), because PyPI keys a pending publisher on
  (owner, repository, workflow, environment) and refuses to register that tuple
  for a second project; artifacts are named per distribution so neither job can
  upload the other's wheel. Each re-checks that every wheel it is about to
  publish carries the tag's version, since the `teleop` extra's pin in
  `sdk/pyproject.toml` is the one version maturin cannot derive from the
  manifest. No sdist is built, for either project: `[tool.maturin]
  manifest-path` points at `sdk/rust/Cargo.toml`, whose path deps into
  `../../waddle-core/crates/*` escape both pyproject directories, so an sdist
  would be an archive nobody can build. `docs/RELEASING.md` is the checklist
  around all of it — the two places a version bump must touch, the gates, the
  changelog stow, the tag, the one-time PyPI account setup with the exact
  pending-publisher field values, and what to do when a leg fails (never
  `continue-on-error`: a failing default leg blocks its publish, and a failing
  teleop build ships the release default-only, which the notes must say).
- **docs (`docs/lease-lifecycle.md`: the session and lease lifecycle, from the
  customer's point of view)**: the one story a customer previously had to
  assemble out of five places (the `waddle/__init__.py` docstrings, `FSM.md`,
  `sdk/README.md`, the toy example and the glossary), told once as prose: the
  lease is a single whole-robot single-writer right whose holder changes by
  phase, and the page follows it through a rollout (your loop holds it,
  `gate()` returns Pass), an intervention (the claimant holds it, per the
  declared `Handoff`, part-tagged on a `Composite`, driven straight to `send`
  when the loop goes quiet, and passed to a successor without a handback on
  retake), a remote reset window before and after the episode (gate mode RESET,
  no-op ticks, handback ordered before READY and before TERMINAL), an
  agent-invited episode, and the `monitor` posture that lends the lease to
  nobody. Cross-references `FSM.md`'s guard rows rather than restating them and
  the `sdk/README.md` posture table rather than duplicating it; linked from the
  root README's quickstart and from that posture section. Says what the layer
  does NOT provide, in the same breath as what it does: advisory enforcement is
  a discipline the caller keeps rather than a guard, since waddle-core's
  divergence detector is implemented and pinned by conformance but not yet fed
  by a live SDK session; the envelope and the physical stop are the owner's
  floor throughout; and a retake successor is opened under the surviving claim
  but handed to no Python caller, so the next `rollout()` raises while it is
  live.
- **docs (robot modules, where a customer actually looks)**: the root
  `README.md` gains the five-line YAM quickstart and the vendor-package
  install command it needs to drive metal — a documented `pip install` of a
  pinned git reference, since I2RT's package is deliberately neither a
  dependency of this SDK nor an extra of it, and quoted verbatim under a test
  (`yam.I2RT_INSTALL` is BUILT from the pin so command and facts cannot
  drift, and a README that writes it out by hand is that drift let back in
  through the one copy no import can reach) — and `sdk/README.md` gains a
  robot-modules section: the layering (every piece usable alone, with the
  hand-wired equivalence that is a test rather than a promise), the
  **envelope-ownership doctrine** (Waddle never provides the envelope; what
  ships is a parameterized default over the owner's own numbers that rejects
  and never clamps, and `send=` replaces it wholesale), the **posture table**
  (`monitor` registers the owner's stop alone and wires no media plane;
  `supervised` registers `send`/`hold`/`estop`; neither is an authority
  decision), the note that `robots/` is owner-side code the hollow-frontend
  checklist binds, and the **template for writing your own vendor module** —
  a facts table, a driver admitted on its ten members rather than its
  ancestry, and a factory, published as the very lines
  `tests/test_robots_base.py` drives end to end and held to that file's own
  source by a test, so the template cannot rot into a snippet nothing runs
  while still claiming to be tested.
  Publishing a customer-side robot module in the open is a deliberate product
  decision, and these READMEs are where it is now said out loud rather than
  only in a module docstring: what somebody needs in order to drive their own
  arm through their own envelope belongs in their hands, and the supervision
  side's in-cell material for keeping a fleet alive is a different artifact
  that stays where it is.
- **sdk (`examples/yam_bimanual.py`: the whole program, for a rig the SDK
  knows)**: two I2RT YAM arms supervised in five Waddle-facing lines — build
  the rig, open the session, ask Waddle to drive an episode — around a table
  of the site numbers that have no defaults (the workspace box, the
  bench-measured gripper motor radians, the cross-arm mounting, each arm's CAN
  interface), each carrying what re-measuring it is for. It is the counterpart
  to `toy_robot.py` rather than a replacement: that file writes the same
  session out by hand, and this one is what a robot module removes. Tested as
  the program it is (a subprocess with nothing configured): the factory still
  takes those arguments, and building a rig — live or sim — still opens no bus
  and starts no thread.
- **sdk (`waddle_sdk.robots.base`: the vendor-neutral half of a robot module)**: a
  new opt-in subpackage — `import waddle_sdk` is unchanged and imports none of it —
  carrying everything a program that drives a real robot writes around its
  vendor's driver, so a robot module is facts + driver + factory and nothing
  else.
  - **`Driver`** is a `typing.Protocol` (ten members: `kind`, `estopped`,
    `read`, `write`, `hold`, `estop`, `re_enable`, `step`, `home`, `close`), so
    a driver written by hand is admitted on its members, never its ancestry.
    `kind` is the DRIVER's own word, and the two questions this layer asks of
    it — does closing this drop all torque, is homing it a motion nobody is
    watching — both have an unsafe answer, so it is read in ONE direction
    (`drives_metal`): `sim` alone selects the harmless branch, and every other
    word, including one this layer has never seen, is treated as metal. A twin
    somebody called something else waits for a park gesture it never needed;
    the other direction would drop torque on real hardware with none of the
    warning.
    **`SimDriver`** is the rate-limited kinematic twin, parameterized by the
    owner's limits/step caps/rate: it walks at most one accepted command's
    worth of travel per control period, which is what makes a sim run a
    rehearsal of the live one rather than an easier version of it.
  - **`Arm`** is the envelope seam every command crosses — width, finiteness,
    declared joint limits, per-step travel against where the unit actually is,
    and (only with forward kinematics) the FK'd TCP inside a declared workspace
    box. It **rejects, never clamps**: a failing target is refused WHOLE, the
    unit holds, and one bounded line (`RejectLog`) names the check. An EMPTY
    value vector is the wire's "hold this part" and is honoured as such; a
    latched e-stop refuses everything else, so `accepted` keeps meaning what it
    says. Waddle never provides the envelope: this is a parameterized default
    built from the owner's own numbers, and `control(arms, send=...)` replaces
    it wholesale with the customer's own callable. The MEASURED side of the
    arithmetic is checked like the commanded one: a driver whose `read()` has
    drifted from the declared joint list is refused by name (and held, and
    counted) rather than broadcast into the step-cap comparison.
  - **Forward kinematics is opt-in and its absence is named, not filled in**: an
    arm built without `fk` reports joint positions only (`ee_pose()` answers
    `None` rather than inventing a frame), and a workspace box declared without
    one is refused at construction instead of silently checking nothing.
    `chain_fk`/`quaternion_wxyz`/`rpy_matrix` are the generic chain math for
    modules that do declare one (wxyz pinned, all four conversion branches
    tested).
  - **The e-stop latch and the one human path out of it**: `estop_all` (every
    part gets the call even if an earlier one raised), `close_all` (the same
    doctrine for dropping connections, with the opposite ending: a close that
    fails is REPORTED, never raised, because closing runs while something else
    is already unwinding and an exception here would replace the reason it is),
    `latched_parts`,
    `ParkGate`, `apply_console_gesture` / `start_console_recovery` — a `resume`
    typed at a foreground TTY, never the wire (`VERB_RESUME` releases a *hold*)
    and never the next episode's reset. There is ONE reader per terminal in a
    process, not one per session: stdin is a single stream and two readers of
    it deal alternate lines to each other, so `start_console_recovery` hands
    back a `ConsoleRecovery` — the arms it is aimed at — which a second
    session RE-AIMS and which its owner RETIRES. A reader left aimed at a
    finished session would answer half the words typed at the machine on arms
    nobody is driving, and on metal a `resume` there re-enables a driver whose
    bus is already closed. `scene_reset` is the default pre-reset hook: it
    refuses a latched scene on every backing, homes a twin, and vouches for
    anything it reads as metal without moving it.
  - **`RobotPump`** runs any tick callable at a declared rate on its own thread
    (`proprio_tick` is the usual one: step every part, report it per part),
    because the robot's own housekeeping cannot pause while the caller's thread
    is blocked inside `waddle_sdk.agent()`.
  - **`Rig`** is composition sugar and nothing more: a declaration, a way to
    open the arms behind it, and the rate they run at. Constructing one opens no
    bus and starts no thread — `arms()` is where hardware opens. `posture`
    (`"monitor"` | `"supervised"`) is the one construction-time choice and it
    maps to **verb presence only** (`monitor` registers the owner's stop alone,
    so the session declares that nothing may command this robot instead of
    accepting motion it intends to drop); it adds no authority logic, and
    agent-driven versus windowed stays a call-site choice. What a `monitor`
    session may be wired to follows from that and is documented where the
    posture is: with no `send` verb it may register no `hold` either, and it
    wires no media plane (the media plane carries the teleoperator's stream,
    so wiring one is an intervention path waddle-core refuses without a
    `send`) — watching rides `transport=` (proprioception + each camera's
    declared low-rate stills) and `recording_dir=`.
  - **`rig.session(project, ...)`** is that composition as a context manager
    (`RigSession`), and it exists for the two ends every hand-written version
    gets wrong at least once. `__enter__` opens the drivers — inside the
    `with`, so a bus that will not open unwinds structurally — registers the
    verbs, calls `waddle_sdk.init`, starts the console recovery and starts the
    reporting pump; a failure part-way through closes what it opened, since a
    context manager whose `__enter__` raises never gets an `__exit__`.
    `__exit__` retires the console reader, stops the pump, shuts the session
    down and closes the drivers whatever the body did, which **retires the
    shutdown footgun**: finalizing the recording is no longer a `finally:` the
    customer remembered to write (pinned by a test that raises mid-rollout and
    then reads the recording back). Every keyword that is not the rig's own
    goes straight through to `waddle_sdk.init` and means what it means there —
    including a customer's own `send`, which still REPLACES the shipped
    envelope through the sugar. The two that ARE the rig's own are documented
    on it: `send`, and `console=False` for a program whose stdin belongs to
    something else (a REPL, a supervising harness) and which therefore has no
    typed way to clear an e-stop latch.
  - **The pump is always on inside a session**, not only for an agent run: a
    program's own loop then only gates and applies, with no interleaved robot
    tick to forget, and a session whose caller is blocked inside
    `waddle_sdk.agent()` (or has no loop at all) keeps reporting. The bandwidth is
    the declared part-count multiplication of the proprio cadence, fixed by
    the declaration and visible to the plane before it accepts.
  - **`hold_until_parked`** (+ `ParkGate.wait` / `.wait_holding`): a finished
    mission on drivers this layer reads as metal keeps holding and keeps
    reporting until a human says the machine is parked, because closing stops
    the vendor's command re-send and the motors' own watchdog then drops ALL
    torque from wherever the mission left the arms. Every park warning this
    layer has is otherwise attached to a Ctrl-C the site operator TYPED —
    finishing normally had none, and that is the one ending nobody is standing
    ready for. WHICH ending it offers is decided by whether anything is
    reading the console (the `ConsoleRecovery` the session started), never by
    whether a terminal exists: a `console=False` session, a stdin that belongs
    to a harness, or a reader already at end-of-input all mean a typed gesture
    nothing would receive, and it says so and names the signal instead. A twin
    returns at once (nothing to sag, and a harness must be
    able to wait for a sim program to exit) and a Ctrl-C skips the hold (that
    operator is already at the machine).
  - **The composition is sugar, and a test says so**: `test_yam_session.py`
    wires `yam.declaration()`, drivers, `base.Arm`, `waddle_sdk.Control`, a plain
    `waddle_sdk.init`, the console recovery and a `RobotPump` by hand, and asserts
    the session that opens is byte-identical to `rig.session()`'s — the same
    registered robot JSON and the same everything else `create_session` is
    handed. Sugar that cannot be reproduced by hand is a wall.
  - **The second-vendor bar is a test**: `sdk/tests/test_robots_base.py` builds a
    toy vendor module (facts table + the shipped twin + one factory, ~50 lines),
    composes it by hand out of these pieces, and drives it end to end through a
    real session — declaration, envelope, gate, pump, MCAP read-back — with
    nothing vendor-specific in `base` to help it. It doubles as the template a
    customer copies: `sdk/README.md` publishes those exact lines, and a test
    holds the published copy to them.
  - No packaging change: no new dependency (numpy was already required), no
    extras key, no `pyproject.toml` edit — maturin sweeps the subpackage with
    `python-source`.
- **sdk (`waddle_sdk.robots.yam`: what an I2RT YAM is, in numbers that are
  checked)**: the facts half of the first vendor module — the six arm joints
  and their limits, the kinematic chain, the tool frame, the hand's stroke —
  plus the forward kinematics those facts describe. Shipping a customer-side
  robot module in the open is a deliberate product decision: what somebody
  needs in order to drive their own arm through their own envelope belongs in
  their hands, and the supervision side's in-cell material for keeping a fleet
  alive is a different artifact that stays where it is.
  - **Every number the shipped model states is gated against it.** The
    vendor's own model ships beside the module
    (`waddle/robots/yam_data/yam.urdf`, pinned at `I2RT_PIN`) and
    `sdk/tests/test_yam_facts.py` reads it with the stdlib XML
    parser: a declared position limit must sit INSIDE the model's interval, an
    effort ceiling must be `<=` it, and every fact that is not an interval —
    chain origins, rpys, axes, the tool frame — must match to a
    nanometre/nanoradian. Tightening a limit for your own rig is therefore
    always allowed; widening one past the hardware is what fails. The gate also
    walks the model's own chain through `base.chain_fk` and lands where
    `yam.forward_kinematics` does, so a transposition that survived an
    element-wise pass still moves the tool and still fails.
  - **What the gate cannot see, it names — and three facts it cannot.**
    The arm limits are the URDF ∧ MJCF intersection and the MJCF is not
    shipped, so those tightenings carry provenance comments; the one that is
    visible from here — `joint1`'s upper, 3.05433 against the URDF's 3.13 — is
    asserted, so the table cannot be quietly "corrected" to the looser model.
    Both hand facts (the normalized gripper row and the jaw stroke below) come
    from the pinned vendor tree rather than the model, which carries no finger
    geometry at all. Their tests pin the value and the arithmetic, which
    catches an edit made here and is not the same thing as a second source;
    re-vendoring against the pin is what catches a change made upstream.
    Naming which of the two a fact has is the convention, not the claim that
    every fact has the first.
  - **`GRIPPER_MAX_OPENING_M` is 0.095 m, re-derived, not copied.** The pinned
    vendor tree models this hand as two equality-coupled slide joints, each
    ranged `0 0.0475` along exactly opposed axes, so the jaw separation moves
    `2 × 0.0475` end to end — 1 mm short of the `gripper_stroke: 0.096` the
    same tree's own config declares, conservative rather than equal by luck.
    This retires the 0.075 m figure (`2 × 0.037524` from the MuJoCo Menagerie
    finger range), which was derived from a vendor commit one hardware
    revision behind the pin. Neither vendor file ships in the wheel, so the
    test pins the ARITHMETIC instead: the number cannot be edited without
    editing its derivation, and the derivation is what a re-vendor re-reads.
  - **`forward_kinematics` is public and opt-in** (an arm handed none reports
    joint positions and says so), and takes the six arm joints rather than the
    seven-row part vector — the seventh row is the gripper, and walking it into
    the chain would put the tool where nobody commanded. The refusal is
    structural.
  - **Third-party content, declared**: `yam_data/` is MIT vendor data inside an
    Apache-2.0 wheel, shipped with the licence verbatim and a README carrying
    the provenance, the pin and the patch list. Text only — the STL meshes are
    not shipped, which the README states so an unresolved `<mesh>` reference is
    not read as a broken file — and two comment repairs, both of which the
    README lists: a `--` inside an XML comment is illegal and made the file
    unparseable by strict parsers, the stdlib's among them; and the patch
    notes now point only at what a reader of this wheel can open, an internal
    task label and the path of the check that caught the wrong-axis tool frame
    having been dropped while the correction itself is still stated. A test
    refuses either class in any file of the shipped data, since a comment in
    vendored data is where such a pointer survives. No element of the model
    differs.
  - No packaging change here either, and this time it is checked: a built wheel
    carries `waddle/robots/yam_data/{yam.urdf,LICENSE,README.md}`, no meshes
    and no bytecode, and the module reads them through `importlib.resources`
    from the installed package rather than from a path beside its own file.
- **sdk (`waddle_sdk.robots.yam`: the live driver, and the rigs its factories
  build)**: the other two thirds of the first vendor module — what moves a
  real arm, and what assembles one into a session-ready rig out of the site's
  own numbers.
  - **`yam.LiveDriver`** is the thin honest layer over four vendor calls, and
    the LATCH they make necessary. The stop the vendor offers
    (`zero_torque_mode()`) zeroes the arm's kp/kd along with its setpoint, so
    after it the vendor's own thread accepts every command and the arm hangs
    limp under gravity compensation: a driver without a latch reports commands
    as applied while nothing moves, and every episode after the stop reads
    SUCCESS. The latch is set BEFORE the vendor call (a stop that
    half-happened is still a stop), every write is refused while it holds, and
    the only exit restores the gains snapshotted at connect — or refuses,
    because a made-up kp is how an arm slams. The gripper range is passed on
    every connect rather than discovered: building without it runs an
    auto-calibration that physically drives the jaws. It reads defensively in
    one direction only: an absent velocity is reported as zero because the wire
    has no "unknown" for one, and an absent POSITION is a fault — the hand
    included, since this module declares it as a joint row rather than a
    sidechannel. A fabricated 0.0 there would be recorded as a measured closed
    hand, and it is the number the envelope measures the next command's
    per-step cap against, so guessing it would let a large uncommanded jaw
    motion through the check that exists to refuse one.
  - **A refusal never leaves an arm open.** The bus is already up by the time
    anything can refuse (an arm that reports a different DOF than this module
    declares), and a constructor that raises hands its caller an exception
    rather than a driver — so the handle it opened is closed before the refusal
    propagates, or the vendor's own ~1 kHz server thread keeps re-sending the
    last setpoint to an arm nothing can reach.
  - **The vendor package is a documented command, not a dependency.** It is
    imported lazily inside `__init__`, so importing `waddle_sdk.robots.yam` on a
    machine that has never seen a YAM is an ordinary import; asking for a live
    arm without it raises with `I2RT_INSTALL` in the text, which is BUILT from
    `I2RT_PIN` and so cannot drift from the commit every fact in the module is
    stated against. It cannot be a `waddle-sdk[yam]` extra: PyPI rejects
    direct references, and the tree behind it (an exact numpy, a simulator
    stack) is not something an install that only supervises a policy should
    resolve.
  - **`yam.declaration()` is public and stands alone.** A program that wants
    only the declaration — its own driver, its own loop, a plain
    `waddle_sdk.init` — gets exactly the robot a factory would have registered.
    That is a test, not a claim: `sdk/tests/test_robots_yam.py` compiles
    `yam.bimanual(...)` to JSON and compares it field for field against a
    verbatim replica of the customer program already running at the reference
    rig, with every number typed out independently.
  - **`yam.bimanual()` / `yam.arm()`** hand back a `base.Rig`: declaration,
    drivers, the owner's envelope and the reporting loop, still opening
    nothing until `rig.arms()` is called. The site facts have no defaults —
    the workspace box, the bench-measured `[closed, open]` motor radians, the
    CAN channel, the cross-arm mounting — while the choices that do (rate,
    speeds, part and frame names, twin homes) are named as choices where they
    are written down. The per-step cap the envelope enforces is DERIVED from
    the declared speed and rate, so the number a teleoperator reads off the
    declaration and the number that refuses a jump cannot disagree. Arms open
    one at a time, so `arms()` can fail with some of them already connected —
    and it closes those before it re-raises: half a rig is not a rig, the
    caller is holding no handle to close them with, and on metal they would
    otherwise stay energized under the vendor's own re-send. The exception that
    started the unwind is still the one the caller sees.
  - **Two things are opt-in, and both say what opting out costs.** Forward
    kinematics: `fk=None` is a legal rig that reports joint positions only,
    and a workspace box — a statement about a TCP — is then refused rather
    than silently unenforced. The cross-arm edge: with none declared, a pose
    expressed in the other arm's frame refuses downstream instead of resolving
    through an identity nobody measured.
  - **One chain may carry the shipped model; two may not.** `yam.arm()`
    declares `kinematics_urdf` from the vendored URDF and `yam.bimanual()`
    does not — that field describes ONE chain, and naming one arm's would name
    the other's tool frame as something it is not (asked for explicitly, it is
    refused rather than dropped). Where the model IS carried, the rename
    between its own root link and the frame poses are reported in is declared
    as the identity edge it is, so a consumer is not handed two unrelated
    trees.
  - `posture=` is a factory argument, mapping to which `Control` verbs the
    session registers and — on live hardware — to opening the arms in the
    vendor's zero-gravity mode under `monitor`, where the driver then refuses
    to write at all. No authority logic anywhere near it.
  - `base.CrossArm` (vendor-neutral, re-exported by `yam`) carries a measured
    xyz + rpy mounting and converts it ONCE to the wire's wxyz quaternion, so
    no call site is in a position to declare an xyzw one — the classic silent
    corruption, and one that reads as a plausible pose.
- **waddle-protocol (part-addressed control, new feature flag
  `waddle.v0.parts`)**: the normative surface for intervening on ONE declared
  part of a `Composite` robot — one arm of a bimanual cell — without
  inventing values for the others. No `.proto` change: `Action.part`,
  `CompositeAction`, and `ProprioSample.part` have been on the v0 wire since
  the beginning; what is new is that a connection may negotiate having them
  honored.
  - **VERSIONING.md registry row**: the flag gates honoring `Action.part` at
    the intervention-chunk intake (validate and flatten against *that part's*
    space, dispatch part-tagged) and emitting a non-empty `ProprioSample.part`
    on the `StreamObservations` uplink. Declared **iff** the client's declared
    action space is `Composite`. It is a flag rather than a defect fix because
    the pre-flag behavior is defined and legible — a part-scoped action
    flattens against the whole space and the chunk is refused with
    `Fault{FAULT_KIND_VALIDATION_ERROR}` — and §3 forbids a plane planning
    against behavior a connection did not declare. The row defines the pre-flag
    behavior in BOTH directions, so neither is left to an implementation: on a
    connection without the flag, per-part proprio is withheld from the uplink
    entirely rather than relabeled `""`, which would put one arm's joint vector
    on the wire as the whole robot's and let parts overwrite each other.
    Local MCAP recording is not connection-scoped and always records `part`
    (withholding is an uplink rule, never a recording one); media-plane part routing
    (`PartTarget.part`, `ClutchTransition.part`) is explicitly NOT gated here
    and remains unimplemented in v0.
  - **GLOSSARY.md** gains **part**: a named sub-space of a `Composite`
    declaration, normative declaration order, depth pinned to 1, `""` = the
    sole/default part — and the line that keeps it honest, that a part is an
    addressing axis on actions and proprioception, never an authority axis
    (claims, leases, and handoffs stay whole-robot single-writer in v0).
  - **FSM.md §4/§5 prose, no guard-row changes** (this is intake validation
    and dispatch shape, not a transition): a part-scoped action validates
    against the addressed part's dims (wrong width → the ordinary whole-chunk
    dims refusal; undeclared name or nested composite → not executable, its
    own words), and executes as **"move this part, hold the rest"** — the
    generalization of the shipped gripper-only contract, where "hold" means no
    new command is sent. The two rejected alternatives are written down with
    their defects: passing the caller's values through for the unaddressed
    parts resumes the paused policy's actuation under the intervenor's
    provenance with no transition saying so, and hold-filling from the last
    commanded point fabricates a full-width command the sender never issued
    into `/waddle/actions` (with no anchor at all when the caller never
    ticked). §5: a part-scoped action does **not** cross-fade into a
    whole-robot point — a cross-fade needs two endpoints of the same scope, so
    the gate holds, without faulting, until the blend window closes. That hold
    **discards** what comes
    due rather than deferring it: an action whose playout time falls inside the
    window is consumed and dropped, so a streaming sender pays `blend_ns` of
    hold and nothing else, while a sender that issues one part-scoped chunk
    instead of a stream loses it entirely and must declare HOLD_FIRST or
    `IMMEDIATE{blend_ns: 0}`.
  - **Conformance**: `scenario-format.md` gains the `intervention_chunk`
    inject kind (the control-plane chunk arm had no scenario surface at all)
    and the `expect_output.part` matcher; `fixtures/wire/action_chunk_part_scoped.json`
    pins the wire shape; five scenarios pin the behavior
    (`bimanual_part_scoped_substitute`, `bimanual_part_dims_mismatch_faults`,
    `bimanual_unknown_part_refused`, `bimanual_part_scoped_blend_holds`,
    `bimanual_part_scoped_gripper_crosses_anchorless_blend`).
    Runners that do not implement the flag skip them by the `requires_features`
    rule. `scenario-format.md` also states, once and for all, that scenarios
    pin the SHAPE of an emission and never an implementation's wording:
    `Fault.source` is implementation-named, so a scenario asserts
    `"source": "$nonempty"` — that a producer is named — rather than freezing
    one runner's spelling into an append-only golden.
  - **waddle-core (`waddle-conformance`), the reference runner implements
    both**: an injected `intervention_chunk` runs the same intake shape the
    runtime's does — admitted on an active claim alone (so a chunk arriving
    mid-ENGAGE is ready the instant the handoff completes), validated whole
    against the declared space, buffered on the intervention stream at
    receive-time plus each step's own `t_offset_ns`, on its own channel and
    seq space — and `expect_output` reports the part a substitute or a blend
    addresses. Every scenario in `fixtures/behaviors/` now RUNS: the suite
    asserts the skipped set is empty by name, so a fixture written ahead of
    its flag fails rather than passing with its behavior unchecked. The
    runner's once-per-claim-window refusals are keyed to the claim window's
    identity, as the runtime's are, and its three refusal reasons latch
    independently — "which parts exist" and "how wide this part is" are
    different disagreements and a sender is owed each of them once.
  - **`expect_send` names the part too** (`scenario-format.md`,
    `waddle-conformance`): the bypass pump is the one path an intervention
    action takes to the robot without passing through `gate()`, so the
    dispatched action's part rides its send log, spelled exactly as
    `expect_output` spells it (the addressed part's name, `""` for a
    whole-robot action). Without it a part-addressed command dispatched
    during a stalled caller loop was a width and nothing more, and the
    reference runner dropped identity the standard's own dispatch preserves.
  - **Eight behavior scenarios** turning that intake's prose into
    assertions, since a contract only the commit message states is a
    contract nothing checks: a chunk buffers mid-ENGAGE and substitutes on
    the first tick after the handoff (`agent_chunk_buffers_during_engage_handoff`),
    is dropped without a claim and does not resurface in the next one
    (`agent_chunk_dropped_without_a_claim`), plays each step out at
    receive-time plus its own `t_offset_ns`
    (`agent_chunk_step_offsets_play_out`), and is refused at most once per
    reason per claim window — with the successor claim owed its own answer
    (`agent_chunk_refusals_latch_per_reason_and_window`, and
    `teleop_dims_refusal_is_per_claim_window` for the media intake's guard,
    now the same latch); a part-scoped action dispatched by the bypass pump
    names its part (`bimanual_part_scoped_bypass_send`); and per-part
    proprioception is scored against the part it describes
    (`bimanual_part_scoped_proprio_scoped_to_its_part`,
    `bimanual_part_scoped_dual_write_detected`). Each one was measured to
    fail against a runner mutated to break exactly the sentence it pins.
  - **waddle-core (`waddle-types`)**: the wire↔row seam learns parts.
    `flatten_action` / `ActionChunk::from_pb` take a `PartPolicy`
    (`Honor` | `Ignore`) — the flag decision belongs to whoever negotiated the
    connection, and nothing below that line reads a flag. Under `Honor` an
    action naming a declared part is flattened and validated against **that
    part's** own space and dims and comes out as a `Step` tagged with
    `part: Option<Arc<str>>` (`Arc` because the tag is minted once at the
    intake and then cloned per tick on the gate's fast path). The part is
    resolved before anything is decoded, so an undeclared name is refused as
    an unknown part rather than reported as a width mismatch — the two are
    different facts and senders fix them differently — and a part-scoped
    action carrying a `CompositeAction` is refused by name, since v0 pins
    nesting to depth 1. `unflatten_action` takes the tag and rebuilds the
    part's wire `Action`, without which a part-scoped dispatch would land in
    `/waddle/actions` as an empty action list: a recording that says a tick
    commanded nothing when it commanded one arm. `Ignore` is the pre-flag
    reading, byte-for-byte, and is what every intake still passes until the
    plane connection negotiates the flag.
  - **waddle-core (`waddle-gate`)**: an action leaving the gate carries the
    part it commands. `OwnedAction` gains `part: Option<Arc<str>>` (`None` =
    the whole declared space), so a substitute's return tells the caller which
    part to write and the gate record behind it names the part that moved —
    an untagged row would claim the whole robot did. The tag is a shared
    pointer rather than owned bytes because a claimed tick clones the
    dispatched action twice on the customer's real-time thread (three times
    when it blends); `tests/alloc_free.rs` grows a part-tagged CLAIMED-arm
    proof — the first to drive that arm with an action actually pending —
    measured differentially against the identical untagged loop, and the gate
    benchmarks are unchanged.
  - **waddle-core (`waddle-gate`), cross-fade endpoints**: `blend_step` takes
    the anchor as an `Option` and refuses any pair that is not two endpoints
    of the **same scope** — each refusal is the hold FSM.md §5 specifies.
    Beyond the shipped width defense (a part-width action against a
    whole-robot point), two refusals are new: a pair naming two DIFFERENT
    parts, whose widths match whenever the parts are symmetric, so no width
    check can see it and blending would fade one arm's last commanded point
    into the other arm's target — a trajectory no sender issued, dispatched
    and recorded under that sender's provenance; and a part-scoped ARM ROW
    with no anchor at all (an episode whose caller never ticked, FSM.md E24),
    where the gate used to manufacture an anchor out of the target itself and
    so would have faded a one-arm command in as if it commanded the whole
    robot. The scope rule binds a gripper-only action too: it is exempt from
    the width check (it has no arm row by construction) but not from the scope
    one, since it still fades from the last commanded grip — the shape
    `flatten_action` builds from a part-scoped noop plus a gripper. Scope and
    width stay two rules rather than one: a whole-robot point is held out of a
    proper part's cross-fade by *width* (the gate has no part layout to slice
    it with), so a part-tagged gripper-only action, which has no rows to
    slice, does fade out of that point — it commands every part's grip, and v0
    carries one gripper channel per action, never one per part. The same
    premise decides the anchorless case, where §5 now says so explicitly: a
    part-scoped gripper-only action crosses that window as itself, part tag
    and all, because it fabricates nothing for anyone; narrowing the exception
    to "no anchor and no tag" would drop a commanded grip in exactly the
    episodes that have no anchor, and
    `bimanual_part_scoped_gripper_crosses_anchorless_blend` scores the arm row
    and the grip against each other in one window. A blended
    action now carries its target's part tag, so the one-part `Composite`
    that legitimately cross-fades does not silently widen into a whole-robot
    command. FSM.md §4 and §5 state the scope rule.
  - **waddle-core (`waddle-runtime`), the connection that negotiates it**: the
    SDK declares `waddle.v0.parts` at Register **iff** the declared action
    space is `Composite` (the `waddle.v0.obs.stills` rule — declare from the
    robot's declaration, never claim a behavior this session cannot exhibit:
    a single-part robot has no part to address, and `""` is already core).
    The plane pump refreshes acceptance on every `Registered` (flags are
    re-negotiated on each reconnect) onto `Status.parts_negotiated`, and the
    intervention-chunk intake honors `Action.part` only while it is set —
    without it the chunk is read against the whole declared space and refused
    exactly as before, once per claim window, which is the behavior a plane
    that did not negotiate the flag is entitled to plan against. Acceptance
    belongs to the connection that gave it, in both directions — see the two
    lifecycle entries under **Fixed**, which this flag's uplink is the first
    non-droppable producer to depend on.
  - **waddle-core (`waddle-runtime`), the dispatch and the row**: the part tag
    rides from the intake through the gate to the declared `send` verb —
    including the BYPASS/reset path, where the pump, not the caller's gate,
    is what reaches the robot (and is the ONLY path an agent-invited episode
    has). The `/waddle/actions` rows behind both name the part: a part-width
    row is rebuilt against that part's space, where before it did not decode
    against the whole space at all and the recording said the tick commanded
    NOTHING when it commanded one arm.
  - **waddle-core (`waddle-runtime`), per-part proprioception**:
    `ProprioReport` gains `part` and `joint_pos`, and
    `Session::report_proprio` is now fallible — a part the declaration does
    not have is refused BY NAME at the call, rather than landing under a key
    nothing will ever read. `joint_pos` is what makes a named part
    reportable at all: a per-part sample cannot ride the gate's flat `obs`
    vector, because the observation layout is not the action layout and
    slicing one by the other would invent a mapping the customer never
    declared. The reducer keeps its latest known state per part instead of
    once for the robot, so one arm's report can never be read as the other's
    (or as the whole robot's), each part gets its own `/waddle/observations`
    row naming itself, and the 10 Hz `StreamObservations` cap is charged
    **per part** — parts are independent content streams (the per-camera
    `still_fps` precedent), and one shared slot would deliver each of N parts
    at ~10/N Hz and could starve one entirely while the plane's freshness
    checks key on exactly the part they ask about.
  - **waddle-core (`waddle-runtime`), `push_intervention_chunk`**: the local
    counterpart of a plane `intervention_chunk`, exactly as
    `grant_and_engage` is the local counterpart of a `ClaimDirective` — a
    test, or the SDK's testing hooks, can now drive an intervention without
    standing up a control-plane transport. It is the SAME intake the plane
    pump runs, extracted rather than reimplemented, so the two cannot drift
    on validation, on the once-per-claim-window faults, or on the ring-seq
    discipline. The one thing it cannot inherit is the negotiation: with no
    connection to have negotiated with, it honors `Action.part` from the
    same fact Register declares the flag from — whether the declared space
    has parts at all.
  - **sdk (`waddle-sdk`), one rule for every payload that names a part**: on a
    `Composite` declaration, an intervention payload crossing into Python is
    **keyed by part**. `episode.gate(...)` returns `{"right": ndarray}` for an
    action addressing one arm and every declared part, sliced by the declared
    layout, for a whole-robot one; a dispatched `Chunk`'s step values follow
    the same rule; `GateInfo.part` names the addressed part on both
    (`None` = the whole robot). The parts absent from the dict are commanded
    nothing — "move this part, hold the rest" said in the shape of the
    payload — and a gripper-only step maps its parts to empty arrays, the
    same "hold the arm, move the gripper" an empty array has always meant.
    The slicing is arithmetic over the customer's own declaration (declaration
    order IS the concatenated layout), never invention. Without this, one
    arm's 7 values arrive indistinguishable from a 14-row whole-robot
    command, on the DEFAULT (gRPC-enabled) wheel, for any customer whose
    declaration is `Composite` — which is exactly the confusion the flag
    exists to prevent. **This closes the release gate** the `GateInfo.part`
    work left open: `waddle.v0.parts` is now expressible end to end on this
    distribution.
  - **sdk (`waddle-sdk`), `report_proprio(part=, joint_pos=)`**: report one
    part's state at a time (`ProprioSample.part`), refused by name with a
    `ValueError` if the declaration has no such part. `joint_pos` is a kwarg
    here for the same reason it is a field in core: a per-part sample cannot
    ride the flat `gate(obs=...)` vector, because the observation layout is
    the customer's own and no declaration describes it. `part=""` (the
    default) is the robot as declared and behaves exactly as before.
  - **sdk (`waddle-sdk`), `waddle_sdk._testing.push_chunk`**: the private test
    hook for the local intervention seam — one part-addressed (or
    whole-robot) step pushed into a session with no control-plane transport,
    running the core's own intake. A whole-robot push on a `Composite`
    declaration is marshalled into the `CompositeAction` the wire requires,
    split by the same declared layout the returns are keyed by.
  - **waddle-core (`waddle-ffi`), the C ABI names the part too** (ABI
    UNSTABLE, N5): `WaddleGateResult` gains `part` (an inline NUL-terminated
    buffer, `WADDLE_MAX_PART_NAME`) and `part_len`, and the `send` callback
    takes a fifth argument, `part_or_null`. Both are source-breaking for C
    consumers, deliberately: without them a part-scoped substitute crossed as
    a bare 7-wide vector that a 14-row cell can only read as the whole robot,
    which is one arm's setpoint written into the other's. `part_len` reports
    the name's true length so a truncated name is detectable rather than
    silently answering to another part's; a name that cannot cross as a C
    string fails the verb rather than crossing as NULL.
  - **What a part still does NOT address, stated rather than left to be
    discovered.** Parts are an addressing axis on intervention actions and on
    proprioception, and nothing else moved: a claim, a lease and a handoff
    stay whole-robot single-writer, the sidecar and episode schemas keep no
    part axis, and a `rate_hz` declared below the top level is still parsed
    and unused. The **teleop media plane is not part-routed** — a
    `TeleopStreamPacket`'s `PartTarget.part` is ignored and its targets are
    concatenated in packet order, `ClutchTransition.part` is dropped, and
    (the one outright defect in that set, now written down in
    `flatten_packet`'s own doc comment and mirrored in the reference runner's)
    **only the first target's gripper survives a packet**, so a bimanual
    teleoperator closing both hands in one packet closes one. It is deferred
    rather than patched because v0 carries a single gripper scalar per action
    end to end, so every local repair either invents a channel or turns
    working single-gripper teleop into refusals; the honest fix is part-scoped
    targets, which is the same deferred stage that would route
    `PartTarget.part` at all. None of it is reachable from the canonical
    bimanual declaration this flag serves, which folds each part's gripper
    into that part's joint vector (`Gripper.parallel(dim = -1)`), where it is
    an ordinary row that the part-scoped path above already carries.
- **waddle-protocol/waddle-core (agent-invited episodes, new feature flag
  `waddle.v0.agent`)**: a customer can now ask Waddle to drive an episode
  rather than driving it themselves — `Session::run_agent(prompt, timeout_ns,
  opts)` opens an *agent-invited* episode and blocks until it terminates,
  returning `AgentOutcome { outcome, episode_id, recording_ref, detail }`.
  **The invite adds no authority concepts.** It is one `EpisodeEvent`
  (`AgentInviteEvent { prompt, timeout_ns }`, arm 18) forwarded to the plane
  like any other emission; the hosted agent then claims the episode with the
  EXISTING intervention machinery (`ClaimDirective{GRANT, ACTOR_KIND_AGENT}`
  → engage), streams chunks on the EXISTING `intervention_chunk` arm, and
  finishes with the EXISTING `EpisodeDirective{MARK_DONE}` +
  `ClaimDirective{RELEASE}`. Everything else about the episode — E7 engage,
  chunk handoff, E10 termination, both reset phases — applies verbatim.
  - **Protocol**: `AgentInviteEvent` (episode.proto);
    `AgentTaskUpdate { episode_id, kind, detail, recording_ref,
    directive_id? }` with `AgentTaskUpdateKind`
    (UNSPECIFIED/QUEUED/DENIED/COMPLETED) as `GateServerMessage` arm 7 — the
    plane's status channel for the ask itself, distinct from the episode's
    own timeline; `NOOP_REASON_AGENT_EPISODE = 4` (control.proto). All
    append-only; registry row in VERSIONING.md.
  - **FSM.md §1.5** (guard rows E23–E26/E26b, C8): E23 opens the episode,
    emits the invite, and arms `agent_invite_timeout`; **E24** — the caller's
    own `gate()` ticks NEVER dispatch while no claim is engaged (plan Noop,
    reason `NOOP_REASON_AGENT_EPISODE`; no fault, no state change), which is
    what makes "you asked, Waddle drives" honest rather than a race between
    two writers; **E25/E26** (deadline elapsed, or a plane `DENIED` while the
    invite is open) are declared **members of E10's trigger set** with the
    outcome fixed to ABORT, so E14 routes them through the episode's normal
    termination (TERMINAL{ABORT}, or POST_RESET{ABORT pinned} when
    post-reset is declared) instead of around it; **E26b** records a late
    `DENIED` as an event only, so it can never disturb a pinned outcome.
    **C8** admits `ACTOR_KIND_AGENT` claims only, and records the refusal of
    any other actor as `claim{DENIED}` — a declared reset window on an
    agent-invited episode still admits its teleoperator under C6, which
    C8 does not touch. Two latches are emission-invisible state:
    `agent_engaged` (set by the first agent ENGAGE; never re-arms the invite
    timer on a release/re-engage) and `invite_aborted` (set by E25/E26 and
    nothing else, so an embedder can tell "the ask went unanswered" from
    "the episode broke for unrelated reasons" without parsing reasons).
    The invite closes on the first of an agent ENGAGE or any exit from
    {RESETTING, READY, RUNNING}; every closing row cancels the timer, and a
    stale expiry after close is discarded.
  - **Conformance**: eight new scenarios in `fixtures/behaviors/`
    (`agent_invite_happy`, `agent_caller_tick_noop`, `agent_invite_timeout`,
    `agent_invite_timeout_post_reset`, `agent_invite_denied`,
    `agent_invite_denied_after_engage`, `agent_invite_wrong_actor_denied`,
    `agent_invite_denied_in_post_reset`), all listing `waddle.v0.agent` in
    `requires_features`; a ninth, `agent_invite_retake_successor`, arrives
    with the E24 re-projection fix below, and a tenth,
    `agent_invite_clutch_denied`, with the C8 clutch fix under Fixed.
    `scenario-format.md` gains the `episode_open`
    `agent_invite` key, the `agent_task_update` inject kind (the update
    nests as a canonical `waddle.v0.AgentTaskUpdate` under `update`, the
    shape `reset_result` already uses — the message's own `kind` field
    cannot ride flat beside the inject dispatcher's `kind`), the
    `episode.agent_invited` / `episode.agent_engaged` snapshot paths, and
    the `agent_invite_timeout` timer id. The runner now captures a Noop's
    reason from the plan mode that produced the tick instead of hardcoding
    `BYPASS_ACTIVE`, which also fixed a latent runner-vs-E20 drift.
  - **Runtime**: `EpisodeOptions.agent_invite` (waddle-fsm's own
    `AgentInvite`, re-exported — the frontend stays hollow) opens without
    blocking; `run_agent` is the blocking convenience and fails loudly up
    front when the engage-path verbs the invite needs are unwired. The plane
    pump retains every `AgentTaskUpdate` on the mirror
    (`Status.agent_task`); only a DENIED addressed to the ACTIVE episode is
    dispatched to the FSM, which alone picks E26 vs E26b — QUEUED/COMPLETED
    never touch it, and COMPLETED's `recording_ref`/`detail` feed the
    outcome. The mirror publishes `agent_invited` / `agent_engaged` /
    `agent_invite_aborted`. The flag is declared at Register
    unconditionally whenever a transport is configured: the SDK always
    supports being agent-driven, and a plane that did not accept it simply
    never routes an invite (the deadline then closes the episode via E25).
  - **Gate**: `waddle-gate` gains `PlanMode::AgentEpisode { provenance }`
    (mirroring `Bypass` and `Reset`): `Gate::gate()` returns `Noop` and
    records the new `GateDecision::AgentEpisode`, same cost class as the
    existing NOOP paths (no locks/syscalls/allocations); the runtime
    reducer derives that plan for an agent-invited episode with no engaged
    claim (E24) and renders `NoopReason::AGENT_EPISODE` distinctly from
    `BYPASS_ACTIVE` and `RESET_ACTIVE`, so a recording says *why* a tick
    dispatched nothing. Both variants are **source-breaking for an
    exhaustive match** on `PlanMode` or `GateDecision` (pre-1.0 / API
    unstable per N5).
- **waddle-protocol/waddle-runtime (control-plane stills, new feature flag
  `waddle.v0.obs.stills`)**: a hosted agent needs to SEE the scene, and until
  now `publish_frame` fed only the media plane — an agent-only session with no
  LiveKit anywhere had no frame path to the control plane at all. A camera
  declaring `StreamPolicy.still_fps > 0` (descriptors.proto field 3; 0/absent
  means no stills) now has each published frame teed into a latest-wins
  per-camera slot, which the existing `waddle-media-uplink` pump samples at
  the declared rate, JPEG-encodes (never on the customer's thread, never on
  the gate path), and sends as `ObservationUpdate{ still: FrameStill }`
  (`FrameStill { camera, frame_seq, encoding, width, height, data }`,
  services.proto payload arm 5). **Bounded by declaration — never a video
  path; LiveKit media remains the only video transport**, and the file-header
  and StreamPolicy comments asserting that nothing high-bandwidth rides these
  RPCs now name this bounded exception explicitly.
  - The intake grew a second, independent leg rather than a branch: the media
    leg keeps its own fps throttle and bounded drop-oldest queue, the stills
    leg its own frame-timeline throttle and capacity-one slot; neither
    rate-limits the other. `CameraUplink` is built whenever EITHER leg has
    somewhere to go, so a stills-only camera works with no media plane, while
    a session with neither keeps `publish_frame`'s cheap no-op early return
    and declared-uplink validation stays scoped to cameras that would
    actually be wired.
  - The throttle reads the frame's own `SessionClock` stamp, never a
    pump-side clock, so the sampled rate is a property of the frames and the
    sampling is deterministic under test; a not-yet-due frame is kept rather
    than discarded, so a publisher slower than its declared rate still gets
    every frame sampled. `frame_seq` is minted once per validated
    `publish_frame`, before either throttle — it numbers the camera's frames,
    not the subset a policy admitted, and is THE per-camera `FrameNotice`
    counter for whatever emits `FrameNotice` later. Stills stay out of
    `camera_frames_dropped`, which keeps meaning exactly media-uplink loss.
  - The flag is declared at Register iff some camera asks for stills
    (declaring it otherwise would claim a behavior the session cannot
    produce), and stills are emitted only while the CURRENT connection
    accepted it (VERSIONING §3), refreshed on every registration by the plane
    pump — the emitting thread is not the one that sees the response.
    Control-plane stills are also the first *droppable* history-free message
    class: see the `waddle-controlplane` entry under Fixed for how they are
    shed rather than buffered while a plane is unreachable or stalled.
- **sdk (Python: `init(transport=…, media=…)`, `waddle_sdk.agent()`, and the
  `[teleop]` companion distribution)**: the connected surface, in the two
  places a user actually touches it.
  - **`waddle_sdk.init(transport=waddle_sdk.Grpc(url, token=None))`** connects the
    supervision plane, and **`waddle_sdk.init(media=waddle_sdk.LiveKit(url, token))`**
    the teleop media plane — both small frozen config dataclasses handed
    straight to core constructors; nothing in Python inspects a connection.
    The two are mutually exclusive with `_testing=True` (which wires the
    in-process loopback), and `media` requires its token, because the plane
    mints room tokens and this SDK never does.
  - **`waddle_sdk.agent(prompt, *, timeout_s=600.0, pre_reset=…, post_reset=…)
    -> AgentResult`**: hand a
    whole episode to Waddle instead of driving it. It blocks until the
    episode reaches an outcome and returns a frozen
    `AgentResult { outcome, episode_id, recording_ref, detail }`, each field
    the core's word verbatim. An ask nobody answers comes back
    `outcome == "abort"` at the invite deadline — a result, not an
    exception. `pre_reset`/`post_reset` override this one episode's reset
    phases exactly as they do on `rollout()` (an agent run is an episode
    like any other, and the reset kwargs `_core`'s `Session.agent` takes
    were unreachable from Python without them). It refuses up front only
    when there is nobody to ask (neither a `transport` nor the `_testing`
    loopback that stands in for one) or when the verbs an invited claimant
    would need are
    unwired; everything else — who may claim the episode, what the caller's
    own ticks do meanwhile — is an FSM row, not a Python branch.
  - **Two distributions from one source tree** (the psycopg / psycopg-binary
    shape): `pip install waddle-sdk` carries the gRPC control transport, and
    `pip install 'waddle-sdk[teleop]'` adds the exact-pinned
    **`waddle-sdk-teleop`** companion — the SAME shim, same
    `rust/Cargo.toml`, built with `livekit` as well. Splitting them keeps
    libwebrtc's ~690 MB of build out of installs that only supervise a
    policy; measured, the companion is ~4.5x the default wheel. Either way
    you `import waddle_sdk`: **`waddle_sdk._native`** is the one module that picks a
    core, preferring the companion when a version-matched one is installed,
    warning and falling back to the bundled core when the two versions
    disagree (a half-upgraded environment must not load a core built from
    other sources), and honouring `WADDLE_NO_TELEOP=1`. The extra's exact
    pin is the one version maturin cannot derive from the manifest, so a
    test holds it to `waddle_sdk.__version__` rather than to memory.
  - `waddle_sdk.__version__` re-exports `_core.__version__`, so the Python
    surface and the compiled core can always be reported as one thing.
- **sdk (connected shim: transport features, `_core.FEATURES`,
  `Session.agent`)**: the PyO3 shim can now be BUILT with the transports the
  core has always been able to drive. Two cargo features — `grpc` (the tonic
  `ControlTransport`) and `livekit` (the media plane) — plus four
  `create_session` kwargs (`transport_url`/`transport_token`,
  `media_url`/`media_token`). A build without the matching feature REFUSES
  the kwarg with an actionable error instead of degrading to a silent
  offline session: quietly losing supervision is the one failure mode this
  layer exists to prevent (the LiveKit refusal names the `[teleop]` extra,
  since "not compiled" alone is not something a caller can act on). Each
  core states its own `FEATURES` (a frozenset of the built feature names)
  and `__version__`; the frozenset the Python layer actually probes is
  `waddle_sdk._native.FEATURES`, re-exported from whichever core `_native`
  selected — reading `waddle_sdk._core.FEATURES` would describe the bundled
  core even in a process running the teleop one. It is the only feature
  detection the Python layer may do. The shim gains
  kwargs, never logic. Neither feature is a cargo default (the featureless
  build stays the tokio- and libwebrtc-free clippy baseline), but no shipped
  distribution is featureless — see the two-distribution entry above.
  - `Session.agent(prompt, timeout_ns, **reset overrides)` binds
    `Session::run_agent` (flag `waddle.v0.agent`) and returns
    `AgentResult { outcome, episode_id, recording_ref, detail }`, each field
    core's word verbatim. It blocks for a whole episode, so the core call
    runs on a dedicated `waddle-py-agent` thread while the calling thread
    waits in 50 ms GIL-released slices and runs Python's signal handlers
    between them. A Ctrl-C is therefore heard within a slice instead of at
    the invite deadline; it asks the core to abort the live agent-invited
    episode (the same abort `Episode.terminate()` requests — the shim
    decides nothing about the timeline, and the core no-ops when that
    episode is no longer live) and keeps asking on every later slice until
    the run reports finished, since the first ask can land in a window with
    no live agent-invited episode to abort (the run thread has not opened
    one yet, or is still waiting out a predecessor's POST_RESET) and a
    one-shot ask would then leave the caller blocked to the deadline with
    the interrupt already latched. `KeyboardInterrupt` is raised only once
    the core reports the run finished, so no agent is left driving a robot
    whose caller has walked away, and no thread is orphaned.
  - `sdk/rust/Cargo.lock` pins the livekit crates to the set
    `waddle-core/Cargo.lock` already resolves (livekit 0.7.52, -api 0.5.5,
    -protocol 0.7.10, -common 0.1.0, -data-stream 0.1.0, -datatrack
    0.1.11): the newest published set does not compile (livekit-api 0.5.6
    is missing a field livekit-protocol 0.7.12 added). Keeping two
    lockfiles honest is the standing cost of the shim's separate workspace.
  - New pytest module `tests/test_agent.py` covers the probe's shape, the
    refusals, an invite-timeout `agent()` returning `outcome == "abort"`,
    and the interrupt path (a real `SIGINT` mid-run, asserting both that
    `KeyboardInterrupt` arrives promptly and that the session can open a
    fresh episode afterwards — i.e. the run really ended).
- **sdk (`examples/toy_robot.py`: a whole robot integration in one file)**:
  the program a customer would actually run, and the first artifact in this
  repo that exercises the declaration surface, the loop, the media/stills
  legs and `waddle_sdk.agent()` together rather than one at a time. A 6-dof arm
  with per-joint limits, a parallel gripper declared in **metres** (`0.0`
  open / `0.04` closed — deliberately not 0/1, so the normalized-to-declared
  mapping a claimant's gripper command goes through is actually exercised), a
  generated 6-joint URDF, and one 320x240 camera declaring both an `uplink`
  (media plane) and `still_fps=2` (control plane). The robot is a small
  kinematic simulator in the same file, so the example runs with no hardware,
  no network and no plane — offline it still gates every action and lands a
  sidecar + MCAP per episode, which is what makes it usable as a first
  five-minute experience. One environment variable
  (`WADDLE_TOY_TRANSPORT`) turns the same program into a supervised session;
  `WADDLE_TOY_MODE=agent` makes it hand a whole episode to Waddle and report
  the `AgentResult`. Status lines are prefixed `[toy] ` and flushed, so a
  harness can drive it. `examples/README.md` documents the three
  configurations and what the `[teleop]` extra adds on top.
  Three details the reference integration is deliberate about, because a
  customer copies this file:
  - **The claimant's gripper is sent, not the policy's.** A grasp does not
    ride `gate()`'s return value — it arrives on `ep.last_gate.gripper`,
    already converted out of the normalized 0..1 wire into the metres this
    robot declared — so the loop sends that whenever it is present and its
    own value only on a passthrough tick. Sending the scripted value beside
    substituted joints would move the arm where the teleoperator asked while
    silently dropping the grasp.
  - **A latched e-stop survives the scene reset.** `ToyArm.home()` refuses
    (returns `False`, moving nothing) while stopped, and `pre_reset` returns
    `False` rather than vouching for a scene it did not reset; only
    `clear_estop()` — standing in for the human at the machine — releases
    it. The envelope is the owner's, so no supervision flow may clear it.
  - **An empty environment variable means unset**, at both layers
    (`WADDLE_TOY_TOKEN=` is "no credential", `--token ""` likewise), because
    `VAR=${MAYBE_UNSET}` is how a harness parameterizes a child; previously
    those died in `int("")` or in credential validation before the first
    status line.
- **sdk (`waddle_sdk.StreamPolicy(still_fps=…)`)**: the Python declaration for
  control-plane stills (flag `waddle.v0.obs.stills`), which core has
  honoured since the entry above but no Python program could ask for. `None`
  (the default) and `0` both mean no stills, matching the wire; a negative
  rate is rejected as a shape error. It compiles to the `stillFps` key, held
  to that by reading the value back out of core rather than by a
  hand-written expectation: the new `_core.robot_json_roundtrip(json)`
  decodes a compiled robot exactly as core does and returns core's own
  canonical JSON of what it *understood*. Validation alone could not have
  said — decoding tolerates unknown fields by design (append-only
  evolution), so a misspelled key validates perfectly and is dropped in
  silence, and the first symptom would have been a customer's connected
  session sending no stills.
- **waddle-protocol (fixture `remote_reset_caller_tick_noop`)**: pins FSM.md
  E20's caller-tick marker, previously asserted by no golden — a `gate_tick`
  during an ENGAGED remote reset window returns
  `Noop{NOOP_REASON_RESET_ACTIVE}` and causes no episode transition (the
  stale-handle contract), and the first tick after `reset_window_complete`
  passes through and drives E6 READY→RUNNING, pinning the marker as
  window-scoped rather than sticky. Before this fixture the conformance
  runner's `(Noop, PlanMode::Reset)` marker-translation arm could be
  reverted to `BYPASS_ACTIVE` with the whole suite staying green; the E20
  row's Fixture column now lists it.
- **waddle-runtime (`ServerMsg::ResetProgress` handling)**: the
  plane-executed reset completion path (`RequestReset`/`ResetProgress`,
  `waddle.v0.reset`) is no longer dropped — every message updates a new
  `Status.reset_progress` mirror field (observational only; `episode.proto`
  doesn't model this as an `EpisodeEvent`), and `ResetProgress{DONE, result}`
  injects `SessionEvent::ResetResult` exactly like the inline/pump paths
  already do, completing the pipeline. No episode-id filtering (the message
  carries none — session-scoped, like `HeartbeatAck`); the FSM's own E19b
  guard (`ResetResult` requires `Phase::Resetting` with no open remote
  window) makes a stray or out-of-order DONE harmless. **Closes a
  long-documented gap**: a retake successor under a session-level
  `Remote` PRE spec is born-claimed, so its pre-reset window never opens
  (D7 edge 5); nothing else in the runtime could ever complete that
  successor's RESETTING. `RequestReset` issuance (the outbound half) stays
  unimplemented — no `ResetSpec` variant models "the plane executes this
  reset automatically," so there is no clean trigger to fire it from — a
  known open item.
- **waddle-runtime (`Session::report_proprio` + `StreamObservations`
  uplink)**: `report_proprio(ProprioReport { joint_vel, ee_pose, gripper })`
  reports a richer proprioceptive sample than the bare `joint_pos` every
  `gate(obs=...)` call already records; the reducer merges it with the
  latest gate-tick `joint_pos` into every recorded `ProprioSample` (Local
  mode, `/waddle/observations`) and into a periodic `ClientMsg::Observation`
  uplink sent whenever a transport is configured (10 Hz conservative
  default — no declared per-robot rate exists on this control-plane RPC to
  key off; see `Reducer::DEFAULT_OBSERVATION_UPLINK_HZ`'s doc). Every field
  PATCHES the reducer's latest known sample; `None` leaves a previously
  reported value in place (no way to clear one in v0). `ee_pose` is a
  frame-tagged `EePose` (position + wxyz orientation + a non-empty
  `frame_id`, per `descriptors.proto`'s `Pose` invariant) rather than the
  design sketch's bare `[f64; 7]`, since an untagged pose is exactly the
  silent-corruption failure mode that invariant exists to prevent.
- **sdk (Python `session.report_proprio`)**: `session.report_proprio(
  joint_vel=..., ee_pose=..., ee_pose_frame="ee", gripper=...)` — numpy
  `float64` ndarray or plain list accepted for `joint_vel`/`ee_pose` (same
  zero-copy-when-possible convention as `gate(action, obs)`); `ee_pose`
  raises `ValueError` unless it has exactly 7 values (xyz + wxyz).
  `ee_pose_frame` (default `"ee"`) names the frame the pose is expressed
  in — deliberately one kwarg wider than a bare 7-value pose signature,
  for the same frame-tagging reason as the Rust `EePose` type.
  Dev-only new dependency: `mcap-protobuf-support` (pulls in `protobuf`),
  used by the extended `test_nominal_episode` MCAP read-back test to
  decode `/waddle/observations` messages via the channel's own embedded
  `FileDescriptorSet` schema and assert the merged field values, not just
  topic/message counts. Not a runtime dependency of the SDK itself.
- **waddle-protocol/waddle-runtime (directive acks, new feature flag
  `waddle.v0.plane.acks`)**: plane→SDK directives are no longer blind
  fire-and-forget — an FSM rejection is now observable to the plane.
  services.proto gains an optional `directive_id` on `ClaimDirective` (field
  3), `EpisodeDirective` (field 5), and `ResetWindowDirective` (field 5),
  plus `DirectiveAck { directive_id, accepted, reason }` as
  `GateClientMessage` arm 4 (append-only). When a directive carries a
  `directive_id` AND the connection negotiated the flag, the SDK answers
  with exactly one ack per directive: `accepted=true` when the session FSM
  applied every event the directive decoded into, `accepted=false` with the
  FSM's rejection reason in guard-row language (e.g. "engage outside RUNNING
  (E7)", "terminate rejected in POST_RESET (E14b)", the C6 reset-claim
  admission reason) when any was rejected — a directive that decodes into
  two events (claim GRANT, reset-window ENGAGE) acks once, with the first
  rejection's reason. Zero guard-semantics changes: the FSM accepts and
  rejects exactly what it did before; acks are a runtime/plane behavior and
  never appear on the `EpisodeEvent` stream, in sidecars, or in fixtures.
  Directives without an id stay fire-and-forget; the flag is always declared
  at Register when a transport is configured (safe — emission still requires
  the id). Registry row in VERSIONING.md; normative ack paragraph in FSM.md
  §8.
  - `AgentTaskUpdate` (flag `waddle.v0.agent`) carries a `directive_id`
    (field 5) and is acked on the same rule, now stated in both normative
    enumerations rather than only in the code: an update is a directive
    exactly where it decodes into a session event — a DENIED addressed to
    the ACTIVE episode, accepted under E26 and rejected under E26b.
    QUEUED, COMPLETED, and a DENIED naming any other episode are recorded
    without an FSM step, so they never ack whatever their id says. Pinned
    by a new case in `waddle-runtime/tests/directive_acks.rs`.
- **waddle-gate/waddle-runtime (Claimed-mode agent-chunk intake + jitter
  horizon + `ReplanPolicy`)**: cloud-agent interventions are now real
  outside a reset window too. `forward_server_msg`'s `InterventionChunk` arm
  (previously Reset-mode-only intake) now accepts a chunk whenever a
  claim is active — the same `claim_active`-alone gate `spawn_media_intake`'s
  teleop path already uses, so a chunk arriving during the ENGAGE handoff
  sub-phase still buffers correctly and is ready the instant the handoff
  completes.
  - `waddle-gate::jitter::JitterBuffer` is chunk-aware on the `AgentChunk`
    channel: each arrival carries the wire chunk's `ChunkMeta`
    (`seq`/`t_emitted_ns`); a chunk boundary (a step from a different chunk
    than the channel's currently-executing one) decides stale-vs-supersede —
    `chunk_seq` (the one field `control.proto` normatively requires to be
    monotone per stream) is the primary staleness signal, so a chunk whose
    `seq` is not strictly newer is rejected wholesale (`dropped_stale_chunks`);
    `t_emitted_ns` is consulted only as an additional rejection when BOTH the
    executing and candidate chunk declare a nonzero value and the new one
    isn't strictly newer, so a wire-legal producer that leaves it at the
    proto3 default 0 (or ties it) is never wrongly locked out (a fixed review
    finding: the original `chunk_seq` **AND** `t_emitted_ns` rule rejected
    every subsequent chunk of a claim window forever the moment a producer
    left the timestamp unset). A genuinely newer chunk applies the declared
    `descriptors.proto` `ChunkingSemantics.replan`:
    `REPLAN_POLICY_IMMEDIATE`/`REPLAN_POLICY_BLEND` drop the executing
    chunk's still-pending steps (BLEND has no declared blend duration/curve
    for a chunk-to-chunk splice and its own comment steers away from it, so
    it maps onto the same replace-remaining behavior as IMMEDIATE — a
    documented simplification); `REPLAN_POLICY_CHUNK_BOUNDARY` lets them finish
    first. `clear_pending` (the existing claim/window-teardown discard) also
    forgets the executing-chunk pointer, so a brand-new claim's first chunk
    is never wrongly rejected as stale against an unrelated prior claim's
    last one. `GateShared::new`/`JitterBuffer::new` take the declared
    `ReplanPolicy` (from `ActionSpace.chunking.replan`) as a new parameter.
  - Playout scheduling stays session-receive-time + each step's
    `t_offset_ns` (unchanged from the Reset-mode intake) — chunk
    `seq`/`t_emitted_ns` are
    used only for the boundary/staleness decision, never as the playout
    anchor.
  - Dims validation: a chunk whose flattened width doesn't match the
    declared action space now raises `SessionEvent::InterventionRejected`
    (once per claim window, chunk dropped) — the same event/fault the
    teleop path already uses. The event gained a `source` field
    (`"media-intake"` / `"agent-chunk"`) so the emitted fault names the
    actual rejecting producer instead of always saying "teleop action" /
    "media-intake"; every other wire-validation error (missing field, wrong
    target arm, Opaque space, …) still only gets a `tracing::warn!` (Task
    10's reasoning: a dims-only event would misreport those).
  - New runtime e2e tests (`claimed_chunk_intake.rs`, `InMemoryTransport`):
    a 5-step Claimed-mode chunk substitutes in order via the caller's own
    `gate()`, tagged `Provenance::Agent`, with MCAP read-back; a superseding
    chunk mid-horizon under `IMMEDIATE` drops the executing chunk's
    remaining steps; a dims-mismatched chunk faults once per claim window
    and drops, a subsequent correct one still substitutes.
- **waddle-controlplane (real tonic gRPC `ControlTransport`, feature
  `tonic-transport`)**: the `tonic-transport` feature is no longer an empty
  stub — `waddle_controlplane::grpc::{GrpcConfig, GrpcTransport}` implements
  the same `ControlTransport` trait the in-memory transport does, over the
  eight `ControlPlane` RPCs of services.proto.
  - Mapping: `Register`/`Negotiate`/`ClaimEpisode`/`HandoffLease` are unary;
    `GateActions` + `Heartbeat` are eager long-lived bidi streams (the
    plane's directive/demotion down-paths); `StreamObservations` opens
    lazily on the first observation (acks are drained); `RequestReset`
    progress funnels back through the single ordered rx. Any transport-level
    error severs the connection channels, handing recovery to the client's
    existing backoff/replay machinery — the transport duplicates none of it.
  - Tokio confinement (the Task-14 pattern): one dedicated
    `waddle-controlplane-grpc` thread per live connection owns a private
    current-thread runtime (plus a `waddle-controlplane-grpc-tx` forwarder
    thread); the trait surface stays sync/channel-based and featureless
    builds stay tokio-free (`cargo tree` verified).
  - Auth per services.proto: `GrpcConfig { url, token }` sends
    `authorization: Bearer <token>` metadata on every RPC (`Debug` redacts
    the token). `https://` URLs use rustls with the platform's native roots.
  - Codegen stays protoc-free: `waddle-controlplane/build.rs` feeds a
    protox-compiled descriptor set to `tonic-prost-build` with `extern_path`
    mapping every message back to `waddle_types::pb::v0`, so only service
    glue is generated and exactly one copy of the wire types exists.
  - In-process integration tests (generated tonic server as the test plane):
    connect → auto-Register with bearer metadata, gate round-trip both ways,
    hard server kill → `Disconnected`, restart on the same port →
    re-register + in-order replay of messages buffered while offline.
  - Deps (all optional, behind the feature): tonic 0.14.6 + tonic-prost
    0.14.6 (the prost-0.14 pairing), tokio (rt/sync/macros), tokio-stream;
    build-deps tonic-prost-build + protox. tonic's `server`/`router`
    features ride the same feature solely for the in-process test plane
    (cargo cannot feature-gate dev-dependencies) — compile-time cost only.
- **waddle-runtime (`Session::publish_frame` — cameras are live) + tripwires
  evaluate real observations + `session.publish_frame` (Python)**: the
  biggest Milestone-A gap closes — declared cameras and tripwires actually
  do something.
  - `Session::publish_frame(camera, FrameData)` (`FrameData::rgb8(width,
    height, bytes)`, RGB8 only for now — typed as an enum so a future
    `Depth16` variant can land without breaking the constructor): validates
    `camera` against the robot's declared `cameras` (unknown → `Err`;
    declared but no media plane wired → a cheap `Ok(())` no-op — Local mode
    still records no video in v0), applies the declared
    `StreamPolicy.uplink` fps throttle (a wait-free atomic-timestamp check;
    a throttled frame is silently dropped, never an error, never counted),
    and enqueues onto a small (4-deep) per-camera bounded queue that
    drops the OLDEST frame under backpressure — counted, and surfaced via
    the new `Session::camera_frames_dropped(camera)`. Everything past the
    queue (the lazy, once-per-camera `publish_track` call; encode — raw
    passthrough for `RGB8`/`BGR8`/`JPEG`, the declared encoding being
    bandwidth-intent for the track rather than a literal wire format (see
    the Fixed entry below); `push_frame`) runs on one new dedicated
    `waddle-media-uplink` pump thread, never the customer's own thread. A
    declared `CAMERA_ENCODING_H264` uplink policy is a build-time error
    (`RuntimeError::UnsupportedCameraEncoding`) for any camera a wired
    media plane will actually publish — never a silent per-frame failure
    later.
  - `waddle-tripwire`'s `ObsSource` is no longer wired to an always-`None`
    stub: the reducer now publishes every gate tick's `obs` (the
    customer's `gate(obs=...)` argument) onto a wait-free `LatestSlot`
    (`waddle_ingest::LatestSlot`) as it drains the gate-record ring —
    unconditionally, whether or not local MCAP recording is on, and never
    touching `Gate::gate()`'s fast path. The flat customer vector maps onto
    `ObsSnapshot::joint_pos` verbatim, so a declared `JointLimitMargin` or
    `Staleness` tripwire now genuinely fires a HOLD (or whatever verb it
    declares) through dispatch; `WorkspaceAabb`/`ForceThreshold` still need
    a capture integration publishing structured `ee_pos`/`force_n`.
  - `session.publish_frame(camera, frame)` (PyO3): accepts a numpy `uint8`
    ndarray shaped `(height, width, 3)` (packed row-major RGB8); a
    contiguous array is copied once into the frame the core queues. A
    wrong dtype/rank/shape (or a non-contiguous array) raises `TypeError`;
    an unknown camera or a resolution mismatch raises `RuntimeError` (from
    the core). `waddle_sdk.init(..., media=waddle_sdk.LiveKit(url, token))`
    declares a real WebRTC-backed media plane. The heavy `livekit` Cargo
    feature (~690 MB webrtc-sys download, tokio) rides the `[teleop]`
    companion wheel rather than the default one, so a core built without
    it refuses `media=` with a clean `RuntimeError` naming the extra
    instead of degrading to a silent offline session (the `grpc` control
    transport, by contrast, IS in the default wheel — see the
    two-distribution entry above); `_testing=True` (the in-process
    loopback) is unaffected and is how `publish_frame` is exercised
    end-to-end in tests (`waddle_sdk._testing.frames(session, camera)`
    observes the far end).
- **waddle-media (real LiveKit `MediaPlane` behind the `livekit` feature)**:
  `livekit::LiveKitMedia` is the first real transport.
  `LiveKitMedia::connect(LiveKitConfig { url, token, track_resolutions })`
  spawns ONE dedicated thread (`waddle-media-livekit`) owning a private
  current-thread tokio runtime; all `MediaPlane` methods stay synchronous
  and forward over channels, so **tokio stays confined to this feature** —
  no tokio type crosses the public API and featureless builds have no
  tokio in the tree at all. `DataTopic` maps to LiveKit data-channel
  publishes on the normative `media.proto` topic strings with the
  normative reliability classes (TeleopPose/Telemetry lossy latest-wins,
  TeleopClutch/TeleopMark reliable ordered); inbound packets route by
  topic into the existing `DataRx` seam. `publish_track` publishes a
  native video track at the camera's declared resolution (default
  640x480); because LiveKit video sources consume RAW frames (libwebrtc
  encodes uplink itself, no pre-encoded JPEG accepted), `push_frame`
  accepts RGB8 (converted via `rgb8_to_i420`) or already-planar I420 —
  the JPEG encoder is for the data-channel/recording path. Feature-gated
  tests: a CI-safe unreachable-server test plus an `#[ignore]`d live
  end-to-end test driven by `WADDLE_LIVEKIT_URL`/`WADDLE_LIVEKIT_TOKEN`.
  Build note: with `--features livekit`, `webrtc-sys` downloads a
  prebuilt libwebrtc at build time (network on cold builds, ~690 MB
  extracted per target dir, ~30 s cold check); default builds unaffected.
- **waddle-media (real JPEG `VideoEncoder` + RGB8→I420 conversion)**:
  `JpegEncoder` (Motion JPEG over RGB8 via the pure-Rust `jpeg-encoder`
  crate; every frame a keyframe) joins `PassthroughEncoder` behind a new
  `VideoEncoding` selector (`make_encoder(encoding, width, height)`).
  `VideoEncoding::H264` stays a typed TODO — requesting it returns
  `MediaError::Unimplemented`, never a silent fallback. `rgb8_to_i420`
  converts RGB8 frames to planar I420 (BT.601 studio swing, 2x2
  block-averaged chroma, odd dims round chroma up) for raw-frame WebRTC
  video sources. `MediaError` gains `BadFrame`/`Encode`/`Transport`
  variants and `Unimplemented` now names what is deferred.
- **sdk (Python reset API: `TeleopReset`/`AgentReset`, `init`/`rollout`
  kwargs)**: the headline user-facing surface for the reset-phases branch.
  `waddle_sdk.TeleopReset(prompt, *, timeout_s=600.0)` and
  `waddle_sdk.AgentReset(prompt, *, timeout_s=600.0)` are small frozen,
  repr-friendly dataclasses declaring a remote reset window for a
  teleoperator/agent respectively (their docstrings name what a window
  needs to run: a connected supervision plane to grant and complete it —
  `waddle_sdk.init(transport=waddle_sdk.Grpc(url, token))` — since with no plane
  declared a window can only run out its timeout, and only the private
  `waddle_sdk._testing` reset-window hooks drive one without a plane).
  `waddle_sdk.init` gains
  `pre_reset=None`, `post_reset=None` (`None` | callable | `TeleopReset` |
  `AgentReset`) and `reset_verification="blocking"` (`"blocking"` |
  `"optimistic"`); `waddle_sdk.rollout(task, *, pre_reset=_UNSET,
  post_reset=_UNSET)` gains the same two kwargs with a module-level
  `_UNSET` sentinel distinguishing "inherit `init()`'s declaration"
  (`_UNSET`, the default) from "disable this phase for this one episode
  only" (explicit `None`) from "override it" (a fresh marker/callable).
  Callables are normalized **in Python** (`_normalize_reset_hook`) so the
  `_core` FFI always receives `(bool, Optional[bool])`: a bare `bool`
  return vouches for its own verification (`(ok, ok)`, matching the
  existing FFI-level default, now pure defense-in-depth underneath this);
  anything else — wrong arity, a non-bool first element, a second element
  that is neither `bool` nor `None` — raises `TypeError` naming the
  contract. That `TypeError` is diagnostic only: `PyResetHook::call`
  (Rust) catches every exception from the hook callable, including this
  wrapper's own, and reports it solely via `sys.unraisablehook` before
  normalizing to `(False, None)` — the `rollout()` caller sees the same
  generic `RuntimeError: reset failed` as a hook that legitimately
  returns `False`, and cannot tell the two apart from that exception
  alone. `waddle_sdk._testing` gains `reset_window_engage`/
  `reset_window_complete` thin wrappers alongside the existing
  `engage`/`release`/`push_teleop` (they deserved the same wrapper
  treatment). `rollout`'s docstring now
  documents the post-reset exit contract: `ep.done` flips to `True` at
  POST_RESET entry (before cleanup finishes); the ordinary
  `ep.terminate(...)` call already blocks the `with`-exit through it
  (unchanged); a `with` block that exits some other way while
  POST_RESET is still running finds `__exit__` already a no-op (it never
  aborts, or otherwise touches, an in-flight post-reset); a failed
  post-reset never changes the pinned outcome, only
  `ep.post_reset_failed`. `init`'s docstring documents
  `reset_verification` and the remote-window build-time-negotiation
  narrowing rule. `sdk/README.md`'s hollow-frontend checklist gains a
  Reset API bullet: markers/callables are pure type dispatch and
  input-shape validation, never reset decisions — every actual behavior
  stays in waddle-core.
- **sdk (PyO3 shim: reset kwargs, `PyResetHook`, testing hooks)**: the
  `_core` module surface now exposes the full reset-config vocabulary.
  `create_session` gains `{pre,post}_reset_kind` (`"none"`|`"hook"`|
  `"teleop"`|`"agent"`), `{pre,post}_reset_hook`, `{pre,post}_reset_prompt`,
  `{pre,post}_reset_timeout_ns` (default 600s), and `reset_verification`
  (`"blocking"`|`"optimistic"`) — all defaulted for full back-compat,
  mapping onto `SessionBuilder::pre_reset`/`post_reset`/`verification_mode`.
  `PySession::start_episode` gains the same eight kwargs as per-episode
  overrides (`None` = inherit the session default) → `start_episode_with`.
  A Python callable crosses as a `PyResetHook` (`sdk/rust/src/verbs.rs`,
  the `PyUnit` GIL/shutdown pattern): it normalizes a bare `bool` return to
  `(bool, Some(bool))` (a hook with no separate verification opinion is
  read as vouching for its own `ok` — otherwise a bare `True` would hang
  forever in RESETTING under the default Blocking verification mode,
  which requires `verified = Some(true)`), passes an explicit
  `(bool, Optional[bool])` tuple through as-is, and — for anything else
  (a raised exception, or a return value of neither shape) — reports it
  via `PyErr::write_unraisable` (CPython's "log, don't propagate" hook for
  background-thread callbacks) and normalizes to `(false, None)`; the hook
  never panics or unwinds into Rust. `PyEpisode` gains a `post_reset_failed`
  getter (mirror read); `done`'s docstring documents the POST_RESET flip;
  `outcome` now reads `status().outcome.or(status().pinned_outcome)` so it
  returns the pinned value (not `None`) once `done` flips true at
  POST_RESET entry, matching `waddle_runtime::Episode::outcome()`'s own
  contract without touching the episode's inner mutex.
  Two new `_testing`-gated hooks (`testing_loopback=True` only, following
  the existing `_testing_engage`/`_testing_push_teleop` pattern):
  `_testing_reset_window_engage(claim_id, actor)` and
  `_testing_reset_window_complete(claim_id, ok, verified=None)` inject the
  window `SessionEvent`s directly (mirroring the exact `ClaimGranted` +
  `ResetWindowEngage` / `ResetWindowComplete` sequences
  `forward_server_msg`'s plane ENGAGE/COMPLETE arms produce), backed by two
  new `waddle-runtime` convenience functions (`reset_window_engage`,
  `reset_window_complete`, alongside the existing `grant_and_engage`/
  `release_claim`) so the shim never mints its own clock stamps. Verified
  (not changed): the reset pump's shutdown ordering — it checks
  `mirror.status.shutdown` at the top of its loop exactly like the bypass
  pump, `Session::shutdown()` sets that flag before joining any thread, and
  `PySession::shutdown`/`Drop` already run the blocking join with the GIL
  detached — so a `PyResetHook`'s `Python::try_attach` on the pump thread
  can never deadlock against a Python caller holding the GIL during
  interpreter teardown.
- **waddle-runtime (reset-window actuation + plane directives)**: the
  bypass pump (`pumps::spawn_bypass_pump`) gains a RESET arm — while the
  mirror shows `GateMode::Reset` with an active claim, due intervention
  actions (teleop via the existing media intake, agent chunks via the new
  plane arm below) are driven straight to `send`, identical mechanics to
  the BYPASS arm (provenance from the mirror, same chunk shape), no stall
  detection. `forward_server_msg` now handles
  `GateServerMessage.reset_window` (flag `waddle.v0.reset.remote`): ENGAGE
  injects `ClaimGranted` (from the directive's claim) then
  `ResetWindowEngage`; COMPLETE injects `ResetWindowComplete{ok, verified}`
  from the attached result; CANCEL injects `ResetWindowComplete{ok:false}`
  (no dedicated FSM event exists for a plane-initiated cancel — it is
  observably the same as a failed completion). `forward_server_msg` also
  handles `GateServerMessage.intervention_chunk` while the mirror shows
  `GateMode::Reset`: the chunk's steps (dims-validated via
  `ActionChunk::from_pb`) join the intervention ring as timed actions,
  keyed off this arrival plus each step's declared offset; every other
  gate mode still drops this arm silently (the general Claimed-mode chunk
  intake — jitter horizon, `ReplanPolicy` — remains a later milestone).
  The intervention ring's single write end is now Mutex-shared
  (`StreamProducer`) between the media-intake thread and the plane pump,
  since `rtrb` is strictly SPSC and both now need to push. Proven
  end-to-end over a real `ControlPlaneClient` + `InMemoryTransport` script
  (not direct FSM injection): a remote PRE window engaging over teleop and
  completing to READY; a remote POST window engaging an agent, dispatching
  an `intervention_chunk`, and completing to `Terminal`; a POST window
  that is never engaged, timing out for real (short `timeout_ns`, no
  `TimerFired` shortcut) to `Terminal{pinned}` + `post_reset_failed`; the
  same window-timer slot reused correctly across a PRE-then-POST window
  pair inside one episode; and a born-claimed retake successor confirmed
  to never open a remote pre-window even when the session default is
  `Remote`. An MCAP read-back confirms the actuation lands on
  `/waddle/actions` (as the caller-tick's `RESET_ACTIVE` `NoopMarker`,
  tagged with the claimant's provenance — the gate's per-tick record
  remains the only writer onto that topic; the bypass pump's direct `send`
  dispatch is a separate verb call, not itself an MCAP record, unchanged
  from BYPASS).
- **waddle-runtime (reset pump + post-reset recording)**: a new core thread,
  `waddle-reset-hooks` (`pumps::spawn_reset_pump`, mirror-watch like the
  bypass pump), is the single scripted-hook invocation site for resets: a
  LIVE episode in RESETTING that no `start_episode_with` call is driving
  inline gets the effective PRE hook run there (session/per-episode config;
  the trivial `(true, Some(true))` default when none) and its `ResetResult`
  injected, and a LIVE episode in POST_RESET whose effective POST spec is a
  `Hook` gets that hook run there and its `PostResetResult` injected
  (E15/E16) — so blocking `terminate` now completes on post-reset-declared
  episodes. `Remote` specs are untouched by the pump: the FSM's window
  machinery owns them, including the timeout. `start_episode_with` publishes
  the episode's resolved specs (a new internal slot, written before
  `EpisodeOpen`) so per-episode overrides are honored by the pump; hooks run
  off the caller thread (the `ResetHook` type already requires
  `Send + Sync`) and must return — shutdown joins the pump. The mirror
  `Status` gains `pinned_outcome` and `post_reset_failed`, and the sidecar
  now carries the full post-reset record: `post_reset_declared` (stamped
  from `EpisodeOpen`), `post_reset_failed`
  (`Effect::SetPostResetFailed`, permanent, never alters the outcome),
  `post_reset_result` (derived from the emitted `PostResetResult` event),
  and `post_reset_bounds` (opens at the →POST_RESET transition, closes at
  →TERMINAL; left open if force-finalized mid-cleanup).
  `Effect::RunPostReset` is a documented reducer no-op — the mirror-watch
  pump sees the same transition, and user hooks must never run on the
  reducer thread. A Reset-mode gate tick's RESET_ACTIVE `NoopMarker` on
  `/waddle/actions` (wired earlier, untested) is now pinned by an
  end-to-end remote-post-window test.
- **waddle-runtime (reset config surface)**: the first runtime seam for
  reset phases (`waddle-core/crates/waddle-runtime`) — `ResetSpec { Hook(ResetHook) |
  Remote { actor, prompt, timeout_ns } }`; `SessionBuilder::pre_reset`/
  `post_reset` (declaring `post_reset` at all — either variant — is what
  makes an episode detour through `Phase::PostReset`, FSM.md row E14) and
  the previously-missing `verification_mode` setter; `reset_hook` stays as
  an alias for `pre_reset(ResetSpec::Hook(hook))`, now `#[deprecated]` since
  no internal caller exists anywhere in the workspace that would need
  migrating first. `EpisodeOptions {
  pre_reset: Option<Option<ResetSpec>>, post_reset: Option<Option<ResetSpec>> }`
  (outer `None` inherits the session default, inner `None` disables that
  phase for this episode only) plus `Session::start_episode_with`, with
  `start_episode` now a thin default-options delegate. `start_episode_with`
  resolves the effective pre/post specs and injects them onto
  `EpisodeOpen`; a `Hook` (or no spec at all) runs inline on the caller
  thread exactly as before; a `Remote` pre-spec skips the hook/`ResetResult`
  injection entirely and lets the FSM's window machinery (rows E19–E22)
  drive RESETTING to READY or Terminal on its own — no runtime-side
  timeout is added, the FSM window timer owns it. New
  `inline_reset_owner: Mutex<Option<EpisodeId>>` on `SessionInner`, set
  before `EpisodeOpen` for every inline pre-reset path and cleared when the
  call returns, for the reset pump (a later task) to consult so it never
  double-services an episode `start_episode_with` already handled. New
  guard: a predecessor episode that has reached `Phase::PostReset` (its own
  cleanup, past the pinned outcome) is waited out to Terminal and opened
  over instead of erroring `EpisodeActive` — POST_RESET self-resolves, so
  back-to-back rollouts started without an explicit `terminate` + wait no
  longer race the guard. The `Register` feature-flag declaration always
  includes `waddle.v0.reset` (alongside the existing unconditional
  `waddle.v0.core`) and adds `waddle.v0.reset.phases`/`.remote` whenever the
  session-level config declares a matching spec; per-episode `Remote`
  overrides can only narrow what the session already declared, never widen
  it (documented on `EpisodeOptions`, not runtime-enforced — the simpler
  sound option). The reset pump (the actual hook
  invocation for post-reset, and the successor-episode fix for
  reducer-opened retakes), the RESET bypass-pump arm, and
  `forward_server_msg` window handling are explicitly out of scope here —
  reducer/mirror fields are untouched.
- **waddle-protocol (reset-phases vocabulary, inert)**: two new feature
  flags, `waddle.v0.reset.phases` and `waddle.v0.reset.remote` (registered
  in `VERSIONING.md`), gate the wire vocabulary for pre/post-reset phases
  and remote reset windows: `EPISODE_STATE_POST_RESET`, `GATE_MODE_RESET`,
  `ResetKind`, `PostResetResult`, `ResetWindowEvent`/`ResetWindowEventKind`,
  `ResetWindowDirective`/`ResetWindowDirectiveKind`, `EpisodeEvent` arms 16
  (`post_reset`) and 17 (`reset_window`), `GateServerMessage.reset_window`
  (arm 6), `Sidecar` fields 32-35 (`post_reset_declared`,
  `post_reset_failed`, `post_reset_result`, `post_reset_bounds`), and
  `NOOP_REASON_RESET_ACTIVE` — purely additive on the wire; nothing emits or
  reads any of it yet. `waddle-types` mirrors: `EpisodeStateKind::PostReset`,
  `GateMode::Reset`, new `ResetKind { Pre, Post }`, with pb round-trip
  conversions and a unit test per new enum. Every exhaustive match this
  touched across `waddle-fsm`/`waddle-runtime`/`waddle-conformance`/
  `waddle-sidecar` gained an inert arm (behavior unchanged; the FSM/gate/
  runtime behavior for these flags lands in a later change on this branch).
- **waddle-fsm (POST_RESET phase + remote reset windows)**: the FSM now
  implements FSM.md rows E14–E22 and C6/C7 behind the reset-phases flags. An
  episode that declares a post-reset runs a cleanup pipeline INSIDE the
  finishing episode: the terminal outcome is pinned at POST_RESET entry (E14)
  and never changes — a post-reset failure only sets the permanent
  `post_reset_failed` flag (E16), and an estop during cleanup keeps the pinned
  outcome rather than flipping an earned SUCCESS to ABORT (E17). A late
  terminate is rejected and a late END_* mark records the mark without
  transitioning (E14b). Remote reset windows (E19–E22) let a plane-directed
  actor perform a scene reset through the SDK: a window opens in RESETTING
  (pre) or POST_RESET (post), a reset claim is admitted with an actor check
  (C6: a TELEOPERATOR window also admits SITE_OPERATOR, an AGENT window admits
  AGENT only), the claimant engages (lease → claimant, gate → RESET, E20), and
  on completion the lease hands back to the loop client BEFORE the pipeline
  result applies (E21, the deferred-apply invariant), releasing the reset
  claim (C7); a deadline aborts (pre) or pins + flags (post) (E22). The
  central run-closing block is factored into `close_run` (shared by terminal
  and post-reset entry, byte-identical for undeclared episodes) and the E10
  trigger set routes through `request_terminal`, which detours to POST_RESET
  only when declared. New `SessionEvent`s (`PostResetResult`,
  `ResetWindowEngage`, `ResetWindowComplete`), `EpisodeOpen` fields
  (`post_reset`, `pre_window`/`post_window`), `TimerId::ResetWindowTimeout`,
  `AfterLease::{ResetEngageComplete, ResetHandback}`, and
  `Effect::{SetPostResetFailed, RunPostReset}`; the reducer-side handling of
  the new effects stays inert (runtime reset seams land in a later change).
  Undeclared episodes behave exactly per E1–E13 (the additive guarantee); all
  13 conformance fixtures stay byte-identical green.
- **waddle-fsm proptests (I9–I14) + waddle-gate `PlanMode::Reset`**: the
  random-walk harness (`tests/properties.rs`) now drives POST_RESET and
  remote reset windows too — `Cmd` gained `OpenPostReset` (varying pre/post
  window declarations), `PostResetOk`/`PostResetFail`, `WindowEngage`,
  `WindowComplete{ok}` — and checks six new invariants: I9 (PostReset ⇒
  declared), I10 (`pinned_outcome` set-once; PostReset is followed only by
  TERMINAL{pinned}, including via estop), I11 (estop from PostReset ⇒
  TERMINAL ∧ lease Vacant), I12 (`post_reset_failed` monotone; false at
  TERMINAL ⇒ the last post-reset result was ok), I13 (gate RESET ⇒ an active
  claim ∧ phase ∈ {RESETTING, POST_RESET}), I14 (retake acceptance ⇒
  TERMINAL{ABORTED_RETAKE} with no intervening POST_RESET). `Cmd` also gained
  `GateTick`, directly proptesting D7 edge 3: a gate tick landing in
  RESETTING/POST_RESET must never transition the phase (only the READY→RUNNING
  first-gated-action trigger, E6, may). A new deterministic smoke test drives
  a full remote POST-window lifecycle, asserting E21's deferred-apply
  emission order. `waddle-gate` gains
  `PlanMode::Reset { provenance }` (mirroring `Bypass`): `Gate::gate()`
  returns `Noop` and records the new `GateDecision::ResetActive`, same cost
  class as the existing NOOP paths (no locks/syscalls/allocations); the
  runtime reducer wires `GateMode::Reset` to it and renders
  `NoopReason::RESET_ACTIVE` distinctly from `BYPASS_ACTIVE` — the D7 edge 3
  stale-handle protection (a caller ticking `gate()` while a remote actor
  resets dispatches nothing).
- **Bug fix (found by proptest I13) — FSM.md row E19b**: `reset_result` /
  `post_reset_result` are now rejected while a remote reset window is open
  (`waddle-fsm`). Previously the pipeline-hook completion path (E2–E5 /
  E15–E16) could land while a window was OPEN or ENGAGED, abandoning the
  window/claim/lease bookkeeping and leaving `gate_mode == RESET` stuck
  alongside a phase that had already moved past RESETTING/POST_RESET. Not
  reachable through a config-correct runtime (`ResetSpec` is `Hook` XOR
  `Remote`; the reset pump skips hook injection for `Remote`, D4) but guarded
  in `waddle-fsm` anyway per the hollow-frontend rule. Two regression tests
  pin it in `tests/remote_reset_windows.rs`; `docs/FSM.md` §1.4 gains the
  E19b row. New conformance fixture `remote_window_owns_pipeline_result`
  (`fixtures/behaviors/`) asserts the guard on both the pre- and post-window
  path; it is currently runner-skipped (needs `waddle.v0.reset.phases` +
  `waddle.v0.reset.remote`, neither implemented by `waddle-conformance`
  yet — a D6 conformance-runner task) and will activate once that lands.
- **Conformance coverage for reset phases + remote reset windows**
  (`waddle-conformance`): the runner now implements scenario-format.md's
  reset-phase vocabulary — `SUPPORTED_FEATURES` gains
  `waddle.v0.reset.phases`/`waddle.v0.reset.remote`; `episode_open` parses
  the optional `post_reset`/`pre_reset_window`/`post_reset_window` keys; the
  new inject kinds `post_reset_result`, `reset_window_engage`,
  `reset_window_complete`; the state-snapshot document gains
  `episode.post_reset_declared`/`post_reset_failed`/`pinned_outcome` and the
  top-level `reset_window` document; `GateMode::Reset` now maps to
  `waddle-gate`'s real `PlanMode::Reset` instead of a Passthrough
  placeholder. Activating the flags also brought the previously
  runner-skipped `remote_window_owns_pipeline_result` fixture (added above)
  online; running it for the first time surfaced an emission-cursor
  authoring gap (three legitimate transitions never explicitly consumed via
  `expect_emission`, so a later `expect_no_emission` tripped on one of them)
  — fixed by adding the missing assertions, with no change to the guard it
  pins. Nine new fixtures added (`fixtures/behaviors/`), covering FSM.md
  rows E14–E22 and C6/C7: `post_reset_happy`, `post_reset_failure_flags`,
  `post_reset_skipped_when_undeclared` (core-only, the additive guarantee),
  `estop_during_post_reset` (E17), `retake_skips_post_reset` (E18),
  `post_reset_from_intervention` (E14 from INTERVENTION),
  `remote_pre_reset_claim_engage_complete` (the full E19→C6→E20→E21 flow,
  emission order asserted), `remote_post_reset_timeout` (E22), and
  `remote_reset_wrong_actor_denied` (C6). 23/23 scenarios pass (13 original +
  the newly-activated `remote_window_owns_pipeline_result` + these 9).
- New conformance fixture `teleop_dims_mismatch_holds` (`waddle-conformance`,
  gate target): pins the media-intake action-space-validation contract as
  gate-observable — a teleop injection whose flattened width doesn't match
  the declared action space is never dispatched, `gate_tick` returns hold
  however many mismatched packets arrive in the blend window, exactly one
  `Fault{FAULT_KIND_VALIDATION_ERROR}` fires per claim window, and a
  subsequent dims-correct packet still substitutes normally. FSM.md §5
  (IMMEDIATE{blend_ns}) now states the dims-mismatch contract explicitly.
- **waddle-core (obs logging)**: `gate()` now takes the observation the
  caller computed its action from (`obs: Option<&[f64]>`) and records it on
  every decision arm — Pass records are the training pairs;
  Substitute/Blend records are pre-labeled DAgger pairs. New
  `waddle_types::ObsValues` (inline to 32 dims; wider observations spill to
  the heap, never truncate); `GateRecord.obs`. The alloc-free proof now
  covers a 30-dim obs; new `gate_passthrough_14dof_obs30` bench.
- **waddle-core (gate-record persistence)**: the reducer now drains the
  per-episode gate-record ring every wake and persists it to the Local-mode
  MCAP — the obs as `ObservationUpdate` on the new `/waddle/observations`
  topic, the decision as a single-step `ActionChunk` on `/waddle/actions`
  (Noop/Hold write `NoopMarker` actions, making the topic the complete
  per-tick trace). "The reducer owns all recording" is now structural. New
  `waddle_types::unflatten_action` (exact inverse of the flattening path)
  and `ProvenanceTag::to_pb`; `McapEpisodeWriter::write_observation`.
- **`sdk/` — the Python `waddle-sdk` frontend (Tier-1 minimum)**: the
  six-line tutorial loop against a real customer loop with Local recording,
  fully offline (no control plane, no relay). PyO3 0.29 shim binding
  `waddle-runtime` directly (abi3-py310, own cargo workspace with path-deps
  into waddle-core), `uv`/maturin packaging. Public surface: `init` /
  `rollout` / `shutdown`, `Control` (five verb callables → derived grants),
  `Handoff`, `Outcome`, and pure-Python descriptor sugar (`Robot`,
  `JointSpace`, `EEDelta`, `Composite`, `Opaque`, `Camera`, `Chunking`,
  `Gripper`) compiling to canonical proto3 JSON. `ep.gate(action, obs)`
  returns "what you should send, or None if you must not send" (Pass
  returns the caller's exact object; Noop/Hold return None); exiting
  `rollout()` non-terminal aborts, never succeeds. Private `waddle_sdk._testing`
  hooks (engage/release/push_teleop over the loopback media plane) drive the
  intervention pytest; the nominal pytest reads the episode MCAP back as the
  Python-side proof of obs logging + gate-record persistence. New core
  helper `waddle_runtime::release_claim` (counterpart of
  `grant_and_engage`).
- **waddle-core M2 (`waddle-fsm`)**: pure Mealy session machine (episode ×
  claim × lease × grant health) implementing every FSM.md guard row — reset
  verification modes (N12), retake → born-claimed successor under the
  surviving claim (N2/N18), per-policy handoff sub-protocol with delta-space
  degradation, clutch self-initiated claims, estop revoke-all, grant liveness
  with hysteresis and never-mid-lease demotion (N11), dual-write hold
  requests (N14), FSM-owned bypass transitions. 256-case random-walk proptest
  holding eight invariants.
- **waddle-core M3 (`waddle-ingest`, `waddle-gate`, `waddle-tripwire`)**:
  SessionClock (sole OS-clock reader) + FakeClock + per-source offset
  estimation; the gate fast path (ArcSwap plan + SPSC stream + jitter buffer
  + blend math + NOOP bypass + DivergenceDetector), proven allocation-free
  over 1M passthrough calls; tripwire evaluator + heartbeat watchdog on
  dedicated threads, edge-triggered, verbs requested never enforced.
- **waddle-core M4–M5 (`waddle-codecs`, `waddle-sidecar`)**: codec
  trait/registry with version pinning, mandatory round-trip certification and
  signing seam (N4/N15) + lerobot-async/openpi dialects; sidecar records as
  wire-exact canonical JSON (prost-reflect over the embedded descriptor set),
  span derivation from the event stream, atomic writes + manifest, Local-mode
  MCAP recorder with clock-anchor metadata, Reference-mode resolver seam.
- **waddle-core M6 (`waddle-controlplane`, `waddle-media`)**: control-plane
  client thread (backoff reconnect, in-order offline replay, N11 heartbeat
  proxy signals, N7/N13 negotiation) over a scriptable in-memory transport;
  MediaPlane trait + loopback with the media.proto topic table.
- **waddle-core M7 (`waddle-conformance`, `waddle-runtime`)**: the
  behavioral-scenario runner implementing `conformance/scenario-format.md`
  exactly (canonical-JSON matching via prost-reflect, virtual time, FSM and
  gate targets with a reference bypass pump) — **all 12 protocol scenarios
  pass with zero changes to waddle-fsm/waddle-gate**, plus mutation tests
  proving the runner detects wrong values/order/forbidden emissions; the
  runtime Session/Episode API — five-verb dispatch thread (serialized,
  catch_unwind, estop priority path), single-writer FSM reducer interpreting
  effects, per-episode sidecar + MCAP finalization, blocking-through-reset
  episode open, bypass pump, media intake, plane pump, ordered shutdown.
  e2e: nominal recording, teleop engage/substitute/release, and the
  claimed-while-stalled NOOP-spectator contract.
- **waddle-core M8 (`waddle-ffi` → libwaddle, `xtask`)**: the C ABI — opaque
  handles, pb-bytes configuration, five-verb C callbacks invoked only on the
  dispatch thread, status codes + thread-local `waddle_last_error`,
  panic-proof entry points; `cargo run -p xtask -- gen-header` emits
  `target/include/waddle.h` (marked `WADDLE_ABI_UNSTABLE` per N5); verified
  by Rust round-trip tests and a real C caller compiled with gcc against the
  generated header and linked to `libwaddle.so`.
- **sdk (descriptors: intrinsics, stream policy, URDF, frame graph, joint
  limits)**: `sdk/python/waddle_sdk/descriptors.py` widens to cover the rest of
  `descriptors.proto`'s declaration surface (shape only — the hollow-frontend
  rule: no new semantic validation, `RobotDescription::try_from` remains the
  one semantic validator). `Camera` gains optional `intrinsics: Intrinsics`
  (`fx, fy, cx, cy`, `distortion_model` — short names, defaults to
  `"unspecified"` — `distortion: tuple[float, ...]`, `depth_scale_mm`),
  optional `stream_policy: StreamPolicy` (`local_full_rate: bool`, optional
  `uplink: Uplink(fps, encoding, max_kbps)`, compiling to the `stream` wire
  field), and `vendor: dict[str, str]`. `Robot` gains optional
  `kinematics_urdf: bytes | str | Path` (`bytes` passes through as-is;
  `str`/`Path` is read from disk **at compile time** — pick one, document
  it, no silent XML-vs-path guessing), `frames: tuple[FrameTransform, ...]`
  (new dataclass: `parent`, `child`, `position` (x, y, z), `quaternion`
  — **wxyz**, pinned by a dedicated non-symmetric test — compiling to a
  `FrameGraph`; the nested `Pose.frame_id` is filled from `parent`, the
  frame the transform's numbers are expressed in), and `series: dict[str,
  TimeSeries]` (`dtype`, `shape`, `units`, `frame_id`, `rate_hz`).
  `JointSpace.joints` and the new `Gripper.dexterous(joints)` both now
  accept either a bare name (names-only form, unchanged) or a new `Joint`
  dataclass (`min_position`, `max_position`, `max_velocity`, `max_effort`)
  via a shared `_compile_joint` helper. Validation stays minimal and
  objective: `min_position <= max_position`, `max_velocity`/`max_effort`
  `>= 0`, `fps > 0`, `max_kbps > 0`, `depth_scale_mm > 0` — everything else
  is shape-only and deferred to waddle-core. Back-compat is a golden assert:
  descriptors that set none of the new fields compile to the exact same
  dict as before.

### Changed
- **`examples/toy_robot.py` runs its background loop on the shipped pump**
  (`waddle_sdk.robots.base.RobotPump`) instead of a class of its own. The example's
  copy and the one a robot module used were the same monotonic-deadline thread
  written twice, which is how the two start drifting; the pump knows nothing
  about arms, so the example hands it its own `robot_tick` and keeps every
  other line. Nothing about the program's behaviour or its status lines
  changes, and the agent-mode path that only this loop keeps alive is now
  covered: a test drives the example's own `run_agent_mode` with the caller
  blocked inside `waddle_sdk.agent()` and asserts the camera keeps publishing.
- **An anchorless cross-fade now emits the commanded setpoint exactly**
  (`waddle-gate`): with nothing yet commanded (an episode whose caller has
  never ticked — every agent-invited one), a whole-robot action is its own
  endpoint and crosses the blend window unchanged. It always crossed, but the
  previous code got there by anchoring the missing `from` ON the target and
  interpolating, and `x * (1 - w) + x * w` is `x` only up to rounding: over
  uniformly sampled joint values and weights, ~1.5% of pairs came out one ULP
  off the value the sender asked for. The anchorless arm now returns the
  target itself, so the first commanded setpoint of such an episode is the
  one that reaches the robot, bit for bit, rather than one that merely looks
  like it in a printout.
- **sdk (`waddle-sdk`) — a `Composite` session's intervention payloads are
  dict-by-part**: `episode.gate(...)`'s Substitute/Blend return and a
  dispatched `Chunk`'s step values are a `dict[str, ndarray]` keyed by
  declared part where they were a flat float64 ndarray. Source-breaking for a
  customer whose declaration is `Composite` and who receives interventions;
  **every other declaration is untouched**, and Pass (your own object) and
  Noop/Hold (`None`) are unchanged everywhere. The old shape could not
  express what the wire had already been able to say since v0 — an action
  addressing one arm — so a bimanual customer's only alternative was to read
  a 7-row command as a 14-row one. `sdk/tests/test_bimanual.py` pins both
  halves, the changed one and the unchanged one.
- **Public types (pre-1.0 / API unstable per N5) — a claim's actor and a
  custom provenance are now SHARED, not owned**: `ActiveClaim.actor` is an
  `Arc<ActorRef>` where it was a bare `ActorKind` (a claim now carries who
  holds it whole — kind and the identity the granting side stamped),
  `ProvenanceTag.actor` is `Option<Arc<ActorRef>>`, and
  `Provenance::Custom` carries an `Arc<str>` where it carried a `String`.
  Source-breaking for anything that constructs or destructures them. The
  `Arc`s are not decoration: the gate clones the active tag twice per tick
  on the customer's real-time thread, so nothing owned may live on it —
  the identity is minted once per claim, off that thread (see the
  per-tick-allocation regression under Fixed for the measurement).
- **`Session::episode_done` / `Episode::done` flip at `Phase::PostReset`**,
  not only at Terminal: the terminal outcome is pinned at POST_RESET entry
  (E14), so the rollout is over from the caller's view while only the scene
  cleanup (which self-resolves) is still running. Consequences:
  `terminate_episode`/`Episode::terminate` are now no-ops during POST_RESET
  (a teardown path — e.g. a context-manager `__exit__` racing a plane
  directive — can no longer inject a second Terminate against a pinned
  outcome), and `Episode::outcome()` returns the pinned outcome while the
  cleanup runs (the same value the eventual →TERMINAL carries). A terminate
  that itself detours through POST_RESET still blocks to Terminal,
  unchanged.
- **`FSM.md`** gains §1.3 "Post-reset" (flag `waddle.v0.reset.phases`,
  guard rows E14-E18 + E14b) and §1.4 "Remote reset windows" (flag
  `waddle.v0.reset.remote`, guard rows E19-E22), plus claim-lifecycle rows
  C6/C7 (§2) and two gate-mode-table rows (PASSTHROUGH↔RESET). No FSM
  behavior changes: these rows are prose/normative only in this change: the
  9 fixtures that pin them, and the FSM implementation itself, land together
  in a later change on this branch (the repo rule that guard rows +
  fixtures + a green runner land in one change is satisfied at the
  branch level, not this commit). **`conformance/scenario-format.md`**
  gains the `post_reset_result` / `reset_window_engage` /
  `reset_window_complete` inject kinds, `episode_open`'s optional
  `post_reset?`/`pre_reset_window?`/`post_reset_window?` keys, state-
  snapshot additions (`episode.post_reset_declared`,
  `episode.post_reset_failed`, `episode.pinned_outcome`, top-level
  `reset_window`), and effects-vocabulary additions (`GATE_MODE_RESET`,
  `set_flag{post_reset_failed}`, `arm_timer{reset_window_timeout}`) — all
  gated by the same two flags, documenting what the conformance runner will
  implement in a later task on this branch.
- **Golden fixture amendment (pre-release; no tagged versions exist) —
  `handoff_immediate_mid_chunk`**: its teleop packets now carry Pose
  targets for both parts (7+7=14 values) to stay dims-consistent with the
  declared bimanual composite, instead of single-part Twist packets. The
  fixture previously reached its `blend` expectation only via the
  since-fixed silent dim-truncation defect (see the media-intake fix
  below) — it pinned the defect, not intended behavior. This is a
  deliberate, documented exception to the append-only-goldens rule, made
  possible only because no version has shipped yet; the other 11 existing
  fixtures are untouched.
- **Signatures (pre-1.0 / ABI unstable per N5)**: `Gate::gate`,
  `Episode::gate`, and the C ABI `waddle_gate` gained the obs parameter
  (`obs`/`obs_len` on the ABI; NULL or 0 = no observation). Header
  regenerated.
- Six behavioral fixtures aligned to implementation emission order where
  FSM.md deliberately does not pin intra-step order (each documented in its
  fixture description); `backend_partition_degradation` now asserts buffer
  counts + reconnect re-promotion (transport replay is waddle-controlplane's
  tested contract).
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

### Removed
- `Episode::drain_records` and the episode-held record consumer: gate
  records now flow to the reducer (via an internal hand-off slot) and land
  in the episode MCAP; callers no longer see the ring.

### Fixed
- **A recording directory that does not exist yet no longer costs the entire
  archive** (`waddle-runtime`, and every binding above it): every file the
  recorder writes — the per-episode sidecar, the per-episode MCAP, the
  appended `manifest.jsonl` — is created INSIDE `recording_dir`, and each of
  those opens was fallible-and-swallowed. So a program that passed
  `recording_dir="recordings"` from a working directory where no such
  directory existed opened a session, streamed, drove episodes to their
  terminal outcome and left NOTHING on disk, with no error on any path. The
  shipped five-line example is exactly that program.
  `SessionBuilder::build` now creates the directory (parents included) and
  proves it writable by opening the manifest there; a path nothing can make a
  writable directory at — an existing file, a read-only parent — is the new
  `RuntimeError::RecordingDirUnusable`, naming the path. Same family as the
  camera-encoding check next to it: a wiring mistake fails at build time
  instead of silently at every episode. The local recorder holds the
  full-rate archive, so it may not quietly hold nothing.
- **A step-cap refusal now says which of its two rates is the cap**
  (`waddle_sdk.robots.base.Arm`): the line read `joint1 would move 1.3000 in one
  command, cap 0.1000 (at 10 Hz that is 13.000 per second)`, where the 13.000
  is what the command ASKED for — sitting immediately after the cap, where it
  reads as the cap's own allowance. The declared cap at 10 Hz is 1.0 per
  second. Both numbers are now named as what they are: `cap 0.1000 (1.000 per
  second at 10 Hz); this asks for 13.000 per second`. An arm with no declared
  `rate_hz` still says only the per-command pair rather than inventing a
  cadence.
- **The five-line example no longer describes an offline mode it does not
  have** (`examples/yam_bimanual.py`): its docstring promised that with no
  transport "the twins move, both parts report, and every episode lands in
  the recording directory", one sentence away from the exit it actually
  takes. Those five lines end in `waddle_sdk.agent()`, so with no transport the
  program says what it needs and exits 2 before a session opens — nothing
  steps and nothing is recorded. A rig needs no plane in general
  (`rig.session(...)` without one is a local recorder a program drives from
  its own loop); the file and `examples/README.md` now say which of those is
  which.
- **A negotiated flag no longer outlives the connection that gave it**
  (`waddle-runtime`): `Status.parts_negotiated` and `Status.stills_negotiated`
  (and the plane pump's own `acks_negotiated`) say what the CURRENT connection
  accepted, but were only ever WRITTEN, at `Registered` — never cleared. So
  across a partition the reducer kept minting named-part `ProprioSample`s
  under a dead connection's answer, the uplink kept sending control-plane
  stills to nobody, and the chunk intake kept an `Action.part` policy the next
  plane had not agreed to. A connection that has ended has accepted nothing,
  and one that has not registered yet has accepted nothing either: all three
  are now forgotten at every connection boundary. Pinned by
  `named_part_samples_never_survive_a_partition`, which fails without it.
- **A flag-scoped message never crosses a connection that did not accept its
  flag** (`waddle-controlplane`): the other half of the same hole, and the
  half that survives a reconnect. `ClientMsg::connection_scoped_flag` is now
  the one place that answers "is this message's content legal only under a
  negotiated flag?" — a named `ProprioSample.part` (`waddle.v0.parts`), a
  `FrameStill` (`waddle.v0.obs.stills`), a `DirectiveAck`
  (`waddle.v0.plane.acks`) — and, like `is_droppable`, it binds at every point
  the message could escape: filtered on the way out against the current
  `RegisterResponse`, and kept out of the offline buffer entirely, because
  that buffer replays onto the NEXT connection and replays right after
  Register, before that connection has said what it accepts. A `DirectiveAck`
  produced during a partition was therefore replayed at a plane that had never
  asked for acks, and per-part proprioception — non-droppable history, unlike
  a still, so it BUFFERS — became a queue of one arm's joint vectors handed to
  a plane that had just refused `waddle.v0.parts`. Holding them until the new
  answer arrives is not an option either: history replays in order, and a
  partial hold reorders the stream it belongs to. Withholding is not data
  loss — the local recorder keeps the full-rate archive, part and all. The
  flag names now live once, in `waddle_controlplane::flags`.
- **A claim window is owed its own refusal** (`waddle-runtime`,
  `waddle-conformance`): every "at most once per claim window" fault guard was
  reset by whoever happened to notice `claim_active` go false, which nobody
  can promise — the plane pump polls every 20 ms, the media intake polls per
  packet, and `push_intervention_chunk` has no loop at all, running only when
  someone calls it. Two claim windows meeting inside any of those gaps shared
  one set of guards and the SECOND window's refusal was swallowed; a retake,
  where the claim changes with no gap at all, does the same to all of them.
  The guards are now `WindowLatch`es keyed to `Status.claim_generation`, the
  claim window's identity, published whenever the active claim changes — so
  the lifecycle rides the window rather than an observer's cadence. A
  recording missing a refusal says the sender was never told, when it was.
  Pinned by `each_claim_window_is_owed_its_own_refusal` and the scenarios
  `agent_chunk_refusals_latch_per_reason_and_window` and
  `teleop_dims_refusal_is_per_claim_window`.
- **Dual-write detection compares like with like once a command addresses one
  part** (`waddle-conformance`): the reference runner kept ONE
  `last_commanded` vector and fed it into ONE `DivergenceDetector`, which was
  right while every command was whole-robot and became wrong the moment a
  part-tagged action could reach it. A part-width command was compared
  position by position against a whole-robot `ProprioSample.joint_pos`,
  scoring an intervenor driving the right arm against the LEFT arm's joints:
  measured on the bimanual fixture, a chunk commanding `right` to a pose the
  robot then reported reaching exactly still raised
  `DualWriteDetected{VERB_HOLD, divergence 1.31}` — a fabricated safety
  escalation against a sender doing precisely what it said it would, where
  the identical scenario with a whole-robot chunk stayed silent. Two rules
  now hold, and each is pinned by a scenario: the commanded side is keyed by
  the SCOPE each command addressed — a whole-robot command commands every
  part and clears the part-scoped commands it supersedes, a part-scoped one
  replaces its own part and leaves the whole-robot command standing as the
  anchor for the parts it did not address — and a sample is compared only
  against its own scope. A sample for a part with no part-scoped command is
  compared against the last whole-robot command's slice for it (declaration
  order defines that slice; "hold the rest" means it still stands), while a
  whole-robot sample says nothing at all under a standing part-scoped
  command, because the composition it would need is not what its layout
  describes — an observation's layout is the customer's own and no
  declaration describes it, so it is never re-laid-out by action parts. Each
  part also gets its own divergence run: an arm arriving where it was told
  must not reset, and so mask, the run of an arm someone else is writing.
- **A gripper-only intervention step is an action, not a drop**
  (`waddle-types`/`waddle-gate`/`waddle-runtime`): control.proto has
  `Action.gripper` "ride alongside the target in one logical tick", and
  `NoopMarker` is a target arm like any other — so `Action{noop, gripper}`
  says *hold the arm, move the gripper*, and is the only shape available to
  a sender whose gripper command has no arm target beside it. `flatten_action`
  called it non-executable, and the `intervention_chunk` intake then dropped
  the WHOLE chunk with nothing but a `tracing::warn!`: observed live, a
  four-step stream (three joint waypoints, then a gripper close) actuated
  three times and the grip vanished with no recorded fault, no ack-visible
  refusal, nothing the sender could see. Now:
  - **noop + gripper flattens to a gripper-only `Step`** — no arm values at
    all (`Step::is_gripper_only`), the gripper in the declared
    `GripperSpec`'s own units, unmapped (an `ActionChunk`'s
    `GripperCommand.position` is already in those units, unlike a raw teleop
    packet's normalized trigger). It dispatches through the same paths every
    other step does, and `unflatten_action` rebuilds the wire shape it came
    from, so the grip lands on `/waddle/actions` instead of vanishing from
    the recording. `blend_step` treats it as an action rather than a dims
    mismatch: the gripper channel cross-fades and the arm holds.
  - **An inert step (noop, no gripper) is skipped, not fatal to its chunk.**
    `ActionChunk::from_pb` returns a `FlattenedChunk { chunk, inert }`: the
    steps around an inert one still execute — one step with nothing in it
    must never cost the sender the waypoints around it — and the skip is
    REPORTED. Anything that doesn't fit the declared space at all (wrong
    target arm, missing field, opaque space) still refuses the whole chunk,
    since a partial trajectory from a sender that disagrees about the space
    is not a degraded-but-safe thing to actuate.
  - **Every intake refusal is now observable.** `SessionEvent::InterventionRejected`
    carries a `RejectReason` (`Dims` / `NotExecutable` / `InertStepsSkipped`)
    instead of a dims-shaped pair, and each reason emits its own
    `Fault{FAULT_KIND_VALIDATION_ERROR}` text, deduped per reason to once per
    claim window. The non-dims refusals used to be trace warnings only,
    because they could not be told truthfully in the dims-shaped event.
  - FSM.md §4 states the rule ("What an intervention action may carry") and
    §5's dims-mismatch sentence names the gripper-only exception. No guard
    row, no state transition, and no golden fixture changes: this is intake
    and gate behavior, covered by `waddle-runtime`'s end-to-end intake suite.
    Customer-visible: a `send`/`gate()` step may now carry an EMPTY action
    array with its `gripper` set — command the gripper, leave the arm target
    where it was (`Control.send` docs; `toy_robot.py` shows it).
- **A clutch refused by C8 now says so** (`waddle-fsm`): on an agent-invited
  episode, a clutch edge whose declared actor is not `ACTOR_KIND_AGENT` was
  dropped in silence — no claim, no emission, nothing on the timeline. But a
  clutch grants its own claim in one step, so C8 governs it exactly as it
  governs a plane GRANT, and FSM.md's C8 row says a refused grant is
  recorded as `claim{DENIED}`. The .md wins: the refusal now emits the same
  `claim{DENIED}` shape the wrong-actor grant path emits, naming the
  refused actor, with no `claim{REQUESTED}` before it (a clutch has no
  request pending) and no state change. A site operator squeezing the
  clutch on an episode Waddle is driving is exactly the moment a recording
  must be able to explain why nothing happened. FSM.md §2's
  self-initiated-claims paragraph states the rule; new fixture
  `agent_invite_clutch_denied` pins it, and is the first scenario to
  exercise the `clutch` inject kind at all. Clutches refused for any other
  reason (a claim is already active, the episode is not RUNNING) stay
  silent, unchanged.
- **The gate stopped allocating on every claimed tick** (regression, caught
  in review before release): carrying the claimant's `ActorRef` onto the
  provenance tag (the fix below) put two owned `String`s on the tag that
  `Gate::gate()` clones TWICE per tick — once into the record ring, once
  into the returned `GateOutput`. Measured: **four mallocs per `gate()`
  call**, on the customer's real-time thread, on every CLAIMED / BYPASS /
  RESET / agent-episode tick of every claim the plane granted (i.e. every
  production teleop session and every agent intervention; only local clutch
  grants, whose actor has no stamped identity, were spared). The tag's two
  variable-length fields are now shared rather than owned —
  `Provenance::Custom(Arc<str>)` and `Option<Arc<ActorRef>>`, minted once per
  claim by `ActiveClaim::provenance` off the caller's thread — so cloning a
  tag is a refcount bump. `Custom`'s name was already an owned clone before
  the actor work, so a SITE_OPERATOR claim had been allocating twice per tick
  for as long as that path has existed; that is fixed by the same change.
  Measured after: 0 allocations per tick on every arm, and the BYPASS arm
  under a plane-granted claim benches at the same ~75 ns as passthrough.
  `waddle-gate`'s `alloc_free` proof now covers **every plan arm** with a
  plane-shaped claim (it was passthrough-only, which is why nothing caught
  this; its counter is also per-thread now, so concurrently-running tests
  cannot pollute a measurement), and `cargo bench -p waddle-gate` gained the
  claimed-arm case.
- **A `report_proprio` call still queued when an episode ends is recorded in
  that episode**: `finalize` tail-drained the gate-record ring and the
  bypass-pump dispatches before closing the MCAP but not the proprio-report
  channel, so anything reported in the last reducer wake before TERMINAL was
  drained later with no episode open and discarded — or, worse, written into
  whichever episode opened next. A caller reporting at 100 Hz lost the tail
  of every episode. All three side channels are now drained together, in the
  same order the reducer loop drains them.
- **An agent-driven episode's recording contains its actions and its
  observations**: actions and observations were written only on the
  gate-tick record path, and the caller of an agent-invited episode never
  ticks `gate()` (FSM.md E24) — so a warm-up episode recorded 40 actions and
  40 observations while the agent episode that followed it recorded 0 and 0.
  An episode with neither is not a data product: it cannot be judged, scored
  or trained on. Two independent paths were missing:
  - The **bypass pump's dispatch** — the moment an intervenor's action
    actually reaches the robot without passing through the caller's gate —
    now produces an `/waddle/actions` row like any gate tick's, stamped by
    the `SessionClock` at dispatch and carrying the claim's provenance
    (which, per the two fixes above, now names the agent). Its rows are
    tagged `source_id: "waddle_sdk.bypass-pump"` with their own `seq` space,
    since `ActionChunk.seq` is monotone *per stream* and the caller's gate
    (`waddle_sdk.gate`) is a different stream into the same episode. The row is
    written before the `send` request, so a dispatch that fails is still on
    the record. This covers remote reset-window actuation too — the same
    pump drives it.
  - **`Session::report_proprio` now records an observation of its own**, for
    every episode, rather than only surviving as a merge into a subsequent
    gate tick's `ProprioSample`. This gap predates agent episodes: any
    caller that reported proprioception and passed no `obs` to `gate()` lost
    it from the recording entirely; agent episodes, whose caller ticks
    nothing at all, just made it total. `joint_pos` rides the latest known
    one (as the periodic `StreamObservations` uplink already did). A caller
    doing both now records both, each stamped when it happened — which other
    call a caller happens to make cannot decide whether an observation is
    recorded.
  Neither of these two writes touches the gate fast path: both happen on the
  reducer thread, fed by side channels, exactly like `report_proprio`'s
  existing merge. (The actor work below did briefly regress that path — see
  the allocation entry at the top of this section.)
- **A sidecar's claimed provenance spans now say who actually drove**: the
  span opened by `GateModeChange{→INTERVENTION|→BYPASS}` was minted
  `PROVENANCE_KIND_TELEOP` unconditionally, so an agent-driven episode's
  spans read POLICY/TELEOP/TELEOP/POLICY — the recording asserted a
  teleoperator drove an episode no teleoperator touched. The span's kind now
  comes from the open claim's actor through `Provenance::for_claim` (the
  same mapping the per-action tags use, so spans and `/waddle/actions` rows
  can never disagree), and it carries that actor. A teleoperator's span is
  unchanged (TELEOP, naming the teleoperator); an AGENT claim's says AGENT;
  a SITE_OPERATOR's is `custom:<source_name>` rather than being folded into
  TELEOP, because N17 makes a customer-side human at the cell a different
  actor from a Waddle work-plane teleoperator and the corpus has to be able
  to tell them apart. A claimed span also carries the claim's
  `bypass_approval` stamp now, as the per-action tags already did. No proto
  change: `PROVENANCE_KIND_AGENT` has been in the vocabulary since v0.
- **Claim events now name their claimant (`Claim.actor` was being dropped)**:
  the plane grants a claim with a full `ActorRef` — kind, the id it stamped,
  a display name — and the SDK carried only the *kind* into its FSM, so
  every `claim{REQUESTED|GRANTED|DENIED|RELEASED}` it emitted, and therefore
  every journal, sidecar claim span and judge downstream, saw an empty
  `actor`. The recording of an agent-driven episode could name the
  intervention *stream* ("agent") and nothing about who drove it. The FSM's
  `ActiveClaim` now holds the `ActorRef` whole and every claim emission
  carries it; a LOCAL grant with no stamped identity (a clutch edge,
  `grant_and_engage`) carries kind only, via the new
  `ActorRef::of_kind`. `Provenance::for_claim` (new, waddle-types) is now
  the ONE mapping from an actor's kind to the provenance vocabulary — the
  reducer's gate plan and the conformance target both call
  `ActiveClaim::provenance` rather than each re-deriving it, and the tag
  they mint now carries the actor too. No proto change: `Claim.actor` has
  been a declared field since v0 and was simply never populated. FSM.md §2
  states the requirement; new fixtures `claim_events_name_the_claimant` and
  `agent_claim_events_name_the_agent` pin it for a teleoperator and an
  agent. `SessionEvent::ClaimRequested`/`ClaimGranted` take an `ActorRef`
  where they took an `ActorKind` (source-breaking for direct FSM drivers;
  `grant_and_engage`/`reset_window_engage` keep their `ActorKind`
  signatures).
- **`waddle-controlplane` — the offline-classification tests no longer race
  the reconnect clock**: both tests that assert what a message does while
  the plane is unreachable held the window open with a 300 ms backoff step
  and then sent into it, so a test thread stalled past that step (a loaded
  CI runner, or the run right after a heavy build) sent AFTER the client had
  reconnected. The messages were then forwarded live and legitimately, but
  the assertions scan everything the plane ever received and cannot tell a
  live forward from a replay — a real failure of a gate this repo requires
  clean before every commit, and a false one. `InMemoryTransport` gained
  `refuse_connections`/`allow_connections`, so the offline window lasts as
  long as the test needs rather than as long as a step happens to take, and
  the tests wait for a full drain cycle (two refused dials, a happens-before
  they can observe) before healing the partition. No production behavior
  changed; the suite is also ~60x faster for dropping the long sleeps.
- **Every declared camera's resolution is now declared to the media
  plane**: a LiveKit track publishes at ONE declared resolution and drops
  every frame that disagrees, so a session that inherited the 640x480
  default dropped **100% of the frames of every camera that was not exactly
  640x480** — with nothing raising (the uplink pump warns and counts the
  drop; `publish_frame` still returns Ok), so it presents as "the
  teleoperator sees nothing" on a session that reports success.
  `LiveKitConfig::with_robot_cameras(&RobotDescription)` (new, in
  waddle-media) declares a track resolution for every `cameras` entry, from
  the same declaration `Session::publish_frame` validates frames against;
  the Python `create_session` calls it, and any other binding with a robot
  declaration must. The mapping lives in core — with the test that a
  two-camera robot yields both resolutions and no camera inherits the
  default — rather than in each binding, where nothing would have caught it
  going missing again. Unreachable until this branch wired the media plane
  through the Python surface at all.
- **sdk — a wheel no longer ships whatever bytecode the build machine left
  behind**: `[tool.maturin].python-source` is the working tree, so both
  `pyproject.toml`s now carry `exclude = ["python/**/__pycache__/**"]`.
  Without it a build run after `uv run pytest` swept
  `__pycache__/*.cpython-3XX.pyc` for whichever interpreter ran the tests
  into the wheel while a build on a clean checkout did not — the same
  commit producing two different wheels, with stale bytecode for one
  interpreter riding along to every other. pip compiles bytecode at
  install time for the interpreter that will actually import it.
- **sdk — `_core.pyi` no longer under-describes the extension**: the type
  stub had drifted behind the shim (`publish_frame`, `report_proprio`,
  `records_dropped`, the `_testing_*` hooks and the new connected seam were
  all missing), so type checkers reported errors on correct calls and, worse,
  silently accepted wrong ones. It is now cross-checked method by method
  against `sdk/rust/src/*.rs`. One stub types BOTH cores, because both are
  the same shim built with different features.
- **`waddle-fsm` — the E24 agent-episode gate plan is now re-projected
  whenever its inputs move, not only when the gate MODE does**: the gate plan
  is derived state, and plan derivers (the runtime reducer, the conformance
  target) re-derive it only when they see `Effect::SetGateMode`. E24's Noop
  plan also depends on the EPISODE (agent-invited, phase), so any row that
  moved that without touching the mode left every deriver holding a stale
  plan. **The reachable failure**: an agent-invited episode closed by a
  **retake** (C5 — the claim survives, so the shared run-closing block skips
  the re-projection it does on a claim-releasing close) handed its
  born-claimed successor, a NORMAL episode, the predecessor's Noop plan; the
  customer's own `gate()` ticks then returned
  `Noop{NOOP_REASON_AGENT_EPISODE}` forever, with no fault — a control loop
  that silently stops actuating. Retake is plane-reachable from the engage
  timeout, and no fixture covered retake on an agent-invited episode, so
  every gate stayed green. The mode-unchanged re-projection now happens
  centrally, in the one place every row funnels through (`Ctx::finish`,
  keyed on the FSM-owned plan inputs `(gate_mode, agent_episode_noop())`),
  replacing the two per-row pushes that covered only episode open and
  claim-releasing closes; new session invariant **I20** asserts it for every
  step of the random walk, and the new fixture
  `agent_invite_retake_successor` pins the retake path end to end. E24's
  scope also gained INTERVENTION — the *engage window*, where the handoff is
  in flight, the gate is still PASSTHROUGH and nothing is engaged yet, so
  E24's own guard ("no engaged claim") holds: the predicate said otherwise
  while the installed plan noop'd, and the plan was right. **FSM.md**'s E24
  row stated `RUNNING` alone in its From column while the implementation (and
  the fixtures) also noop'd in RESETTING and READY; it now states the full
  set, and §1.5 says outright that the plan is scoped to the episode it was
  derived for and must be re-derived when the episode state behind it moves
  — a second implementation can no longer dispatch the caller's actions
  inside an agent-invited episode's reset, or keep noop'ing after it, and
  still pass the suite.
- **The bypass pump exempted a never-ticked gate from stall detection, so an
  engaged claim in a session whose gate never ticks would have gone undriven
  forever**: `spawn_bypass_pump` only reported a stall when a previous
  `gate_tick` existed and was older than the threshold, so a `None` last tick
  was silently exempt. FSM.md §6's condition is "no `gate_tick` within the
  stall threshold", which holds *vacuously* when there has never been one —
  and that is exactly the shape of an agent-invited episode driven through
  `Session::run_agent`, which reaches RUNNING with the caller's thread
  blocked and therefore never ticks at all. A `None` last tick now counts as
  stalled (the `Some` threshold contract is unchanged); the FSM's own
  `StallDetected` guard still decides whether anything follows, so no guard
  row changed — the pump only reports.
- **`waddle-controlplane` — droppable messages can no longer queue without
  bound while the plane is unreachable or stalled**: `ClientMsg` now answers
  "is this perception/liveness, or history?" in exactly one place
  (`is_droppable`; `buffer_when_offline` is its negation), and BOTH moments a
  message can be shed honour it. (1) While connect attempts fail, the client
  thread now drains its command channel into the bounded offline buffer on
  every backoff slice (`backoff_draining`) instead of only before and after
  the sleep: an unreachable plane used to let the unbounded command channel
  grow for a whole backoff plateau (16 s in production) with the drop-oldest
  bound and its loud `BufferOverflowed` never applying, and every message
  parked there — including droppable ones — was handed to the plane the
  moment it came up, so a partition's worth of stale pictures replayed as if
  fresh. (2) The gRPC transport meters every outbound stream with its own
  `InflightLimit` (new `inflight` module; cap 4 per stream, shed count on
  `GrpcTransport::droppable_dropped`): a plane that accepts
  `StreamObservations` and then stops reading it never errors, so no
  `Disconnected` is ever raised and the offline classification never runs —
  the stills piled up in the transport's internal channels behind a stream
  h2 had stopped polling, unbounded, until OOM. History is never shed by
  either mechanism; only heartbeats and control-plane stills are droppable.
  The `ControlTransport` trait now states the contract: a transport that
  buffers internally must bound what it holds for droppable messages.
- **`Session::run_agent` no longer masks a genuine pre-reset failure (E5)
  as a normal-looking agent ABORT**: the recovery arm that turns a
  `ResetFailed` from the start path into an `AgentOutcome` exists for
  closes the invite machinery itself produces while the caller is still
  blocked in RESETTING (E25's deadline expiry, E26's pre-engage DENIED),
  but it keyed on `agent_invited` alone — the mirror carried no "why", so
  a failing pre-reset hook on an agent-invited episode returned
  `Ok(AgentOutcome{ABORT, detail: ""})` (indistinguishable from "no agent
  engaged") instead of the `RuntimeError::ResetFailed` every other start
  path surfaces, and retry loops would grind against broken reset hardware
  with no error ever raised. The FSM now latches `episode.invite_aborted`
  on exactly E25/E26 (documented in FSM.md §1.5 alongside the
  `agent_engaged` latch; pinned by session-invariant I19), the mirror
  publishes it as `Status.agent_invite_aborted`, and the recovery arm keys
  on that: E25/E26-during-RESETTING still return the ABORT outcome (new
  test drives a real invite timeout under a slow pre-reset hook), while an
  E5 reset failure surfaces as `ResetFailed` (new test). Also pinned by
  test: the unconditional `waddle.v0.agent` Register advertisement
  (deleting it previously kept the whole suite green while silently
  severing real-plane invite routing).
- **`waddle-fsm` — a wrong-actor grant on an agent-invited episode now
  records `claim{DENIED}` (FSM.md C8) instead of being silently dropped**:
  C8 specifies "any other actor's grant is rejected, `claim{DENIED}`" — the
  plane already sent GRANT, so the SDK's refusal must go on the timeline —
  but the reference FSM returned a bare rejection and emitted nothing, and
  the `agent_invite_wrong_actor_denied` fixture asserted only the absence of
  a GRANTED emission, so a spec-following implementation (emitting DENIED)
  and a silently-dropping one both passed conformance. The refusal now
  emits `ClaimEvent{DENIED, detail}` with no state change (same shape as the
  stale reset-engage mint's `lease{DENIED}`; the first production emitter of
  `CLAIM_EVENT_KIND_DENIED`), the fixture asserts the emission, and the FSM
  lifecycle smoke test walks it. C6's wrong-actor rejection stays silent —
  its row never specified a DENIED record and its released golden pins that.
- **`waddle-fsm` — a `reset_window_complete` racing an in-flight engage
  lease mint panicked the reducer thread and hung every blocked caller**:
  E20's lease routing is asynchronous (the runtime answers
  `Effect::MintLeaseToken` via the tail of its single event queue), so a
  plane sending ENGAGE and COMPLETE back-to-back gets the COMPLETE processed
  before the engage's mint answer. The COMPLETE handler had no
  engage-in-flight guard: it saw the window un-engaged, closed it, released
  the reset claim, and (PRE, ok) went READY with the engage's
  `pending_lease` still populated — the stale mint answer then handed the
  lease to the released claimant and panicked (`expect("reset claim
  held")`), killing the reducer (no catch_unwind) so `start_episode*` /
  `terminate_episode` waits hung forever. Two-part fix, pinned as normative
  prose in FSM.md §1.4 ("Engage atomicity"): (1) a COMPLETE arriving while
  an engage mint is in flight is **rejected** — a window that never
  observably ENGAGED has nothing to honorably complete; the plane retries
  after it sees `reset_window{ENGAGED}`; (2) a minted engage lease whose
  reset claim (or window) is gone by the time it applies — e.g. a legal
  `claim_released` raced the answer — is discarded (`lease{DENIED}`, lease
  unmoved, window still serviceable) instead of panicking: the FSM never
  panics on a legal event ordering. The invisibility root cause was that
  every existing harness (the FSM test drivers and the conformance runner)
  answered mints synchronously, so the interleaving was inexpressible;
  the FSM test driver and the property-test alphabet now support deferred
  mint answers (`DeferMints`/`AnswerMint` random-walk commands run all 14
  session invariants over these interleavings), four deferred-mint FSM
  regression tests pin rejection/degradation/benign-overwrite/timeout
  behavior, and two runtime tests drive ENGAGE+COMPLETE back-to-back
  through the production plane-directive path and assert the session
  always resolves with the reducer alive.
- **`waddle-controlplane`'s `tonic-transport` test build (pre-existing
  break)**: `grpc_transport.rs`'s two
  `ClaimDirective` struct literals predated the directive-acks feature
  (`waddle.v0.plane.acks`) that added its `directive_id` field and were
  never updated for it — a compile break invisible to `cargo test
  --workspace` (featureless) and never caught because nothing since had
  re-run this crate's feature-gated tests (they are now part of the
  standing pre-commit gates in CLAUDE.md). Fixed with `directive_id: None`
  (no production code touched).
- **`Session::publish_frame` — a declared `CAMERA_ENCODING_JPEG` uplink
  policy would fail every frame against a real LiveKit-backed session**:
  the previous behavior ran a declared JPEG uplink through the real
  `JpegEncoder` (Motion JPEG bytes) before handing it to `MediaPlane::
  push_frame`, but a WebRTC video track (the only real transport wired,
  `LiveKitMedia`) ingests raw RGB8/I420 only — libwebrtc encodes the
  uplink itself, and a still-image byte stream is not a track format at
  all. Neither `media.proto` nor `descriptors.proto`'s `StreamPolicy`/
  `UplinkPolicy` comments promise JPEG-on-the-wire for tracks, so the fix
  reconciles the encoding contract instead of the transport: a declared
  `UplinkPolicy.encoding` is now bandwidth-intent for the customer, not a
  literal byte format — `RGB8`, `BGR8`, and `JPEG` all resolve to raw
  passthrough on the track path and publish identically (the transport
  converts to whatever the track needs; `LiveKitMedia::push_frame` already
  did this conversion, `rgb8_to_i420`, for RGB8 — it now also receives
  correctly-shaped bytes for a JPEG-declared camera instead of a mismatched
  compressed buffer). `CAMERA_ENCODING_H264` is unchanged: still the one
  genuinely unsupported encoding, still a build-time
  `RuntimeError::UnsupportedCameraEncoding`, never a silent per-frame
  failure. `waddle-media`'s real `JpegEncoder` is untouched and
  remains available for a genuine still-image byte stream path (e.g. a
  future data-channel/recording snapshot) — nothing on the track path
  calls it today. Regression-tested with a LiveKit-shaped `MediaPlane`
  test double (validates the same RGB8-or-I420 track shape `LiveKitMedia`
  does, without the `livekit` feature or a live server): RGB8 and JPEG
  declarations both publish a raw frame through to the track with zero
  drops; H264 stays a clear build-time error.
- **`sdk/tests/test_e2e.py::test_intervention`'s pre-existing flake**: the
  test declared a 3-joint robot but pushed teleop `Twist` packets, which
  `pumps::flatten_packet` always flattens to exactly 6 values (linear xyz +
  angular xyz) — media intake's dims validation (already landed)
  correctly rejected every packet as a dims mismatch (3 declared vs. 6
  incoming), so the intervention stream never reached the gate and the
  test's 5s wait for a substitution always timed out. This was a stale test
  fixture, not a timing race or a core regression (confirmed deterministic
  across repeated runs, and identical on the commit immediately before this
  change) — fixed by giving the test's robot a 6-joint action space to
  match the raw twist width it actually exercises.
- **Reducer-opened retake successors hung in RESETTING forever**: only
  `start_episode`'s inline path ever ran the pre-reset pipeline, and a
  retake successor is opened by the reducer (`Effect::OpenSuccessor`) with
  no blocked caller — so nothing injected its `ResetResult` and the
  born-claimed successor never reached READY. The reset pump now services
  it (regression-tested by driving a retake through the runtime and
  asserting the successor passes through reset to READY, with the
  session's PRE hook run exactly once for it).
- **Verb-registration validation at session build**: `SessionBuilder::build`
  now fails fast with a new `RuntimeError::MissingVerb` instead of letting a
  missing callable surface only at first dispatch. Previously, the default
  handoff policy (HOLD_FIRST) issues `Verb::Hold` on every engage; with a
  media plane wired but no `hold` callable registered, dispatch failed
  `NotRegistered` silently and the engage fail-closed only at the 10s engage
  timeout — the teleoperator's clutch did nothing, with no diagnosable
  error. `hold` is now required at build time whenever the handoff policy is
  HOLD_FIRST and the session has a live engage path; `send` is now required
  under that same condition, independent of handoff policy (the bypass pump
  can drive `Verb::Send` directly once a claimed loop stalls). A live engage
  path is a wired media plane **or** `hold`/`send` registered in `Control`
  directly — `grant_and_engage` (the local-intervention convenience,
  exported from the crate root and used by "tests and local intervention
  sources") injects `ClaimGranted`/`Engage` with zero dependency on
  `self.media`, so a session that registers `send` for local intervention
  without ever calling `.media(...)` is exactly as live an engage path as
  one wired to a media plane, and is now checked the same way. Both errors
  name the fix directly (e.g. "handoff HOLD_FIRST requires a registered
  `hold` verb — register one in your Control, or choose a different handoff
  policy"). Sessions built with no Control and no media plane (the
  descriptors-only / minimal-local case, including the PyO3 shim's
  all-None-verbs `create_session`) are unaffected and stay buildable — that
  shape has no build-time-visible engage path at all; `grant_and_engage`'s
  own doc comment now carries an explicit safety note that direct callers
  outside that shape are still responsible for registering `hold`/`send`
  themselves. A missing `estop` is deliberately never build-fatal, but the
  degradation is now recorded on the status mirror
  (`Status::estop_unregistered`) so it stays observable. The `hold` check
  reasons about the *effective* handoff policy, not the raw declared enum
  variant: `waddle_fsm::begin_engage` silently degrades a declared
  `HandoffPolicy::Immediate` to HOLD_FIRST on the very first engage whenever
  the robot's action space contains a delta component (FSM.md §5 — delta
  spaces refuse mid-chunk splice entry), so `build()` now applies that same
  degrade before checking `hold`, closing a gap where a declared-IMMEDIATE
  session over an `EePoseDelta`/composite-with-delta space built clean and
  then stalled at the first engage the same way the undegraded bug did.
- The task passed to `Session::start_episode` now reaches the episode
  sidecar (it was previously dropped after the reset hook; sidecars always
  recorded an empty task).
- Episode-lifecycle hardening (from an adversarial review of this series):
  `start_episode` while an episode is live now returns
  `RuntimeError::EpisodeActive` instead of destroying the live episode's
  recording and blocking forever; a stale `Episode` handle can no longer
  write records into a later episode's MCAP (the fresh ring reaches the
  reducer before the open event; stale leftovers are discarded, while
  retake successors still inherit the caller's ring) or terminate a later
  episode (`terminate` is a no-op unless the episode is still live);
  `Episode::done` now also flips on session shutdown, so the tutorial loop
  cannot spin forever after `waddle_sdk.shutdown()`. New
  `Episode::records_dropped()` surfaces ring overflow (training-data
  loss). A gated action that does not fit the declared space (raw teleop
  stream ahead of closed-side retargeting) now records an action-less
  chunk instead of silently skipping the tick, keeping `/waddle/actions`
  obs-aligned. The Python shim's `Session` also shuts the core down safely
  when dropped without `shutdown()`, and `terminate` no longer holds the
  episode lock across its blocking wait (other threads' `gate`/`done`
  stay responsive).
- **Media intake — stale-backlog replay**: intake now pushes a teleop pose
  into the intervention ring only while a claim is active; previously every
  pose was queued regardless of claim state, so up to a whole ring's worth
  of pre-claim poses could all become "due" the instant a claim engaged and
  replay as stale motion while fresh packets were dropped by the full ring.
- **Media intake — no action-space validation on injected teleop actions**:
  a flattened teleop action whose width doesn't match the session's
  declared action space is now dropped at intake instead of substituted
  verbatim, with a `Fault{VALIDATION_ERROR}` recorded once per claim window
  (not once per packet at 60-90 Hz) via a new
  `SessionEvent::InterventionRejected`. `waddle-gate`'s blend step no longer
  zip-truncates a dims mismatch between the blend anchor and the
  intervention target (a false "validated upstream" comment is now true in
  practice, and a real defense-in-depth guard on the rare mismatch that
  still reaches it); it now returns no blend and the gate falls back to
  Hold.
- **Conformance runner — `teleop_action` injection only read the first part
  target**: `waddle-conformance`'s scripted intervention-stream flattening
  now concatenates every part target in packet order (pose → 7 values
  wxyz, twist → 6), matching production `flatten_packet` semantics.
  `scenario-format.md`'s `teleop_action` payload never pinned "first target
  only"; the narrower reading was a runner defect, surfaced by the
  media-intake dims-validation fix above.
- **GripperSpec never applied**: the teleop gripper command (normalized
  0..1, 1 = open — the media-plane convention) is now mapped through the
  session's declared `GripperSpec` at intake — linearly onto
  `[closed_value, open_value]` for `Parallel`, thresholded at 0.5 for
  `Suction` — instead of being copied onto the wire verbatim. No declared
  spec still passes the command through unchanged.
- **Clutch claim provenance mislabeled as non-teleop**: a clutch edge on the
  media plane (the leader-arm/console-clutch takeover path) self-initiates a
  claim; `waddle-runtime`'s `SessionBuilder` now defaults that claim's actor
  to `ActorKind::Teleoperator` (source `"teleop-clutch"`) instead of
  inheriting `waddle-fsm`'s `SiteOperator`/"custom" default, so the
  reducer's provenance mapping records these interventions as teleop —
  provenance-labeled training data (DAgger pairs) was silently mislabeled,
  and the N17 actor vocabulary was violated. `waddle-fsm`'s own default is
  unchanged (fixture stability); `SessionConfig` gains `clutch_actor`
  (alongside `clutch_source`), and the new `SessionBuilder::clutch_identity`
  setter lets integrators override both.
- **Jitter buffer — one shared reorder cursor for two independent
  producers**: the intervention ring's `JitterBuffer` kept a single
  session-wide `last_popped_seq` watermark, but two producers write into
  it — the media-intake thread (teleop, seq = wire
  `TeleopStreamPacket.seq`) and the plane pump's reset-window
  `intervention_chunk` arm (agent chunks, seq = a fresh pump-local counter
  starting at 0). An ordinary teleop claim earlier in the session (nothing
  to do with any reset window) would advance that one shared cursor well
  past 1, so the first agent-chunk step of a *later* reset window — the
  exact `pre_reset=TeleopReset`/`post_reset=AgentReset` shape the design
  suggests as normal — would look "late" and be silently, permanently
  dropped, with the window then just timing out and no diagnostic trail
  (`dropped_late` has no readers). `JitterBuffer` now keeps one reorder
  cursor per `TimedAction::channel` (`StreamChannel::Teleop` /
  `StreamChannel::AgentChunk`), so neither producer's activity can starve
  or drop the other's arrivals. Regression-tested by driving an ordinary
  teleop claim to completion (advancing the teleop channel's cursor well
  past a small number) and then confirming a later Remote POST window's
  agent chunk still dispatches.
- **`intervention_chunk` during a reset window — malformed chunks dropped
  with zero signal**: a wire chunk that fails `ActionChunk::from_pb`
  (dims mismatch, wrong target variant, an Opaque space, …) during a
  Reset-mode window was silently ignored, unlike the parallel teleop path
  (which raises `SessionEvent::InterventionRejected` on a dims mismatch).
  Since this is the only actuation channel for an Agent-kind reset window
  (no teleop fallback), `forward_server_msg` now logs a `tracing::warn!`
  naming the rejection instead of dropping it with no trace; behaviorally
  verified (no dispatch, no corruption, the window still resolves
  normally on the plane's COMPLETE).
- **Intervention ring — a released claim's leftover, not-yet-due actions
  could outlive it and dispatch under a LATER, unrelated claim's
  provenance**: per-channel reorder cursors (above) stop the wrong-channel
  seq collision, but not this — an arrival pushed but not yet due when its
  claim releases or its reset window closes sat in that channel's pending
  map with nothing left to drain it (the caller stopped ticking `Claimed`,
  and the bypass pump only polls while `Bypass`/`Reset` is active). It
  resurfaced the next time anything popped that same channel, which could
  be a much later, entirely unrelated claim or reset window, dispatched
  tagged with THAT claimant's mirror provenance — corrupting the
  provenance-tagged actuation record during a reset window, a scene-reset-
  sensitive context. With a 20ms playout delay and typical teleop packet
  rates this triggered routinely (at least one in-flight packet pending at
  essentially every claim release), not as a rare race. `waddle-gate`'s
  `JitterBuffer::clear_pending`/`StreamIntake::clear` now discard every
  channel's pending, not-yet-due arrivals (cursors untouched); the reducer
  (`Effect::SetGateMode`) calls it on every transition back to
  `GateMode::Passthrough` — the one point every claim/reset-window
  teardown funnels through, while `Bypass`<->`Intervention` toggling for
  the SAME live claim never passes through it, so nothing still
  legitimately in flight is discarded. Regression-tested at the
  `JitterBuffer` level and end-to-end (an ordinary teleop claim releases
  with in-flight packets still pending, then a later Remote POST window's
  agent chunk dispatches with zero teleop residue reaching `send`, checked
  on the dispatched values rather than provenance alone).
- **Retake successors never inherited the session's `post_reset` config**:
  `Effect::OpenSuccessor` hardcoded `post_reset: false` (with a stale
  comment claiming a runtime start path applied the config — no such path
  runs for a reducer-opened episode), so a retaken episode's own
  termination skipped straight to `Terminal` with no cleanup at all, even
  when the session declared one. The reducer now carries the session-level
  `post_reset` default and resolves it the same way `start_episode_with`
  does (`Hook` → `post_reset: true`; `Remote` → the declared `post_window`
  too) when answering `OpenSuccessor`; a `Remote` post-reset opens the
  successor's own POST window exactly as it would for any other episode —
  the born-claimed suppression (D7 edge 5) is a PRE-window-only guard and
  never applied to POST. A predecessor's per-episode `post_reset` override
  still does not carry across a retake (documented on `EpisodeOptions`):
  the successor only ever sees the session-level default, matching the PRE
  side's existing behavior. `pre_reset` on successors is unchanged (the
  reset pump already fell back to the session default for the PRE phase;
  a declared `Remote` PRE spec on a successor remains the known gap noted
  above, pending the closed-side retake/hand-reset flow).

## Stowed changelogs

_None yet. On first release, the released section moves to
`docs/changelogs/CHANGELOG-<artifact>-<version>.md` and is linked here._
