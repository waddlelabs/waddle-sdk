# waddle-sdk (Python)

The Python frontend owns one customer site: strict configuration, hardware and
camera lifecycle, the owner envelope, paired timestamps, and raw recordings.
Metal consumes the structural `waddle_sdk.runtime.SdkRuntimePort`; the SDK never
imports or discovers Metal.

## Site lifecycle

```python
import waddle_sdk

site = waddle_sdk.load_site("site.yaml")
with site.open(transport=waddle_sdk.Grpc(url, token)) as session:
    with session.run(task={"id": "pick"}, actor={"id": "metal"}) as run:
        observation = run.observe()
        result = run.step(action, observation)
        if not result.dispatched:
            run.hold(result.detail or "command withheld")
    session.estop("emergency")
```

Exiting a `Run` without an explicit terminal result records ABORT. Exiting the
site context finalizes the recording and closes cameras, pumps, robot handles,
and native threads even when opening or the body failed. `Run.step` asks the
native gate first, then preflights every addressed owner-envelope target before
writing any part; one refusal holds the addressed set and moves none of it. The public
lifecycle exposes no lease or handoff selector: handoff is fixed to hold-first and
enforcement placement is derived from the selected integration.

The root package deliberately exports only `Site`, `SiteSession`, `Run`,
`load_site`, `Grpc`, `LiveKit`, `Outcome`, and manifest errors. Driver-extension
contracts live in their subpackages. The older module-global lifecycle, local
web UI, hosted task/artifact facades, and execution-backend discovery are not
part of the public surface.

## `site.yaml`

```yaml
api_version: waddle.site/v1
kind: Site
metadata: {id: customer-cell-1}
parts:
  left:
    driver: waddle_sdk.robots.yam:arm
    posture: supervised
    connection: {channel: can_left}
    joint_limits: {}
    gripper:
      joint: gripper
      closed_m: 0.0
      open_m: 0.095
      closed_action: 0.0
      open_action: 1.0
    options:
      gripper_limits: [1.7, 0.1]
      arm_gain_scale: 1.0
      gripper_gain_scale: 1.0
cameras:
  overhead:
    driver: waddle_sdk.cameras.realsense
    connection: {serial: "207322250310"}
    stream: {width: 1280, height: 720, fps: 30}
frames: {}
calibration: {artifacts: calib/}
workspace_bounds: {}
envelope:
  static_keepouts: []
  self_collision: {}
recording: {root: data/, format: mcap}
```

The Draft 2020-12 schema rejects unknown fields. Site paths are relative,
portable, normalized, and confined beneath the manifest directory. Values with
credential-like names must be `{secret: NAME}` and are resolved only while the
site is being opened. A site manifest cannot contain graphs, skills, models,
chat configuration, API keys, lease modes, or Metal workspace paths.

`parts.*.gripper` is driver-neutral control metadata: it maps a physical jaw
opening in metres onto one declared action row. It is visible through
`Site.describe()` but is never forwarded into the driver factory. Adapter
construction facts such as a YAM unit's measured motor limits remain under
`options`. YAM limits retain I2RT's semantic `[closed, open]` order; depending
on that unit's motor direction, the measured pair may be descending and must
not be sorted.
`arm_gain_scale` changes only the first six I2RT kp/kd rows;
`gripper_gain_scale` changes only the seventh. Both default to the vendor's
gains and, when configured, are restored unchanged after an e-stop recovery.

`static_keepouts` accepts strict axis-aligned `box` and `sphere` records with
an ID, collision frame, optional part filter, and non-negative margin.
`self_collision` enables deterministic named-body checks within and across
parts, with a shared margin and explicit `ignore_pairs` for adjacent bodies.
The selected driver adapter supplies conservative `base.CollisionSphere`
geometry in one `collision_frame`; all intersection policy and reject-whole
dispatch remain in the SDK. A configured rule with missing geometry or a frame
mismatch fails closed while the arms are opened.

## Runtime port and remote admission

`SdkRuntimePort` is the same contract for direct-library and transport adapters:
`describe`, `begin_run`, `observe`, `submit`, `hold`, `estop`, cursor-based
`events`, and calibration measurements. Its DTOs and structured faults are in
`waddle_sdk.runtime`. Authority and time stay native-core owned.

A connector first advertises `waddle.v0.connector.binding` in an
`authorization_only` Register carrying the exact customer, project, and workspace.
The host must authenticate that tuple and accept the flag before `SiteSession`
invokes any hardware builder; a refusal, old host, or timeout opens nothing. The
probe does not advertise hosted runs. The runnable connection then registers the
same tuple again, and every reconnect clears and renegotiates the answer. The runtime emits the existing v0
heartbeat every 500 ms after registration; a revoked key therefore severs the transport
and reaches the core-owned partition/hosted-run hold path.

A runnable remote SDK advertises `waddle.v0.hosted.runs`. Accepted requests open through
the ordinary reset/recording path; duplicate IDs return the original admission
result. Requests and statuses never buffer across connections. Timeout or
disconnect requests core HOLD and aborts the episode, and reconnect never
replays motion.

RGB-D calibration resolves the selected pixel against the exact latest local
depth frame and transmits only the bounded 3-D measurement. Image/depth arrays
remain customer-side. Guided calibration orchestration belongs to Metal and the
hosted UI, not to an SDK-local web server.

## Robot modules (`waddle_sdk.robots`)

A vendor module is facts plus a lazy driver factory over the vendor-neutral
`robots.base` layer. Factory construction opens no bus and starts no thread;
`Site.open()` owns the actual lifecycle. The built-in YAM, xArm 6/7,
Alicia-M, Alicia-D, MuJoCo, and camera adapters import vendor SDKs lazily, so ordinary
imports require no hardware packages. The three manifest-native physical
families expose a single joint space with a normalized 0..1 gripper row;
Metal owns IK and planning above this boundary.
For no-hardware work, `waddle_sdk.robots.mock:arm` is a manifest-native
configurable simulated arm with planar FK and conservative body geometry.
`waddle_sdk.robots.mujoco:arm` loads a site-relative MJCF only when the Site
opens, maps declared scalar joints and actuators explicitly, and evaluates TCP
and conservative body geometry on a separate scratch state before dispatch.

The owner supplies hard safety. `base.Arm` checks declared width, finiteness,
joint limits, per-step travel, optional workspace bounds, and SDK-owned static
keep-out/body-collision rules over adapter-supplied conservative spheres. It
rejects a complete target and never clamps it; malformed or missing configured
geometry is itself a refusal. Posture chooses
which verbs a driver can expose (`monitor` or `supervised`); it never decides
who holds authority. Claims, leases, and handoff remain native internals.

Managed cameras implement `capture() -> CameraFrame` plus idempotent `close()`.
Each RGB/RGB-D sample receives one paired monotonic/Unix stamp, RGB follows the
recording/media path, and aligned depth remains in the SDK process.

### Write your own vendor module

A driver is any object with the ten members of `base.Driver` (`kind`,
`estopped`, `read`, `write`, `hold`, `estop`, `re_enable`, `step`, `home`,
`close`) — a `typing.Protocol`, so yours is admitted on its members and never
on its ancestry. `kind` is your driver's own word for itself, and this layer
reads it in ONE direction: `"sim"` alone selects the harmless branch of the
two questions it asks (does closing this drop all torque, is homing it a
motion nobody is watching), and every other word is treated as metal.

The rest is a facts table, a driver and a factory. These are the exact lines
`tests/test_robots_base.py` builds a whole toy vendor module out of and drives
end to end through a real session — declaration, envelope, gate, pump, MCAP
read-back — with nothing vendor-specific in `base` to help it:

```python
import waddle_sdk
from waddle_sdk.robots import base

TOY_FACTS = {
    # The vendor's own numbers, with their provenance in the comment beside
    # them in a real module. A toy crane: two arm joints and a hand.
    "joints": ("boom", "stick", "grip"),
    "limits": ((-1.0, 1.0), (-1.5, 1.5), (0.0, 1.0)),
    "step_caps": (0.10, 0.10, 0.25),
    "max_effort_nm": 4.0,
    "rate_hz": 20.0,
    "home": (0.0, 0.0, 1.0),
}


def toy_driver() -> base.SimDriver:
    return base.SimDriver(
        TOY_FACTS["home"],
        lower=[lo for lo, _ in TOY_FACTS["limits"]],
        upper=[hi for _, hi in TOY_FACTS["limits"]],
        step_caps=TOY_FACTS["step_caps"],
        rate_hz=TOY_FACTS["rate_hz"],
    )


def toy_crane(*, posture: str = "supervised") -> base.Rig:
    """The whole of a second vendor module: declare the robot, say how to open
    it, hand back a rig."""
    space = waddle_sdk.descriptors.JointSpace(
        joints=[
            waddle_sdk.descriptors.Joint(
                name=name,
                min_position=lo,
                max_position=hi,
                max_effort=TOY_FACTS["max_effort_nm"],
            )
            for name, (lo, hi) in zip(TOY_FACTS["joints"], TOY_FACTS["limits"])
        ],
        rate_hz=TOY_FACTS["rate_hz"],
        chunking=waddle_sdk.descriptors.Chunking(
            horizon=1, replan="immediate", interp="hold"
        ),
    )

    def build_arms() -> dict[str, base.Arm]:  # the bus opens HERE
        return {
            "": base.Arm(
                part="",
                driver=toy_driver(),
                joint_names=TOY_FACTS["joints"],
                joint_limits=TOY_FACTS["limits"],
                step_caps=TOY_FACTS["step_caps"],
                rate_hz=TOY_FACTS["rate_hz"],
                home_values=TOY_FACTS["home"],
            )
        }

    return base.Rig(
        declaration=waddle_sdk.descriptors.Robot(
            name="toy-crane", robot_id="toy-crane-01", action_space=space
        ),
        build_arms=build_arms,
        rate_hz=TOY_FACTS["rate_hz"],
        posture=posture,
    )
```

That is the whole of a second vendor module, and it is a **test** rather than
a docs snippet in the only sense that survives a year: the claim it makes —
"the base layer carries all of the behaviour" — is true exactly while that
test passes, and the block above is not a retelling of it but the same text,
held to the test's own source by
`test_the_published_template_is_these_same_lines`. (The two imports are yours;
every line below them is that file's.)

`toy_driver()` returns the shipped twin because a test has no bus to open.
Yours returns your own driver there — any object with the ten members above —
and a module that ships both takes `sim: bool` and branches, as
`waddle_sdk.robots.yam` does. Either way it is constructed inside `build_arms` and
never at import, so the factory call opens no bus and starts no thread: a
program may build a rig and then decide not to run it.

For more than one part, declare a `waddle_sdk.Composite` and return one `Arm` per
part name (declaration order IS the concatenated action layout). Add `fk=` to
each `Arm` if you have forward kinematics, and only then a `workspace=` box.
To enforce manifest keep-outs or self-collision, also pass
`collision_frame="..."` and a deterministic `collision_spheres(q)` callable
that returns conservative named `base.CollisionSphere` values for every robot
body the rule protects. The callable reports geometry only: SDK code owns
intersection, margins, ignored adjacent-body pairs, whole-command refusal, and
cross-part atomicity.
Ship the source that can gate a fact next to the facts — `waddle_sdk.robots.yam`
vendors the model its numbers come from (MIT data inside this Apache-2.0
wheel: [Third-party content in the wheel](#third-party-content-in-the-wheel))
and `tests/test_yam_facts.py` compares every one of them against it,
directionally (a declared limit may only be TIGHTER than the model's) — and
where nothing can gate a number, say in the comment which pinned artifact it
came from. An unsourced number is one nothing checks.

## Installing

```bash
pip install waddle-sdk                    # control plane included
pip install 'waddle-sdk[orbbec]'          # + lazy Orbbec RGB-D adapter
pip install 'waddle-sdk[realsense]'       # + lazy RealSense RGB-D adapter
pip install 'waddle-sdk[usb]'             # + lazy OpenCV USB/UVC adapter
pip install 'waddle-sdk[cameras]'         # + Orbbec, RealSense, and USB
pip install 'waddle-sdk[xarm]'            # + UFactory xArm SDK
pip install 'waddle-sdk[alicia]'          # + Alicia-M SDK (Python 3.11+)
pip install 'waddle-sdk[alicia-d]'        # + Alicia-D SDK (Python 3.11+)
pip install 'waddle-sdk[robots]'          # + all three physical families
pip install 'waddle-sdk[mujoco]'          # + MuJoCo 3.x simulation
pip install 'waddle-sdk[teleop]'          # + the LiveKit media plane
pip install 'waddle-sdk[cameras,teleop]'  # camera adapters + LiveKit media plane
```

Two distributions from one source tree (the psycopg / psycopg-binary
shape): `waddle-sdk` bundles the core built with the `grpc` control
transport, and the `teleop` extra adds the exact-pinned `waddle-sdk-teleop`
wheel, which is the SAME shim built with `livekit` too. LiveKit's libwebrtc
dependency chain is ~690 MB of build, and an install whose job is to
supervise a policy should not pay for a teleop media plane it will never
open. Either way you `import waddle_sdk`: `waddle_sdk._native` picks the richer
core when it is installed, warns and falls back to the bundled one if the
two versions disagree (a half-upgraded environment), and honours
`WADDLE_NO_TELEOP=1`.

### Third-party content in the wheel

This repo is Apache-2.0 and the wheel carries one deliberate exception:
`waddle/robots/yam_data/` is a vendored snapshot of I2RT's YAM robot
description, shipped under its own MIT licence (`yam_data/LICENSE`, verbatim
from the source repo) and pinned to the upstream commit its README names. It
is data rather than code, text only — the STL meshes are not shipped — and it
earns its 16 KB twice: `waddle_sdk.robots.yam` hands it to a single-arm
declaration as `kinematics_urdf`, and `tests/test_yam_facts.py` compares every
constant in that module against the vendor's own numbers in it.

## Development

```bash
cd sdk
uv sync --dev && uv run pytest              # full build + test
uv run maturin develop --uv && uv run --no-sync pytest   # iterate on Rust
cargo clippy --manifest-path rust/Cargo.toml --all-targets -- -D warnings
cargo fmt --manifest-path rust/Cargo.toml --check
```

The shim is its own cargo workspace (`rust/`) with path-deps into
`../waddle-core/crates/*`; pyo3's `extension-module` feature lives only in
`[tool.maturin].features` so plain `cargo check/clippy` keep working.

### Connected builds

The transports are cargo features on the shim. `grpc` (the control plane:
`create_session(transport_url=, transport_token=)`) is in the DEFAULT
build, so `uv sync --dev`/`maturin develop` already carry it; `livekit`
(the media plane: `media_url=, media_token=` — the plane mints both
tokens, this SDK never does) is not, since it belongs to the
`waddle-sdk-teleop` companion:

```bash
uv run maturin develop --uv --features grpc,livekit    # the companion's flavour
cargo clippy --manifest-path rust/Cargo.toml --features grpc,livekit --all-targets -- -D warnings
```

Clippy must be clean featureless, `--features grpc`, and
`--features grpc,livekit`. A build that lacks a feature refuses the
matching kwarg rather than running offline in silence.
**`waddle_sdk._native.FEATURES` is the probe**, not `waddle_sdk._core.FEATURES`:
`_native` selects which core this process runs on and re-exports that
core's features, so on a `[teleop]` install `waddle_sdk._core.FEATURES` still
reports the bundled core's grpc-only set while the process is running the
teleop core. It is the only feature detection the Python layer does, and
the package reads it only at native construction to refuse a `transport=` or `media=` this build cannot honour.

The companion wheel is `sdk/teleop/pyproject.toml`: same
`manifest-path = "../rust/Cargo.toml"` (hence the same version by
construction), `module-name = "waddle_teleop._core"`, features
`grpc,livekit`. `sdk/pyproject.toml`'s `[tool.uv.sources]` points the
`teleop` extra at it so `uv` can resolve the lock; nothing builds it
unless the extra is actually installed. The extra's exact pin
(`waddle-sdk-teleop==X`) is the ONE version here that maturin cannot
derive from the manifest, so a version bump must edit it too —
`tests/test_features.py` holds it to `waddle_sdk.__version__` (and the two
projects to one manifest) rather than to memory.

## Hollow-frontend checklist

All claim, lease, handoff, gate, timeline, and timestamp decisions live in
`waddle-core` exactly once.

- `SiteSession` owns construction and cleanup, not authority. It never stores
  claim IDs, lease IDs, gate modes, or connection-derived permissions.
- `Run.step` marshals one native gate result into the owner envelope. It may
  index the part named by core and reject the whole owner command; it never
  decides which actor may command it.
- `SiteSession.hold` and `estop` call native priority paths. They do not invoke
  driver callbacks on the caller thread or fabricate timeline events.
- `SessionClock` remains the only production source of paired timestamps.
  Camera samples and runtime events copy native stamps.
- gRPC and LiveKit URLs/tokens are constructor data. Python consults only
  `_native.FEATURES`, a build fact, and never infers connection state.
- Calibration deprojects local aligned depth and submits one bounded point.
  It grants no motion and carries no image through the control plane.
- Driver modules may enforce the owner's physical envelope and lifecycle.
  They contain no claim, lease, hosted-task, workspace, or Metal logic.

Private `_testing` loopback hooks remain only for core conformance tests. They
are not exported by the package and are not an alternate application API.
