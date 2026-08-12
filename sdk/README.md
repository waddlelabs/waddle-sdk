# waddle-sdk (Python)

The Python frontend for Waddle — supervision for real-world robot policy
rollouts. Tier-1 integration is the six-line loop:

```python
import waddle

waddle.init(
    project="towels",
    robot=waddle.Robot(
        name="arm",
        action_space=waddle.JointSpace(joints=[f"j{i}" for i in range(7)], rate_hz=50),
    ),
    control=waddle.Control(send=robot.command, hold=robot.hold, resume=robot.resume),
    recording_dir="./recordings",
)

with waddle.rollout(task="fold the towel") as ep:
    while not ep.done:
        obs = get_obs()
        action = policy(obs)
        action = ep.gate(action, obs)   # the only touch point
        if action is not None:
            send(action)
```

`ep.gate()` always answers **"what you should send, or `None` if you must
not send"**: Pass returns your exact object, Substitute/Blend a fresh
float64 ndarray, Noop and Hold return `None`. Exiting the `with` block
before a terminal outcome terminates the episode `abort` — never success.

A robot that declares named parts (`waddle.Composite` — a bimanual cell,
say) can be intervened on ONE part at a time, so on such a declaration
Substitute/Blend comes back **keyed by part** instead: `{"right": ndarray}`
for a command addressing the right arm — the parts not in the dict are
commanded nothing, "move this part, hold the rest" — and every declared
part, sliced by the declared layout, for a whole-robot one.
`ep.last_gate.part` names the addressed part (`None` = the whole robot),
and a dispatched chunk's step values follow the same rule. Report state
back the same way: `session.report_proprio(part="right",
joint_pos=[...])`. Declarations without parts are untouched.

Works fully offline: with `recording_dir` set, every episode lands as a
sidecar JSON + MCAP (`/waddle/actions`, `/waddle/observations`) plus a
`manifest.jsonl`, no control plane required. Add generic correlation context
with `waddle.rollout(task, task_metadata={"trace_id": ...})`; it is stored
in the sidecar but never used for capability or authority. `session.stamp()`
returns one immutable `SessionStamp(session_ns, unix_ns)` whose two clocks were
captured as a pair, so external evidence can be located on the session timeline
without deriving an epoch twin later.

Connect a supervision plane with `waddle.init(transport=waddle.Grpc(url,
token))` and the same loop is supervised: a teleoperator can intervene, a
reset window can be handed out — and you can hand over a whole episode:

```python
result = waddle.agent("clear the table and stack the cups")
print(result.outcome, result.episode_id, result.detail)
```

`waddle.agent()` blocks until the episode reaches an outcome. It is the
one-shot synchronous surface over the same hosted task execution used by
named UI conversations. The invited agent claims it through the very same
intervention machinery a
teleoperator uses, so its actions arrive at the `send` verb you already
registered; your own `gate()` ticks would not dispatch in such an episode
anyway (FSM.md E24), which is why this call takes the thread instead of
handing back an episode handle. An ask nobody answers comes back
`outcome == "abort"` at the deadline — a result, not an exception.

## In-process browser UI (`waddle.ui()`)

After `waddle.init()`, start the session's local control and inspection page
from the same process:

```python
dashboard = waddle.ui(
    joint_step_rad=0.01,
    linear_step_m=0.005,
    angular_step_rad=0.02,
)
print(dashboard.url)
```

`waddle.ui()` starts one background server for the active session, bound to
`127.0.0.1` on an OS-selected port. Repeated calls return the same `UIHandle`;
`dashboard.close()` (or leaving `with waddle.ui() as dashboard:`) stops it,
and `waddle.shutdown()` always closes it before core teardown. There is
deliberately no `waddle ui` command: the page is useful because it is attached
to this process's native session and the exact `Control` callbacks the robot
owner registered.

The printed URL contains a fresh 256-bit token in its fragment. The fragment
does not reach the HTTP server; the bundled application presents it in the
required `X-Waddle-Token` header on every data/control request, alongside the
custom request marker. The server enforces its exact loopback `Host` and
same-origin `Origin`, has no CORS path, bounds request bodies, sets
`Cache-Control: no-store`, CSP and no-referrer headers, and ships no web
framework or image dependency. Treat the URL as a bearer secret: the page has
a permanent warning because it exposes lab imagery and real motion controls.

State is the native session's authoritative status, rendered directly. E-stop
sets the existing native priority e-stop path and reports **requested**, never
confirmed; it sends no control-plane request. Before jog is enabled, the site
operator must press **Take Local Control**. That asks core for an exclusive
remote-to-local handoff; a refusal leaves jog disabled, and Python/JavaScript
never infer that remote authority has gone away. Once the handoff completes,
holding the browser deadman heartbeats every 250 ms, and core releases its
local claim after one second without a heartbeat or immediately on release,
page close or SDK shutdown. Joint jog derives one absolute target from the
freshest reported proprio; Cartesian jog follows the declared delta
frame/rotation convention. Missing proprio and unsupported or opaque spaces
are typed refusals. Each accepted press is one normal action chunk through
`Control.send`, so the owner's envelope still accepts or refuses the whole
command—nothing in the UI clamps it. The three positive finite step sizes may
be changed in the page for this UI run only.

The camera view is the bounded latest RGB frame per declared camera, drawn
directly to a canvas. A managed RGB-D rig keeps the pixel-aligned depth beside
that exact frame in the customer process. Calibration clicks carry the frame
sequence shown by the UI; the SDK rejects a stale sequence, deprojects the
pixel locally from the declared intrinsics, and submits only bounded IDs,
timestamps and the 3-D point. Image and depth arrays never enter the
calibration message. Recordings are read from `manifest.jsonl`: the list shows
task/outcome/timestamps, and downloads resolve only the manifest-named
sidecar/MCAP files beneath `recording_dir`; there is no playback or MCAP web
dependency.

With a connected plane the page also exposes durable named task sessions:
create a conversation, watch public-safe live output, send a message
or interjection, and interrupt the active turn. Internal prompts, tool
names/payloads, traces and private errors are not task events. The equivalent
Python handle is `waddle.task_session(name, task_session_id=...)`; its bounded
`history` is the events observed through that handle, while the plane-issued
ID resumes the durable conversation. A resumed handle requests the first
bounded history page immediately; `refresh()` requests the next suffix using
the last durable cursor:

```python
task = waddle.task_session("bench setup")
while task.task_session_id is None:
    task.events(timeout_s=20.0)

task.message("Use the wrist camera for the next check")
for event in task.events(timeout_s=20.0):
    print(event)

task.interject("Leave the red fixture in place")
# task.interrupt() stops the active hosted turn.
```

resumed = waddle.task_session("bench setup", task_session_id=task.task_session_id)
resumed.events(timeout_s=20.0)  # first durable page requested by construction

Workspace export is similarly metadata-only on the session stream. A request
selects allowlisted graphs and calibrations; its ready event contains an
opaque, one-time `download_ref` for the plane's separate authenticated artifact
endpoint, never archive bytes:

```python
artifact = waddle.request_workspace_artifact(
    graph_ids=["clear-table"],
    calibration_names=["bench-v3"],
)
for event in artifact.events(timeout_s=20.0):
    print(event.get("download_ref"))
```

Execution choices are **Hosted** plus any explicitly installed **Local**
integration. Optional local integrations register the versioned
`waddle.execution.v1` entry-point group. Discovery lists metadata without
importing them; only selecting one in the UI, or explicitly calling its
`ExecutionBackend.load()`, loads its package:

```python
local = next(backend for backend in waddle.execution_backends() if backend.local)
integration = local.load()  # the sole optional local-runtime import boundary
```

A missing transport, unnegotiated service, dead plane or absent hosted worker
is an explicit unavailable status. Local state, e-stop, camera viewing,
recordings, and the handoff-controlled jog path continue to work.

All of the above in one runnable file — a simulated 6-dof arm with a
camera, the loop, and `waddle.agent()`, offline by default — is
[`examples/toy_robot.py`](examples/): `uv run python
examples/toy_robot.py`.

## Robot modules (`waddle.robots`)

Everything above is what you write for a robot Waddle has never heard of.
For one it has, `waddle.robots.<vendor>` carries that machine's facts and
its driver, and [`waddle.robots.base`](python/waddle/robots/base.py) carries
everything about driving a robot that is *not* a vendor fact: the kinematic
twin, the envelope seam, the e-stop latch and the console gesture that
clears it, the reporting loop, and a session's two ends. A vendor module is
then facts + driver + factory and nothing else. The subpackage is opt-in —
`import waddle` imports none of it.

```python
import waddle
from waddle.robots import yam

rig = yam.bimanual(
    workspace=WORKSPACE_M,
    gripper_limits=(0.1, 1.7),
    sim=True,
)
waddle.init(
    "towels",
    rig=rig,
    transport=waddle.Grpc(url, token),
    recording_dir="./recordings",
)
try:
    dashboard = waddle.ui()
    result = waddle.agent("stack the cups")
finally:
    waddle.shutdown()
```

`sim` is EXPLICIT either way, never inferred: no code path try-imports a
vendor package to decide what you meant. Driving real YAM hardware needs
I2RT's own package, which is not a dependency of this SDK and cannot be an
extra of it — PyPI rejects direct references, and the tree behind it is not
something an install that only supervises a policy should resolve. It is a
documented command instead: `yam.I2RT_INSTALL`, BUILT from `yam.I2RT_PIN` so
it cannot drift from the commit those facts are stated against, printed by
the driver when the import fails, and quoted in the [root
README](../README.md). Importing `waddle.robots.yam` needs none of it.

`waddle.init(rig=...)` and `rig.session(...)` share one `RigSession`
lifecycle. The former is useful when a process already owns shutdown; the
latter is its context-manager spelling. `rig=` is mutually exclusive with
the legacy `robot`/`control` pair, so hardware is never registered twice.
Opening starts the arms, optional cameras, proprio pump and capture pumps;
`waddle.shutdown()` (or `RigSession.__exit__`) stops blocked capture, joins
the pumps, finalizes the recording and closes everything deterministically.

The context-manager spelling exists for the two ends every hand-written
version gets wrong at least once. `__enter__` opens the drivers **inside the `with`** —
so a bus that will not open unwinds structurally, and a rig that opens half
its arms closes them rather than leaving them energized under a vendor's own
re-send — registers the verbs, calls `waddle.init`, and starts the reporting
pump. `__exit__` runs whatever the body did: on live hardware it holds (still
reporting) until a human says the machine is parked — closing stops the
vendor's command re-send, and the motors' own watchdog then drops all torque
from wherever the mission left the arms — then stops the pump, shuts the
session down and closes the drivers. (A `Ctrl-C` skips that hold: whoever
typed it is already at the machine. A twin returns at once.)
**Finalizing the recording is
no longer a `finally:` you remembered to write.** The pump is always on, not
only for an agent run, so your own loop only gates and applies — there is no
interleaved robot tick to forget, and a session whose thread is blocked
inside `waddle.agent()` keeps reporting.

### Managed cameras (RGB and RGB-D)

A camera driver is structural: it implements `capture() -> CameraFrame` and
an idempotent `close()` that unblocks a pending capture. `CameraFrame.rgb` is
a contiguous `uint8[height, width, 3]` array; optional `depth` is a
pixel-aligned `uint16[height, width]` array. The SDK copies reusable vendor
buffers when necessary and exposes immutable `CameraSample`s stamped once
with the session-monotonic/Unix clock pair plus a monotonically increasing
`frame_sequence`.

```python
from waddle.robots import base


class SiteCamera:
    def capture(self) -> waddle.CameraFrame:
        return waddle.CameraFrame(
            rgb=capture_rgb_uint8(),
            depth=capture_aligned_depth_uint16(),
        )

    def close(self) -> None:
        close_device_and_unblock_capture()


rig = base.Rig(
    declaration=robot_with_declared_wrist_camera,
    build_arms=open_arms,
    build_cameras=lambda: {"wrist": SiteCamera()},
    rate_hz=50.0,
)
```

The camera names returned by `build_cameras` must exactly equal the robot
declaration's camera names. Capture publishes only RGB through the existing
declared camera paths and retains only the latest correlated local RGB-D
sample for calibration. `waddle.calibration_click(calibration_id, sample_id,
camera, frame_sequence, x, y)` requires rectified depth plus declared
intrinsics and `frame_id`; it refuses stale frames, invalid depth and non-zero
distortion rather than guessing.

The built-in `OrbbecDriver` and `RealSenseDriver` adapters are imported lazily.
Their vendor SDKs are optional extras; importing `waddle` or
`waddle.cameras` never imports either vendor package.

### Every piece is usable alone

The rig is composition sugar over pieces that each stand on their own —
which is the design, not an accident:

| piece | what it is | alone |
|---|---|---|
| `yam.declaration(...)` / `rig.robot()` | the `waddle.Robot` this rig registers | hand it to `waddle.init` yourself; nothing else here is involved |
| `rig.arms()` | one `base.Arm` per declared part, each an owner's envelope over a driver | the hardware opens **here**, never at the factory call |
| `rig.cameras()` | one structural `CameraDriver` per declared camera | capture opens **here** and aligned depth remains local |
| `rig.control(arms)` | the posture as `waddle.Control` verbs | `send=` replaces the envelope wholesale (below) |
| `rig.pre_reset(arms)` | the default scene reset — refuses a latched scene, homes a twin, vouches for metal without moving it | pass your own callable to `waddle.init` instead |
| `rig.pump(session, arms)` | `base.RobotPump` reporting every part at the declared rate | `RobotPump(tick, rate_hz)` runs any tick callable you write |
| `rig.session(project, ...)` | all of the above, with the two ends | — |

Sugar that cannot be reproduced by hand is a wall, so that claim is a test
rather than a promise: `tests/test_yam_session.py` wires `yam.declaration()`,
drivers, `base.Arm`, `waddle.Control`, a plain `waddle.init`, the console
recovery and a `RobotPump` by hand and asserts the session that opens is
byte-identical to `rig.session()`'s.

### The envelope is yours

Waddle never provides the envelope — the owner's hard safety is the owner's
(see [GLOSSARY.md](../waddle-protocol/docs/GLOSSARY.md)). What ships here is
a **parameterized default built from your own numbers**, on the one object
every path to the hardware crosses: `base.Arm` checks width, finiteness, your
declared joint limits, per-step travel against where the unit actually is,
and — only when the rig was given forward kinematics — the FK'd TCP inside
your declared workspace box. It **rejects, never clamps**: a failing target
is refused WHOLE, the unit holds, and one bounded line names the check. A
clamped command is a command nobody wrote, executed faithfully.

The factory provides the check; you own the envelope by choosing and
parameterising it — or by replacing it. Pass `send=` to `rig.session(...)`
(or `rig.control(arms, send=...)`) and your callable is the whole envelope,
while you keep the twin, the latch, the loop and the console recovery.

A vendor module's shipped limits are a **default, not a ceiling on what you may
declare**: `joint_limits=` states the intervals *your* machine has, and they
become both what the envelope enforces and what the declaration carries to the
plane — one number, so a teleoperator or a Waddle-hosted agent is shown the
range the rig really has. The usual reason to need it is a motor zeroed a few
milliradians off: it rests just outside a theoretical range, and a hold of its
own measured pose is then a command the envelope refuses forever. Any row wider
than the shipped model's is reported at every start, never silent. (The
directional fact gate below binds what this wheel *ships*, which is a claim
about a YAM; what a particular rig accepts is the owner's to state.)

Forward kinematics is opt-in and its absence is named rather than filled in:
an arm built without `fk` reports joint positions only (`ee_pose()` answers
`None` instead of inventing a frame), and a workspace box declared without
one is refused at construction rather than silently checking nothing.

### Postures

`posture=` is the one construction-time choice, and it maps to which control
verbs the session registers — nothing else:

| `posture` | verbs registered | what that buys |
|---|---|---|
| `"monitor"` | the owner's `estop` alone | Nothing may command this robot: the session says so on the wire instead of accepting motion it intends to drop. Where a vendor has a compliant mode the driver is *constructed* that way too (a `monitor` YAM opens in zero gravity and then refuses to write), so it is a property of the object rather than of a flag somebody remembered to check. No `hold` — waddle-core reads a registered `hold` as a live engage path and refuses any session offering one with no `send` — and no media plane, which carries the teleoperator's stream as well as the video and so IS an intervention path, refused by the same rule. Watching is undiminished: `transport=` uplinks proprioception and each camera's declared low-rate stills, `recording_dir=` keeps the full-rate archive. |
| `"supervised"` | `send`, `hold`, `estop` | The ordinary posture: a teleoperator, a reset agent or a Waddle-hosted agent may drive this robot — through the owner's envelope. |

A posture is **not** an authority decision and adds none: who may command a
robot, when, and under what claim is waddle-core's, identical under both.
Whether a rollout is agent-driven or windowed stays a call-site choice
(`waddle.agent()` vs `waddle.rollout()`), never a construction one. For that
authority story in full (which phase hands the lease to whom, what `gate()`
returns while they hold it, and what a `monitor` session therefore cannot do),
see [`docs/lease-lifecycle.md`](../docs/lease-lifecycle.md).

The same rule that governs the rest of this package governs `robots/`: it is
owner-side code that ships in the frontend, it enforces the OWNER's envelope
(limits arithmetic on the owner's own numbers), and it asks nothing about who
may command what. The part an action addresses is the core's answer —
indexed, never validated. See the hollow-frontend checklist below.

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
import waddle
from waddle.robots import base

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
    space = waddle.JointSpace(
        joints=[
            waddle.Joint(name=name, min_position=lo, max_position=hi,
                         max_effort=TOY_FACTS["max_effort_nm"])
            for name, (lo, hi) in zip(TOY_FACTS["joints"], TOY_FACTS["limits"])
        ],
        rate_hz=TOY_FACTS["rate_hz"],
        chunking=waddle.Chunking(horizon=1, replan="immediate", interp="hold"),
    )

    def build_arms() -> dict[str, base.Arm]:      # the bus opens HERE
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
        declaration=waddle.Robot(
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
`waddle.robots.yam` does. Either way it is constructed inside `build_arms` and
never at import, so the factory call opens no bus and starts no thread: a
program may build a rig and then decide not to run it.

For more than one part, declare a `waddle.Composite` and return one `Arm` per
part name (declaration order IS the concatenated action layout). Add `fk=` to
each `Arm` if you have forward kinematics, and only then a `workspace=` box.
Ship the source that can gate a fact next to the facts — `waddle.robots.yam`
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
pip install 'waddle-sdk[cameras]'         # + both camera adapters
pip install 'waddle-sdk[teleop]'          # + the LiveKit media plane
pip install 'waddle-sdk[cameras,teleop]'  # camera adapters + LiveKit media plane
```

Two distributions from one source tree (the psycopg / psycopg-binary
shape): `waddle-sdk` bundles the core built with the `grpc` control
transport, and the `teleop` extra adds the exact-pinned `waddle-sdk-teleop`
wheel, which is the SAME shim built with `livekit` too. LiveKit's libwebrtc
dependency chain is ~690 MB of build, and an install whose job is to
supervise a policy should not pay for a teleop media plane it will never
open. Either way you `import waddle`: `waddle._native` picks the richer
core when it is installed, warns and falls back to the bundled one if the
two versions disagree (a half-upgraded environment), and honours
`WADDLE_NO_TELEOP=1`.

### Third-party content in the wheel

This repo is Apache-2.0 and the wheel carries one deliberate exception:
`waddle/robots/yam_data/` is a vendored snapshot of I2RT's YAM robot
description, shipped under its own MIT licence (`yam_data/LICENSE`, verbatim
from the source repo) and pinned to the upstream commit its README names. It
is data rather than code, text only — the STL meshes are not shipped — and it
earns its 16 KB twice: `waddle.robots.yam` hands it to a single-arm
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
**`waddle._native.FEATURES` is the probe**, not `waddle._core.FEATURES`:
`_native` selects which core this process runs on and re-exports that
core's features, so on a `[teleop]` install `waddle._core.FEATURES` still
reports the bundled core's grpc-only set while the process is running the
teleop core. It is the only feature detection the Python layer does, and
two places read it — `_native` itself, to pick a core, and `init()`, to
refuse a `transport=`/`media=` this build cannot honour.

The companion wheel is `sdk/teleop/pyproject.toml`: same
`manifest-path = "../rust/Cargo.toml"` (hence the same version by
construction), `module-name = "waddle_teleop._core"`, features
`grpc,livekit`. `sdk/pyproject.toml`'s `[tool.uv.sources]` points the
`teleop` extra at it so `uv` can resolve the lock; nothing builds it
unless the extra is actually installed. The extra's exact pin
(`waddle-sdk-teleop==X`) is the ONE version here that maturin cannot
derive from the manifest, so a version bump must edit it too —
`tests/test_features.py` holds it to `waddle.__version__` (and the two
projects to one manifest) rather than to memory.

## Hollow-frontend checklist (review gate for every change here)

All claim/lease/handoff/timeline logic lives in waddle-core exactly once.
Concretely, in this package:

- No Python variable ever holds gate mode, claim id, lease id, or engage
  stage; the Noop/Hold→`None` mapping marshals a core decision, it is not
  an `if` about claims.
- No handoff math (blend, chunk-boundary waits, HOLD_FIRST sequencing,
  retake/successor logic) — Python only *declares* `Handoff`.
- No FSM/timeline: `ep.done`/`ep.outcome` are single reads of the core
  mirror; no Python timers, stall detection, or state mirroring.
- No tripwire evaluation, reset verification, grant negotiation/demotion
  logic; no provenance computation (strings surfaced verbatim); no
  recording writes (the core reducer owns all recording).
- **Reset API** (`TeleopReset`/`AgentReset`, `init(pre_reset=, post_reset=,
  reset_verification=)`, `rollout(pre_reset=, post_reset=)`): the markers
  and callables are pure config — which reset *kind* string names a
  marker's type, and normalizing a hook's return shape to `(bool,
  Optional[bool])`, are type dispatch and input-shape validation, not
  reset decisions. Every actual behavior (which `ResetStrategy` runs, how
  `reset_verification` gates the RESETTING→READY transition, the window's
  claim/lease/gate-mode sequencing, `post_reset_failed`'s permanence, the
  outcome-pinning at POST_RESET entry) lives in waddle-core; Python never
  branches on any of it.
- **Connected build** (`waddle.Grpc`/`waddle.LiveKit`, the
  `transport_url`/`media_url` kwargs, `FEATURES`): URLs and tokens are
  config handed to core constructors; nothing here inspects a connection.
  Feature detection answers "can this build do it at all", never "what
  should happen now" — no branch on plane state, negotiated flags, or
  connectivity. `waddle._native`'s core selection is packaging, decided
  once at import, and the only state `init` records about a session is
  *whether a plane was declared at all* (a declaration fact, not plane
  state).
- **Feature raises key off `_native.FEATURES`, never a try-import.** A
  `try: ... except (ImportError, AttributeError)` reads as "can this build
  do it?" and answers "did this particular call happen to fail?", so a
  genuine runtime error becomes a not-compiled message and the user chases
  the wrong thing. `FEATURES` is a build fact the core states outright.
  Teleop-only surfaces must name the extra in the error text (`pip install
  'waddle-sdk[teleop]'`): the caller cannot act on "not compiled" alone,
  and the whole point of the two-wheel split is that this is a one-command
  fix rather than a rebuild.
- **Agent runs** (`waddle.agent`, `Session.agent`): a prompt goes in, an
  `AgentResult` comes out. The invite, its deadline, who may claim the
  episode, and what the caller's own ticks do meanwhile are all FSM rows;
  Python refuses early only when there is nobody to ask. The only other
  decision made here is *when to reattach and run Python's signal
  handlers*.
- **Local UI** (`waddle.ui`, `UIHandle`): Python owns loopback HTTP security,
  static assets, positive-finite presentation settings and safe manifest path
  resolution only. It renders `Session.status()` and marshals typed native
  operations. E-stop priority, exclusive remote-to-local handoff,
  jog/deadman timing, claim engage/release, declared-space action construction,
  service negotiation/connection scoping and every refusal live in core;
  JavaScript and Python never infer authority.
- **Managed rig and cameras** (`init(rig=...)`, `CameraDriver`): Python owns
  hardware construction, capture/reporting pump lifetime, deterministic close,
  array shape validation and local RGB-D deprojection. Core still owns session
  time, intake and authority. Calibration submits only the selected frame's
  bounded 3-D measurement; no Python branch decides whether motion may occur.
- **Hosted services and local integration** (`TaskSession`, calibration and
  artifact handles, `execution_backends`): Python validates bounded public
  shapes and tracks client cursors. Core owns negotiation, correlation and
  handoff. Optional local execution enters only through the versioned
  `waddle.execution.v1` entry point after explicit selection.
- **Part-keyed payloads** (`Composite` sessions: dict-by-part `gate()`
  returns and `Chunk` step values, `report_proprio(part=)`): the layout is
  read off the customer's own declaration — each part's name and width, in
  declaration order — and applied as arithmetic to a row the core already
  decided. Which part an action addresses is the core's answer
  (`Step.part` / `OwnedAction.part`, honored only under a negotiated
  `waddle.v0.parts`); Python never decides it, never validates a part name
  (the core refuses an undeclared one), and must never grow an `if` about
  which part may command what — that is authority, and v0's lease is
  whole-robot single-writer.
- Review heuristic: descriptors may validate *shape* ("must declare"),
  never *behavior*.

## Private test hooks

`waddle._testing` (`engage`/`release`/`push_teleop`/`push_chunk`/
`reset_window_engage`/`reset_window_complete`/`mark_done`/`frames`) requires
`waddle.init(_testing=True)`, which wires an in-process loopback media
plane. Private and unstable — it exists so the intervention, remote-reset-
window and agent-invited paths are testable with no real plane at all. Each
hook stands in for one thing a plane would send: `mark_done`, for instance,
is an `EpisodeDirective{MARK_DONE}` — the only way to end a
`waddle.agent()` run in a test, since its caller is blocked and holds no
episode handle. Because it stands in for a plane, `_testing=True` counts as
a plane declaration for `waddle.agent()`'s "nobody to ask" refusal, exactly
as a `transport` does.
