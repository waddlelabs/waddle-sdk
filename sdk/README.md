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
`manifest.jsonl`, no control plane required.

Connect a supervision plane with `waddle.init(transport=waddle.Grpc(url,
token))` and the same loop is supervised: a teleoperator can intervene, a
reset window can be handed out — and you can hand over a whole episode:

```python
result = waddle.agent("clear the table and stack the cups")
print(result.outcome, result.episode_id, result.detail)
```

`waddle.agent()` blocks until the episode reaches an outcome. The invited
agent claims it through the very same intervention machinery a
teleoperator uses, so its actions arrive at the `send` verb you already
registered; your own `gate()` ticks would not dispatch in such an episode
anyway (FSM.md E24), which is why this call takes the thread instead of
handing back an episode handle. An ask nobody answers comes back
`outcome == "abort"` at the deadline — a result, not an exception.

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

rig = yam.bimanual(workspace=WORKSPACE_M, gripper_limits=(0.1, 1.7), sim=True)
with rig.session("towels", transport=waddle.Grpc(url, token)) as session:
    result = waddle.agent("stack the cups")
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

`rig.session(...)` exists for the two ends every hand-written version gets
wrong at least once. `__enter__` opens the drivers **inside the `with`** —
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

### Every piece is usable alone

The rig is composition sugar over pieces that each stand on their own —
which is the design, not an accident:

| piece | what it is | alone |
|---|---|---|
| `yam.declaration(...)` / `rig.robot()` | the `waddle.Robot` this rig registers | hand it to `waddle.init` yourself; nothing else here is involved |
| `rig.arms()` | one `base.Arm` per declared part, each an owner's envelope over a driver | the hardware opens **here**, never at the factory call |
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
(`waddle.agent()` vs `waddle.rollout()`), never a construction one.

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

The rest is a facts table and a factory:

```python
import waddle
from waddle.robots import base

FACTS = {                                  # your vendor's numbers, each with
    "joints": ("boom", "stick", "grip"),   # its source in the comment beside it
    "limits": ((-1.0, 1.0), (-1.5, 1.5), (0.0, 1.0)),
    "step_caps": (0.10, 0.10, 0.25),       # largest jump one command may make
    "rate_hz": 20.0,
    "home": (0.0, 0.0, 1.0),
}

def crane(*, sim: bool = False, posture: str = "supervised") -> base.Rig:
    space = waddle.JointSpace(
        joints=[
            waddle.Joint(name=n, min_position=lo, max_position=hi)
            for n, (lo, hi) in zip(FACTS["joints"], FACTS["limits"])
        ],
        rate_hz=FACTS["rate_hz"],
        chunking=waddle.Chunking(horizon=1, replan="immediate", interp="hold"),
    )

    def build_arms() -> dict[str, base.Arm]:      # the bus opens HERE
        driver = base.SimDriver(
            FACTS["home"],
            lower=[lo for lo, _ in FACTS["limits"]],
            upper=[hi for _, hi in FACTS["limits"]],
            step_caps=FACTS["step_caps"],
            rate_hz=FACTS["rate_hz"],
        ) if sim else MyVendorDriver(...)         # your ten members
        return {
            "": base.Arm(
                part="", driver=driver,
                joint_names=FACTS["joints"], joint_limits=FACTS["limits"],
                step_caps=FACTS["step_caps"], rate_hz=FACTS["rate_hz"],
                home_values=FACTS["home"],
            )
        }

    return base.Rig(
        declaration=waddle.Robot(name="crane", action_space=space),
        build_arms=build_arms, rate_hz=FACTS["rate_hz"], posture=posture,
    )
```

That is the whole of a second vendor module, and it is a **test** rather than
a docs snippet: `tests/test_robots_base.py` carries this same toy vendor and
drives it end to end through a real session — declaration, envelope, gate,
pump, MCAP read-back — with nothing vendor-specific in `base` to help it. The
claim "the base layer carries all of the behaviour" is only true while that
keeps passing.

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
pip install waddle-sdk              # control plane included
pip install 'waddle-sdk[teleop]'    # + the LiveKit media plane
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
earns its ~15 KB twice: `waddle.robots.yam` hands it to a single-arm
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
