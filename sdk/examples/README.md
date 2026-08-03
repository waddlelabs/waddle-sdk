# Examples

## `toy_robot.py` — a whole robot integration in one file

A 6-dof arm with a parallel gripper and one camera, running the rollout
loop at 20 Hz. The robot itself is a small kinematic simulator inside the
file, so the program is self-contained and needs nothing but `waddle-sdk`
and numpy; the Waddle-facing half is exactly what you would write for a
real machine.

It shows, in one place:

- a full [`Robot`](../python/waddle/descriptors.py) declaration — per-joint
  limits, a parallel gripper declared in **metres** (`0.0` open, `0.04`
  closed, deliberately not 0/1), a generated 6-joint URDF, and a camera
  with a `StreamPolicy`;
- the five-verb `Control` contract, three of whose verbs this robot
  declares — `send`, `hold`, `estop` — which is how anything Waddle grants
  a lease to actually moves the robot;
- the six-line loop: `ep.gate(action, obs)` returns your action, a
  different action, or `None` — and, when something intervened,
  `ep.last_gate.gripper` carries the claimant's grasp already converted
  into the metres the robot declared;
- `session.publish_frame(...)` and `session.report_proprio(...)` every
  tick;
- a scripted `pre_reset` hook, run before every episode — which declines,
  rather than vouching for a scene it did not reset, while the simulated
  e-stop is latched (only a human clears that, never the reset flow);
- `waddle.agent(prompt)` — handing a whole episode to Waddle.

### Run it offline

No configuration, no network, nothing to sign up for:

```bash
cd sdk
uv sync --dev
uv run python examples/toy_robot.py
```

Every episode lands in `./toy-recordings/` as a sidecar JSON plus an MCAP
of the full timeline. Nothing is supervised — there is no plane at the
other end — but no code path is stubbed: the gate really gates, the
recorder really records. `Ctrl-C` stops it cleanly.

Useful knobs (all have `--flag` equivalents, see `--help`):

```bash
WADDLE_TOY_EPISODES=3 \
WADDLE_TOY_EPISODE_SECONDS=2 \
WADDLE_TOY_RECORDING_DIR=/tmp/toy \
uv run python examples/toy_robot.py
```

### Point it at a supervision plane

One environment variable turns the same program into a supervised session:

```bash
WADDLE_TOY_TRANSPORT=http://<plane-host>:<port> \
WADDLE_TOY_TOKEN=<the plane's token for this session> \
uv run python examples/toy_robot.py
```

Now the session's timeline goes up as it happens, and the plane can
intervene: it grants a claim, the lease hands over, and the actions
arriving at the program's `send` verb are the claimant's instead of the
policy's. Your loop keeps calling `gate()` throughout and keeps getting a
truthful answer — including `None`, meaning "do not send" — which is the
whole point of routing every action through it.

The token is the plane's own credential for the session; the SDK never
mints one. A plane that asks for no credential (a local development plane)
needs no `WADDLE_TOY_TOKEN` — leaving it unset and passing it empty mean
the same thing, so a harness can forward `VAR=${MAYBE_UNSET}` without
knowing which case it is in.

The declared camera also carries `still_fps=2`, so with a plane connected
the program samples 2 JPEG stills per second onto the **control** plane.
That is the one bounded exception to "no pixels on the control plane" — it
exists so a Waddle-hosted agent can see the scene without any media plane
at all, and it is capped by that declaration rather than by hope.

### Hand an episode to Waddle

```bash
WADDLE_TOY_MODE=agent \
WADDLE_TOY_TRANSPORT=http://<plane-host>:<port> \
WADDLE_TOY_TOKEN=<token> \
WADDLE_TOY_PROMPT="pick up the block" \
uv run python examples/toy_robot.py
```

After one warm-up rollout the program calls `waddle.agent(prompt)`, which
opens an episode Waddle drives and blocks until it ends. The robot's own
20 Hz loop moves to a background thread for the duration — the arm still
integrates the agent's commands and the camera still feeds the stills the
agent perceives through — and the program prints:

```
[toy] agent result <outcome> episode=<id>
```

then exits 0 on success. An unanswered invite (nobody claimed the episode
before the deadline) and a declined task both come back as `abort` with a
`detail`, never as a crash. Agent mode needs a plane, and says so instead
of pretending: with no `WADDLE_TOY_TRANSPORT` it exits immediately with a
message rather than running a warm-up rollout first.

### What `[teleop]` adds

```bash
pip install 'waddle-sdk[teleop]'
```

The teleop companion adds the **media plane**: live WebRTC video from your
declared cameras to a human teleoperator, plus the teleop input stream
coming back. It is a separate install because it carries libwebrtc — ~690 MB
of build, and about 4.5x the wheel — which an install that only supervises a
policy has no reason to pay for. Point the example at one with:

```bash
WADDLE_TOY_MEDIA=wss://<livekit-host> \
WADDLE_TOY_MEDIA_TOKEN=<room token from the plane> \
uv run python examples/toy_robot.py
```

and the camera's declared `uplink` (10 fps here) becomes a real video
track. Without that extra installed, asking for a media plane is a clean
error naming the install — never a session that quietly runs with no video.

Note the two paths are different things and do not substitute for each
other: video on the media plane is for a **human** watching and driving;
the bounded stills on the control plane are for an **agent** perceiving.
The example declares both, and each one activates only when its half is
configured.

### Status lines

Everything the program wants you (or another process) to see is prefixed
`[toy] ` and flushed immediately, so it can be driven from a harness:

| line | meaning |
|---|---|
| `[toy] session up ...` | the session is open; the configuration is echoed |
| `[toy] pre_reset '<task>'` | the scripted scene reset ran |
| `[toy] pre_reset refused: e-stop latched ...` | the reset declined instead of vouching; clear the e-stop at the robot |
| `[toy] rollout <n> start id=<episode> task=...` | an episode opened |
| `[toy] rollout <n> done <outcome>` | it reached a terminal outcome |
| `[toy] agent invite prompt=... timeout_s=...` | the invite went out |
| `[toy] agent result <outcome> episode=<id>` | the agent run finished |
| `[toy] shutdown` | core threads joined, recordings flushed |

### Exit codes

| code | meaning |
|---|---|
| 0 | finished the requested rollouts, or the agent run succeeded, or `Ctrl-C` |
| 1 | agent mode: the run did not succeed (`abort`, `failure`, …) |
| 2 | agent mode was asked for with no supervision plane configured |
