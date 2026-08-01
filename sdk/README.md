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
`waddle._core.FEATURES` reports which are present — it is the only feature
detection the Python layer does, and `waddle._native` is the only place
that reads it to decide anything.

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
- **Agent runs** (`waddle.agent`, `Session.agent`): a prompt goes in, an
  `AgentResult` comes out. The invite, its deadline, who may claim the
  episode, and what the caller's own ticks do meanwhile are all FSM rows;
  Python refuses early only when there is nobody to ask. The only other
  decision made here is *when to reattach and run Python's signal
  handlers*.
- Review heuristic: descriptors may validate *shape* ("must declare"),
  never *behavior*.

## Private test hooks

`waddle._testing` (`engage`/`release`/`push_teleop`/`reset_window_engage`/
`reset_window_complete`/`mark_done`/`frames`) requires
`waddle.init(_testing=True)`, which wires an in-process loopback media
plane. Private and unstable — it exists so the intervention, remote-reset-
window and agent-invited paths are testable with no plane at all. Each hook
stands in for one thing a plane would send: `mark_done`, for instance, is
an `EpisodeDirective{MARK_DONE}` — the only way to end a `waddle.agent()`
run in a test, since its caller is blocked and holds no episode handle.
