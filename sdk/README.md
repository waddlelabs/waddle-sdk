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
- Review heuristic: descriptors may validate *shape* ("must declare"),
  never *behavior*.

## Private test hooks

`waddle._testing` (`engage`/`release`/`push_teleop`/`reset_window_engage`/
`reset_window_complete`) requires `waddle.init(_testing=True)`, which
wires an in-process loopback media plane. Private and unstable — it
exists so the intervention and remote-reset-window paths are testable
without a control plane (the open-source runtime has no supervision-plane
transport wired yet; see `TeleopReset`/`AgentReset`'s docstrings).
