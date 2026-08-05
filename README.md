# waddle-sdk

The open half of **Waddle** — a supervision layer for real-world robot policy
rollouts. Waddle attaches to your existing stack (robot, cameras, policy server,
control loop) and owns everything *around* the policy's decisions: watching,
intervening, resetting, judging, and improving.

> Weights & Biases instrumented your training loop; Waddle instruments your
> deployment loop.

This monorepo hosts the open artifacts:

| Artifact | Status | What it is |
|---|---|---|
| [`waddle-protocol/`](waddle-protocol/) | v0 | The standard: protobuf schemas, the episode/claim/lease FSM spec, the sidecar schema, conformance fixtures. Implementable without waddle-core — that is the point. |
| [`waddle-core/`](waddle-core/) | 0.1 | The Rust reference implementation: episode/claim/lease FSMs, the gate, tripwires, sidecar + MCAP recording, codecs, control-plane client. Emits the `libwaddle` C ABI. |
| [`sdk/`](sdk/) | 0.1 | The Python frontend (`waddle-sdk`): the six-line rollout loop, local recording, the connected surface, and the opt-in robot modules (`waddle.robots`). A hollow frontend — every decision lives in waddle-core. |
| `waddle-proxy`, `waddle-cpp`, `waddle_ros` | planned | Further hollow frontends over waddle-core, per the design doc's artifact family. |

The design rationale (including three adversarial stress-test passes) lives at
[`waddle-protocol/docs/rationale/waddle_api_design_doc.md`](waddle-protocol/docs/rationale/waddle_api_design_doc.md).
The normative docs are
[`GLOSSARY.md`](waddle-protocol/docs/GLOSSARY.md),
[`FSM.md`](waddle-protocol/docs/FSM.md), and
[`VERSIONING.md`](waddle-protocol/docs/VERSIONING.md).

## Build quickstart

```bash
cd waddle-core
cargo test --workspace          # builds protos via protox — no system protoc needed

cd ../sdk
uv sync --dev && uv run pytest  # the Python frontend: build + test
```

The Python package ships as two distributions from this one source tree —
`pip install waddle-sdk` carries the control-plane transport, and
`pip install 'waddle-sdk[teleop]'` adds the LiveKit media plane on top. See
[`sdk/README.md`](sdk/README.md).

For a running program rather than a snippet,
[`sdk/examples/toy_robot.py`](sdk/examples/) is a whole robot integration in
one file — a simulated 6-dof arm with a camera, the rollout loop, and
`waddle.agent()`. It needs no hardware and no plane:

```bash
cd sdk && uv run python examples/toy_robot.py
```

## If the SDK already knows your robot

That whole integration is a factory call. `waddle.robots.<vendor>` carries a
machine's model facts, its driver and the owner's per-command envelope; the
first vendor module is the I2RT YAM, and two of them supervised is five
lines:

```python
import waddle
from waddle.robots import yam

rig = yam.bimanual(workspace=WORKSPACE_M, gripper_limits=(0.1, 1.7), sim=True)
with rig.session("towels", transport=waddle.Grpc(url, token)) as session:
    result = waddle.agent("stack the cups")
```

`WORKSPACE_M` and the gripper's `[closed, open]` motor radians are SITE facts
and have no defaults — measure them at your own bench. What a YAM *is* (joint
limits, the chain, the tool frame) is a model fact that ships in the module,
gated against the vendor's own model. Driving metal — `sim=False`, plus each
arm's CAN interface — also needs I2RT's own package, which is deliberately not
a dependency of this SDK and cannot be an extra of it (it is not published on
PyPI), so it is a documented command, pinned to the commit every fact in the
module is stated against:

```bash
pip install "i2rt @ git+https://github.com/i2rt-robotics/i2rt@570ef66681ff12bd8298aba34084307cfecc9f05"
```

The runnable program is [`sdk/examples/yam_bimanual.py`](sdk/examples/); the
layering, the postures, the envelope-ownership doctrine and the template for
writing your own vendor module are in
[`sdk/README.md`](sdk/README.md#robot-modules-waddlerobots).

Contributors and agents: read [`CLAUDE.md`](CLAUDE.md) first.

## License

Apache-2.0. See [LICENSE](LICENSE).
