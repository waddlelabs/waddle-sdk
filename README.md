# waddle-sdk

The open, hardware-owning layer of Waddle. The SDK loads a strict site
manifest, opens robot and camera drivers, enforces the owner envelope, and
records timestamped raw evidence. Claims, leases, gating, clocks, and recording
semantics live once in the Rust core. The public lifecycle has no lease or handoff
selector: handoff is fixed to hold-first and enforcement placement is derived from
the selected integration.

The dependency direction is deliberately one-way:

```text
closed Waddle -> waddle-metal -> waddle-sdk -> hardware/cameras/simulators
```

The SDK never imports or discovers Metal or closed Waddle. Local callers use
the Python library; remote SDK-only sites use the existing `waddle.v0` control
protocol. `waddle.v0.hosted.runs` lets an authorized host start one ordinary
episode on an idle remote SDK without creating another authority path. The
`waddle-sdk connect` sends `site.metadata.id` with a customer/project API key to
resolve the authoritative hosted binding, then completes a hardware-free transport
registration; arms and cameras open only after the host accepts it.

## Primary Python API

```python
import waddle_sdk

site = waddle_sdk.load_site("site.yaml")
with site.open(transport=waddle_sdk.Grpc(url, token)) as session:
    with session.run(task={"id": "inspect"}, actor={"id": "metal"}) as run:
        observation = run.observe()
        result = run.step(action, observation)
        if not result.dispatched:
            run.hold(result.detail or "command withheld")
    session.estop("emergency")
```

`site.yaml` is `waddle.site/v1`: unknown fields fail, paths are confined to the
manifest directory, and credentials must be named secret references. Hardware
opens only on entry to `site.open()` and every half-open resource is closed on
failure. Static box/sphere keep-outs and named body self/cross-part collision
rules are enforced by the SDK over conservative geometry supplied by each
driver adapter; configured missing or frame-incompatible geometry fails closed.
A connector's exact customer/project/workspace binding and a fresh per-connection
nonce accompany every gRPC method; Register is the barrier before any other message
can flow. A runnable connector emits bounded native heartbeats; credential revocation
closes the transport, requests the core-owned hold verb, and aborts any active
hosted run. Driver-extension APIs live under `waddle_sdk.robots`,
`waddle_sdk.cameras`, and `waddle_sdk.descriptors`; they are not part of the
small root surface.

SDK-only customer sites connect with:

```bash
waddle-sdk connect --site site.yaml
```

The API key comes from `WADDLE_API_KEY` or a secret prompt and carries the
customer/project provenance. SDK-only deployments do not have a separate workspace
argument: `site.metadata.id` is their hosted workspace identity. The default target is
`https://connect.waddlelabs.ai:443` and can be overridden by
`WADDLE_CONNECTOR_TARGET` or `--target`. Once the complete site is open, that same API
key derives one short-lived `https://api.waddlelabs.ai/ui?token=wui_...` invitation,
which the command prints. The long-lived API key never enters the URL or browser.

See [Porting a hardware or simulator backend](docs/hardware-backends.md) for the
minimal external integration surface and [`sdk/README.md`](sdk/README.md) for the
complete Python manifest and driver contracts.

## Repository

| Artifact | Role |
|---|---|
| [`waddle-protocol/`](waddle-protocol/) | Append-only `waddle.v0` schemas, normative FSM/versioning docs, and golden fixtures |
| [`waddle-core/`](waddle-core/) | Reference authority, gate, clock, recording, media, and control-plane implementation |
| [`sdk/`](sdk/) | Python Site/SiteSession/Run facade plus opt-in driver subpackages |

The normative documents are [`GLOSSARY.md`](waddle-protocol/docs/GLOSSARY.md),
[`FSM.md`](waddle-protocol/docs/FSM.md), and
[`VERSIONING.md`](waddle-protocol/docs/VERSIONING.md).

## Build and test

```bash
cd waddle-core
cargo test --workspace

cd ../sdk
uv sync --dev
uv run pytest
```

The default `waddle-sdk` wheel carries gRPC.
`pip install "waddle-sdk[teleop]"` adds the Linux-x86_64 LiveKit companion.
Camera adapters are lazy extras: `[orbbec]`, `[realsense]`, or `[cameras]`.
Physical adapters are lazy too: `[xarm]`, `[alicia]`, `[alicia-d]`, or the
combined `[robots]`; the Synria vendor SDKs currently require Python 3.11+
while the base SDK remains Python 3.10+.
MuJoCo simulation is independently lazy behind `[mujoco]`.

Driving an I2RT YAM also requires the vendor package pinned to the model facts
shipped in this tree:

```bash
pip install "i2rt @ git+https://github.com/i2rt-robotics/i2rt@570ef66681ff12bd8298aba34084307cfecc9f05"
```

Contributors and agents must read [`CLAUDE.md`](CLAUDE.md) before changing the
repository.

## License

Apache-2.0. See [LICENSE](LICENSE).
