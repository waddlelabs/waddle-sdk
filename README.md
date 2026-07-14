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
| Python SDK, `waddle-proxy`, `waddle-cpp`, `waddle_ros` | planned | Hollow frontends over waddle-core, per the design doc's artifact family. |

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
```

Contributors and agents: read [`CLAUDE.md`](CLAUDE.md) first.

## License

Apache-2.0. See [LICENSE](LICENSE).
