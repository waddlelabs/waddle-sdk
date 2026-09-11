# Robot and independent twin adapter contract

## Factory and lifecycle

A manifest part names `module:factory`. Prefer:

```python
def arm(*, config: waddle_sdk.robots.site.PartConfig) -> base.Rig: ...
```

The factory returns a `Rig` containing a `Robot` declaration, `build_arms`, rate, and posture. It must not open a bus, construct a live vendor client, or start a thread. `build_arms()` is the opening boundary. A manifest-loaded factory returns exactly one `Arm` under the bare key `""`; site composition assigns the manifest part name.

Keep the declaration's joint names, order, widths, units, limits, rate, gripper row, and frame facts byte-for-byte consistent with the opened `Arm` and driver. Use SI units: revolute radians, prismatic metres, and per-second rate units.

## Structural driver

Inheritance is optional. A driver provides:

- `kind`: `"sim"` only for a harmless twin; all other values are live.
- `estopped`: the local owner stop-latch state.
- `read()`: position and velocity arrays in declared order and width.
- `write(target)`: accept one already-admitted joint-position target.
- `hold()`: hold the measured current position through a real vendor-safe mechanism.
- `estop()`: set a latch and refuse subsequent writes.
- `re_enable()`: clear the local latch only through the site-operator recovery path.
- `step(dt)`: advance a twin; do nothing for hardware.
- `home(values) -> bool`: twins may reset; live hardware normally refuses unattended homing.
- `close()`: deterministic, idempotent resource release.

Make all methods safe under concurrent SDK pump and dispatch threads. Preserve the e-stop latch across writes and holds. Do not report re-enable success until the vendor operation succeeds.

An optional `write_position_velocity(target, velocity_feedforward_rad_s) -> bool` consumes a producer-known feedforward for the same admitted position target. Return `False` only after deliberately issuing the unchanged position-only target. Never derive feedforward from measurements.

## Owner envelope

Every command crosses `Arm`. Supply exact joint limits and per-command step caps. The default seam checks declared width, finite values, limits, step caps, optional workspace bounds, and configured geometry rules; it rejects whole and never clamps.

Workspace bounds require FK. Static keep-outs and self/cross-part collision require deterministic conservative `CollisionSphere` values and compatible frames. Omit unsupported optional geometry rather than fabricating it. If a configured envelope rule depends on absent geometry, opening must fail closed.

`posture="monitor"` registers observation and the owner stop without a send path. `posture="supervised"` registers send, hold, and e-stop. Posture is not authority.

## Facts and packaging

Ship hardware facts with citations to a pinned manual, model, firmware contract, or vendored machine-readable artifact. Pass that citation through the scaffold's required `--facts-source`; for a simulator, cite the explicit synthetic or test model. The generated `FACTS_SOURCE` records the text verbatim but cannot prove its authority. Directionally test facts where possible: a declared safe limit may be tighter, never wider, than the source. Keep vendor packages optional and lazily imported. Put credentials in named secret references, never source or ordinary manifest values.

Build external adapters as ordinary installable packages. Do not patch an SDK registry: `site.yaml` imports the package factory directly.

When multiple parts or cameras need one shared simulator scene, implement the
[shared simulation backend contract](simulation-contract.md) instead of opening one
world from every part.

## Initializer safety suggestions

Adapters may expose the optional, non-opening `safety_presets(factory=, options=)`
contract from `waddle_sdk.robots`. The YAM reference suggests an arm-base workspace
from `[-0.7, -0.7, 0.0]` to `[0.7, 0.7, 1.0]` metres, without keepouts or a
self-collision configuration. These are site-operator-reviewed starting values,
not vendor joint limits or evidence that a particular mounting is collision-free.
Preserve existing site declarations and validate the selected envelope normally.

Applications can use `waddle_sdk.robots.metadata.part_action_spaces` and `gripper_mapping` without vendor imports or topology assumptions. Supply an explicit physical gripper mapping when supported; reversed action ranges are valid. Every adapter can expose the optional `model_sources(*, factory, part_name, part)` extension returning `robots.models.ModelSources`. YAM implements this same contract. Missing support returns `None`; selected failure raises `ModelSourceError`. Source inspection never opens a driver or selects a planner. See the public source-models guide for asset and binding requirements.
