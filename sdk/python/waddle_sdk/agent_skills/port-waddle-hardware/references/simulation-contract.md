# Portable scenes and shared simulation backends

## Prefer the shipped scene workflow

For a URDF arm plus cameras, lights, appearance, coatings, and environment geometry,
start with the strict `waddle.scene/v1` build path:

```bash
waddle-sdk sim backends
waddle-sdk sim init arm.urdf --tool-link tool0 --output portable-scene
waddle-sdk sim validate portable-scene/scene.yaml
waddle-sdk sim compile portable-scene/scene.yaml \
  --backend mujoco --output portable-scene/build
```

The initializer copies referenced assets and refuses absolute, escaping, unresolved
`package://`, or existing output paths. It does not guess workspace bounds, keep-outs,
tool offsets, or conservative body geometry. Review those facts before supervised
motion.

The compiler emits ordinary `site.yaml`, a backend-native world, normalized sources,
and `resolved-scene.json` with the seed, sampled values, and file hashes. Position,
velocity, and effort overrides may tighten URDF limits but never widen them. Continuous
joints need finite reviewed overrides; unsupported coupled joints fail explicitly.
The built-in MuJoCo compiler/runtime requires MuJoCo 3.5 or newer.

Portable camera poses use optical axes (+X right, +Y down, +Z forward). Scene and wrist
cameras compile into the simulator and the same SDK frame declaration. Materials and
visual-only coatings cannot affect contact or owner safety. A physical coating requires
explicit collision properties and a conservative link-local safety sphere.

The packaged YAM/xArm7 reference worlds preserve site-selected robot-part,
base-frame, and camera names. Change wrist-camera owner names in both the site
and its simulation profile when renaming a part. One reference world contains
one robot; two part names cannot alias that robot. Camera placements, calibration,
and object layouts may differ between sites, while reported intrinsics and depth
units must describe the rendered pixels.
Reference YAM worlds start with TCP near `(0.36, 0, 0.14)` m and the open hand
pitched 45 degrees down, in the overlap of forward and downward reach.
Opening/reset establishes that start; a new run preserves the current scene.
`make_site(..., render_quality="fast" | "standard" | "high")` selects native
renderer detail independently of camera resolution and physics. Missing quality
in older reference scenes means `standard`. Higher quality can reduce throughput;
MuJoCo/SAPIEN remain raster renderers, while Isaac high uses path tracing.
Reference native dynamics include a passive damped drawer, a single-motor
MuJoCo hand tendon, and the engine's recommended elliptic friction cone to reduce
grasp creep. These use ordinary physics constraints; manipulation tests must
observe contact, prop motion, and retention separately from commanded robot pose.
For threaded props, test release away from joint limits and confirm that applied
torque can still turn them in both directions. End-stop tests can hide missing
passive resistance; a position servo that holds progress is not thread friction.
Check imported native mass and inertia together. Changing a body's mass after
automatic inertia calculation can silently leave the two inconsistent; the
reference cubes now use explicit shared URDF mass properties in every backend.
Likewise, equal opposing-jaw targets do not replace mechanical coupling under
asymmetric contact. Reference SAPIEN hands retain the native URDF relation.
The normative model settings and limitations are in `docs/python/simulation.md`
and the packaged `simulators/data/README.md`.
Reference model builds decompose every concave collision mesh, including arm
links, before native convex import. A single convex hull can otherwise fill
physical recesses and cause false self-collisions in reachable wrist poses.
Public planning bounds cover each physical link independently of that convex
partition; native collision detail must not multiply redundant planning bounds.

Reference workers advance elapsed real time in fixed substeps before each request,
using the previous target. Slow capture/IPC therefore cannot drop physics time or
apply a new command retroactively. Set `worlds.cell.options.real_time: false` for
explicit SDK stepping. Camera pumps skip missed frame slots; available compute
still bounds delivered frame rate and simulation throughput.
Queued state/control requests take priority over queued captures on the shared
native connection. An in-progress capture still finishes before the next request.

## When to implement another world

Use a world when parts and cameras share physics, render state, scene objects, or one
simulator process. Consumers must not know the choice: the opened site exposes only the
ordinary `SdkRuntimePort`, support rows, observations, actions, and camera samples.

The contracts live in `waddle_sdk.simulation`:

```python
def backend(*, config: WorldConfig) -> SimulationBackend: ...
```

Calling the factory must not import a heavyweight runtime, load a scene, start a
process/thread, connect a socket, or allocate a renderer. The backend provides
`open()`, exactly-once `step(dt)`, `reset() -> bool`, and idempotent `close()`. Optional
`part(config=PartConfig) -> Rig` is declaration-only and runs before `open()`;
`camera(config=CameraConfig) -> CameraDriver` runs after it. Protect shared state from
concurrent command, step, capture, reset, and shutdown.

Register installable short names rather than editing the SDK:

```toml
[project.entry-points."waddle_sdk.simulation_backends"]
acme = "acme_waddle.backend:backend"

[project.entry-points."waddle_sdk.simulation_compilers"]
acme = "acme_waddle.compiler:compile_scene"
```

A compiler accepts `config=SceneConfig, output_dir=Path` and returns `SceneArtifacts`.
It must emit a complete ordinary site manifest and backend world plus deterministic
evidence without overwriting its output.

## ROS 2 and Isaac Sim

The shipped `driver: ros2` backend owns a dedicated ROS context/node/executor. Parts
consume `sensor_msgs/JointState` and publish either ros2_control
`std_msgs/Float64MultiArray` commands or, with `command_message: joint_state`, Isaac
Sim-compatible JointState commands. Cameras pair `sensor_msgs/Image` RGB/depth by
acquisition time and consume `CameraInfo` intrinsics. Supported depth is 16UC1 with a
declared scale or 32FC1 metres.

ROS/Isaac/Gazebo still need one command publisher below the SDK envelope, deterministic
reset/shutdown, matching optical frames, and paired RGB-D. ROS clocks and topics do not
replace SDK authority or paired SDK timestamps.

## Required tests

- Prove import, factory construction, and `part()` are non-opening.
- Prove world → arms → cameras open order and reverse close order.
- Prove one composite tick/reset reaches each shared world once.
- Prove RGB/depth pairing, shape, dtype, units, intrinsics, and optical frame.
- Prove renderer thread affinity, process/topic loss, and partial-open cleanup.
- Open the complete site and assert `SdkRuntimePort`; do not stop at unit-testing the
  backend object.

Reference engines use 2 ms native substeps; SAPIEN robot shapes use 2 mm
contact margins while retaining native rest offsets and material friction. Keep integration accuracy separate
from SDK control and sensor rates; a stable-looking pose alone does not prove
that native joint velocities have converged under contact. Consumer completion
thresholds must not mask a native solver defect.
