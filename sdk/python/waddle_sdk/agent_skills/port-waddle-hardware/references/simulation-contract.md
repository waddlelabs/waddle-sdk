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

The SDK pumps resume after slow work without catch-up bursts. Camera capture
skips missed frame slots; physics never applies a newly issued target to ticks
missed during an earlier planning pause. Real-time factor and delivered camera
frame rate still depend on available compute.

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
