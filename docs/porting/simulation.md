# Simulation backends and portable scenes

The SDK has two separate simulation extension points:

- A **portable scene compiler** turns a reviewed URDF-based `waddle.scene/v1`
  document into backend-native assets plus an ordinary `site.yaml`.
- A **shared-world backend** owns one simulator clock, scene, renderer, and every
  attached robot and camera while presenting the usual SDK runtime to applications.

The runtime interface exposes no simulator identity or simulation-specific behavior. A generated
MuJoCo site, a ROS-connected Isaac Sim stage, and physical hardware all cross the same
`SdkRuntimePort`, support matrix, owner envelope, observation, action, and RGB-D sample
contracts.

The base wheel currently discovers these built-ins without opening either runtime:

| Short name | Runtime backend | Portable scene compiler |
|---|---:|---:|
| `mujoco` | Yes, with `[mujoco]` | Yes, with `[mujoco]` |
| `ros2` | Yes, from a sourced ROS 2 installation | No |

Gazebo and Isaac Sim use the `ros2` runtime after their world, robot, cameras, lights,
and ROS graph are configured in the simulator. A one-command portable-scene compiler
for either simulator is not bundled; install one through the compiler entry-point group
when it exists. `waddle-sdk sim backends` reports the runtime/compiler combination that
is actually installed.

## Start with any portable URDF bundle

Install the MuJoCo reference implementation and initialize an editable scene:

```bash
python -m pip install 'waddle-sdk[mujoco]'
waddle-sdk sim backends
waddle-sdk sim init path/to/arm.urdf --tool-link tool0 --output my-scene
waddle-sdk sim validate my-scene/scene.yaml
waddle-sdk sim compile my-scene/scene.yaml \
  --backend mujoco --output my-scene/build
MUJOCO_GL=egl waddle-sdk sim run my-scene/scene.yaml \
  --backend mujoco --output my-scene/smoke
```

`sim init` copies the URDF and every referenced relative asset into a new portable
directory; it never overwrites a target. Resolve `package://arm_description/...`
references explicitly with `--package arm_description=/path/to/package`. Absolute and
escaping asset paths are refused. The initializer supplies a scene camera, RGB-D,
lighting, a ground plane, and editable appearance defaults. It deliberately does not
guess a tool offset, workspace bounds, keep-outs, or conservative body spheres.

`sim compile` is also non-overwriting and atomic. A successful MuJoCo bundle contains:

- `site.yaml`, which is the complete ordinary SDK manifest;
- `world.xml`, the compiled shared MuJoCo scene;
- `calib/`, deterministic simulator-ground-truth scene and wrist transforms;
- `sources/` and `assets/`, the normalized source model and copied inputs; and
- `resolved-scene.json`, the exact random seed, every sampled value, and a size/SHA-256
  inventory of the generated files.

The checked-in `sdk/examples/portable-simulation/` scene exercises one URDF arm,
reviewed controls, scene and wrist RGB-D cameras, lights, material variation, a
visual-only coating, conservative body spheres, and environment geometry.

Portable builds also seed `calib/` with deterministic scene-camera and wrist-mount
artifacts derived from the resolved simulator frame graph. These records carry
`simulation_ground_truth` provenance, zero measured pairs, and the resolved scene
seed. Applications may read these explicit simulation records from `calib/`; they
are evidence from the compiler, not measured physical calibration. A physical site
is never auto-calibrated. A wrist camera must be rigidly mounted on the declared tool
link; compilation refuses any geometry for which a constant camera-to-TCP transform
cannot be proven.

## Portable `waddle.scene/v1`

The strict schema accepts these backend-neutral sections:

```yaml
api_version: waddle.scene/v1
kind: SimulationScene
metadata: {id: customer-cell-sim}
physics:
  timestep_s: 0.002
  gravity_m_s2: [0.0, 0.0, -9.81]
robots:
  arm:
    urdf: robot.urdf
    pose:
      position_m: [0.0, 0.0, 0.0]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
    base_frame: world
    joints:
      shoulder: {lower: -2.0, upper: 2.0, home: 0.0,
                 max_velocity: 1.0, effort: 8.0, kp: 80.0}
    tool: {link: tool0}
    appearance:
      default: {rgba: [0.9, 0.45, 0.08, 1.0], roughness: 0.5}
    collision_spheres:
      - {name: upper, link: upper_link, radius_m: 0.12,
         center_m: [0.0, 0.0, 0.15]}
cameras:
  scene:
    mount: {kind: scene}
    pose:
      position_m: [0.0, -2.0, 1.0]
      quaternion_wxyz: [0.70710678, -0.70710678, 0.0, 0.0]
    stream: {width: 1280, height: 720, fps: 30}
    vertical_fov_deg: 58
    depth: true
    depth_scale_mm: 1.0
lights:
  key:
    type: directional
    position_m: [0.0, -1.0, 3.0]
    direction: [0.2, 0.3, -1.0]
    color: [1.0, 0.95, 0.9]
    intensity: {uniform: [0.8, 1.2]}
    cast_shadows: true
geometry:
  floor:
    geometry: {kind: plane, size_m: [4.0, 4.0]}
    pose: {position_m: [0.0, 0.0, 0.0]}
    material: {rgba: [0.3, 0.3, 0.3, 1.0], roughness: 0.8}
    collision: {friction: [0.8, 0.01, 0.001]}
randomization: {seed: 42}
```

The compiler imports every scalar revolute, continuous, and prismatic URDF joint.
Joint overrides may tighten source limits but cannot widen position, velocity, or
effort. Continuous joints need explicit finite bounds. Planar, floating, and mimic
joints fail with a targeted refusal until a backend-neutral coupled-joint contract is
declared. The compiler generates position actuators, internal simulator names, a tool
site, cameras, lights, material assignments, and the SDK manifest mapping. The MuJoCo
compiler currently requires `base_frame: world`: the robot pose is already the URDF
base-to-world transform, and claiming another reporting frame without transforming FK
would be false.

Numeric camera positions, light properties, material components, and geometry values
may use `{uniform: [min, max]}` or `{choice: [...]}`. `--seed` selects a replayable
build; omitted seeds use `randomization.seed`. Sampled values never apply to the owner
joint envelope.

### Appearance and coatings

Robot appearance may set one default material and per-link overrides. `coatings` add
explicit primitive or mesh geometry on a named link. A coating without `collision` is
visual-only: it cannot alter mass, inertia, friction, contact, or the SDK owner
envelope. A physical coating must declare both its collision properties and a
conservative link-local `safety_sphere`; compilation folds that reviewed sphere into
the ordinary SDK body-geometry facet. This keeps renderer convenience from silently
changing motion safety.

### Cameras and transforms

Portable camera poses are optical-frame transforms: +X points image-right, +Y points
down, and +Z points forward. `mount: {kind: scene}` attaches to the world;
`mount: {kind: wrist, part: arm, link: tool0}` attaches to that exact URDF link. The
compiler writes the same reviewed optical transform into the simulator and generated
SDK `frames` declaration. RGB is contiguous `uint8`; depth is pixel-aligned metric
Z16 `uint16`, with its millimetres-per-unit scale declared alongside intrinsics.

A camera that moves independently during an episode still requires a timestamp-matched
dynamic transform. Otherwise base-frame localization degrades while RGB-D capture
remains available.

## Generated shared-world site

The compiler emits the same form a customer can author directly:

```yaml
worlds:
  cell:
    driver: mujoco
    connection: {model: world.xml}
    options: {evidence: resolved-scene.json}
parts:
  arm:
    world: cell
    posture: supervised
    base_frame: world
    connection: {}
    joint_limits: {shoulder: [-2.0, 2.0]}
    options:
      model_joint_names: [arm/shoulder]
      actuator_names: [arm/shoulder_position]
      home: [0.0]
      rate_hz: 100
      max_joint_speed_per_s: 1.0
cameras:
  scene:
    world: cell
    connection: {camera: scene}
    stream: {width: 1280, height: 720, fps: 30}
    frame_id: scene_optical
    mount: {kind: scene}
    options: {depth: true, depth_scale_mm: 1.0}
```

A part or camera declares exactly one of `driver` and `world`. A world-backed wrist
camera must share the owning part's world.

## Public backend and compiler contracts

A backend factory accepts `config=WorldConfig`; calling it and its `part()` facet must
remain non-opening. The returned backend provides:

- `open()` to load or connect to the world after connector authorization;
- `step(dt)`, called once per composite SDK tick regardless of part count;
- `reset() -> bool`, called once before ordinary arm homing; and
- idempotent `close()`, after cameras and arms close.

Optional facets are `part(config=PartConfig) -> Rig` and
`camera(config=CameraConfig) -> CameraDriver`. Shared physics/rendering state must be
protected against concurrent command, step, capture, reset, and shutdown.

An installed distribution can register short names instead of requiring module paths:

```toml
[project.entry-points."waddle_sdk.simulation_backends"]
acme_sim = "acme_waddle.backend:backend"

[project.entry-points."waddle_sdk.simulation_compilers"]
acme_sim = "acme_waddle.compiler:compile_scene"
```

The compiler callable accepts `config=SceneConfig, output_dir=Path` and returns
`SceneArtifacts` containing portable relative paths. `site.yaml` can then use
`driver: acme_sim`, while the CLI uses `--backend acme_sim`. Full module targets remain
valid. These public contracts live in `waddle_sdk.simulation` and `waddle_sdk.scene`.

## MuJoCo

The built-in `mujoco` backend and compiler require MuJoCo 3.5 or newer and live behind
`[mujoco]`. The compiler uses
MuJoCo's own URDF parser and model-editing/attachment API, so multiple robots can share
one generated MJCF without a hand-written URDF translator. The runtime shares one
`MjModel` and `MjData`, renders RGB and metric depth from named cameras, derives
intrinsics from vertical field of view, and uses a separate scratch state for FK and
link-local conservative body spheres. OpenGL/EGL renderer creation and destruction
remain on the SDK camera-pump thread that owns the context.

## ROS 2, Gazebo, and Isaac Sim

`driver: ros2` is a shipped shared-world backend. It creates a dedicated ROS context,
node, and executor only from `open()`. A site selects ordinary ROS graph endpoints:

```yaml
worlds:
  cell:
    driver: ros2
    connection:
      node_name: sdk_scene
      namespace: /cell
      domain_id: 7
      reset_service: /reset_simulation   # optional std_srvs/Empty
    options: {reset_timeout_s: 5.0}
parts:
  arm:
    world: cell
    posture: supervised
    base_frame: world
    connection:
      joint_state_topic: /joint_states
      command_topic: /position_controller/commands
    joint_limits: {shoulder: [-2.0, 2.0], elbow: [-1.5, 1.5]}
    options:
      command_message: float64_multi_array
      state_timeout_s: 5.0
      home: [0.0, 0.0]
cameras:
  scene:
    world: cell
    connection:
      rgb_topic: /scene/rgb
      depth_topic: /scene/depth
      camera_info_topic: /scene/camera_info
    stream: {width: 1280, height: 720, fps: 30}
    frame_id: scene_optical
    mount: {kind: scene}
    options: {depth: true, depth_scale_mm: 1.0, sync_tolerance_ms: 5.0}
```

The default part command is ros2_control's `std_msgs/Float64MultiArray` position input.
Set `command_message: joint_state` for a `sensor_msgs/JointState` subscriber such as
Isaac Sim's ROS2 Subscribe Joint State node. The backend orders names from the reviewed
SDK joint declaration, waits for a complete initial state, converts RGB/BGR/RGBA image
encodings, converts 32FC1 metres or preserves declared-scale 16UC1 depth, pairs RGB and
depth by acquisition timestamp, and reads active intrinsics from `CameraInfo`.
When site intrinsics are omitted, opening waits up to
`options.camera_info_timeout_s` (five seconds by default) for that active calibration.

Gazebo can expose its ros2_control joint state/controller topics and camera sensor
topics directly. For Isaac Sim 6.0, configure its ROS 2 graph with Read/Publish Joint
State, Subscribe Joint State plus Articulation Controller, and RGB,
DistanceToImagePlane, and CameraInfo publishers. Use `command_message: joint_state`.
NVIDIA's current graph and sensor APIs are documented in the
[joint-control tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_manipulation.html),
[camera publishing tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_camera_publishing.html),
and [URDF importer API](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/importer_exporter/import_urdf.html).
Isaac stays in its own process and GPU/Python environment; only bounded ROS messages
cross into the SDK process.

ROS topics do not replace SDK authority, timestamps, or the owner envelope. Ensure the
ROS graph has no second command publisher, and do not expose an independent control
path around the SDK.

## Lifecycle and conformance

The Site lifecycle validates topology without importing a backend, completes connector
authorization, opens worlds in manifest order, then arms and cameras. It advances each
world once per pump tick, resets each once per episode, stops pumps, closes devices,
and closes worlds in reverse order. An independently clocked ROS service validates
`step(dt)` but owns its own clock.

Every backend package should test:

- non-opening import, factory, and `part()` construction;
- world → arms → cameras open order and the reverse close order;
- exactly one world step and reset for multipart sites;
- paired RGB-D shape, dtype, scale, intrinsics, and optical frame;
- partial-open cleanup, blocked capture shutdown, and process loss; and
- the complete opened site satisfying `SdkRuntimePort`, not only backend classes.

Continue with the [camera contract](camera.md) and
[validation checklist](testing.md).
