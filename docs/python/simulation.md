# Physics simulation sites

Physics uses the same `Site`, `SdkRuntimePort`, joint-position, gripper, RGB-D,
kinematics, body-geometry, recording, hold, and e-stop contracts as physical devices.
The existing optional `PositionVelocityDriver` port accepts known trajectory
velocities, bounded by the site's declared arm speed; the hand remains a position
latch. Ordinary position commands and holds clear velocity feedforward.
Only the site adapters and their configuration select an engine. Programs must use
capability facts; importing a simulator does not establish sensor or motion support.

## Reference engines and scenes

| Selector | Role | Installation |
| --- | --- | --- |
| `mujoco` | Native URDF import, MuJoCo position drives and RGB-D | `pip install 'waddle-sdk[mujoco]'` |
| `isaac` | NVIDIA USD/PhysX and RTX rendering | Separate Isaac Sim 6.0.1 Python 3.12 installation; pass its interpreter as `worker_python` |
| `sapien` | PhysX manipulation and Vulkan RGB-D; a foundation for ManiSkill assets | `pip install 'waddle-sdk[sapien]'` on a supported platform |

MuJoCo and SAPIEN can run headlessly with working EGL/Vulkan drivers. The worker
currently requires POSIX. Isaac has its own GPU, driver, Python, and license
requirements. Installation and license acceptance are operator actions; opening a
site never downloads packages, accepts a license, or substitutes a mock engine.

Each engine accepts `yam` or `xarm7` and these environments:

- `two_cubes`: two free rigid cubes on a table, with frictional grasp contacts.
- `bottle_cap`: a fixed bottle fixture and a cap with coupled passive rotation and
  axial travel (5 mm/revolution). This reference thread model has three turns of
  travel; the cap remains on its axial guide, rather than becoming a free body.
- `drawer`: a fixed cabinet and a physical drawer with 220 mm of passive travel
  and 5 N·s/m native joint damping. The damping dissipates a pull after release;
  there is no spring returning the drawer to its starting position.

MuJoCo's reference cap uses `0.001 N·m` of native dry thread resistance to retain
progress after release. This is an illustrative prop setting, not a measured
bottle seal. MuJoCo's soft constraints permit small residual drift; native
acceptance bounds axial drift to 0.1 mm over a five-second released interval.
PhysX joint-friction parameters have different semantics and are not assigned
the same numeric torque as a coefficient.

The reference robot models preserve the live adapters' joint names, order, limits,
radian units, FK, and normalized hand action (0 closed, 1 open). YAM uses the SDK's
pinned I2RT URDF and tool convention, assembled with the live adapter's 95 mm
LINEAR_4310 hand. xArm7 uses UFACTORY's expanded URDF with the G2 gripper and its
standard TCP. The original visual meshes, link masses, centers of mass and inertia
tensors are retained. Concave hand collision meshes are decomposed offline into
convex pieces so each engine preserves recesses and cable clearance. Native constraints
close the xArm linkage; its revolute hand is mapped nonlinearly to jaw travel.
SAPIEN follows ManiSkill's PD mimic-controller pattern for the coupled jaws, sharing
the actuator's gains, force limit, and reflected inertia across the two native
drives. This avoids the imported URDF tendon's contact oscillation, but approximates
the physical transmission under asymmetric contact. The other engines retain native
jaw coupling. No per-step contact forces or object attachments implement grasping.

MuJoCo uses Menagerie's single-motor fixed tendon to distribute force between
opposing jaws, preserving the combined gain, force limit, and reflected inertia.
The native hand equality uses Menagerie's 5 ms time constant. All reference scenes
use MuJoCo's recommended elliptic friction cone with impedance ratio 10 and Newton
tolerance `1e-10` to reduce gradual grasp slip. This costs more solver work than
the default pyramidal cone; it does not increase material friction or eliminate
all compliance. See [MuJoCo's slip guidance](https://mujoco.readthedocs.io/en/stable/modeling.html#preventing-slip).
Finger collisions also use Menagerie's pad contact response (`solref="0.004 1"`,
`solimp="0.95 0.99 0.001"`, priority 1) on the original decomposed finger meshes.
This prevents the default soft contacts from producing excessive jaw penetration
and oscillation under a grasp. Material friction and the 2 ms timestep are unchanged.

The YAM reference scene starts with its TCP near `(0.36, 0, 0.14)` m and the open
hand pitched 45 degrees down. This lies in the overlap of the model's forward and
downward reach, with room for nearby tabletop motion. The previous near-neutral
pose sat at the inner boundary for forward approaches. This is a reference
working pose; the manufacturer's model and joint limits are unchanged, and it
does not specify a physical arm's resting configuration.
The bottle sits to the side of this central region, and the cabinet's closed
front sits beyond it, so neither prop intersects the starting hand. The drawer
handle travels from x=0.503 m to x=0.283 m as it opens.
The SDK's distinct high bimanual test poses are not used as reference-scene homes.
Opening or explicitly resetting a world establishes
this pose. Ordinary run boundaries preserve the current pose. Camera calibration
and robot kinematics are independent of this initial joint configuration.

Sources, licenses, conversion steps and SHA-256 hashes ship in
`waddle_sdk/simulators/data/`. `tools/vendor_simulation_models.py` rebuilds those
assets from pinned public manufacturer revisions. No model is downloaded at site
startup. The reference collision spheres are conservative covers derived from
these same meshes and link transforms.

Reference worlds use real-time physics by default. SAPIEN uses ManiSkill's
100 Hz physics default; MuJoCo and Isaac retain 500 Hz native stepping. The worker advances fixed
native substeps up to each request's monotonic arrival time **before** reading
state or changing a target. Rendering and IPC delays therefore preserve elapsed
physics under the previous target; a new command is never applied retroactively.
Fractional substeps carry between requests. State reporting uses the ordinary SDK
pump at twice the declared control rate, with a 100 Hz floor. New scenes use the physical SDK control defaults: YAM 10 Hz, xArm7 50 Hz,
and 1 rad/s joint speed for both. Explicit site settings remain authoritative. Available compute still bounds achievable throughput.

For explicitly stepped rollouts, set `worlds.cell.options.real_time: false`; only
SDK world-step durations then advance physics. This is independent of scene reset:
runs preserve the scene by default, while `worlds.cell.options.reset_on_episode:
true` restores native snapshots for each rollout. Reopening creates a fresh scene.
Both modes use the same model, renderer, sensors and SDK lifecycle.

Position servos, gravity compensation,
friction and the primitive task props remain
simulation settings. Manufacturer CAD and inertial properties do not establish
calibrated actuator dynamics or a match to every hardware revision. A different
physical hand, TCP offset, link revision or camera mounting needs matching site
configuration. The LINEAR_4310 pinch offset is derived from the current manufacturer
model (approximately 9.95 mm along the declared TCP's +Z). The SDK TCP stays
unchanged, and the older hand's lateral pinch offset does not apply.
Its wrist optical pose comes from I2RT's published LINEAR_4310/D405 bracket;
the reference intrinsics remain explicit site settings.

The reference table uses a bundled 1K CC0 Poly Haven wood texture (about 5.3 MB
installed). MuJoCo and SAPIEN use native material highlights and warm key/cool
fill lighting. These are raster rendering defaults, not calibrated photographic
appearance; RTX rendering remains an Isaac capability requiring separate native
validation. Select `render_quality="fast"`, `"standard"` (default), or `"high"`
in `make_site()`; this becomes `render_quality` in `simulation.json`. Older scene
files without this field retain standard rendering.

| Quality | MuJoCo | SAPIEN | Isaac |
| --- | --- | --- | --- |
| `fast` | No multisampling, 1024 shadow map | No key-light shadows | RTX lighting, no antialiasing |
| `standard` | 4 samples, 4096 shadow map | 2048 shadow map | RTX lighting, DLSS antialiasing |
| `high` | 8 samples, 8192 shadow map | 4096 shadow map | Path tracing, 16 samples/frame, denoising |

These select native renderer settings. MuJoCo and SAPIEN remain raster renderers.
Higher detail consumes more GPU memory and render time and can reduce delivered
camera and control throughput because sensors share a worker with physics. Camera
resolution and requested frame rate remain separate settings. Appearance changes
can affect visual detections and GPU cost. They do
not change contact geometry, camera intrinsics, depth encoding, or owner limits.

## Create and open a site

Use a new directory and write the two returned documents:

```python
import json
from pathlib import Path
import yaml
from waddle_sdk import load_site
from waddle_sdk.simulators import make_site

root = Path("cube-site")
root.mkdir()
site, simulation = make_site(
    "cube-site", backend="mujoco", robot="yam", environment="two_cubes",
    width=640, height=480, render_quality="standard",
)
(root / "site.yaml").write_text(yaml.safe_dump(site, sort_keys=False))
(root / "simulation.json").write_text(json.dumps(simulation, indent=2))
with load_site(root / "site.yaml").open(console=False) as session:
    print(session.observe())
```

`make_site()` is non-opening. Its site has one part, `arm`, and two cameras, `scene`
and `wrist`. Every backend uses those same names and profiles. Names, resolutions,
intrinsics, and optical transforms are explicit configuration, not engine defaults.
Edit camera declarations and their matching simulation profiles together. Robot
part names and base-frame labels also come from the site; when renaming a part,
update its wrist-camera owner in both documents. A base-frame label names the
robot's existing base coordinates and does not transform them. The reference
scene has a single robot; multi-arm/custom arrangements can use ordinary
external SDK adapter packages without an SDK registry change.

## Sensor and frame contract

RGB is RGB8, depth is Z16 axial optical depth, and each pair comes from one frozen
physics state. `depth_scale_mm=1` means millimetres per integer; zero denotes missing
or out-of-range depth. Intrinsics are rectified pinhole intrinsics. The default
resolution is 640×480 at a requested 15 Hz, with explicit focal lengths and principal
point. Actual frame rate depends on rendering performance. If capture or publication
misses a scheduled frame, the camera pump resumes at the next future frame slot.
It never builds a backlog of render requests that can starve shared physics/control.
Every acquired frame still follows the ordinary publication and recording path.

Each site owns its camera placements, calibration, and object layout. These may
differ between physical and simulated environments. To reproduce a particular
sensor, use its active focal lengths, principal point, resolution, and depth scale.
The principal point
is expressed in optical image pixels measured from the top-left corner, including
when it is off-center. These reference cameras require rectified input profiles;
copying a distorted lens's coefficients into a pinhole renderer is not supported.

Optical frames use +X right, +Y down, +Z forward. Robot poses use metres and wxyz
quaternions, in the site's declared base frame (`yam_base` or `xarm_base` by default).
A scene-camera transform is
`base_from_camera`; a wrist transform is `tcp_from_camera` and follows the robot.
Renderer-specific camera axes, depth conventions, joint ordering, and two-finger
coordinates are converted inside the engine adapter. No privileged object pose,
segmentation, teleport, or task-completion operation is exposed through the runtime.

## Ownership and extension

Reference scenes implement the existing [shared-world contract](../porting/simulation.md).
The ordinary `worlds.cell` declaration owns one lazy worker and exposes part/camera
facets. `Site` opens it after authorization, calls its reset hook once per episode,
and closes it after devices. The reference reset hook preserves state unless
`reset_on_episode` is explicitly enabled; ordinary per-arm hooks never home it
independently. Each opening gets independent physics state. Worker
isolation keeps incompatible engine/Python runtimes out of the control process.

These presets add coupled grippers and articulated objects beyond the generic URDF
compiler's supported scene subset. For custom URDF bundles, use the existing portable
scene compiler; for an already configured Isaac stage, the existing ROS 2 backend is
also available. No second SDK lifecycle or agent API is introduced.

The worker continuously advances physics, serializes sensor/control requests, and
fails closed on startup or connection loss. Recovery requires reopening the site;
it never replays a motion after reconnecting. Site camera declarations are compared
to simulation sensor profiles before exposing the camera. Joint owner limits may tighten the
reference limits, but must include the reference home pose.

External simulations can implement the existing [robot](../porting/robot.md) and
[camera](../porting/camera.md) factories. The bundled worker protocol and native
`Engine` classes are private implementation details, not another agent API.

## Research basis

The three engines cover complementary execution needs:
[MuJoCo](https://mujoco.readthedocs.io/en/stable/overview.html) for a compact reference,
[Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html)
for NVIDIA rendering and USD tooling, and
[SAPIEN/ManiSkill](https://maniskill.readthedocs.io/en/latest/) for manipulation assets.
[SimFoundry](https://research.nvidia.com/labs/gear/simfoundry/) is a scene-creation
pipeline to evaluate as an asset source, rather than a fourth interchangeable
physics runtime.

The implementation review uses repositories maintained during the preceding year
(reviewed 2026-09-08): MuJoCo Menagerie (2026-09-04), I2RT 1.3.5 (2026-09-07),
mjlab (2026-08-31), ManiSkill (2026-08-02), and Isaac Lab (2026-09-04).
Exact revisions, source paths, mechanism choices and limitations are recorded in
the packaged `waddle_sdk/simulators/data/README.md`.
All three backends use native URDF import and physics constraints. Engine-specific
geometry conversion, axis handling and inertia conversion stay in the importer.
