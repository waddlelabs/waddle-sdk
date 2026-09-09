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

Isaac's loaded `urdf-usd-converter` also needs the principal-axis fixes verified
in upstream release 0.3.3. The older 0.1.3 converter bundled with Isaac 6.0.1
misorients non-diagonal link inertias. Check the converter actually selected by
the activated importer extension; a separately installed version alone does not
prove which bundled module Kit loads. These are upstream dependency fixes,
documented in the [converter changelog](https://github.com/newton-physics/urdf-usd-converter/blob/v0.3.3/CHANGELOG.md).

Each engine accepts `yam` or `xarm7` and these environments:

- `two_cubes`: two free 60 g, 50 mm rigid cubes on a table, with frictional
  grasp contacts and the inertia of a uniform solid cube.
- `bottle_cap`: a fixed bottle fixture and a passive cap. MuJoCo uses native
  thread contact with a 25 g free cap and 4.166667 mm/revolution pitch; the cap
  can leave the thread. SAPIEN/Isaac currently use a finite rotation/axial guide
  with 5 mm/revolution pitch and three turns of travel; free removal on those
  backends is not yet implemented.
- `drawer`: a fixed cabinet and a physical drawer with 220 mm of passive travel
  and 5 N·s/m native joint damping. The damping dissipates a pull after release;
  there is no spring returning the drawer to its starting position.

MuJoCo's cap reuses its installed first-party nut/bolt SDFs through a small
dimensional wrapper. Native contact and friction retain the cap under axial
load; no guide or attachment is released by task logic. The roof and packaged
convex surfaces let the same robot geometry grasp the cap and interact with
scenery. These are illustrative prop settings, not a measured commercial seal.
A C++17 compiler (`c++`, or one selected with `CXX`) is required for this scene.
Each bottle-cap worker compiles the wrapper against its own installed MuJoCo
headers/library in a private temporary directory, then loads it through the
native plugin API. This adds compilation to worker startup; it does not download
assets or overwrite an installation. Cube and drawer workers do not compile it.

The reference robot models preserve the live adapters' joint names, order, limits,
radian units, FK, and normalized hand action (0 closed, 1 open). YAM uses the SDK's
pinned I2RT URDF and tool convention, assembled with the live adapter's 95 mm
LINEAR_4310 hand. xArm7 uses UFACTORY's expanded URDF with the G2 gripper and its
standard TCP. The original visual meshes, link masses, centers of mass and inertia
tensors are retained. Concave collision meshes are decomposed offline into
convex pieces so each engine preserves arm recesses, finger geometry and housing
clearance. Convex meshes remain unchanged. Public planning bounds conservatively
cover complete collision triangles per physical link, so their number does not
scale with the importer's convex partition. Native constraints
close the xArm linkage; its revolute hand is mapped nonlinearly to jaw travel.
Isaac marks both loop-closing spherical joints as excluded from the articulation
tree. They remain enabled physical constraints; all imported manufacturer joints
stay in the tree. This follows [PhysX's closed-loop articulation setup](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/articulations.html#closed-loops).
Its native motor limit is 4.2039 N m, derived from the G2's published maximum
50 N gripping force and the minimum jaw transmission over its stroke. This
bounds the ideal quasi-static jaw force; it does not calibrate impact forces or
emulate the hardware's force-control electronics. The former 50 N m motor limit
could overload the simulated linkage. See [UFACTORY's G2 specifications](https://docs.accessories.ufactory.cc/xArm_Gripper_G2/6.Technical_Specifications.html).
SAPIEN retains the URDF's native jaw coupling and shares the actuator's gains,
force limit, and reflected inertia across the two native drives. Equal position
targets alone cannot keep the jaws coupled under asymmetric contact. Splitting
the drive load also avoids making the coupling transfer all of one jaw's force.
No per-step contact forces or object attachments implement grasping.

MuJoCo uses Menagerie's single-motor fixed tendon to distribute force between
opposing jaws, preserving the combined gain, force limit, and reflected inertia.
The native hand equality uses Menagerie's 5 ms time constant. MuJoCo also
retains the xArm passive-hand joints' reference armature of 0.1 kg m² and the
driver/follower limits' 5 ms response. These native numerical settings support
the four-bar closures under contact; the manufacturer link inertias remain
unchanged. They are simulator settings, not measured motor inertias.
All reference scenes use MuJoCo's recommended elliptic friction cone with impedance ratio 10 and Newton
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
New drawer sites place the scene camera in front of the cabinet, at
`(-0.45, -0.55, 0.55)` m looking toward `(0.4, 0, 0.2)` m, so its handle face is
visible at the reference home. The other scenes retain their overhead oblique
view. Existing scene files retain their explicit camera transforms; updating a
transform also requires regenerating calibration artifacts bound to that scene.
SAPIEN's guided cap uses a force-limited velocity damper with a 0.001 N m
budget and 1 N m s/rad damping, giving a viscous transition below 0.001 rad/s.
No position servo holds that cap after release. Its native tests check retention
away from an end stop and continued turning in either direction. MuJoCo's
separate free-cap tests check axial-load retention, thread pitch, natural exit,
and subsequent free-body motion.
The SDK's distinct high bimanual test poses are not used as reference-scene homes.
Opening or explicitly resetting a world establishes
this pose. Ordinary run boundaries preserve the current pose. Camera calibration
and robot kinematics are independent of this initial joint configuration.

Sources, licenses, conversion steps and SHA-256 hashes ship in
`waddle_sdk/simulators/data/`. `tools/vendor_simulation_models.py` rebuilds those
assets from pinned public manufacturer revisions. No model is downloaded at site
startup. `tools/vendor_thread_model.py` rebuilds the native thread collision
assets from the installed first-party SDFs; its data README records versions and
reproduction. The reference collision spheres are conservative covers derived from
these same meshes and link transforms.

Reference worlds use real-time physics by default with 500 Hz native stepping
(2 ms substeps) in all three engines. SAPIEN's coupled hand contacts require finer
integration than the generic 100 Hz manipulation default. This increases native
physics work; camera and SDK control rates remain independent. SAPIEN uses 2 mm
per-shape robot contact margins so the closed hand's convex pieces do not generate
thousands of distant candidate contacts. This changes contact generation distance,
not the meshes, resting separation, exclusions or material friction. The worker
advances fixed
native substeps up to each request's monotonic arrival time **before** reading
state or changing a target. Rendering and IPC delays therefore preserve elapsed
physics under the previous target; a new command is never applied retroactively.
Fractional substeps carry between requests. State reporting uses the ordinary SDK
pump at twice the declared control rate, with a 100 Hz floor. New scenes use the physical SDK control defaults: YAM 10 Hz, xArm7 50 Hz,
and 1 rad/s joint speed for both. Explicit site settings remain authoritative. Available compute still bounds achievable throughput.
Queued state/control requests take priority over queued camera captures, retaining
FIFO order within each group and one native transaction at a time. An in-progress
capture cannot be interrupted. Real-time pump ticks need no extra clock request:
reads, writes and captures already advance elapsed time. Explicit-step rollouts
still dispatch their requested durations.

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

Isaac's USD camera authoring converts the principal point into aperture offsets
with OpenUSD's frustum convention. The projection/deprojection regression runs
without Isaac: install the optional `usd-core` package in a development test
environment, then run `python -m pytest tests/test_simulation.py -k isaac_usd_camera`
from `sdk/`. It checks off-center and unequal-focal-length calibration using
OpenUSD's actual projection matrix. It does not validate RTX raster sampling,
annotator alignment or frame delivery; those remain native acceptance checks.
A separate test environment with `urdf-usd-converter==0.3.3` supplies OpenUSD
through its dependencies. Allow prerelease dependency wheels when installing
that converter, then use `-k isaac_usd` to also check the complete imported robot
mass properties, joint trees and loop-anchor alignment. Converter tests run in subprocesses so
their USD schema registration is independent of earlier camera tests. They do
not establish loaded-grasp or other native Isaac physics acceptance.

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

The worker advances physics, serializes sensor/control requests, and
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
