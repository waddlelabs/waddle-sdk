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
| `sapien` | PhysX manipulation and Vulkan RGB-D; a foundation for ManiSkill assets | `pip install 'waddle-sdk[sapien]'`; bottle scenes require `waddle-sdk[sapien-gpu]` and CUDA |

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

SAPIEN uses CPU physics for cubes and drawers. Bottle scenes select GPU physics
because PhysX 5.3 requires it for native SDF contact. Install `waddle-sdk[sapien-gpu]`
in that scene's worker interpreter; the extra adds Torch for CUDA state buffers.
Prepare SAPIEN's native GPU library once in that environment with
`python -c 'import sapien; sapien.physx.enable_gpu()'`; that explicit setup command
downloads SAPIEN's matching library if absent. Opening a workspace requires the
library to be present and never performs that download. This has a larger
installation and GPU-memory footprint than the CPU scenes. No extra SDK control
or camera API is needed. For native GPU tests, set `WADDLE_SAPIEN_GPU_TEST_PYTHON`
to that interpreter with pytest installed. An available CUDA device is required;
missing support raises a startup error instead of substituting a guided cap.

Each engine accepts `so101`, `yam`, or `xarm7` and these environments:

- `two_cubes`: two free 60 g, 50 mm rigid cubes on a table, with frictional
  grasp contacts and the inertia of a uniform solid cube.
- `bottle_cap`: a fixed bottle fixture and a 25 g free cap with native thread
  surfaces at 4.166667 mm/revolution pitch. All adapters use a removable assembly;
  MuJoCo and SAPIEN have native axial-retention/free-exit coverage. Isaac's USD
  assembly is checked offline; its runtime contact and removal acceptance remain
  pending licensed native execution.
- `drawer`: a fixed cabinet and a physical drawer with 220 mm of passive travel
  and 5 N·s/m native joint damping. The damping dissipates a pull after release;
  there is no spring returning the drawer to its starting position.

MuJoCo also provides twenty-nine interactive development task environments:

- `touch_target`: one red contact target and two blue distractors.
- `pick_lift`: one free 46 mm cube.
- `place_in_bin`: one free cube and an open five-sided bin.
- `push_to_region`: one free cube and a non-colliding visual goal region.
- `operate_control`: one red and two blue independently sliding push controls.
- `select-distractors`: one requested cube, two differently colored distractor
  cubes, and an open bin.
- `close-drawer`: the physical drawer initialized at 150 mm open so an ordinary
  reset restores the task's starting state.
- `stack-two-cubes`: two independently movable 46 mm cubes.
- `ring-on-peg`: a free compound ring and a fixed vertical peg with physical
  clearance between them.
- `use-hook`: a free compound hook, a movable target, and a marked goal region.
- `open-hinged-door`: a passive lever and sliding bolt on a hinged door. The
  extended bolt physically contacts a fixed strike; holding the lever retracts
  it so the door can open.
- `stack-three-cubes`: three independently movable 46 mm cubes.
- `insert-peg`: a free cylindrical peg and a compound socket whose native rim
  contact admits an aligned peg and supports an offset one.
- `retrieve-from-drawer`: a target object initially concealed inside the physical
  drawer and a marked destination. The higher front camera exposes the object
  after the drawer opens.
- `store-in-drawer`: an initially open physical drawer, a free target object, and
  a named interior frame that moves with the drawer.
- `insert-usb`: a free handled connector with a missing-quadrant tongue and a
  fixed close-clearance port. An internal tab admits the correct roll and blocks
  the reversed roll through ordinary native contact.
- `load-clear-test-tubes`: four translucent free tubes and a blue rack with two
  named rows of physical slot collars. The elevated scene camera exposes the row
  order.
- `candy-bin-transfer`: eighteen same-color candy-sized wrapped objects scattered
  and piled irregularly in a shallow tray beside a blue destination cup. Each
  candy is an independent rigid
  body, so clutter contacts, grasping, lifting, and released placement are native
  physics rather than a task-specific transfer shortcut. Appearance variation
  changes their shared color together rather than creating color distractors.
  Native acceptance requires both reference fingers to contact one candy and
  carry it upward by at least 45 mm for every reference robot family.
- `split_workspace_sorting`: a matched two-arm scene with one cube initially in
  each arm's outer workspace and two open bins near the center. Each cube's
  matching bin is on the opposite side.
- `handover-block`: a matched two-arm scene with a free block at the giver and a
  marked region at the receiver.
- `stabilize-open-drawer`: a matched two-arm scene whose complete cabinet is a
  free body, so opening load moves it unless another arm stabilizes it.
- `hold-container-place`: a matched two-arm scene with a movable open container
  and a free object to place inside.
- `stabilize-remove-lid`: a matched two-arm scene with a movable box, removable
  lid, and lid goal. Bounded spring-loaded pads press on the lid skirt, so native
  friction transfers removal load to the unstabilized box.
- `oriented-tool-handover`: a handled asymmetric tool and a marked final-pose
  silhouette in the receiving arm's workspace.
- `two-arm-peg-insertion`: a free receiving body with a physical socket and a
  separate peg; insertion load can move the receiver unless it is stabilized.
- `joint-lift`: a free tray with two opposite named handles. Balanced handle loads
  lift it level while a single-handle load tilts it.
- `loaded-tray-transport`: a two-handle tray, two free contents, and a marked
  transport goal.
- `uncap-return-test-tube`: a translucent tube in a named rack slot, a separate
  cap, and a cap goal. Preloaded passive pads create finite cap-removal resistance.
- `retrieve-bottle-clutter`: a target and three distractor bottles in a physical
  bin plus a table goal region.

These environments use the ordinary robot, camera, and lifecycle interfaces.
Their scene camera exposes every task-relevant object at the starting pose for
SO-101, YAM, and xArm7. Native tests verify visibility, initial separation from
the robot, free-body mechanics, bin containment, passive-joint initialization,
stacking, ring/peg clearance, tool contact, and independent control travel.
The initial native-contact gate covers all 90 task/family cells and rejects
penetration between a robot body and the tabletop or any task prop. Dual-arm
scenes additionally reject contact between the two robot assemblies.
The door test applies the same opening load before and after lever retraction to
verify that native latch contact, rather than task logic, controls its motion.
The two-arm fixture tests verify that drawer/container loads can move their free
bases and that the box lid's finite friction fit transfers load before separation.
Hard single-arm tests verify stable three-body support, aligned versus rim-blocked
peg motion, object transport with the drawer, and containment through closing.
USB tests compare correct and reversed roll under the same load and retain native
contact force. Tube tests fill distinct back-row slots before front-row slots and
verify upright settled bodies. MuJoCo depth still reports ideal geometry for the
translucent tubes; D405/D435 invalid-depth behavior remains a sensor-parity gate.
Hard two-arm tests cover asymmetric tool pose, movable receiver insertion,
balanced versus one-sided lift, loose-content containment, finite cap retention
and release, and target-only bottle removal. D07-D10 require coordinated
multi-part execution from the downstream capability matrix.
They are development fixtures, not calibrated physical twins. Isaac and SAPIEN
selection is rejected until the same native evidence exists for those backends.

The reference layouts preserve the same task geometry while fitting each robot's
physical workspace. In particular, SO-101 drawer cabinets sit 40 mm nearer the
base than the YAM/xArm7 cabinets. Other close-clearance workpieces use common
positions that all three families can reach. The drawer and rack tasks use an
elevated scene-camera mount so the open drawer interior and back rack row remain
visible through the ordinary camera stream.

The reference MuJoCo evaluator reset accepts an explicit `SimulationVariation`
profile with independent `pose`, `appearance`, `physics`, and `geometry`
dimensions. Each dimension is `canonical`, `bounded`, or `held_out`. The
development implementation provides:

- bounded XY/yaw changes for each task pose group, preserving a fitted or loaded
  free assembly as one rigid group;
- per-prop RGB factors within 14% and a shared lighting factor within 12%;
- per-prop mass/inertia, friction, and passive-joint damping factors within 15%;
  and
- one task-prop geometry scale within 2%, or a disjoint held-out scale between 2%
  and 4% from canonical.

For the YAM `pick_lift` scene, a trusted simulation configuration may also set
`pose_profile` with `x_offset_m`, `x_half_range_m`, and `y_half_range_m`. It shifts
the center of noncanonical cube resets by `x_offset_m` from the canonical
`x=0.32 m`, then samples within the declared half ranges. Validation keeps the
bounded X range inside `0.24–0.34 m` and Y within `±0.06 m`. The default
distribution stays at `±0.012 m` in X and Y when this setting is absent. The
canonical seed still places the cube at `(0.32, 0) m`.

The same seed, environment, and profile reproduce the same resolved initial state
across worker restarts. Appearance does not alter mechanics, physics does not
alter geometry or appearance, and ordinary `reset()` restores every model and
state array to the canonical interactive scene. Seed `0` is reserved for the
fully canonical profile. These ranges are development settings pending physical
measurement; `held_out` identifies a disjoint development range and does not by
itself make a private benchmark distribution.

Variation is evaluator administration, so neither the seed nor the resolved
parameters are added to participant-facing site or observation contracts. The
privileged snapshot records only the selected profile and a digest of the resolved
initial variation alongside ordinary ground truth. Evaluation layers choose
profiles and seeds outside the participant runtime.

Before confirming an evaluator reset, reference MuJoCo checks the resolved native
contact set and refuses robot/table, robot/prop, or dual-arm cross-arm penetration.
The trusted administration boundary treats that refusal as a failed simulation and
requires the site to reopen. Complete-matrix acceptance additionally advances each
noncanonical state for 250 ms and verifies that the same clearance still holds
after gravity and contact settling.

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

SAPIEN imports packaged high-resolution meshes of the same first-party SDFs
through PhysX's native mesh cooker. Its cap, roof, mass, center of mass and inertia
use the same assembly as MuJoCo. Native contact supplies the thread coupling;
the cap has no guide or attachment. This path needs neither MuJoCo nor a compiler
in the worker. GPU episode reset restores both joint and free-body state.

Isaac references a packaged USD version of those exact surfaces and the same
mass, inertia and roof. It selects GPU dynamics and GPU broadphase for the bottle
scene, with a native SDF collider on the free cap and a static triangle-mesh neck.
This follows [NVIDIA's SDF collider setup](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/collision.html#create-an-sdf-collider).
Geometry conversion happens offline; its worker needs no additional converter,
MuJoCo installation or compiler for the cap. Native SDF cooking adds startup work
and GPU memory use. These USD checks do not prove Isaac rendering, reset, contact,
robot control or task completion; run the licensed native suite before qualifying
an Isaac installation.

The native rendering-preset, episode reset, shared-world lifecycle and custom
camera/part-name tests include Isaac. They skip unless `WADDLE_ISAAC_TEST_PYTHON`
names an operator-prepared, licensed interpreter. From the SDK Python project,
select it explicitly when running `pytest tests/test_simulation.py -k isaac`.
Collecting these tests or skipping them does not qualify that installation.

The reference robot profiles preserve their source joint names, order, limits,
radian units, FK, and normalized hand action (0 closed, 1 open). SO-101
uses Robot Studio's maintained five-axis arm, Feetech gripper range and wrist-camera
mount. YAM uses the SDK's pinned I2RT URDF and tool convention, assembled with the
live adapter's 95 mm LINEAR_4310 hand. xArm7 uses UFACTORY's expanded URDF with
the G2 gripper and its standard TCP. The original visual meshes, link masses,
centers of mass and inertia tensors are retained. Concave collision meshes are decomposed offline into
convex pieces so each engine preserves arm recesses, finger geometry and housing
clearance. Convex meshes remain unchanged. Planning models for every reference
family group complete source collision pieces by centroid into deterministic
20 mm bands along each link's longest axis and use the convex hull of every piece
in a band. This bounds model size while preserving
the complete source surface inside each planner hull; the serialized mesh retains
MuJoCo's hull boundary instead of the much larger input triangle set. Native constraints
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

The YAM reference scene uses a collision-clear tabletop working pose with its TCP
near `(0.28, 0, 0.16)` m. This is a reference working pose; the manufacturer's
model and joint limits are unchanged, and it does not specify a physical arm's
resting configuration. SO-101 uses Robot Studio's maintained box-pickup posture;
xArm7 retains its established reference posture.
The bottle sits to the side of this central region, and the cabinet's closed
front sits beyond it, so neither prop intersects the starting hand. The drawer
handle travels from x=0.503 m to x=0.283 m as it opens.
New drawer and single-arm MuJoCo development-task sites place the scene camera in front of
the interactive fixture, at
`(-0.45, -0.55, 0.55)` m looking toward `(0.4, 0, 0.2)` m, so its handle face is
visible at the reference home and task targets do not face away from the camera.
The dual-arm sorting scene uses an overhead camera that shows both arms, cubes,
and bins in the shared workspace.
The other scenes retain their overhead oblique view. Existing scene files retain
their explicit camera transforms; updating a transform also requires regenerating
calibration artifacts bound to that scene.
Free-cap tests check axial-load retention, thread pitch, natural exit, subsequent
free-body motion and reset. SAPIEN and Isaac share the same physical assertions;
Isaac cases require an explicitly selected licensed test interpreter.
The SDK's distinct high bimanual test poses are not used as reference-scene homes.
Opening or explicitly resetting a world establishes
this pose. Ordinary run boundaries preserve the current pose. Camera calibration
and robot kinematics are independent of this initial joint configuration.

Sources, licenses, conversion steps and SHA-256 hashes ship in
`waddle_sdk/simulators/data/`. `tools/vendor_simulation_models.py` rebuilds those
assets from pinned public manufacturer revisions. No model is downloaded at site
startup. `tools/vendor_thread_model.py` rebuilds the native thread collision
assets from the installed first-party SDFs; `tools/vendor_physx_thread_model.py`
rebuilds the finer PhysX surfaces. `tools/vendor_thread_usd.py` packages those
surfaces as a self-contained USD assembly without launching Isaac. The data README records versions and
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
state or changing a target. Rendering and IPC delays advance physics under the
previous target; a new command is never applied retroactively. Catch-up work has
a 20 ms wall-time budget, checked between native steps. If physics cannot keep up,
the worker discards remaining wall-clock lag and warns once, so controls and
sensors keep responding while simulated motion runs slower than real time.
This follows [PhysX's overload guidance](https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/docs/BestPractices.html#the-well-of-despair).
Native timesteps, forces and measured velocities are preserved. Explicit rollouts
execute every requested substep without this budget. Fractional substeps carry
between requests unless overload rebases the clock. State reporting uses the
ordinary SDK pump at the declared physical command rate: SO-101 30 Hz, YAM 10 Hz,
and xArm7 50 Hz. New scenes use a 1 rad/s joint-speed limit. Explicit site settings
remain authoritative. Available compute still bounds achievable throughput.
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

## Trusted task-evaluator control

An automated task evaluator may retain an explicit administration capability
while giving its participant only the ordinary SDK runtime:

```python
from waddle_sdk import load_site
from waddle_sdk.simulation import SimulationAdministration, SimulationVariation

administration = SimulationAdministration()
with load_site("site.yaml").open(
    simulation_administration=administration,
) as session:
    # Give only `session` (or an SdkRuntimePort facade over it) to the participant.
    initial = administration.snapshot()
    # After the participant's tool and every admitted motion are terminal:
    next_initial = administration.reset(
        seed=1234,
        variation=SimulationVariation(
            pose="bounded",
            appearance="bounded",
            physics="bounded",
        ),
    )
```

The capability binds only to a fully simulated site whose worlds implement the
optional `SimulationAdministrationBackend` facet. It is unavailable before site
open and after close. A failed or refused reset becomes uncertain and latches the
capability until the site is reopened; it is never retried automatically.

`snapshot()` returns immutable evaluator-owned state plus a content digest and an
episode revision. MuJoCo facets report named joint position and velocity vectors,
named body poses and velocities, contacts, simulation time, and a schema version.
Reference task engines additionally report an evaluator-only contact-event summary
accumulated at every physics step in the current episode. Each finite named-geometry
pair retains its first and latest observed time, minimum distance, maximum normal
force, and sample count. This lets a slower evaluator recognize a brief contact or
force peak after it has ended. The summary is bounded by the model's geometry pairs
and reset clears it. It is event evidence only: current grasp, release, pose,
velocity, and dwell predicates must continue to use the current snapshot.
Reference task presets also bind the provider/revision, robot family/embodiment
revision, arm count, environment ID, scene revision, and asset revision used to
resolve the world, so a higher-level evaluator can fail closed on a mismatched run
request. Portable compiled scenes expose state and reset but omit that identity until
their optional `metadata.identity` declares every required revision; the compiler
adds the robot count and the backend refuses a mismatch. Generated
`simulation.json` files carry the scene, asset, and embodiment revisions; older files still run
interactively but report missing revisions and cannot satisfy an exact task-run
identity check. This state is privileged ground truth. Do not pass the
capability, snapshot, seed, predicate, or resolved initial state to participant
code, prompts, tools, workspaces, errors, camera metadata, or participant traces.
Process and filesystem isolation remain evaluator responsibilities.

`reset(seed=..., variation=...)` validates and copies an optional complete
variation profile, then serializes against SDK action dispatch, world stepping,
capture, and shutdown. A backend that does not implement the selected profile
must refuse it. Reset restores and varies simulated robots, props, sensors,
controls, and clock, then returns the new initial snapshot. It does not end or
replace the active SDK run, clear a caller's trace or counters, or drain a
higher-level motion executor. The evaluator must pause participant dispatch and
confirm that the current tool call and all admitted operations are terminal before
reset. A reset advances only the administration episode revision, so prepared
plans from a higher layer must bind and check that revision themselves.

The built-in trusted facet is currently available for MuJoCo. Other installed
worlds remain valid interactive simulations and can opt in by implementing
`evaluation_snapshot()` and `evaluation_reset(seed=..., variation=...)` on their
backend. An
evaluation that requests administration fails closed if any selected world or
device is outside that complete simulated authority.

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
    width=640, height=480, render_quality="standard", arms=1,
)
(root / "site.yaml").write_text(yaml.safe_dump(site, sort_keys=False))
(root / "simulation.json").write_text(json.dumps(simulation, indent=2))
with load_site(root / "site.yaml").open(console=False) as session:
    print(session.observe())
```

`make_site()` is non-opening. One-arm sites expose part `arm` and cameras `scene`
and `wrist`. `arms=2` creates a shared MuJoCo world with `left` and `right`, base
frames for each placement, and cameras `scene`, `left_wrist`, and `right_wrist`.
The other reference engines currently accept one arm. Part-scoped reads, writes,
holds, resets, TCP poses, and wrist images address the same shared physics state.

Names, resolutions, intrinsics, and optical transforms are explicit configuration,
not engine defaults. Pass `camera_profiles` with exactly one row per generated
camera to bind a physical calibration. Each row has `stream`, `intrinsics`, and a
4x4 `transform`; the transform is world-from-camera for `scene` and TCP-from-camera
for a wrist camera. The site and simulator documents are generated from the same
rows so they cannot silently drift. Omitting the profile selects visible reference
defaults that are useful for development but are not physical calibration evidence.
Robot part names and base-frame labels also come from the site; when renaming a
part, update its wrist-camera owner in both documents. A base-frame label names the
robot's existing base coordinates and does not transform them.

| Robot family | Wrist camera | Scene camera | Depth |
| --- | --- | --- | --- |
| SO-101 | Configured RGB camera | Configured RGB camera | Unavailable |
| YAM | RealSense D405 | RealSense D435 | Available |
| xArm7 | RealSense D435 | RealSense D435 | Available |

Dual sites repeat the wrist-camera row for `left` and `right`; they never mix arm
families. Sensor model labels and depth capability are fixed by the robot profile.
The calibration profile supplies device-specific stream and optical values.

## Sensor and frame contract

RGB is RGB8. YAM and xArm7 also expose Z16 axial optical depth, and each RGB-D pair
comes from one frozen physics state. Their `depth_scale_mm=1` reference default means
millimetres per integer; zero denotes missing or out-of-range depth. SO-101 scene
and wrist cameras expose RGB only, return no depth sample, and omit depth scale.
Intrinsics are rectified pinhole intrinsics. The development defaults are 640×480
at 30 Hz for SO-101 and 15 Hz for YAM/xArm7, with explicit focal lengths and
principal point. Replace these with measured profiles for physical parity. Actual
frame rate depends on rendering performance. If capture or publication
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
quaternions, in the site's declared base frame (`so101_base`, `yam_base`, or
`xarm_base` by default; dual sites prefix the part name).
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

These presets add coupled grippers and specialized threaded objects beyond the
generic URDF compiler's supported scene subset. The portable compiler supports
fixed/free bodies and passive one-axis slide/hinge fixture composition. For custom
URDF bundles, use that portable scene compiler; for an already configured Isaac
stage, the existing ROS 2 backend is also available. No second SDK lifecycle or
agent API is introduced.

`reference_model_sources(robot, part_name=...)` supplies a non-opening, hash-bound
planner model for each generated arm. It uses the same pinned kinematic chain,
joint limits, base, TCP, and source collision links as the runtime assembly. Every
reference family uses deterministic spatial hulls of complete source collision
pieces. The model records the method, source-piece count,
planner-geometry count, and spatial bucket width in `collision_proxy` provenance.
All variants freeze the complete hand at maximum opening and omit task props and
another arm; those limits remain explicit in provenance and require separate
fixture/inter-arm checks for safety qualification.

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
(reviewed through 2026-09-11): Robot Studio's pinned SO-ARM100 assembly, MuJoCo
Menagerie (2026-09-04), I2RT 1.3.5 (2026-09-07), mjlab (2026-08-31), ManiSkill
(2026-08-02), and Isaac Lab (2026-09-04).
Exact revisions, source paths, mechanism choices and limitations are recorded in
the packaged `waddle_sdk/simulators/data/README.md`.
All three backends use native URDF import and physics constraints. Engine-specific
geometry conversion, axis handling and inertia conversion stay in the importer.
