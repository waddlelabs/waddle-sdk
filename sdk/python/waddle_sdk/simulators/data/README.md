# Manufacturer robot models

These are third-party robot descriptions and meshes, distributed under the
licenses inside each model directory. `models.json` records the public source
URLs, source SHA-256 values and hashes of every packaged model file. Loading a
reference robot verifies its packaged files before an engine sees the model.

| Assembly | Source revision | License |
| --- | --- | --- |
| YAM arm | [I2RT](https://github.com/i2rt-robotics/i2rt/tree/570ef66681ff12bd8298aba34084307cfecc9f05) | `yam/LICENSE`, MIT |
| LINEAR_4310 hand | [I2RT 1.3.5](https://github.com/i2rt-robotics/i2rt/tree/5b72c47239bd056d0fa6c1a39edeb0537c89443c) | `yam/LICENSE`, MIT |
| xArm7 and G2 hand | [UFACTORY xarm_ros](https://github.com/xArm-Developer/xarm_ros/tree/aad7e1611c9c46eb719045414394bfdd42dcb0f8) | `xarm7/LICENSE`, BSD-3-Clause |

## YAM assembly

The arm comes from the SDK's `robots/yam_data/yam.urdf`, with the same six
joint transforms, masses, inertia tensors, SDK limits and fixed `grasp_link`.
Its meshes are fetched from that URDF's pinned manufacturer revision.
The older hand housing at `link_6` is replaced by the manufacturer's
`i2rt/robot_models/gripper/linear_4310/linear_4310.xml` assembly: housing, two
fingers, original inertias and 47.5 mm travel per jaw. The native source XML
ships alongside the assembled URDF.

The fixed hand mount is `Rx(pi)` followed by `Rz(-1.5708)`. It preserves
all SDK arm joints and the declared `Tz(.1347) Rz(-1.5708)` TCP. I2RT 1.3.5
corrects the physical pinch position to the distal finger facets at
`(0.0000968103407, 0.0000388982673, -0.144650259)` m in the native hand frame.
The corresponding offset from the SDK TCP is approximately `(0.097, -0.039,
9.950)` mm. The scene builder derives that offset from the packaged XML; it
does not silently redefine the hardware TCP. The three hand meshes are byte
identical to the previous pinned revision. Public hand action is zero closed
and one open, with a 95 mm nominal stroke and two coupled physical slides.

`yam/wrist_camera.json` retains the optical frame of I2RT 1.3.5's published
LINEAR_4310/D405 bracket, extracted from its station URDF. The site expresses
that frame relative to the SDK TCP. This looks through the open jaw gap, instead
of placing the wrist camera behind a finger. Intrinsics remain the explicit
reference sensor profile; this does not substitute for physical calibration.

## xArm7 assembly

`xarm7/robot.urdf` expands the manufacturer's `xarm7_urdf` and
`xarm_gripper_urdf` macros with `gripper_version=G2`, the default xArm7
kinematics and `xarm7_type7_HT_BR2` inertial parameters. COLLADA hand meshes
are converted to binary STL with their scene transforms applied, preserving
metres and triangle geometry. Arm meshes are unchanged STL files. Material
textures are not copied; URDF material colors and neutral hand colors are used.

At load time the arm's `joint1` through `joint7` names map to the live SDK's
`joint_1` through `joint_7`; the scalar coordinates remain radians. The URDF's
`link_tcp` maps to the common `tcp` frame. Its flange offset is 0.172 m.
The G2 hand's joint angle is a nonlinear function of jaw opening: its outer
knuckle lever has offsets 0.035465 m and 0.042039 m. The native 0.85 rad closed
stop anchors the mapping; the nominal SDK opening is 84 mm. This preserves
physical jaw travel instead of treating normalized opening as a joint angle.
The URDF gives the geometric finger-angle relationships used for FK. Native
physics uses Menagerie's two ball-joint linkage closures and the single
opposing-driver relation. MuJoCo distributes one motor through a native fixed
tendon; Isaac retains one driven motor and a native mimic. SAPIEN retains the
native mimic while sharing the actuator budget across opposing drives.
This avoids overconstraining the four-bar mechanism with six position drives.

## Collision and dynamics

Visuals always use the manufacturer's complete meshes. Collisions use its
declared collision meshes. Every concave mesh is decomposed offline by CoACD
into at most 32 convex pieces per link; already convex meshes remain unchanged.
This retains forearm and finger recesses and empty space around housings/cables.
Applying this only to fingers left a convex hull across the YAM forearm recess,
producing false housing contacts during folded wrist motion. All engines use
the same decomposed pieces. No self-collision exclusion removes that contact pair.
Fixed frame links retain physical adjacency; exclusions inside the G2 linkage
cover its connected pins and finger/base pairs declared noncolliding by the
manufacturer's `xarm7_with_gripper.srdf`. There is no exclusion between a robot
and a task object. Native equalities, linkage constraints, and the SAPIEN coupled
jaw controller implement the hand's relations.

Planning bounds conservatively cover complete triangles per physical link,
independently of how native collision meshes are partitioned. They use the same
URDF transforms. Manufacturer masses, COM offsets and full
inertia tensors are preserved. Reference position servos, gravity compensation
and contact friction are simulation settings, not measured actuator dynamics.
Known trajectory velocity uses the existing SDK optional driver port; no velocity
is inferred from successive position commands. Ordinary position commands and
holds clear velocity targets, matching the native PD interfaces. Adapters
report hand position and velocity from the driven joint's encoder, matching
the hardware convention rather than averaging passive linkage joints. Task props
remain primitive reference models. The screw uses a native joint equality in
MuJoCo and native constraints in PhysX; no per-step force callback moves the cap.
The 60 g, 50 mm cubes explicitly declare uniform-body inertia of
`0.000025 kg m²` about each central axis. All engines import these mass properties
from the shared URDF. SAPIEN uses its native URDF actor builder for single-body
props as well as its articulation builder for robots and jointed props; setting
only actor mass after creation would retain inertia at the wrong density.
Standalone static scenery has no dynamic inertial record.
SAPIEN's fixed tendon uses coefficients `[0, 1, -pitch]` for both length and
force, so generalized forces conserve work across metres/radians. Its reference
axial compliance is 5000 N/m with 20 Ns/m damping.
MuJoCo's cap joint has 0.001 N m native `frictionloss`, preventing gravity from
back-driving the otherwise frictionless helix after release. This is reference
prop resistance, not measured seal torque. PhysX joint-friction coefficients
have different units/semantics; they do not receive that value as a coefficient.
SAPIEN 3 exposes only that legacy friction coefficient. The screw instead uses
a native zero-stiffness, zero-velocity drive with 1 N m s/rad damping and a
0.001 N m torque limit, following PhysX's
[joint drive/friction guidance](https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/_api_build/classPxArticulationJointReducedCoordinate.html).
This approximates dry resistance with a viscous transition below 0.001 rad/s;
it has no angular position target and does not latch a released cap. Native
retention tests start away from either end stop, bound released axial drift
to 0.1 mm over five seconds, and require turning under torque in both directions.

## Maintained implementation references

Reviewed on 2026-09-08. Repositories below had activity in the preceding year;
the revision pins make the basis reproducible even as upstream moves.

| Project | Reviewed revision / date | Reused basis |
| --- | --- | --- |
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/8161bba264d7fa7c99ca301e91e7fb44737676ad) | `8161bba`, 2026-09-04 | Native URDF import, reflected rotor inertia, xArm arm drives and four-bar linkage |
| [I2RT](https://github.com/i2rt-robotics/i2rt/tree/5b72c47239bd056d0fa6c1a39edeb0537c89443c) | `5b72c47`, 2026-09-07 | LINEAR_4310 model/TCP correction; YAM motor gains and gripper transmission |
| [mjlab](https://github.com/mujocolab/mjlab/tree/8ee51fbcf806a7419189f706d9e394cbeb7790fa) | `8ee51fb`, 2026-08-31 | Reflection of motor gains/inertia through a gripper transmission |
| [ManiSkill](https://github.com/mani-skill/ManiSkill/tree/62ff3a5896b4d5b4cf0ac4c8d79afe600c9404a3) | `62ff3a5`, 2026-08-02 | Native PD position/velocity drives, passive linkage closures, scene defaults |
| [Isaac Lab](https://github.com/isaac-sim/IsaacLab/tree/99f1423e5d4a26216c0eedeb2aa78099a8c3a7d1) | `99f1423`, 2026-09-04 | Isaac Sim 6 URDFImporter workflow; no hand-written mesh/inertia/axis conversion |

YAM gains come from I2RT `i2rt/robots/config/yam_v1.yml`: arm kp 80/10,
kd 5/1.5 for shoulder/wrist motor groups. LINEAR_4310 kp 20, kd 0.5,
6.57 rad motor stroke, 96 mm jaw stroke and 0.5 Nm force-limiter threshold
come from `linear_4310.yml`. Reflecting through one jaw's half-stroke uses
`r = .096 / (2 * 6.57)`: linear kp/kd/inertia divide by `r²`, force divides
by `r`. xArm arm gains and effort bounds use Menagerie's size1/2/3 drives;
the hand retains its 100/10 position drive, with the motor limit derived from
[UFACTORY's G2 AG1200 specifications](https://docs.accessories.ufactory.cc/xArm_Gripper_G2/6.Technical_Specifications.html)
(84±1 mm travel and 10–50 N gripping force; accessed 2026-09-09).
The manufacturer's finger-link offsets are `a=.035465`, `b=.042039` m.
For the parallel-jaw linkage, `abs(dwidth/dq)=2*(a*sin(q)+b*cos(q))` over
`q∈[0,.85]`. Virtual work gives `motor_torque=jaw_force*abs(dwidth/dq)`.
The minimum transmission is `2*b`, so a constant 4.2039 N m native motor cap
keeps the ideal quasi-static force below 50 N throughout the stroke. The
former Menagerie-derived 50 N m setting did not represent G2's jaw-force rating.
This is not a calibrated force controller or a bound on impact forces.
These are reference controller configurations, not calibrated hardware twins.

Menagerie's YAM and mjlab's YAM lift task use an older crank hand. They are
references for implementation patterns, not replacements for LINEAR_4310.
I2RT's current `SimRobot.command_joint_pos` teleports coordinates, so it is
not used as a contact-physics backend. The xArm physics retains the G2 CAD and
manufacturer inertias; Menagerie's linkage anchor frames match its joint geometry.
SAPIEN uses ManiSkill's explicit zero joint-friction default and 15/1 solver
iterations. Equal targets drive both opposing jaws, sharing the single actuator's
gain, force cap and reflected inertia equally. The manufacturer's native URDF
mimic relation is also retained: matching targets alone let the two joints drift
apart under asymmetric contact. With the shared drive load, the native tendon
need not transfer the entire motor force from one jaw to the other. A native
YAM handle regression checks jaw agreement within 0.5 mm and vertical TCP
retention within 1 mm. xArm's passive four-bar links retain native closure constraints.

MuJoCo follows Menagerie's [xArm](https://github.com/google-deepmind/mujoco_menagerie/blob/8161bba264d7fa7c99ca301e91e7fb44737676ad/ufactory_xarm7/xarm7.xml)
and [Robotiq](https://github.com/google-deepmind/mujoco_menagerie/blob/8161bba264d7fa7c99ca301e91e7fb44737676ad/robotiq_2f85/2f85.xml)
fixed-tendon transmission pattern: two coefficients of 0.5 distribute the original
single motor's force to the opposing drive joints. The total reflected inertia
is split equally; gains and force limits retain the original combined budget.
The hand mimic equality uses the source's 5 ms time constant. Passive xArm
four-bar links remain unactuated. Their MuJoCo joints retain the source's
inherited 0.1 kg m² armature, and driver/follower limits use its 5 ms response.
This numerical armature supports the soft closure constraints; it does not
replace or recalibrate the manufacturer's link inertia. Omitting it allowed
loaded fingers to fold well beyond their joint limits. Native contact regressions
check physical jaw width, settled motion, arm tracking and reopening.
This avoids making a soft equality transfer
the entire motor force from one jaw to the other.
Both reference hands also use the xArm model's finger-pad contact response:
`solref="0.004 1"`, `solimp="0.95 0.99 0.001"`, and contact priority 1. These
parameters apply to the same manufacturer finger collision meshes, without
increasing their material friction or replacing them with primitive pads.
This keeps the stiff linear hand from deeply penetrating a held cube and
oscillating under the default 20 ms contact response.

All MuJoCo reference scenes use elliptic friction cones, impedance ratio 10,
and Newton tolerance 1e-10, following the engine's maintained
[manipulation guidance](https://mujoco.readthedocs.io/en/stable/modeling.html#preventing-slip).
This reduces regularization-induced creep without extra friction, contact-force
callbacks, or NoSlip post-processing. The manufacturer's meshes and linkage
geometry are unchanged. The reference drawer has 5 N·s/m passive damping in its
URDF; Isaac applies the equivalent zero-stiffness velocity drive explicitly.

All three reference engines use 2 ms substeps. The generic ManiSkill 100 Hz
step left SAPIEN's coupled hand/contact solve with significant residual joint
velocities during a stationary grasp, preventing ordinary motion completion.
Reducing SAPIEN's step to 2 ms improves that solve while retaining its motor
budget and material friction. See PhysX's
[drive stability guidance](https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/docs/Articulations.html#articulation-drive-stability)
on competing drives/contact constraints and timestep size. This does not make
reported native velocities exact derivatives of sampled poses or calibrate the
reference servos. It performs five times as many native substeps as the earlier
SAPIEN default; SDK command and camera rates stay independent.

SAPIEN 3.0.3's native shape contact margin defaults to 10 mm. On the decomposed xArm
hand, closure generated 1,152 contact records and about 4.84 ms per physics step
in an isolated local diagnostic, exceeding a 2 ms real-time budget. Robot
shapes now use 2 mm margins, yielding 24 contacts and about 0.18 ms per step in
that same fixture. These timings are illustrative local measurements, not runtime
guarantees. Native rest offsets and material friction remain unchanged. This
follows [ManiSkill's contact-generation guidance](https://github.com/mani-skill/ManiSkill/blob/main/docs/source/user_guide/tutorials/custom_robots.md)
and [PhysX's contact-offset guidance](https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/docs/AdvancedCollisionDetection.html).
Custom faster dynamics require their own timestep/contact validation.

Reference workers default to real-time physics. Before each request they advance
elapsed monotonic time with the existing target, retaining fixed native substeps
and fractional remainders. Rendering/IPC delays do not discard simulation time
or apply new commands retroactively. `worlds.cell.options.real_time: false`
retains explicit SDK stepping for rollouts. The ordinary SDK reporting pump
runs at least twice the command rate with a 100 Hz floor; new scenes derive command-rate and speed defaults
from the physical SDK declarations (YAM 10 Hz, xArm7 50 Hz, both 1 rad/s). Available compute still bounds throughput.
Runs preserve scene state by default.
Explicit `worlds.cell.options.reset_on_episode: true` uses native state
reset/snapshots; it does not recompile the robot or restart its renderer. The world
owns initialization, and per-arm episode hooks never home these robots again.

Visual-only texture provenance is in `appearance/README.md`. Materials and fill
lighting affect RGB appearance, not the manufacturer geometry or contact settings.

## Rebuilding

Run `tools/vendor_simulation_models.py` from the SDK repository using the
build-only package versions listed in its docstring. The script downloads
only pinned public manufacturer files, performs the described conversions,
and writes the assemblies and hash manifest. Those build tools are not runtime
dependencies. Commit the regenerated models, hashes and any changed provenance
together, then run the model, native engine, documentation and wheel checks.

`xarm7-kinematics.yaml` and `xarm-LICENSE` retain the original kinematic snapshot
used by the reference profile, from the same xarm_ros revision.
