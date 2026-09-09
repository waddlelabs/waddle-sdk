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
opposing-driver mimic, with one driven motor. This avoids overconstraining
the four-bar mechanism with six independent position drives.

## Collision and dynamics

Visuals always use the manufacturer's complete meshes. Collisions use its
declared collision meshes, with finger geometry and the G2 housing/cable mesh
decomposed offline by CoACD into at most 32 convex pieces per link. This retains
finger recesses and the empty space around the cable. All engines use those same pieces.
Fixed frame links retain physical adjacency; exclusions inside the G2 linkage
cover its connected pins and finger/base pairs declared noncolliding by the
manufacturer's `xarm7_with_gripper.srdf`. There is no exclusion between a robot
and a task object. Native MuJoCo equalities and PhysX tendon/mimic constraints
enforce the hand's relations.

Planning bounds are conservative sphere covers of the same mesh triangles,
transformed through the same URDF. Manufacturer masses, COM offsets and full
inertia tensors are preserved. Reference position servos, gravity compensation
and contact friction are simulation settings, not measured actuator dynamics.
Known trajectory velocity uses the existing SDK optional driver port; no velocity
is inferred from successive position commands. Ordinary position commands and
holds clear velocity targets, matching the native PD interfaces. Adapters
report hand position and velocity from the driven joint's encoder, matching
the hardware convention rather than averaging passive linkage joints. Task props
remain primitive reference models. The screw uses a native joint equality in
MuJoCo and native constraints in PhysX; no per-step force callback moves the cap.
SAPIEN's fixed tendon uses coefficients `[0, 1, -pitch]` for both length and
force, so generalized forces conserve work across metres/radians. Its reference
axial compliance is 5000 N/m with 20 Ns/m damping.

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
the hand uses its 100/10 position drive with a 50 Nm generalized-force cap.
These are reference controller configurations, not calibrated hardware twins.

Menagerie's YAM and mjlab's YAM lift task use an older crank hand. They are
references for implementation patterns, not replacements for LINEAR_4310.
I2RT's current `SimRobot.command_joint_pos` teleports coordinates, so it is
not used as a contact-physics backend. The xArm physics retains the G2 CAD and
manufacturer inertias; Menagerie's linkage anchor frames match its joint geometry.
SAPIEN uses ManiSkill's explicit zero joint-friction default and 15/1 solver
iterations. The SDK's existing shared-world robot pump owns physics time; the worker has no
independent clock or catch-up loop. The pump runs at the native physics cadence
(500 Hz by default), while the part declares its normal 50 Hz command rate and
retains the corresponding owner step limits. Runs preserve scene state by default.
Explicit `worlds.cell.options.reset_on_episode: true` uses native state
reset/snapshots; it does not recompile the robot or restart its renderer. The world
owns initialization, and per-arm episode hooks never home these robots again.

## Rebuilding

Run `tools/vendor_simulation_models.py` from the SDK repository using the
build-only package versions listed in its docstring. The script downloads
only pinned public manufacturer files, performs the described conversions,
and writes the assemblies and hash manifest. Those build tools are not runtime
dependencies. Commit the regenerated models, hashes and any changed provenance
together, then run the model, native engine, documentation and wheel checks.

`xarm7-kinematics.yaml` and `xarm-LICENSE` retain the original kinematic snapshot
used by the reference profile, from the same xarm_ros revision.
