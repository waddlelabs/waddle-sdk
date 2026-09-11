# Porting a custom hardware backend

A custom adapter is an ordinary installable Python package. A `site.yaml` names its
factory as `module:callable`; there is no Waddle registry to patch. The package
implements only public Waddle SDK contracts and must not depend on a higher product
layer.

## What a port contains

| Device | Required public surface |
|---|---|
| Robot or independent twin | `PartConfig` factory → lazy `Rig` → one `Arm` → structural `Driver` |
| Camera | `CameraConfig` factory → structural `CameraDriver` |
| Portable simulation source | `waddle.scene/v1` → installable compiler → native world + ordinary `site.yaml` |
| Shared simulation world | `WorldConfig` factory → `SimulationBackend` → optional part and camera facets |

A robust port also includes provenance for hardware facts, fake-vendor tests, an
example manifest, and a site-specific commissioning record. Optional forward
kinematics, conservative body spheres, camera intrinsics, and point resolution add
support facts without changing the required seam.

## Reading hardware metadata without opening devices

`waddle_sdk.robots.metadata.part_action_spaces(description)` resolves the named
part action spaces in a public `describe()` result. It supports arbitrary joint
counts and explicit composite parts; an unnamed multi-part space is not guessed.
`gripper_mapping(raw, joints)` validates an optional jaw-metres/action mapping
against its named action row and returns an immutable `GripperMapping`, or `None`
when unavailable. `opening(action)` preserves out-of-range measurements;
`action(opening_m)` refuses targets outside the physical range. Reversed action
ranges work without adapter-specific conversion code. Timing, command admission
and completion remain the application's responsibility and all actions still
cross the SDK gate.

`waddle_sdk.cameras.metadata.camera_declarations(description)` merges effective
runtime declarations with explicit site mounts/configuration. The runtime has
already applied explicit site intrinsics over optional driver intrinsics.
`camera_intrinsics(raw)` parses manifest and wire fields, retaining distortion
model identity. RGB-only calibration may omit a depth scale; metric consumers
must pass `require_depth_scale=True`. Missing/invalid values raise
`CameraMetadataError` with `intrinsics_missing` or `intrinsics_invalid`; callers
can disable only the dependent behavior for that camera. `CameraSample.point_at`
uses the same validation before resolving paired depth. No helper infers a mount,
rectifies distortion or substitutes another camera.

Robot source assets use the optional shared `ModelSourceProvider`/`ModelSources`
contract. The same resolver loads any configured adapter's source extension;
YAM is one implementation. See [source models](porting/source-models.md) for the
contract, independent adapter example, absence/failure behavior and source evidence.

The xArm G2 adapter uses UFactory's `get/set_gripper_g2_position` millimetre APIs.
Its underlying motor-pulse/linkage relationship is nonlinear: the vendor getter
uses a sine conversion and its setter the inverse arcsine conversion. That stays
inside the vendor API; our normalized action maps linearly to 0–84 mm opening.
Do not substitute the older raw-pulse `get/set_gripper_position` methods.
UFactory's [Python API](https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/xarm/wrapper/xarm_api.py)
and [implementation](https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/xarm/x3/gripper.py)
were reviewed on 2026-09-10. The current getter returns integer millimetres, so
this API does not establish sub-millimetre measurement precision. Physical jaw
calibration and replaceable-finger geometry still require site-specific evidence.

## Non-negotiable boundaries

- Importing the package and calling a part factory open no bus, device, or thread.
- Live hardware must never report `kind = "sim"`.
- The adapter does not implement claims, leases, handoffs, hosted behavior, or a
  parallel control surface.
- Limits come from reviewed vendor or unit-specific evidence. Do not invent, copy, or
  silently widen them.
- The owner envelope remains in force for caller, teleoperator, and higher-layer
  actions alike.
- Live motion starts only in an attended commissioning procedure with explicit site
  approval.

## Manifest composition

```yaml
parts:
  arm:
    driver: customer_robot.backend:arm
    posture: supervised
    base_frame: arm_base
    connection:
      device: /dev/customer-arm
    joint_limits: {}
    options: {}
```

The SDK imports `customer_robot.backend`, finds `arm`, and calls it as
`arm(config=PartConfig(...))`. The returned `Rig` is still a declaration: its
`build_arms` callback is where the device opens.

One manifest part factory must return:

- one bare action space, not `Composite`;
- one `Arm` when `Rig.arms()` is called;
- the manifest's declared base frame; and
- the same rate and posture as every other part in the site.

The site layer renames the returned arm to the manifest part name and combines all
part spaces into the registered composite declaration. This is why an adapter package
does not need multipart composition logic.

`PartConfig` carries the part name, posture, connection values, owner-supplied joint
and workspace bounds, static envelope configuration, base frame, adapter-specific
options, and confined site root. Prefer this typed object over legacy keyword
factories. In particular, a legacy factory cannot accept configured static envelope
rules safely and the SDK refuses that combination.

A simulator that shares one scene, clock, or renderer across parts and cameras uses a
manifest `worlds` declaration instead. Each simulated part or camera names the world
instead of a device driver. The SDK opens that backend once, advances it once per
composite tick, and closes it after its cameras and arms. The complete contract and
MuJoCo and ROS 2 manifests, the URDF scene compiler, Isaac Sim composition, and the
installable backend/compiler entry-point groups are in
[simulation backends](porting/simulation.md).

## Work in this order

1. Record joint order, units, limits, control rate, step caps, hold/e-stop/recovery
   behavior, frames, and gripper mapping with source provenance.
2. Package a non-opening `PartConfig` factory and fake vendor API.
3. Implement and test the structural driver lifecycle.
4. Build the `Arm` envelope and exact `Robot` declaration from the same facts.
5. Add optional kinematics, body geometry, or cameras only when their facts are known.
6. Exercise the complete site with the fake backend and inspect its recording.
7. Commission the actual unit under the site's physical safety process.

Continue with [robot adapters](porting/robot.md),
[camera adapters](porting/camera.md),
[simulation backends](porting/simulation.md), and the
[validation and commissioning checklist](porting/testing.md).

## What higher layers learn

After open, `SiteSession.describe()` publishes the exact registered robot declaration
and grants. `SiteSession.support()` derives a versioned support matrix from those
declarations and the opened implementation. The adapter does not author support rows.

A support fact says that one prerequisite exists. It is not motion permission and is
not a robot skill capability. Missing optional facts degrade only dependent behavior
unless the owner configured an envelope rule that requires the fact, in which case
open fails closed.

Per-scope embodiment digests let a consumer match a hardware-specific implementation
without tying it to unrelated cameras or site identity. They exclude credentials,
connection details, grants, and live status. Adapters never calculate or override
these digests.
