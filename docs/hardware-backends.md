# Porting a custom hardware backend

A custom adapter is an ordinary installable Python package. A `site.yaml` names its
factory as `module:callable`; there is no Waddle registry to patch. The package
implements only public Waddle SDK contracts and must not depend on a higher product
layer.

## What a port contains

| Device | Required public surface |
|---|---|
| Robot or simulator | `PartConfig` factory → lazy `Rig` → one `Arm` → structural `Driver` |
| Camera | `CameraConfig` factory → structural `CameraDriver` |

A robust port also includes provenance for hardware facts, fake-vendor tests, an
example manifest, and a site-specific commissioning record. Optional forward
kinematics, conservative body spheres, camera intrinsics, and point resolution add
support facts without changing the required seam.

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
[camera adapters](porting/camera.md), and the
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
