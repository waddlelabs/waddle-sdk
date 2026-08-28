# Robot adapters

## The factory seam

Expose a callable that explicitly accepts `config: PartConfig` and returns a `Rig`:

```python
from __future__ import annotations

from waddle_sdk import descriptors
from waddle_sdk.robots import PartConfig, base


def arm(*, config: PartConfig) -> base.Rig:
    facts = load_reviewed_facts(config)
    space = descriptors.JointSpace(
        joints=[
            descriptors.Joint(
                name=name,
                min_position=lower,
                max_position=upper,
                max_effort=effort,
            )
            for name, lower, upper, effort in facts.joints
        ],
        rate_hz=facts.rate_hz,
        chunking=descriptors.Chunking(
            horizon=1,
            replan="immediate",
            interp="hold",
        ),
    )

    def build_arms() -> dict[str, base.Arm]:
        driver = VendorDriver(config.connection)  # device opens here
        try:
            opened_arm = base.Arm(
                part="",
                driver=driver,
                joint_names=facts.names,
                joint_limits=facts.limits,
                step_caps=facts.step_caps,
                rate_hz=facts.rate_hz,
                base_frame=config.base_frame or "",
                home_values=facts.home,
            )
        except BaseException:
            driver.close()
            raise
        return {"": opened_arm}

    return base.Rig(
        declaration=descriptors.Robot(
            name=facts.model,
            robot_id=facts.model,
            action_space=space,
        ),
        build_arms=build_arms,
        rate_hz=facts.rate_hz,
        posture=config.posture,
    )
```

`load_reviewed_facts()` must be pure configuration work. `VendorDriver` construction
belongs inside `build_arms()` because that callback is the hardware-opening boundary.
If anything after driver construction can fail, close the driver before re-raising.

The returned dictionary uses the empty internal part name. The site composer replaces
it with the manifest part name. Returning multiple arms or a composite action space
from one manifest part is rejected.

## Structural `Driver`

Inheritance is unnecessary. The object satisfies `waddle_sdk.robots.Driver` by
providing the following members:

| Member | Contract |
|---|---|
| `kind` | Exactly `"sim"` only for a harmless twin; every other value is treated as live hardware |
| `estopped` | True while the owner's stop latch is set |
| `read()` | `(joint_position, joint_velocity)` arrays in declared order and width |
| `write(target)` | Latch one already-admitted joint-position target |
| `hold()` | Stop commanded motion and hold the unit at its current state |
| `estop()` | Latch the owner's stop; later writes remain refused |
| `re_enable()` | Clear that latch only through the site operator recovery path |
| `step(dt)` | Advance a simulator; normally a no-op for live hardware |
| `home(values)` | Attempt the declared reset pose and report success |
| `close()` | Deterministically release resources; safe after partial initialization |

Methods must be thread-safe. Runtime-owned dispatch and pump threads can call the
driver concurrently with shutdown or observation. `close()` must be idempotent in
practice even where the structural type does not express that property.

## Declaration and envelope must agree

One facts source should produce the driver width/order, action declaration, `Arm`
joint limits, step caps, rate, and home values. Assert that agreement in tests. Every
number needs provenance a wheel holder can inspect: a vendor manual, shipped model,
or explicit unit measurement.

`Arm` is the one route to an SDK-managed driver. It validates a command against the
owner's declaration and refuses whole. The adapter should not add a second command
path around it.

## Optional extensions

### Position and known velocity

A driver may additionally implement
`write_position_velocity(target, velocity_feedforward_rad_s) -> bool`. The velocity
is a trajectory producer's known hint for the same already-admitted position target.
Return false only after deliberately issuing the identical position-only fallback.
Never differentiate measurements or an IK stream to invent feedforward.

### Forward kinematics

Pass `fk(q) -> (position_xyz, rotation_3x3)` to `Arm` when the implementation is
deterministic and frame-correct. The callable evaluates the first `arm_dof` rows in
the declared `base_frame`. A configured Cartesian workspace requires it.

### Conservative body geometry

Pass `collision_spheres(q)` and `collision_frame` when the adapter can return
conservative, named `CollisionSphere` values for every protected body. The adapter
reports geometry; SDK code owns intersection, margins, ignored pairs, and atomic
refusal. Configured keep-outs or self/cross-part collision fail closed if geometry is
missing or uses incompatible frames.

### Grippers and portable models

The manifest's `gripper` block maps a physical jaw opening to one declared action row
and may add complete grasp geometry. It is public metadata and is not forwarded into
the adapter factory. Unit-specific motor calibration stays in `options`.

A portable URDF can let a higher layer construct generic kinematics. It is independent
of the adapter's `fk` callback: publish either, both, or neither, and claim only what is
actually valid.
