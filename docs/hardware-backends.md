# Porting a hardware or simulator backend

This is the customer-facing boundary for adding a robot, simulator, or camera to
`waddle-sdk`. An SDK-only integration implements SDK contracts only. It does not
import `waddle-metal`, closed Waddle, or any internal service, and neither Metal nor
Waddle is required to open, observe, command, protect, or record the hardware.

When Metal is present, it consumes the same public SDK description and optional
runtime facets. Metal can use a hardware-specific implementation exposed by the SDK,
use a generic implementation when its declared prerequisites are present, or mark
only the dependent skill unavailable. The hardware backend itself does not select a
Metal implementation.

## The minimal robot backend

A robot package provides one importable factory and the objects that factory returns:

1. A `waddle_sdk.descriptors.Robot` declaration: canonical action space, joint order,
   limits and rates, plus optional portable URDF and frames.
2. A lazy `build_arms()` callable that opens the hardware and returns one
   vendor-neutral `waddle_sdk.robots.base.Arm` per declared part.
3. A `waddle_sdk.robots.base.Rig` containing the declaration, `build_arms`, control
   rate, and posture.
4. A structural `waddle_sdk.robots.Driver` for each opened arm.

The manifest names the factory directly:

```yaml
parts:
  arm:
    driver: customer_robot.backend:arm
    posture: supervised
    connection: {device: can0}
    joint_limits: {}
```

Prefer a factory accepting `config: waddle_sdk.robots.site.PartConfig`. It can read
the selected connection, posture, owner-supplied joint/workspace bounds, static
envelope rules, options, and confined site root. Calling the factory must not open a
bus or start a thread; hardware opens only when the SDK later calls `build_arms()`
inside `Site.open()`.

There is no vendor registry or central switch to patch. `site.yaml` resolves the
declared `module:factory`, the factory returns `Rig`, and the SDK composes it with the
same gate, owner envelope, recording, and cleanup used by built-in adapters.

The maintained implementation template is [Write your own vendor module](../sdk/README.md#write-your-own-vendor-module).
That code block is copied from the toy backend between the `--8<--` markers in
[`test_robots_base.py`](../sdk/tests/test_robots_base.py), and a test requires the two
copies to remain identical. Start from that template rather than copying a built-in
vendor module or inventing another lifecycle.

### `Driver`: the hardware lifecycle and I/O seam

`Driver` is structural: inheritance is unnecessary. An object satisfies it by
providing these members:

| Member | Contract |
|---|---|
| `kind` | Use `"sim"` only for a harmless twin. Every other value is conservatively treated as hardware that can hurt somebody. |
| `estopped` | Whether the owner's stop latch is set. |
| `read()` | Return `(joint_position, joint_velocity)` arrays in the declared joint order and width. This is the source of SDK part observations. |
| `write(target)` | Latch one joint-position target already admitted by the owner envelope. |
| `hold()` | Hold the unit at its current position. |
| `estop()` | Set the owner's stop latch; later writes must remain refused. |
| `re_enable()` | Clear that local latch through the site operator recovery path. |
| `step(dt)` | Advance a simulator; real hardware normally performs no work here. |
| `home(values)` | Attempt the declared reset pose and return whether the unit moved there. |
| `close()` | Release the connection. Half-open and normal session cleanup call this for every opened unit. |

`re_enable()` is not the protocol's `VERB_RESUME`. The former clears an owner-side
e-stop latch at the machine; the latter releases a Waddle hold and exists only when an
actual resume callable is registered. Likewise, implementing `home()` supports the
rig's reset lifecycle but does not manufacture a `VERB_HOME` grant. Grants are derived
from the control callables the opened session really registers.

The shipped `Rig` composition currently maps `posture: supervised` to send, hold, and
e-stop callables; `posture: monitor` registers only e-stop and cannot be commanded.
Posture never decides who may command the robot. Claims, leases, and the single-writer
decision remain native-core behavior.

Every command still crosses `Arm`, which applies the owner's declared joint limits,
per-command travel caps, optional workspace, and configured static collision rules.
The complete target is accepted or refused; it is never clamped into a command the
caller did not write.

### Optional initializer safety presets

A robot module may expose a configuration-only `safety_presets` function beside its
factory:

```python
from waddle_sdk.robots import SafetyPreset

def safety_presets(*, factory, options):
    if factory != "arm":
        return ()
    return (
        SafetyPreset(
            identifier="openarm-bench",
            label="OpenArm bench starter",
            workspace_bounds={
                "min": [-0.4, -0.4, 0.0],
                "max": [0.4, 0.4, 0.6],
            },
            review="Measure the mounting and bench before use.",
        ),
    )
```

`waddle_sdk.robots.safety_presets_for_driver()` imports that module without opening
hardware and returns immutable presets plus isolated warnings. A preset may suggest
workspace bounds, fixed keep-outs, and self-collision configuration. Initializers copy
the selected values into `site.yaml`; neither the identifier nor a new authority mode
reaches runtime. The site owner must still review the actual mounting, floor/table,
tooling, payload, and neighboring equipment. Adapters without enforceable FK or body
geometry should omit rules they cannot enforce rather than advertising optimistic
bounds.

### Optional position/velocity feedforward

A driver may additionally satisfy
`waddle_sdk.robots.PositionVelocityDriver.write_position_velocity(target,
velocity_feedforward_rad_s) -> bool`. This receives only a trajectory producer's
known velocity for an already-admitted position target. Return `True` when both were
accepted or `False` after deliberately issuing the identical position-only target.

This extension is optional. Without it, position motion continues through
`Driver.write()` unchanged. A backend must never derive feedforward by differentiating
measurements or an IK stream.

## Optional kinematics and body geometry

The robot backend does not implement another session or registration interface. It
adds optional functions to the existing `Arm`:

- `fk(q) -> (position_xyz, rotation_3x3)` evaluates the first `arm_dof` rows in the
  arm's declared `base_frame`.
- `collision_spheres(q) -> Sequence[base.CollisionSphere]` returns deterministic,
  conservative named body geometry in `collision_frame`.

Supplying `workspace` requires `fk`. Enabling configured static keep-outs or
self/cross-part collision requires usable body spheres and a compatible collision
frame. Those are owner-envelope requirements and fail closed; they are not Metal
safety claims.

For an opened local session, the SDK automatically exposes these same callbacks as
`waddle_sdk.runtime.SdkKinematicsPort.forward_kinematics()` and
`SdkGeometryPort.body_geometry()`. A customer backend therefore implements FK or body
geometry once. It does not implement the runtime ports separately and does not add a
Metal adapter. A frame-tagged FK call requires a non-empty `Arm.base_frame`; omitting
the frame leaves the callable unusable for frame-aware consumers.

A portable `Robot.kinematics_urdf` is separate from the callback. It lets a higher
layer construct a generic model implementation. A backend may provide the model, the
SDK callback, both, or neither.

## Cameras

A camera manifest entry names an importable camera factory. The factory accepts
`waddle_sdk.cameras.site.CameraConfig` (or the documented legacy width/height/fps
keywords), opens the device only when the site opens, and returns a structural
`waddle_sdk.cameras.CameraDriver`:

- `capture() -> CameraFrame` returns immutable RGB or pixel-aligned RGB-D data.
- `close()` is idempotent and must unblock a pending capture so shutdown can join the
  camera pump deterministically.

`CameraCalibrationDriver.intrinsics()` is an optional structural extension for a
driver that can report its active aligned color-grid intrinsics. Explicit
`site.yaml` intrinsics win. A `CameraFrame` may also attach a local point resolver for
vendor-correct distorted-depth deprojection; raw depth remains process-local.

The current support contract deliberately does not publish `camera.depth`: one
transient RGB-D sample is not a stable hardware declaration. RGB acquisition remains
available without depth, while operations requiring persistent metric depth must wait
for an explicit supported source instead of assuming it.

## Support facts, grants, and fallback

After hardware opens, `SiteSession.support()` returns an immutable
`waddle.sdk.support/v1` matrix and `SiteSession.describe()` publishes the same JSON.
Backends do not author this matrix. The SDK derives it from the exact registered
action space and grants, opened `Arm`/driver/camera implementations, and public site
facts.

A **support fact** says that one prerequisite exists, for example
`kinematics.fk`, `geometry.body_spheres`, `camera.intrinsics`, or
`limits.velocity`. It is not permission and is not a robot skill capability. A
**grant** is permission for one registered control verb. A higher layer may enable a
skill only from the conjunction of the exact action space, required live grants,
support facts, and its own healthy implementation/artifacts. A custom FK or planner
can replace one implementation term; it cannot create a send grant or an undeclared
action interface.

Omissions degrade independently:

| Omitted declaration or implementation | What remains | What cannot be assumed |
|---|---|---|
| Send grant | Observation, recording, judging, and declared sensors | Any motion, even if custom planning code is installed |
| Position/velocity limits | Operations not requiring the missing bound | A generic algorithm that requires that bound |
| `Arm.fk` and portable model | Joint observation and admitted joint motion | Generic FK, IK, or Cartesian planning |
| `Arm.base_frame` | Unframed joint behavior | Frame-tagged SDK FK and frame-aware Cartesian behavior |
| Body geometry | Joint/FK behavior not requiring it | Generic collision-aware planning and configured geometry-dependent claims |
| Workspace bounds | Other owner-envelope checks | A declared Cartesian workspace box |
| Position/velocity driver extension | The identical position target via `write()` | Hardware consumption of velocity feedforward |
| Gripper mapping | Arm behavior | Physical jaw-unit commands and gripper-dependent skills |
| Complete grasp geometry | Joint/gripper open-close behavior | Generic grasp, pick, or place geometry |
| Camera intrinsics | RGB acquisition and image-only consumers | Metric deprojection/localization |
| Camera entirely | Robot-only behavior | Skills requiring that camera scope |

Missing optional support must not prevent the site from opening unless the owner has
configured a hard envelope rule that requires it. Consumers mark only dependent
behavior unavailable and retain the rest of the matrix.

## Embodiment identity and hardware-specific matching

The support matrix carries a lowercase SHA-256 digest for the complete public site
embodiment and a separate digest on every robot-part and camera row. Exact per-scope
digests let a Metal-enabled installation select a hardware-specific implementation
without making unrelated changes invalidate it:

- A robot-part digest binds the support contract version, that part's exact action
  space, opened `base_frame`, relevant portable model/action-frame ancestry, and that
  part's gripper declaration.
- A camera digest binds the support contract version and that camera's exact public
  declaration.
- The composite digest binds the complete public site embodiment.

Grants, live runtime status, connection configuration and secrets, and unit/site
identity are excluded. An unrelated camera change therefore does not change a robot
part's digest. Missing, malformed, uppercase, or conflicting scope digests are not an
exact match; consumers must not substitute the composite digest for part matching.

## What an SDK-only customer owns

An SDK-only customer package needs only:

- its `Robot` declaration and site factory;
- its structural `Driver` and optional `PositionVelocityDriver` implementation;
- optional `Arm.fk` and `Arm.collision_spheres` callbacks;
- optional camera factories and `CameraDriver` implementations; and
- its measured owner-envelope values and site configuration; and
- optionally, configuration-only safety presets that give initializers reviewed
  hardware-aware starting values without opening a device.

That package can run locally, connect an SDK site, enforce the owner envelope, and
record episodes with no Metal or Waddle import. If the customer later installs Metal,
the same contracts and derived support matrix are the entire downward integration
surface; no patch to SDK dispatch or closed Waddle is required.
