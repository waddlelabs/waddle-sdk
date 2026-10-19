# Robot adapters

## OpenArm observation bring-up

`waddle_sdk.robots.openarm:arm` supplies one seven-axis OpenArm plus its eighth
gripper motor through the ordinary named-part Site lifecycle. Declare two parts
for two independently connected buses. This stage implements **monitor only**:
any other posture fails before opening hardware. It has no motor-enable,
motor-disable, control-mode, zero-setting or motion command encoder. Opening,
observing and closing send only Damiao parameter reads (`0x33`) and state refresh
requests (`0xCC`) to arbitration ID `0x7ff`. Unsupported write, hold, e-stop and
re-enable methods raise `RuntimeFault(UNSUPPORTED)` without sending a frame.
It does not physically stop another controller's motion.

The adapter requires 64-bit Linux SocketCAN; no vendor Python package is needed.
The factory is non-opening. Its connection fields are:

| Field | Meaning |
| --- | --- |
| `interface` | Exact Linux interface name, such as `can0`; never selected by enumeration order. |
| `usb_serial` | Exact serial of that interface's USB adapter, checked before link setup and after socket binding. |
| `motor_ids` | Eight unique standard CAN transmit IDs, in seven arm motors then gripper-motor order. |
| `feedback_ids` | Eight unique standard CAN reply IDs in the same order; queried MST_ID/ESC_ID must match. |
| `bitrate` | Positive integer bit/s, default `1000000`. |
| `configure_can` | Default `false`: require an already-up link at the declared bitrate. Explicit `true` uses the ordinary bounded SocketCAN setup helper; an up mismatch is refused. |

An interface opened with `configure_can: true` remains up after socket close,
as with the shared SocketCAN helper. The site operator owns subsequent link
restoration. Adapter serial binding identifies a bus, not physical left/right
placement. Reserved motor SN/hardware fields are not used as unique identities.
A serial-keyed local lock prevents duplicate OpenArm adapter owners (including
different Site IDs) for one OS user; it cannot exclude unrelated raw CAN programs
or other users.

Supply eight explicit finite `joint_limits` pairs, or a mapping keyed by
`joint_1` through `joint_7` and `gripper_motor_rad`. These are the site's
declared coordinate bounds, not evidence of commissioned safe motion. The
adapter publishes raw encoder radians and rad/s. No encoder-zero calibration,
joint direction, FK, physical jaw metres, gripper force, velocity limit, body
geometry or source model is inferred. In particular, the queried PMAX/VMAX
values only decode packed state feedback; they are never used as mechanical
limits. The eighth coordinate is **gripper motor radians**, not jaw opening.
Workspace bounds requiring FK are refused. Use reviewed physical mappings and
models before a subsequent motion-capable adapter stage.

Options are `rate_hz` (default `10`, positive and at most `100`) and `timeout_s`
(default `0.2`, positive and at most `1`). All parts in one Site still use the
same declared rate. Each read serializes a complete eight-motor request/reply
batch against the background pump. Queued old frames are drained; matching
arbitration and motor IDs plus Linux receive timestamps reject pre-request
traffic. Missing, malformed, nonfinite or faulted feedback raises a scoped
error instead of returning a partial batch, zero velocities or cached samples.
After a transaction failure the handle refuses further queries until reopened,
so a delayed failed reply cannot satisfy a retry. No automatic retry extends
the deadline.

Damiao feedback has no transaction counter or acquisition timestamp. The retained
`MonitorDriver.last_feedback` records kernel receipt times, motor status and
temperatures; these are **receipt freshness**, not proof of simultaneous motor
sampling or causal pairing with the refresh. Both ordinary disabled (`0`) and
enabled (`1`) states are readable; other status nibbles fail with original motor
ID/status/payload evidence. The monitor does not alter the motor's enable state.

The wire implementation follows the pinned manufacturer
[OpenArm CAN encoder/decoder](https://github.com/enactic/openarm_can/blob/f340d4b808fb177e1f297af54eb55fd51c6c7c10/src/openarm/damiao_motor/dm_motor_control.cpp)
and [status/register definitions](https://github.com/enactic/openarm_can/blob/f340d4b808fb177e1f297af54eb55fd51c6c7c10/include/openarm/damiao_motor/dm_motor_constants.hpp).
`sdk/tests/test_robots_openarm.py` exercises the fake-wire and ordinary monitor
Site paths, including cleanup, stale/malformed/faulted replies and duplicate bus
owners. These tests establish software behavior, not physical motion acceptance.
The maintained opt-in `sdk/tests/live/test_05_openarm_monitor.py` additionally
checks both declared buses through a real passive Site, with guarded wire capture
and advancing kernel receipt evidence. See the live test guide for selection and
retained JSON evidence. This case never commands physical motion.

## Attended OpenArm single-motor commissioning

`waddle_sdk.robots.openarm_commissioning.CommissioningOwner` owns one explicitly
bound bus and permits actuation of exactly one declared motor address. It is a
commissioning API, separate from the monitor-only `arm` factory. It does not open
a Site/run or grant ordinary robot motion capabilities. Supply all eight motor
addresses and exact adapter serial. The same local bus lock excludes simultaneous
monitor and commissioning owners. External raw-CAN programs remain outside this
lock and must be closed.

Construction reads identity, PMAX/VMAX/TMAX, current mode/TIMEOUT and fresh status
only. Initial mode must be POS_VEL (`2`) and every peer must be disabled. Explicit
`Guards` require whole-bus actuator support/clearance, an attended cutoff, and
positive local excursion, speed, nominal effort, temperature and peer-drift
bounds. These bounds describe this attended experiment; encoding ranges and
monitor manifest limits do not establish mechanical safety. Raw motor IDs need
not yet be assigned physical joint or left/right labels when support and
clearance cover every possible actuator, including the gripper.

`run(offset_rad=0)` performs watchdog qualification only. It sets TIMEOUT to
2000 ticks in RAM, switches the disabled selected motor to MIT, seeds zero KP/KD
and nearest-representable zero feedforward, requires fresh enabled feedback,
then sends no frames for 200 ms. A subsequent fresh batch must show selected
communication-loss status `13` and disabled peers. This deliberately induced
fault is cleared once with `FF FF FF FF FF FF FF FB`; fresh refresh must show
normal disabled `0`, followed by a confirmed disable command. An immediate
clear-command reply is not required. Unexpected faults are never cleared.
The manufacturer [control protocol V1.4](https://gitee.com/kit-miao/damiao-document/blob/master/%E8%B0%83%E8%AF%95%E5%8A%A9%E6%89%8B%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6%EF%BC%88%E8%BE%BE%E5%A6%99%E9%A9%B1%E5%8A%A8%E6%8E%A7%E5%88%B6%E5%8D%8F%E8%AE%AE%EF%BC%89V1.4.pdf)
specifies 50 microseconds per tick. This test measures withdrawal by the next
observation, not exact firmware timing or whether read/refresh traffic resets
the watchdog. Mode switching clears command values according to the
[manufacturer protocol description](https://damiao.enactic.ai/en/products/hardware/dm-j4340p-2ec-v1.0/).
Firmware behavior must still pass the selected physical qualification.

A nonzero offset additionally executes one raw-coordinate out-and-back after
qualification: explicitly configured offset at most 0.1 rad, KP at most 2,
positive KD at most 0.5, reference slew at most 0.025 rad/s, total run at most
ten seconds. Defaults remain 0.03 rad, 0.02 rad/s and KD 0.1. A larger local trial or
damping change requires
a separately reviewed profile and excursion window; shortfall never changes
these settings automatically. Every actually
encoded reference is checked against the original local window; implied nominal
effort uses decoded quantized position, velocity, KP, KD and feedforward.
Feedback checks raw excursion, speed, nominal effort, temperature and peer state.
Local guard violations report `safety_refusal`; received motor fault statuses
retain `motor_failure`, while packet/timeout failures retain `transport_lost`.
MIT's symmetric 12-bit velocity/torque encoding has no exact zero. Low gains and
these checks are not a hard motor current cap or calibrated torque estimate.
POS_VEL's speed ceiling alone is not a force limit.

Every run is terminal. `close()` requests cancellation before waiting for the
current bounded transaction. Ordinary reads can overrun the run deadline by at
most their configured transaction timeout; cleanup has separate finite deadlines.
Cleanup attempts selected-motor disable even after a poisoned transaction or
receive error, requires fresh disabled status, then restores original mode and
TIMEOUT with write acknowledgements and readback. An uncertain disable prevents
parameter restoration. No zero-setting, flash storage, automatic re-enable or
gain escalation is available. Returned reports preserve original and cleanup
faults independently, quantized commands, encoder arrival/nonarrival and host/
kernel receipt times. Receipt times do not establish sensor acquisition timing.
Use the maintained live commissioning case described in the live test guide;
review a watchdog-only result before a separate attended motion trial.

For a retained watchdog-only failure that left status `13` latched,
`CommissioningOwner.recover_watchdog(prior_report=..., **connection)` is an
explicit separate recovery entry. It requires matching retained bus/motor
identity, original mode `2`/TIMEOUT `0`, a 2000-tick watchdog observation and only
zero-gain/nearest-zero-feedforward commands. Current mode `1`/TIMEOUT `2000`,
selected status `13` and disabled peers must still match. It sends one clear,
checks fresh disabled state, disables and restores RAM; it sends no MIT or
enable. A mismatch remains a scoped failure. The control manual's section 4.8
and the pinned [manufacturer clear-error command](https://github.com/enactic/openarm_can/blob/f340d4b808fb177e1f297af54eb55fd51c6c7c10/setup/cli/commands/clear_error_commands.cpp)
define this packet; a fault code itself is never relabeled normal disabled.

Motion reports distinguish observed requested-direction displacement from exact
arrival. The maintained first-motion case requires outward displacement greater
than two encoder bins, and retains start, peak, outbound end, return end and
per-leg target errors. Ordinary shortfall is not a motor fault and never causes
automatic gain escalation. A motionless run can pass communication/cleanup while
failing the live motion-evidence predicate.

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

### Passive hardware observation

A driver that only reads hardware can declare these optional boolean facts:

| Driver attribute | Meaning when `False` | Default when absent |
|---|---|---|
| `actuation_supported` | No motion or hold implementation; action support facts are withheld, and supervised control construction is refused | Existing action support |
| `estop_supported` | No physical e-stop implementation | Existing e-stop support |
| `close_releases_torque` | Closing the handle cannot release motor holding torque | Every non-`"sim"` driver requires the parking/support wait |

Declare these facts on the driver before constructing `Arm`; non-boolean values
are refused. A passive hardware observer keeps its real `kind`, uses monitor
posture, and ordinarily sets all three to `False`. Unsupported `write`, `hold`,
`estop`, and `re_enable` methods must explicitly refuse; a software latch alone
must not report that physical motors stopped. Its `close()` releases resources
without enabling, disabling or commanding motors.

Monitor posture still controls registration only: it does not by itself prove
that hardware access or closing is passive. The whole-session e-stop verb is
omitted if any part explicitly lacks that implementation; the SDK never claims
an all-parts stop from a subset. `SiteSession.hold()` and `estop()` report
`unsupported` when their implementation is absent. Named observations remain
available, including original per-part faults. Support facts describe device
implementation separately from grants; legacy monitor drivers that omit these
attributes retain their existing local-policy action behavior.

Test ordinary context exit, partial startup cleanup, unavailable control calls,
and a fresh open after close. Assert actual outgoing device messages so that
read-only status is demonstrated throughout the lifecycle.

## Declaration and envelope must agree

One facts source should produce the driver width/order, action declaration, `Arm`
joint limits, step caps, rate, and home values. Assert that agreement in tests. Every
number needs provenance a wheel holder can inspect: a vendor manual, shipped model,
or explicit unit measurement.

`Arm` is the one route to an SDK-managed driver. It validates a command against the
owner's declaration and refuses whole. The adapter should not add a second command
path around it.

### Position tracking allowance

`Arm.step_caps` historically bounds each target's displacement from the latest
measured position. It does not compare consecutive commands or measure physical
velocity. A servo can therefore exhaust a speed/rate-derived allowance while
following a smooth reference with ordinary tracking lag.

Adapters may pass keyword-only `position_error_caps`: one finite positive number
per declared action row, in that row's units. These replace `step_caps` for
target admission; omission preserves the existing bound and refusal behavior.
Joint limits, workspace/collision checks, Hold and e-stop are unchanged. A refused
command's `RuntimeFault.context.max_position_error` records the actual enforced
vector alongside the legacy `step_caps`, target and measured positions.

An open `SiteSession.describe()` publishes `command_limits[part]` with
`joint_names` and `max_position_error` in matching order. The support matrix
advertises `limits.position_error` for these opened parts. Consume the reported
limits rather than reconstructing them from the action-space rate and velocity.
These are local runtime metadata; they add no protocol field or separate writer.

YAM `arm()` and `bimanual()` accept `max_joint_position_error_rad`, also available
under `parts.<part>.options` in `site.yaml`. It applies the explicit bound to the
six arm joints and retains the existing gripper bound. Pass one positive scalar
for all six joints, or six positive values in declared joint order for separate
allowances. For example, `[0.04, 0.2, 0.04, 0.04, 0.04, 0.04]` permits a larger
joint 2 position-reference lead without changing the other five joints. Invalid
values fail during factory construction, before CAN or vendor drivers open.
The YAM factories default to 0.2 rad on all six arm joints at 25 Hz; explicitly
passing `None` retains the legacy `max_joint_speed_rad_s / rate_hz` arm bound.
Declared velocity and simulator speed are separate from this allowance.

Select this allowance as part of the site's reviewed envelope. It is not a
velocity, acceleration, force or torque limit: a larger error can request more
effort from the vendor's position controller. Callers remain responsible for
reference timing, interpolation and convergence; the SDK does not add a motion
controller or prove physical arrival.

## Optional extensions

### Position and known velocity

A driver may additionally implement
`write_position_velocity(target, velocity_feedforward_rad_s) -> bool`. The velocity
is a trajectory producer's known hint for the same already-admitted position target.
Return false only after deliberately issuing the identical position-only fallback.
Never differentiate measurements or an IK stream to invent feedforward.
The YAM LINEAR_4310 reference adapter smooths each accepted position-only jaw
target over one 40 ms host period at motor rate, with a 1.0 normalized action/s
slew ceiling for large direct goals. It leaves the vendor's contact/force
correction immediate and sends zero jaw velocity feedforward. This changes the
motor reference after admission; it does not claim fingertip-force control.
Positive YAM force mode makes a bounded correction for steady motor-effort
shortfall only after a stationary hold has been established. The correction
resets when motion resumes and never raises its nominal command above the
driver's declared force-request ceiling. Motor effort remains an estimate of
fingertip contact force.

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

## YAM gravity compensation

The YAM factories accept `gravity_comp_factor`, an absolute six-joint vector in
arm joint order. Its default is `[1.0, 1.1, 1.2, 1.3, 1.0, 1.0]`; the third and
fourth factors are rounded two-arm bench calibration, not vendor ratings or a
universal calibration. Site owners can override this under `parts.<part>.options`
for each arm. Values must be finite and positive; invalid vectors fail before
hardware opens. The declaration freezes the vector, and the live driver passes
it to I2RT before control threads start. The vendor appends its unchanged gripper
factor of 1.0. PD gains, friction compensation, and motion limits are independent.
Reopen the hardware to apply a configuration change. The kinematic YAM simulator
ignores these factors.

## YAM gain configuration

The YAM factories and `LiveDriver` accept optional
`arm_gains={"kp": [...], "kd": [...]}`. Each vector names joints 1–6 in order and
contains six finite positive numbers. In `site.yaml`, place this mapping under
`parts.<part>.options`. Values are copied at declaration, so mutating the input
later does not change what opens. Explicit arm gains cannot be combined with
`arm_gain_scale != 1`. The independent `gripper_gain_scale` still affects only the
hand. The YAM factories default to arm KP `[80, 150, 180, 10, 10, 10]` and KD
`[5, 5, 5, 1.5, 1.5, 1.5]`, preserving the vendor hand gains. An explicit
`arm_gains=None` selects vendor gains. The low-level `LiveDriver` still defaults
to vendor gains when used directly. An explicit `arm_gain_scale` without explicit
arm gains retains its vendor-relative behavior.

The factories also default their owner joint envelope to the published YAM
joint range plus 0.010 rad on each side of the six arm joints; the gripper
range stays 0–1. The exact vendor model facts remain in `JOINT_LIMITS`.
Pass explicit `joint_limits=JOINT_LIMITS` to keep those tighter values. The
factory reports each widened row when constructing a rig. Site authors still
need to review mechanical stops and their planner model before moving hardware.

The pinned DM4340/DM4310 MIT encoding supports KP up to 500 and KD up to 5.
Validate both explicit gains and effective scaled gains before CAN startup, and
reject requests outside these bounds instead of allowing silent vendor clamping.
Default KD on joints 1–3 is already 5, so increasing the legacy combined
`arm_gain_scale` is refused. Use explicit P/D values for reviewed independent
calibration. Encoding limits are not tuning recommendations; retain the site's
motion envelope and validate measured tracking, arrival and recovery.

I2RT opens with its original gains holding the measured initial position; the SDK
then applies valid configured gains through `update_kp_kd`. Reopen to change a site
configuration. Hold and e-stop recovery retain the configured requested vectors.
The vendor quantizes gains to 12 bits. Benchmark evidence records requested
`kp`/`kd` and predicted `encoded_kp`/`encoded_kd` separately; the latter are codec
calculations, not motor gain readback. Simulation and monitor/zero-gravity modes
validate the options but do not apply PD gains.
