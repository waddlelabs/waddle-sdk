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

## Periodic host scheduling

An adapter's dedicated periodic thread may use
`waddle_sdk.scheduling.periodic_thread(period_s=..., computation_s=...,
constraint_s=...)` around its active loop. Values are seconds and must satisfy
`0 < constraint/2 <= computation <= constraint <= period`. On macOS this requests
Mach time-constraint scheduling for the calling thread, restores its original
policy (including nested requests) on exit and releases the acquired Mach port.
Other platforms retain their current policy. `SchedulingError.as_dict()` preserves
the original operation and kernel return under `runtime.scheduling_failed`.

Declare a budget appropriate to the measured work and keep unrelated planning off
that thread. A request is not a timing guarantee: macOS may demote over-budget
threads, and timers, hardware and Python execution still need measurement under
load. Retain the adapter's existing stale-feedback, missed-deadline, cancellation
and stop gates. Never extend those bounds to compensate for scheduling failures.
The yielded mapping identifies the requested policy; it does not claim measured
cadence, physical arrival or real-time qualification.

`periodic_wait(event, timeout_s, period_s=...)` confines the request to the blocking
wait and restores policy before capture, rendering or customer callbacks. The
standard robot/camera pumps and camera-only inspection use it. Already-stopped
and zero-duration waits return without requesting a policy. Other hosts retain
their ordinary wait behavior; fresh feedback and measured deadlines still govern.

`RobotPump.fault` retains a terminal publishing/wait exception. Its optional
`on_fault(error)` callback receives that exact exception; `Rig.on_pump_fault` can
connect it to an application fault channel. Normal site composition emits the
original fault for every part served by that shared publisher and refuses to
stamp their observations as fresh after it fails. This is a common publisher
failure, distinct from one driver's ordinary part fault, which remains independent
and does not stop siblings. Hold/e-stop remain available.

## USB-CAN transport candidate

`waddle-sdk[can-usb]` provides the optional
`waddle_sdk.robots.usbcan.{UsbCanSelector,UsbCanBus,UsbCanFault}` transport.
This is a software-tested qualification candidate. YAM accepts explicit selection
with the reviewed runtime below; `transport_evidence.qualified` is explicitly false. Installing the
extra imports no bus and does not identify the attached robot.

The selector requires a supported Candlelight VID/PID, exact serial number and
current USB bus/address. A different serial at the declared location is refused
before any control request. Opening claims interface zero and configures only the
selected, single-channel adapter. It never resets USB, detaches a kernel driver,
falls back to an enumeration index or reconfigures another adapter on shutdown.
A usable libusb library is required; `libusb_library` can select its exact path.
Distribution of that native dependency remains part of host qualification.

The provider implements classic CAN only. It validates device bit-timing limits,
uses finite control/read/write timeouts and stamps receives at host acquisition.
It does not claim a hardware clock. `recv(0)` inspects only the bounded queue;
positive submillisecond timeouts never become an infinite USB read. Sending keeps
the original message intact and acknowledges USB submission, not CAN delivery or
motor arrival. A positive send timeout is required because this synchronous USB
provider cannot perform a genuine nonblocking write. Millisecond USB timeout
rounding is explicit: an observed submission overrun is a terminal fault.
`hardware.usb_can_write_timeout` retains the phase, elapsed caller time, original
budget and rounded USB timeout. That duration includes Python/host scheduling;
it does not isolate USB transfer time or establish a hardware timestamp.

USB access/disconnect failures, controller error frames, malformed frames and
device/host overflow fence subsequent reads and writes. `UsbCanFault` retains the
operation, selected device, original exception/cause, errno and backend code;
the ordinary runtime fault wrapper preserves that chain. It deliberately does not
inherit retryable `can.CanError`, so a vendor transaction cannot replace terminal
transport evidence with a final generic motor timeout. Device error reporting is
enabled when its firmware advertises that feature, and absence remains visible
in `transport_evidence.berr_reporting`. This is not yet qualified fault parity.

Shutdown wakes waiting receivers, drains the reader and in-flight write, then
stops/releases the selected interface. An undrained operation retains the device
and reports `hardware.usb_can_drain`; it does not release a live handle. Site locks
and actual USB claim semantics remain relevant: direct vendor access or another
site ID is outside the SDK's cooperative site lock. Multi-owner exclusion must be
qualified before deployment.

[The I2RT injection patch](patches/i2rt-570ef666-transport-injection.patch) applies
to upstream commit `570ef66681ff12bd8298aba34084307cfecc9f05`. It adds optional
keyword-only `get_yam_robot(..., motor_interface_factory=..., control_wait=...)`,
`DMChainCanInterface(..., motor_interface_factory=..., control_wait=...)` and
`DMSingleMotorCanInterface(..., bus_factory=...)` seams. Each factory returns a
fresh owned interface/bus. The default SocketCAN path and original motor/gain/
gripper code remain in the vendor; simulation ignores the injected transport/wait.
Partial interface initialization and notifier failures still drain their bus and
retain the primary exception plus cleanup diagnostics.

The optional `control_wait(stop_event, timeout_s, *, period_s)` callback surrounds
only the blocking delay; `waddle_sdk.scheduling.periodic_wait` supplies the native
macOS policy. The server and CAN writer retain their original computation, motor
recovery and command values. Control loops retain one thread and the original
background exception. Startup waits for a completed first iteration rather than
a fixed sleep; stop wakes cadence waits and bounds drain before transport close.
A failed drain retains the selected handle and reports its original scoped timeout.
Constructor/factory failures clean up started loops and owned interfaces, preserving
the primary fault plus any cleanup uncertainty. Readiness proves software progress,
not measured hardware timing or arrival.

YAM consumes optional retained `control_fault` evidence before a generic stopped-
thread observation, and explicit server/writer drain methods before closing the
transport. The pristine vendor retains its existing compatibility path. The patch's
software cases live in `tests/test_can_transport.py`, `tests/test_control_loop.py`
and `tests/test_control_lifecycle.py`; SDK cases live in `sdk/tests/test_usbcan.py`
and `sdk/tests/test_robots_yam.py`. These selections open no hardware.

Apply and test the patch in a separate checkout of that exact upstream commit.
Do not change an installed vendor tree or its `RECORD`/`direct_url.json` to hide
source differences. The [reviewed derived runtime](#reviewed-i2rt-runtime-artifact)
retains unchanged source geometry and supports non-opening model inspection.
Physical cadence under load, stop/fault behavior and multi-owner checks remain
required before deployment. Explicit selection does not establish that acceptance.
The patch is a shared transport seam, not a separate macOS controller fork.

### Reviewed I2RT runtime artifact

`tools/build_i2rt_transport_runtime.py` builds
`i2rt==1.1.2+waddle.transport.570ef666` from an immutable archive of the upstream
pin plus the reviewed SDK patch. It leaves the input checkout and installed
packages untouched and refuses an existing output directory. With `uv==0.11.26`
available, run from the SDK repository root:

```bash
python tools/build_i2rt_transport_runtime.py \
  --source-repo /path/to/i2rt \
  --output tmp/i2rt-transport-build
```

The source repository must contain the exact upstream commit. The tool creates
the visibly versioned wheel, retains the MIT license and source provenance,
checks every package file and selected metadata/license byte against the SDK's
reviewed manifest, and writes `build-evidence.json` with the actual wheel digest.
It fixes archive timestamps to the upstream commit time. The wheel build backend
is pinned to Flit Core 4.1.0. Five production files contain the transport/lifecycle changes;
the other 112 production files, including model assets, retain their upstream
bytes. The extra provenance file declares these differences without claiming a
pristine Git installation or physical qualification.

Linux retains the vendor NumPy 2.2.6 requirement. Darwin requires NumPy 2.3.1 or
later to include the upstream native SME/FPSR corrections. The unchanged vendor
Ruckig 0.15.3 build uses an older option; with the current build backend, install
the candidate using its documented compatibility setting:

```bash
uv pip install --config-settings-package ruckig:minimum-version=0.9 \
  tmp/i2rt-transport-build/wheels/i2rt-1.1.2+waddle.transport.570ef666-py3-none-any.whl
```

The YAM model-source provider accepts this exact reviewed distribution as well as
the original Git pin. It verifies the candidate's version, complete source/asset
inventory, SDK-approved hashes, installed package records, metadata and license.
A rewritten RECORD, self-declared provenance, extra vendor source or changed
model cannot establish acceptance. The returned geometry remains identical to
the pristine source. Provenance retains the upstream base pin and the actual
distribution's license-record path; inspection imports no vendor Python code.
These checks establish source identity, not a sandbox against a malicious user
who can change the interpreter or installed SDK. This artifact does not select
USB-CAN, open a device or qualify physical control behavior.

### YAM USB selection

The shared YAM adapter accepts `arm(usb_can=...)` and `ArmSite(usb_can=...)`
for explicitly selected live parts. `waddle_sdk.robots.usbcan_config` supplies
immutable `UsbCanSelector` and `UsbCanConfig` without importing optional CAN/USB
or vendor code. The existing selector import from `robots.usbcan` remains valid.
An ordinary Site manifest can supply the equivalent configuration under its
YAM part's `connection`:

```yaml
connection:
  usb_can:
    selector:
      vendor_id: 7504       # 0x1d50; use the discovered supported VID/PID
      product_id: 24687     # 0x606f
      bus: 7               # replace with this adapter's current USB location
      address: 3
      serial_number: "replace-with-exact-adapter-serial"
    # Optional absolute path to the host's selected libusb library:
    # libusb_library: /absolute/path/libusb-1.0.dylib
```

Omit `channel` and `configure_can`. The current reviewed vendor chain assumes
1,000,000 bit/s; another `can_bitrate` is refused before any USB/motor command.
Selectors sharing either a location or a VID/PID/serial identity are refused
for distinct live parts before the first arm opens. Configuration mappings are
frozen at declaration, so later caller mutation cannot change the selected arm.
Simulation can retain valid live configuration without importing/opening its
transport. Reconnection that changes the bus/address requires updating the site.

Opening a USB-selected arm first verifies the exact derived I2RT source against
the SDK manifest, then imports the optional transport/vendor implementation.
Each injected motor interface owns a fresh selected bus. The existing YAM
receive/transaction handling, gains, gravity compensation, measured gripper
decisions, feedback freshness, envelopes and stop authority remain in place.
The scoped `periodic_wait` callback surrounds only each control thread's wait;
it does not change control frequency or execute the controller remotely.

`LiveDriver.transport_evidence` includes the original bus evidence and exact
reviewed `runtime_source`, with physical qualification false. A startup report
makes that scope visible. There is no discovery-based automatic USB selection
or reduced motion/arrival acceptance. Physical feedback, transport timing under
load, error reporting, multi-owner exclusion and clean measured stopping still
require live qualification before production use.
An original control-thread fault takes precedence over generic stopped-worker
evidence during live startup. Draining a worker and releasing USB are distinct
from confirming a delivered zero-torque command; a transport fault can prevent
that command. Preserve physical stop and the exact uncertainty after such a fault.

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
