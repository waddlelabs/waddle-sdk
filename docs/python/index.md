# Python SDK

The Python frontend owns one configured site. Its public lifecycle is deliberately
small:

```python
import waddle_sdk

site = waddle_sdk.load_site("site.yaml")
with site.open() as session:
    with session.run(task={"id": "inspect"}, actor={"id": "policy"}) as run:
        observation = run.observe()
        result = run.step(action, observation)
        if not result.dispatched:
            if result.fault is not None:
                raise result.fault
            run.hold(result.detail or "command withheld")
            run.finish(waddle_sdk.Outcome.ABORT.value, "command withheld")
        else:
            run.finish(waddle_sdk.Outcome.SUCCESS.value)
```

`load_site()` validates and resolves confined paths but opens no device. `site.open()`
returns an unopened context; entering it opens passive cameras before supervised robot
drivers, then constructs pumps, recording, and one native session. Context exit
finalizes recording before closing hardware.

The reference YAM adapter also installs the exact-signature workaround required by
the pinned I2RT receive loop before opening SocketCAN. The workaround makes one
kernel wait for the receive budget and one final non-blocking drain; it refuses an
unverified vendor signature rather than silently running with starvation-created
motor timeouts. This is adapter-local and does not alter custom robot drivers.

The package root exports only the lifecycle, transport declarations, outcomes, and
manifest errors. Hardware extension contracts live under `waddle_sdk.robots` and
`waddle_sdk.cameras`. The structural integration port for higher layers lives in
`waddle_sdk.runtime`.

## Installation

```console
python -m pip install waddle-sdk
```

Robot and camera vendor packages are lazy optional extras. Importing `waddle_sdk`,
loading a manifest, or importing an adapter module must not open a device. A custom
adapter declares its own vendor dependency in its own package.

## Runtime contract

`SdkRuntimePort` is the shared direct and remote shape: `describe`, `begin_run`,
`observe`, `submit`, `hold`, `estop`, cursor-based `events`, and bounded calibration
measurements. Faults crossing that boundary preserve their original diagnostic
text, device scope, JSON metadata and lower-level cause chain. Applications can
use `RuntimeFault.from_exception` to wrap untyped implementation failures without
discarding those details; existing `RuntimeFault` objects pass through unchanged.
Only explicitly credential-labelled fields/assignments and Bearer values are
redacted. Device paths and benign vendor messages remain diagnostic evidence.
Non-JSON attributes are named as omitted rather than serialized with `repr`;
metadata and cause nesting are bounded. Typed fault producers must exclude secrets.
YAM control-thread and feedback failures originate as `motor_failure` with the
channel and specific failure reason, so consumers can retain their scope.

An owner-envelope refusal keeps `SubmitResult.dispatched=False` and
`gate="owner_refusal"`. Its additive `fault` field carries `safety_refusal` with
the exact admission reason, named part/frame, measured and requested joints,
limits and workspace. `detail` and `part` mirror that origin, and the `run.step`
event retains its serialized fault. Applications should propagate that fault
unchanged. Other gate decisions may have no fault. The low-level
`robots.base.apply_decision` retains its boolean return and offers an optional
`on_refusal` callback for consumers needing these diagnostics. If holding a
refused multipart command fails, all remaining holds are attempted and the
refusal is raised with each separate failure in `context.hold_errors`.

An opened `SiteSession` also implements optional support, kinematics, and conservative
geometry facets. These report implementation facts; they do not widen the action space
or grant permission.
