# Python SDK

Applications with a remote policy can use the optional
[device relay](device-relay.md), keeping hardware ownership and buffered
execution in the SDK on the robot computer.

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
motor timeouts. Transactions also retain matching late replies throughout their
existing 10 ms initial and 9 ms recovery windows, skipping unrelated IDs without
restarting either deadline. Explicit encoder reply IDs, matching motor errors,
retry counts and scoped communication failures are preserved. This is adapter-local
and does not alter custom robot drivers.

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

### Optional private exception evidence

An application can configure `waddle_sdk.diagnostics.configure(directory,
context={...}, secrets=(...))` before opening devices. `RuntimeFault.from_exception`
and relay error conversion then add a `context.diagnostic_id` linking the original
fault to a private JSON incident. `diagnostic_scope(command_id="...")` adds scalar
correlation fields to errors raised within that scope. Applications can explicitly
call `capture_exception(error, context={...})` at other exception boundaries.
The SDK never uploads evidence or changes device ownership, gates or outcomes.
Unconfigured applications create no diagnostic worker or files.

Capture copies the exception type/message, causes/groups and bounded stack
locations. It retains no local variables, frame objects, source text, camera data
or environment dump. Credential-labelled values and application-supplied secrets
are redacted. A low-priority worker adds Python/distribution versions and hashes
of at most eight source files (2 MiB total reading budget), then atomically
publishes mode-0600 files in a private user-owned directory. On Linux only that
worker's thread receives niceness 19. Spawned application workers can inherit the
directory and scalar context with `inherit=True`; do not put credentials in that
context. Optional additional distribution names are application-selected through
`WADDLE_DIAGNOSTICS_DISTRIBUTIONS`.

The producer never waits for disk or network: its queue holds eight incidents,
each published record is at most 48 KiB, and the directory retains at most 64
unacknowledged incidents. Saturation preserves earlier evidence and increments
`store.status()` counters; it does not replace the original fault. Capture IDs
are best-effort references, not acknowledgments of durable storage. Storage
failures, truncation, process death before persistence and hard native crashes
can leave incomplete evidence. `close()` never waits for storage during control
drain. Applications own authenticated retrieval, acknowledgment and health
reporting, and should retire a file only after its consumer confirms the exact ID.

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
