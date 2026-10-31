# Device relay

The optional `waddle-sdk[relay]` extra provides `waddle_sdk_relay.device.Device`
for applications that run their policy on another computer. The SDK keeps the
Site, Run, hardware ownership, native command gate, finite playback and immediate
supervision on the robot computer. The relay does not plan motion, implement
account login or open a network listener.

The relay currently targets POSIX hosts. Its confined workspace uses directory
file descriptors, `flock` and no-follow opens; atomic moves to an absent destination
use Linux `renameat2`. Windows and non-Linux atomic moves are not qualified by this
implementation. Ordinary SDK imports do not load these modules or require msgpack.

## Application integration

```python
from pathlib import Path

from waddle_sdk import load_site
from waddle_sdk_relay.device import Device

root = Path("my-rig").resolve()
device = Device(load_site(root / "site.yaml"), root)

# In your application's async connection handler, after authenticating the peer:
# await device.serve(authenticated_socket)
```

Construction and inventory access open no hardware. The caller supplies a socket
with async iteration over binary messages, `send(bytes)` and
`close(code=..., reason=...)`. The application must authenticate and authorize the
peer and use TLS before calling `serve`; the relay itself supplies no identity
or access-control service. Do not expose it as an unauthenticated listener.

The wheel ships `waddle_sdk_relay` as a separate Python namespace so a server
can import its protocol constants, publisher schema and structured errors without
loading `waddle_sdk`, native bindings or hardware modules. Device and wire imports
load the actual SDK runtime only when needed. It is part of the same SDK wheel,
not a separately installed application or transport service.

## Saved connection settings

`waddle_sdk_relay.connection` provides shared settings without importing the SDK
runtime. `ConnectionConfig(endpoint, key, allow_loopback_ws=False, rig=None)`
validates a credential-free WSS URL, a bounded visible-ASCII token and an optional
rig name. Explicit numeric loopback addresses may use WS for local development.
The token is excluded from the object's representation.

`read_key(path)` reads a bounded private regular file owned by the current OS user,
refusing symlinks and group/world permissions. `load_connection(path, root=...)`
reads version-1 saved connection JSON without opening hardware or reading its
referenced key. It retains the endpoint, key-file path, loopback flag, optional rig
and optional launcher arguments; it never executes a launcher. Relative key paths
resolve against the caller's explicit root, or the configuration directory when
omitted. Missing configuration returns `None`; malformed, duplicate, oversized or
insecure configuration raises a structured refusal. Applications choose the file
location and retain responsibility for account authorization.

## Relay contract

`waddle_sdk_relay.protocol.PROTOCOL` is the existing binary device protocol. Encode/decode
binary envelopes with `waddle_sdk_relay.wire`; it accepts an explicit SDK DTO
allowlist, immutable bounded array copies and structured faults, never pickle or
executable values. A request contains exactly `id`, `method` and `arguments`.
IDs are increasing integers from 1 through `2**53 - 1`; replies retain the ID and
either `result` or the original structured `error`. Workspace errors additionally
carry `error_kind: workspace`.

The peer explicitly opens the Site, begins/ends a Run and submits observations,
commands or finite playback through their ordinary SDK methods. `playback_start`
returns an execution receipt; `playback_status` and `playback_cancel` require its
original run and execution IDs. Capability facts still govern admission. A
transport acknowledgement is not proof of arrival or physical stopping. See
[finite playback](../core/finite-reference-playback.md) for measured outcomes and
completed cleanup receipts.

Matching binding API 18 also accepts `playback_update` with the retained `run_id`,
execution `id` and a `PlaybackCurveUpdate`. It returns `request_id` and `snapshot`
after queueing, without waiting on device I/O under the admission lock. Poll the
same execution for acceptance and acknowledged revision. This operation requires
the opt-in revisable policy; neither a lost reply nor an old handle may start or
replay execution. Ordinary policies omit the new false flag on the wire so older
receivers can still decode them.

Binding API 20 adds `playback_withdraw` with the original `run_id`, execution
`id`, accepted update `request_id` and `expected_revision`. It returns a new
request ID and snapshot. Require `execution.withdrawable_joint_playback` and
reconcile the retained withdrawal receipt; queueing does not prove removal or
stopping, and a lost reply never permits replay. A terminal or wrong-run handle
cannot withdraw work from a replacement execution.

Binding API 21 carries opt-in `PlaybackPolicy.credit_pacing_delay` with the
`execution.paced_playback_deadline` fact. Read accumulated credit from native
`curve_progress.pacing_extension_ns`; remote reply latency never contributes.
The false default is omitted on the wire for older receivers, which must refuse
an enabled policy they cannot decode.

Sixteen ordinary calls and two reserved Hold/e-stop calls are admitted separately.
Safety calls bypass ordinary admission and use the existing native SDK stop path.
Before opening, an idle connection expires after 30 seconds; after a request or
open, the default idle watchdog is three seconds. The peer must send heartbeats
(`ping`) while idle. Disconnect fences further calls, posts Hold and drains the
original Run/Site. Unconfirmed closure retains ownership and requires local
operator review; it never opens a replacement owner or retries uncertain work.

Only explicit local operator authorization may set
`torque_release_authorized=True`. A remote close argument cannot grant that
authority. An optional asyncio `stop` event sends a normal-stop notice to the peer;
it does not implicitly authorize torque release.

## Workspace and camera data

`waddle_sdk.workspace.files.Workspace` supplies confined, revisioned local file
operations. Reads and mutations reject symlinks, hardlinks, traversal and stale
revisions; replacements are atomic and synchronized. The relay exposes only
stat/list/read/write/mkdir/move/delete/apply-patch operations, with existing size
and concurrency bounds. It cannot mutate the root.

Transfer inventory and direct relay paths exclude every dot-prefixed component,
`recordings` and `__pycache__`. Keep credentials in excluded paths or outside the
workspace; do not treat a public filename as a secret. Directory listings can
contain excluded names, but those paths cannot be read or changed by the relay.
Inventory is bounded to 10,000 regular files and 512 MiB; individual file operations
have their own smaller bounds. These robot-local files do not define a hosted
application's durable persistence policy.

Camera copies retain exact source pixels, acquisition sequence and paired
session/Unix timing; host-monotonic timestamps do not cross hosts. Retained camera
evidence is bounded separately from control observations. A publisher grant must
come from the authenticated application; the shared validator checks its shape,
credential-free TLS URL and bounded fields. Explicit loopback development URLs
are the only plaintext exception. The matching media extra is still required for
actual LiveKit publication.

The software tests exercise native mock hardware, lifecycle and transport limits.
They do not establish physical stopping distances, WAN timing or task success.
