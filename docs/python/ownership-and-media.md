# Site ownership and publisher evidence

`Site.open()` creates an unopened context. Entering it acquires an exclusive
kernel file lock for the declared site ID before calling adapter factories or
opening devices. All applications using this entry point participate in the same
per-user namespace, including copies of a site manifest in different directories.
POSIX uses `flock`; Windows uses a nonblocking byte-range lock. Lock files remain
in place after release to avoid competing inodes. This change requires draining
older applications before upgrading: earlier SDK versions do not take this lock.

Catch `waddle_sdk.ownership.SiteOwnershipError` and inspect its `code`:

- `site_owned`: another context or process retains this site ID.
- `site_close_unknown`: device, world or session teardown was not confirmed.
- `ownership_directory` / `ownership_file`: the lock location is unsuitable.
- `ownership_unsupported`: the platform has no supported locking implementation.

Confirmed teardown releases the lock. An uncertain teardown retains the session
and lock for the process lifetime, even if application code loses its reference.
Repeated context exit does not turn that uncertainty into success. Process exit
releases OS resources; it does not establish that a physical robot is safe.
A failed configuration before hardware opens can release ownership normally.

This is cooperative site ownership for one OS user, not hardware identity discovery
or command authority. Different site IDs, different users, direct vendor access
and lower-level driver construction do not share this guarantee. Keep one stable
site ID for one physical installation. Motion authority remains native-core owned.
Third-party factories must clean up partial resources they never return and report
uncertain cleanup; the SDK cannot verify hidden handles inside a vendor factory.

## Optional media contract

`SiteSession.media_tracks()` implements the optional structural
`waddle_sdk.runtime.MediaRuntimePort`. It returns native publisher snapshots:

```python
with site.open(console=False, media=selected_media) as session:
    for track in session.media_tracks():
        print(track["camera_id"], track["stream"], track["track_name"], track["status"])
```

Each row also includes `frames_dropped`, the local uplink drop count. Rows contain
no pixels, transport addresses, credentials or viewer grants. Sessions without a
media plane return an empty list. RGB entries start pending; depth entries appear
only after the native publisher receives a depth preview. Depth preview is
colorized video, not metric depth. Closing the SiteSession makes this method refuse
with the ordinary not-open runtime fault.

`status` describes the last local publication attempt: `pending`, `published`,
`publish_failed`, `encode_failed`, or `push_failed`. A successful subsequent attempt
clears an earlier failure for that stream. Other streams remain independent.
`published` means the transport accepted a frame locally; it does not confirm
remote reception or current connectivity. A quiet stream retains its last attempt
status across missing samples and transport reconnects. Consumers must not infer
track identity or publication from camera observations.

The native publisher owns transport outcomes. Ordinary sessions serialize their
local native track facts; isolated sessions read bounded status from the media-only
child. Binding API version 8 requires rebuilding both base and media extensions
together. Runtime adapters without `MediaRuntimePort` remain valid:
applications should disable only the media-dependent feature.


## Bounded previews

`LiveKit` optionally accepts `preview_width` (maximum output width, aspect ratio
preserved without upscaling), `preview_fps` (finite positive rate ceiling), and
`preview_max_kbps` (positive per-track encoder bitrate ceiling in kilobits/s).
`depth_preview=False` suppresses the colorized depth sibling and its capture-side
colorization. It does not disable acquisition or change exact local RGB-D samples.
`demand_driven=True` enables publisher dynacast and skips resize/color conversion
while the room has no remote participants or all sender layers are paused by
the SFU. The empty-room gate is independent of SFU pause timing and is cleared
on disconnect/worker exit. It also starts signaling on the
native media worker and retries failed initial connections without delaying
session startup. Unavailable video drops publication attempts; it does not stop
control, local capture or agent stills. `video_only=True` excludes media data topics and their teleoperation intake;
video workers read shutdown state without locking the controller snapshot.
Native media metadata lookup releases the Python interpreter lock. Frame
admission stays short and bounded without an extra per-frame interpreter handoff. The default settings preserve
ordinary full-size publication. For an occasional RGB monitor:

```python
from waddle_sdk import LiveKit

media = LiveKit(
    url=authorized_url, token=authorized_token,
    preview_width=320, preview_fps=1.0, preview_max_kbps=128,
    depth_preview=False, demand_driven=True, video_only=True,
    isolated=True, worker_cpu_ids=(7,),
)
```

The frame-timeline rate ceiling applies before media enqueue/encoding. There is
one waiting frame per camera; newer samples replace older samples during congestion.
Video publication and agent stills use separate native workers, with lower Linux
scheduling priority. Source capture, full-size agent stills and gate commands retain
their own paths. Previews capped at 2 fps or less poll idle video queues at 10 Hz,
adding at most 100 ms of presentation/shutdown polling latency. LiveKit publication waits have a deadline. Custom media transports
must still bound synchronous calls for orderly shutdown.

These limits bound optional work; shared CPU, memory bandwidth and network still
have a cost. Encoder bitrate excludes signaling and transport overhead, and multiplies
by the number of camera tracks. Evaluate control timing on the configured host under
viewing, recording and congestion before qualifying a deployment.


## Isolated RGB previews

`isolated=True` selects a Linux media-only child. It requires RGB-only,
video-only publication and an explicit fps ceiling. The child receives camera
names/dimensions, raw RGB slots and the media grant; it receives no site,
hardware adapter, control callback, session, lease or recording authority.
Owner capture and source RGB-D remain unchanged. Each camera has one anonymous
shared-memory slot, with a 32 MiB aggregate source-slot limit. Frame handoff
copies at most once per presentation interval, takes only nonblocking locks,
and drops a preview frame while the child reads. It never waits for encoding,
startup, signaling or a network acknowledgement. Credentials use a private
inherited socket rather than argv, environment or persistent files.

`worker_cpu_ids` places the child before importing numerical/native libraries
and creating WebRTC threads. Reserve a disjoint CPU set for the owner in the
caller or service configuration; affinity alone does not reserve CPUs against
unrelated processes. The child uses Linux `SCHED_IDLE` before library/thread initialization, so normal
robot work takes precedence; one numerical-library thread avoids unnecessary
parallelism. Leaving `worker_cpu_ids` unset preserves the owner's full CPU set. `worker_memory_mb` defaults to a 2048 MiB process address-space ceiling,
including loaded libraries, not just buffers. Resource failure disables optional
video and retains its scoped error through media metadata. Native controller cleanup
precedes bounded child termination; optional teardown never replaces an original
SDK fault. Unsupported platforms retain ordinary SDK control and report optional
isolated media as unavailable. Legacy non-isolated transports remain supported.

This separates interpreters and codec scheduling, but uses real host memory,
copy bandwidth and uplink capacity. Qualify the chosen resource placement with
same-owner timing trials, recording, intended processing load and actual viewers.
