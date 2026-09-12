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

The native `Session::media_tracks()` owns these facts. The Python shim only
serializes them; binding API version 4 requires rebuilding both the base and media
extensions together. Runtime adapters without `MediaRuntimePort` remain valid:
applications should disable only the media-dependent feature.
