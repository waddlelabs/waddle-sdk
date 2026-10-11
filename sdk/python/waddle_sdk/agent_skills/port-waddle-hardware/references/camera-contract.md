# Camera adapter contract

Optional `CameraFrame.content_timing` / `CameraSample.content_timing` uses
`CameraContentTiming` with separate RGB/depth intervals on the local host's
monotonic-nanosecond clock and an explicit `clock_revision`. Declare
`content_timing_kind` (`sensor_exposure` or `simulated_state`) to advertise
`camera.content_timing`. Every bound must include readout and mapping uncertainty;
buffered-read or delivery timestamps alone are insufficient. Missing timing stays
unknown. Preserve the ordinary paired session/Unix stamps independently; these
optional content clocks are local-only. Built-in physical adapters do not yet
provide verified content-clock mappings.

## Factory and capture

A camera manifest entry names `module:factory`. Prefer a factory accepting `CameraConfig`. The SDK calls it only while the full site enters or an explicit `inspect_cameras()` context enters. Module import, discovery, `CameraInspectionSpec` construction, and the `inspect_cameras()` call itself must remain non-opening, and vendor imports must stay lazy.

A structural camera driver provides:

- `capture() -> CameraFrame`
- idempotent `close()` that unblocks any pending capture

`capture()` may block for a frame. Return contiguous RGB `uint8` shaped `(height, width, 3)`. Optional raw depth is `uint16` shaped `(height, width)` and pixel-aligned to RGB. Do not reuse mutable vendor buffers after returning; `CameraFrame` freezes/copies as required.

The declared width, height, rate, encoding, frame ID, and mount must match the active stream. A wrist mount names one existing robot part; a scene mount names none. Do not infer transforms from discovery proximity.

Record the pinned vendor stream-profile source through the scaffold's required `--facts-source`. The generated `FACTS_SOURCE` preserves the supplied text but does not establish that it is authoritative.

## Calibration and depth

`CameraCalibrationDriver.intrinsics()` is optional for a live driver that can report the active aligned color-grid intrinsics. Explicit site configuration wins. A frame may carry a local vendor point resolver for distorted aligned depth. Without one, generic deprojection must refuse non-zero distortion rather than treating it as rectified.

Raw metric depth stays in the customer process. A deterministic colorized preview may be published separately, but it does not replace the raw paired sample or declare persistent depth support. Do not infer durable support from a transient RGB-D frame.

## Shutdown and failures

Test a capture blocked in vendor I/O while another thread calls `close()`. Close must signal the device, unblock capture, join deterministically through the SDK pump, and remain safe when called twice. A half-open camera sequence must close every previously opened camera when a later device fails.

Use `CameraInspectionSpec.from_candidate()` only when discovery identified an exact camera driver. Entering `inspect_cameras()` opens cameras but no robot parts, transport, control path, media publication, or recording. Its latest-only frames are appropriate for local camera identification, not rollout data capture.

macOS IORegistry discovery reports raw RealSense USB evidence without an executable
camera selector. Only non-streaming vendor enumeration can resolve that selector;
neither USB presence nor successful native-binding import proves stream access or
aligned metric RGB-D. Preserve the device's exact access/capture errors.

Native source qualification must retain the original vendor identity and each
reviewed patch. The RealSense 2.58.4 macOS recipe additionally fixes cleanup
transfer under libc++ before software-device alignment checks; a moved-from
cleanup callback cannot remain a second owner. Verify actual wheel architecture,
deployment floor, shared USB linkage and source/notices. Compare exact aligned
RGB/depth bytes and concurrent Python progress in separate reference/candidate
processes. Those checks establish neither physical camera access nor control
timing under representative load. Never modify an installed vendor in place or
hide a loader/access failure behind an RGB-only fallback.

Never put motion, robot authority, or owner-envelope logic in a camera adapter.

A camera rendered by a shared simulation world names `world` instead of `driver`. The
world's `camera(config=...)` facet opens it after the shared scene. It returns the same
`CameraFrame` and optional calibration facet as a physical camera. Read the
[simulation contract](simulation-contract.md) for world timing and cleanup.

`waddle_sdk.cameras.metadata` normalizes effective runtime declarations and parses intrinsics, retaining the declared distortion model. Missing optional calibration disables only dependent camera behavior. RGB-only intrinsics need no depth scale; metric resolution explicitly requires one and shares validation with `CameraSample.point_at`.
