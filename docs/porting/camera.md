# Camera adapters

A camera entry names an importable factory:

```yaml
cameras:
  wrist:
    driver: customer_camera.backend:open_camera
    mount: {kind: wrist, part: arm}
    connection: {serial: "CAMERA-01"}
    stream: {width: 1280, height: 720, fps: 30}
```

Prefer a factory with the current typed signature:

```python
from waddle_sdk.cameras import CameraConfig, CameraDriver


def open_camera(*, config: CameraConfig) -> CameraDriver:
    return VendorCamera(
        serial=str(config.connection["serial"]),
        width=int(config.stream["width"]),
        height=int(config.stream["height"]),
        fps=int(config.stream["fps"]),
    )
```

Unlike a part factory, the camera factory itself is retained as an opening callback.
The full site lifecycle invokes it while the site enters. An explicit camera-only
inspection may invoke the same factory without opening any robot part. Importing the
module must still be safe without the vendor SDK or device; import optional vendor
packages inside the factory or driver constructor.

## Identify camera views without opening a site

Hardware discovery reports evidence without constructing a driver or starting a
stream. When a site operator needs to match several physical cameras to their views,
use the separate inspection lifecycle:

```python
from waddle_sdk.cameras import CameraInspectionSpec, inspect_cameras
from waddle_sdk.discovery import discover_hardware

report = discover_hardware()
specs = [
    CameraInspectionSpec.from_candidate(candidate, width=640, height=480, fps=30)
    for candidate in report.candidates
    if candidate.kind == "camera" and candidate.driver is not None
]

with inspect_cameras(specs) as inspection:
    for name in inspection.names:
        frame = inspection.wait(name, timeout_s=5.0)
        if frame is not None:
            print(name, frame.sequence, frame.rgb.shape)
```

Creating `CameraInspectionSpec` objects and calling `inspect_cameras()` open nothing.
Entering the returned context opens only the named cameras. It does not construct a
site session, open robot parts, connect a transport, publish media, or record frames.
The session keeps one latest immutable frame per camera, so a slow viewer cannot build
an unbounded image queue. `wait()` returns `None` after a timeout or capture failure;
the `errors` mapping distinguishes the latter case with an SDK-owned error category.
It never includes arbitrary vendor exception text.

Context exit calls every camera driver's idempotent `close()` before it joins the
capture threads. This is why `close()` must unblock a pending `capture()`. A driver
that violates that contract produces `CameraInspectionError` after the bounded close
timeout instead of hanging shutdown indefinitely.

## Structural `CameraDriver`

Only two methods are required:

- `capture() -> CameraFrame` may block until a sample is ready.
- `close()` is idempotent and must unblock a pending `capture()` so shutdown can join
  the camera pump deterministically.

`CameraFrame.rgb` must be a height × width × 3 RGB `uint8` array. Optional depth is a
height × width pixel-aligned `uint16` array. The SDK freezes arrays so reuse of a
vendor buffer cannot mutate an earlier sample.

One `CameraSample` receives an atomic session-monotonic/Unix timestamp pair after
capture. Raw metric depth remains local. A media-enabled session may publish RGB and
a deterministic colorized depth preview on separate tracks; the adapter does not
encode or route those tracks.

## Intrinsics and deprojection

A driver may structurally implement `CameraCalibrationDriver.intrinsics()` to report
the active aligned color-grid intrinsics after opening. Explicit manifest intrinsics
win. Drivers without this extension are valid.

A `CameraFrame` may carry a local `point_resolver(x, y, depth_m)` for a vendor-correct
distorted-depth projection. The generic pinhole fallback refuses non-zero distortion
rather than producing an optimistic point.

Do not infer persistent depth support from a single RGB-D sample. The support matrix
reports only stable declared or implemented facts; transient frame content does not
become a support fact.

## Mounts are topology, not transforms

`CameraConfig.mount` is either a fixed scene camera or a wrist camera naming one
declared part. It describes mobility only. Calibration owns the transform; the SDK
does not infer one because a camera and robot were discovered on the same machine.
