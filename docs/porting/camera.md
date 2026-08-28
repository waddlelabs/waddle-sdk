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

Unlike a part factory, the camera factory itself is retained as the opening callback
and is invoked only while the site enters. Importing its module must still be safe
without the vendor SDK or device; import optional vendor packages inside the factory
or driver constructor.

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
reports only stable declared/implemented facts; transient frame content does not
become a capability.

## Mounts are topology, not transforms

`CameraConfig.mount` is either a fixed scene camera or a wrist camera naming one
declared part. It describes mobility only. Calibration owns the transform; the SDK
does not infer one because a camera and robot were discovered on the same machine.
