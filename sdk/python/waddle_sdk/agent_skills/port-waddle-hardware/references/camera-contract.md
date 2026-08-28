# Camera adapter contract

## Factory and capture

A camera manifest entry names `module:factory`. Prefer a factory accepting `CameraConfig`. The SDK calls it only while the site opens; module import must remain non-opening and vendor imports must stay lazy.

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

Never put motion, robot authority, or owner-envelope logic in a camera adapter.
