# Orbbec startup metadata correction

Customer startup rejected camera3 depth with `timeline.number` on depth_scale_mm.
The API29 adapter captured a scale but had no CameraCalibrationDriver.intrinsics
interface, so the rig declaration and Metal recording exported no scale. Discovery
intentionally omitted serial because the adapter did not support selection.

Acquire an aligned frame during passive camera initialization, expose its measured
scale with active color-profile calibration, and retain native depth bytes. Use the
vendor serial selector when configured; no fallback to another camera. Reject
invalid or changed units. Existing empty-connection sites remain supported.

64 focused camera/discovery/native-rig cases pass. Old source fails the new
rig-calibration regression. One unrelated installed-version/pyproject fixture
fails on old and new source; the test environment also has incompatible optional
media API5, so these checks use the matching bundled API29 core and do not qualify
LiveKit. Fake-vendor tests do not qualify actual Gemini305 capture.

The Python-only support patch checks exact old/new source hashes, preserves originals,
refuses unrelated installed edits, and supports rollback. It does not change native
SDK/media, Metal, dependency versions, hardware configuration, or running services.
Customer camera-only capture is required before normal full connector startup.
Do not invent a depth scale or rewrite historical failed evidence.

Vendor interfaces: https://github.com/orbbec/pyorbbecsdk/blob/v2-main/stubs/pyorbbecsdk.pyi
