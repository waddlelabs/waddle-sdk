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

## First customer capture and startup follow-up

Hotfix v1 applied successfully on omen-2 with pyorbbecsdk2 2.1.3. The camera-only
probe timed out on its first one-second wait, before calibration. No full connector
restart or motion occurred. This does not verify depth-scale publication on hardware.

V2 permits ten one-second startup waits for a complete aligned pair, retaining
the one-wait normal capture limit. Only absent/incomplete frames retry; vendor
exceptions and invalid calibration still fail. Explicit as_frame_set follows the
vendor alignment API. 67 focused cases pass, with the unchanged version-fixture
exclusion/media warning above. V1-to-V2 installation, repeat apply and rollback
to original bytes pass. Customer hardware recheck remains pending.

## Customer recovery diff review

Customer reports v2 succeeds on freshly rebooted Gemini305 (scale0.1), but
SIGKILL/SIGHUP without Pipeline.stop wedges subsequent acquisition. Their installed
reboot-and-retry patch passed three induced-fault camera probes; these are customer
results, not a test of this refined source.

Incorporated one startup-only retry on a typed frame timeout. Stop must succeed
before reboot. Only the configured serial is reset; bounded enumeration requires
disappearance and return, and enumeration errors are not treated as absence.
Reboot/retry failures remain inside cleanup.74 focused cases pass (same baseline
version-fixture exclusion). This source is not installed on the customer. Existing
customer site must select serial to enable recovery; an empty connection cannot
authorize a reboot.

Shutdown audit: RigSession.close currently calls hold_until_parked before stopping
camera pumps in _finish_resources. Thus waiting for supported can retain camera
pipelines. No shutdown reorder or signal change is shipped here; it needs separate
shutdown/hold tests. SIGKILL cannot execute Python cleanup.
