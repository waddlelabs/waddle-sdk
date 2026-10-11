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

Standard capture pumps and inspection use the SDK's scoped macOS `periodic_wait`
for cadence only. Capture, alignment, rendering and publication run after the prior
host policy is restored. Scheduling failures follow the existing capture fault
boundary; they do not extend freshness or stream deadlines. See
[host scheduling](robot.md#periodic-host-scheduling) for the request and its limits.

Hardware discovery reports evidence without constructing a driver or starting a
stream. Linux uses sysfs; macOS reads IORegistry and serial-device entries. Raw
RealSense USB identities on either platform remain evidence, with no executable
camera selector. Optional non-streaming librealsense enumeration supplies the
exact serial accepted by the selected camera driver. A discovered USB-CAN adapter
does not identify its attached robot or qualify a transport.

When a site operator needs to match several physical cameras to their views,
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

The declared frame rate schedules capture attempts. Slow capture or publication
can reduce the delivered rate: the SDK skips missed frame slots and resumes at a
future slot instead of issuing catch-up bursts. Every acquired frame is still
published through the ordinary sample and recording paths.

### Optional image-content timing

`CameraFrame.content_timing` may carry immutable `CameraContentTiming` from
`waddle_sdk.cameras`. The camera pump preserves it on the exact `CameraSample`.
It contains `kind`, `clock_revision`, and separate `rgb_monotonic_ns` and optional
`depth_monotonic_ns` inclusive interval pairs on this host's `time.monotonic_ns`
clock. Supply depth bounds exactly when a depth plane exists. These local values
do not replace the paired session/Unix stream stamp and are not sent on the wire.

A driver implementing this extension declares `content_timing_kind` as
`sensor_exposure` or `simulated_state`; the opened camera's support row then
advertises `camera.content_timing`. This declares implementation support, not a
guarantee that every frame supplies bounds. Missing timing stays unknown. The pump
rejects a supplied timing kind that contradicts the driver's declaration.

For `sensor_exposure`, bounds must contain every pixel's exposure/readout time,
including sensor-to-host clock uncertainty and RGB/depth alignment latency.
Change `clock_revision` whenever that mapping changes. A buffered read's start,
frame delivery stamp, nominal frame rate or wall-clock timestamp alone cannot
establish these bounds. Drivers with an unverified mapping should omit timing.
Built-in physical camera adapters currently omit it.

The RealSense adapter resolves an explicit `connection.serial` through a
serial-selected vendor configuration, so opening or recovering one RSUSB camera
does not globally enumerate a peer's active streaming interfaces. It retains one
process-lifetime vendor context and each camera's independent bounded recovery.
Omitting the serial selects the first enumerated device; use explicit serials for
multi-camera sites.

For source-built librealsense 2.58.4, the optional
[alignment binding patch](patches/librealsense-2.58.4-align-gil.patch) releases
Python's GIL while native `align.process` runs. Apply it to the upstream source
with `patch -p1 < /path/to/librealsense-2.58.4-align-gil.patch`, then rebuild the
Python binding with the same selected USB backend, interpreter and native build
options. The SDK does not install this patch or modify vendor packages at runtime.
Retain the original binding for a hardware-free software-device pixel-parity
comparison and record concurrent Python progress before selecting the rebuilt
binding. A local 50-frame RGB/depth comparison was byte-identical; it does not
qualify arbitrary builds or establish hardware timing. This native dependency
patch is separate from serial selection in the Python adapter.

### Native macOS source qualification

The reviewed source candidate targets arm64/macOS 15+ and CPython 3.12. It is a
separately identified `pyrealsense2==2.58.4.post2` wheel, based on upstream commit
`34d6c778e1134d8505adcd56bb57acbe7598a459`, with shared libusb 1.0.30. Other
architectures, interpreters and older macOS releases require separate builds and
acceptance. The ordinary extra does not select this candidate automatically.

Apply the [cleanup ownership patch](patches/librealsense-2.58.4-cleanup-ownership.patch)
before software-device checks. Upstream rsutils assumes moving a `std::function`
empties its source, although the [C++ move contract](https://eel.is/c++draft/func.wrap.func.con)
allows an unspecified valid state. Native libc++ retains small callbacks; an
unmodified deferred cleanup can free one frame buffer twice. The patch explicitly
empties transferred callbacks in detach and both moves, preserving reset,
self-move and no-throw move behavior. It changes ownership semantics without
disabling cleanup or altering pixels. The separate alignment patch then permits
Python threads to progress during native alignment.

Build in new private scratch, with Xcode Command Line Tools and uv available.
Run these commands from the SDK repository root. They install nothing globally
and open no physical camera or robot. The upstream USB tests may enumerate USB.

```sh
set -eu
SDK_SOURCE="$PWD"
NATIVE_ROOT="$PWD/tmp/realsense-native"
mkdir "$NATIVE_ROOT"
uv venv --python 3.12 "$NATIVE_ROOT/env"
uv pip install --python "$NATIVE_ROOT/env/bin/python" \
  cmake==3.31.10 ninja==1.13.0 wheel==0.46.3 delocate==0.13.0 numpy==2.2.6
export PATH="$NATIVE_ROOT/env/bin:$PATH"

curl -fL --max-time 60 \
  https://github.com/libusb/libusb/releases/download/v1.0.30/libusb-1.0.30.tar.bz2 \
  -o "$NATIVE_ROOT/libusb-1.0.30.tar.bz2"
printf '%s  %s\n' \
  fea36f34f9156400209595e300840767ab1a385ede1dc7ee893015aea9c6dbaf \
  "$NATIVE_ROOT/libusb-1.0.30.tar.bz2" | shasum -a 256 -c -
tar -xjf "$NATIVE_ROOT/libusb-1.0.30.tar.bz2" -C "$NATIVE_ROOT"
(
  cd "$NATIVE_ROOT/libusb-1.0.30"
  ac_cv_func_pipe2=no CFLAGS='-O2 -mmacosx-version-min=15.0' \
    LDFLAGS='-mmacosx-version-min=15.0' \
    ./configure --prefix="$NATIVE_ROOT/usb" \
      --enable-shared --enable-static --enable-tests-build
  make -j4
  make check
  make install
)

git clone --filter=blob:none --no-checkout \
  https://github.com/realsenseai/librealsense.git "$NATIVE_ROOT/source"
git -C "$NATIVE_ROOT/source" checkout -b macos-reviewed \
  34d6c778e1134d8505adcd56bb57acbe7598a459
git -C "$NATIVE_ROOT/source" apply \
  "$SDK_SOURCE/docs/porting/patches/librealsense-2.58.4-cleanup-ownership.patch"
cmake -S "$NATIVE_ROOT/source" -B "$NATIVE_ROOT/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=15.0 -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_SHARED_LIBS=OFF -DBUILD_PYTHON_BINDINGS=ON \
  -DBUILD_LEGACY_PYBACKEND=OFF -DBUILD_RS2_ALL=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
  -DBUILD_UNIT_TESTS=OFF -DBUILD_GLSL_EXTENSIONS=OFF \
  -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_DDS=OFF -DUSE_EXTERNAL_USB=OFF \
  -DPYTHON_EXECUTABLE="$NATIVE_ROOT/env/bin/python" \
  -DLIBUSB_LIB="$NATIVE_ROOT/usb/lib/libusb-1.0.0.dylib" \
  -DLIBUSB_INC="$NATIVE_ROOT/usb/include/libusb-1.0" \
  -DCMAKE_INSTALL_PREFIX="$NATIVE_ROOT/install" \
  -DPYTHON_INSTALL_DIR="$NATIVE_ROOT/install/python/pyrealsense2"
cmake --build "$NATIVE_ROOT/build" --target pyrealsense2 --parallel 4

clang++ -std=c++14 -O2 -Wall -Wextra -Werror \
  -mmacosx-version-min=15.0 \
  -I "$NATIVE_ROOT/source/third-party/rsutils/include" \
  tools/check_realsense_cleanup.cpp -o "$NATIVE_ROOT/check-cleanup"
"$NATIVE_ROOT/check-cleanup"
mkdir -p "$NATIVE_ROOT/reference/pyrealsense2"
cp "$NATIVE_ROOT/source/wrappers/python/pyrealsense2/__init__.py" \
  "$NATIVE_ROOT/reference/pyrealsense2/"
cp "$NATIVE_ROOT/build/Release/pyrealsense2.cpython-312-darwin.so" \
  "$NATIVE_ROOT/reference/pyrealsense2/"
PYTHONPATH="$NATIVE_ROOT/reference" python tools/check_realsense_alignment.py \
  --output "$NATIVE_ROOT/reference.json"

git -C "$NATIVE_ROOT/source" apply \
  "$SDK_SOURCE/docs/porting/patches/librealsense-2.58.4-align-gil.patch"
cmake --build "$NATIVE_ROOT/build" --target pyrealsense2 --parallel 4
python tools/package_macos_realsense.py \
  --source "$NATIVE_ROOT/source" --build "$NATIVE_ROOT/build" \
  --libusb-archive "$NATIVE_ROOT/libusb-1.0.30.tar.bz2" \
  --output "$NATIVE_ROOT/wheels"
uv venv --python "$NATIVE_ROOT/env/bin/python" "$NATIVE_ROOT/wheel-test"
uv pip install --python "$NATIVE_ROOT/wheel-test/bin/python" numpy==2.2.6 \
  "$NATIVE_ROOT/wheels/pyrealsense2-2.58.4.post2-cp312-cp312-macosx_15_0_arm64.whl"
"$NATIVE_ROOT/wheel-test/bin/python" tools/check_realsense_alignment.py \
  --output "$NATIVE_ROOT/candidate.json"
```

Compare every frame's `pixels` row in the two JSON files; RGB and aligned depth
hashes must match exactly. Retain both timing distributions and observer progress.
The diagnostic suppresses ordinary Python GIL handoffs during each short alignment
call, while a sleeping observer records actual concurrent progress. That is a
binding check, not a physical control deadline test. Native cleanup acceptance
also fails on the original header before either source patch.

The assembler verifies both source patches against the exact upstream bytes,
the selected interpreter and CMake options, actual arm64/minimum-OS binaries,
libusb version and absence of the newer `pipe2` import. It bundles one shared USB
library using delocate, rejects nonportable dependencies, and retains USB source,
patches, notices and a build manifest. The static USB archive exists for upstream
tests only; it is not embedded in the binding. Artifact directories and evidence
files refuse overwrite. The recipe is repeatable; bit-identical compiler output
is not established.

Source or wheel import does not establish macOS interface authorization. A camera
access refusal stays explicit; do not run an entire application/evaluator with
administrator authority or replace metric RGB-D with RGB-only capture. Qualify
the chosen camera access provider, exact camera streams, calibration, shutdown,
and control deadlines under representative load before selecting this candidate.

Mock and both native MuJoCo camera paths report `simulated_state`, bounding the
state snapshot used for rendering. The subprocess worker preserves those host
intervals through IPC; rendering/transport delays do not restamp the content.
This is simulation state timing, not physical sensor calibration or exposure.

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
