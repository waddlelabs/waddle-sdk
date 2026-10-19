"""Orbbec paired-frame, calibration and lifecycle behavior without hardware."""

from types import SimpleNamespace as NS

import numpy as np
import pytest
from waddle_sdk.cameras import orbbec


class Vendor:
    def __init__(self):
        self.now = 0.0
        self.stops = 0
        self.align_calls = 0
        self.empty = False
        self.selected = None
        self.rgb = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        self.depth = np.full((2, 3), 500, dtype=np.uint16)
        self.distortion = NS(
            model=NS(name="BROWN_CONRADY_K6"),
            k1=0.1,
            k2=0.2,
            k3=0.3,
            k4=0.4,
            k5=0.5,
            k6=0.6,
            p1=0.01,
            p2=0.02,
        )
        self.profile = NS(
            get_intrinsic=lambda: NS(
                width=3, height=2, fx=100.0, fy=101.0, cx=1.0, cy=1.0
            ),
            get_distortion=lambda: self.distortion,
        )
        self.color = NS(
            get_width=lambda: 3,
            get_height=lambda: 2,
            get_format=lambda: "RGB",
            get_data=lambda: self.rgb,
            get_stream_profile=lambda: NS(as_video_stream_profile=lambda: self.profile),
        )
        self.depth_frame = NS(
            get_width=lambda: self.depth.shape[1],
            get_height=lambda: self.depth.shape[0],
            get_format=lambda: "Y16",
            get_data=lambda: self.depth,
            get_depth_scale=lambda: 0.5,
        )
        self.frames = NS(
            get_color_frame=lambda: self.color, get_depth_frame=lambda: self.depth_frame
        )
        self.pipeline = NS(
            get_stream_profile_list=lambda _: NS(
                get_video_stream_profile=lambda *args: args
            ),
            start=lambda _: None,
            stop=self.stop,
            wait_for_frames=self.wait,
            enable_frame_sync=lambda: None,
        )
        self.sdk = NS(
            Pipeline=lambda *args: self.pipeline,
            Config=lambda: NS(
                enable_stream=lambda _: None,
                set_frame_aggregate_output_mode=lambda _: None,
            ),
            OBSensorType=NS(COLOR_SENSOR="color", DEPTH_SENSOR="depth"),
            OBStreamType=NS(COLOR_STREAM="color"),
            OBFormat=NS(RGB="RGB", Y16="Y16"),
            OBFrameAggregateOutputMode=NS(FULL_FRAME_REQUIRE="full"),
            AlignFilter=lambda **_: NS(process=self.align),
            Context=lambda: NS(
                query_devices=lambda: NS(get_device_by_serial_number=self.device)
            ),
        )

    def device(self, serial):
        self.selected = serial
        if serial != "camera-1":
            raise RuntimeError("selected serial not connected")
        return NS(get_device_info=lambda: NS(get_serial_number=lambda: serial))

    def stop(self):
        self.stops += 1

    def wait(self, timeout):
        self.now += timeout / 1000
        return None if self.empty else self.frames

    def align(self, frames):
        self.align_calls += 1
        if self.align_calls == 1:
            return None  # Startup can precede the first complete aligned pair.
        return NS(as_frame_set=lambda: frames)


@pytest.fixture
def vendor(monkeypatch):
    value = Vendor()
    monkeypatch.setattr(orbbec, "_vendor_module", lambda: value.sdk)
    return value


def test_pairing_owns_pixels_and_preserves_full_color_calibration(vendor):
    driver = orbbec.OrbbecDriver(width=3, height=2, serial="camera-1")
    frame = driver.capture()
    assert vendor.selected == "camera-1"
    assert driver.intrinsics().distortion_model == "rational_polynomial"
    assert driver.intrinsics().distortion == (0.1, 0.2, 0.01, 0.02, 0.3, 0.4, 0.5, 0.6)
    assert driver.intrinsics().depth_scale_mm == 0.5
    assert driver.intrinsics().fx == 100.0
    vendor.rgb[:] = 0
    vendor.depth[:] = 0
    assert frame.rgb.sum() > 0 and np.all(frame.depth == 500)
    driver.close()
    driver.close()
    assert vendor.stops == 1
    with pytest.raises(RuntimeError, match="closed"):
        driver.capture()


def test_missing_serial_does_not_fall_back_to_another_camera(vendor):
    with pytest.raises(RuntimeError, match="selected serial not connected"):
        orbbec.OrbbecDriver(serial="missing")
    assert vendor.selected == "missing" and vendor.stops == 0


@pytest.mark.parametrize("returned_serial", [None, "different-camera"])
def test_serial_lookup_must_resolve_the_exact_camera(vendor, returned_serial):
    vendor.sdk.Context = lambda: NS(
        query_devices=lambda: NS(
            get_device_by_serial_number=lambda _: (
                None
                if returned_serial is None
                else NS(
                    get_device_info=lambda: NS(
                        get_serial_number=lambda: returned_serial
                    )
                )
            )
        )
    )
    with pytest.raises(RuntimeError, match="selected serial"):
        orbbec.OrbbecDriver(serial="camera-1")
    assert vendor.stops == 0


def test_startup_timeout_is_bounded_and_closes_stream(vendor, monkeypatch):
    vendor.empty = True
    monkeypatch.setattr(orbbec.time, "monotonic", lambda: vendor.now)
    with pytest.raises(RuntimeError, match="timed out"):
        orbbec.OrbbecDriver()
    assert vendor.stops == 1 and vendor.now <= 3.1


def test_alignment_shape_failure_closes_stream(vendor):
    vendor.depth = np.ones((1, 3), dtype=np.uint16)
    with pytest.raises(RuntimeError, match="aligned"):
        orbbec.OrbbecDriver(width=3, height=2)
    assert vendor.stops == 1


def test_partial_start_failure_preserves_original_error_and_closes(vendor):
    def fail(_):
        raise RuntimeError("USB stream start failed")

    vendor.pipeline.start = fail
    with pytest.raises(RuntimeError, match="USB stream start failed"):
        orbbec.OrbbecDriver()
    assert vendor.stops == 1


def test_depth_scale_change_cannot_silently_change_declared_metric_units(vendor):
    driver = orbbec.OrbbecDriver(width=3, height=2)
    driver.capture()  # Consume startup sample.
    vendor.depth_frame.get_depth_scale = lambda: 1.0
    try:
        with pytest.raises(RuntimeError, match="scale changed"):
            driver.capture()
    finally:
        driver.close()
