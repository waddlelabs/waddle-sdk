"""Fake-vendor tests for the SDK mock and USB camera adapters."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
from waddle_sdk.cameras import CameraDriver, realsense, usb
from waddle_sdk.cameras import mock as mock_camera


class _Capture:
    def __init__(self, *, opened: bool = True) -> None:
        self.opened = opened
        self.releases = 0
        self.settings: list[tuple[int, int]] = []
        self.frame = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)

    def isOpened(self) -> bool:
        return self.opened

    def set(self, key: int, value: int) -> None:
        self.settings.append((key, value))

    def read(self):
        return True, self.frame

    def release(self) -> None:
        self.releases += 1


def _cv2(capture: _Capture):
    return SimpleNamespace(
        VideoCapture=lambda _device: capture,
        CAP_PROP_FRAME_WIDTH=1,
        CAP_PROP_FRAME_HEIGHT=2,
        CAP_PROP_FPS=3,
        COLOR_BGR2RGB=4,
        cvtColor=lambda value, _code: value[..., ::-1],
    )


def test_usb_adapter_is_lazy_converts_bgr_and_closes_once(monkeypatch):
    capture = _Capture()
    monkeypatch.setattr(usb, "_vendor_module", lambda: _cv2(capture))

    driver = usb.USBDriver(device="/dev/video-test", width=2, height=1, fps=20)
    assert isinstance(driver, CameraDriver)
    frame = driver.capture()
    assert frame.rgb.tolist() == [[[3, 2, 1], [6, 5, 4]]]
    assert capture.settings == [(1, 2), (2, 1), (3, 20)]

    driver.close()
    driver.close()
    assert capture.releases == 1
    with pytest.raises(RuntimeError, match="closed"):
        driver.capture()


def test_usb_adapter_releases_a_half_open_capture(monkeypatch):
    capture = _Capture(opened=False)
    monkeypatch.setattr(usb, "_vendor_module", lambda: _cv2(capture))

    with pytest.raises(RuntimeError, match="failed to open"):
        usb.USBDriver()
    assert capture.releases == 1


def test_mock_adapter_produces_mutable_pixel_aligned_rgbd():
    driver = mock_camera.MockDriver(
        width=8,
        height=6,
        object_u=2,
        object_v=3,
        object_radius_px=1,
        object_depth_m=0.25,
    )
    assert isinstance(driver, CameraDriver)

    first = driver.capture()
    assert first.rgb.shape == (6, 8, 3)
    assert first.depth is not None and first.depth.shape == (6, 8)
    assert first.rgb[3, 2].tolist() == [0, 220, 0]
    assert int(first.depth[3, 2]) == 250

    driver.set_object(u=6, v=1, depth_m=0.5)
    second = driver.capture()
    assert second.rgb[1, 6].tolist() == [0, 220, 0]
    assert int(second.depth[1, 6]) == 500
    assert first.rgb[3, 2].tolist() == [0, 220, 0]

    driver.close()
    driver.close()
    with pytest.raises(RuntimeError, match="closed"):
        driver.capture()


def test_mock_adapter_can_be_rgb_only():
    driver = mock_camera.MockDriver(width=4, height=3, has_depth=False)
    assert driver.capture().depth is None
    driver.close()


class _RsDevice:
    def __init__(self, serial: str) -> None:
        self.serial = serial
        self.resets = 0

    def get_info(self, _field):
        return self.serial

    def first_depth_sensor(self):
        return SimpleNamespace(get_depth_scale=lambda: 0.001)

    def hardware_reset(self) -> None:
        self.resets += 1


class _RsFrames:
    def __init__(self) -> None:
        self._rgb = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        self._depth = np.arange(6, dtype=np.uint16).reshape(2, 3)

    def get_color_frame(self):
        return SimpleNamespace(get_data=lambda: self._rgb)

    def get_depth_frame(self):
        return SimpleNamespace(get_data=lambda: self._depth)


class _RsPipeline:
    def __init__(self, behavior: dict[str, bool], device: _RsDevice) -> None:
        self.behavior = behavior
        self.device = device
        self.stops = 0

    def start(self, _config):
        return SimpleNamespace(get_device=lambda: self.device)

    def try_wait_for_frames(self, _timeout_ms: int):
        return self.behavior["confirm"], _RsFrames()

    def wait_for_frames(self):
        if self.behavior.pop("capture_error", False):
            raise RuntimeError("Frame didn't arrive within 5000")
        return _RsFrames()

    def stop(self) -> None:
        self.stops += 1


class _RsConfig:
    def __init__(self) -> None:
        self.serial = ""
        self.streams: list[tuple[object, ...]] = []

    def enable_device(self, serial: str) -> None:
        self.serial = serial

    def enable_stream(self, *args: object) -> None:
        self.streams.append(args)


class _RsVendor:
    def __init__(self, behaviors: list[dict[str, bool]]) -> None:
        self.behaviors = list(behaviors)
        self.device = _RsDevice("rs-test")
        self.pipelines: list[_RsPipeline] = []
        self.stream = SimpleNamespace(color="color", depth="depth")
        self.format = SimpleNamespace(rgb8="rgb8", z16="z16")
        self.camera_info = SimpleNamespace(serial_number="serial_number")

    def context(self):
        return SimpleNamespace(query_devices=lambda: [self.device])

    def pipeline(self, _context):
        pipeline = _RsPipeline(self.behaviors.pop(0), self.device)
        self.pipelines.append(pipeline)
        return pipeline

    def config(self):
        return _RsConfig()

    def align(self, _stream):
        return SimpleNamespace(process=lambda frames: frames)


def _fast_realsense(monkeypatch, vendor: _RsVendor) -> None:
    monkeypatch.setattr(realsense, "_vendor_module", lambda: vendor)
    monkeypatch.setattr(realsense.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(realsense.RealSenseDriver, "_STREAM_CONFIRM_TIMEOUT_S", 0.001)


def test_realsense_resets_a_pipeline_that_opens_without_frames(monkeypatch):
    vendor = _RsVendor([{"confirm": False}, {"confirm": True}])
    _fast_realsense(monkeypatch, vendor)

    driver = realsense.RealSenseDriver(serial="rs-test", width=3, height=2, fps=30)
    frame = driver.capture()
    driver.close()
    driver.close()

    assert vendor.device.resets == 1
    assert len(vendor.pipelines) == 2
    assert vendor.pipelines[0].stops == 1
    assert vendor.pipelines[1].stops == 1
    assert frame.rgb.shape == (2, 3, 3)
    assert frame.depth is not None and frame.depth.shape == (2, 3)
    assert driver.depth_scale_mm == pytest.approx(1.0)


def test_realsense_rebuilds_after_a_later_capture_timeout(monkeypatch):
    vendor = _RsVendor(
        [
            {"confirm": True, "capture_error": True},
            {"confirm": True},
        ]
    )
    _fast_realsense(monkeypatch, vendor)

    driver = realsense.RealSenseDriver(serial="rs-test", width=3, height=2)
    assert driver.capture().rgb.shape == (2, 3, 3)
    driver.close()

    assert vendor.device.resets == 0
    assert len(vendor.pipelines) == 2
    assert [pipeline.stops for pipeline in vendor.pipelines] == [1, 1]


def test_realsense_fails_after_one_bounded_hardware_reset(monkeypatch):
    vendor = _RsVendor([{"confirm": False}, {"confirm": False}])
    _fast_realsense(monkeypatch, vendor)

    with pytest.raises(RuntimeError, match="after one hardware reset"):
        realsense.RealSenseDriver(serial="rs-test")

    assert vendor.device.resets == 1
    assert [pipeline.stops for pipeline in vendor.pipelines] == [1, 1]
