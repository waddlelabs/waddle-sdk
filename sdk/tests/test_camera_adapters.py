"""Fake-vendor tests for the SDK mock and USB camera adapters."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
from waddle_sdk.cameras import CameraDriver, usb
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
