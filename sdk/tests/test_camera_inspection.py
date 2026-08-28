"""Camera-only inspection remains separate from the full site lifecycle."""

from __future__ import annotations

import queue
import sys
import threading
import types

import numpy as np
import pytest
from waddle_sdk.cameras import (
    CameraFrame,
    CameraInspectionError,
    CameraInspectionSpec,
    inspect_cameras,
)
from waddle_sdk.discovery import HardwareCandidate


class _BlockingCamera:
    def __init__(self, name: str, events: list[str]) -> None:
        self.name = name
        self.events = events
        self.capture_started = threading.Event()
        self.capture_stopped = threading.Event()
        self.frames: queue.Queue[CameraFrame | None] = queue.Queue()
        self._close_lock = threading.Lock()
        self._closed = False

    def capture(self) -> CameraFrame:
        self.capture_started.set()
        frame = self.frames.get()
        if frame is None:
            self.capture_stopped.set()
            raise RuntimeError("camera closed")
        return frame

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self.events.append(f"close:{self.name}")
            self.frames.put(None)


@pytest.fixture
def fake_cameras(monkeypatch):
    events: list[str] = []
    drivers: dict[str, _BlockingCamera] = {}
    module = types.ModuleType("customer_inspection_cameras")

    def open_camera(*, config):
        events.append(f"open:{config.name}")
        driver = _BlockingCamera(config.name, events)
        drivers[config.name] = driver
        return driver

    module.open_camera = open_camera
    monkeypatch.setitem(sys.modules, module.__name__, module)
    return events, drivers, module


def _spec(name: str) -> CameraInspectionSpec:
    return CameraInspectionSpec(
        name=name,
        driver="customer_inspection_cameras:open_camera",
        connection={"serial": name},
        stream={"width": 2, "height": 2, "fps": 30},
    )


def test_inspection_is_unopened_until_enter_and_supports_multiple_latest_frames(
    fake_cameras,
) -> None:
    events, drivers, _module = fake_cameras
    inspection = inspect_cameras((_spec("left"), _spec("right")))
    assert events == []

    with inspection as session:
        assert session.names == ("left", "right")
        assert events == ["open:left", "open:right"]
        assert drivers["left"].capture_started.wait(timeout=1.0)
        assert drivers["right"].capture_started.wait(timeout=1.0)

        source = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
        drivers["left"].frames.put(CameraFrame(rgb=source))
        source[:] = 0
        left = session.wait("left", timeout_s=1.0)
        assert left is not None
        assert left.sequence == 1
        assert left.rgb.flags.writeable is False
        np.testing.assert_array_equal(
            left.rgb, np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
        )

        drivers["right"].frames.put(
            CameraFrame(rgb=np.full((2, 2, 3), 7, dtype=np.uint8))
        )
        right = session.wait("right", timeout_s=1.0)
        assert right is not None and right.camera == "right"
        assert session.latest("left") is left

        drivers["left"].frames.put(
            CameraFrame(rgb=np.full((2, 2, 3), 9, dtype=np.uint8))
        )
        newer = session.wait("left", after_sequence=left.sequence, timeout_s=1.0)
        assert newer is not None and newer.sequence == 2
        assert session.latest("left") is newer

    assert events[-2:] == ["close:right", "close:left"]
    session.close()
    assert events.count("close:left") == 1
    assert events.count("close:right") == 1


def test_candidate_conversion_and_discovery_stay_non_opening(fake_cameras) -> None:
    events, drivers, _module = fake_cameras
    candidate = HardwareCandidate(
        identifier="camera:serial-1",
        kind="camera",
        label="Camera serial-1",
        driver="customer_inspection_cameras:open_camera",
        connection={"serial": "serial-1"},
    )

    spec = CameraInspectionSpec.from_candidate(
        candidate, name="overhead", width=2, height=2
    )
    inspection = inspect_cameras((spec,))
    assert events == []
    assert drivers == {}

    with inspection as session:
        assert session.names == ("overhead",)
    assert events == ["open:overhead", "close:overhead"]

    unresolved = HardwareCandidate(
        identifier="camera:possible",
        kind="camera",
        label="Possible camera",
    )
    with pytest.raises(ValueError, match="does not identify an exact SDK driver"):
        CameraInspectionSpec.from_candidate(unresolved)


def test_partial_open_failure_closes_every_previous_camera(
    fake_cameras, monkeypatch
) -> None:
    events, drivers, module = fake_cameras

    def fail_camera(*, config):
        events.append(f"open:{config.name}")
        raise RuntimeError("vendor refused second camera")

    module.fail_camera = fail_camera
    failing = CameraInspectionSpec(
        name="broken",
        driver="customer_inspection_cameras:fail_camera",
        connection={},
        stream={"width": 2, "height": 2, "fps": 30},
    )

    with (
        pytest.raises(
            CameraInspectionError,
            match=r"camera 'broken' failed to open \(RuntimeError\)",
        ) as raised,
        inspect_cameras((_spec("first"), failing)),
    ):
        pass

    assert events == ["open:first", "open:broken", "close:first"]
    assert drivers["first"].capture_started.is_set() is False
    assert "vendor refused" not in str(raised.value)


def test_context_close_unblocks_pending_capture_without_site_runtime(
    fake_cameras,
) -> None:
    events, drivers, _module = fake_cameras

    with inspect_cameras((_spec("blocked"),), close_timeout_s=1.0) as session:
        assert drivers["blocked"].capture_started.wait(timeout=1.0)
        assert session.latest("blocked") is None

    assert drivers["blocked"].capture_stopped.wait(timeout=1.0)
    assert events == ["open:blocked", "close:blocked"]
    assert session.wait("blocked") is None


def test_capture_failure_is_bounded_and_reported(fake_cameras) -> None:
    _events, drivers, _module = fake_cameras

    with inspect_cameras((_spec("wrong-shape"),)) as session:
        drivers["wrong-shape"].capture_started.wait(timeout=1.0)
        drivers["wrong-shape"].frames.put(
            CameraFrame(rgb=np.zeros((1, 1, 3), dtype=np.uint8))
        )
        assert session.wait("wrong-shape", timeout_s=1.0) is None
        assert "requested 2x2" in session.errors["wrong-shape"]


def test_vendor_capture_detail_is_not_exposed(fake_cameras) -> None:
    events, drivers, module = fake_cameras

    class SecretErrorCamera(_BlockingCamera):
        def capture(self) -> CameraFrame:
            self.capture_started.set()
            raise RuntimeError("token=do-not-share")

    def secret_camera(*, config):
        events.append(f"open:{config.name}")
        driver = SecretErrorCamera(config.name, events)
        drivers[config.name] = driver
        return driver

    module.secret_camera = secret_camera
    spec = CameraInspectionSpec(
        name="secret",
        driver="customer_inspection_cameras:secret_camera",
        connection={},
        stream={"width": 2, "height": 2, "fps": 30},
    )
    with inspect_cameras((spec,)) as session:
        assert session.wait("secret", timeout_s=1.0) is None
        assert session.errors["secret"] == "capture failed (RuntimeError)"
        assert "do-not-share" not in session.errors["secret"]


def test_vendor_close_detail_is_not_exposed(fake_cameras) -> None:
    events, drivers, module = fake_cameras

    class SecretCloseCamera(_BlockingCamera):
        def capture(self) -> CameraFrame:
            self.capture_started.set()
            raise RuntimeError("capture-key=do-not-share")

        def close(self) -> None:
            events.append(f"close:{self.name}")
            raise RuntimeError("close-key=do-not-share")

    def secret_close_camera(*, config):
        events.append(f"open:{config.name}")
        driver = SecretCloseCamera(config.name, events)
        drivers[config.name] = driver
        return driver

    module.secret_close_camera = secret_close_camera
    spec = CameraInspectionSpec(
        name="secret-close",
        driver="customer_inspection_cameras:secret_close_camera",
        connection={},
        stream={"width": 2, "height": 2, "fps": 30},
    )
    with (
        pytest.raises(CameraInspectionError) as raised,
        inspect_cameras((spec,)) as session,
    ):
        assert session.wait("secret-close", timeout_s=1.0) is None

    message = str(raised.value)
    assert "secret-close: close failed (RuntimeError)" in message
    assert "do-not-share" not in message


def test_invalid_driver_and_duplicate_names_fail_before_hardware_opens(
    fake_cameras,
) -> None:
    events, _drivers, _module = fake_cameras

    with pytest.raises(ValueError, match="names must be unique"):
        inspect_cameras((_spec("same"), _spec("same")))
    assert events == []

    missing = CameraInspectionSpec(
        name="missing",
        driver="missing_camera_package:open_camera",
        connection={},
        stream={"width": 2, "height": 2, "fps": 30},
    )
    with (
        pytest.raises(CameraInspectionError, match="cannot load camera driver"),
        inspect_cameras((missing,)),
    ):
        pass
    assert events == []
