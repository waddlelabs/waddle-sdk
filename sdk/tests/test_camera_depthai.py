from __future__ import annotations

from datetime import timedelta
from types import SimpleNamespace
from typing import ClassVar

import numpy as np
import pytest
from waddle_sdk.cameras import CameraCalibrationDriver, CameraDriver, depthai


class _Info:
    def __init__(self, mxid: str):
        self.mxid = mxid

    def getDeviceId(self):
        return self.mxid


class _Calibration:
    def __init__(self, malformed: bool = False):
        self.malformed = malformed

    def getCameraIntrinsics(self, socket, width, height):
        if self.malformed:
            return [[1.0]]
        assert socket in {"CAM_A", "CAM_B"}
        assert (width, height) == (3, 2)
        return [[600.0, 0.0, 1.5], [0.0, 601.0, 1.0], [0.0, 0.0, 1.0]]


class _Device:
    devices: ClassVar[list[_Info]] = [_Info("oak-a"), _Info("oak-b")]
    instances: ClassVar[list[_Device]] = []
    malformed_calibration = False
    connected: ClassVar[list[str]] = ["CAM_A"]

    @classmethod
    def getAllAvailableDevices(cls):
        return list(cls.devices)

    def __init__(self, info):
        self.info = info
        self.closed = False
        self.__class__.instances.append(self)

    def readCalibration(self):
        return _Calibration(self.malformed_calibration)

    def getConnectedCameras(self):
        return list(self.connected)

    def close(self):
        self.closed = True


class _Frame:
    def __init__(self, value):
        self.value = value

    def getFrame(self):
        return self.value


class _Queue:
    def __init__(self):
        self.timeout = None
        self.get_calls = 0
        self.group = {
            "rgb": _Frame(np.arange(18, dtype=np.uint8).reshape(2, 3, 3)),
            "depth": _Frame(np.arange(6, dtype=np.uint16).reshape(2, 3)),
        }

    def get(self, timeout):
        self.timeout = timeout
        self.get_calls += 1
        return self.group


class _Output:
    def __init__(self, queue=None):
        self.links = []
        self.queue = queue

    def link(self, target):
        self.links.append(target)

    def createOutputQueue(self, maxSize, blocking):
        assert (maxSize, blocking) == (4, True)
        return self.queue


class _CameraNode:
    def __init__(self):
        self.requests = []

    def build(self, socket):
        self.socket = socket
        return self

    def requestOutput(self, size, frame_type, **kwargs):
        output = _Output()
        self.requests.append((size, frame_type, kwargs, output))
        return output


class _StereoNode:
    def __init__(self):
        self.depth = _Output()
        self.calls = []

    def build(self, *args):
        self.calls.append(("build", args))
        return self

    def setDepthAlign(self, value):
        self.calls.append(("align", value))

    def setOutputSize(self, width, height):
        self.calls.append(("size", width, height))

    def setLeftRightCheck(self, enabled):
        self.calls.append(("lr", enabled))

    def setSubpixel(self, enabled):
        self.calls.append(("subpixel", enabled))


class _SyncNode:
    def __init__(self, queue):
        self.inputs = {"rgb": "rgb-input", "depth": "depth-input"}
        self.out = _Output(queue)
        self.calls = []

    def setSyncThreshold(self, threshold):
        self.calls.append(("threshold", threshold))

    def setSyncAttempts(self, attempts):
        self.calls.append(("attempts", attempts))


class _Pipeline:
    instances: ClassVar[list[_Pipeline]] = []

    def __init__(self, device):
        self.device = device
        self.queue = _Queue()
        self.nodes = []
        self.started = False
        self.stopped = False
        self.__class__.instances.append(self)

    def create(self, kind):
        if kind is _CameraKind:
            node = _CameraNode()
        elif kind is _StereoKind:
            node = _StereoNode()
        elif kind is _SyncKind:
            node = _SyncNode(self.queue)
        else:
            raise AssertionError(kind)
        self.nodes.append(node)
        return node

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True


class _CameraKind:
    pass


class _StereoKind:
    PresetMode = SimpleNamespace(
        ROBOTICS="ROBOTICS",
        __members__={"ROBOTICS": "ROBOTICS", "DEFAULT": "DEFAULT"},
    )


class _SyncKind:
    pass


def _vendor():
    return SimpleNamespace(
        Device=_Device,
        Pipeline=_Pipeline,
        node=SimpleNamespace(
            Camera=_CameraKind, StereoDepth=_StereoKind, Sync=_SyncKind
        ),
        CameraBoardSocket=SimpleNamespace(CAM_A="CAM_A", CAM_B="CAM_B", CAM_C="CAM_C"),
        ImgFrame=SimpleNamespace(
            Type=SimpleNamespace(RGB888i="RGB888i", GRAY8="GRAY8")
        ),
    )


@pytest.fixture(autouse=True)
def reset_fake():
    _Device.instances = []
    _Device.devices = [_Info("oak-a"), _Info("oak-b")]
    _Device.malformed_calibration = False
    _Device.connected = ["CAM_A"]
    _Pipeline.instances = []


def test_available_devices_is_stable_and_does_not_boot(monkeypatch):
    monkeypatch.setattr(depthai, "_vendor_module", _vendor)
    assert depthai.available_devices() == ("oak-a", "oak-b")
    assert _Device.instances == []


def test_depthai_rgbd_is_aligned_calibrated_and_closes_once(monkeypatch):
    monkeypatch.setattr(depthai, "_vendor_module", _vendor)
    driver = depthai.DepthaiDriver(mxid="oak-b", width=3, height=2, fps=20)
    assert isinstance(driver, CameraDriver)
    assert isinstance(driver, CameraCalibrationDriver)
    assert _Device.instances[-1].info.mxid == "oak-b"
    assert _Pipeline.instances[-1].started

    frame = driver.capture()
    assert frame.rgb.shape == (2, 3, 3)
    assert frame.depth is not None and frame.depth.shape == (2, 3)
    assert frame.rgb.flags.writeable is False
    assert frame.depth.flags.writeable is False
    assert _Pipeline.instances[-1].queue.timeout == timedelta(seconds=1.0)
    assert _Pipeline.instances[-1].queue.get_calls == 3
    calibration = driver.intrinsics()
    assert calibration.fx == pytest.approx(600.0)
    assert calibration.fy == pytest.approx(601.0)
    assert calibration.cx == pytest.approx(1.5)
    assert calibration.cy == pytest.approx(1.0)
    assert calibration.distortion == ()
    assert calibration.depth_scale_mm == 1.0

    driver.close()
    driver.close()
    assert _Pipeline.instances[-1].stopped
    assert _Device.instances[-1].closed
    with pytest.raises(RuntimeError, match="closed"):
        driver.capture()


def test_stereo_only_oak_uses_cam_b_color_grid(monkeypatch):
    monkeypatch.setattr(depthai, "_vendor_module", _vendor)
    _Device.connected = ["CAM_B", "CAM_C"]
    driver = depthai.DepthaiDriver(mxid="oak-a", width=3, height=2, fps=20)
    pipeline = _Pipeline.instances[-1]
    cameras = [node for node in pipeline.nodes if isinstance(node, _CameraNode)]
    stereo = next(node for node in pipeline.nodes if isinstance(node, _StereoNode))
    assert [camera.socket for camera in cameras] == ["CAM_B", "CAM_C"]
    assert [request[1] for request in cameras[0].requests] == [
        "RGB888i",
        "GRAY8",
    ]
    assert [request[1] for request in cameras[1].requests] == ["GRAY8"]
    assert stereo.calls[0][0] == "build"
    assert stereo.calls[1] == ("align", "CAM_B")
    driver.close()


def test_explicit_mxid_prevents_enumeration_order_selection(monkeypatch):
    monkeypatch.setattr(depthai, "_vendor_module", _vendor)
    with pytest.raises(RuntimeError, match=r"available=.*oak-a.*oak-b"):
        depthai.DepthaiDriver(mxid="missing", width=3, height=2)
    assert _Device.instances == []


def test_half_open_pipeline_is_closed_when_calibration_is_bad(monkeypatch):
    monkeypatch.setattr(depthai, "_vendor_module", _vendor)
    _Device.malformed_calibration = True
    with pytest.raises(RuntimeError, match="malformed"):
        depthai.DepthaiDriver(mxid="oak-a", width=3, height=2)
    assert _Device.instances[-1].closed
    assert _Pipeline.instances[-1].stopped


def test_depthai_import_is_lazy_and_names_its_extra(monkeypatch):
    def absent(name):
        raise ModuleNotFoundError(name=name)

    monkeypatch.setattr(depthai.importlib, "import_module", absent)
    with pytest.raises(RuntimeError, match=r"waddle-sdk\[depthai\]"):
        depthai.DepthaiDriver(mxid="oak-a")
