"""Managed rig ownership and the vendor-neutral camera seam."""

from __future__ import annotations

import queue
import sys
import threading
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest
import waddle_sdk
from waddle_sdk import descriptors
from waddle_sdk.cameras import CameraDriver, CameraFrame, CameraSample
from waddle_sdk.robots import base

JOINTS = ("joint",)
LIMITS = ((-1.0, 1.0),)
STEPS = (0.1,)
RATE_HZ = 20.0


class _ClosingTwin(base.SimDriver):
    def __init__(self, closed: list[str]) -> None:
        super().__init__(
            [0.0], lower=[-1.0], upper=[1.0], step_caps=STEPS, rate_hz=RATE_HZ
        )
        self._closed = closed

    def close(self) -> None:
        self._closed.append("arm")


class _BlockingCamera:
    """A structural driver whose close observably unblocks capture."""

    def __init__(self, closed: list[str]) -> None:
        self._frames: queue.Queue[CameraFrame | None] = queue.Queue()
        self._closed = closed
        self._close_lock = threading.Lock()

    def push(self, frame: CameraFrame) -> None:
        self._frames.put(frame)

    def capture(self) -> CameraFrame:
        frame = self._frames.get()
        if frame is None:
            raise RuntimeError("camera closed")
        return frame

    def close(self) -> None:
        with self._close_lock:
            if "camera" in self._closed:
                return
            self._closed.append("camera")
            self._frames.put(None)


def _robot() -> descriptors.Robot:
    return descriptors.Robot(
        name="camera-rig",
        action_space=descriptors.JointSpace(joints=JOINTS, rate_hz=RATE_HZ),
        cameras={
            "overhead": descriptors.Camera(
                width=2,
                height=2,
                fps=RATE_HZ,
                intrinsics=descriptors.Intrinsics(
                    fx=100.0,
                    fy=100.0,
                    cx=0.0,
                    cy=0.0,
                    depth_scale_mm=1.0,
                ),
            )
        },
    )


def _rig(
    closed: list[str],
    camera: _BlockingCamera,
    *,
    camera_name: str = "overhead",
    declaration: descriptors.Robot | None = None,
) -> base.Rig:
    def build_arms() -> dict[str, base.Arm]:
        return {
            "": base.Arm(
                part="",
                driver=_ClosingTwin(closed),
                joint_names=JOINTS,
                joint_limits=LIMITS,
                step_caps=STEPS,
                rate_hz=RATE_HZ,
            )
        }

    return base.Rig(
        declaration=declaration or _robot(),
        build_arms=build_arms,
        build_cameras=lambda: {camera_name: camera},
        rate_hz=RATE_HZ,
        report=lambda _line: None,
    )


def test_rig_session_owns_pumps_and_closes_a_blocked_camera():
    closed: list[str] = []
    camera = _BlockingCamera(closed)
    rig = _rig(closed, camera)

    managed = rig.session("project", console=False)
    with managed:
        assert managed.core is not None
        assert managed.pump is not None and managed.pump.is_alive()
        assert managed.camera_pumps["overhead"].is_alive()

    assert closed == ["camera", "arm"]
    assert managed.pump is None
    assert managed.camera_pumps == {}

    managed.close()
    assert closed == ["camera", "arm"]


def test_rig_session_registers_optional_live_camera_intrinsics():
    closed: list[str] = []

    class CalibratedCamera(_BlockingCamera):
        def intrinsics(self) -> descriptors.Intrinsics:
            return descriptors.Intrinsics(
                fx=612.0,
                fy=613.0,
                cx=321.0,
                cy=239.0,
                depth_scale_mm=1.0,
            )

    camera = CalibratedCamera(closed)
    declared = _robot()
    declared = descriptors.Robot(
        name=declared.name,
        action_space=declared.action_space,
        cameras={"overhead": descriptors.Camera(width=2, height=2, fps=RATE_HZ)},
    )
    managed = _rig(closed, camera, declaration=declared).session(
        "project", console=False
    )

    with managed:
        intrinsics = managed.robot.cameras["overhead"].intrinsics
        assert intrinsics is not None
        assert (intrinsics.fx, intrinsics.fy) == (612.0, 613.0)
        compiled = managed.robot._compile([])["cameras"][0]["intrinsics"]
        assert compiled["fx"] == 612.0
        assert compiled["depthScaleMm"] == 1.0


def test_partial_camera_open_failure_closes_camera_then_arm():
    closed: list[str] = []
    camera = _BlockingCamera(closed)
    rig = _rig(closed, camera, camera_name="not-declared")

    with (
        pytest.raises(ValueError, match="exactly match the declaration"),
        rig.session("project", console=False),
    ):
        pass

    assert closed == ["camera", "arm"]


@dataclass(frozen=True)
class _Stamp:
    session_ns: int
    unix_ns: int


class _RecordingSession:
    def __init__(self) -> None:
        self._next = 0
        self.published: list[tuple[str, np.ndarray]] = []

    def stamp(self) -> _Stamp:
        self._next += 1
        return _Stamp(session_ns=self._next, unix_ns=1_000_000 + self._next)

    def publish_frame(self, camera: str, rgb: np.ndarray) -> None:
        self.published.append((camera, rgb))


def test_capture_keeps_correlated_depth_local_and_resolves_pixels():
    closed: list[str] = []
    driver = _BlockingCamera(closed)
    rig = _rig(closed, driver)
    session = _RecordingSession()
    pump = rig.camera_pumps(session, {"overhead": driver})["overhead"]

    source_rgb = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    source_depth = np.array([[1000, 1500], [1750, 2000]], dtype=np.uint16)
    frame = CameraFrame(
        rgb=source_rgb,
        depth=source_depth,
        point_resolver=lambda x, y, depth_m: (
            x * depth_m / 100.0,
            y * depth_m / 100.0,
            depth_m,
        ),
    )
    source_rgb[:] = 0
    source_depth[:] = 0

    pump.start()
    driver.push(frame)
    sample = rig.wait_camera("overhead", timeout_s=2.0)
    assert sample is not None
    pump.stop()

    assert isinstance(driver, CameraDriver)
    assert isinstance(sample, CameraSample)
    assert (sample.session_ns, sample.unix_ns) == (1, 1_000_001)
    assert sample.rgb.flags.writeable is False
    assert sample.depth is not None and sample.depth.flags.writeable is False
    np.testing.assert_array_equal(
        sample.rgb, np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    )
    np.testing.assert_array_equal(
        sample.depth, np.array([[1000, 1500], [1750, 2000]], dtype=np.uint16)
    )
    assert rig.resolve_pixel("overhead", 1, 1) == pytest.approx((0.02, 0.02, 2.0))

    assert len(session.published) == 1
    camera_name, published = session.published[0]
    assert camera_name == "overhead"
    assert published is sample.rgb
    assert published.ndim == 3  # publish_frame receives RGB, never the depth plane
    assert closed == ["camera"]


def test_camera_sample_refuses_unresolvable_depth():
    intrinsics = descriptors.Intrinsics(
        fx=100.0, fy=100.0, cx=0.0, cy=0.0, depth_scale_mm=1.0
    )
    sample = CameraSample(
        stamp=_Stamp(session_ns=1, unix_ns=2),
        rgb=np.zeros((1, 1, 3), dtype=np.uint8),
        depth=np.zeros((1, 1), dtype=np.uint16),
    )

    with pytest.raises(ValueError, match="no valid depth"):
        sample.point_at(0, 0, intrinsics)
    with pytest.raises(ValueError, match="outside"):
        sample.point_at(1, 0, intrinsics)
    with pytest.raises(ValueError, match="non-zero distortion"):
        sample.point_at(
            0,
            0,
            descriptors.Intrinsics(
                fx=100.0,
                fy=100.0,
                cx=0.0,
                cy=0.0,
                distortion=(0.1,),
                depth_scale_mm=1.0,
            ),
        )


def test_camera_sample_uses_driver_resolver_for_distorted_depth():
    calls: list[tuple[int, int, float]] = []

    def resolve(x: int, y: int, depth_m: float) -> tuple[float, float, float]:
        calls.append((x, y, depth_m))
        return (0.12, -0.03, depth_m)

    sample = CameraSample(
        stamp=_Stamp(session_ns=1, unix_ns=2),
        rgb=np.zeros((2, 2, 3), dtype=np.uint8),
        depth=np.array([[0, 0], [0, 750]], dtype=np.uint16),
        point_resolver=resolve,
    )
    intrinsics = descriptors.Intrinsics(
        fx=100.0,
        fy=100.0,
        cx=0.0,
        cy=0.0,
        distortion=(0.1,),
        depth_scale_mm=1.0,
    )

    assert sample.point_at(1, 1, intrinsics) == pytest.approx((0.12, -0.03, 0.75))
    assert calls == [(1, 1, 0.75)]


def test_vendor_adapters_are_lazy_and_name_their_install_extras(monkeypatch):
    before = set(sys.modules)
    from waddle_sdk.cameras import depthai, orbbec, realsense

    assert "depthai" not in set(sys.modules) - before
    assert "pyorbbecsdk" not in set(sys.modules) - before
    assert "pyrealsense2" not in set(sys.modules) - before

    def absent(name: str):
        raise ModuleNotFoundError(name=name)

    monkeypatch.setattr(depthai.importlib, "import_module", absent)
    with pytest.raises(RuntimeError, match=r"waddle-sdk\[depthai\]"):
        depthai.DepthaiDriver(mxid="oak-test")
    monkeypatch.setattr(orbbec.importlib, "import_module", absent)
    with pytest.raises(RuntimeError, match=r"waddle-sdk\[orbbec\]"):
        orbbec.OrbbecDriver()
    monkeypatch.setattr(realsense.importlib, "import_module", absent)
    with pytest.raises(RuntimeError, match=r"waddle-sdk\[realsense\]"):
        realsense.RealSenseDriver()


def test_camera_extra_metadata_is_orthogonal_to_teleop():
    try:
        import tomllib
    except ModuleNotFoundError:  # pragma: no cover - Python 3.10 only
        import tomli as tomllib

    pyproject = Path(__file__).resolve().parents[1] / "pyproject.toml"
    extras = tomllib.loads(pyproject.read_text(encoding="utf-8"))["project"][
        "optional-dependencies"
    ]
    assert extras["depthai"] == ["depthai>=3,<4"]
    assert extras["orbbec"] == ["pyorbbecsdk2"]
    assert extras["realsense"] == ["pyrealsense2"]
    assert extras["usb"] == ["opencv-python-headless>=4.8"]
    assert set(extras["cameras"]) == set(
        extras["depthai"] + extras["orbbec"] + extras["realsense"] + extras["usb"]
    )
    assert extras["teleop"] == [f"waddle-sdk-teleop=={waddle_sdk.__version__}"]
