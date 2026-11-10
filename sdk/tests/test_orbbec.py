from types import SimpleNamespace as NS

import numpy as np
import pytest
from waddle_sdk.cameras import CameraCalibrationDriver, orbbec


def vendor(monkeypatch, *, scale=0.25, missing=False):
    state = NS(stops=0, captures=0, selected=[], scale=scale)
    intr = NS(fx=400.0, fy=401.0, cx=1.0, cy=0.5)
    dist = NS(k1=0.01, k2=0.0, p1=0.0, p2=0.0, k3=0.0, k4=0.0, k5=0.0, k6=0.0)
    profile = NS(get_intrinsic=lambda: intr, get_distortion=lambda: dist)
    color = NS(
        get_width=lambda: 2,
        get_height=lambda: 1,
        get_data=lambda: np.zeros((1, 2, 3), dtype=np.uint8).tobytes(),
    )
    depth = NS(
        get_width=lambda: 2,
        get_height=lambda: 1,
        get_data=lambda: np.array([[400, 800]], dtype=np.uint16).tobytes(),
        get_depth_scale=lambda: state.scale,
    )
    frames = NS(get_color_frame=lambda: color, get_depth_frame=lambda: depth)

    def wait(_timeout):
        state.captures += 1
        return None if missing else frames

    def stop():
        state.stops += 1

    def select(serial):
        state.selected.append(serial)
        if serial != "CV2L761000XJ":
            raise RuntimeError("configured camera not found")
        return "selected-device"

    def pipeline(*devices):
        state.devices = devices
        return NS(
            get_stream_profile_list=lambda sensor: NS(
                get_video_stream_profile=lambda *args: profile
            ),
            start=lambda config: None,
            stop=stop,
            wait_for_frames=wait,
        )

    sdk = NS(
        Pipeline=pipeline,
        Context=lambda: NS(
            query_devices=lambda: NS(get_device_by_serial_number=select)
        ),
        Config=lambda: NS(
            enable_stream=lambda p: None, set_frame_aggregate_output_mode=lambda m: None
        ),
        OBSensorType=NS(COLOR_SENSOR=1, DEPTH_SENSOR=2),
        OBFormat=NS(RGB=1, Y16=2),
        OBStreamType=NS(COLOR_STREAM=1),
        OBFrameAggregateOutputMode=NS(FULL_FRAME_REQUIRE=1),
        AlignFilter=lambda **kw: NS(process=lambda f: f),
    )
    monkeypatch.setattr(orbbec, "_vendor_module", lambda: sdk)
    return state


def test_orbbec_publishes_acquired_calibration_before_first_pump(monkeypatch):
    state = vendor(monkeypatch)
    driver = orbbec.OrbbecDriver(serial="CV2L761000XJ", width=2, height=1)
    assert isinstance(driver, CameraCalibrationDriver)
    assert state.selected == ["CV2L761000XJ"]
    assert state.devices == ("selected-device",)
    assert state.captures == 1
    intr = driver.intrinsics()
    assert intr._compile()["depthScaleMm"] == 0.25
    assert (intr.fx, intr.fy, intr.cx, intr.cy) == (400.0, 401.0, 1.0, 0.5)
    assert intr.distortion[0] == 0.01
    frame = driver.capture()
    assert frame.depth.tolist() == [[400, 800]]
    assert (frame.depth * intr.depth_scale_mm).tolist() == [[100, 200]]
    driver.close()
    driver.close()
    assert state.stops == 1
    with pytest.raises(RuntimeError, match="closed"):
        driver.capture()


@pytest.mark.parametrize("scale", [0.0, -1.0, float("nan"), float("inf")])
def test_orbbec_invalid_scale_closes_before_rig_can_open(monkeypatch, scale):
    state = vendor(monkeypatch, scale=scale)
    with pytest.raises(ValueError, match="finite positive"):
        orbbec.OrbbecDriver()
    assert state.stops == 1


def test_orbbec_missing_frame_closes_pipeline(monkeypatch):
    state = vendor(monkeypatch, missing=True)
    with pytest.raises(RuntimeError, match="timed out"):
        orbbec.OrbbecDriver()
    assert state.stops == 1


def test_orbbec_never_falls_back_from_missing_serial(monkeypatch):
    state = vendor(monkeypatch)
    with pytest.raises(RuntimeError, match="configured camera not found"):
        orbbec.OrbbecDriver(serial="missing")
    assert state.captures == 0


def test_orbbec_changed_units_are_not_published_with_stale_calibration(monkeypatch):
    state = vendor(monkeypatch)
    driver = orbbec.OrbbecDriver()
    state.scale = 1.0
    with pytest.raises(RuntimeError, match="scale changed"):
        driver.capture()
    driver.close()


def test_orbbec_calibration_reaches_rig_description(monkeypatch):
    from waddle_sdk import descriptors
    from test_managed_rig_camera import _rig, _robot

    vendor(monkeypatch)
    driver = orbbec.OrbbecDriver(width=2, height=1)
    robot = _robot()
    declared = descriptors.Robot(
        name=robot.name,
        action_space=robot.action_space,
        cameras={"overhead": descriptors.Camera(width=2, height=1, fps=20.0)},
    )
    with _rig([], driver, declaration=declared).session(
        "project", console=False
    ) as session:
        intrinsics = session.robot._compile([])["cameras"][0]["intrinsics"]
        assert intrinsics["depthScaleMm"] == 0.25
        assert intrinsics["fx"] == 400.0
