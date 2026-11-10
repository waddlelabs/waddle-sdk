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
    aligned = NS(as_frame_set=lambda: frames)

    def wait(_timeout):
        state.captures += 1
        return (
            None
            if (missing or state.captures <= getattr(state, "delay", 0))
            else frames
        )

    def stop():
        state.stops += 1

    def select(serial):
        state.selected.append(serial)
        if serial != "CV2L761000XJ":
            raise RuntimeError("configured camera not found")
        return getattr(state, "device", "selected-device")

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
        AlignFilter=lambda **kw: NS(process=lambda f: aligned),
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


def test_orbbec_startup_waits_for_actual_first_frame(monkeypatch):
    state = vendor(monkeypatch)
    state.delay = 3
    driver = orbbec.OrbbecDriver()
    assert state.captures == 4
    assert driver.intrinsics().depth_scale_mm == 0.25
    driver.close()


def test_orbbec_startup_wait_is_bounded(monkeypatch):
    state = vendor(monkeypatch, missing=True)
    with pytest.raises(RuntimeError, match="after 10 frame wait"):
        orbbec.OrbbecDriver()
    assert state.captures == 10
    assert state.stops == 1


def test_orbbec_running_capture_does_not_inherit_startup_wait(monkeypatch):
    state = vendor(monkeypatch)
    driver = orbbec.OrbbecDriver()
    state.delay = 100
    with pytest.raises(RuntimeError, match="after 1 frame wait"):
        driver.capture()
    assert state.captures == 2
    driver.close()


def recovering_vendor(monkeypatch, *, disappearance=True, returns=True):
    state = vendor(monkeypatch)
    state.delay = 10
    state.reboots = 0
    state.clock = 0.0
    state.polls = 0
    sdk = orbbec._vendor_module()
    original_context = sdk.Context

    def reboot():
        assert state.stops == 1
        state.reboots += 1

    state.device = NS(reboot=reboot)

    def count():
        state.polls += 1
        if not disappearance:
            return 1
        return int(returns and state.polls > 1)

    def query():
        devices = original_context().query_devices()
        devices.get_count = count
        devices.get_device_serial_number_by_index = lambda i: "CV2L761000XJ"
        return devices

    sdk.Context = lambda: NS(query_devices=query)

    def sleep(seconds):
        state.clock += seconds

    monkeypatch.setattr(orbbec, "time", NS(monotonic=lambda: state.clock, sleep=sleep))
    return state


def test_startup_timeout_stops_reboots_exact_serial_and_retries_once(monkeypatch):
    state = recovering_vendor(monkeypatch)
    driver = orbbec.OrbbecDriver(serial="CV2L761000XJ")
    assert state.reboots == 1
    assert state.captures == 11
    assert driver.intrinsics().depth_scale_mm == 0.25
    driver.close()
    assert state.stops == 2


def test_failed_recovery_is_not_repeated(monkeypatch):
    state = recovering_vendor(monkeypatch)
    state.delay = 100
    with pytest.raises(RuntimeError, match="timed out"):
        orbbec.OrbbecDriver(serial="CV2L761000XJ")
    assert state.reboots == 1
    assert state.captures == 20
    assert state.stops == 2


@pytest.mark.parametrize(
    "disappearance,returns,phase",
    [(False, True, "disconnect"), (True, False, "return")],
)
def test_reboot_requires_bounded_serial_disappearance_and_return(
    monkeypatch, disappearance, returns, phase
):
    state = recovering_vendor(monkeypatch, disappearance=disappearance, returns=returns)
    with pytest.raises(RuntimeError, match="did not " + phase):
        orbbec.OrbbecDriver(serial="CV2L761000XJ")
    assert state.reboots == 1
    assert state.captures == 10
    assert state.clock < 30.3


def test_invalid_scale_does_not_reboot(monkeypatch):
    state = recovering_vendor(monkeypatch)
    state.delay = 0
    state.scale = float("nan")
    with pytest.raises(ValueError, match="finite positive"):
        orbbec.OrbbecDriver(serial="CV2L761000XJ")
    assert state.reboots == 0


def test_busy_timeout_text_does_not_authorize_reboot(monkeypatch):
    state = recovering_vendor(monkeypatch)
    sdk = orbbec._vendor_module()
    original = sdk.Pipeline

    def fail():
        raise RuntimeError("busy device timed out")

    def pipeline(*args):
        p = original(*args)
        p.start = lambda config: fail()
        return p

    sdk.Pipeline = pipeline
    with pytest.raises(RuntimeError, match="busy device timed out"):
        orbbec.OrbbecDriver(serial="CV2L761000XJ")
    assert state.reboots == 0
    assert state.stops == 1


def test_failed_stop_is_not_permission_to_reboot(monkeypatch):
    state = recovering_vendor(monkeypatch)
    sdk = orbbec._vendor_module()
    original = sdk.Pipeline

    def fail():
        raise RuntimeError("stop failed")

    def pipeline(*args):
        p = original(*args)
        p.stop = fail
        return p

    sdk.Pipeline = pipeline
    with pytest.raises(RuntimeError, match="stop failed"):
        orbbec.OrbbecDriver(serial="CV2L761000XJ")
    assert state.reboots == 0
