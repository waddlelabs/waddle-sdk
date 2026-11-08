"""A blocked camera process cannot own the caller's robot-control interpreter."""

import os
import threading

import numpy as np
import pytest
from waddle_sdk.cameras.realsense_process import IsolatedRealSenseDriver


@pytest.fixture
def driver(tmp_path, monkeypatch):
    (tmp_path / "camera_process_fixture.py").write_text("""
import os,sys
from multiprocessing.connection import Connection
import numpy as np
from waddle_sdk.descriptors import Intrinsics
c=Connection(int(sys.argv[1]));config=c.recv()
if config['serial']=='failed':
    c.send((False,(RuntimeError('camera transport failed'),ValueError('original USB cause'))));c.close();sys.exit()
metadata=(.1,Intrinsics(400,401,2,3,depth_scale_mm=.1),None)
c.send((True,metadata))
while True:
    request=c.recv()
    if request is None:break
    if config['serial']=='gated':
        from pathlib import Path
        import time
        gate=Path(config['width']);gate.with_suffix('.entered').touch()
        while not gate.exists():time.sleep(.001)
    c.send((True,(np.full((3,4,3),17,dtype=np.uint8),np.full((3,4),123,dtype=np.uint16),metadata)))
c.close()
""")
    monkeypatch.setenv(
        "PYTHONPATH", str(tmp_path) + os.pathsep + os.environ.get("PYTHONPATH", "")
    )

    class Driver(IsolatedRealSenseDriver):
        _worker_module = "camera_process_fixture"

    return Driver


def test_camera_images_and_calibration_survive_isolated_capture(driver):
    camera = driver(serial="normal")
    process = camera._process
    try:
        assert process.pid != os.getpid()
        sample = camera.capture()
        np.testing.assert_array_equal(
            sample.rgb, np.full((3, 4, 3), 17, dtype=np.uint8)
        )
        np.testing.assert_array_equal(
            sample.depth, np.full((3, 4), 123, dtype=np.uint16)
        )
        assert not sample.rgb.flags.writeable and not sample.depth.flags.writeable
        assert camera.depth_scale_mm == 0.1
        assert camera.intrinsics().fx == 400
        assert camera.intrinsics().fy == 401
    finally:
        camera.close()
    assert process.poll() == 0
    camera.close()
    with pytest.raises(RuntimeError, match="closed"):
        camera.capture()


def test_failed_open_retains_original_error_and_cause(driver):
    with pytest.raises(RuntimeError, match="camera transport failed") as caught:
        driver(serial="failed")
    assert isinstance(caught.value.__cause__, ValueError)
    assert str(caught.value.__cause__) == "original USB cause"


def test_capture_does_not_block_independent_caller_work(driver, tmp_path):
    gate = tmp_path / "release"
    camera = driver(serial="gated", width=str(gate))
    result, errors = [], []

    def capture():
        try:
            result.append(camera.capture())
        except Exception as error:  # noqa: BLE001 -- retain failures from the test capture thread
            errors.append(error)

    thread = threading.Thread(target=capture)
    try:
        thread.start()
        # Explicit admission barrier: the camera cannot complete until this
        # independent caller advances, rather than a timing/scheduler race.
        import time

        deadline = time.monotonic() + 10
        while not gate.with_suffix(".entered").exists():
            if time.monotonic() >= deadline:
                pytest.fail("camera capture did not enter")
            time.sleep(0.001)
        assert thread.is_alive()
        gate.touch()
        thread.join(timeout=10)
        assert not thread.is_alive() and not errors
        assert len(result) == 1
    finally:
        gate.touch()
        thread.join(timeout=10)
        camera.close()


def test_site_factory_selects_vendor_isolation_without_opening_inline(driver):
    from waddle_sdk.cameras._factory import open_camera_driver
    from waddle_sdk.cameras.site import CameraConfig

    class Vendor:
        isolated_camera_factory = driver

        def __init__(self, **kwargs):
            pytest.fail("Site factory opened the vendor inside the control process")

    camera = open_camera_driver(
        Vendor,
        CameraConfig(
            name="scene",
            connection={"serial": "normal"},
            stream={"width": 4, "height": 3, "fps": 30},
            frame_id="optical",
            intrinsics=None,
        ),
    )
    try:
        assert camera._process.pid != os.getpid()
        assert camera.capture().rgb.shape == (3, 4, 3)
    finally:
        camera.close()
