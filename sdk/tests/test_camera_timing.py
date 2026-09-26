"""Content timing bounds remain independent of delivery and session stamps."""

import time
from dataclasses import FrozenInstanceError
from types import SimpleNamespace

import numpy as np
import pytest
from waddle_sdk.cameras import CameraContentTiming, CameraFrame, CameraSample
from waddle_sdk.cameras.mock import MockDriver


@pytest.mark.parametrize("depth", [False, True])
def test_mock_snapshot_intervals_survive_delayed_sample_delivery(depth):
    driver = MockDriver(width=16, height=12, has_depth=depth)
    began = time.monotonic_ns()
    frame = driver.capture()
    ended = time.monotonic_ns()
    driver.set_object(u=0, v=0)
    # Session stamps are an independent clock pair, never reconstructed from
    # host nanoseconds or used to manufacture a new content acquisition time.
    sample = CameraSample(
        SimpleNamespace(session_ns=17, unix_ns=100),
        frame.rgb,
        frame.depth,
        content_timing=frame.content_timing,
    )
    timing = sample.content_timing
    assert timing.kind == "simulated_state"
    assert began <= timing.rgb_monotonic_ns[0] <= timing.rgb_monotonic_ns[1] <= ended
    assert (timing.depth_monotonic_ns is not None) is depth
    assert sample.session_ns == 17 and sample.unix_ns == 100
    with pytest.raises(FrozenInstanceError):
        timing.kind = "sensor_exposure"
    driver.close()


@pytest.mark.parametrize(
    "span", [(-1, 1), (2, 1), (True, 1), (1.0, 2), (1, 2**63), (1,)]
)
def test_invalid_content_intervals_are_rejected(span):
    with pytest.raises(ValueError):
        CameraContentTiming("sensor_exposure", "calibrated-clock-1", span)


def test_timing_must_cover_exactly_the_available_planes():
    source = [10, 11]
    timing = CameraContentTiming("sensor_exposure", "clock-1", source)
    source[0] = 0
    assert timing.rgb_monotonic_ns == (10, 11)
    rgb = np.zeros((2, 2, 3), dtype=np.uint8)
    depth = np.ones((2, 2), dtype=np.uint16)
    with pytest.raises(ValueError, match="planes"):
        CameraFrame(rgb, depth, content_timing=timing)
    assert CameraFrame(rgb).content_timing is None


def test_native_worker_preserves_content_timing_across_ipc(tmp_path, monkeypatch):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    from waddle_sdk.simulators.adapters import Camera, World
    from waddle_sdk.simulators.scene import make_site

    _, config = make_site(
        "timing",
        backend="mujoco",
        robot="yam",
        environment="two_cubes",
        width=64,
        height=48,
    )
    world = World(config, real_time=False)
    world.open()
    try:
        name = next(iter(config["cameras"]))
        camera = Camera(
            world,
            SimpleNamespace(
                name=name, intrinsics=config["cameras"][name]["intrinsics"]
            ),
        )
        began = time.monotonic_ns()
        frame = camera.capture()
        ended = time.monotonic_ns()
        assert frame.depth is not None
        timing = frame.content_timing
        assert camera.content_timing_kind == timing.kind == "simulated_state"
        assert timing.rgb_monotonic_ns == timing.depth_monotonic_ns
        assert (
            began <= timing.rgb_monotonic_ns[0] <= timing.rgb_monotonic_ns[1] <= ended
        )
        assert frame.rgb.shape == (48, 64, 3)
        camera.close()
    finally:
        world.close()
