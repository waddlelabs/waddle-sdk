"""Rendering may be stalled without stalling the world's sole control writer."""

import os
import signal
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import numpy as np
import pytest
from waddle_sdk.simulators.adapters import Camera, World


@pytest.mark.parametrize("operation", ["read", "write", "hold"])
def test_control_finishes_while_a_camera_render_is_blocked(operation):
    entered = threading.Event()
    release = threading.Event()
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)

    class Renderer:
        def capture(self, camera, acquire, cancelled):
            snapshot, stamp = acquire(None)
            entered.set()
            assert release.wait(5)
            return rgb, None, snapshot, stamp

    class Connection:
        def send(self, request):
            self.operation = request[0]

        def poll(self, timeout):
            if self.operation == "capture":
                entered.set()
                assert release.wait(5)
            return True

        def recv(self):
            if self.operation == "capture":
                return True, (rgb, None)
            if self.operation == "render_snapshot":
                return True, {"generation": 0, "parts": {}, "timing": None}
            return True, operation

    world = World({"backend": "mujoco"})
    world._connection = Connection()
    world._renderer = Renderer()
    camera = Camera(
        world,
        SimpleNamespace(
            name="scene",
            intrinsics={
                "fx": 1,
                "fy": 1,
                "cx": 1,
                "cy": 1,
            },
        ),
    )
    with ThreadPoolExecutor(max_workers=2) as pool:
        captured = pool.submit(camera.capture)
        try:
            assert entered.wait(2), "camera did not enter rendering"
            controlled = pool.submit(world.call, operation)
            assert controlled.result(timeout=2) == operation
        finally:
            release.set()
        assert captured.result(timeout=2).rgb.shape == (2, 3, 3)


@pytest.mark.parametrize("reset", [False, True])
def test_native_stalled_renderer_retains_acquisition_and_refuses_pre_reset_frames(
    reset,
):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.scene import make_site

    _, config = make_site(
        "capture",
        backend="mujoco",
        robot="yam",
        environment="two_cubes",
        width=64,
        height=48,
    )
    world = World(config, real_time=False)
    world._part_frames["arm"] = "arm_base"
    world.open()
    camera = Camera(
        world,
        SimpleNamespace(
            name="scene", intrinsics=config["cameras"]["scene"]["intrinsics"]
        ),
    )
    renderer = world._renderer
    sent = threading.Event()
    connection = renderer.connection

    class ObservedConnection:
        def send(self, value):
            connection.send(value)
            sent.set()

        def __getattr__(self, name):
            return getattr(connection, name)

    renderer.connection = ObservedConnection()
    stamps = []

    def stamp():
        value = SimpleNamespace(session_ns=time.monotonic_ns(), unix_ns=time.time_ns())
        stamps.append(value)
        return value

    original = world.call("read")[0].copy()
    os.kill(renderer.process.pid, signal.SIGSTOP)
    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            captured = pool.submit(camera.capture_with_state, stamp)
            try:
                assert sent.wait(3)
                # Real control transactions must complete while the native GL
                # process cannot run, not merely beat a render in a timing race.
                target = original.copy()
                target[-1] = 0.5
                assert (
                    pool.submit(world.call, "write", target).result(timeout=2) is None
                )
                pool.submit(world.call, "step", 0.2).result(timeout=2)
                changed = pool.submit(world.call, "read").result(timeout=2)[0]
                assert abs(changed[-1] - original[-1]) > 0.1
                pool.submit(world.call, "hold").result(timeout=2)
                if reset:
                    pool.submit(world.call, "reset").result(timeout=2)
            finally:
                os.kill(renderer.process.pid, signal.SIGCONT)
            frame = captured.result(timeout=5)
            assert frame.capture_state.revision == str(int(reset))
            assert len(stamps) == (2 if reset else 1)
            assert frame.capture_state.stamp is stamps[-1]
            np.testing.assert_allclose(
                frame.capture_state.parts["arm"].joint_position, original
            )
            assert frame.depth.shape == frame.rgb.shape[:2]
            assert frame.content_timing.rgb_monotonic_ns[1] <= stamps[-1].session_ns
            with pytest.raises(ValueError):
                frame.capture_state.parts["arm"].joint_position.setflags(write=True)
            with pytest.raises(TypeError):
                frame.capture_state.parts["arm"] = None
    finally:
        camera.close()
        world.close()


@pytest.mark.parametrize("handoff_ns", [49_000_000, 51_000_000])
def test_pairing_bound_includes_native_clock_handoff(monkeypatch, handoff_ns):
    now = [1]
    monkeypatch.setattr(time, "monotonic_ns", lambda: now[0])

    class Renderer:
        def capture(self, camera, acquire, cancelled):
            snapshot, stamp = acquire(0)
            return None, None, snapshot, stamp

    def stamp():
        now[0] += handoff_ns
        return SimpleNamespace(session_ns=now[0], unix_ns=now[0])

    world = World({"backend": "mujoco"})
    world._renderer = Renderer()
    world.call = lambda *args: {"generation": 0, "interval": (0, 1)}
    if handoff_ns > 50_000_000:
        with pytest.raises(RuntimeError, match="50 ms pairing bound"):
            world.capture("scene", stamp=stamp)
    else:
        assert world.capture("scene", stamp=stamp)[3].session_ns == now[0]


def test_latest_slot_withdraws_a_prior_source_revision():
    from waddle_sdk.robots.base import _LatestCameraSamples

    latest = _LatestCameraSamples()
    generation = [0]
    sample = SimpleNamespace(session_ns=10)
    latest.publish("scene", sample, current=lambda: generation[0] == 0)
    assert latest.get("scene") is sample
    generation[0] = 1
    assert latest.get("scene") is None
    assert latest.wait("scene", timeout=0) is None


def test_renderer_preserves_the_original_snapshot_failure():
    from waddle_sdk.simulators.render_client import RenderClient

    fault = ValueError("source.snapshot: original acquisition failure")
    client = RenderClient(
        SimpleNamespace(wait=lambda **kwargs: None),
        SimpleNamespace(send=lambda value: None, close=lambda: None),
        cameras=["scene"],
        generation=0,
    )

    def acquire(generation):
        raise fault

    try:
        with pytest.raises(ValueError) as failure:
            client.capture("scene", acquire, lambda: False)
        assert failure.value is fault
        with pytest.raises(ValueError) as repeated:
            client.capture("scene", acquire, lambda: False)
        assert repeated.value is fault
    finally:
        client.close()


def test_owner_holds_physics_before_draining_a_blocked_renderer():
    operations = []

    class Connection:
        def send(self, request):
            operations.append(request[0])

        def poll(self, timeout):
            return True

        def recv(self):
            return True, None

        def close(self):
            operations.append("physics_closed")

    class Renderer:
        def close(self):
            assert operations == ["close", "physics_closed"]
            operations.append("renderer_closed")

    world = World({"backend": "mujoco"})
    world._connection = Connection()
    world._renderer = Renderer()
    world._independent_render = True
    world.close()
    assert operations[-1] == "renderer_closed"
    with pytest.raises(RuntimeError, match="renderer is unavailable"):
        world.call("capture", "scene")


def test_snapshot_renderer_preserves_native_rgb_and_metric_depth(tmp_path):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine
    from waddle_sdk.simulators.render_worker import SnapshotRenderer
    from waddle_sdk.simulators.scene import make_site

    _, config = make_site(
        "pixels",
        backend="mujoco",
        robot="yam",
        environment="two_cubes",
        width=64,
        height=48,
    )
    engine = Engine(config, tmp_path)
    renderer = SnapshotRenderer(config)
    try:
        snapshot = engine.render_snapshot(0, None)
        renderer.restore(snapshot)
        for camera in config["cameras"]:
            expected_rgb, expected_depth = engine.capture(camera)
            actual_rgb, actual_depth = renderer.capture(camera)
            np.testing.assert_array_equal(actual_rgb, expected_rgb)
            np.testing.assert_array_equal(actual_depth, expected_depth)
    finally:
        renderer.close()
        engine.close()


def test_slow_autonomous_physics_services_hold_after_one_native_step(monkeypatch):
    from waddle_sdk.simulators import worker

    clock = SimpleNamespace(now=0.0)
    held, steps = [], []

    class Engine:
        render_snapshot = staticmethod(lambda *args: None)

        def __init__(self, *args):
            pass

        def step(self):
            steps.append(clock.now)
            clock.now += 0.007

        def hold(self):
            held.append(clock.now)

        def close(self):
            pass

    class Connection:
        def __init__(self):
            self.messages = iter(
                [
                    {
                        "backend": "mujoco",
                        "cameras": {},
                        "timestep": 0.002,
                        "_real_time": True,
                    },
                    ("hold", ()),
                    ("close", ()),
                ]
            )

        def poll(self, timeout):
            clock.now = max(clock.now, 0.1)
            return True

        def recv(self):
            return next(self.messages)

        def send(self, result):
            assert result[0], result

        def close(self):
            pass

    monkeypatch.setattr(worker.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(
        worker,
        "importlib",
        SimpleNamespace(import_module=lambda _: SimpleNamespace(Engine=Engine)),
    )
    worker.serve(Connection())
    assert len(steps) == 1
    assert held[0] == pytest.approx(0.107)
