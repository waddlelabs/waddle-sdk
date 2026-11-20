"""Reference-worker control stays available while native pixels are pending."""

import multiprocessing
import os
import threading
from types import SimpleNamespace

import pytest
from waddle_sdk.simulators import worker
from waddle_sdk.simulators.adapters import World


@pytest.mark.parametrize("close_fault", [False, True])
def test_feedback_targets_and_hold_complete_before_pending_pixels(
    monkeypatch, close_fault
):
    clock = SimpleNamespace(now=0.0)
    rendering, release = threading.Event(), threading.Event()
    state = SimpleNamespace(target=0, steps=[], render_thread=None, closed_thread=None)

    class Engine:
        parallel_camera_capture = True
        content_timing_kind = "simulated_state"

        def __init__(self, config, scratch):
            pass

        def step(self):
            state.steps.append(state.target)

        def write(self, target):
            state.target = target

        def read(self):
            return state.target, len(state.steps)

        def feedback_snapshot(self):
            return state.target, len(state.steps), state.target * 10

        def hold(self):
            state.target = 0

        def prepare_capture(self, name):
            return state.target, clock.now

        def render_capture(self, prepared):
            state.render_thread = threading.get_ident()
            rendering.set()
            assert release.wait(30), "test failed to release pixel rendering"
            return prepared

        def capture_timed(self, name):
            return self.render_capture(self.prepare_capture(name))

        def capture(self, name):
            return self.capture_timed(name)

        def close_renderers(self):
            state.closed_thread = threading.get_ident()
            if close_fault:
                raise ValueError("native renderer close fault")

        def close(self):
            pass

    monkeypatch.setattr(worker, "time", SimpleNamespace(monotonic=lambda: clock.now))
    monkeypatch.setattr(
        worker,
        "importlib",
        SimpleNamespace(import_module=lambda _: SimpleNamespace(Engine=Engine)),
    )
    control, child = multiprocessing.Pipe()
    camera, camera_child = multiprocessing.Pipe()
    camera_fd = os.dup(camera_child.fileno())
    camera_child.close()
    control.send(
        {
            "backend": "mujoco",
            "cameras": {},
            "timestep": 0.002,
            "_real_time": True,
            "_camera_connection_fd": camera_fd,
        }
    )
    serving = threading.Thread(target=worker.serve, args=(child,))
    parallel = False
    completed = False
    receipt = None

    def receive(connection):
        assert connection.poll(5), "control was blocked by pending pixel rendering"
        ok, result = connection.recv()
        assert ok, result
        return result

    serving.start()
    try:
        ready = receive(control)
        assert ready.get("feedback_snapshot", False) is True
        parallel = ready.get("parallel_camera_capture", False)
        capture = camera if parallel else control
        control.send(("write", [1]))
        receive(control)
        clock.now = 0.02
        capture.send(("capture", ["scene", True]))
        assert rendering.wait(5), "camera did not enter rendering"
        clock.now = 0.1
        control.send(("write", [2]))
        receive(control)
        clock.now = 0.11
        control.send(("read", []))
        assert receive(control) == (2, 55)
        control.send(("feedback_snapshot", []))
        assert receive(control) == (2, 55, 20)
        control.send(("hold", []))
        receive(control)
        assert state.target == 0
        assert state.steps == [1] * 50 + [2] * 5
        release.set()
        assert receive(capture) == (1, 0.02)
        completed = True
    finally:
        release.set()
        if serving.is_alive():
            control.send(("close", []))
            if completed:
                assert control.poll(5), "missing native camera drain outcome"
                receipt = control.recv()
        serving.join(10)
        assert not serving.is_alive(), "simulation worker failed to drain"
        control.close()
        camera.close()
        if not parallel:
            os.close(camera_fd)
    assert state.render_thread == state.closed_thread
    assert receipt == (
        (False, "ValueError: native renderer close fault")
        if close_fault
        else (True, {"camera_drain_confirmed": True})
    )


def test_camera_channel_leaves_control_available():
    from concurrent.futures import ThreadPoolExecutor

    world = World({"backend": "mujoco"})
    entered, release = threading.Event(), threading.Event()
    calls = []

    class ControlConnection:
        def send(self, request):
            calls.append(request[0])

        def poll(self, timeout):
            return True

        def recv(self):
            return True, {"camera_drain_confirmed": True} if calls[
                -1
            ] == "close" else "state"

        def close(self):
            pass

    class CameraConnection(ControlConnection):
        def send(self, request):
            assert request == ("capture", ("scene",))

        def poll(self, timeout):
            entered.set()
            assert release.wait(10), "test failed to release camera response"
            return True

        def recv(self):
            return True, "pixels"

    world._parallel_camera = True
    world._connection = ControlConnection()
    world._capture_connection = CameraConnection()
    with ThreadPoolExecutor(max_workers=1) as pool:
        capture = pool.submit(world.call, "capture", "scene")
        try:
            assert entered.wait(5)
            assert world.call("read") == "state"
            assert world.call("write", 2) == "state"
            assert world.call("hold") == "state"
        finally:
            release.set()
        assert capture.result(timeout=5) == "pixels"
    world.close()
    world.close()
    assert calls == ["read", "write", "hold", "close"]
    assert world._camera_drain_confirmed and not world._failed


@pytest.mark.parametrize(
    "receipt", [None, {"camera_drain_confirmed": False}, {"camera_drain_confirmed": 1}]
)
def test_missing_camera_drain_receipt_stays_fenced(receipt):
    world = World({"backend": "mujoco"})

    class Connection:
        def send(self, request):
            assert request == ("close", ())

        def poll(self, timeout):
            return True

        def recv(self):
            return True, receipt

        def close(self):
            pass

    world._parallel_camera = True
    world._connection = Connection()
    with pytest.raises(RuntimeError, match="did not confirm camera drain") as first:
        world.close()
    with pytest.raises(RuntimeError) as second:
        world.close()
    assert second.value is first.value
    with pytest.raises(RuntimeError, match="simulation failed"):
        world.open()


def test_camera_transport_loss_never_falls_back_or_clears_on_close():
    world = World({"backend": "mujoco"})

    class CameraConnection:
        def send(self, request):
            pass

        def poll(self, timeout):
            return True

        def recv(self):
            raise EOFError("native camera channel lost")

        def close(self):
            pass

    class ControlConnection(CameraConnection):
        def send(self, request):
            pytest.fail("camera request fell back into control")

    world._parallel_camera = True
    world._capture_connection = CameraConnection()
    world._connection = ControlConnection()
    with pytest.raises(RuntimeError, match="simulation connection lost") as original:
        world.call("capture", "scene")
    with pytest.raises(RuntimeError) as closing:
        world.close()
    assert closing.value is original.value
    assert world._failed and not world._camera_drain_confirmed


def test_pending_native_render_cannot_confirm_teardown():
    entered, release, closed = threading.Event(), threading.Event(), threading.Event()

    class Engine:
        def prepare_capture(self, name):
            return None

        def render_capture(self, prepared):
            entered.set()
            assert release.wait(20)
            return "rgb", "depth", "time"

        def close_renderers(self):
            closed.set()

        def hold(self):
            pass

    parent, child = multiprocessing.Pipe()
    capture = worker._CameraLoop(Engine(), child, threading.RLock(), lambda: None, ())
    capture.start()
    parent.send(("capture", ["scene"]))
    try:
        assert entered.wait(5)
        with pytest.raises(RuntimeError, match="camera worker did not drain"):
            capture.shutdown()
        assert capture.thread.is_alive() and not closed.is_set()
    finally:
        release.set()
        assert parent.poll(5)
        assert parent.recv() == (True, ("rgb", "depth"))
        capture.shutdown()
        parent.close()
    assert closed.is_set() and not capture.thread.is_alive()
