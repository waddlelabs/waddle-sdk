"""Exercise actual child IPC while it runs, stalls, fails and shuts down."""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from waddle_sdk import LiveKit
from waddle_sdk._preview import IsolatedPreview, PreviewSession
from waddle_sdk.cameras import CameraFrame

pytestmark = pytest.mark.skipif(
    not hasattr(os, "memfd_create"), reason="Linux isolation"
)


class ControlSession:
    def __init__(self):
        self.frames = 0
        self.closed = False

    def publish_frame(self, camera, frame):
        assert camera == "scene" and frame.shape == (480, 640, 3)
        self.frames += 1

    def shutdown(self):
        self.closed = True


def start_preview(**kwargs):
    media = LiveKit(
        "ws://unused.test",
        "private-grant",
        preview_fps=10,
        depth_preview=False,
        video_only=True,
        isolated=True,
        **kwargs,
    )
    preview = IsolatedPreview(
        media, {"scene": SimpleNamespace(width=640, height=480)}, testing_loopback=True
    )
    return preview, PreviewSession(ControlSession(), preview)


def wait_published(preview):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        rows = preview.tracks()
        if rows and rows[0]["status"] == "published":
            return
        time.sleep(0.02)
    pytest.fail("preview child did not publish the retained RGB sample")


def test_stalled_or_locked_preview_cannot_block_owner_publication():
    preview, session = start_preview(worker_cpu_ids=(max(os.sched_getaffinity(0)),))
    frame = np.full((480, 640, 3), 123, dtype=np.uint8)
    locker = None
    try:
        session.publish_frame("scene", frame)
        wait_published(preview)
        child = preview._process
        for task in Path(f"/proc/{child.pid}/task").iterdir():
            assert os.sched_getaffinity(int(task.name)) == {
                max(os.sched_getaffinity(0))
            }
            assert os.sched_getscheduler(int(task.name)) == os.SCHED_IDLE
        os.kill(child.pid, signal.SIGSTOP)
        slot = preview._slots["scene"]
        original_size = os.fstat(slot.fd).st_size
        locker = subprocess.Popen(
            [
                sys.executable,
                "-c",
                "import fcntl,sys,time; fcntl.lockf(int(sys.argv[1]),fcntl.LOCK_EX); print('locked',flush=True); time.sleep(30)",
                str(slot.fd),
            ],
            pass_fds=(slot.fd,),
            stdout=subprocess.PIPE,
            text=True,
        )
        assert locker.stdout.readline().strip() == "locked"
        slot.next_frame = 0
        started = time.monotonic()
        for _ in range(200):
            session.publish_frame("scene", frame)
        assert time.monotonic() - started < 1
        assert session._inner.frames == 201 and slot.dropped == 200
        assert os.fstat(slot.fd).st_size == original_size
        assert np.all(frame == 123)
    finally:
        if locker is not None:
            locker.terminate()
            locker.wait(timeout=2)
        if preview._process is not None and preview._process.poll() is None:
            os.kill(preview._process.pid, signal.SIGCONT)
        session.shutdown()
        assert session._inner.closed
        assert preview._process.poll() is not None


def test_dead_preview_retains_scoped_error_without_poisoning_control():
    preview, session = start_preview()
    try:
        preview._process.kill()
        preview._process.wait(timeout=2)
        for _ in range(20):
            session.publish_frame("scene", np.zeros((480, 640, 3), dtype=np.uint8))
        assert session._inner.frames == 20
        with pytest.raises(RuntimeError, match="media_worker_exit"):
            session.media_tracks()
    finally:
        session.shutdown()


def test_invalid_worker_placement_disables_only_optional_preview():
    preview, session = start_preview(worker_cpu_ids=(999999,))
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                preview.tracks()
            except RuntimeError as error:
                assert "media_worker_failed: OSError" in str(error)
                break
            time.sleep(0.02)
        else:
            pytest.fail("invalid CPU placement was not reported")
        session.publish_frame("scene", np.zeros((480, 640, 3), dtype=np.uint8))
        assert session._inner.frames == 1
    finally:
        session.shutdown()


def test_optional_teardown_cannot_replace_original_control_failure():
    class FailedControl:
        def shutdown(self):
            raise RuntimeError("original motor teardown fault")

    class FailedPreview:
        def close(self):
            raise RuntimeError("optional preview teardown fault")

    session = PreviewSession(FailedControl(), FailedPreview())
    with pytest.raises(RuntimeError, match="original motor teardown fault"):
        session.shutdown()


def test_optional_teardown_retains_completed_control_cleanup():
    class FailedPreview:
        def close(self):
            raise RuntimeError("optional preview teardown fault")

    control = ControlSession()
    session = PreviewSession(control, FailedPreview())
    session.shutdown()
    assert control.closed
    assert session._preview_close_error == "optional preview teardown fault"


def test_managed_native_owner_retains_exact_rgb_depth_with_isolated_preview(
    monkeypatch,
):
    import waddle_sdk._preview as previews
    import waddle_sdk._session as composition
    from test_managed_rig_camera import _BlockingCamera, _rig

    class LoopbackPreview(IsolatedPreview):
        def __init__(self, media, cameras):
            super().__init__(media, cameras, testing_loopback=True)

    monkeypatch.setattr(previews, "IsolatedPreview", LoopbackPreview)
    monkeypatch.setattr(composition._native, "FEATURES", frozenset({"grpc", "livekit"}))
    closed = []
    camera = _BlockingCamera(closed)
    media = LiveKit(
        "ws://unused.test",
        "private-grant",
        preview_fps=1,
        depth_preview=False,
        video_only=True,
        isolated=True,
    )
    managed = _rig(closed, camera).session("project", console=False, media=media)
    rgb = np.full((2, 2, 3), 117, dtype=np.uint8)
    depth = np.full((2, 2), 543, dtype=np.uint16)
    with managed:
        camera.push(CameraFrame(rgb=rgb, depth=depth))
        sample = managed.wait_camera("overhead", timeout_s=2)
        assert sample is not None
        np.testing.assert_array_equal(sample.rgb, rgb)
        np.testing.assert_array_equal(sample.depth, depth)
        assert managed.core._inner.media_tracks() == []
        wait_published(managed.core._preview)
        child = managed.core._preview._process
    assert closed == ["camera", "arm"]
    assert child.poll() is not None


def test_preview_child_exits_when_its_owning_process_dies():
    program = """
import time
from types import SimpleNamespace
from waddle_sdk import LiveKit
from waddle_sdk._preview import IsolatedPreview
preview = IsolatedPreview(LiveKit("ws://unused.test", "private-grant", preview_fps=1,
    isolated=True, video_only=True, depth_preview=False),
    {"scene": SimpleNamespace(width=16,height=12)}, testing_loopback=True)
print(preview._process.pid,flush=True)
time.sleep(30)
"""
    parent = subprocess.Popen(
        [sys.executable, "-c", program], stdout=subprocess.PIPE, text=True
    )
    child = None
    try:
        child = int(parent.stdout.readline())
        parent.kill()
        parent.wait(timeout=2)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            status = Path(f"/proc/{child}/stat")
            if not status.exists() or status.read_text().split(") ", 1)[1].startswith(
                "Z "
            ):
                return
            time.sleep(0.05)
        pytest.fail("preview child outlived its owning process")
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait(timeout=2)
        if child is not None and Path(f"/proc/{child}/stat").exists():
            try:
                os.kill(child, signal.SIGKILL)
            except ProcessLookupError:
                pass
