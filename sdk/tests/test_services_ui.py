"""Pure-Python optional-service facades and authenticated UI routes."""

from __future__ import annotations

import json
import urllib.parse
import urllib.request
from dataclasses import dataclass

import numpy as np
import pytest

import waddle_sdk
from waddle_sdk import _services
from waddle_sdk._ui import UIHandle


@dataclass(frozen=True)
class _Stamp:
    session_ns: int = 91
    unix_ns: int = 1_000_091


class _ManagedRig:
    def __init__(self) -> None:
        self.robot = waddle_sdk.Robot(
            name="rgbd",
            action_space=waddle_sdk.JointSpace(joints=["j0"]),
            cameras={
                "wrist": waddle_sdk.Camera(
                    width=2,
                    height=2,
                    fps=10,
                    frame_id="cam_wrist",
                    intrinsics=waddle_sdk.Intrinsics(
                        fx=100,
                        fy=100,
                        cx=0,
                        cy=0,
                        depth_scale_mm=1,
                    ),
                )
            },
        )
        self.sample = waddle_sdk.CameraSample(
            stamp=_Stamp(),
            frame_sequence=7,
            rgb=np.arange(12, dtype=np.uint8).reshape(2, 2, 3),
            depth=np.array([[1000, 1500], [1750, 2000]], dtype=np.uint16),
        )

    def camera_sample(self, name: str):
        if name != "wrist":
            raise ValueError("unknown camera")
        return self.sample

    def resolve_pixel(
        self, name: str, x: int, y: int, *, frame_sequence: int | None = None
    ):
        assert name == "wrist" and frame_sequence == self.sample.frame_sequence
        return self.sample.point_at(x, y, self.robot.cameras[name].intrinsics)


class _FakeSession:
    def __init__(self) -> None:
        self.task_submits: list[tuple] = []
        self.measurements: list[tuple] = []
        self.artifact_requests: list[tuple] = []
        self.handoffs = 0
        self.jogs = 0
        self.releases = 0

    def status(self):
        return {
            "plane_connected": True,
            "task_sessions_negotiated": True,
            "calibration_measurements_negotiated": True,
            "workspace_artifacts_negotiated": True,
            "cameras": [{"name": "wrist", "width": 2, "height": 2}],
            "jog_targets": [],
        }

    def task_session_submit(self, *args):
        self.task_submits.append(args)

    def task_session_events(self, request_id, after_sequence=0, timeout_ms=0):
        operation = next(item[1] for item in self.task_submits if item[0] == request_id)
        sequence = after_sequence + 1
        if operation == "history":
            return [
                {
                    "request_id": request_id,
                    "task_session_id": "task-1",
                    "name": "bench setup",
                    "sequence": sequence,
                    "kind": "text",
                    "text": "durable history",
                    "detail": "",
                },
                {
                    "request_id": request_id,
                    "task_session_id": "task-1",
                    "name": "bench setup",
                    "sequence": sequence + 1,
                    "kind": "history_complete",
                    "text": "",
                    "detail": "history page complete",
                    "history_cursor": sequence,
                },
            ]
        return [
            {
                "request_id": request_id,
                "task_session_id": "task-1",
                "name": "bench setup",
                "sequence": sequence,
                "kind": "done" if operation != "interrupt" else "interrupted",
                "text": "ready" if operation != "interrupt" else "",
                "detail": "",
            }
        ]

    def calibration_measurement_submit(self, *args):
        self.measurements.append(args)

    def calibration_updates(self, calibration_id, after_sequence=0, timeout_ms=0):
        return [
            {
                "calibration_id": calibration_id,
                "sequence": after_sequence + 1,
                "frame_sequence": 7,
                "kind": "accepted",
                "detail": "stored",
            }
        ]

    def workspace_artifact_request(self, *args):
        self.artifact_requests.append(args)

    def workspace_artifact_events(self, request_id, timeout_ms=0):
        return [
            {
                "request_id": request_id,
                "artifact_id": "artifact-1",
                "sha256": "a" * 64,
                "size_bytes": 12,
                "download_ref": "one-time-ref",
                "expires_unix_ns": 2_000_000,
                "detail": "ready",
            }
        ]

    def handoff_remote_to_local(self):
        self.handoffs += 1
        return True, "accepted", "accepted"

    def jog(self, *_args):
        self.jogs += 1
        return True, "accepted", "accepted"

    def jog_heartbeat(self):
        return True, "accepted", "accepted"

    def jog_release(self):
        self.releases += 1

    def request_estop(self):
        return "requested"

    def _ui_frame(self, _camera):
        return None


def _open(handle, path, *, method="GET", value=None):
    parsed = urllib.parse.urlsplit(handle.url)
    token = urllib.parse.parse_qs(parsed.fragment)["token"][0]
    headers = {"X-Waddle-Token": token, "X-Waddle-Request": "1"}
    if method == "POST":
        headers["Origin"] = f"{parsed.scheme}://{parsed.netloc}"
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(
        f"{parsed.scheme}://{parsed.netloc}{path}",
        data=None if value is None else json.dumps(value).encode(),
        headers=headers,
        method=method,
    )
    return urllib.request.urlopen(request, timeout=5)


def test_named_task_facade_tracks_identity_history_and_all_operations():
    native = _FakeSession()
    task = _services.TaskSession(native, "bench setup")
    create_request = task.request_id
    assert native.task_submits[-1] == (
        create_request,
        "create",
        None,
        "bench setup",
        None,
        0,
    )
    assert task.events()[-1]["text"] == "ready"
    assert task.task_session_id == "task-1"

    message = task.message("pick up the block")
    interject = task.interject("use the left side")
    interrupt = task.interrupt()
    assert [item[1] for item in native.task_submits] == [
        "create",
        "message",
        "interject",
        "interrupt",
    ]
    for request_id in (message, interject, interrupt):
        task.events(request_id=request_id)
    assert len(task.history) == 4

    resumed = _services.TaskSession(
        native, "bench setup", task_session_id="task-1"
    )
    history_request = resumed.request_id
    assert native.task_submits[-1] == (
        history_request,
        "history",
        "task-1",
        None,
        None,
        0,
    )
    events = resumed.events()
    assert [event["kind"] for event in events] == ["text", "history_complete"]
    assert resumed.history[-1]["history_cursor"] == 1
    resumed.refresh()
    assert native.task_submits[-1][-1] == 1


def test_named_task_facade_uses_the_protocols_200_byte_session_bounds():
    native = _FakeSession()
    task = _services.TaskSession(native, "n" * 200, task_session_id="s" * 200)
    assert task.name == "n" * 200
    assert task.task_session_id == "s" * 200
    with pytest.raises(ValueError, match="at most 200"):
        _services.TaskSession(native, "n" * 201)
    with pytest.raises(ValueError, match="at most 200"):
        _services.TaskSession(native, "name", task_session_id="s" * 201)


def test_rgbd_click_is_exactly_correlated_and_sends_no_pixel_arrays():
    native = _FakeSession()
    managed = _ManagedRig()
    measurement = _services.submit_calibration_click(
        native,
        managed,
        calibration_id="cal-1",
        sample_id="sample-7",
        camera="wrist",
        frame_sequence=7,
        x=1,
        y=1,
    )
    assert measurement.point == (0.02, 0.02, 2.0)
    assert native.measurements == [
        (
            "cal-1",
            "sample-7",
            "wrist",
            7,
            "cam_wrist",
            91,
            (0.02, 0.02, 2.0),
            2.0,
        )
    ]
    assert all(not isinstance(value, np.ndarray) for value in native.measurements[0])

    managed.sample = waddle_sdk.CameraSample(
        stamp=_Stamp(session_ns=92),
        frame_sequence=8,
        rgb=np.zeros((2, 2, 3), dtype=np.uint8),
        depth=np.ones((2, 2), dtype=np.uint16),
    )
    try:
        _services.submit_calibration_click(
            native,
            managed,
            calibration_id="cal-1",
            sample_id="sample-7",
            camera="wrist",
            frame_sequence=7,
            x=1,
            y=1,
        )
    except RuntimeError as exc:
        assert "no longer retained" in str(exc)
    else:
        raise AssertionError("a stale RGB-D correlation was accepted")


def test_artifact_facade_and_backend_discovery_are_lazy(monkeypatch):
    native = _FakeSession()
    artifact = _services.WorkspaceArtifactRequest(
        native, ["pick"], ["wrist-to-base"]
    )
    assert native.artifact_requests == [
        (artifact.request_id, ("pick",), ("wrist-to-base",))
    ]
    assert artifact.events()[0]["download_ref"] == "one-time-ref"

    loaded: list[str] = []

    class Entry:
        name = "simulator"

        def load(self):
            loaded.append(self.name)
            return object()

    class Entries(tuple):
        def select(self, *, group):
            assert group == "waddle.execution.v1"
            return self

    monkeypatch.setattr(
        _services.importlib.metadata, "entry_points", lambda: Entries((Entry(),))
    )
    hosted, local = _services.execution_backends()
    assert (hosted.label, local.label, local.name) == (
        "Hosted",
        "Local",
        "simulator",
    )
    assert loaded == []
    local.load()
    assert loaded == ["simulator"]


def test_ui_requires_handoff_and_serves_task_calibration_and_artifact_routes():
    native = _FakeSession()
    managed = _ManagedRig()
    handle = UIHandle(
        native,
        None,
        managed,
        joint_step_rad=0.1,
        linear_step_m=0.01,
        angular_step_rad=0.02,
    )
    try:
        with _open(
            handle,
            "/api/jog",
            method="POST",
            value={"kind": "joint", "index": 0, "direction": 1},
        ) as response:
            assert json.load(response)["code"] == "handoff_required"
        assert native.jogs == 0
        with _open(handle, "/api/handoff", method="POST", value={}) as response:
            assert json.load(response)["accepted"] is True
        with _open(
            handle,
            "/api/jog",
            method="POST",
            value={"kind": "joint", "index": 0, "direction": 1},
        ) as response:
            assert json.load(response)["accepted"] is True
        assert native.handoffs == 2 and native.jogs == 1

        with _open(
            handle,
            "/api/tasks/create",
            method="POST",
            value={"name": "bench setup"},
        ) as response:
            created = json.load(response)
        with _open(
            handle,
            f"/api/tasks/events?request_id={created['request_id']}",
        ) as response:
            task_events = json.load(response)
        assert task_events["task_session_id"] == "task-1"
        with _open(handle, "/api/tasks") as response:
            assert json.load(response)["tasks"][0]["history"][0]["text"] == "ready"

        with _open(handle, "/api/cameras/wrist") as response:
            assert response.headers["X-Waddle-Frame-Sequence"] == "7"
            assert response.read() == managed.sample.rgb.tobytes()
        with _open(
            handle,
            "/api/calibration/click",
            method="POST",
            value={
                "calibration_id": "cal-1",
                "sample_id": "sample-7",
                "camera": "wrist",
                "frame_sequence": 7,
                "x": 1,
                "y": 1,
            },
        ) as response:
            assert json.load(response)["measurement"]["depth_m"] == 2.0

        with _open(
            handle,
            "/api/artifacts",
            method="POST",
            value={"graph_ids": ["pick"], "calibration_names": []},
        ) as response:
            request_id = json.load(response)["request_id"]
        with _open(
            handle, f"/api/artifacts/events?request_id={request_id}"
        ) as response:
            assert json.load(response)["events"][0]["artifact_id"] == "artifact-1"
    finally:
        handle.close()
