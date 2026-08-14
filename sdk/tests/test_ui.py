"""The in-process authenticated loopback UI and its native safety seams."""

from __future__ import annotations

import json
import threading
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np
import pytest

import waddle_sdk
from waddle_sdk.robots import base


@pytest.fixture(autouse=True)
def _clean_session():
    yield
    waddle_sdk.shutdown()


def _robot(*, camera: bool = False) -> waddle_sdk.Robot:
    return waddle_sdk.Robot(
        name="ui-bot",
        robot_id="ui-01",
        cell_id="ui-cell",
        action_space=waddle_sdk.JointSpace(joints=["j0", "j1"], rate_hz=20),
        cameras=(
            {"overhead": waddle_sdk.Camera(width=2, height=2, fps=10)}
            if camera
            else {}
        ),
    )


def _open(
    handle: waddle_sdk.UIHandle,
    path: str,
    *,
    method: str = "GET",
    value: object | None = None,
    token: str | None = None,
    origin: str | None = None,
    host: str | None = None,
):
    parsed = urllib.parse.urlsplit(handle.url)
    secret = urllib.parse.parse_qs(parsed.fragment)["token"][0]
    headers = {
        "X-Waddle-Token": secret if token is None else token,
        "X-Waddle-Request": "1",
    }
    if host is not None:
        headers["Host"] = host
    if method == "POST":
        headers["Origin"] = (
            f"{parsed.scheme}://{parsed.netloc}" if origin is None else origin
        )
        headers["Content-Type"] = "application/json"
    data = None if value is None else json.dumps(value).encode()
    request = urllib.request.Request(
        f"{parsed.scheme}://{parsed.netloc}{path}",
        data=data,
        headers=headers,
        method=method,
    )
    return urllib.request.urlopen(request, timeout=5)


def test_ui_requires_init_validates_increments_and_is_one_per_session():
    with pytest.raises(RuntimeError, match="requires an active"):
        waddle_sdk.ui()
    waddle_sdk.init("ui", _robot(), waddle_sdk.Control())
    with pytest.raises(TypeError):
        waddle_sdk.ui(joint_step_rad=True)
    with pytest.raises(ValueError):
        waddle_sdk.ui(linear_step_m=float("nan"))
    handle = waddle_sdk.ui()
    parsed = urllib.parse.urlsplit(handle.url)
    assert parsed.hostname == "127.0.0.1"
    assert parsed.port
    assert len(urllib.parse.parse_qs(parsed.fragment)["token"][0]) >= 43
    assert waddle_sdk.ui() is handle


def test_loopback_security_headers_auth_host_origin_and_shutdown():
    session = waddle_sdk.init("ui", _robot(), waddle_sdk.Control())
    handle = waddle_sdk.ui()
    parsed = urllib.parse.urlsplit(handle.url)
    with urllib.request.urlopen(
        f"{parsed.scheme}://{parsed.netloc}/",
        timeout=5,
    ) as response:
        assert response.status == 200
        assert response.headers["Cache-Control"] == "no-store"
        assert response.headers["Referrer-Policy"] == "no-referrer"
        assert "default-src 'self'" in response.headers["Content-Security-Policy"]
        assert response.headers.get("Access-Control-Allow-Origin") is None

    with pytest.raises(urllib.error.HTTPError) as missing:
        _open(handle, "/api/state", token="")
    assert missing.value.code == 401
    with pytest.raises(urllib.error.HTTPError) as wrong_host:
        _open(handle, "/api/state", host="localhost")
    assert wrong_host.value.code == 403
    with pytest.raises(urllib.error.HTTPError) as wrong_origin:
        _open(
            handle,
            "/api/estop",
            method="POST",
            value={},
            origin="http://example.test",
        )
    assert wrong_origin.value.code == 403

    with _open(handle, "/api/state") as response:
        state = json.load(response)
    assert state["episode_state"] == session.status()["episode_state"]
    assert state["local_controls_available"] is True
    waddle_sdk.shutdown()
    assert handle.closed


def test_estop_is_a_local_requested_operation_and_latest_camera_is_raw_rgb():
    estopped = threading.Event()
    session = waddle_sdk.init(
        "ui",
        _robot(camera=True),
        waddle_sdk.Control(estop=estopped.set),
    )
    handle = waddle_sdk.ui()
    with _open(handle, "/api/estop", method="POST", value={}) as response:
        assert json.load(response) == {"status": "requested"}
    assert estopped.wait(2), "local e-stop callback was not invoked"

    first = np.arange(12, dtype=np.uint8).reshape((2, 2, 3))
    second = np.full((2, 2, 3), 19, dtype=np.uint8)
    session.publish_frame("overhead", first)
    session.publish_frame("overhead", second)
    with _open(handle, "/api/cameras/overhead") as response:
        assert response.headers["X-Waddle-Width"] == "2"
        assert response.headers["X-Waddle-Height"] == "2"
        assert response.headers["X-Waddle-Pixel-Format"] == "RGB8"
        assert response.read() == second.tobytes()


def test_jog_uses_native_proprio_claim_and_normal_send_path():
    sent = threading.Event()
    chunks = []

    def send(chunk):
        chunks.append(chunk)
        sent.set()

    session = waddle_sdk.init(
        "ui",
        _robot(),
        waddle_sdk.Control(send=send, hold=lambda: None, resume=lambda: None),
    )
    handle = waddle_sdk.ui(joint_step_rad=0.125)
    with waddle_sdk.rollout(task="jog test") as episode:
        episode.gate([0.0, 0.0], [0.0, 0.0])
        session.report_proprio(joint_pos=[0.5, -0.25])
        with _open(handle, "/api/handoff", method="POST", value={}) as response:
            assert json.load(response)["accepted"] is True
        with _open(
            handle,
            "/api/jog",
            method="POST",
            value={"kind": "joint", "index": 0, "direction": 1, "part": None},
        ) as response:
            result = json.load(response)
        assert result["accepted"] is True
        assert sent.wait(2), "accepted jog never reached Control.send"
        values = chunks[-1].steps[0][0]
        assert list(values) == pytest.approx([0.625, -0.25])
        with _open(handle, "/api/jog/heartbeat", method="POST", value={}) as response:
            assert json.load(response)["accepted"] is True
        with _open(handle, "/api/jog/release", method="POST", value={}) as response:
            assert json.load(response) == {"released": True}
        episode.terminate("abort", "test complete")


def test_jog_reaches_the_owner_envelope_and_is_refused_whole():
    refused = threading.Event()
    driver = base.SimDriver(
        [0.5, -0.25],
        lower=[-1.0, -1.0],
        upper=[1.0, 1.0],
        step_caps=[0.05, 0.05],
        rate_hz=20.0,
    )
    arm = base.Arm(
        part="robot",
        driver=driver,
        joint_names=("j0", "j1"),
        joint_limits=((-1.0, 1.0), (-1.0, 1.0)),
        step_caps=(0.05, 0.05),
        rate_hz=20.0,
        report=lambda _line: refused.set(),
    )
    arms = {"robot": arm}
    session = waddle_sdk.init(
        "ui",
        _robot(),
        waddle_sdk.Control(
            send=base.chunk_sender(arms),
            hold=lambda: base.hold_all(arms),
            resume=lambda: None,
        ),
    )
    handle = waddle_sdk.ui(joint_step_rad=0.125)
    with waddle_sdk.rollout(task="envelope refusal") as episode:
        episode.gate([0.0, 0.0], [0.0, 0.0])
        session.report_proprio(joint_pos=[0.5, -0.25])
        with _open(handle, "/api/handoff", method="POST", value={}) as response:
            assert json.load(response)["accepted"] is True
        with _open(
            handle,
            "/api/jog",
            method="POST",
            value={"kind": "joint", "index": 0, "direction": 1, "part": None},
        ) as response:
            assert json.load(response)["accepted"] is True
        assert refused.wait(2), "accepted jog never reached the owner's envelope"
        assert arm.rejected == 1 and arm.accepted == 0
        assert list(driver.read()[0]) == pytest.approx([0.5, -0.25])
        episode.terminate("abort", "test complete")


def test_manifest_downloads_are_resolved_beneath_recording_root(tmp_path: Path):
    root = tmp_path / "recordings"
    root.mkdir()
    safe_sidecar = root / "safe.sidecar.json"
    safe_sidecar.write_text(
        json.dumps({"episodeId": "safe", "tEndUnixNs": "20"}),
        encoding="utf-8",
    )
    safe_mcap = root / "safe.mcap"
    safe_mcap.write_bytes(b"mcap-data")
    outside = tmp_path / "evil.sidecar.json"
    outside.write_text("{}", encoding="utf-8")
    rows = [
        {
            "episodeId": "evil",
            "outcome": "TERMINAL_OUTCOME_FAILURE",
            "task": "escape",
            "tStartUnixNs": "1",
            "path": str(outside),
        },
        {
            "episodeId": "safe",
            "outcome": "TERMINAL_OUTCOME_SUCCESS",
            "task": "inside",
            "tStartUnixNs": "10",
            "path": str(safe_sidecar),
        },
    ]
    (root / "manifest.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
    )
    waddle_sdk.init("ui", _robot(), waddle_sdk.Control(), recording_dir=root)
    handle = waddle_sdk.ui()
    with _open(handle, "/api/recordings") as response:
        recordings = json.load(response)["recordings"]
    assert [item["episode_id"] for item in recordings] == ["safe"]
    assert recordings[0]["t_end_unix_ns"] == "20"
    entry = recordings[0]["entry"]
    with _open(handle, f"/api/recordings/download?entry={entry}&kind=mcap") as response:
        assert response.read() == b"mcap-data"
    with pytest.raises(urllib.error.HTTPError) as missing:
        _open(handle, "/api/recordings/download?entry=0&kind=sidecar")
    assert missing.value.code == 404
