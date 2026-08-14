"""Generic task context: paired stamps and metadata persistence."""

from __future__ import annotations

import json

import pytest

import waddle_sdk


@pytest.fixture(autouse=True)
def _clean_session():
    yield
    waddle_sdk.shutdown()


def _robot() -> waddle_sdk.Robot:
    return waddle_sdk.Robot(
        name="task-context-bot",
        robot_id="task-context-01",
        cell_id="cell-task-context",
        action_space=waddle_sdk.JointSpace(joints=["j0", "j1"], rate_hz=50),
    )


def _control() -> waddle_sdk.Control:
    return waddle_sdk.Control(send=lambda chunk: None, hold=lambda: None)


def _sidecar(recording_dir, episode_id: str) -> dict:
    return json.loads(
        (recording_dir / f"{episode_id}.sidecar.json").read_text(encoding="utf-8")
    )


def test_session_stamp_is_one_immutable_paired_clock_read():
    session = waddle_sdk.init("stamp", _robot(), _control())

    first = session.stamp()
    second = session.stamp()

    assert isinstance(first, waddle_sdk.SessionStamp)
    assert second.session_ns >= first.session_ns >= 0
    assert second.unix_ns >= first.unix_ns
    assert first.unix_ns - first.session_ns == second.unix_ns - second.session_ns
    with pytest.raises(AttributeError):
        first.session_ns = 0


@pytest.mark.parametrize("outcome", ["success", "abort"])
def test_rollout_persists_metadata_for_every_terminal_outcome(tmp_path, outcome):
    waddle_sdk.init("metadata", _robot(), _control(), recording_dir=tmp_path)
    metadata = {"trace_id": f"trace-{outcome}", "workspace_digest": "sha256:abc"}

    with waddle_sdk.rollout("place the cup", task_metadata=metadata) as episode:
        episode_id = episode.id
        episode.gate([0.0, 0.0])
        episode.terminate(outcome)
    waddle_sdk.shutdown()

    sidecar = _sidecar(tmp_path, episode_id)
    assert sidecar["task"] == "place the cup"
    assert sidecar["taskMetadata"] == metadata


def test_failed_reset_still_persists_metadata(tmp_path):
    waddle_sdk.init(
        "metadata-reset-failure",
        _robot(),
        _control(),
        recording_dir=tmp_path,
        pre_reset=lambda task: False,
    )

    with pytest.raises(RuntimeError, match="reset failed"):
        with waddle_sdk.rollout(
            "reset must fail", task_metadata={"trace_id": "trace-reset-failure"}
        ):
            pass
    waddle_sdk.shutdown()

    sidecars = list(tmp_path.glob("*.sidecar.json"))
    assert len(sidecars) == 1
    sidecar = json.loads(sidecars[0].read_text(encoding="utf-8"))
    assert sidecar["taskMetadata"] == {"trace_id": "trace-reset-failure"}


def test_task_metadata_is_string_only_and_bounded(tmp_path):
    waddle_sdk.init("metadata-validation", _robot(), _control(), recording_dir=tmp_path)

    with pytest.raises(TypeError, match="strings to strings"):
        waddle_sdk.rollout("bad", task_metadata={"attempt": 1})
    with pytest.raises(ValueError, match="invalid task metadata"):
        waddle_sdk.rollout("bad", task_metadata={"x": "y" * 4097})

