"""Port Metal's pending-write journal assertions to the actual SDK callback."""

import json
import threading
import time

import pytest
from test_finite_playback import motion
from test_finite_playback import opened as opened  # noqa: PLC0414
from waddle_sdk.runtime import FaultCode, RuntimeFault


@pytest.mark.parametrize("fail_after_write", [False, True])
def test_history_retains_pending_write_and_original_error(
    opened, monkeypatch, fail_after_write
):
    session, run = opened
    driver = session._managed.arms["arm"].driver
    entered, release = threading.Event(), threading.Event()
    write = driver.write
    sent = []
    failure = RuntimeFault(
        FaultCode.MOTOR_FAILURE,
        "Command sent; reply unavailable",
        context={"bus": "test"},
    )

    def blocked(target):
        if not sent:
            entered.set()
            assert release.wait(5), "Test did not release the pending write"
        sent.append(tuple(target))
        write(target)
        if fail_after_write:
            raise failure

    monkeypatch.setattr(driver, "write", blocked)
    handle = run.start_playback(motion((0.0, 0.0), (0.02, -0.01)))
    try:
        assert entered.wait(3)
        frozen = handle.history()
        assert frozen["api"] == "waddle.playback-history/v1"
        assert frozen["clock"] == "sdk_session_monotonic"
        assert frozen["run_id"] == run.id
        assert frozen["execution_id"] == handle.snapshot()["id"]
        first = frozen["submissions"][0]
        assert frozen["started"] == first["started"]
        assert first["result"] == "pending"
        assert first["returned"] is None
        assert first["reference"]["offset_ns"] == 0
        assert first["measured"]["positions"] == [0.0, 0.0]
        assert first["measured"]["observed_ns"] <= first["started"]["session_ns"]
        assert first["started"]["session_ns"] <= frozen["sampled"]["session_ns"]
        assert first["started"]["unix_ns"] > 0
        assert (
            0
            < first["started"]["host_monotonic_ns"]
            <= frozen["sampled"]["host_monotonic_ns"]
            <= time.monotonic_ns()
        )
    finally:
        release.set()
        result = handle.wait()
    history = handle.history()
    json.dumps(history, allow_nan=False)
    assert frozen["submissions"][0]["result"] == "pending"
    assert history["submission_count"] == len(sent)
    assert history["dropped_submissions"] == 0
    for event, target in zip(history["submissions"], sent, strict=True):
        assert event["reference"]["positions"] == list(target)
        assert event["returned"]["session_ns"] >= event["started"]["session_ns"]
        assert event["returned"]["unix_ns"] > 0
        assert (
            event["started"]["host_monotonic_ns"]
            <= event["returned"]["host_monotonic_ns"]
            <= history["sampled"]["host_monotonic_ns"]
        )
    first = history["submissions"][0]
    if fail_after_write:
        assert first["result"] == "error"
        assert first["error"] == result["failure"]["evidence"] == failure.as_dict()
    else:
        assert result["status"] == "arrived", result
        assert all(row["result"] == "acknowledged" for row in history["submissions"])
        assert history["submissions"][-1]["reference"]["offset_ns"] > 0


def test_history_cancellation_finishes_pending_write_before_hold(opened, monkeypatch):
    session, run = opened
    driver = session._managed.arms["arm"].driver
    entered, release = threading.Event(), threading.Event()
    write = driver.write

    def blocked(target):
        entered.set()
        assert release.wait(5)
        write(target)

    monkeypatch.setattr(driver, "write", blocked)
    handle = run.start_playback(motion((0.0, 0.0), (0.02, -0.01)))
    try:
        assert entered.wait(3)
        handle.cancel("cancel pending call")
        assert handle.history()["submissions"][0]["result"] == "pending"
    finally:
        release.set()
        result = handle.wait()
    assert result["cleanup"]["outcome"] == "acknowledged"
    history = handle.history()
    assert history["submission_count"] == 1
    assert history["submissions"][0]["result"] == "acknowledged"
    assert (
        history["submissions"][0]["returned"]["session_ns"]
        <= result["cleanup"]["completed_ns"]
    )


def test_history_is_bounded_detached_and_retains_the_pending_tail(opened):
    from waddle_sdk._playback_history import CAPACITY, History

    session, run = opened
    stamp = session._managed.core.stamp
    history = History(run.id, {"arm": ["j0", "j1"]}, stamp)
    reference = {"part": "arm", "offset_ns": 0, "positions": [0.0, 0.0]}
    measured = {"observed_ns": 0, "positions": [0.0, 0.0], "velocities": [0.0, 0.0]}
    for index in range(CAPACITY + 17):
        sequence = history.begin(stamp(), reference, measured)
        if index < CAPACITY + 16:
            history.end(sequence, result="acknowledged", gate="pass")
    reference["positions"][0] = 99
    frozen = history.snapshot()
    assert frozen["dropped_submissions"] == 17
    assert len(frozen["submissions"]) == CAPACITY
    assert frozen["submissions"][0]["sequence"] == 18
    assert (
        frozen["started"]["session_ns"]
        < frozen["submissions"][0]["started"]["session_ns"]
    )
    assert frozen["submissions"][-1]["result"] == "pending"
    assert frozen["submissions"][-1]["reference"]["positions"] == [0.0, 0.0]
    frozen["submissions"].clear()
    assert len(history.snapshot()["submissions"]) == CAPACITY


def test_history_does_not_acknowledge_a_superseded_reference(opened):
    session, run = opened
    run.step([0, 0])
    native = session._managed.core
    native._testing_engage("history-takeover", "agent")
    deadline = time.monotonic() + 3
    while native.status()["gate_mode"] != "Intervention":
        assert time.monotonic() < deadline
        threading.Event().wait(0.005)
    handle = run.start_playback(motion())
    assert handle.wait()["failure"]["code"] == "superseded"
    event = handle.history()["submissions"][0]
    assert event["result"] == "superseded"
    assert event["gate"] != "pass"
