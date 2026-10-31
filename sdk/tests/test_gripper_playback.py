"""Measured jaw outcomes through the native worker and ordinary owner gate."""

import threading
from dataclasses import replace
from uuid import uuid4

import numpy as np
import pytest
import site_fixtures
from test_site_api import _write_site
from waddle_sdk import load_site
from waddle_sdk.runtime import (
    FaultCode,
    FiniteJointMotion,
    GripperContactPolicy,
    GripperForce,
    GripperForcePolicy,
    GripperPlaybackPolicy,
    JointPositionCommand,
    JointReference,
    PlaybackPolicy,
    RuntimeFault,
    SupportFact,
)


@pytest.fixture
def jaw(tmp_path, monkeypatch):
    path = _write_site(tmp_path)
    path.write_text(path.read_text().replace("id: test-cell", f"id: jaw-{uuid4().hex}"))
    path.write_text(
        path.read_text().replace(
            "    posture: supervised",
            "    gripper: {joint: j1, closed_m: 0, open_m: 0.08, "
            "closed_action: 0, open_action: 1}\n    posture: supervised",
        )
    )
    session = load_site(path).open(console=False, _testing=True)
    session.__enter__()
    run = session.begin_run(task="measured jaw", actor="test")
    driver = session._managed.arms["arm"].driver
    state = {"q": 0.8, "stop": 0.3, "force": 0.0, "missing": False}
    writes = []
    lock = threading.Lock()

    def read():
        with lock:
            return np.array([0.0, state["q"]]), np.zeros(2)

    def write(target):
        with lock:
            writes.append(list(target))
            state["q"] = max(state["stop"], float(target[1]))

    def force_write(target, force_n, velocity=None):
        state["force"] = force_n
        write(target)

    def read_force():
        if state["missing"]:
            return None
        return GripperForce(
            state["force"] if state["q"] <= state["stop"] else 0.0,
            "test.jaw_sensor",
            False,
        )

    def hold():
        state["force"] = 0

    monkeypatch.setattr(driver, "read", read)
    monkeypatch.setattr(driver, "write", write)
    monkeypatch.setattr(driver, "hold", hold)
    monkeypatch.setattr(driver, "gripper_force_supported", True, raising=False)
    monkeypatch.setattr(driver, "gripper_force_limits_n", (1, 10), raising=False)
    monkeypatch.setattr(driver, "read_gripper_force", read_force, raising=False)
    monkeypatch.setattr(driver, "write_gripper_force", force_write, raising=False)
    try:
        yield session, run, driver, state, writes
    finally:
        session.close(torque_release_authorized=True)  # fake owner only
        site_fixtures.reset()


def motion(start=0.8, goal=0.0, *, contact=False, force=None):
    return FiniteJointMotion(
        "arm",
        tuple(
            JointReference(i * 50_000_000, JointPositionCommand((0.0, q)))
            for i, q in enumerate((start, goal))
        ),
        PlaybackPolicy(
            (1,),
            (0.01,),
            (0.001,),
            (0.01,),
            50_000_000,
            200_000_000,
            2_000_000_000,
            gripper=GripperPlaybackPolicy(
                50_000_000,
                0.2,
                0.2,
                0,
                1,
                0,
                0.08,
                contact=GripperContactPolicy(
                    0.0025,
                    0.002,
                    0.0005,
                    0.0005,
                    0.0002,
                    250_000_000,
                )
                if contact
                else None,
                force=GripperForcePolicy(force, 0.5, 0.0005)
                if force is not None
                else None,
            ),
        ),
    )


def test_contact_requires_measured_closure_stall_and_continued_demand(jaw):
    session, run, _, state, writes = jaw
    assert SupportFact.GRIPPER_PLAYBACK in session.support().rows[0].facts
    result = run.start_playback(motion(contact=True)).wait()
    assert result["status"] == "contact_stall", result
    assert result["failure"] is None
    assert result["gripper"]["progress_m"] == pytest.approx(0.04)
    assert result["gripper"]["window_ns"] >= 250_000_000
    assert result["gripper"]["commanded_shortfall_m"] > 0.0005
    assert result["feedback"]["positions"][1] == state["q"] == 0.3
    assert writes[-1][1] < 0.3


def test_force_completion_retains_hold_until_an_explicit_zero_force_open(jaw):
    _, run, _, state, _ = jaw
    result = run.start_playback(motion(force=5)).wait()
    assert result["status"] == "force_reached", result
    assert result["gripper"]["force_samples"] == 3
    assert result["feedback"]["gripper_force"] == {
        "force_n": 5.0,
        "source": "test.jaw_sensor",
        "estimated": False,
    }
    assert state["force"] == 5
    opened = run.start_playback(motion(start=0.3, goal=0.8, force=0)).wait()
    assert opened["status"] == "arrived", opened
    assert opened["gripper"]["arrival_samples"] == 3
    assert state["q"] == 0.8 and state["force"] == 0


@pytest.mark.parametrize("missing", [False, True])
def test_empty_stop_and_missing_force_are_distinct_failures(jaw, missing):
    _, run, _, state, _ = jaw
    state["stop"] = 0
    state["missing"] = missing
    result = run.start_playback(motion(force=5)).wait()
    assert result["status"] == "failed", result
    assert result["failure"]["code"] == (
        "gripper_force_unavailable" if missing else "gripper_empty"
    )
    assert result["failure"]["outcome_unknown"] is False
    assert state["force"] == 0  # existing native failure Hold


def test_positional_stall_cannot_claim_contact_or_force(jaw):
    _, run, _, state, _ = jaw
    request = motion()
    request = replace(request, policy=replace(request.policy, timeout_ns=700_000_000))
    result = run.start_playback(request).wait()
    assert result["status"] == "failed", result
    assert result["failure"]["code"] == "deadline"
    assert state["q"] == 0.3


@pytest.mark.parametrize(
    "change",
    [
        lambda p: replace(p, indices=(0,)),
        lambda p: replace(p, gripper=replace(p.gripper, open_m=0.09)),
        lambda p: replace(p, gripper=replace(p.gripper, period_ns=1_000_000)),
    ],
)
def test_wrong_jaw_mapping_and_excess_cadence_refuse_before_write(jaw, change):
    _, run, _, _, writes = jaw
    request = motion()
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(replace(request, policy=change(request.policy)))
    assert caught.value.code == FaultCode.INVALID_REQUEST
    assert writes == []


def test_force_requires_declared_capability_before_io(jaw, monkeypatch):
    _, run, driver, _, writes = jaw
    monkeypatch.setattr(driver, "gripper_force_supported", False)
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(motion(force=5))
    assert caught.value.code == FaultCode.UNSUPPORTED
    assert writes == []


def test_force_request_still_crosses_the_ordinary_owner_force_limit(jaw):
    _, run, _, state, writes = jaw
    result = run.start_playback(motion(force=11)).wait()
    assert result["status"] == "failed", result
    assert result["failure"]["code"] == "dispatch"
    # The ordinary owner raises this invalid-request fault rather than returning
    # a refusal receipt. Playback conservatively retains raised-write uncertainty.
    assert result["failure"]["outcome_unknown"] is True
    assert result["failure"]["evidence"]["code"] == "invalid_request"
    assert result["failure"]["evidence"]["detail"] == "gripper.force_range"
    assert result["failure"]["evidence"]["context"]["limits_n"] == [1, 10]
    assert result["dispatched"] == 0
    assert writes == [] and state["force"] == 0


def test_unknown_force_write_is_not_retried_or_reported_as_contact(jaw, monkeypatch):
    _, run, driver, state, _ = jaw
    original = RuntimeFault(FaultCode.MOTOR_FAILURE, "lost force write acknowledgement")
    calls = []

    def write(target, force_n, velocity=None):
        calls.append(force_n)
        raise original

    monkeypatch.setattr(driver, "write_gripper_force", write)
    result = run.start_playback(motion(force=5)).wait()
    assert result["status"] == "failed"
    assert result["failure"]["code"] == "dispatch"
    assert result["failure"]["outcome_unknown"] is True
    assert result["failure"]["evidence"] == original.as_dict()
    assert calls == [5] and state["force"] == 0
