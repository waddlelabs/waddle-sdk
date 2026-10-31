"""Actual native worker through Site/Run and a local fake hardware owner."""

import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace

import numpy as np
import pytest
import site_fixtures
from test_site_api import _write_site
from waddle_sdk import load_site
from waddle_sdk.ownership import SiteOwnershipError
from waddle_sdk.robots import base
from waddle_sdk.runtime import (
    FaultCode,
    FiniteJointMotion,
    FinitePlaybackRunPort,
    GripperForce,
    JointPositionCommand,
    JointReference,
    PlaybackPolicy,
    RuntimeFault,
    SupportFact,
)


@pytest.fixture
def opened(tmp_path):
    site_fixtures.reset()
    session = load_site(_write_site(tmp_path)).open(console=False, _testing=True)
    session.__enter__()
    run = session.begin_run(task="finite local references", actor="test")
    try:
        yield session, run
    finally:
        session.close(torque_release_authorized=True)  # fake hardware only
        site_fixtures.reset()


def motion(*targets, feedforward=False):
    return FiniteJointMotion(
        "arm",
        tuple(
            JointReference(
                i * 50_000_000,
                JointPositionCommand(q, (0.0, 0.0) if feedforward else None),
            )
            for i, q in enumerate(targets or ((0.0, 0.0),))
        ),
        PlaybackPolicy(
            (0, 1),
            (0.01, 0.01),
            (0.001, 0.001),
            (0.01, 0.01),
            40_000_000,
            200_000_000,
            2_000_000_000,
        ),
    )


def test_counted_settling_is_admitted_by_its_own_support_fact(opened, monkeypatch):
    session, run = opened
    request = motion((0, 0), (0.02, -0.01))
    request = replace(request, policy=replace(request.policy, minimum_settle_samples=3))
    result = run.start_playback(request).wait()
    assert result["status"] == "arrived", result
    support = session.support()
    monkeypatch.setattr(
        session,
        "support",
        lambda: replace(
            support,
            rows=tuple(
                replace(
                    row,
                    facts=tuple(
                        f for f in row.facts if f != SupportFact.SAMPLED_JOINT_PLAYBACK
                    ),
                )
                for row in support.rows
            ),
        ),
    )
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(request)
    assert caught.value.code == FaultCode.UNSUPPORTED


def test_reference_receipt_reports_native_ack_time_without_claiming_arrival(
    opened, monkeypatch
):
    session, run = opened
    driver = session._managed.arms["arm"].driver
    measured = np.zeros(2)

    def write(target):
        measured[:] = np.asarray(target) / 2

    monkeypatch.setattr(driver, "write", write)
    monkeypatch.setattr(driver, "read", lambda: (measured.copy(), np.full(2, 0.02)))
    request = motion((0, 0), (0.1, -0.05))
    request = replace(request, policy=replace(request.policy, reference_only=True))
    result = run.start_playback(request).wait()
    assert result["status"] == "references_dispatched", result
    assert result["dispatched"] == 2
    assert result["feedback"]["positions"] == pytest.approx([0.05, -0.025])
    assert result["feedback"]["velocities"] == pytest.approx([0.02, 0.02])
    assert 0 <= result["last_acknowledged_ns"] < result["feedback"]["observed_ns"]
    assert result["cleanup"] is None
    support = session.support()
    monkeypatch.setattr(
        session,
        "support",
        lambda: replace(
            support,
            rows=tuple(
                replace(
                    row,
                    facts=tuple(
                        f
                        for f in row.facts
                        if f != SupportFact.REFERENCE_JOINT_PLAYBACK
                    ),
                )
                for row in support.rows
            ),
        ),
    )
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(request)
    assert caught.value.code == FaultCode.UNSUPPORTED


def test_measured_completion_and_exact_feedforward_use_the_real_owner(opened):
    session, run = opened
    assert isinstance(run, FinitePlaybackRunPort)
    assert SupportFact.FINITE_JOINT_PLAYBACK in session.support().rows[0].facts
    handle = run.start_playback(motion((0, 0), (0.1, -0.05), feedforward=True))
    result = handle.wait()
    assert result["status"] == "arrived", result
    assert result["failure"] is None
    assert result["episode_id"] == run.id
    assert result["dispatched"] == result["references"] == 2
    assert result["feedback"]["positions"] == pytest.approx([0.1, -0.05])
    assert [q.tolist() for q, _ in site_fixtures.velocity_commands] == [
        [0, 0],
        [0.1, -0.05],
    ]
    assert all(v.tolist() == [0, 0] for _, v in site_fixtures.velocity_commands)
    assert handle.snapshot() == result


def test_checkpoint_evidence_survives_without_polling_and_requires_support(
    opened, monkeypatch
):
    session, run = opened
    observations = {}
    observe = session.observe_parts

    def record(parts):
        sample = observe(parts)
        observations[sample.session_ns] = sample.parts["arm"].joint_position.tolist()
        return sample

    monkeypatch.setattr(session, "observe_parts", record)
    request = motion((0, 0), (0.02, -0.01), (0.04, -0.02))
    request = replace(
        request,
        policy=replace(request.policy, checkpoint_offsets_ns=(50_000_000, 100_000_000)),
    )
    result = run.start_playback(request).wait()
    assert result["status"] == "arrived", result
    samples = result["checkpoints"]
    assert [s["offset_ns"] for s in samples] == list(
        request.policy.checkpoint_offsets_ns
    )
    for sample in samples:
        assert (
            sample["feedback"]["positions"]
            == observations[sample["feedback"]["observed_ns"]]
        )
        assert sample["acknowledged_ns"] < sample["feedback"]["observed_ns"]
        assert sample["acknowledged_offset_ns"] >= sample["offset_ns"]
    assert samples[0]["feedback"]["positions"] != result["feedback"]["positions"]
    support = session.support()
    monkeypatch.setattr(
        session,
        "support",
        lambda: replace(
            support,
            rows=tuple(
                replace(
                    row,
                    facts=tuple(
                        f
                        for f in row.facts
                        if f != SupportFact.CHECKPOINTED_JOINT_PLAYBACK
                    ),
                )
                for row in support.rows
            ),
        ),
    )
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(request)
    assert caught.value.code == FaultCode.UNSUPPORTED


def test_owner_refusal_retains_fault_and_never_dispatches_successor(opened):
    session, run = opened
    result = run.start_playback(motion((0, 0), (0.9, 0), (0.95, 0))).wait()
    assert result["status"] == "failed"
    assert result["dispatched"] == 1
    failure = result["failure"]
    assert failure["code"] == "dispatch"
    assert failure["outcome_unknown"] is False
    assert failure["evidence"]["code"] == "safety_refusal"
    assert failure["evidence"]["context"]["part"] == "arm"
    assert session._managed.rejected == 1


def test_stale_start_never_writes(opened):
    session, run = opened
    result = run.start_playback(motion((0.5, 0))).wait()
    assert result["failure"]["code"] == "stale_start"
    assert result["dispatched"] == 0
    assert session._managed.accepted == 0


@pytest.mark.parametrize(
    "bad",
    [
        lambda m: replace(m, references=()),
        lambda m: replace(m, references=(replace(m.references[0], offset_ns=1),)),
        lambda m: replace(m, policy=replace(m.policy, indices=(0, 0))),
        lambda m: replace(m, policy=replace(m.policy, timeout_ns=1)),
        lambda m: replace(
            m, references=(JointReference(0, JointPositionCommand((0,))),)
        ),
    ],
)
def test_native_admission_rejects_invalid_buffers_before_owner_io(opened, bad):
    session, run = opened
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(bad(motion()))
    assert caught.value.code == FaultCode.INVALID_REQUEST
    assert session._managed.accepted == session._managed.rejected == 0


def test_dispatch_exception_retains_structured_uncertainty_without_retry(
    opened, monkeypatch
):
    session, run = opened
    calls = []
    original = RuntimeFault(
        FaultCode.MOTOR_FAILURE,
        "motor 4 feedback lost",
        context={"motor": 4, "can_error": "bus-off"},
    )

    def write(_target):
        calls.append("write")
        raise original

    monkeypatch.setattr(session._managed.arms["arm"].driver, "write", write)
    result = run.start_playback(motion((0, 0), (0.1, 0))).wait()
    failure = result["failure"]
    assert calls == ["write"]
    assert result["dispatched"] == 0
    assert failure["code"] == "dispatch"
    assert failure["outcome_unknown"] is True
    assert failure["evidence"] == original.as_dict()


def test_observation_failure_is_not_fabricated_as_arrival(opened, monkeypatch):
    session, run = opened
    original = RuntimeFault(
        FaultCode.MOTOR_FAILURE, "encoder unavailable", context={"motor": 2}
    )

    def read(*_args):
        raise original

    monkeypatch.setattr(session, "observe_parts", read)
    result = run.start_playback(motion()).wait()
    assert result["failure"]["code"] == "observation"
    assert result["failure"]["evidence"] == original.as_dict()
    assert result["dispatched"] == 0


def test_cancel_serializes_writes_and_old_handle_cannot_stop_successor(
    opened, monkeypatch
):
    _, run = opened
    entered, release = threading.Event(), threading.Event()
    original = base.apply_decision

    def blocked(*args, **kwargs):
        entered.set()
        assert release.wait(5), "test did not release owner callback"
        return original(*args, **kwargs)

    monkeypatch.setattr(base, "apply_decision", blocked)
    handle = run.start_playback(motion((0, 0), (0.1, 0)))
    try:
        assert entered.wait(3)
        for submit in (
            lambda: run.step([0, 0]),
            lambda: run.step_parts({"arm": JointPositionCommand((0, 0))}),
            lambda: run.start_playback(motion()),
        ):
            with pytest.raises(RuntimeFault) as caught:
                submit()
            assert caught.value.code == FaultCode.BUSY
        handle.cancel("test local cancellation")
        assert handle.snapshot()["status"] == "running"
    finally:
        release.set()
    result = handle.wait()
    assert result["failure"]["code"] == "cancelled"
    assert result["dispatched"] == 1
    successor = run.start_playback(motion())
    handle.cancel("stale handle")
    assert successor.wait()["status"] == "arrived"
    assert handle.snapshot() == result


def test_close_drains_inflight_write_before_driver_close(opened, monkeypatch):
    session, run = opened
    entered, release, closing = threading.Event(), threading.Event(), threading.Event()
    driver = session._managed.arms["arm"].driver
    original_write = driver.write
    order = []

    def write(target):
        entered.set()
        assert release.wait(5)
        original_write(target)
        order.append("write finished")

    monkeypatch.setattr(driver, "write", write)
    monkeypatch.setattr(driver, "close", lambda: order.append("closed"))
    handle = run.start_playback(motion((0, 0), (0.1, 0)))

    def close():
        closing.set()
        session.close(torque_release_authorized=True)

    with ThreadPoolExecutor(1) as pool:
        assert entered.wait(3)
        future = pool.submit(close)
        try:
            assert closing.wait(3)
            assert "closed" not in order
        finally:
            release.set()
        future.result(timeout=5)
    assert order == ["write finished", "closed"]
    assert handle.snapshot()["status"] == "failed"
    assert handle.snapshot()["dispatched"] <= 1


def test_stationary_miss_requires_explicit_settled_policy(opened, monkeypatch):
    session, run = opened
    # An admitted command is not evidence of arrival: this fake actuator never moves.
    monkeypatch.setattr(session._managed.arms["arm"].driver, "write", lambda _q: None)
    strict = motion((0, 0), (0.1, 0))
    strict = replace(strict, policy=replace(strict.policy, timeout_ns=250_000_000))
    assert run.start_playback(strict).wait()["failure"]["code"] == "deadline"
    permitted = replace(strict, policy=replace(strict.policy, allow_settled=True))
    result = run.start_playback(permitted).wait()
    assert result["status"] == "settled"
    assert result["feedback"]["positions"] == [0, 0]


def test_force_modifiers_are_explicitly_unsupported(opened):
    _, run = opened
    request = replace(
        motion(),
        references=(
            JointReference(0, JointPositionCommand((0, 0), gripper_force_n=5)),
        ),
    )
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(request)
    assert caught.value.code == FaultCode.UNSUPPORTED


def test_native_takeover_ends_buffer_without_acknowledging_a_noop(opened):
    session, run = opened
    run.step([0, 0])
    native = session._managed.core
    native._testing_engage("finite-takeover", "agent")
    deadline = time.monotonic() + 3
    while native.status()["gate_mode"] != "Intervention":
        assert time.monotonic() < deadline, "native claim did not engage"
        threading.Event().wait(0.005)
    accepted = session._managed.accepted
    result = run.start_playback(motion()).wait()
    assert result["status"] == "failed"
    assert result["failure"]["code"] == "superseded"
    assert result["dispatched"] == 0
    assert session._managed.accepted == accepted


def test_held_jaw_mode_and_complete_command_survive_arm_only_completion(
    opened, monkeypatch
):
    session, run = opened
    driver = session._managed.arms["arm"].driver
    original_write, original_hold = driver.write, driver.hold
    force = [0]
    force_writes, targets = [], []

    def write(target):
        targets.append(list(target))
        original_write([target[0], 0])  # jaw cannot reach the requested position

    def force_write(target, force_n, velocity=None):
        force_writes.append(force_n)
        force[0] = force_n
        write(target)

    def hold():
        force[0] = 0
        original_hold()

    monkeypatch.setattr(driver, "gripper_force_supported", True, raising=False)
    monkeypatch.setattr(driver, "gripper_force_limits_n", (1, 10), raising=False)
    monkeypatch.setattr(
        driver,
        "read_gripper_force",
        lambda: GripperForce(force[0], "test.sensor", False),
        raising=False,
    )
    monkeypatch.setattr(driver, "write_gripper_force", force_write, raising=False)
    monkeypatch.setattr(driver, "write", write)
    monkeypatch.setattr(driver, "hold", hold)
    receipt = run.step_parts({"arm": JointPositionCommand((0, 0), gripper_force_n=5)})
    assert receipt["arm"].dispatched
    request = motion((0, 0.1), (0.1, 0.1))
    request = replace(
        request,
        policy=replace(
            request.policy,
            indices=(0,),
            start_tolerance=(0.01,),
            position_tolerance=(0.001,),
            stationary_velocity=(0.01,),
        ),
    )
    result = run.start_playback(request).wait()
    assert result["status"] == "arrived", result
    assert result["feedback"]["positions"] == pytest.approx([0.1, 0])
    assert targets[-2:] == [[0, 0.1], [0.1, 0.1]]
    assert force_writes == [5]
    assert force == [5]


def test_missing_part_support_refuses_before_dispatch(opened):
    session, run = opened
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(replace(motion(), part="absent"))
    assert caught.value.code == FaultCode.UNSUPPORTED
    assert session._managed.accepted == session._managed.rejected == 0


def test_unconfirmed_run_drain_keeps_devices_and_site_ownership(opened, monkeypatch):
    session, run = opened
    entered, release = threading.Event(), threading.Event()
    episode = run._episode
    original = base.apply_decision

    def blocked(*args, **kwargs):
        entered.set()
        assert release.wait(5)
        return original(*args, **kwargs)

    class UnconfirmedDrain:
        def __getattr__(self, name):
            return getattr(episode, name)

        def terminate(self, *_args):
            raise RuntimeError("injected unconfirmed native drain")

    monkeypatch.setattr(base, "apply_decision", blocked)
    handle = run.start_playback(motion((0, 0), (0.1, 0)))
    try:
        assert entered.wait(3)
        run._episode = UnconfirmedDrain()
        with pytest.raises(SiteOwnershipError, match="Finite playback did not drain"):
            session.close(torque_release_authorized=True)
        assert session._active is run
        assert session._ownership is not None
        assert session._teardown_failed
        assert site_fixtures.closed == {"arms": 0, "cameras": 0}
    finally:
        handle.cancel("test cleanup")
        release.set()
        handle.wait()
        run._episode = episode
        # Test-only recovery after independently confirming the fake callback
        # has drained. Production callers cannot clear uncertain ownership.
        session._teardown_failed = False


@pytest.mark.parametrize("estop", [False, True])
def test_completed_stop_after_terminal_reference_only_motion(opened, estop):
    session, run = opened
    request = motion((0, 0), (0.02, -0.01))
    request = replace(request, policy=replace(request.policy, reference_only=True))
    handle = run.start_playback(request)
    original = handle.wait()
    assert original["status"] == "references_dispatched"
    assert original["cleanup"] is None
    receipt = session.stop("stop at held endpoint", estop=estop)
    assert receipt["request"] == ("estop" if estop else "hold")
    assert receipt["outcome"] == "acknowledged", receipt
    assert receipt["completed_ns"] > original["last_acknowledged_ns"]
    assert receipt["measured_stationary"] is False
    assert handle.snapshot() == original
    assert SupportFact.COMPLETED_STOP in session.support().rows[0].facts
