"""Whole-site references through the actual native worker and mock owners."""

import threading
from dataclasses import replace

import pytest
from test_named_parts import DRIVERS
from test_named_parts import site_path as site_path  # noqa: PLC0414 -- pytest fixture
from waddle_sdk import load_site
from waddle_sdk.runtime import (
    CoordinatedJointMotion,
    FaultCode,
    JointPositionCommand,
    JointReference,
    PacedJointCurve,
    PlaybackPolicy,
    RuntimeFault,
    SupportFact,
)
from waddle_sdk_relay.wire import decode, encode


def request(*, paced=False):
    target = (0.04, -0.02, -0.03, 0.01)
    curve = (
        PacedJointCurve(
            (0.0, 0.2),
            ((0.0,) * 4, target),
            (
                tuple(
                    tuple(scale * q for q in target) for scale in (0, 0, 0, 10, -15, 6)
                ),
            ),
            50_000_000,
            (0.2,) * 4,
            (0.2,) * 4,
        )
        if paced
        else None
    )
    return CoordinatedJointMotion(
        ("left", "right"),
        (
            JointReference(0, JointPositionCommand((0.0,) * 4)),
            JointReference(200_000_000, JointPositionCommand(target)),
        ),
        PlaybackPolicy(
            (0, 1, 2, 3),
            (0.01,) * 4,
            (0.001,) * 4,
            (0.01,) * 4,
            40_000_000,
            200_000_000,
            2_000_000_000,
            paced_curve=curve,
            minimum_settle_samples=3,
        ),
    )


@pytest.mark.parametrize("paced", [False, True])
def test_all_parts_share_one_buffer_and_measured_completion(
    site_path, monkeypatch, paced
):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="group", actor="test") as run,
    ):
        writes = {name: [] for name in DRIVERS}
        for name, driver in DRIVERS.items():
            write = driver.write

            def record(target, *, name=name, write=write):
                writes[name].append(tuple(target))
                write(target)

            monkeypatch.setattr(driver, "write", record)
        assert all(
            SupportFact.COORDINATED_JOINT_PLAYBACK in row.facts
            for row in sdk.support().rows
        )
        motion = request(paced=paced)
        assert decode(encode(motion)) == motion
        result = run.start_playback(decode(encode(motion))).wait()
        assert result["status"] == "arrived", result
        assert result["feedback"]["positions"] == pytest.approx(
            motion.references[-1].command.positions
        )
        assert result["last_reference"]["part"] is None
        assert len(writes["left"]) == len(writes["right"]) == result["dispatched"]
        assert result["dispatched"] >= 2
        assert result["cleanup"] is None
        for left, right in zip(writes["left"], writes["right"], strict=True):
            # The common curve phase reaches both parts, with their own direction/scale.
            assert right == pytest.approx((-0.75 * left[0], -0.5 * left[1]))


@pytest.mark.parametrize(
    "invalid", ["order", "missing", "duplicate", "width", "support", "cadence"]
)
def test_invalid_group_refuses_before_device_io(site_path, monkeypatch, invalid):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="admission", actor="test") as run,
    ):
        motion = request(paced=True)
        if invalid in {"order", "missing", "duplicate"}:
            motion = replace(
                motion,
                parts={
                    "order": ("right", "left"),
                    "missing": ("left",),
                    "duplicate": ("left", "left"),
                }[invalid],
            )
        elif invalid == "width":
            motion = replace(
                motion, references=(JointReference(0, JointPositionCommand((0, 0, 0))),)
            )
        elif invalid == "support":
            support = sdk.support()
            monkeypatch.setattr(
                sdk,
                "support",
                lambda: replace(
                    support,
                    rows=tuple(
                        replace(
                            row,
                            facts=tuple(
                                f
                                for f in row.facts
                                if f != SupportFact.COORDINATED_JOINT_PLAYBACK
                            ),
                        )
                        if row.scope == "robot:right"
                        else row
                        for row in support.rows
                    ),
                ),
            )
        else:
            motion = replace(
                motion,
                policy=replace(
                    motion.policy,
                    paced_curve=replace(motion.policy.paced_curve, period_ns=1_000_000),
                ),
            )
        with pytest.raises(RuntimeFault) as caught:
            run.start_playback(motion)
        assert caught.value.code == (
            FaultCode.UNSUPPORTED if invalid == "support" else FaultCode.INVALID_REQUEST
        )
        assert sdk._managed.accepted == 0


def test_second_part_envelope_refusal_prevents_the_entire_reference(
    site_path, monkeypatch
):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="envelope", actor="test") as run,
    ):
        writes = []
        for name, driver in DRIVERS.items():
            write = driver.write

            def record(target, *, name=name, write=write):
                writes.append((name, tuple(target)))
                write(target)

            monkeypatch.setattr(driver, "write", record)
        motion = request()
        # Start is valid for both parts; the right target exceeds its native owner bound.
        motion = replace(
            motion,
            references=(
                motion.references[0],
                JointReference(200_000_000, JointPositionCommand((0.04, 0, 4.0, 0))),
            ),
        )
        result = run.start_playback(motion).wait()
        assert result["status"] == "failed", result
        assert result["failure"]["evidence"]["code"] == FaultCode.SAFETY_REFUSAL
        assert result["failure"]["evidence"]["context"]["part"] == "right"
        assert writes == [("left", (0, 0)), ("right", (0, 0))]
        assert result["cleanup"]["error"] is None, result


def test_cancel_drains_the_whole_inflight_reference_before_hold(site_path, monkeypatch):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="cancel", actor="test") as run,
    ):
        entered, release = threading.Event(), threading.Event()
        events = []
        for name, driver in DRIVERS.items():
            write, hold = driver.write, driver.hold

            def record(target, *, name=name, write=write):
                events.append(f"{name}:write")
                if name == "left":
                    entered.set()
                    assert release.wait(5)
                write(target)

            def stop(*, name=name, hold=hold):
                events.append(f"{name}:hold")
                hold()

            monkeypatch.setattr(driver, "write", record)
            monkeypatch.setattr(driver, "hold", stop)
        handle = run.start_playback(request())
        try:
            assert entered.wait(5)
            with pytest.raises(RuntimeFault) as caught:
                run.step_parts({"right": JointPositionCommand((0, 0))})
            assert caught.value.code == FaultCode.BUSY
            handle.cancel("cancel coordinated reference")
            assert handle.snapshot()["status"] == "running"
            assert events == ["left:write"]
        finally:
            release.set()
        result = handle.wait()
        assert result["status"] == "failed", result
        assert "cancel coordinated reference" in result["failure"]["detail"]
        assert result["cleanup"]["error"] is None, result
        assert events == ["left:write", "right:write", "left:hold", "right:hold"]


def test_partial_driver_failure_retains_uncertainty_and_holds_both_parts(
    site_path, monkeypatch
):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="failure", actor="test") as run,
    ):
        holds = []
        for name, driver in DRIVERS.items():
            hold = driver.hold

            def stop(*, name=name, hold=hold):
                holds.append(name)
                hold()

            monkeypatch.setattr(driver, "hold", stop)

        def fail(_target):
            raise RuntimeFault(
                FaultCode.MOTOR_FAILURE,
                "right writer lost its reply",
                context={"part": "right"},
            )

        monkeypatch.setattr(DRIVERS["right"], "write", fail)
        result = run.start_playback(request()).wait()
        assert result["status"] == "failed", result
        assert result["failure"]["outcome_unknown"] is True
        assert result["failure"]["evidence"]["code"] == FaultCode.MOTOR_FAILURE
        assert result["failure"]["evidence"]["context"]["part"] == "right"
        assert sdk._managed.arms["left"].accepted == 1
        assert sdk._managed.arms["right"].accepted == 0
        assert holds == ["left", "right"]
        assert result["cleanup"]["error"] is None, result
