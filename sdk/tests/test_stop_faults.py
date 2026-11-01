"""Stop callbacks retain structured faults through the existing native worker."""

import threading
from dataclasses import replace
from types import SimpleNamespace

import pytest
from test_finite_playback import motion
from test_finite_playback import opened as opened  # noqa: PLC0414
from waddle_sdk.robots import base
from waddle_sdk.runtime import FaultCode, RuntimeFault


@pytest.mark.parametrize("estop", [False, True])
@pytest.mark.parametrize("terminal", [False, True])
def test_site_stop_keeps_original_fault_in_completed_receipt(
    opened, monkeypatch, estop, terminal
):
    session, run = opened
    driver = session._managed.arms["arm"].driver
    verb = "estop" if estop else "hold"
    stop, write = getattr(driver, verb), driver.write
    entered, release = threading.Event(), threading.Event()
    fault = RuntimeFault(
        FaultCode.TRANSPORT_LOST,
        "Stop reply was lost",
        retryable=True,
        context={"motor": 4, "vendor_code": 19},
    )

    def failed_stop():
        stop()
        raise fault

    def blocked_write(target):
        entered.set()
        assert release.wait(5)
        write(target)

    request = motion((0, 0), (0.02, -0.01))
    request = replace(request, policy=replace(request.policy, reference_only=True))
    if not terminal:
        monkeypatch.setattr(driver, "write", blocked_write)
    handle = run.start_playback(request)
    try:
        original = handle.wait() if terminal else None
        if not terminal:
            assert entered.wait(3)
        monkeypatch.setattr(driver, verb, failed_stop)
        if terminal:
            assert original["status"] == "references_dispatched"
            receipt = session.stop("test endpoint cleanup", estop=estop)
            assert handle.snapshot() == original
        else:
            getattr(session, verb)("test in-flight cleanup")
            assert handle.snapshot()["status"] == "running"
            release.set()
            result = handle.wait()
            assert result["status"] == "failed"
            assert result["dispatched"] <= 1
            receipt = result["cleanup"]
        assert receipt["request"] == verb
        assert receipt["outcome"] == "unknown"
        assert receipt["measured_stationary"] is False
        assert receipt["evidence"] == fault.as_dict()
    finally:
        release.set()
        monkeypatch.setattr(driver, verb, stop)
        monkeypatch.setattr(driver, "write", write)


@pytest.mark.parametrize("verb", ["hold", "estop"])
@pytest.mark.parametrize("failures", [1, 2])
def test_every_part_gets_stop_and_all_faults_keep_their_fields(verb, failures):
    calls = []
    faults = [
        RuntimeFault(
            FaultCode.MOTOR_FAILURE,
            "drive reply lost",
            retryable=True,
            context={"motor": i},
        )
        for i in range(failures)
    ]
    arms = {}
    for index in range(3):

        def stop(index=index):
            calls.append(index)
            if index < failures:
                raise faults[index]

        arms[str(index)] = SimpleNamespace(**{verb: stop, "estopped": False})
    with pytest.raises(RuntimeFault) as caught:
        if verb == "estop":
            base.estop_all(arms, report=lambda message: None)
        else:
            base.hold_all(arms)
    assert calls == [0, 1, 2]
    if failures == 1:
        assert caught.value is faults[0]
    else:
        assert caught.value.context["failures"] == [
            {"part": str(i), "fault": fault.as_dict()} for i, fault in enumerate(faults)
        ]
