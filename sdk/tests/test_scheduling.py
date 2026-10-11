"""Host requests preserve caller policy and original exceptions across reuse."""

import sys
import threading
from concurrent.futures import ThreadPoolExecutor

import pytest
from waddle_sdk import scheduling


def test_stop_wait_does_not_request_a_policy(monkeypatch):
    def forbidden():
        pytest.fail("A stopped/nonblocking wait requested a host policy")

    monkeypatch.setattr(scheduling, "_DarwinPolicy", forbidden)
    stopping = threading.Event()
    assert not scheduling.periodic_wait(stopping, 0, period_s=0.02)
    stopping.set()
    assert scheduling.periodic_wait(stopping, 1, period_s=0.02)


def test_wait_restores_policy_before_customer_work(monkeypatch):
    from contextlib import contextmanager

    active = []

    @contextmanager
    def requested(**budget):
        assert budget == {
            "period_s": 0.02,
            "computation_s": 0.001,
            "constraint_s": 0.002,
        }
        active.append(True)
        try:
            yield {}
        finally:
            active.pop()

    class Waiting:
        def is_set(self):
            return False

        def wait(self, timeout):
            assert active == [True] and timeout == 0.017
            return False

    monkeypatch.setattr(scheduling, "periodic_thread", requested)
    assert not scheduling.periodic_wait(Waiting(), 0.017, period_s=0.02)
    assert not active


def test_other_platforms_keep_host_policy(monkeypatch):
    monkeypatch.setattr(scheduling.sys, "platform", "linux")

    def forbidden():
        pytest.fail("Other hosts must not call Darwin scheduling APIs")

    monkeypatch.setattr(scheduling, "_DarwinPolicy", forbidden)
    with scheduling.periodic_thread(
        period_s=0.02, computation_s=0.001, constraint_s=0.002
    ) as evidence:
        assert evidence["kind"] == "system_default/v1"


@pytest.mark.parametrize(
    "period,computation,constraint",
    [(0, 0, 0), (0.02, 0.001, 0.003), (0.02, 0.03, 0.03), (float("nan"), 1, 1)],
)
def test_invalid_host_budgets_are_refused(period, computation, constraint):
    with (
        pytest.raises(ValueError),
        scheduling.periodic_thread(
            period_s=period, computation_s=computation, constraint_s=constraint
        ),
    ):
        pytest.fail("Invalid budget was admitted")


@pytest.mark.skipif(sys.platform != "darwin", reason="Native Mach host request")
def test_native_nested_requests_restore_policy_before_thread_reuse():
    def snapshot():
        policy = scheduling._DarwinPolicy()
        try:
            extended, extended_default = policy.get(1, 1)
            constrained, constrained_default = policy.get(2, 4)
            precedence, precedence_default = policy.get(3, 1)
            return (
                tuple(extended),
                extended_default,
                tuple(constrained),
                constrained_default,
                tuple(precedence),
                precedence_default,
            )
        finally:
            policy.close()

    original = RuntimeError("Original caller failure")

    def work():
        before = snapshot()
        with (
            pytest.raises(RuntimeError) as caught,
            scheduling.periodic_thread(
                period_s=0.02, computation_s=0.001, constraint_s=0.002
            ) as evidence,
        ):
            outer = snapshot()
            assert evidence["kind"] == "darwin_thread_time_constraint/v1"
            assert not outer[3]
            with scheduling.periodic_thread(
                period_s=0.01, computation_s=0.0005, constraint_s=0.001
            ):
                assert snapshot()[2] != outer[2]
            assert snapshot() == outer
            raise original
        assert caught.value is original
        assert snapshot() == before
        return before

    with ThreadPoolExecutor(max_workers=1) as pool:
        before = pool.submit(work).result()
        assert pool.submit(snapshot).result() == before
