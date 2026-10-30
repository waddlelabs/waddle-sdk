"""Stable capability reads do not contend with fresh native force feedback."""

from types import SimpleNamespace

import pytest
from waddle_sdk.simulators.adapters import Driver


def test_descriptions_reuse_model_limits_but_force_stays_live_after_reset():
    calls = []
    state = {"limits": (0.0, 60.0), "force": 4.0}

    def call(operation, part):
        calls.append(operation)
        assert part == "arm"
        return (
            state["limits"] if operation == "gripper_force_limits" else state["force"]
        )

    world = SimpleNamespace(config={"robot": "yam"}, _generation=0, part_call=call)
    driver = Driver(world, part="arm", posture="supervised")
    assert driver.gripper_force_limits_n == (0.0, 60.0)
    # A capability description must not repeatedly enter the physics RPC queue.
    for _ in range(20):
        assert driver.gripper_force_limits_n == (0.0, 60.0)
    assert calls.count("gripper_force_limits") == 1
    assert driver.read_gripper_force().force_n == 4.0
    state["force"] = 7.0
    assert driver.read_gripper_force().force_n == 7.0
    state["limits"] = (0.0, 40.0)
    world._generation += 1
    assert driver.gripper_force_limits_n == (0.0, 40.0)
    assert calls.count("gripper_force_limits") == 2
    assert calls.count("gripper_force") == 2


def test_failed_limit_query_remains_original_and_does_not_poison_future_reads():
    failure = RuntimeError("original simulator metadata refusal")
    first = True

    def call(operation, part):
        nonlocal first
        if first:
            first = False
            raise failure
        return (0.0, 60.0)

    world = SimpleNamespace(config={"robot": "yam"}, _generation=0, part_call=call)
    driver = Driver(world, part="arm", posture="supervised")
    with pytest.raises(RuntimeError) as caught:
        _ = driver.gripper_force_limits_n
    assert caught.value is failure
    assert driver.gripper_force_limits_n == (0.0, 60.0)
