"""Reference adapters obey the same declared owner envelopes as other drivers."""

from types import SimpleNamespace

import numpy as np
import pytest
from waddle_sdk.robots.site import PartConfig
from waddle_sdk.simulators.adapters import _arm
from waddle_sdk.simulators.scene import ROBOTS, profile


def reference_arm(robot="yam", **options):
    model = profile(robot)
    measured = np.array(model.home, dtype=float)

    def call(operation, part, *args):
        assert part == "arm"
        if operation == "read":
            return measured.copy(), np.zeros_like(measured)
        if operation == "write":
            measured[:] = args[0]
        else:
            assert operation == "hold"

    owner = SimpleNamespace(config={"robot": robot}, part_call=call)
    config = PartConfig(
        name="arm",
        posture="supervised",
        connection={},
        joint_limits=dict(zip(model.names, model.limits, strict=True)),
        workspace_bounds={},
        envelope={},
        base_frame="arm_base",
        options={"rate_hz": 25, "max_joint_speed_rad_s": 1, **options},
    )
    return _arm(owner, config=config).arms()[""]


@pytest.mark.parametrize("robot", ROBOTS)
@pytest.mark.parametrize("explicit", [False, True])
def test_scalar_or_omitted_reference_envelope_controls_actual_admission(
    robot, explicit
):
    options = {"max_joint_position_error_rad": 0.2 if explicit else None}
    arm = reference_arm(robot, **options)
    target = arm.state()[0]
    target[0] += 0.05
    assert arm.command(target) is explicit
    assert arm.step_caps[0] == pytest.approx(0.04)
    assert arm.position_error_caps == (
        (0.2,) * arm.arm_dof + (0.04,) if explicit else None
    )


def test_ordered_bounds_allow_j2_lag_without_widening_other_joints_or_jaws():
    arm = reference_arm(
        max_joint_position_error_rad=[0.04, 0.2, 0.04, 0.04, 0.04, 0.04]
    )
    target = arm.state()[0]
    target[1] += 0.19
    assert arm.command(target)
    for index, change in [(0, 0.05), (1, 0.21), (6, -0.05)]:
        target = arm.state()[0]
        target[index] += change
        assert not arm.command(target)


@pytest.mark.parametrize(
    "value", [True, "0.2", 0, -1, float("nan"), float("inf"), [], [0.2] * 5, [True] * 6]
)
def test_invalid_envelopes_are_rejected_before_a_driver_is_created(value):
    with pytest.raises(ValueError, match="max_joint_position_error_rad"):
        reference_arm(max_joint_position_error_rad=value)
