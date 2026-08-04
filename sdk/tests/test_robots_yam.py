"""The I2RT YAM's live driver, against a stand-in vendor package.

`test_yam_facts.py` gates the NUMBERS this module states against the vendor's
own model. This file gates what it DOES with a real arm: the calls the driver
makes, and the refusals it raises around them — an absent vendor package, an
arm that reports a different number of joints than this module declares, a
command after an e-stop.

The vendor package is a direct git dependency that is not installed here and
needs a CAN bus to do anything, so the calls are made against a stand-in
installed under its real names. That is exactly what the lazy import makes
possible: nothing is imported until an arm is asked for.
"""

from __future__ import annotations

import sys
import types

import numpy as np
import pytest

from waddle.robots import base, yam

#: The reference rig's bench-measured [closed, open] in motor radians. Per
#: unit, and yours are not these.
GRIPPER_LIMITS_MOTOR_RAD = (0.1, 1.7)


# --------------------------------------------------------------------------
# The live driver, against a stand-in vendor package
# --------------------------------------------------------------------------


class _FakeYamRobot:
    """The four vendor calls this driver makes, and nothing else."""

    def __init__(self, *, dofs: int = 7, info: dict | None = None) -> None:
        self.dofs = dofs
        self.info = {"kp": [10.0] * 7, "kd": [1.0] * 7} if info is None else info
        self.commands: list[np.ndarray] = []
        self.gains: list[tuple] = []
        self.zeroed = 0
        self.closed = 0
        self.observations = {
            "joint_pos": [0.1] * 6,
            "joint_vel": [0.0] * 6,
            "gripper_pos": [0.5],
        }
        self.zero_torque_raises = False

    def num_dofs(self) -> int:
        return self.dofs

    def get_robot_info(self) -> dict:
        return self.info

    def get_observations(self) -> dict:
        return self.observations

    def command_joint_pos(self, values) -> None:
        self.commands.append(np.asarray(values, dtype=float))

    def zero_torque_mode(self) -> None:
        self.zeroed += 1
        if self.zero_torque_raises:
            raise RuntimeError("the bus write timed out")

    def update_kp_kd(self, kp, kd) -> None:
        self.gains.append((kp, kd))

    def close(self) -> None:
        self.closed += 1


class _FakeVendor:
    def __init__(self) -> None:
        self.calls: list[dict] = []
        self.robots: list[_FakeYamRobot] = []
        self.next_kwargs: dict = {}

    def get_yam_robot(self, **kwargs) -> _FakeYamRobot:
        self.calls.append(kwargs)
        robot = _FakeYamRobot(**self.next_kwargs)
        self.robots.append(robot)
        return robot


@pytest.fixture
def vendor(monkeypatch) -> _FakeVendor:
    """A stand-in for the vendor package, installed under its real names.

    The real one is a direct git dependency that is not installed here and
    needs a CAN bus to do anything; what this proves is the shape of the calls
    this driver makes and the refusals it raises around them."""
    fake = _FakeVendor()
    i2rt = types.ModuleType("i2rt")
    robots = types.ModuleType("i2rt.robots")
    get_robot = types.ModuleType("i2rt.robots.get_robot")
    utils = types.ModuleType("i2rt.robots.utils")
    get_robot.get_yam_robot = fake.get_yam_robot

    class GripperType:
        LINEAR_4310 = "LINEAR_4310"

    utils.GripperType = GripperType
    for name, module in (
        ("i2rt", i2rt),
        ("i2rt.robots", robots),
        ("i2rt.robots.get_robot", get_robot),
        ("i2rt.robots.utils", utils),
    ):
        monkeypatch.setitem(sys.modules, name, module)
    return fake


def _live(vendor: _FakeVendor, **overrides) -> yam.LiveDriver:
    kwargs: dict = dict(
        channel="can_left", gripper_limits=GRIPPER_LIMITS_MOTOR_RAD, report=lambda _: None
    )
    kwargs.update(overrides)
    return yam.LiveDriver(**kwargs)


@pytest.mark.skipif(
    "i2rt" in sys.modules, reason="the real vendor package is importable here"
)
def test_a_missing_vendor_package_names_the_command_that_installs_it():
    """The import is lazy — inside `__init__`, so importing this module on a
    machine with no vendor package is fine — and the failure carries the exact
    command, pinned to the same commit every fact in this module is."""
    try:
        import i2rt  # noqa: F401
    except ImportError:
        pass
    else:
        pytest.skip("the real vendor package is installed")
    with pytest.raises(RuntimeError) as excinfo:
        yam.LiveDriver(channel="can_left", gripper_limits=GRIPPER_LIMITS_MOTOR_RAD)
    message = str(excinfo.value)
    assert (
        'pip install "i2rt @ git+https://github.com/i2rt-robotics/i2rt@'
        f'{yam.I2RT_PIN}"' in message
    )


def test_the_live_driver_pins_the_gripper_range_instead_of_calibrating(vendor):
    """Constructing with no override runs a physical auto-calibration that
    DRIVES THE JAWS on every connect. This module never auto-ranges a hand:
    the bench-measured pair is passed every time."""
    driver = _live(vendor)
    assert isinstance(driver, base.Driver)
    (call,) = vendor.calls
    assert call["channel"] == "can_left"
    assert call["zero_gravity_mode"] is False
    assert np.allclose(call["gripper_limits_override"], GRIPPER_LIMITS_MOTOR_RAD)


def test_an_arm_that_reports_other_joints_is_refused_not_adapted_to(vendor):
    vendor.next_kwargs = {"dofs": 6}
    with pytest.raises(RuntimeError, match="6 DOF"):
        _live(vendor)


def test_the_live_driver_reads_the_hand_as_the_seventh_row(vendor):
    driver = _live(vendor)
    position, velocity = driver.read()
    assert position.shape == (yam.JOINT_COUNT,)
    assert position[yam.ARM_JOINT_COUNT] == 0.5
    assert velocity.shape == (yam.JOINT_COUNT,)


def test_an_absent_velocity_reads_as_zero_and_an_absent_position_is_a_fault(vendor):
    """The wire has no "unknown" for a velocity, so an absent one is reported
    as zero; an absent POSITION is a fault, because guessing one would put a
    pose nobody measured into the record."""
    driver = _live(vendor)
    vendor.robots[0].observations = {"joint_pos": [0.1] * 6}
    position, velocity = driver.read()
    assert np.allclose(velocity, 0.0)
    assert position[yam.ARM_JOINT_COUNT] == 0.0
    vendor.robots[0].observations = {}
    with pytest.raises(RuntimeError, match="joint_pos"):
        driver.read()


def test_the_estop_latches_before_the_vendor_call(vendor):
    """A stop that half-happened is still a stop, and the one thing that must
    not follow it is a program that believes it can drive again."""
    driver = _live(vendor)
    vendor.robots[0].zero_torque_raises = True
    with pytest.raises(RuntimeError, match="timed out"):
        driver.estop()
    assert driver.estopped is True
    with pytest.raises(RuntimeError, match="e-stopped"):
        driver.write(np.zeros(yam.JOINT_COUNT))


def test_re_enable_restores_the_snapshotted_gains_and_holds_the_measured_pose(vendor):
    driver = _live(vendor)
    driver.estop()
    driver.re_enable()
    robot = vendor.robots[0]
    assert robot.gains == [([10.0] * 7, [1.0] * 7)]
    assert np.allclose(robot.commands[-1][: yam.ARM_JOINT_COUNT], 0.1)
    assert driver.estopped is False


def test_re_enable_refuses_to_guess_gains_it_never_snapshotted(vendor):
    """A made-up kp is how a demo arm slams. Refusing leaves the latch set and
    the arm floating, which is the state the site operator can already see."""
    vendor.next_kwargs = {"info": {}}
    driver = _live(vendor)
    driver.estop()
    with pytest.raises(RuntimeError, match="refusing to guess"):
        driver.re_enable()
    assert driver.estopped is True


def test_a_zero_gravity_driver_commands_nothing(vendor):
    """`posture="monitor"` builds the arm compliant, and this driver then
    refuses to write at all — so "nothing can command it" is a property of the
    object rather than of a flag somebody remembered to check."""
    driver = _live(vendor, zero_gravity=True)
    assert vendor.calls[0]["zero_gravity_mode"] is True
    with pytest.raises(RuntimeError, match="zero-gravity"):
        driver.write(np.zeros(yam.JOINT_COUNT))


def test_a_live_arm_has_no_home_and_integrates_itself(vendor):
    driver = _live(vendor)
    assert driver.home([0.0] * yam.JOINT_COUNT) is False
    assert driver.step(0.1) is None
    driver.close()
    assert vendor.robots[0].closed == 1
