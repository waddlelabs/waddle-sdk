"""Fresh coherent feedback remains optional and preserves driver fault authority."""

import threading

import numpy as np
import pytest
import site_fixtures
from test_site_api import _write_site
from waddle_sdk import load_site
from waddle_sdk.robots import base
from waddle_sdk.runtime import FaultCode, GripperForce, RuntimeFault, SupportFact
from waddle_sdk.simulators.adapters import Driver, World


class SnapshotDriver(site_fixtures._Driver):
    feedback_snapshot_supported = True
    gripper_force_supported = True
    gripper_force_limits_n = (1, 50)

    def __init__(self):
        super().__init__()
        self.sample = 0
        self.sample_lock = threading.Lock()
        self.failure = None

    def _sample(self):
        with self.sample_lock:
            self.sample += 1
            return self.sample * 0.001

    def read(self):
        value = self._sample()
        return np.array([value, 0.0]), np.array([value, 0.0])

    def read_gripper_force(self):
        return GripperForce(self._sample(), "test.counter", True)

    def write_gripper_force(self, target, force_n, velocity=None):
        raise AssertionError("read-only feedback never commands force")

    def read_feedback_snapshot(self):
        if self.failure is not None:
            raise self.failure
        value = self._sample()
        force = (
            GripperForce(value, "test.counter", True)
            if self.gripper_force_supported
            else None
        )
        return np.array([value, 0.0]), np.array([value, 0.0]), force


@pytest.mark.parametrize("snapshot", [False, True])
@pytest.mark.parametrize("force_supported", [False, True])
def test_site_feedback_coherence_and_capabilities_follow_the_selected_driver(
    tmp_path, monkeypatch, snapshot, force_supported
):
    driver = SnapshotDriver()
    driver.feedback_snapshot_supported = snapshot
    driver.gripper_force_supported = force_supported
    monkeypatch.setattr(site_fixtures, "_Driver", lambda: driver)
    with load_site(_write_site(tmp_path)).open(console=False, _testing=True) as sdk:
        facts = next(
            row.facts for row in sdk.support().rows if row.scope == "robot:arm"
        )
        assert (SupportFact.FEEDBACK_SNAPSHOT in facts) is snapshot
        assert (SupportFact.GRIPPER_FORCE_OBSERVATION in facts) is force_supported
        previous = None
        for _ in range(3):
            observation = sdk.observe_parts(["arm"])
            assert not observation.faults
            row = observation.parts["arm"]
            position, velocity = row.joint_position[0], row.joint_velocity[0]
            assert position == velocity
            if previous is not None:
                assert position > previous
            previous = position
            if force_supported:
                assert row.gripper_force.source == "test.counter"
                assert row.gripper_force.estimated is True
                if snapshot:
                    assert row.gripper_force.force_n == position
                else:
                    assert row.gripper_force.force_n > position
            else:
                assert row.gripper_force is None


def test_selected_snapshot_fault_is_scoped_without_another_driver_read(
    tmp_path, monkeypatch
):
    driver = SnapshotDriver()
    failure = RuntimeFault(
        FaultCode.MOTOR_FAILURE, "encoder snapshot failed", context={"motor": 2}
    )
    monkeypatch.setattr(site_fixtures, "_Driver", lambda: driver)
    with load_site(_write_site(tmp_path)).open(console=False, _testing=True) as sdk:
        driver.failure = failure
        observation = sdk.observe_parts(["arm"])
        assert not observation.parts
        assert observation.faults["arm"] is failure
        with pytest.raises(RuntimeFault) as caught:
            sdk.observe()
        assert caught.value is failure


@pytest.mark.parametrize(
    "force_supported,force",
    [(False, GripperForce(1, "test.sensor")), (True, None), (True, 1.0)],
)
def test_snapshot_cannot_invent_or_drop_the_declared_force_measurement(
    monkeypatch, force_supported, force
):
    driver = SnapshotDriver()
    driver.gripper_force_supported = force_supported
    monkeypatch.setattr(
        driver,
        "read_feedback_snapshot",
        lambda: (np.zeros(2), np.zeros(2), force),
    )
    arm = base.Arm(
        part="arm",
        driver=driver,
        joint_names=("j0", "j1"),
        joint_limits=((-1, 1), (-1, 1)),
        step_caps=(0.2, 0.2),
    )
    with pytest.raises(TypeError, match="force does not match"):
        arm.feedback()


@pytest.mark.parametrize("supported", [0, 1, "yes", None])
def test_snapshot_capability_requires_a_boolean(supported):
    driver = SnapshotDriver()
    driver.feedback_snapshot_supported = supported
    with pytest.raises(TypeError, match="must be a boolean"):
        base.feedback_snapshot_supported(driver)


@pytest.mark.parametrize(
    "snapshot,robot", [(False, "yam"), (True, "yam"), (True, "xarm7")]
)
def test_reference_driver_selects_only_negotiated_feedback(snapshot, robot):
    world = World({"backend": "mujoco", "robot": robot, "parts": {"arm": {}}})
    world._feedback_snapshot = snapshot
    driver = Driver(world, part="arm", posture="supervised")
    width = len(driver.profile.names)
    generation = [0]

    def acquire(operation, part):
        assert part == "arm"
        if operation == "gripper_force_limits":
            return (1, 50)
        generation[0] += 1
        value = generation[0] * 0.001
        position = np.full(width, value)
        velocity = np.full(width, value)
        if snapshot:
            assert operation == "feedback_snapshot"
            return position, velocity, value if robot == "yam" else None
        if operation == "read":
            return position, velocity
        assert operation == "gripper_force"
        return value

    world.part_call = acquire
    arm = base.Arm(
        part="arm",
        driver=driver,
        joint_names=driver.profile.names,
        joint_limits=driver.profile.limits,
        step_caps=(0.2,) * width,
    )
    assert base.feedback_snapshot_supported(driver) is snapshot
    previous = -1
    for _ in range(2):
        position, velocity, force = arm.feedback()
        assert position[0] == velocity[0] > previous
        previous = position[0]
        if robot == "yam":
            assert force.source == "mujoco.yam.actuator_force"
            assert force.estimated is True
            if snapshot:
                assert force.force_n == position[0]
            else:
                assert force.force_n > position[0]
        else:
            assert force is None


def test_position_only_driver_never_probes_force_limits(tmp_path, monkeypatch):
    class PositionOnly(SnapshotDriver):
        gripper_force_supported = False

        @property
        def gripper_force_limits_n(self):
            raise AssertionError("unsupported force limits must not be read")

    driver = PositionOnly()
    monkeypatch.setattr(site_fixtures, "_Driver", lambda: driver)
    with load_site(_write_site(tmp_path)).open(console=False, _testing=True) as sdk:
        facts = next(
            row.facts for row in sdk.support().rows if row.scope == "robot:arm"
        )
        assert SupportFact.GRIPPER_FORCE_OBSERVATION not in facts
        assert "gripper_force_n" not in sdk.describe()["command_limits"]["arm"]
        observation = sdk.observe_parts(["arm"])
        assert not observation.faults
        assert observation.parts["arm"].gripper_force is None
