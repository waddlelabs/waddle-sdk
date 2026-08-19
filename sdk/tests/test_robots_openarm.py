from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import ClassVar

import numpy as np
import pytest
import yaml
from waddle_sdk.robots import openarm

STANDARD_LEFT = openarm.ArmSite(
    side="left",
    channel="can1",
    mount_xyz=(0.0, 0.031, 0.698),
    mount_rpy=(-math.pi / 2.0, 0.0, 0.0),
    gripper_limits=(0.7, -0.2),
)
STANDARD_RIGHT = openarm.ArmSite(
    side="right",
    channel="can0",
    mount_xyz=(0.0, -0.031, 0.698),
    mount_rpy=(math.pi / 2.0, 0.0, 0.0),
    gripper_limits=(0.6, -0.3),
)


def test_v1_facts_match_the_pinned_vendor_sources() -> None:
    root = Path(openarm.__file__).with_name("openarm_data")
    limits = yaml.safe_load((root / "arm/joint_limits.yaml").read_text())
    gains = yaml.safe_load((root / "arm/control_gains.yaml").read_text())
    right = openarm.joint_limits("right", (0.5, -0.5))
    for index in range(7):
        row = limits[f"joint{index + 1}"]["limit"]
        if index == 1:
            assert right[index] == pytest.approx(
                (-0.17453267320510335, 3.3161253267948965)
            )
        else:
            assert right[index] == pytest.approx((row["lower"], row["upper"]))
        assert openarm.CONTROL_GAINS[index] == pytest.approx(
            (gains[f"joint{index + 1}"]["kp"], gains[f"joint{index + 1}"]["kd"])
        )
    assert openarm.GRIPPER_GAINS == pytest.approx(
        (gains["hand"]["kp"], gains["hand"]["kd"])
    )
    macro = (root / "gripper/openarm_parallel_gripper.xacro").read_text()
    assert "tcp_xyz:='0 0 0.0835'" in macro


def test_side_limits_are_mirrored_and_gripper_order_is_semantic() -> None:
    left = openarm.joint_limits("left", (0.8, -0.1))
    right = openarm.joint_limits("right", (-0.1, 0.8))
    assert left[0] == pytest.approx((-right[0][1], -right[0][0]))
    assert left[1] == pytest.approx((-right[1][1], -right[1][0]))
    assert left[2:7] == right[2:7]
    assert left[-1] == right[-1] == (-0.1, 0.8)


@pytest.mark.parametrize("values", [(0.0,), (0.1, 0.1), (0.0, float("nan"))])
def test_gripper_endpoint_measurement_is_required(values) -> None:
    with pytest.raises(ValueError, match="gripper"):
        openarm.joint_limits("right", values)


def test_fk_uses_the_common_site_frame_and_calibrated_tcp() -> None:
    left, left_rotation = openarm.forward_kinematics(
        np.zeros(7),
        side="left",
        mount_xyz=STANDARD_LEFT.mount_xyz,
        mount_rpy=STANDARD_LEFT.mount_rpy,
    )
    right, right_rotation = openarm.forward_kinematics(
        np.zeros(7),
        side="right",
        mount_xyz=STANDARD_RIGHT.mount_xyz,
        mount_rpy=STANDARD_RIGHT.mount_rpy,
    )
    assert left == pytest.approx((0.0, 0.1535, 0.076))
    assert right == pytest.approx((0.0, -0.1535, 0.076))
    assert left_rotation @ left_rotation.T == pytest.approx(np.eye(3))
    assert right_rotation @ right_rotation.T == pytest.approx(np.eye(3))
    spheres = openarm.collision_spheres(
        np.zeros(7),
        side="right",
        mount_xyz=STANDARD_RIGHT.mount_xyz,
        mount_rpy=STANDARD_RIGHT.mount_rpy,
    )
    assert [sphere.name for sphere in spheres] == [
        "base",
        "link1",
        "link2",
        "link3",
        "link4",
        "link5",
        "link6",
        "link7",
        "hand",
    ]
    assert all(sphere.radius_m > 0.0 for sphere in spheres)


def test_bimanual_factory_is_lazy_and_declares_part_specific_rows() -> None:
    rig = openarm.bimanual(
        left=STANDARD_LEFT,
        right=STANDARD_RIGHT,
        workspace=None,
        sim=True,
    )
    compiled = rig.robot()._compile([])["actionSpace"]["composite"]["parts"]
    assert [row["name"] for row in compiled] == ["left_arm", "right_arm"]
    left_joints = compiled[0]["space"]["jointPosition"]["joints"]
    right_joints = compiled[1]["space"]["jointPosition"]["joints"]
    assert len(left_joints) == len(right_joints) == openarm.JOINT_COUNT
    assert left_joints[0]["minPosition"] == pytest.approx(-3.490659)
    assert right_joints[0]["maxPosition"] == pytest.approx(3.490659)
    frames = rig.robot()._compile([])["frames"]["transforms"]
    assert [frame["child"] for frame in frames] == [
        "openarm_left_base",
        "openarm_right_base",
    ]
    arms = rig.arms()
    assert list(arms) == ["left_arm", "right_arm"]
    assert all(arm.base_frame == openarm.BASE_FRAME for arm in arms.values())
    assert all(arm.collision_frame == openarm.BASE_FRAME for arm in arms.values())


def test_live_factory_requires_channels_before_any_hardware_import() -> None:
    missing = openarm.ArmSite(
        side="left",
        mount_xyz=(0, 0, 0),
        mount_rpy=(0, 0, 0),
        gripper_limits=(0, 1),
    )
    with pytest.raises(ValueError, match="SocketCAN"):
        openarm.bimanual(
            left=missing,
            right=STANDARD_RIGHT,
            workspace=None,
            sim=False,
        )


class _Motor:
    def __init__(self, position: float):
        self.position = position
        self.velocity = 0.0
        self.sequence = 0

    def get_position(self):
        return self.position

    def get_velocity(self):
        return self.velocity

    def get_state_sequence(self):
        return self.sequence


class _Collection:
    def __init__(self, owner, motors):
        self.owner = owner
        self.motors = motors
        self.commands = []
        self.positions = []

    def get_motors(self):
        return self.motors

    def get_motor(self):
        return self.motors[0]

    def mit_control_all(self, params):
        self.commands.append(params)
        self.owner.events.append(("command", self, params))
        for motor, param in zip(self.motors, params, strict=True):
            motor.position = param.q
            motor.velocity = param.dq
        self.owner.pending = True

    def set_position(self, position, *, speed_rad_s, torque_pu):
        self.positions.append((position, speed_rad_s, torque_pu))
        self.owner.events.append(("command", self, self.positions[-1]))
        self.motors[0].position = position
        self.motors[0].velocity = 0.0
        self.owner.pending = True

    def enable_all(self):
        self.owner.events.append(("collection_enable", self))


class _FakeOpenArm:
    instances: ClassVar[list] = []
    fresh = True

    def __init__(self, channel, enable_fd):
        self.channel = channel
        self.enable_fd = enable_fd
        self.enabled = False
        self.disabled = 0
        self.closed = False
        self.pending = False
        self.events = []
        self.arm = None
        self.gripper = None
        self.__class__.instances.append(self)

    def init_arm_motors(self, motor_types, send_ids, recv_ids, modes=()):
        self.motor_types = motor_types
        self.send_ids = send_ids
        self.recv_ids = recv_ids
        self.modes = modes
        self.arm = _Collection(self, [_Motor(index / 10.0) for index in range(7)])

    def init_gripper_motor(self, motor_type, send_id, recv_id, mode):
        self.gripper_ids = (send_id, recv_id)
        self.gripper_mode = mode
        self.gripper = _Collection(self, [_Motor(0.25)])

    def get_arm(self):
        return self.arm

    def get_gripper(self):
        return self.gripper

    def set_callback_mode_all(self, mode):
        self.callback_mode = mode

    def refresh_all(self):
        if self.fresh:
            self.pending = True

    def recv_all(self, timeout_us):
        if self.pending and self.fresh:
            motors = list(self.arm.motors)
            if self.gripper is not None:
                motors.extend(self.gripper.motors)
            for motor in motors:
                motor.sequence += 1
            self.pending = False

    def enable_all(self):
        self.enabled = True
        self.events.append(("enable",))

    def disable_all(self):
        self.enabled = False
        self.disabled += 1

    def close(self):
        self.closed = True


class _MITParam:
    def __init__(self, kp, kd, q, dq, tau):
        self.kp = kp
        self.kd = kd
        self.q = q
        self.dq = dq
        self.tau = tau


@pytest.fixture
def fake_vendor(monkeypatch):
    _FakeOpenArm.instances = []
    _FakeOpenArm.fresh = True
    module = SimpleNamespace(
        OpenArm=_FakeOpenArm,
        MITParam=_MITParam,
        MotorType=SimpleNamespace(DM8009="DM8009", DM4340="DM4340", DM4310="DM4310"),
        ControlMode=SimpleNamespace(MIT="MIT", POS_FORCE="POS_FORCE"),
        CallbackMode=SimpleNamespace(STATE="STATE"),
    )
    monkeypatch.setitem(sys.modules, "openarm_can", module)
    return module


def test_live_driver_seeds_measured_pose_and_maps_position_velocity(
    fake_vendor,
    monkeypatch,
) -> None:
    sleeps = []

    def record_sleep(duration):
        sleeps.append(duration)
        if _FakeOpenArm.instances:
            _FakeOpenArm.instances[-1].events.append(("sleep", duration))

    monkeypatch.setattr(openarm.time, "sleep", record_sleep)
    driver = openarm.LiveDriver("can-test", state_timeout_s=0.002)
    device = _FakeOpenArm.instances[-1]
    assert device.enable_fd is True
    assert device.enabled is True
    assert device.send_ids == list(range(1, 8))
    assert device.modes == ["MIT"] * 7
    assert device.gripper_ids == (8, 24)
    assert device.gripper_mode == "POS_FORCE"
    startup_commands = (
        openarm._SOFT_START_SETTLE_REPEATS + openarm._SOFT_START_RAMP_STEPS + 1
    )
    assert len(device.arm.commands) == len(device.gripper.positions) == startup_commands
    enable_index = next(
        index for index, event in enumerate(device.events) if event[0] == "enable"
    )
    assert [event[0] for event in device.events[enable_index : enable_index + 5]] == [
        "enable",
        "collection_enable",
        "command",
        "command",
        "sleep",
    ]
    assert device.events[enable_index + 1][1] is device.gripper
    first_arm = device.arm.commands[0]
    assert [param.q for param in first_arm] == pytest.approx(
        [index / 10.0 for index in range(7)]
    )
    assert first_arm[0].kp == pytest.approx(0.0)
    assert first_arm[0].kd == pytest.approx(0.0)
    assert device.events[enable_index - 1][0] == "command"
    first_ramp = device.arm.commands[1 + openarm._SOFT_START_SETTLE_REPEATS]
    assert first_ramp[0].kp == pytest.approx(
        openarm.CONTROL_GAINS[0][0] * openarm._SOFT_START_GAIN_FRACTION
    )
    assert first_ramp[0].kd == pytest.approx(
        openarm.CONTROL_GAINS[0][1] * openarm._SOFT_START_GAIN_FRACTION
    )
    arm_kp = [params[0].kp for params in device.arm.commands]
    assert arm_kp == sorted(arm_kp)
    assert arm_kp[-1] == pytest.approx(openarm.CONTROL_GAINS[0][0])
    assert device.gripper.positions[0][2] == pytest.approx(0.0)
    assert device.gripper.positions[-1] == pytest.approx(
        (
            0.25,
            openarm.DEFAULT_GRIPPER_COMMAND_SPEED_RAD_S,
            openarm.DEFAULT_GRIPPER_TORQUE_PU,
        )
    )
    assert sleeps == (
        [openarm._SOFT_START_SETTLE_INTERVAL_S] * openarm._SOFT_START_SETTLE_REPEATS
        + [openarm._SOFT_START_RAMP_INTERVAL_S] * (openarm._SOFT_START_RAMP_STEPS - 1)
    )
    assert [param.q for param in device.arm.commands[-1]] == pytest.approx(
        [index / 10.0 for index in range(7)]
    )
    target = np.linspace(-0.2, 0.5, 8)
    velocity = np.linspace(0.1, 0.8, 8)
    assert driver.write_position_velocity(target, velocity)
    assert [param.dq for param in device.arm.commands[-1]] == pytest.approx(
        velocity[:7]
    )
    assert device.gripper.positions[-1] == pytest.approx(
        (
            target[-1],
            openarm.DEFAULT_GRIPPER_COMMAND_SPEED_RAD_S,
            openarm.DEFAULT_GRIPPER_TORQUE_PU,
        )
    )
    before_pump = len(device.arm.commands)
    driver.step(0.05)
    assert len(device.arm.commands) == before_pump + 1
    assert [param.q for param in device.arm.commands[-1]] == pytest.approx(target[:7])
    position, measured_velocity = driver.read()
    assert position == pytest.approx(target)
    assert measured_velocity[:7] == pytest.approx(velocity[:7])
    driver.close()
    assert device.closed and not device.enabled


def test_live_seven_joint_arm_does_not_initialize_or_wait_for_a_gripper(
    fake_vendor,
) -> None:
    driver = openarm.LiveDriver(
        "can-no-gripper", has_gripper=False, state_timeout_s=0.002
    )
    device = _FakeOpenArm.instances[-1]

    assert device.gripper is None
    assert not hasattr(device, "gripper_ids")
    position, velocity = driver.read()
    assert position.shape == velocity.shape == (7,)

    target = np.linspace(-0.2, 0.4, 7)
    driver.write(target)
    assert [param.q for param in device.arm.commands[-1]] == pytest.approx(target)
    driver.close()


def test_stop_latches_and_reenable_seeds_current_measurement(fake_vendor) -> None:
    driver = openarm.LiveDriver("can-test", state_timeout_s=0.002)
    device = _FakeOpenArm.instances[-1]
    driver.estop()
    assert driver.estopped and not device.enabled
    with pytest.raises(RuntimeError, match="e-stop latch"):
        driver.write(np.zeros(8))
    device.arm.motors[0].position = 0.42
    driver.re_enable()
    assert not driver.estopped and device.enabled
    assert device.arm.commands[-1][0].q == pytest.approx(0.42)


def test_monitor_driver_never_enables_or_accepts_commands(fake_vendor) -> None:
    driver = openarm.LiveDriver("can-test", monitor=True, state_timeout_s=0.002)
    device = _FakeOpenArm.instances[-1]
    assert not device.enabled
    assert not device.arm.commands
    with pytest.raises(RuntimeError, match="monitor posture"):
        driver.write(np.zeros(8))
    driver.close()


def test_stale_motor_state_fails_open_and_closes_the_socket(fake_vendor) -> None:
    _FakeOpenArm.fresh = False
    with pytest.raises(TimeoutError, match="no fresh OpenArm state"):
        openarm.LiveDriver("can-test", state_timeout_s=0.0001)
    device = _FakeOpenArm.instances[-1]
    assert device.closed


def test_bimanual_site_manifest_composes_two_openarm_parts(tmp_path: Path) -> None:
    import waddle_sdk

    manifest = tmp_path / "site.yaml"
    manifest.write_text(
        """
api_version: waddle.site/v1
kind: Site
metadata:
  id: openarm-test
parts:
  left_arm:
    driver: waddle_sdk.robots.openarm:arm
    posture: supervised
    connection: {}
    joint_limits: {}
    gripper:
      joint: gripper
      closed_m: 0.0
      open_m: 0.08
      closed_action: 0.7
      open_action: -0.2
    options:
      sim: true
      side: left
      mount_xyz: [0.0, 0.031, 0.698]
      mount_rpy: [-1.5707963267948966, 0.0, 0.0]
      gripper_limits: [0.7, -0.2]
  right_arm:
    driver: waddle_sdk.robots.openarm:arm
    posture: supervised
    connection: {}
    joint_limits: {}
    gripper:
      joint: gripper
      closed_m: 0.0
      open_m: 0.08
      closed_action: 0.6
      open_action: -0.3
    options:
      sim: true
      side: right
      mount_xyz: [0.0, -0.031, 0.698]
      mount_rpy: [1.5707963267948966, 0.0, 0.0]
      gripper_limits: [0.6, -0.3]
cameras: {}
frames: {}
calibration:
  artifacts: calib/
workspace_bounds:
  min: [-0.7, -0.7, 0.02]
  max: [0.7, 0.7, 1.1]
envelope:
  static_keepouts: []
  self_collision: {}
recording:
  root: recordings/
  format: mcap
""".strip()
        + "\n"
    )
    site = waddle_sdk.load_site(manifest)
    with site.open() as session:
        description = session.describe()
        assert list(description["parts"]) == ["left_arm", "right_arm"]
        parts = description["robot"]["actionSpace"]["composite"]["parts"]
        assert [row["name"] for row in parts] == ["left_arm", "right_arm"]
        assert all(
            row["space"]["jointPosition"]["joints"][-1]["name"] == "gripper"
            for row in parts
        )
        assert session.observe().parts["right_arm"].frame_id == openarm.BASE_FRAME
