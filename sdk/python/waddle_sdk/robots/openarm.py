"""OpenArm v1 hardware, model facts, kinematics, and rig factories.

The action vector for one arm is seven motor radians followed by the raw
gripper motor angle.  Gripper endpoint measurements and mounting transforms
are site facts and therefore have no global defaults.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass

import numpy as np

from ..descriptors import (
    Camera,
    Chunking,
    Composite,
    FrameTransform,
    Joint,
    JointSpace,
    Robot,
)
from . import base
from .site import PartConfig

__all__ = [
    "ARM_JOINT_COUNT",
    "ARM_JOINT_NAMES",
    "BASE_FRAME",
    "CONTROL_GAINS",
    "DEFAULT_RATE_HZ",
    "GRIPPER_JOINT_NAME",
    "JOINT_COUNT",
    "JOINT_NAMES",
    "LEFT_PART",
    "MODEL_PIN",
    "RIGHT_PART",
    "TCP_FRAME",
    "ArmSite",
    "LiveDriver",
    "arm",
    "bimanual",
    "collision_spheres",
    "declaration",
    "forward_kinematics",
    "joint_limits",
]

MODEL_PIN = "openarm_description@1fba2cbc05001f05b4514120b70130b4ac06f409"
CAN_PIN = "openarm_can@98666042b5e9cd5b55d0bd1d7fc3aa5c42caae4d"
CAN_INSTALL = (
    'uv pip install "openarm-can @ '
    "git+https://github.com/enactic/openarm_can.git@"
    '98666042b5e9cd5b55d0bd1d7fc3aa5c42caae4d#subdirectory=python"'
)

ARM_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))
GRIPPER_JOINT_NAME = "gripper"
JOINT_NAMES = ARM_JOINT_NAMES + (GRIPPER_JOINT_NAME,)
ARM_JOINT_COUNT = 7
JOINT_COUNT = 8

LEFT_PART = "left_arm"
RIGHT_PART = "right_arm"
BASE_FRAME = "openarm_site"
TCP_FRAME = "hand_tcp"

ARM_SEND_CAN_IDS = tuple(range(0x01, 0x08))
ARM_RECV_CAN_IDS = tuple(range(0x11, 0x18))
DEFAULT_GRIPPER_SEND_CAN_ID = 0x08
DEFAULT_GRIPPER_RECV_CAN_ID = 0x18

MOTOR_TYPE_NAMES = (
    "DM8009",
    "DM8009",
    "DM4340",
    "DM4340",
    "DM4310",
    "DM4310",
    "DM4310",
)
CONTROL_GAINS = (
    (70.0, 2.75),
    (70.0, 2.5),
    (70.0, 2.0),
    (60.0, 2.0),
    (10.0, 0.7),
    (10.0, 0.6),
    (10.0, 0.5),
)
GRIPPER_GAINS = (5.0, 0.1)
MAX_EFFORT_NM = (40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0)

_RIGHT_ARM_LIMITS = (
    (-1.396263, 3.490659),
    (-0.17453267320510335, 3.3161253267948965),
    (-1.570796, 1.570796),
    (0.0, 2.443461),
    (-1.570796, 1.570796),
    (-0.785398, 0.785398),
    (-1.570796, 1.570796),
)
_LEFT_ARM_LIMITS = (
    (-3.490659, 1.396263),
    (-3.3161253267948965, 0.17453267320510335),
    *_RIGHT_ARM_LIMITS[2:],
)

DEFAULT_RATE_HZ = 20.0
DEFAULT_MAX_JOINT_SPEED_RAD_S = 0.5
DEFAULT_MAX_GRIPPER_SPEED_RAD_S = 1.0
DEFAULT_GRIPPER_COMMAND_SPEED_RAD_S = 5.0
DEFAULT_GRIPPER_TORQUE_PU = 0.5
DEFAULT_MAX_FEEDFORWARD_RAD_S = 1.0
DEFAULT_STATE_TIMEOUT_S = 0.025

_SOFT_START_GAIN_FRACTION = 0.05
_SOFT_START_SETTLE_REPEATS = 10
_SOFT_START_SETTLE_INTERVAL_S = 0.01
_SOFT_START_RAMP_STEPS = 20
_SOFT_START_RAMP_INTERVAL_S = 0.02

_ORIGINS = (
    (0.0, 0.0, 0.0625),
    (-0.0301, 0.0, 0.06),
    (0.0301, 0.0, 0.06625),
    (0.0, 0.0315, 0.15375),
    (0.0, -0.0315, 0.0955),
    (0.0375, 0.0, 0.1205),
    (-0.0375, 0.0, 0.0),
)
_COMMON_AXES = (
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
)
_HAND_ORIGIN = (0.0, 0.0, 0.1025)
_TCP_OFFSET = (0.0, 0.0, 0.0835)


@dataclass(frozen=True)
class ArmSite:
    """Facts belonging to one installed arm.

    mount_xyz and mount_rpy place the arm's link0 in the common site frame.
    gripper_limits is the measured (closed, open) pair in raw motor radians,
    or None when this installed arm has no responsive hand motor.
    """

    side: str
    mount_xyz: Sequence[float]
    mount_rpy: Sequence[float]
    gripper_limits: Sequence[float] | None
    channel: str | None = None
    arm_send_can_ids: Sequence[int] = ARM_SEND_CAN_IDS
    arm_recv_can_ids: Sequence[int] = ARM_RECV_CAN_IDS
    gripper_send_can_id: int = DEFAULT_GRIPPER_SEND_CAN_ID
    gripper_recv_can_id: int = DEFAULT_GRIPPER_RECV_CAN_ID
    enable_fd: bool = True
    sim_home: Sequence[float] | None = None


def _checked_side(side: str) -> str:
    value = str(side).lower()
    if value not in {"left", "right"}:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    return value


def _finite_vector(
    values: Sequence[float], width: int, where: str
) -> tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        result = ()
    if len(result) != width or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{where} must contain {width} finite numbers")
    return result


def _checked_gripper_limits(values: Sequence[float], where: str) -> tuple[float, float]:
    result = _finite_vector(values, 2, where)
    if result[0] == result[1]:
        raise ValueError(f"{where} closed and open motor angles must differ")
    return result[0], result[1]


def joint_limits(
    side: str, gripper_limits: Sequence[float] | None
) -> tuple[tuple[float, float], ...]:
    """Owner envelope rows for one side and its installed hand, if any."""
    side = _checked_side(side)
    arm_rows = _LEFT_ARM_LIMITS if side == "left" else _RIGHT_ARM_LIMITS
    if gripper_limits is None:
        return arm_rows
    closed, opened = _checked_gripper_limits(gripper_limits, "gripper_limits")
    return arm_rows + ((min(closed, opened), max(closed, opened)),)


def _axis_angle(axis: Sequence[float], angle: float) -> np.ndarray:
    vector = np.asarray(axis, dtype=float)
    vector = vector / np.linalg.norm(vector)
    x, y, z = vector
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    one = 1.0 - c
    return np.array(
        [
            [c + x * x * one, x * y * one - z * s, x * z * one + y * s],
            [y * x * one + z * s, c + y * y * one, y * z * one - x * s],
            [z * x * one - y * s, z * y * one + x * s, c + z * z * one],
        ]
    )


def _chain_poses(
    q: Sequence[float],
    *,
    side: str,
    mount_xyz: Sequence[float],
    mount_rpy: Sequence[float],
) -> tuple[tuple[np.ndarray, ...], np.ndarray]:
    values = np.asarray(q, dtype=float).reshape(-1)
    if values.size != ARM_JOINT_COUNT or not np.all(np.isfinite(values)):
        raise ValueError(f"OpenArm FK needs {ARM_JOINT_COUNT} finite arm joints")
    side = _checked_side(side)
    position = np.asarray(_finite_vector(mount_xyz, 3, "mount_xyz"))
    rotation = base.rpy_matrix(*_finite_vector(mount_rpy, 3, "mount_rpy"))
    axes = _COMMON_AXES + (((0.0, -1.0, 0.0) if side == "left" else (0.0, 1.0, 0.0)),)
    fixed_rpys = ((0.0, 0.0, 0.0),) * 7
    fixed_rpys = list(fixed_rpys)
    fixed_rpys[1] = (
        (-math.pi / 2.0, 0.0, 0.0) if side == "left" else (math.pi / 2.0, 0.0, 0.0)
    )
    points: list[np.ndarray] = [position.copy()]
    for origin, fixed_rpy, axis, angle in zip(
        _ORIGINS, fixed_rpys, axes, values, strict=True
    ):
        position = position + rotation @ np.asarray(origin)
        rotation = rotation @ base.rpy_matrix(*fixed_rpy)
        rotation = rotation @ _axis_angle(axis, float(angle))
        points.append(position.copy())
    hand = position + rotation @ np.asarray(_HAND_ORIGIN)
    tcp = hand + rotation @ np.asarray(_TCP_OFFSET)
    points.extend((hand, tcp))
    return tuple(points), rotation


def forward_kinematics(
    q: Sequence[float],
    *,
    side: str,
    mount_xyz: Sequence[float],
    mount_rpy: Sequence[float],
) -> tuple[np.ndarray, np.ndarray]:
    """TCP pose in the common site frame."""
    points, rotation = _chain_poses(
        q, side=side, mount_xyz=mount_xyz, mount_rpy=mount_rpy
    )
    return points[-1].copy(), rotation.copy()


def collision_spheres(
    q: Sequence[float],
    *,
    side: str,
    mount_xyz: Sequence[float],
    mount_rpy: Sequence[float],
) -> tuple[base.CollisionSphere, ...]:
    """Conservative segment-covering body spheres in the common site frame."""
    points, _ = _chain_poses(q, side=side, mount_xyz=mount_xyz, mount_rpy=mount_rpy)
    names = (
        "base",
        "link1",
        "link2",
        "link3",
        "link4",
        "link5",
        "link6",
        "link7",
        "hand",
    )
    spheres: list[base.CollisionSphere] = []
    for name, first, second in zip(names, points[:-1], points[1:], strict=True):
        center = (first + second) / 2.0
        radius = float(np.linalg.norm(second - first) / 2.0 + 0.055)
        spheres.append(base.CollisionSphere(name, tuple(center), radius))
    return tuple(spheres)


class LiveDriver:
    """One OpenArm v1 over SocketCAN with fresh-state and stop latches."""

    kind = "live"

    def __init__(
        self,
        channel: str,
        *,
        arm_send_can_ids: Sequence[int] = ARM_SEND_CAN_IDS,
        arm_recv_can_ids: Sequence[int] = ARM_RECV_CAN_IDS,
        gripper_send_can_id: int = DEFAULT_GRIPPER_SEND_CAN_ID,
        gripper_recv_can_id: int = DEFAULT_GRIPPER_RECV_CAN_ID,
        has_gripper: bool = True,
        enable_fd: bool = True,
        monitor: bool = False,
        arm_gain_scale: float = 1.0,
        gripper_command_speed_rad_s: float = DEFAULT_GRIPPER_COMMAND_SPEED_RAD_S,
        gripper_torque_pu: float = DEFAULT_GRIPPER_TORQUE_PU,
        max_feedforward_rad_s: float = DEFAULT_MAX_FEEDFORWARD_RAD_S,
        state_timeout_s: float = DEFAULT_STATE_TIMEOUT_S,
        report: Callable[[str], None] = base.status,
    ) -> None:
        try:
            import openarm_can as oa
        except ImportError as error:
            raise RuntimeError(
                f"{channel}: driving OpenArm needs the pinned openarm_can binding. "
                f"Install it with:\n\n    {CAN_INSTALL}"
            ) from error
        self.channel = str(channel)
        self._oa = oa
        self._report = report
        self._lock = threading.RLock()
        self._monitor = bool(monitor)
        self._estopped = False
        self._closed = False
        self._command_target: tuple[np.ndarray, np.ndarray] | None = None
        self._has_gripper = bool(has_gripper)
        self._joint_count = ARM_JOINT_COUNT + int(self._has_gripper)
        self._state_timeout_s = float(state_timeout_s)
        self._max_feedforward_rad_s = float(max_feedforward_rad_s)
        if not math.isfinite(self._state_timeout_s) or self._state_timeout_s <= 0.0:
            raise ValueError("state_timeout_s must be finite and > 0")
        if (
            not math.isfinite(self._max_feedforward_rad_s)
            or self._max_feedforward_rad_s <= 0.0
        ):
            raise ValueError("max_feedforward_rad_s must be finite and > 0")
        arm_scale = float(arm_gain_scale)
        if not math.isfinite(arm_scale) or arm_scale <= 0.0:
            raise ValueError("arm_gain_scale must be finite and > 0")
        self._arm_gains = tuple(
            (kp * arm_scale, kd * arm_scale) for kp, kd in CONTROL_GAINS
        )
        self._gripper_command_speed_rad_s = float(gripper_command_speed_rad_s)
        self._gripper_torque_pu = float(gripper_torque_pu)
        if (
            not math.isfinite(self._gripper_command_speed_rad_s)
            or not 0.0 < self._gripper_command_speed_rad_s <= 100.0
        ):
            raise ValueError(
                "gripper_command_speed_rad_s must be finite and in (0, 100]"
            )
        if (
            not math.isfinite(self._gripper_torque_pu)
            or not 0.0 < self._gripper_torque_pu <= 1.0
        ):
            raise ValueError("gripper_torque_pu must be finite and in (0, 1]")
        sends = tuple(int(value) for value in arm_send_can_ids)
        recvs = tuple(int(value) for value in arm_recv_can_ids)
        if len(sends) != 7 or len(recvs) != 7:
            raise ValueError("OpenArm arm CAN ID lists must each contain seven IDs")
        if self._has_gripper and len(set(sends + (int(gripper_send_can_id),))) != 8:
            raise ValueError("OpenArm send CAN IDs must be unique")
        if self._has_gripper and len(set(recvs + (int(gripper_recv_can_id),))) != 8:
            raise ValueError("OpenArm receive CAN IDs must be unique")

        self._device = oa.OpenArm(self.channel, bool(enable_fd))
        try:
            motor_types = [getattr(oa.MotorType, name) for name in MOTOR_TYPE_NAMES]
            self._device.init_arm_motors(
                motor_types,
                list(sends),
                list(recvs),
                [oa.ControlMode.MIT] * ARM_JOINT_COUNT,
            )
            if self._has_gripper:
                self._device.init_gripper_motor(
                    oa.MotorType.DM4310,
                    int(gripper_send_can_id),
                    int(gripper_recv_can_id),
                    oa.ControlMode.POS_FORCE,
                )
                self._device.recv_all(20_000)
            self._arm = self._device.get_arm()
            self._gripper = self._device.get_gripper() if self._has_gripper else None
            self._device.set_callback_mode_all(oa.CallbackMode.STATE)
            position, _ = self._fresh_state_locked()
            if self._monitor:
                self._report(
                    f"live {self.channel}: monitor posture; motors remain disabled"
                )
            else:
                self._enable_at_measured_pose_locked(position)
                self._report(f"live {self.channel}: enabled at measured pose")
        except BaseException:
            try:
                self.close()
            except Exception as close_error:  # noqa: BLE001
                self._report(
                    f"live {self.channel}: close failed while backing out of open: "
                    f"{close_error!r}"
                )
            raise

    def _motors(self) -> tuple[object, ...]:
        motors = tuple(self._arm.get_motors())
        if self._gripper is not None:
            return motors + (self._gripper.get_motor(),)
        return motors

    def _enable_at_measured_pose_locked(self, position: np.ndarray) -> None:
        zeros = np.zeros(self._joint_count)

        # Preload a torque-free MIT frame while disabled so an enable cannot
        # revive gains or a target retained by the motor controller. Repeat the
        # zero-gain frame immediately after enable while every motor settles.
        self._emit_command_locked(position, zeros, gain_fraction=0.0)
        self._device.enable_all()
        if self._gripper is not None:
            # The gripper is the eighth motor in the aggregate enable burst.
            # Re-send its enable through the dedicated collection so a dropped
            # tail frame cannot leave a state-reporting hand torque-disabled.
            self._gripper.enable_all()
        for _ in range(_SOFT_START_SETTLE_REPEATS):
            self._emit_command_locked(position, zeros, gain_fraction=0.0)
            time.sleep(_SOFT_START_SETTLE_INTERVAL_S)

        # Stiffen around where the arm is at every step, not around one earlier
        # sample. This engages position hold without asking the arm to move back.
        target = position
        for step in range(1, _SOFT_START_RAMP_STEPS + 1):
            target, _ = self._fresh_state_locked()
            fraction = _SOFT_START_GAIN_FRACTION + (
                (1.0 - _SOFT_START_GAIN_FRACTION)
                * (step - 1)
                / (_SOFT_START_RAMP_STEPS - 1)
            )
            self._emit_command_locked(target, zeros, gain_fraction=fraction)
            if step != _SOFT_START_RAMP_STEPS:
                time.sleep(_SOFT_START_RAMP_INTERVAL_S)

        # The last ramp frame is already the full-gain hold. Latch that exact
        # measured target for the ordinary command pump without emitting again.
        self._command_target = (target.copy(), zeros.copy())

    def _fresh_state_locked(self) -> tuple[np.ndarray, np.ndarray]:
        before = tuple(int(motor.get_state_sequence()) for motor in self._motors())
        self._device.refresh_all()
        deadline = time.monotonic() + self._state_timeout_s
        after = before
        while time.monotonic() < deadline:
            remaining_us = max(1, int((deadline - time.monotonic()) * 1_000_000))
            self._device.recv_all(min(remaining_us, 5000))
            motors = self._motors()
            after = tuple(int(motor.get_state_sequence()) for motor in motors)
            if all(new > old for new, old in zip(after, before, strict=True)):
                position = np.asarray(
                    [motor.get_position() for motor in motors], dtype=float
                )
                velocity = np.asarray(
                    [motor.get_velocity() for motor in motors], dtype=float
                )
                if not np.all(np.isfinite(position)) or not np.all(
                    np.isfinite(velocity)
                ):
                    raise RuntimeError(
                        f"{self.channel}: OpenArm returned non-finite state"
                    )
                return position, velocity
            self._device.refresh_all()
        stale = [
            JOINT_NAMES[index]
            for index, (new, old) in enumerate(zip(after, before, strict=True))
            if new <= old
        ]
        raise TimeoutError(
            f"{self.channel}: no fresh OpenArm state for {', '.join(stale)} "
            f"within {self._state_timeout_s:.3f}s"
        )

    @property
    def estopped(self) -> bool:
        return self._estopped

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        with self._lock:
            if self._closed:
                raise RuntimeError(f"{self.channel}: OpenArm driver is closed")
            return self._fresh_state_locked()

    def _checked_command(
        self, target: np.ndarray, velocity: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        position = np.asarray(target, dtype=float).reshape(-1)
        feedforward = np.asarray(velocity, dtype=float).reshape(-1)
        if position.size != self._joint_count or feedforward.size != self._joint_count:
            raise ValueError(
                f"{self.channel}: position/velocity widths must be {self._joint_count}, "
                f"got {position.size}/{feedforward.size}"
            )
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(feedforward)):
            raise ValueError(f"{self.channel}: OpenArm commands must be finite")
        if np.any(np.abs(feedforward[:ARM_JOINT_COUNT]) > self._max_feedforward_rad_s):
            raise ValueError(
                f"{self.channel}: arm velocity feedforward exceeds "
                f"{self._max_feedforward_rad_s:g} rad/s"
            )
        bounded = feedforward.copy()
        if self._has_gripper:
            bounded[ARM_JOINT_COUNT] = 0.0
        return position, bounded

    def _emit_command_locked(
        self,
        position: np.ndarray,
        feedforward: np.ndarray,
        *,
        gain_fraction: float = 1.0,
    ) -> None:
        gain = float(gain_fraction)
        if not math.isfinite(gain) or not 0.0 <= gain <= 1.0:
            raise ValueError("gain_fraction must be finite and in [0, 1]")
        params = [
            self._oa.MITParam(kp * gain, kd * gain, float(q), float(dq), 0.0)
            for (kp, kd), q, dq in zip(
                self._arm_gains,
                position[:ARM_JOINT_COUNT],
                feedforward[:ARM_JOINT_COUNT],
                strict=True,
            )
        ]
        self._arm.mit_control_all(params)
        if self._gripper is not None:
            self._gripper.set_position(
                float(position[-1]),
                speed_rad_s=self._gripper_command_speed_rad_s,
                torque_pu=self._gripper_torque_pu * gain,
            )
        self._device.recv_all(0)

    def _command_locked(self, target: np.ndarray, velocity: np.ndarray) -> None:
        position, feedforward = self._checked_command(target, velocity)
        self._command_target = (position.copy(), feedforward.copy())
        self._emit_command_locked(position, feedforward)

    def _assert_commandable(self) -> None:
        if self._closed:
            raise RuntimeError(f"{self.channel}: OpenArm driver is closed")
        if self._monitor:
            raise RuntimeError(
                f"{self.channel}: monitor posture leaves motors disabled and commands nothing"
            )
        if self._estopped:
            raise RuntimeError(
                f"{self.channel}: e-stop latch is set; re-enable only at the machine"
            )

    def write(self, target: np.ndarray) -> None:
        with self._lock:
            self._assert_commandable()
            self._command_locked(target, np.zeros(self._joint_count))

    def write_position_velocity(
        self, target: np.ndarray, velocity_feedforward_rad_s: np.ndarray
    ) -> bool:
        with self._lock:
            self._assert_commandable()
            self._command_locked(target, velocity_feedforward_rad_s)
            return True

    def hold(self) -> None:
        with self._lock:
            self._assert_commandable()
            position, _ = self._fresh_state_locked()
            self._command_locked(position, np.zeros(self._joint_count))

    def estop(self) -> None:
        with self._lock:
            self._estopped = True
            self._command_target = None
            if not self._closed:
                self._device.disable_all()
                self._device.recv_all(0)
            self._report(
                f"live {self.channel}: motors disabled; the arm is compliant, not position-held"
            )

    def re_enable(self) -> None:
        with self._lock:
            if self._monitor:
                raise RuntimeError(
                    f"{self.channel}: monitor posture cannot be re-enabled"
                )
            if self._closed:
                raise RuntimeError(f"{self.channel}: OpenArm driver is closed")
            position, _ = self._fresh_state_locked()
            self._enable_at_measured_pose_locked(position)
            self._estopped = False
            self._report(f"live {self.channel}: re-enabled at measured pose")

    def step(self, dt: float) -> None:
        del dt
        with self._lock:
            if (
                self._closed
                or self._monitor
                or self._estopped
                or self._command_target is None
            ):
                return
            position, feedforward = self._command_target
            self._emit_command_locked(position, feedforward)

    def home(self, values: Sequence[float]) -> bool:
        return False

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._command_target = None
            try:
                self._device.disable_all()
                self._device.recv_all(0)
            finally:
                self._device.close()


def _site(site: ArmSite, *, sim: bool, where: str) -> ArmSite:
    side = _checked_side(site.side)
    mount_xyz = _finite_vector(site.mount_xyz, 3, f"{where}.mount_xyz")
    mount_rpy = _finite_vector(site.mount_rpy, 3, f"{where}.mount_rpy")
    grip = (
        None
        if site.gripper_limits is None
        else _checked_gripper_limits(site.gripper_limits, f"{where}.gripper_limits")
    )
    if not sim and not site.channel:
        raise ValueError(f"{where}: a live OpenArm needs a SocketCAN channel")
    sends = tuple(int(value) for value in site.arm_send_can_ids)
    recvs = tuple(int(value) for value in site.arm_recv_can_ids)
    if len(sends) != 7 or len(recvs) != 7:
        raise ValueError(f"{where}: arm CAN ID lists must contain seven IDs")
    if site.sim_home is None:
        arm_home = (0.0, (-0.4 if side == "left" else 0.4), 0.0, 0.7, 0.0, 0.0, 0.0)
        home = arm_home if grip is None else arm_home + ((grip[0] + grip[1]) / 2.0,)
    else:
        width = ARM_JOINT_COUNT + int(grip is not None)
        home = _finite_vector(site.sim_home, width, f"{where}.sim_home")
    return ArmSite(
        side=side,
        mount_xyz=mount_xyz,
        mount_rpy=mount_rpy,
        gripper_limits=grip,
        channel=site.channel,
        arm_send_can_ids=sends,
        arm_recv_can_ids=recvs,
        gripper_send_can_id=int(site.gripper_send_can_id),
        gripper_recv_can_id=int(site.gripper_recv_can_id),
        enable_fd=bool(site.enable_fd),
        sim_home=home,
    )


def _part_space(
    side: str,
    gripper_limits: Sequence[float] | None,
    *,
    rate_hz: float,
    max_joint_speed_rad_s: float,
    max_gripper_speed_rad_s: float,
) -> JointSpace:
    limits = joint_limits(side, gripper_limits)
    width = len(limits)
    names = JOINT_NAMES[:width]
    velocities = ((max_joint_speed_rad_s,) * 7 + (max_gripper_speed_rad_s,))[:width]
    efforts = (MAX_EFFORT_NM + (None,))[:width]
    return JointSpace(
        joints=[
            Joint(
                name=name,
                min_position=limit[0],
                max_position=limit[1],
                max_velocity=velocity,
                max_effort=effort,
            )
            for name, limit, velocity, effort in zip(
                names, limits, velocities, efforts, strict=True
            )
        ],
        rate_hz=rate_hz,
        chunking=Chunking(horizon=1, replan="immediate", interp="hold"),
    )


def declaration(
    sites: Mapping[str, ArmSite],
    *,
    name: str = "openarm-v1",
    robot_id: str = "",
    cell_id: str = "",
    site_frame: str = BASE_FRAME,
    rate_hz: float = DEFAULT_RATE_HZ,
    max_joint_speed_rad_s: float = DEFAULT_MAX_JOINT_SPEED_RAD_S,
    max_gripper_speed_rad_s: float = DEFAULT_MAX_GRIPPER_SPEED_RAD_S,
    cameras: Mapping[str, Camera] | None = None,
) -> Robot:
    """Canonical multi-part declaration; part order is action row order."""
    if not sites:
        raise ValueError("OpenArm declaration needs at least one part")
    spaces = {
        part: _part_space(
            site.side,
            site.gripper_limits,
            rate_hz=rate_hz,
            max_joint_speed_rad_s=max_joint_speed_rad_s,
            max_gripper_speed_rad_s=max_gripper_speed_rad_s,
        )
        for part, site in sites.items()
    }
    frames = tuple(
        FrameTransform(
            parent=site_frame,
            child=f"openarm_{site.side}_base",
            position=tuple(float(value) for value in site.mount_xyz),
            quaternion=base.quaternion_wxyz(base.rpy_matrix(*site.mount_rpy)),
        )
        for site in sites.values()
    )
    action_space = (
        next(iter(spaces.values()))
        if len(spaces) == 1
        else Composite(
            rate_hz=rate_hz,
            chunking=Chunking(horizon=1, replan="immediate", interp="hold"),
            **spaces,
        )
    )
    return Robot(
        name=name,
        robot_id=robot_id,
        cell_id=cell_id,
        action_space=action_space,
        cameras=dict(cameras or {}),
        frames=frames,
    )


def _step_caps(
    rate_hz: float, max_joint_speed_rad_s: float, max_gripper_speed_rad_s: float
) -> tuple[float, ...]:
    if rate_hz <= 0.0 or max_joint_speed_rad_s <= 0.0 or max_gripper_speed_rad_s <= 0.0:
        raise ValueError("rate and speed limits must be > 0")
    return (max_joint_speed_rad_s / rate_hz,) * 7 + (max_gripper_speed_rad_s / rate_hz,)


def _build_arms(
    sites: Mapping[str, ArmSite],
    *,
    sim: bool,
    monitor: bool,
    site_frame: str,
    workspace: Sequence[Sequence[float]] | None,
    rate_hz: float,
    caps: Sequence[float],
    arm_gain_scale: float,
    gripper_command_speed_rad_s: float,
    gripper_torque_pu: float,
    max_feedforward_rad_s: float,
    report: Callable[[str], None],
) -> Callable[[], dict[str, base.Arm]]:
    def build() -> dict[str, base.Arm]:
        result: dict[str, base.Arm] = {}
        opened: dict[str, base.Driver] = {}
        try:
            for part, site in sites.items():
                limits = joint_limits(site.side, site.gripper_limits)
                site_caps = tuple(caps[: len(limits)])
                if sim:
                    driver: base.Driver = base.SimDriver(
                        site.sim_home or (),
                        lower=[lo for lo, _ in limits],
                        upper=[hi for _, hi in limits],
                        step_caps=site_caps,
                        rate_hz=rate_hz,
                    )
                else:
                    driver = LiveDriver(
                        site.channel or "",
                        arm_send_can_ids=site.arm_send_can_ids,
                        arm_recv_can_ids=site.arm_recv_can_ids,
                        gripper_send_can_id=site.gripper_send_can_id,
                        gripper_recv_can_id=site.gripper_recv_can_id,
                        has_gripper=site.gripper_limits is not None,
                        enable_fd=site.enable_fd,
                        monitor=monitor,
                        arm_gain_scale=arm_gain_scale,
                        gripper_command_speed_rad_s=gripper_command_speed_rad_s,
                        gripper_torque_pu=gripper_torque_pu,
                        max_feedforward_rad_s=max_feedforward_rad_s,
                        report=report,
                    )
                opened[part] = driver
                fk = lambda q, s=site: forward_kinematics(
                    q, side=s.side, mount_xyz=s.mount_xyz, mount_rpy=s.mount_rpy
                )
                geometry = lambda q, s=site: collision_spheres(
                    q, side=s.side, mount_xyz=s.mount_xyz, mount_rpy=s.mount_rpy
                )
                result[part] = base.Arm(
                    part=part,
                    driver=driver,
                    joint_names=JOINT_NAMES[: len(limits)],
                    joint_limits=limits,
                    step_caps=site_caps,
                    base_frame=site_frame,
                    workspace=workspace,
                    fk=fk,
                    arm_dof=ARM_JOINT_COUNT,
                    collision_spheres=geometry,
                    collision_frame=site_frame,
                    home_values=site.sim_home if sim else None,
                    rate_hz=rate_hz,
                    report=report,
                )
        except BaseException:
            base.close_all(opened, report=report)
            raise
        return result

    return build


def bimanual(
    *,
    left: ArmSite,
    right: ArmSite,
    workspace: Sequence[Sequence[float]] | None,
    sim: bool = False,
    posture: str = "supervised",
    site_frame: str = BASE_FRAME,
    rate_hz: float = DEFAULT_RATE_HZ,
    max_joint_speed_rad_s: float = DEFAULT_MAX_JOINT_SPEED_RAD_S,
    max_gripper_speed_rad_s: float = DEFAULT_MAX_GRIPPER_SPEED_RAD_S,
    arm_gain_scale: float = 1.0,
    gripper_command_speed_rad_s: float = DEFAULT_GRIPPER_COMMAND_SPEED_RAD_S,
    gripper_torque_pu: float = DEFAULT_GRIPPER_TORQUE_PU,
    max_feedforward_rad_s: float = DEFAULT_MAX_FEEDFORWARD_RAD_S,
    name: str = "openarm-v1-bimanual",
    robot_id: str = "",
    cell_id: str = "",
    cameras: Mapping[str, Camera] | None = None,
    estop_hardware: bool = False,
    report: Callable[[str], None] = base.status,
) -> base.Rig:
    """Build a two-arm rig without opening either CAN interface."""
    sites = {
        LEFT_PART: _site(left, sim=sim, where=LEFT_PART),
        RIGHT_PART: _site(right, sim=sim, where=RIGHT_PART),
    }
    if sites[LEFT_PART].side != "left" or sites[RIGHT_PART].side != "right":
        raise ValueError("left_arm must declare side='left' and right_arm side='right'")
    caps = _step_caps(rate_hz, max_joint_speed_rad_s, max_gripper_speed_rad_s)
    return base.Rig(
        declaration=declaration(
            sites,
            name=name,
            robot_id=robot_id,
            cell_id=cell_id,
            site_frame=site_frame,
            rate_hz=rate_hz,
            max_joint_speed_rad_s=max_joint_speed_rad_s,
            max_gripper_speed_rad_s=max_gripper_speed_rad_s,
            cameras=cameras,
        ),
        build_arms=_build_arms(
            sites,
            sim=sim,
            monitor=posture == "monitor",
            site_frame=site_frame,
            workspace=workspace,
            rate_hz=rate_hz,
            caps=caps,
            arm_gain_scale=arm_gain_scale,
            gripper_command_speed_rad_s=gripper_command_speed_rad_s,
            gripper_torque_pu=gripper_torque_pu,
            max_feedforward_rad_s=max_feedforward_rad_s,
            report=report,
        ),
        rate_hz=rate_hz,
        posture=posture,
        estop_hardware=estop_hardware,
        report=report,
    )


def arm(
    *,
    config: PartConfig | None = None,
    site: ArmSite | None = None,
    workspace: Sequence[Sequence[float]] | None = None,
    sim: bool = False,
    posture: str = "supervised",
    part: str = "arm",
    site_frame: str = BASE_FRAME,
    rate_hz: float = DEFAULT_RATE_HZ,
    max_joint_speed_rad_s: float = DEFAULT_MAX_JOINT_SPEED_RAD_S,
    max_gripper_speed_rad_s: float = DEFAULT_MAX_GRIPPER_SPEED_RAD_S,
    arm_gain_scale: float = 1.0,
    gripper_command_speed_rad_s: float = DEFAULT_GRIPPER_COMMAND_SPEED_RAD_S,
    gripper_torque_pu: float = DEFAULT_GRIPPER_TORQUE_PU,
    max_feedforward_rad_s: float = DEFAULT_MAX_FEEDFORWARD_RAD_S,
    name: str = "openarm-v1",
    robot_id: str = "",
    cell_id: str = "",
    cameras: Mapping[str, Camera] | None = None,
    estop_hardware: bool = False,
    report: Callable[[str], None] = base.status,
) -> base.Rig:
    """Build one lazy OpenArm part directly or from a strict site manifest."""
    if config is not None:
        if site is not None:
            raise ValueError("pass either config or site, not both")
        if config.joint_limits:
            raise ValueError(
                "OpenArm site joint_limits overrides are not supported yet; "
                "use the pinned v1 limits and re-zero an out-of-range motor"
            )
        options = config.options
        site = ArmSite(
            side=str(options.get("side", "")),
            mount_xyz=options.get("mount_xyz", ()),
            mount_rpy=options.get("mount_rpy", ()),
            gripper_limits=(
                options.get("gripper_limits", ())
                if bool(options.get("has_gripper", True))
                else None
            ),
            channel=str(config.connection.get("channel", "")) or None,
            arm_send_can_ids=options.get("arm_send_can_ids", ARM_SEND_CAN_IDS),
            arm_recv_can_ids=options.get("arm_recv_can_ids", ARM_RECV_CAN_IDS),
            gripper_send_can_id=int(
                options.get("gripper_send_can_id", DEFAULT_GRIPPER_SEND_CAN_ID)
            ),
            gripper_recv_can_id=int(
                options.get("gripper_recv_can_id", DEFAULT_GRIPPER_RECV_CAN_ID)
            ),
            enable_fd=bool(options.get("enable_fd", True)),
            sim_home=options.get("sim_home"),
        )
        bounds = config.workspace_bounds
        workspace = (bounds.get("min"), bounds.get("max")) if bounds else None
        sim = bool(options.get("sim", False))
        posture = config.posture
        part = ""
        site_frame = str(options.get("site_frame", BASE_FRAME))
        rate_hz = float(options.get("rate_hz", DEFAULT_RATE_HZ))
        max_joint_speed_rad_s = float(
            options.get("max_joint_speed_rad_s", DEFAULT_MAX_JOINT_SPEED_RAD_S)
        )
        max_gripper_speed_rad_s = float(
            options.get("max_gripper_speed_rad_s", DEFAULT_MAX_GRIPPER_SPEED_RAD_S)
        )
        arm_gain_scale = float(options.get("arm_gain_scale", 1.0))
        gripper_command_speed_rad_s = float(
            options.get(
                "gripper_command_speed_rad_s", DEFAULT_GRIPPER_COMMAND_SPEED_RAD_S
            )
        )
        gripper_torque_pu = float(
            options.get("gripper_torque_pu", DEFAULT_GRIPPER_TORQUE_PU)
        )
        max_feedforward_rad_s = float(
            options.get("max_feedforward_rad_s", DEFAULT_MAX_FEEDFORWARD_RAD_S)
        )
        name = config.name
        robot_id = ""
        cell_id = ""
        cameras = None
        estop_hardware = bool(options.get("estop_hardware", False))
    if site is None:
        raise ValueError("OpenArm arm() needs site=ArmSite(...) or config=PartConfig")
    resolved = _site(site, sim=sim, where=part or name)
    caps = _step_caps(rate_hz, max_joint_speed_rad_s, max_gripper_speed_rad_s)
    sites = {part: resolved}
    return base.Rig(
        declaration=declaration(
            sites,
            name=name,
            robot_id=robot_id,
            cell_id=cell_id,
            site_frame=site_frame,
            rate_hz=rate_hz,
            max_joint_speed_rad_s=max_joint_speed_rad_s,
            max_gripper_speed_rad_s=max_gripper_speed_rad_s,
            cameras=cameras,
        ),
        build_arms=_build_arms(
            sites,
            sim=sim,
            monitor=posture == "monitor",
            site_frame=site_frame,
            workspace=workspace,
            rate_hz=rate_hz,
            caps=caps,
            arm_gain_scale=arm_gain_scale,
            gripper_command_speed_rad_s=gripper_command_speed_rad_s,
            gripper_torque_pu=gripper_torque_pu,
            max_feedforward_rad_s=max_feedforward_rad_s,
            report=report,
        ),
        rate_hz=rate_hz,
        posture=posture,
        estop_hardware=estop_hardware,
        report=report,
    )
