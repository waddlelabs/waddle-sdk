"""OpenArm seven-axis + gripper-motor observation adapter.

Only ``monitor`` posture is implemented. Construction is non-opening; opening
queries the declared bus and never enables, disables, zeros or commands motors.
Encoder coordinates are raw radians, including ``gripper_motor_rad``. No FK,
physical jaw width, calibrated zero, source model or motion readiness is inferred.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Mapping

import numpy as np

from .. import descriptors
from ..runtime import FaultCode, RuntimeFault
from . import base
from ._openarm_transport import MotorAddress, ReadOnlyBus
from .site import PartConfig

JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 8)) + ("gripper_motor_rad",)


class MonitorDriver:
    """Fresh raw encoder feedback with an explicitly read-only driver lifecycle."""

    kind = "live"
    actuation_supported = False
    estop_supported = False
    close_releases_torque = False

    def __init__(self, *, connection, motors, timeout_s):
        self._lock = threading.Lock()
        self._timeout_s = timeout_s
        self._bus = None
        self._motors = motors
        self.last_feedback = ()
        self.encoding_spans = ()
        try:
            self._bus = ReadOnlyBus(**connection, timeout_s=timeout_s)
            spans = []
            for motor in motors:
                for register, expected in ((7, motor.receive_id), (8, motor.send_id)):
                    actual = self._bus.parameter(motor, register)
                    if actual != expected:
                        raise RuntimeFault(
                            FaultCode.CONFLICT,
                            "OpenArm motor CAN identity mismatch",
                            context={
                                "reason": "openarm.motor_identity_mismatch",
                                "interface": connection["interface"],
                                "motor_id": motor.send_id,
                                "register": register,
                                "expected": expected,
                                "actual": actual,
                            },
                        )
                pmax, vmax = (
                    self._bus.parameter(motor, register) for register in (21, 22)
                )
                if not all(
                    math.isfinite(value) and value > 0 for value in (pmax, vmax)
                ):
                    raise RuntimeFault(
                        FaultCode.MOTOR_FAILURE,
                        "OpenArm encoding spans must be finite and positive",
                        context={
                            "reason": "openarm.invalid_encoding_spans",
                            "interface": connection["interface"],
                            "motor_id": motor.send_id,
                            "pmax": pmax,
                            "vmax": vmax,
                        },
                    )
                spans.append((pmax, vmax))
            self.encoding_spans = tuple(spans)
            self.read()
        except BaseException:
            self.close()
            raise

    @property
    def estopped(self):
        # This owner has no physical stop facility and never claims a latch.
        return False

    def read(self):
        if not self._lock.acquire(timeout=self._timeout_s):
            raise RuntimeFault(
                FaultCode.BUSY,
                "OpenArm monitor read is already in progress",
                context={"reason": "openarm.read_busy"},
            )
        try:
            if self._bus is None:
                raise RuntimeFault(FaultCode.NOT_OPEN, "OpenArm monitor is closed")
            feedback = self._bus.feedback(self._motors, self.encoding_spans)
            self.last_feedback = feedback
            return (
                np.array([row.position_rad for row in feedback]),
                np.array([row.velocity_rad_s for row in feedback]),
            )
        finally:
            self._lock.release()

    @staticmethod
    def _unsupported(operation):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED,
            f"OpenArm monitor is read-only; {operation} is unavailable",
            context={"reason": "openarm.monitor_only", "operation": operation},
        )

    def write(self, target):
        self._unsupported("write")

    def hold(self):
        self._unsupported("hold")

    def estop(self):
        self._unsupported("estop")

    def re_enable(self):
        self._unsupported("re_enable")

    def step(self, dt):
        pass

    def home(self, values):
        return False

    def close(self):
        with self._lock:
            if self._bus is not None:
                self._bus.close()
                self._bus = None


def arm(*, config: PartConfig) -> base.Rig:
    """Declare one monitor-only OpenArm part; no hardware opens until Rig.arms().

    ``connection`` requires ``interface``, ``usb_serial``, ``motor_ids`` and
    ``feedback_ids`` (eight unique standard CAN IDs each). ``bitrate`` defaults
    to 1 Mbit/s; ``configure_can`` defaults false. Explicit finite eight-row
    ``joint_limits`` are owner declarations, never inferred from motor encoding.
    Options are bounded ``timeout_s`` (default .2) and ``rate_hz`` (default 10).
    """
    if config.posture != "monitor":
        raise ValueError(
            "OpenArm currently supports only monitor posture; motion is not implemented"
        )
    connection = dict(config.connection)
    allowed = {
        "interface",
        "usb_serial",
        "motor_ids",
        "feedback_ids",
        "bitrate",
        "configure_can",
    }
    if set(connection) - allowed:
        raise ValueError(
            f"Unknown OpenArm connection keys: {sorted(set(connection) - allowed)}"
        )
    if set(config.options) - {"timeout_s", "rate_hz"}:
        raise ValueError("OpenArm options are timeout_s and rate_hz")
    try:
        send_ids, receive_ids = (
            connection.pop("motor_ids"),
            connection.pop("feedback_ids"),
        )
        if len(send_ids) != 8 or len(receive_ids) != 8:
            raise ValueError("OpenArm requires seven arm motors and one gripper motor")
        motors = tuple(
            MotorAddress(send_id, receive_id)
            for send_id, receive_id in zip(send_ids, receive_ids, strict=True)
        )
        if len(set(send_ids)) != 8 or len(set(receive_ids)) != 8:
            raise ValueError("OpenArm motor and feedback IDs must each be unique")
        if not all(
            isinstance(connection[key], str) and connection[key]
            for key in ("interface", "usb_serial")
        ):
            raise ValueError("OpenArm requires an exact interface and USB serial")
    except (KeyError, TypeError) as error:
        raise ValueError(
            "OpenArm requires interface, usb_serial and eight motor_ids/feedback_ids"
        ) from error
    connection.setdefault("bitrate", 1_000_000)
    connection.setdefault("configure_can", False)
    raw_limits = config.joint_limits
    if isinstance(raw_limits, Mapping):
        if set(raw_limits) != set(JOINT_NAMES):
            raise ValueError(f"OpenArm joint_limits must name exactly {JOINT_NAMES}")
        raw_limits = [raw_limits[name] for name in JOINT_NAMES]
    try:
        if len(raw_limits) != 8 or any(len(row) != 2 for row in raw_limits):
            raise ValueError("OpenArm needs eight explicit joint-limit pairs")
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for row in raw_limits
            for value in row
        ):
            raise ValueError("OpenArm joint limits must be finite numeric values")
        limits = tuple((float(lo), float(hi)) for lo, hi in raw_limits)
        if any(lo >= hi for lo, hi in limits):
            raise ValueError("OpenArm joint limits require lower < upper")
    except TypeError as error:
        raise ValueError("OpenArm needs eight explicit joint-limit pairs") from error
    if config.workspace_bounds:
        raise ValueError(
            "OpenArm workspace bounds require calibrated FK, which this adapter does not provide"
        )
    timeout_s = config.options.get("timeout_s", 0.2)
    rate_hz = config.options.get("rate_hz", 10.0)
    for name, value, maximum in (
        ("timeout_s", timeout_s, 1),
        ("rate_hz", rate_hz, 100),
    ):
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or not 0 < value <= maximum
        ):
            raise ValueError(f"OpenArm {name} must be finite in (0, {maximum}]")

    def build_arms():
        driver = MonitorDriver(
            connection=connection, motors=motors, timeout_s=timeout_s
        )
        try:
            opened = base.Arm(
                part="",
                driver=driver,
                joint_names=JOINT_NAMES,
                joint_limits=limits,
                step_caps=(0.0,) * 8,
                base_frame=config.base_frame or "",
                arm_dof=7,
                rate_hz=rate_hz,
            )
        except BaseException:
            driver.close()
            raise
        return {"": opened}

    return base.Rig(
        declaration=descriptors.Robot(
            name="OpenArm raw encoder monitor",
            action_space=descriptors.JointSpace(
                joints=tuple(
                    descriptors.Joint(name=name, min_position=lo, max_position=hi)
                    for name, (lo, hi) in zip(JOINT_NAMES, limits, strict=True)
                ),
                rate_hz=rate_hz,
            ),
        ),
        build_arms=build_arms,
        rate_hz=rate_hz,
        posture=config.posture,
    )


__all__ = ["JOINT_NAMES", "MonitorDriver", "arm"]
