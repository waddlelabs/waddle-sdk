"""Narrow compatibility patches for the pinned I2RT YAM transport.

The pinned I2RT receive loop polls SocketCAN in one-millisecond slices against
one wall-clock deadline.  If the Python thread is descheduled across that
deadline, it returns without checking the socket once more.  A healthy reply
then remains queued, is consumed by the next motor transaction, and starts a
cascade of false per-motor timeouts.

The transaction also discards its recovery receive even when that frame is the
expected reply. Match replies throughout its existing 10 ms initial and 9 ms
recovery budgets, skipping unrelated frames without restarting either deadline.

This module keeps the workaround inside the vendor adapter.  It patches only
the exact verified method signature, preserves I2RT's public behavior, and
fails closed when the installed vendor package has drifted.  Custom robot
drivers and the generic SDK contracts are unaffected.

The pinned ``MotorChainRobot.command_joint_state`` also replaces its shared
command object with an all-zero object *before* taking the command lock.  The
server thread can therefore publish one gravity-only tick between ordinary
position/velocity commands.  The atomic replacement below builds the complete
command off to the side and swaps it under the vendor's own lock.
"""

from __future__ import annotations

import inspect
import logging
import time
from typing import Any

_EXPECTED_RECEIVE_PARAMETERS = (
    "self",
    "motor_id",
    "timeout",
    "supress_warning",
)
_EXPECTED_COMMAND_STATE_PARAMETERS = ("self", "joint_state")
_EXPECTED_TRANSACTION_PARAMETERS = (
    "self",
    "id",
    "motor_id",
    "data",
    "max_retry",
    "expected_id",
)
_ROOT_LOG = logging.getLogger()


def _receive_message_starvation_tolerant(
    self: Any,
    motor_id: int | None = None,
    timeout: float = 0.009,
    supress_warning: bool = False,
) -> Any | None:
    """Preserve I2RT receive semantics without starvation-created timeouts."""

    return _receive_before(
        self, motor_id, time.monotonic() + max(float(timeout), 0.0), supress_warning
    )


def _receive_before(
    self: Any,
    motor_id: int | None,
    deadline: float,
    supress_warning: bool = False,
    expected_id: int | None = None,
) -> Any | None:
    """Use one deadline and at most one final nonblocking poll after it expires."""

    while True:
        remaining = deadline - time.monotonic()
        wait = max(remaining, 0.0)
        if self.use_buffered_reader:
            message = self.buffered_reader.get_message(timeout=wait)
        else:
            message = self.bus.recv(timeout=wait)
        if message is not None and (
            expected_id is None or message.arbitration_id == expected_id
        ):
            return message
        if remaining <= 0.0:
            break
    if not supress_warning:
        _ROOT_LOG.warning(
            "\033[91m"
            f"Failed to receive message, {self.name} motor id {motor_id} motor timeout."
            " Check if the motor is powered on or if the motor ID exists."
            "\033[0m"
        )
    return None


_receive_message_starvation_tolerant._waddle_starvation_patch = True  # type: ignore[attr-defined]


def _send_message_get_response_matching(
    self: Any,
    id: int,
    motor_id: int,
    data: Any,
    max_retry: int = 5,
    expected_id: int | None = None,
) -> Any:
    """Retain matching late replies within the pinned vendor's existing budgets."""

    import can

    message = can.Message(arbitration_id=id, data=data, is_extended_id=False)
    if expected_id is None:
        expected_id = self.receive_mode.get_receive_id(motor_id)
    for _ in range(max_retry):
        try:
            self.bus.send(message)
            start = time.monotonic()
            response = _receive_before(
                self, motor_id, start + 0.010, expected_id=expected_id
            )
            if response is None:
                response = _receive_before(
                    self, motor_id, start + 0.019, True, expected_id
                )
            if response is not None:
                return response
        except (can.CanError, AssertionError) as error:
            _ROOT_LOG.warning(error)
            _ROOT_LOG.warning(
                "\033[91m"
                f"CAN Error {self.name}: Failed to communicate with motor {id} over can bus. Retrying..."
                "\033[0m"
            )
        time.sleep(0.001)
    raise AssertionError(
        f"fail to communicate with the motor {id} on {self.name} at can channel {self.bus.channel_info}"
    )


_send_message_get_response_matching._waddle_matching_patch = True  # type: ignore[attr-defined]


def apply_recv_starvation_patch() -> None:
    """Install the verified I2RT receive workaround or refuse the live driver."""

    try:
        from i2rt.motor_drivers.can_interface import CanInterface
    except Exception as error:
        raise RuntimeError(
            "the pinned I2RT CAN receive implementation is unavailable; "
            "refusing to open a YAM without the starvation-safe receive path"
        ) from error

    # Verify both originals before mutating either class method. An already
    # installed receive patch must not bypass the separate transaction guard.
    patches = (
        (
            "_receive_message",
            _EXPECTED_RECEIVE_PARAMETERS,
            "_waddle_starvation_patch",
            _receive_message_starvation_tolerant,
        ),
        (
            "_send_message_get_response",
            _EXPECTED_TRANSACTION_PARAMETERS,
            "_waddle_matching_patch",
            _send_message_get_response_matching,
        ),
    )
    for name, expected, marker, _replacement in patches:
        current = getattr(CanInterface, name, None)
        if getattr(current, marker, False):
            continue
        parameters = (
            tuple(inspect.signature(current).parameters) if callable(current) else ()
        )
        if parameters != expected:
            raise RuntimeError(
                f"the installed I2RT CanInterface.{name} signature is "
                f"{parameters!r}, expected {expected!r}; "
                "re-verify the YAM receive workaround before using this vendor revision"
            )
    for name, _expected, marker, replacement in patches:
        if not getattr(getattr(CanInterface, name), marker, False):
            setattr(CanInterface, name, replacement)


def _command_joint_state_atomic(self: Any, joint_state: Any) -> None:
    """Publish one complete I2RT PD command under the vendor command lock."""

    position = self._clip_robot_joint_pos_command(joint_state["pos"])
    velocity = joint_state["vel"]
    commands = type(self._commands).init_all_zero(len(self.motor_chain))
    commands.pos = self.remapper.to_robot_joint_pos_space(position)
    commands.vel = self.remapper.to_robot_joint_vel_space(velocity)
    commands.kp = joint_state.get("kp", self._kp)
    commands.kd = joint_state.get("kd", self._kd)
    with self._command_lock:
        self._commands = commands


_command_joint_state_atomic._waddle_atomic_command_patch = True  # type: ignore[attr-defined]


def apply_command_state_atomic_patch(robot_type: type[Any] | None = None) -> bool:
    """Patch one I2RT robot type before use; return false when it has no state API."""

    if robot_type is None:
        try:
            from i2rt.robots.motor_chain_robot import MotorChainRobot
        except Exception as error:
            raise RuntimeError(
                "the pinned I2RT YAM command implementation is unavailable; "
                "cannot verify atomic position/velocity commands"
            ) from error
        robot_type = MotorChainRobot

    current = getattr(robot_type, "command_joint_state", None)
    if current is None:
        return False
    if getattr(current, "_waddle_atomic_command_patch", False):
        return True
    parameters = tuple(inspect.signature(current).parameters)
    if parameters != _EXPECTED_COMMAND_STATE_PARAMETERS:
        raise RuntimeError(
            "the installed I2RT MotorChainRobot.command_joint_state signature is "
            f"{parameters!r}, expected {_EXPECTED_COMMAND_STATE_PARAMETERS!r}; "
            "re-verify atomic YAM commands before using this vendor revision"
        )
    robot_type.command_joint_state = _command_joint_state_atomic
    return True


__all__ = ["apply_command_state_atomic_patch", "apply_recv_starvation_patch"]
