"""Narrow compatibility patches for the pinned I2RT YAM transport.

The pinned I2RT receive loop polls SocketCAN in one-millisecond slices against
one wall-clock deadline.  If the Python thread is descheduled across that
deadline, it returns without checking the socket once more.  A healthy reply
then remains queued, is consumed by the next motor transaction, and starts a
cascade of false per-motor timeouts.

This module keeps the workaround inside the vendor adapter.  It patches only
the exact verified method signature, preserves I2RT's public behavior, and
fails closed when the installed vendor package has drifted.  Custom robot
drivers and the generic SDK contracts are unaffected.
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
_ROOT_LOG = logging.getLogger()


def _receive_message_starvation_tolerant(
    self: Any,
    motor_id: int | None = None,
    timeout: float = 0.009,
    supress_warning: bool = False,
) -> Any | None:
    """Preserve I2RT receive semantics without starvation-created timeouts."""

    deadline = time.monotonic() + max(float(timeout), 0.0)
    while True:
        remaining = deadline - time.monotonic()
        wait = max(remaining, 0.0)
        if self.use_buffered_reader:
            message = self.buffered_reader.get_message(timeout=wait)
        else:
            message = self.bus.recv(timeout=wait)
        if message is not None:
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


def apply_recv_starvation_patch() -> None:
    """Install the verified I2RT receive workaround or refuse the live driver."""

    try:
        from i2rt.motor_drivers.can_interface import CanInterface
    except Exception as error:
        raise RuntimeError(
            "the pinned I2RT CAN receive implementation is unavailable; "
            "refusing to open a YAM without the starvation-safe receive path"
        ) from error

    current = CanInterface._receive_message
    if getattr(current, "_waddle_starvation_patch", False):
        return
    parameters = tuple(inspect.signature(current).parameters)
    if parameters != _EXPECTED_RECEIVE_PARAMETERS:
        raise RuntimeError(
            "the installed I2RT CanInterface._receive_message signature is "
            f"{parameters!r}, expected {_EXPECTED_RECEIVE_PARAMETERS!r}; "
            "re-verify the YAM receive workaround before using this vendor revision"
        )
    CanInterface._receive_message = _receive_message_starvation_tolerant


__all__ = ["apply_recv_starvation_patch"]
