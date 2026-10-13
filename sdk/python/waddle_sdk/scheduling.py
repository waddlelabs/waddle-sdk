"""Scoped host scheduling for periodic device/control threads.

This is a host request, never a timing guarantee or a substitute for measured
freshness/deadline supervision. It changes only the calling thread and restores
its previous policy on exit. Non-Darwin hosts retain their current scheduling.
"""

from __future__ import annotations

import ctypes
import math
import sys
from contextlib import contextmanager


class SchedulingError(OSError):
    code = "runtime.scheduling_failed"

    def __init__(self, operation: str, kernel_return: int):
        self.operation, self.kernel_return = operation, kernel_return
        super().__init__(f"macOS {operation} failed with Mach return {kernel_return}")

    def as_dict(self) -> dict:
        return {
            "code": self.code,
            "detail": str(self),
            "context": {
                "operation": self.operation,
                "kernel_return": self.kernel_return,
            },
        }


class _Timebase(ctypes.Structure):
    _fields_ = [("numer", ctypes.c_uint32), ("denom", ctypes.c_uint32)]


class _DarwinPolicy:
    def __init__(self):
        self.lib = ctypes.CDLL("/usr/lib/libSystem.B.dylib")
        pointer = ctypes.POINTER(ctypes.c_uint32)
        self.lib.mach_thread_self.restype = ctypes.c_uint32
        self.lib.mach_task_self.restype = ctypes.c_uint32
        self.lib.thread_policy_get.argtypes = (
            ctypes.c_uint32,
            ctypes.c_int,
            pointer,
            pointer,
            ctypes.POINTER(ctypes.c_int),
        )
        self.lib.thread_policy_set.argtypes = (
            ctypes.c_uint32,
            ctypes.c_int,
            pointer,
            ctypes.c_uint32,
        )
        self.lib.mach_port_deallocate.argtypes = (ctypes.c_uint32, ctypes.c_uint32)
        self.lib.mach_timebase_info.argtypes = (ctypes.POINTER(_Timebase),)
        self.thread = self.lib.mach_thread_self()

    @staticmethod
    def _require(code, operation):
        if code:
            raise SchedulingError(operation, code)

    def get(self, flavor, length):
        values = (ctypes.c_uint32 * length)()
        count, default = ctypes.c_uint32(length), ctypes.c_int(0)
        self._require(
            self.lib.thread_policy_get(
                self.thread, flavor, values, ctypes.byref(count), ctypes.byref(default)
            ),
            "thread_policy_get",
        )
        if count.value != length:
            raise SchedulingError("thread_policy_get length", -1)
        return values, bool(default.value)

    def set(self, flavor, values):
        self._require(
            self.lib.thread_policy_set(self.thread, flavor, values, len(values)),
            "thread_policy_set",
        )

    def request(self, period_s, computation_s, constraint_s):
        timebase = _Timebase()
        self._require(
            self.lib.mach_timebase_info(ctypes.byref(timebase)), "mach_timebase_info"
        )
        if not timebase.numer or not timebase.denom:
            raise SchedulingError("mach_timebase_info ratio", -1)
        values = [
            round(value * 1e9 * timebase.denom / timebase.numer)
            for value in (period_s, computation_s, constraint_s)
        ]
        if any(not 1 <= value <= 0xFFFFFFFF for value in values):
            raise ValueError(
                "Periodic scheduling values exceed the Mach time representation"
            )
        self.set(2, (ctypes.c_uint32 * 4)(*values, 1))

    def close(self):
        self._require(
            self.lib.mach_port_deallocate(self.lib.mach_task_self(), self.thread),
            "mach_port_deallocate",
        )


@contextmanager
def periodic_thread(*, period_s: float, computation_s: float, constraint_s: float):
    """Request a periodic calling-thread budget, restoring the prior policy.

    Supply 0 < computation <= constraint <= period, in seconds. Darwin enforces
    a computation floor of half the constraint; declare it rather than relying on
    implicit adjustment. Keep blocking I/O and unrelated search off critical
    threads where possible. The kernel can demote an over-budget thread; callers
    must retain their ordinary measured deadline/freshness checks.
    """
    if any(
        type(value) not in (int, float) or not math.isfinite(value)
        for value in (period_s, computation_s, constraint_s)
    ) or not (0 < constraint_s / 2 <= computation_s <= constraint_s <= period_s):
        raise ValueError(
            "Declare finite 0 < constraint/2 <= computation <= constraint <= period"
        )
    evidence = {
        "kind": "system_default/v1",
        "period_s": period_s,
        "computation_s": computation_s,
        "constraint_s": constraint_s,
    }
    if sys.platform != "darwin":
        yield evidence
        return
    policy = _DarwinPolicy()
    restore = None
    try:
        extended, _default = policy.get(1, 1)
        previous, previous_default = policy.get(2, 4)
        restore = (1, extended) if previous_default else (2, previous)
        policy.request(period_s, computation_s, constraint_s)
        evidence["kind"] = "darwin_thread_time_constraint/v1"
        try:
            yield evidence
        finally:
            policy.set(*restore)
    finally:
        policy.close()


def periodic_wait(event, timeout_s: float, *, period_s: float) -> bool:
    """Wait for stop/cadence with a scoped budget around the blocking primitive.

    Unlike a full-loop request, this restores the caller's policy BEFORE capture,
    rendering, serialization or other work. Non-Darwin waits keep their existing
    policy. A zero timeout or already-set event returns without requesting a policy.
    Request/restoration faults propagate; callers must publish them faithfully.
    """
    if (
        type(timeout_s) not in (int, float)
        or not math.isfinite(timeout_s)
        or timeout_s < 0
        or type(period_s) not in (int, float)
        or not math.isfinite(period_s)
        or period_s <= 0
    ):
        raise ValueError("Declare finite non-negative wait and positive period seconds")
    if event.is_set() or timeout_s == 0:
        return event.is_set()
    computation = min(0.001, period_s / 4)
    with periodic_thread(
        period_s=period_s, computation_s=computation, constraint_s=2 * computation
    ):
        return event.wait(timeout_s)


__all__ = ["SchedulingError", "periodic_thread", "periodic_wait"]
