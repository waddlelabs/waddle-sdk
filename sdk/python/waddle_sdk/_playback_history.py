"""Bounded diagnostics for playback calls; no execution decisions or I/O."""

from collections import deque
from copy import deepcopy
from threading import Lock
from time import monotonic_ns

CAPACITY = 256


def _stamp(value):
    # Both values were captured together by the existing native session clock.
    return {
        "session_ns": value.session_ns,
        "unix_ns": value.unix_ns,
        # A separate local sample for optional camera-content diagnostics. This
        # is neither the session origin nor another host's monotonic clock.
        "host_monotonic_ns": monotonic_ns(),
    }


class History:
    def __init__(self, run_id, joint_names, stamp):
        self.run_id = run_id
        self.joint_names = joint_names
        self.stamp = stamp
        self._lock = Lock()
        self._events = deque(maxlen=CAPACITY)
        self._count = 0
        self._started = None

    def begin(self, started, reference, measured, force_n=None):
        with self._lock:
            self._count += 1
            began = _stamp(started)
            if self._started is None:
                self._started = began
            self._events.append(
                {
                    "sequence": self._count,
                    "started": began,
                    "returned": None,
                    "result": "pending",
                    "reference": deepcopy(reference),
                    "gripper_force_n": force_n,
                    "measured": {
                        key: deepcopy(measured[key])
                        for key in ("observed_ns", "positions", "velocities")
                    },
                    "gate": None,
                    "error": None,
                }
            )
            return self._count

    def end(self, sequence, *, result, gate=None, error=None):
        with self._lock:
            event = self._events[-1]
            assert event["sequence"] == sequence
            event.update(
                returned=_stamp(self.stamp()),
                result=result,
                gate=gate,
                error=deepcopy(error),
            )

    def snapshot(self):
        with self._lock:
            return {
                "api": "waddle.playback-history/v1",
                "clock": "sdk_session_monotonic",
                "run_id": self.run_id,
                "joint_names": deepcopy(self.joint_names),
                "started": deepcopy(self._started),
                "submissions": deepcopy(list(self._events)),
                "submission_count": self._count,
                "dropped_submissions": self._count - len(self._events),
                "sampled": _stamp(self.stamp()),
            }
