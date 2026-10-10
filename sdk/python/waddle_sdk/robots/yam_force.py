"""Optional LINEAR_4310 force servo on the pinned vendor's limiter seam.

Motor feedback is torque (Nm), inferred from current, rather than a load cell.
The nominal transmission is 6.57 rad / 0.096 m. No friction compensation or
object-specific calibration is assumed. The vendor still owns CAN timing,
gains and hard-stop clipping; this adapter changes only its jaw reference.
"""

import math
import threading
import time

from ..runtime import GripperForce


class ForceLimiter:
    _POSITION_BLEND_S = 0.04
    _POSITION_MAX_SPEED_ACTION_S = 1.0
    _FORCE_APPROACH_END_ACTION = 0.05
    _FORCE_BIAS_MAX_N = 5.0
    _FORCE_BIAS_GAIN_S = 2.0

    def __init__(self, original, *, kp, kd, stroke, max_speed, max_force_n=50.0):
        if not callable(getattr(original, "update", None)):
            raise TypeError("missing vendor gripper limiter")
        if (
            not all(
                math.isfinite(x) and x > 0
                for x in (kp, abs(stroke), max_speed, max_force_n)
            )
            or not math.isfinite(kd)
            or kd <= 0
        ):
            raise ValueError("invalid force servo gains or transmission")
        self.original = original
        self.kp, self.kd, self.stroke, self.max_speed = kp, kd, stroke, max_speed
        self.max_force_n = max_force_n
        self.sign = -math.copysign(1, stroke)
        self._lock = threading.RLock()
        self.force_n = None
        self.holding = False
        self.approaching = False
        self._approach_pending = False
        self._last_time = None
        self._pending = None
        self._position_target = None
        self._position_from = None
        self._position_changed_at = None
        self._position_last_time = None
        self._force_bias_n = 0.0

    def set_force(self, force_n, target_qpos=None):
        with self._lock:
            if target_qpos is not None:
                self._pending = (force_n, float(target_qpos))
                return
            self._pending = None
            if force_n != self.force_n:
                self.holding = False
                self.approaching = False
                self._approach_pending = force_n is not None and force_n > 0
                self._last_time = None
                self._position_target = None
                self._position_from = None
                self._position_changed_at = None
                self._position_last_time = None
                self._force_bias_n = 0.0
            self.force_n = force_n

    @property
    def has_mode(self):
        with self._lock:
            return self.force_n is not None or self._pending is not None

    def discard_pending(self):
        with self._lock:
            self._pending = None

    def measurement(self, torque):
        if not math.isfinite(float(torque)):
            raise ValueError("non-finite gripper motor effort")
        return GripperForce(
            max(0.0, self.sign * float(torque)) * 6.57 / 0.096,
            "yam.linear_4310.motor_torque.nominal",
            True,
        )

    def _position_mode(self, state):
        # The host sends one checked jaw reference about every 40-45 ms. The
        # vendor repeats that held position at motor rate, making the motor
        # accelerate and brake at every edge. Spread each admitted change over
        # one host period, with a separate speed ceiling for a direct large
        # goal. Keep the vendor's force-clog correction immediate.
        target = self.original.update(state)
        if not math.isclose(target, state["target_qpos"], rel_tol=0, abs_tol=1e-9):
            self._position_target = None
            return target
        now = time.monotonic()
        previous = state["last_command_qpos"]
        if self._position_target is None or not math.isclose(
            target, self._position_target, rel_tol=0, abs_tol=1e-9
        ):
            self._position_target = target
            self._position_from = previous
            self._position_changed_at = now
        elapsed = max(0.0, now - self._position_changed_at)
        fraction = min(1.0, elapsed / self._POSITION_BLEND_S)
        blended = self._position_from + fraction * (target - self._position_from)
        dt = (
            0.004
            if self._position_last_time is None
            else min(0.02, max(0.0, now - self._position_last_time))
        )
        self._position_last_time = now
        step = (
            abs(self.stroke)
            * min(self.max_speed, self._POSITION_MAX_SPEED_ACTION_S)
            * dt
        )
        return previous + max(-step, min(step, blended - previous))

    def update(self, state):
        with self._lock:
            # The vendor copies commands before entering this callback. Match
            # the admitted jaw reference so an old snapshot cannot pair a new
            # force mode with an earlier position command.
            if self._pending is not None and math.isclose(
                state["target_qpos"], self._pending[1], rel_tol=0, abs_tol=1e-9
            ):
                self.set_force(self._pending[0])
            if self.force_n is None:
                return self._position_mode(state)
            if self.force_n == 0:
                # A neutral/opening release must latch the admitted position,
                # without reviving a historical clogged-state force hold. New
                # substantial closing demand retains the vendor position-mode
                # force limiter; its defaults still protect later jaw moves.
                error = self.sign * (
                    state["target_qpos"]
                    - state.get("current_qpos", state["target_qpos"])
                )
                if error * self.kp > getattr(
                    self.original, "clog_force_threshold", 0.5
                ):
                    return self._position_mode(state)
                return state["target_qpos"]
            now = time.monotonic()
            dt = (
                0.004
                if self._last_time is None
                else min(0.02, max(0, now - self._last_time))
            )
            self._last_time = now
            current, target = state["current_qpos"], state["target_qpos"]
            measured = self.measurement(state["current_eff"]).force_n
            if self._approach_pending:
                # A fresh, lightly loaded start uses the vendor's ordinary fast
                # position controller from any opening; contact or the final 5%
                # of stroke hands over to force mode. A start already pressing
                # on an object, or a force change during an established grasp,
                # stays in force mode. Requiring a mostly open start made grasps
                # from a partly open jaw creep under torque control the whole way.
                self.approaching = measured < min(15.0, self.force_n * 0.5)
                self._approach_pending = False
            if self.approaching:
                if (
                    measured < self.force_n
                    or abs(state["current_qvel"] * self.stroke) > 0.5
                ) and state[
                    "current_normalized_qpos"
                ] > self._FORCE_APPROACH_END_ACTION:
                    return self._position_mode(state)
                self.approaching = False
            if (
                measured >= self.force_n * 0.9
                and abs(state["current_qvel"]) < 0.01
                and state["current_normalized_qpos"] > 0.0005 / 0.095
            ):
                self.holding = True
            if self.holding and abs(state["current_qvel"]) < 0.01:
                # Motor friction leaves the proportional force reference a
                # little short at rest. Correct only a stationary established
                # hold; reset when motion resumes so free travel cannot wind
                # up a large contact impulse.
                self._force_bias_n = min(
                    min(
                        self._FORCE_BIAS_MAX_N,
                        max(0.0, self.max_force_n - self.force_n),
                    ),
                    max(
                        0.0,
                        self._force_bias_n
                        + self._FORCE_BIAS_GAIN_S * (self.force_n - measured) * dt,
                    ),
                )
            else:
                self._force_bias_n = 0.0
            # Choose a PD reference whose nominal motor torque equals the
            # requested force, accounting for damping using raw motor velocity.
            torque = self.sign * (self.force_n + self._force_bias_n) * 0.096 / 6.57
            raw_speed = state["current_qvel"] * self.stroke
            closing_speed = self.sign * raw_speed
            demand = min(
                abs(torque) + self.kd * closing_speed,
                self.kd * self.max_speed * abs(self.stroke),
            )
            limited = current + self.sign * max(0.0, demand) / self.kp
            if not self.holding:
                limited = current + self.sign * min(
                    max(0.0, self.sign * (target - current)),
                    max(0.0, self.sign * (limited - current)),
                )
            previous = state["last_command_qpos"]
            step = abs(self.stroke) * self.max_speed * dt
            # Apply the force ceiling immediately; the speed bound limits only
            # increasing demand, so a high old position latch is released now.
            return current + self.sign * min(
                max(0.0, self.sign * (limited - current)),
                max(0.0, self.sign * (previous - current)) + step,
            )
