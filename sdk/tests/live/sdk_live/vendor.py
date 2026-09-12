"""Pinned I2RT adapter for measured comparisons, used only in a fresh process.

The command and receive implementations are the vendor's originals. SDK model
envelope arithmetic and the existing site lock are reused without opening an SDK
driver. Private reads below are labelled diagnostics, not SDK observation fields.
"""

# ruff: noqa: BLE001 -- retain original faults while attempting safe teardown.

import threading
import time
from contextlib import contextmanager
from dataclasses import replace

import numpy as np
from waddle_sdk.ownership import _SiteLock
from waddle_sdk.robots import yam
from waddle_sdk.runtime import FaultCode, RuntimeFault

from .session import Bench

VENDOR_GRAVITY_DEFAULT = [1, 1.1, 1.1, 1.2, 1, 1]
_UNCERTAIN_OWNERS = []


def wait_for_startup(owner, robot, channel, *, timeout_s=1.0):
    """Test-only proof that constructor holding has yielded fresh public state.

    I2RT publishes computed torque before a CAN reply and can rebuild joint state
    from old CAN cache. Observe the hold in its command cache, two later complete
    CAN generations (one old scan may be pending publication), then a subsequent
    robot-state update. No target is sent and no position/effort threshold is used.
    """
    started = time.monotonic()
    deadline = started + timeout_s
    evidence = owner.report["startup_evidence"] = {
        "source": "test-only pinned I2RT command/CAN/robot-state observations",
        "timeout_s": timeout_s,
        "ready": False,
        "phase": "hold_command",
        "can_generations": 0,
    }
    chain = robot.motor_chain
    previous_cache = previous_joint_state = None

    def timeout():
        return RuntimeFault(
            FaultCode.MOTOR_FAILURE,
            f"{channel}: startup motor feedback not ready within {timeout_s:.3f} s",
            context={
                "channel": channel,
                "reason": "startup_feedback_timeout",
                "phase": evidence["phase"],
                "timeout_s": timeout_s,
                "can_generations": evidence["can_generations"],
            },
        )

    @contextmanager
    def locked(lock):
        if not lock.acquire(timeout=max(0, deadline - time.monotonic())):
            raise timeout()
        try:
            yield
        finally:
            lock.release()

    try:
        while not evidence["ready"]:
            if getattr(owner, "_background_error", None) is not None:
                raise owner._background_error
            if not chain.running or not robot._server_thread.is_alive():
                raise RuntimeFault(
                    FaultCode.MOTOR_FAILURE,
                    f"{channel}: I2RT motor feedback worker stopped during startup",
                    context={"channel": channel, "reason": "startup_worker_stopped"},
                )
            if time.monotonic() >= deadline:
                raise timeout()
            if evidence["phase"] == "hold_command":
                # Same lock order as I2RT's update: robot state -> CAN command.
                with locked(robot._state_lock):
                    evidence.setdefault(
                        "initial_position_rad", robot._joint_state.pos.tolist()
                    )
                    with locked(robot._command_lock):
                        hold = {
                            key: np.asarray(getattr(robot._commands, key))[:6].copy()
                            for key in ("pos", "kp", "kd")
                        }
                    with locked(chain.command_lock):
                        commands = chain.commands[:6]
                        matched = robot._last_motor_torques is not None and all(
                            np.array_equal(
                                hold[key],
                                [getattr(command, key) for command in commands],
                            )
                            for key in hold
                        )
                        with locked(chain.state_lock):
                            previous_cache = chain.state
                if matched:
                    evidence["holding_kp"] = hold["kp"].tolist()
                    evidence["holding_kd"] = hold["kd"].tolist()
                    evidence["phase"] = "can_feedback"
            elif evidence["phase"] == "can_feedback":
                with locked(chain.state_lock):
                    cache = chain.state
                if cache is not None and cache is not previous_cache:
                    previous_cache = cache
                    evidence["can_generations"] += 1
                if evidence["can_generations"] >= 2:
                    with locked(robot._state_lock):
                        previous_joint_state = robot._joint_state
                    evidence["phase"] = "robot_state"
            else:
                with locked(robot._state_lock):
                    if robot._joint_state is not previous_joint_state:
                        evidence["position_rad"] = robot._joint_state.pos.tolist()
                        evidence["ready"] = True
                        evidence["phase"] = "ready"
            if not evidence["ready"]:
                time.sleep(min(0.001, max(0, deadline - time.monotonic())))
    except BaseException as error:
        evidence["error"] = RuntimeFault.from_exception(error).as_dict()
        raise
    finally:
        evidence["elapsed_s"] = time.monotonic() - started


def diagnostics(robot):
    """Copy measured effort separately from computed motor feedforward torque."""
    observations = robot.get_observations()
    result = {"source": "I2RT diagnostic API; not the public SDK PartObservation"}
    for name in ("joint_eff", "gripper_eff", "temp_mos", "temp_rotor"):
        if name in observations:
            result[name] = np.asarray(observations[name]).copy().tolist()
    torques = robot.get_motor_torques()
    if torques is not None:
        result["computed_feedforward_torque"] = np.asarray(torques).copy().tolist()
    result["can_cycle_hz"] = float(robot.motor_chain.comm_freq)
    result["can_cycle_rate_provenance"] = "vendor RateRecorder last reporting window"
    return result


def settings(robot):
    info = robot.get_robot_info()
    result = {
        key: np.asarray(info[key]).tolist()
        for key in (
            "kp",
            "kd",
            "grav_comp_kd",
            "coulomb_friction",
            "joint_limits",
            "gripper_limits",
            "gravity_comp_factor",
            "gripper_index",
        )
        if info.get(key) is not None
    }
    # Pinned motor_drivers/utils.py float_to_uint/uint_to_float: clamp, truncate
    # to 12 bits, decode. These are encoded requests, not measured gain telemetry.
    for name, maximum in (("kp", 500.0), ("kd", 5.0)):
        if name in result:
            requested = np.asarray(result[name])
            result["encoded_" + name] = (
                np.trunc(np.clip(requested, 0, maximum) * 4095 / maximum)
                * maximum
                / 4095
            ).tolist()
    result["gain_provenance"] = (
        "kp/kd are requested; encoded_kp/kd are predicted MIT 12-bit values, not hardware readback"
    )
    return result


def close_vendor(robot, timeout_s=3.0):
    """Quiesce pinned vendor workers before its public close shuts the socket."""
    robot._stop_event.set()
    robot._server_thread.join(timeout_s)
    if robot._server_thread.is_alive():
        raise RuntimeError("I2RT server did not stop; CAN socket remains owned")
    chain = robot.motor_chain
    writers = [
        thread
        for thread in threading.enumerate()
        if getattr(getattr(thread, "_target", None), "__self__", None) is chain
        and getattr(getattr(thread, "_target", None), "__name__", "")
        == "_set_torques_and_update_state"
    ]
    chain.running = False
    for thread in writers:
        thread.join(timeout_s)
        if thread.is_alive():
            raise RuntimeError("I2RT CAN writer did not stop; CAN socket remains owned")
    robot.close()


class VendorBench(Bench):
    mode = "vendor"

    def __init__(self, config, part):
        super().__init__(config, part)
        self.part = part
        self.robot = None
        self.ownership = None
        self._opening = False
        self._cache = None
        self._cache_changed = None
        row = self.site.manifest["parts"][part]
        if (
            row.get("driver") != "waddle_sdk.robots.yam:arm"
            or row["posture"] != "supervised"
        ):
            raise ValueError("The raw I2RT comparator requires a supervised YAM arm")
        # Compile exactly the manifest's envelope onto a non-hardware SimDriver.
        # Neither Site.open nor LiveDriver is used in this process.
        model_row = {**row, "connection": {**row["connection"], "sim": True}}
        model_site = replace(
            self.site, manifest={**self.site.manifest, "parts": {part: model_row}}
        )
        self.envelope = model_site._assembly(None).rig.arms()[part]
        self.period = 1 / self.envelope.rate_hz
        self.spaces = {
            part: {
                "jointPosition": {
                    "joints": [{"name": name} for name in self.envelope.joint_names]
                }
            }
        }

    def __enter__(self):
        if self.config.get("torque_release_authorized") is not True:
            raise ValueError("Raw I2RT requires explicit torque_release_authorized")
        from i2rt.motor_drivers.can_interface import CanInterface
        from i2rt.robots.get_robot import get_yam_robot
        from i2rt.robots.motor_chain_robot import MotorChainRobot
        from i2rt.robots.utils import GripperType

        for method in (
            CanInterface._receive_message,
            MotorChainRobot.command_joint_state,
        ):
            if not method.__module__.startswith("i2rt."):
                raise RuntimeError("Raw I2RT comparison requires a pristine subprocess")
        self.ownership = _SiteLock(self.site.id)
        row = self.site.manifest["parts"][self.part]
        options = row.get("options", {})
        self.channel = row["connection"]["channel"]
        try:
            self._opening = True
            self.robot = get_yam_robot(
                channel=self.channel,
                gripper_type=GripperType.LINEAR_4310,
                zero_gravity_mode=False,
                gravity_comp_factor=np.asarray(
                    options.get("gravity_comp_factor", yam.DEFAULT_GRAVITY_COMP_FACTOR),
                    dtype=float,
                ),
                gripper_limits_override=options.get("gripper_limits"),
            )
            self._opening = False
            info = self.robot.get_robot_info()
            kp, kd = yam._gain_vectors(
                info["kp"],
                info["kd"],
                arm_gains=options.get("arm_gains"),
                arm_gain_scale=options.get("arm_gain_scale", 1.0),
                gripper_gain_scale=options.get("gripper_gain_scale", 1.0),
                where=f"channel={self.channel}",
            )
            self.robot.update_kp_kd(kp, kd)
            self.report["control_settings"] = settings(self.robot)
            self.report["vendor_pristine"] = True
            self.report["command_period_s"] = self.period
            self.report["velocity_feedforward"] = self.config.get(
                "velocity_feedforward", False
            )
            wait_for_startup(self, self.robot, self.channel)
            self.observe(self.part)
            self.save()
        except BaseException as error:
            self.__exit__(type(error), error, error.__traceback__)
            raise
        return self

    def __exit__(self, exc_type, exc, tb):
        errors = []
        try:
            if self._opening and self.robot is None:
                raise RuntimeError(
                    "I2RT constructor failed before returning a robot; hardware teardown is unconfirmed"
                )
            if self.robot is not None:
                close_vendor(self.robot)
            if self.ownership is not None:
                self.ownership.release()
        except BaseException as error:
            errors.append(error)
            _UNCERTAIN_OWNERS.append(self)
        self.report["shutdown_errors"] = [
            RuntimeFault.from_exception(e).as_dict() for e in errors
        ]
        if exc is not None:
            self.report["error"] = RuntimeFault.from_exception(exc).as_dict()
        self.save()
        if errors:
            if exc is not None:
                exc.add_note(f"Additional teardown error: {errors[0]}")
            else:
                raise errors[0]

    def observe(self, part):
        robot = self.robot
        chain = robot.motor_chain
        now = time.monotonic()
        if not chain.running or not robot._server_thread.is_alive():
            raise RuntimeFault(
                FaultCode.MOTOR_FAILURE,
                "I2RT motor feedback worker stopped",
                context={"channel": self.channel},
            )
        with chain.state_lock:
            cache = chain.state
        if cache is not self._cache:
            self._cache, self._cache_changed = cache, now
        if self._cache_changed is None or now - self._cache_changed > 0.5:
            raise RuntimeFault(
                FaultCode.MOTOR_FAILURE,
                "I2RT CAN feedback cache stalled",
                context={"channel": self.channel, "max_age_s": 0.5},
            )
        values = robot.get_observations()
        q = np.concatenate([values["joint_pos"], values["gripper_pos"]]).copy()
        self._measured = q
        if not np.isfinite(q).all():
            raise RuntimeError("I2RT returned non-finite measured joint positions")
        return q, {
            "joint_velocity": np.concatenate(
                [values["joint_vel"], values["gripper_vel"]]
            ).tolist(),
            "seconds_since_observed_cache_change": now - self._cache_changed,
            "timestamp_provenance": "local receipt; vendor provides no CAN acquisition timestamp",
            "i2rt": diagnostics(robot),
        }

    def positions(self, part):
        return self.observe(part)[0]

    def command(self, part, position, velocity):
        reason = self.envelope.check(position, self._measured)
        if reason is not None:
            raise RuntimeFault(
                FaultCode.SAFETY_REFUSAL,
                reason,
                context={
                    "part": part,
                    "channel": self.channel,
                    "operation": "stream_joint_target",
                    "target": position.tolist(),
                    "measured": self._measured.tolist(),
                    "joint_names": list(self.envelope.joint_names),
                    "joint_limits": [list(row) for row in self.envelope.joint_limits],
                    "workspace_bounds": self.envelope.workspace,
                    "guidance": "Re-establish a supported pose inside the configured envelope before restarting; do not relax the envelope.",
                },
            )
        if self.config.get("velocity_feedforward", False):
            self.robot.command_joint_state(
                {"pos": position.copy(), "vel": velocity.copy()}
            )
        else:
            self.robot.command_joint_pos(position.copy())

    def hold(self):
        self.robot.command_joint_pos(self.positions(self.part))

    def pose(self, part, joints):
        return yam.forward_kinematics(joints[:6])[0]
