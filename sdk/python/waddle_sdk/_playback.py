"""Local owner adaptation for native playback; no Python timing loop."""

from __future__ import annotations

import json
from typing import TYPE_CHECKING

from ._native import core
from .runtime import (
    FaultCode,
    FiniteJointMotion,
    JointPositionCommand,
    RuntimeFault,
    SupportFact,
)

if TYPE_CHECKING:
    from .site import Run


def _error(error: Exception, *, unknown: bool):
    fault = RuntimeFault.from_exception(error)
    return core.PlaybackIoError(fault.detail, json.dumps(fault.as_dict()), unknown)


class PlaybackHandle:
    """A retained native handle. Waiting releases the interpreter lock."""

    def __init__(self, native):
        self._native = native

    @staticmethod
    def _snapshot(value):
        failure = value["failure"]
        if failure is not None and failure["evidence"]:
            failure["evidence"] = json.loads(failure["evidence"])
        return value

    def snapshot(self):
        return self._snapshot(self._native.snapshot())

    def wait(self):
        return self._snapshot(self._native.wait())

    def cancel(self, reason: str) -> None:
        self._native.cancel(reason)


def start(run: Run, motion: FiniteJointMotion) -> PlaybackHandle:
    """Called under the ordinary dispatch lock; admission does not wait for I/O."""
    session = run._session
    if run._episode is None:
        raise RuntimeFault(FaultCode.NOT_OPEN, "run has not started")
    if run._episode.done:
        raise RuntimeFault(FaultCode.CONFLICT, "run is already terminal")
    if not isinstance(motion, FiniteJointMotion):
        raise RuntimeFault(FaultCode.INVALID_REQUEST, "provide a FiniteJointMotion")
    support = session.support()
    row = next((r for r in support.rows if r.scope == f"robot:{motion.part}"), None)
    if row is None or SupportFact.FINITE_JOINT_PLAYBACK not in row.facts:
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support finite playback"
        )
    if (
        motion.policy.paced_curve is not None
        and SupportFact.PACED_JOINT_PLAYBACK not in row.facts
    ):
        raise RuntimeFault(FaultCode.UNSUPPORTED, "part does not support paced curves")
    if any(r.command.gripper_force_n is not None for r in motion.references):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED,
            "set constant playback force through the gripper policy",
        )
    gripper = motion.policy.gripper
    if (
        motion.policy.minimum_settle_samples != 1
        and SupportFact.SAMPLED_JOINT_PLAYBACK not in row.facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support counted settling samples"
        )
    if gripper is not None:
        if SupportFact.GRIPPER_PLAYBACK not in row.facts:
            raise RuntimeFault(
                FaultCode.UNSUPPORTED, "part does not support gripper playback"
            )
        declared = session.site.manifest["parts"][motion.part]["gripper"]
        names = session._require().arms[motion.part].joint_names
        if motion.policy.indices != (names.index(declared["joint"]),) or (
            gripper.closed_position,
            gripper.open_position,
            gripper.closed_m,
            gripper.open_m,
        ) != (
            declared["closed_action"],
            declared["open_action"],
            declared["closed_m"],
            declared["open_m"],
        ):
            raise RuntimeFault(
                FaultCode.INVALID_REQUEST,
                "playback must use the Site's declared jaw and mapping",
            )
        if gripper.force is not None and not {
            SupportFact.GRIPPER_FORCE,
            SupportFact.GRIPPER_FORCE_OBSERVATION,
        } <= set(row.facts):
            raise RuntimeFault(
                FaultCode.UNSUPPORTED, "part does not support sourced gripper force"
            )
    if (
        motion.policy.feedback is not None
        and SupportFact.FEEDBACK_JOINT_PLAYBACK not in row.facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support feedback playback"
        )
    if SupportFact.VELOCITY_FEEDFORWARD not in row.facts and any(
        r.command.velocity_feedforward_rad_s is not None for r in motion.references
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support velocity feedforward"
        )
    robot = session._registered_robot_description(session._require())
    composite = "composite" in robot["actionSpace"]
    part = motion.part if composite else None
    actions = []
    for reference in motion.references:
        command = reference.command
        action = {
            "jointPosition": {"values": command.positions},
            "tOffsetNs": reference.offset_ns,
        }
        if part is not None:
            action["part"] = part
        if command.velocity_feedforward_rad_s is not None:
            action["jointVelocityFeedforward"] = {
                "values": command.velocity_feedforward_rad_s
            }
        actions.append(action)
    chunk = json.dumps(
        {
            "actions": actions,
            "horizonNs": motion.references[-1].offset_ns if motion.references else 0,
        }
    )

    def observe():
        try:
            observation = session.observe_parts((motion.part,))
            if motion.part in observation.faults:
                raise observation.faults[motion.part]
            measured = observation.parts[motion.part]
            return {
                "observed_ns": observation.session_ns,
                "positions": measured.joint_position.tolist(),
                "velocities": measured.joint_velocity.tolist(),
                "gripper_force": measured.gripper_force,
            }
        except Exception as error:  # noqa: BLE001 -- retain any vendor fault across the native callback
            return _error(error, unknown=False)

    def dispatch(reference, measured, force_n):
        try:
            with session._dispatch_lock:
                started_ns = session._require().core.stamp().session_ns
                result = run._step(
                    JointPositionCommand(
                        reference["positions"],
                        reference["velocity_feedforward"],
                        gripper_force_n=force_n,
                    ),
                    measured["positions"],
                    part=reference["part"],
                )
            if result.fault is not None or result.gate == "owner_refusal":
                return _error(
                    result.fault
                    or RuntimeFault(FaultCode.SAFETY_REFUSAL, result.detail),
                    unknown=False,
                )
            return {
                "dispatched": result.dispatched and result.gate == "pass",
                "detail": result.gate,
                "started_ns": started_ns,
            }
        except Exception as error:  # noqa: BLE001 -- an unexpected write failure must retain uncertainty
            # A raising write may have reached the device. Preserve its evidence,
            # never retry, and do not confuse local loop drain with physical stop.
            return _error(error, unknown=True)

    try:
        native = run._episode.start_playback(chunk, motion.policy, observe, dispatch)
    except core.PlaybackBusyError as error:
        raise RuntimeFault(FaultCode.BUSY, str(error)) from error
    except (TypeError, ValueError) as error:
        raise RuntimeFault.from_exception(
            error, code=FaultCode.INVALID_REQUEST
        ) from error
    return PlaybackHandle(native)
