"""Local owner adaptation for native playback; no Python timing loop."""

from __future__ import annotations

import json
from typing import TYPE_CHECKING

from ._native import core
from ._playback_bounds import observe as observe_bounded
from .robots import base
from .runtime import (
    CoordinatedJointMotion,
    FaultCode,
    FiniteJointMotion,
    JointPositionCommand,
    PlaybackCurveUpdate,
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
        cleanup = value["cleanup"]
        if cleanup is not None and cleanup.get("evidence"):
            cleanup["evidence"] = json.loads(cleanup["evidence"])
        return value

    def snapshot(self):
        return self._snapshot(self._native.snapshot())

    def wait(self):
        return self._snapshot(self._native.wait())

    def cancel(self, reason: str) -> None:
        self._native.cancel(reason)

    def refresh_watchdog(self, observed_ns: int) -> None:
        """Queue feedback time; acceptance is retained in the native snapshot."""
        try:
            self._native.refresh_watchdog(observed_ns)
        except core.PlaybackBusyError as error:
            raise RuntimeFault(FaultCode.BUSY, str(error)) from error
        except (TypeError, ValueError, OverflowError) as error:
            raise RuntimeFault.from_exception(
                error, code=FaultCode.INVALID_REQUEST
            ) from error

    def withdraw_curve(self, request_id: int, expected_revision: int) -> int:
        """Queue withdrawal; only an accepted native receipt confirms removal."""
        try:
            return self._native.withdraw_curve(request_id, expected_revision)
        except core.PlaybackBusyError as error:
            raise RuntimeFault(FaultCode.BUSY, str(error)) from error
        except (TypeError, ValueError, OverflowError) as error:
            raise RuntimeFault.from_exception(
                error, code=FaultCode.INVALID_REQUEST
            ) from error

    def update_curve(self, update: PlaybackCurveUpdate) -> int:
        """Queue a future suffix; inspect its retained receipt before another."""
        if not isinstance(update, PlaybackCurveUpdate):
            raise RuntimeFault(
                FaultCode.INVALID_REQUEST, "provide a PlaybackCurveUpdate"
            )
        try:
            return self._native.update_curve(update)
        except core.PlaybackBusyError as error:
            raise RuntimeFault(FaultCode.BUSY, str(error)) from error
        except (TypeError, ValueError, OverflowError) as error:
            raise RuntimeFault.from_exception(
                error, code=FaultCode.INVALID_REQUEST
            ) from error


def start(
    run: Run, motion: FiniteJointMotion | CoordinatedJointMotion
) -> PlaybackHandle:
    """Called under the ordinary dispatch lock; admission does not wait for I/O."""
    session = run._session
    if run._episode is None:
        raise RuntimeFault(FaultCode.NOT_OPEN, "run has not started")
    if run._episode.done:
        raise RuntimeFault(FaultCode.CONFLICT, "run is already terminal")
    if not isinstance(motion, (FiniteJointMotion, CoordinatedJointMotion)):
        raise RuntimeFault(
            FaultCode.INVALID_REQUEST,
            "provide a FiniteJointMotion or CoordinatedJointMotion",
        )
    group = isinstance(motion, CoordinatedJointMotion)
    parts = motion.parts if group else (motion.part,)
    arms = session._require().arms
    subset = group and parts != tuple(arms)
    if group and (
        len(parts) < 2 or parts != tuple(name for name in arms if name in parts)
    ):
        raise RuntimeFault(
            FaultCode.INVALID_REQUEST,
            "coordinated parts must be distinct and follow Site declaration order",
        )
    support = session.support()
    rows = [
        next((r for r in support.rows if r.scope == f"robot:{p}"), None) for p in parts
    ]
    if any(
        row is None or SupportFact.FINITE_JOINT_PLAYBACK not in row.facts
        for row in rows
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support finite playback"
        )
    facts = set.intersection(*(set(row.facts) for row in rows))
    if (
        motion.policy.timeout_ns is None
        and SupportFact.STREAMED_JOINT_PLAYBACK not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support streamed playback"
        )
    if subset and SupportFact.COORDINATED_SUBSET_PLAYBACK not in facts:
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "parts do not support coordinated subsets"
        )
    if run._episode.playback_active and (
        group and not subset or SupportFact.PARALLEL_JOINT_PLAYBACK not in facts
    ):
        raise RuntimeFault(FaultCode.BUSY, "finite playback is still active")
    if (
        motion.policy.watchdog is not None
        and SupportFact.WATCHDOG_PLAYBACK not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support playback watchdogs"
        )
    if motion.policy.part_hold and (
        group or SupportFact.PART_HOLD_PLAYBACK not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part Hold requires supported named playback"
        )
    if group and SupportFact.COORDINATED_JOINT_PLAYBACK not in facts:
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "parts do not support coordinated playback"
        )
    if (
        motion.policy.credit_pacing_delay
        and SupportFact.PACED_PLAYBACK_DEADLINE not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support paced playback deadlines"
        )
    if (
        motion.policy.paced_curve is not None
        and SupportFact.PACED_JOINT_PLAYBACK not in facts
    ):
        raise RuntimeFault(FaultCode.UNSUPPORTED, "part does not support paced curves")
    if any(r.command.gripper_force_n is not None for r in motion.references):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED,
            "set constant playback force through the gripper policy",
        )
    gripper = motion.policy.gripper
    if group and gripper is not None:
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "coordinated playback has no scalar gripper policy"
        )
    if (
        motion.policy.allow_curve_updates
        and SupportFact.REVISABLE_JOINT_PLAYBACK not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support revisable playback"
        )
    if (
        motion.policy.checkpoint_offsets_ns
        and SupportFact.CHECKPOINTED_JOINT_PLAYBACK not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support playback checkpoint evidence"
        )
    if (
        motion.policy.reference_only
        and SupportFact.REFERENCE_JOINT_PLAYBACK not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support reference-only completion"
        )
    if (
        motion.policy.reference_only
        and motion.policy.feedback is not None
        and motion.policy.feedback.position_window is not None
        and SupportFact.REFERENCE_SETTLING not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support reference-only window evidence"
        )
    if (
        motion.policy.minimum_settle_samples != 1
        and SupportFact.SAMPLED_JOINT_PLAYBACK not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support counted settling samples"
        )
    if gripper is not None:
        if SupportFact.GRIPPER_PLAYBACK not in facts:
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
        if (
            gripper.force is not None
            and not {
                SupportFact.GRIPPER_FORCE,
                SupportFact.GRIPPER_FORCE_OBSERVATION,
            }
            <= facts
        ):
            raise RuntimeFault(
                FaultCode.UNSUPPORTED, "part does not support sourced gripper force"
            )
    if (
        motion.policy.feedback is not None
        and SupportFact.FEEDBACK_JOINT_PLAYBACK not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support feedback playback"
        )
    if SupportFact.VELOCITY_FEEDFORWARD not in facts and any(
        r.command.velocity_feedforward_rad_s is not None for r in motion.references
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support velocity feedforward"
        )
    robot = session._registered_robot_description(session._require())
    composite = "composite" in robot["actionSpace"]
    part = motion.part if composite and not group else None
    actions = []
    for reference in motion.references:
        command = reference.command
        if group:
            try:
                targets = base.split_by_part(
                    {name: arms[name] for name in parts}, command.positions
                )
            except (TypeError, ValueError) as error:
                raise RuntimeFault.from_exception(
                    error, code=FaultCode.INVALID_REQUEST
                ) from error
            action = {
                "composite": {
                    "parts": [
                        {
                            "name": name,
                            "action": {"jointPosition": {"values": target.tolist()}},
                        }
                        for name, target in targets.items()
                    ]
                }
            }
        else:
            action = {"jointPosition": {"values": command.positions}}
        action["tOffsetNs"] = reference.offset_ns
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
            with session._dispatch_lock:
                observation = observe_bounded(run, parts)
                for name in parts:
                    if name in observation.faults:
                        raise observation.faults[name]
                measured = [observation.parts[name] for name in parts]
                return {
                    "observed_ns": observation.session_ns,
                    "positions": [
                        q for row in measured for q in row.joint_position.tolist()
                    ],
                    "velocities": [
                        v for row in measured for v in row.joint_velocity.tolist()
                    ],
                    "gripper_force": None if group else measured[0].gripper_force,
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
                    **({"parts": parts} if subset else {}),
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

    def hold_part(name):
        try:
            with session._dispatch_lock:
                arms[name].hold()
        except Exception as error:  # noqa: BLE001 -- retain unknown owner cleanup
            return _error(error, unknown=True)
        return None

    try:
        native = run._episode.start_playback(
            chunk,
            motion.policy,
            observe,
            dispatch,
            hold_part if motion.policy.part_hold else None,
            **({"parts": parts} if subset else {}),
        )
    except core.PlaybackBusyError as error:
        raise RuntimeFault(FaultCode.BUSY, str(error)) from error
    except (TypeError, ValueError) as error:
        raise RuntimeFault.from_exception(
            error, code=FaultCode.INVALID_REQUEST
        ) from error
    return PlaybackHandle(native)
