"""Local owner adaptation for native playback; no Python timing loop."""

from __future__ import annotations

import json
from collections.abc import Sequence
from typing import TYPE_CHECKING

from ._native import core
from ._playback_bounds import observe as observe_bounded
from ._playback_history import History
from .robots import base
from .runtime import (
    CoordinatedJointMotion,
    FaultCode,
    FiniteJointMotion,
    JointPositionCommand,
    PlaybackCurveUpdate,
    PlaybackDependency,
    PlaybackGoal,
    PlaybackGoalDwell,
    PlaybackGoalUpdate,
    Pose,
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

    def __init__(self, native, history):
        self._native = native
        self._history = history

    def history(self):
        """Copy bounded SDK command-call diagnostics without waiting for I/O."""
        return self._history.snapshot()

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

    def finish_references(self) -> None:
        """Queue normal completion; wait for a terminal native snapshot."""
        try:
            self._native.finish_references()
        except (TypeError, ValueError, OverflowError) as error:
            raise RuntimeFault.from_exception(
                error, code=FaultCode.INVALID_REQUEST
            ) from error

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

    def update_goal(self, update: PlaybackGoalUpdate) -> int:
        """Queue one pose target; retain its native acceptance or refusal."""
        if not isinstance(update, PlaybackGoalUpdate):
            raise RuntimeFault(
                FaultCode.INVALID_REQUEST, "provide a PlaybackGoalUpdate"
            )
        try:
            return self._native.update_goal(update)
        except core.PlaybackBusyError as error:
            raise RuntimeFault(FaultCode.BUSY, str(error)) from error
        except (TypeError, ValueError, OverflowError) as error:
            raise RuntimeFault.from_exception(
                error, code=FaultCode.INVALID_REQUEST
            ) from error


def start(
    run: Run,
    motion: FiniteJointMotion | CoordinatedJointMotion,
    *,
    after: Sequence[PlaybackDependency] = (),
    start_before_ns: int | None = None,
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
        after or start_before_ns is not None
    ) and SupportFact.DEPENDENT_PLAYBACK not in facts:
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support dependent playback"
        )
    if any(
        not isinstance(dependency, PlaybackDependency)
        or not isinstance(dependency.handle, PlaybackHandle)
        for dependency in after
    ):
        raise RuntimeFault(
            FaultCode.INVALID_REQUEST, "dependencies require local playback handles"
        )
    goal = motion.policy.goal
    if (goal is not None or motion.policy.settle_timeout_ns is not None) and (
        SupportFact.GOAL_PLAYBACK not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support local goal completion"
        )
    if goal is not None and not isinstance(goal, PlaybackGoal):
        raise RuntimeFault(FaultCode.INVALID_REQUEST, "provide a PlaybackGoal")
    if (
        motion.policy.allow_curve_updates
        and not motion.policy.reference_only
        and goal is None
        and SupportFact.REVISABLE_SETTLED_PLAYBACK not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support revisable settled playback"
        )
    if (
        goal is not None
        and goal.allow_updates
        and SupportFact.UPDATABLE_GOAL_PLAYBACK not in facts
    ):
        raise RuntimeFault(FaultCode.UNSUPPORTED, "part does not support goal updates")
    if goal is not None and goal.dwell is not None:
        if not isinstance(goal.dwell, PlaybackGoalDwell):
            raise RuntimeFault(FaultCode.INVALID_REQUEST, "provide a PlaybackGoalDwell")
        if SupportFact.DWELL_GOAL_PLAYBACK not in facts:
            raise RuntimeFault(
                FaultCode.UNSUPPORTED, "part does not support goal dwell"
            )
    if (
        motion.policy.complete_in_resting_tail
        and SupportFact.RESTING_GOAL_PLAYBACK not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support resting-tail goal completion"
        )
    kinematics = None
    if goal is not None and goal.kinematics is not None:
        if SupportFact.KINEMATIC_GOAL_PLAYBACK not in facts:
            raise RuntimeFault(
                FaultCode.UNSUPPORTED, "part does not support selected TCP goals"
            )
        if group or goal.pose is None:
            raise RuntimeFault(
                FaultCode.INVALID_REQUEST, "kinematics requires a named TCP goal"
            )
        from ._playback_kinematics import Kinematics

        try:
            kinematics = Kinematics(
                goal.kinematics,
                width=len(arms[motion.part].joint_names),
                frame=arms[motion.part].base_frame,
            )
        except (TypeError, ValueError, OverflowError) as error:
            raise RuntimeFault.from_exception(
                error, code=FaultCode.INVALID_REQUEST
            ) from error
    if (
        goal is not None
        and motion.policy.allow_curve_updates
        and SupportFact.REVISABLE_GOAL_PLAYBACK not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support revisable goal playback"
        )
    if goal is not None and goal.pose is not None:
        if (
            group
            or kinematics is None
            and SupportFact.CARTESIAN_GOAL_PLAYBACK not in facts
        ):
            raise RuntimeFault(
                FaultCode.UNSUPPORTED, "TCP goal requires a named part with local FK"
            )
        if (
            not isinstance(goal.pose, Pose)
            or goal.pose.frame_id != arms[motion.part].base_frame
        ):
            raise RuntimeFault(
                FaultCode.INVALID_REQUEST,
                "TCP goal must use the SDK part's declared base frame",
            )
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
    if (
        not after
        and run._episode.playback_active
        and (group and not subset or SupportFact.PARALLEL_JOINT_PLAYBACK not in facts)
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
        motion.policy.defer_reference_completion
        and SupportFact.DEFERRED_REFERENCE_COMPLETION not in facts
    ):
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "part does not support deferred reference completion"
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
            FaultCode.UNSUPPORTED,
            "part does not support reference-only window evidence",
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
                pose = None
                if goal is not None and goal.pose is not None:
                    sample = measured[0]
                    if kinematics is not None:
                        pose = kinematics(sample.joint_position)
                    elif (
                        sample.ee_pose_wxyz is None
                        or sample.frame_id != goal.pose.frame_id
                    ):
                        raise RuntimeFault(
                            FaultCode.INTERNAL,
                            "local TCP feedback is unavailable or has the wrong frame",
                        )
                    else:
                        pose = Pose(
                            tuple(sample.ee_pose_wxyz[:3]),
                            tuple(sample.ee_pose_wxyz[3:]),
                            sample.frame_id,
                        )
                return {
                    "observed_ns": observation.session_ns,
                    "positions": [
                        q for row in measured for q in row.joint_position.tolist()
                    ],
                    "velocities": [
                        v for row in measured for v in row.joint_velocity.tolist()
                    ],
                    "gripper_force": None if group else measured[0].gripper_force,
                    "pose": pose,
                }
        except Exception as error:  # noqa: BLE001 -- retain any vendor fault across the native callback
            return _error(error, unknown=False)

    history = History(
        run.id,
        {name: list(arms[name].joint_names) for name in parts},
        session._require().core.stamp,
    )

    def dispatch(reference, measured, force_n):
        sequence = None
        try:
            with session._dispatch_lock:
                started = session._require().core.stamp()
                sequence = history.begin(started, reference, measured, force_n)
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
                fault = result.fault or RuntimeFault(
                    FaultCode.SAFETY_REFUSAL, result.detail
                )
                history.end(
                    sequence, result="error", gate=result.gate, error=fault.as_dict()
                )
                return _error(
                    fault,
                    unknown=False,
                )
            history.end(
                sequence,
                result="acknowledged"
                if result.dispatched and result.gate == "pass"
                else "superseded"
                if result.gate != "pass"
                else "refused",
                gate=result.gate,
            )
            return {
                "dispatched": result.dispatched and result.gate == "pass",
                "detail": result.gate,
                "started_ns": started.session_ns,
            }
        except Exception as error:  # noqa: BLE001 -- an unexpected write failure must retain uncertainty
            # A raising write may have reached the device. Preserve its evidence,
            # never retry, and do not confuse local loop drain with physical stop.
            if sequence is not None:
                history.end(
                    sequence,
                    result="error",
                    error=RuntimeFault.from_exception(error).as_dict(),
                )
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
            after=[
                (dependency.handle._native, dependency.outcomes) for dependency in after
            ],
            start_before_ns=start_before_ns,
            **({"parts": parts} if subset else {}),
        )
    except core.PlaybackBusyError as error:
        raise RuntimeFault(FaultCode.BUSY, str(error)) from error
    except (TypeError, ValueError, OverflowError) as error:
        raise RuntimeFault.from_exception(
            error, code=FaultCode.INVALID_REQUEST
        ) from error
    return PlaybackHandle(native, history)
