"""One public SDK owner and measured, bounded local joint trials."""

# ruff: noqa: BLE001 -- defer failures until Hold/teardown and evidence are attempted, then re-raise.

import hashlib
import json
import time
from dataclasses import replace
from uuid import uuid4

import numpy as np
from waddle_sdk import load_site
from waddle_sdk.robots.metadata import part_action_spaces
from waddle_sdk.runtime import (
    FaultCode,
    JointPositionCommand,
    RuntimeFault,
    RuntimeFaultCause,
)

from .metrics import duration, endpoint, quintic, write_report


def _recorded_fault(value):
    def cause(row):
        return RuntimeFaultCause(
            row["code"],
            row["detail"],
            row["context"],
            tuple(cause(parent) for parent in row["causes"]),
        )

    return RuntimeFault(
        FaultCode(value["code"]),
        value["detail"],
        value["retryable"],
        value["context"],
        tuple(cause(row) for row in value["causes"]),
    )


class Bench:
    mode = "sdk"

    def __init__(self, config, part=None, *, parts=None):
        self.config = config
        selected = load_site(config["site"])
        manifest = dict(selected.describe())
        # Camera-only tests own their independent lifecycle; joint tests need no
        # camera dependencies. Keep the site identity and robot envelope intact.
        if part is not None and parts is not None:
            raise ValueError("Select part or parts, not both")
        selected_parts = [part] if part is not None else parts
        parts = {
            name: {
                **row,
                "options": {
                    **row.get("options", {}),
                    **config.get("part_options", {}).get(name, {}),
                },
            }
            for name, row in manifest["parts"].items()
            if selected_parts is None or name in selected_parts
        }
        if selected_parts is not None and set(parts) != set(selected_parts):
            raise ValueError("Selected bench parts are absent from the site")
        self.site = replace(
            selected, manifest={**manifest, "parts": parts, "cameras": {}}
        )
        self.session = None
        self._event_cursor = 0
        self._part_faults = {}
        self.report = {
            "schema": "waddle.live-benchmark/v1",
            "run_id": uuid4().hex,
            "site_id": self.site.id,
            "config": config,
            "trials": [],
            "observations": [],
            "cameras": [],
            "mode": self.mode,
            "source_manifest_sha256": hashlib.sha256(
                json.dumps(manifest, sort_keys=True).encode()
            ).hexdigest(),
            "part_configuration": parts,
        }

    def __enter__(self):
        self.session = self.site.open(console=False).__enter__()
        try:
            self.run = self.session.run(
                task={"id": "live-acceptance"}, actor={"id": "local-test"}
            ).__enter__()
            self.spaces = part_action_spaces(self.session.describe())
            self.period = max(
                1 / float(space["rateHz"]) for space in self.spaces.values()
            )
            self.report["command_period_s"] = self.period
            self.report["velocity_feedforward"] = self.config.get(
                "velocity_feedforward", False
            )
            for part in self.spaces:
                if (
                    self.site.manifest["parts"][part].get("driver")
                    == "waddle_sdk.robots.yam:arm"
                ):
                    from .vendor import settings, wait_for_startup

                    driver = self.session._managed.arms[part].driver
                    self.report["control_settings"] = settings(driver._robot)
                    try:
                        wait_for_startup(self, driver._robot, driver.channel)
                    finally:
                        if len(self.spaces) > 1:
                            evidence = self.report.setdefault("part_startup", {})
                            evidence[part] = {
                                "control_settings": self.report.pop("control_settings"),
                                "startup_evidence": self.report.pop(
                                    "startup_evidence", None
                                ),
                            }
            self._check_part_faults()
        except BaseException as error:
            self.__exit__(type(error), error, error.__traceback__)
            raise
        return self

    def save(self):
        directory = self.config["evidence_directory"]
        write_report(
            directory + "/" + self.mode + "-" + self.report["run_id"] + ".json",
            self.report,
        )
        write_report(directory + "/" + self.mode + "-report.json", self.report)

    def __exit__(self, exc_type, exc, tb):
        errors = []
        if self.session is not None:
            try:
                self.session.close(
                    torque_release_authorized=self.config["torque_release_authorized"]
                )
            except BaseException as error:
                errors.append(error)
        self.report["shutdown_errors"] = [
            RuntimeFault.from_exception(error).as_dict() for error in errors
        ]
        try:
            self._check_part_faults()
        except BaseException as error:
            if error is not exc:
                errors.append(error)
        try:
            self.save()
        except BaseException as error:
            errors.append(error)
        if exc is not None:
            for error in errors:
                if hasattr(exc, "add_note"):
                    exc.add_note(
                        f"Additional teardown error: {RuntimeFault.from_exception(error)}"
                    )
        elif errors:
            raise errors[0]

    def _check_part_faults(self, part=None):
        if self.session is None:
            return
        for event in self.session.events(self._event_cursor):
            self._event_cursor = event.cursor
            if event.kind != "robot.part_fault":
                continue
            self.report.setdefault("part_faults", []).append(
                {"cursor": event.cursor, "session_ns": event.session_ns, **event.data}
            )
            self._part_faults.setdefault(
                event.data["part"], _recorded_fault(event.data["fault"])
            )
        selected = self.site.manifest["parts"] if part is None else (part,)
        for name in selected:
            if name in self._part_faults:
                raise self._part_faults[name]

    def positions(self, part):
        return self.run.observe().parts[part].joint_position.copy()

    def observe(self, part):
        observation = self.run.observe()
        state = observation.parts[part]
        self._observation = observation
        telemetry = {
            "joint_velocity": state.joint_velocity.tolist(),
            "session_ns": observation.session_ns,
            "timestamp_provenance": "SDK observation envelope, not motor acquisition",
        }
        if (
            self.site.manifest["parts"][part].get("driver")
            == "waddle_sdk.robots.yam:arm"
        ):
            from .vendor import diagnostics

            telemetry["i2rt"] = diagnostics(
                self.session._managed.arms[part].driver._robot
            )
        return state.joint_position.copy(), telemetry

    def command(self, part, position, velocity):
        observed = self._observation
        action = np.concatenate(
            [
                position if name == part else state.joint_position
                for name, state in observed.parts.items()
            ]
        )
        velocities = np.concatenate(
            [
                velocity if name == part else np.zeros_like(state.joint_position)
                for name, state in observed.parts.items()
            ]
        )
        receipt = self.run.step(
            JointPositionCommand(
                action,
                velocities if self.config.get("velocity_feedforward", False) else None,
            ),
            observed,
        )
        if not receipt.dispatched or receipt.gate != "pass":
            if getattr(receipt, "fault", None) is not None:
                raise receipt.fault
            raise RuntimeFault(
                FaultCode.SAFETY_REFUSAL,
                receipt.detail or "SDK gate refused the command",
                context={
                    "part": part,
                    "gate": receipt.gate,
                    "operation": "stream_joint_target",
                },
            )

    def hold(self):
        self.session.hold(reason="live trial completed or interrupted")

    def pose(self, part, joints):
        return np.asarray(self.session.forward_kinematics(part, joints).position_m)

    def rotation(self, part, joints):
        # Orientation diagnostics remain optional for generic SDK adapters.
        if (
            self.site.manifest["parts"][part].get("driver")
            == "waddle_sdk.robots.yam:arm"
        ):
            from waddle_sdk.robots.yam import forward_kinematics

            return forward_kinematics(joints[:6])[1]
        return None

    def move(self, part, target, case, *, report_key="trials"):
        self._check_part_faults(part)
        initial, _ = self.observe(part)
        names = [row["name"] for row in self.spaces[part]["jointPosition"]["joints"]]
        indices = [names.index(name) for name in case["joint_names"]]
        target_full = initial.copy()
        target_full[indices] = target
        seconds = duration(initial[indices], target, case)
        samples = []
        started = time.monotonic()
        deadline = started + seconds + case["settle_s"]
        minimum_settle = case.get("minimum_settle_s", 0.0)
        target_latched_at = None
        accepted = False
        settled = 0
        measured = initial
        target_pose = self.pose(part, target_full)
        failure = None
        try:
            while time.monotonic() <= deadline:
                now = time.monotonic()
                q, v = quintic(initial, target_full, now - started, seconds)
                measured, telemetry = self.observe(part)
                observed_at = time.monotonic()
                error = float(np.max(np.abs(measured[indices] - target)))
                tracking = float(np.max(np.abs(measured[indices] - q[indices])))
                samples.append(
                    {
                        "elapsed_s": observed_at - started,
                        "observation_latency_s": observed_at - now,
                        "measured_rad": measured[indices].tolist(),
                        "command_rad": q[indices].tolist(),
                        "tracking_error_rad": tracking,
                        "telemetry": telemetry,
                    }
                )
                if tracking > self.config["max_tracking_error_rad"]:
                    raise RuntimeFault(
                        FaultCode.SAFETY_REFUSAL,
                        f"Measured tracking error {tracking:.4f} rad exceeds configured bound",
                        context={
                            "part": part,
                            "operation": "stream_joint_target",
                            "tracking_error_rad": tracking,
                            "max_tracking_error_rad": self.config[
                                "max_tracking_error_rad"
                            ],
                        },
                    )
                command_started = time.monotonic()
                if getattr(self, "_background_error", None) is not None:
                    raise self._background_error
                self._check_part_faults(part)
                self.command(part, q, v)
                command_finished = time.monotonic()
                samples[-1]["command_latency_s"] = command_finished - command_started
                if target_latched_at is None and now - started >= seconds:
                    target_latched_at = command_finished
                arrived = (
                    error <= case["joint_tolerance_rad"]
                    and np.linalg.norm(self.pose(part, measured) - target_pose)
                    <= case["tcp_tolerance_m"]
                )
                settled = (
                    settled + 1 if observed_at - started >= seconds and arrived else 0
                )
                window_complete = minimum_settle == 0 or (
                    target_latched_at is not None
                    and observed_at - target_latched_at >= minimum_settle
                )
                if settled >= 3 and window_complete:
                    accepted = True
                    break
                # Missed periods stay missed: never burst old commands to catch up.
                time.sleep(max(0, now + self.period - time.monotonic()))
            self._check_part_faults(part)
        except BaseException as error:
            failure = error
        motion_elapsed = time.monotonic() - started
        errors = []
        # Stop first, then gather whatever evidence remains available. Reporting
        # and teardown failures must never replace the original motion failure.
        hold_started = time.monotonic()
        try:
            self.hold()
        except BaseException as error:
            errors.append(error)
        hold_elapsed = time.monotonic() - hold_started
        post_hold = None
        try:
            post_hold = self.positions(part)
        except BaseException as error:
            errors.append(error)
        trial = {"mode": self.mode, "part": part, "case_id": case["case_id"]}
        try:
            trial.update(
                endpoint(
                    mode=self.mode,
                    part=part,
                    case={**case, "target_rad": list(target)},
                    initial=initial[indices],
                    measured=measured[indices],
                    target_pose=target_pose,
                    measured_pose=self.pose(part, measured),
                    initial_pose=self.pose(part, initial),
                    elapsed=motion_elapsed,
                    seconds=seconds,
                    samples=samples,
                )
            )
        except BaseException as error:
            errors.append(error)
        trial.update(
            outcome="failed"
            if failure or errors
            else "arrived"
            if accepted
            else "not_arrived",
            settled_samples=settled,
            minimum_settle_s=minimum_settle,
            target_latched_elapsed_s=None
            if target_latched_at is None
            else target_latched_at - started,
            cleanup_errors=[
                RuntimeFault.from_exception(error).as_dict() for error in errors
            ],
            tcp_frame_id=self.site.manifest["parts"][part]["base_frame"],
            arrival_elapsed_s=samples[-1]["elapsed_s"] if accepted else None,
            motion_elapsed_s=motion_elapsed,
            endpoint_measurement_elapsed_s=samples[-1]["elapsed_s"] if samples else 0,
            hold_elapsed_s=hold_elapsed,
            total_elapsed_s=time.monotonic() - started,
            post_hold_measured_rad=None
            if post_hold is None
            else post_hold[indices].tolist(),
        )
        target_rotation = self.rotation(part, target_full)
        if target_rotation is not None:
            from .metrics import rotation_distance

            measured_rotation = self.rotation(part, measured)
            trial.update(
                target_rotation=target_rotation.tolist(),
                measured_rotation=measured_rotation.tolist(),
                orientation_error_rad=rotation_distance(
                    target_rotation, measured_rotation
                ),
                orientation_displacement_rad=rotation_distance(
                    self.rotation(part, initial), measured_rotation
                ),
            )
        if failure:
            trial["error"] = RuntimeFault.from_exception(failure).as_dict()
        self.report.setdefault(report_key, []).append(trial)
        try:
            self.save()
        except BaseException as error:
            errors.append(error)
        if failure:
            for error in errors:
                if hasattr(failure, "add_note"):
                    failure.add_note(
                        f"Additional cleanup error: {RuntimeFault.from_exception(error)}"
                    )
            raise failure
        if errors:
            raise errors[0]
        return trial
