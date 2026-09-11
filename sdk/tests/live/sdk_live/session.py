"""One public SDK owner and measured, bounded local joint trials."""

# ruff: noqa: BLE001 -- defer failures until Hold/teardown and evidence are attempted, then re-raise.

import time
from dataclasses import replace
from uuid import uuid4

import numpy as np
from waddle_sdk import load_site
from waddle_sdk.robots.metadata import part_action_spaces
from waddle_sdk.runtime import JointPositionCommand, RuntimeFault

from .metrics import duration, endpoint, quintic, write_report


class Bench:
    def __init__(self, config):
        self.config = config
        selected = load_site(config["site"])
        # Camera-only tests own their independent lifecycle; joint tests need no
        # camera dependencies. Keep the site identity and robot envelope intact.
        self.site = replace(selected, manifest={**selected.manifest, "cameras": {}})
        self.session = None
        self.report = {
            "schema": "waddle.live-benchmark/v1",
            "run_id": uuid4().hex,
            "site_id": self.site.id,
            "config": config,
            "trials": [],
            "observations": [],
            "cameras": [],
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
        except BaseException as error:
            self.__exit__(type(error), error, error.__traceback__)
            raise
        return self

    def save(self):
        directory = self.config["evidence_directory"]
        write_report(directory + "/sdk-" + self.report["run_id"] + ".json", self.report)
        write_report(directory + "/sdk-report.json", self.report)

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

    def positions(self, part):
        return self.run.observe().parts[part].joint_position.copy()

    def pose(self, part, joints):
        return np.asarray(self.session.forward_kinematics(part, joints).position_m)

    def move(self, part, target, case, *, record=True):
        before = self.run.observe()
        initial = before.parts[part].joint_position.copy()
        names = [row["name"] for row in self.spaces[part]["jointPosition"]["joints"]]
        indices = [names.index(name) for name in case["joint_names"]]
        target_full = initial.copy()
        target_full[indices] = target
        seconds = duration(initial[indices], target, case)
        fixed = {
            name: state.joint_position.copy() for name, state in before.parts.items()
        }
        samples = []
        started = time.monotonic()
        deadline = started + seconds + case["settle_s"]
        ticks = 0
        settled = 0
        measured = initial
        target_pose = self.pose(part, target_full)
        failure = None
        try:
            while time.monotonic() <= deadline:
                now = time.monotonic()
                q, v = quintic(initial, target_full, now - started, seconds)
                observed = self.run.observe()
                measured = observed.parts[part].joint_position.copy()
                fixed[part] = q
                action = np.concatenate([fixed[name] for name in observed.parts])
                velocity = np.concatenate(
                    [
                        v if name == part else np.zeros_like(fixed[name])
                        for name in observed.parts
                    ]
                )
                command = JointPositionCommand(
                    action,
                    velocity
                    if self.config.get("velocity_feedforward", False)
                    else None,
                )
                receipt = self.run.step(command, observed)
                if not receipt.dispatched or receipt.gate != "pass":
                    raise AssertionError(f"SDK dispatch refused: {receipt}")
                error = float(np.max(np.abs(measured[indices] - target)))
                tracking = float(np.max(np.abs(measured[indices] - q[indices])))
                samples.append(
                    {
                        "elapsed_s": now - started,
                        "measured_rad": measured[indices].tolist(),
                        "command_rad": q[indices].tolist(),
                        "tracking_error_rad": tracking,
                    }
                )
                if tracking > self.config["max_tracking_error_rad"]:
                    raise AssertionError(
                        f"Measured tracking error {tracking:.4f} rad exceeds configured bound"
                    )
                arrived = (
                    error <= case["joint_tolerance_rad"]
                    and np.linalg.norm(self.pose(part, measured) - target_pose)
                    <= case["tcp_tolerance_m"]
                )
                settled = settled + 1 if now - started >= seconds and arrived else 0
                if settled >= 3:
                    break
                ticks += 1
                time.sleep(max(0, started + ticks * self.period - time.monotonic()))
        except BaseException as error:
            failure = error
        errors = []
        # Stop first, then gather whatever evidence remains available. Reporting
        # and teardown failures must never replace the original motion failure.
        try:
            self.session.hold(reason="live trial completed or interrupted")
        except BaseException as error:
            errors.append(error)
        try:
            measured = self.positions(part)
        except BaseException as error:
            errors.append(error)
        trial = {"mode": "sdk", "part": part, "case_id": case["case_id"]}
        try:
            trial.update(
                endpoint(
                    mode="sdk",
                    part=part,
                    case={**case, "target_rad": list(target)},
                    initial=initial[indices],
                    measured=measured[indices],
                    target_pose=target_pose,
                    measured_pose=self.pose(part, measured),
                    initial_pose=self.pose(part, initial),
                    elapsed=time.monotonic() - started,
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
            if settled >= 3
            else "not_arrived",
            settled_samples=settled,
            cleanup_errors=[
                RuntimeFault.from_exception(error).as_dict() for error in errors
            ],
            tcp_frame_id=self.site.manifest["parts"][part]["base_frame"],
        )
        if failure:
            trial["error"] = RuntimeFault.from_exception(failure).as_dict()
        if record:
            self.report["trials"].append(trial)
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
