"""Measured independent streams through public named SDK ports, for live tests.

This is test stimulus, not a trajectory API. All terminal decisions use encoders;
only explicit stop acceptance and test teardown issue shared supervision verbs.
"""

import time

import numpy as np
import pytest
from waddle_sdk.runtime import JointPositionCommand, RuntimeFault, SupportFact

from . import feedback
from .metrics import duration, quintic
from .session import Bench


class NamedBench(Bench):
    mode = "sdk-named"

    def __init__(self, config):
        self.profile = config["named_parts"]
        self.parts = self.profile["parts"]
        super().__init__(config, parts=self.parts)
        self.probes = {}
        self.fresh = {}
        self.last_commands = {}
        self.report["named"] = {"samples": [], "submissions": [], "arrivals": []}

    def require(self, *, motion=False, action=False, stop=None):
        required = {SupportFact.PART_OBSERVATION}
        if motion:
            required.add(SupportFact.FORWARD_KINEMATICS)
        if motion or action:
            required |= {SupportFact.PART_ACTION, SupportFact.SEND_GRANT}
        if stop:
            required.add(
                SupportFact.HOLD_GRANT if stop == "hold" else SupportFact.ESTOP_GRANT
            )
        support = self.session.support()
        self.report["support"] = support.as_dict()
        rows = {row.scope: set(row.facts) for row in support.rows}
        for part in self.parts:
            absent = required - rows.get(f"robot:{part}", set())
            if absent:
                pytest.skip(f"{part} lacks {sorted(f.value for f in absent)}")
            path = feedback.factory_path(self.config, self.site.manifest, part)
            if path is None:
                pytest.skip(f"{part} needs a device feedback probe")
            self.probes[part] = feedback.create(path, self, part)

    def __exit__(self, exc_type, exc, tb):
        if isinstance(exc, pytest.skip.Exception):
            self.report["skipped"] = str(exc)
        elif exc is not None:
            self.report["error"] = RuntimeFault.from_exception(exc).as_dict()
        # SiteSession.close owns the existing Hold/support/torque-release path.
        return super().__exit__(exc_type, exc, tb)

    def names(self, part):
        return [row["name"] for row in self.spaces[part]["jointPosition"]["joints"]]

    def read(self, parts=None):
        parts = self.parts if parts is None else parts
        for part in parts:
            self._check_part_faults(part)
        before = time.monotonic()
        devices = {part: self.probes[part].sample() for part in parts}
        observation = self.session.observe_parts(parts)
        now = time.monotonic()
        sample = {
            "time_s": now,
            "session_ns": observation.session_ns,
            "observation_latency_s": now - before,
            "devices": devices,
            "parts": {
                part: {
                    "joint_names": self.names(part),
                    "frame_id": state.frame_id,
                    "position_rad": state.joint_position.tolist(),
                    "velocity_rad_s": state.joint_velocity.tolist(),
                }
                for part, state in observation.parts.items()
            },
            "faults": {
                part: error.as_dict() for part, error in observation.faults.items()
            },
        }
        self.report["named"]["samples"].append(sample)
        if observation.faults:
            raise next(iter(observation.faults.values()))
        assert set(observation.parts) == set(parts), sample
        assert now - before <= self.profile["max_observation_latency_s"], sample
        for part, state in observation.parts.items():
            names = self.names(part)
            assert (
                state.joint_position.shape
                == state.joint_velocity.shape
                == (len(names),)
            ), sample
            assert (
                np.isfinite(state.joint_position).all()
                and np.isfinite(state.joint_velocity).all()
            ), sample
            assert state.frame_id == self.site.manifest["parts"][part]["base_frame"], (
                sample
            )
            device = devices[part]
            indices = [names.index(name) for name in device["joint_names"]]
            assert indices and len(set(indices)) == len(indices), sample
            assert len(indices) == len(device["position_rad"]), sample
            assert (
                np.max(np.abs(state.joint_position[indices] - device["position_rad"]))
                <= self.profile["mapping_tolerance_rad"]
            ), sample
            for key in ("generation", "ingestion"):
                counter = device[key]
                assert (
                    isinstance(counter, int)
                    and not isinstance(counter, bool)
                    and counter >= 0
                ), sample
                identity = part, key
                previous, changed, advances = self.fresh.get(
                    identity, (counter, now, 0)
                )
                assert counter >= previous, sample
                if counter > previous:
                    changed, advances = now, advances + 1
                assert now - changed <= self.profile["max_feedback_gap_s"], sample
                self.fresh[identity] = counter, changed, advances
        return observation

    def check_fresh(self):
        assert all(
            self.fresh.get((part, key), (0, 0, 0))[2] >= 2
            for part in self.parts
            for key in ("generation", "ingestion")
        ), self.fresh

    def submit(self, commands, observed, *, refusal=False):
        for part in commands:
            self._check_part_faults(part)
        started = time.monotonic()
        receipts = self.run.step_parts(commands, observed)
        finished = time.monotonic()
        row = {
            "time_s": started,
            "latency_s": finished - started,
            "commands": {p: list(c.positions) for p, c in commands.items()},
            "receipts": {
                p: {
                    "dispatched": r.dispatched,
                    "gate": r.gate,
                    "part": r.part,
                    "detail": r.detail,
                    "fault": r.fault.as_dict() if r.fault else None,
                }
                for p, r in receipts.items()
            },
        }
        self.report["named"]["submissions"].append(row)
        assert set(receipts) == set(commands), row
        if refusal:
            return receipts
        for part, receipt in receipts.items():
            if receipt.fault is not None:
                raise receipt.fault
            assert (
                receipt.dispatched and receipt.gate == "pass" and receipt.part == part
            ), row
            # A newly started phase has no inter-command interval yet.
            if part in self.last_commands:
                gap = started - self.last_commands[part]
                row.setdefault("command_gaps_s", {})[part] = gap
                assert gap <= self.profile["max_command_gap_s"], row
            self.last_commands[part] = started
        assert finished - started <= self.profile["max_command_latency_s"], row
        return receipts

    def motion(self, case, observed, *, target=None):
        part = case["part"]
        start = observed.parts[part].joint_position.copy()
        indices = [self.names(part).index(name) for name in case["joint_names"]]
        goal = start.copy()
        goal[indices] = case["target_rad"] if target is None else target
        self.last_commands.pop(part, None)
        return {
            "case": case,
            "initial": start,
            "target": goal,
            "indices": indices,
            "start_s": time.monotonic(),
            "duration_s": duration(start[indices], goal[indices], case),
            "settled": 0,
            "latched_s": None,
        }

    def tick(self, motions):
        observed = self.read()
        now = time.monotonic()
        commands, arrived = {}, []
        for part, motion in motions.items():
            case, target = motion["case"], motion["target"]
            elapsed = now - motion["start_s"]
            assert elapsed <= motion["duration_s"] + case["settle_s"], (
                f"{part}: measured arrival timed out"
            )
            q, velocity = quintic(
                motion["initial"], target, elapsed, motion["duration_s"]
            )
            state = observed.parts[part].joint_position
            tracking = float(np.max(np.abs(state - q)))
            self.report["named"]["samples"][-1]["parts"][part]["tracking_error_rad"] = (
                tracking
            )
            assert tracking <= self.config["max_tracking_error_rad"], (
                f"{part}: tracking error {tracking}"
            )
            at_target = (
                np.max(np.abs(state - target)) <= case["joint_tolerance_rad"]
                and np.linalg.norm(self.pose(part, state) - self.pose(part, target))
                <= case["tcp_tolerance_m"]
            )
            self.report["named"]["samples"][-1]["parts"][part]["target_error_rad"] = (
                float(np.max(np.abs(state - target)))
            )
            motion["settled"] = (
                motion["settled"] + 1
                if motion["latched_s"] is not None and at_target
                else 0
            )
            if motion["settled"] >= 3 and now - motion["latched_s"] >= case.get(
                "minimum_settle_s", 0
            ):
                displacement = float(
                    np.linalg.norm(
                        self.pose(part, state) - self.pose(part, motion["initial"])
                    )
                )
                assert displacement >= case["min_displacement_m"], (
                    f"{part}: insufficient measured displacement"
                )
                if "minimum_orientation_rad" in case:
                    from .metrics import rotation_distance

                    first, last = (
                        self.rotation(part, motion["initial"]),
                        self.rotation(part, state),
                    )
                    assert first is not None and last is not None, (
                        "orientation evidence unavailable"
                    )
                    assert (
                        rotation_distance(first, last)
                        >= case["minimum_orientation_rad"]
                    )
                self.report["named"]["arrivals"].append(
                    {
                        "part": part,
                        "case_id": case["case_id"],
                        "time_s": now,
                        "target_rad": target.tolist(),
                        "measured_rad": state.tolist(),
                        "joint_error_rad": float(np.max(np.abs(state - target))),
                        "tcp_error_m": float(
                            np.linalg.norm(
                                self.pose(part, state) - self.pose(part, target)
                            )
                        ),
                        "displacement_m": displacement,
                    }
                )
                arrived.append(part)
                continue
            commands[part] = JointPositionCommand(
                q, velocity if self.config.get("velocity_feedforward", False) else None
            )
        if commands:
            self.submit(commands, observed)
            for part in commands:
                motion = motions[part]
                if (
                    now - motion["start_s"] >= motion["duration_s"]
                    and motion["latched_s"] is None
                ):
                    motion["latched_s"] = time.monotonic()
        return observed, arrived

    def pause(self, started):
        time.sleep(max(0, started + self.period - time.monotonic()))

    def finish(self, motions):
        while motions:
            started = time.monotonic()
            _, arrived = self.tick(motions)
            for part in arrived:
                del motions[part]
            self.pause(started)

    def references(self, cases):
        # Reviewed reference acquisition is sequential within the same site.
        for case in cases:
            reference = {**case, "min_displacement_m": 0}
            reference.pop("minimum_orientation_rad", None)
            self.finish(
                {
                    case["part"]: self.motion(
                        reference, self.read(), target=case["start_rad"]
                    )
                }
            )

    def rest(self, cases):
        for case in cases:
            rest = self.config["rest_positions"][case["part"]]
            reference = {
                **case,
                "joint_names": rest["joint_names"],
                "min_displacement_m": 0,
            }
            reference.pop("minimum_orientation_rad", None)
            self.finish(
                {
                    case["part"]: self.motion(
                        reference, self.read(), target=rest["position_rad"]
                    )
                }
            )
