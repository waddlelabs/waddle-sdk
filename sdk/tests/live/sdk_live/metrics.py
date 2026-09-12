"""Identical sampled trajectories and measurement arithmetic for paired trials."""

import json
import math
import os
import tempfile
from pathlib import Path

import numpy as np


def duration(start, target, case):
    distance = float(np.max(np.abs(np.asarray(target) - start)))
    return max(
        1.0,
        1.875 * distance / case["velocity_rad_s"],
        math.sqrt(5.774 * distance / case["acceleration_rad_s2"]),
        (60 * distance / case["jerk_rad_s3"]) ** (1 / 3),
    )


def quintic(start, target, elapsed, seconds):
    u = float(np.clip(elapsed / seconds, 0, 1))
    distance = np.asarray(target) - start
    return (
        np.asarray(start) + distance * (10 * u**3 - 15 * u**4 + 6 * u**5),
        distance * (30 * u**2 - 60 * u**3 + 30 * u**4) / seconds,
    )


def endpoint(
    *,
    mode,
    part,
    case,
    initial,
    measured,
    target_pose,
    measured_pose,
    initial_pose,
    elapsed,
    seconds,
    samples,
):
    return {
        "mode": mode,
        "part": part,
        "case_id": case["case_id"],
        "start_rad": list(initial),
        "target_rad": case["target_rad"],
        "measured_rad": list(measured),
        "joint_errors_rad": np.abs(np.asarray(measured) - case["target_rad"]).tolist(),
        "joint_error_rad": float(
            np.max(np.abs(np.asarray(measured) - case["target_rad"]))
        ),
        "tcp_error_m": float(np.linalg.norm(np.asarray(measured_pose) - target_pose)),
        "displacement_m": float(
            np.linalg.norm(np.asarray(measured_pose) - initial_pose)
        ),
        "elapsed_s": elapsed,
        "trajectory_duration_s": seconds,
        "sample_count": len(samples),
        "samples": samples,
        "initial_tcp_m": list(initial_pose),
        "target_tcp_m": list(target_pose),
        "measured_tcp_m": list(measured_pose),
    }


def rotation_distance(first, second):
    cosine = (np.trace(np.asarray(first).T @ np.asarray(second)) - 1) / 2
    return float(np.arccos(np.clip(cosine, -1, 1)))


def write_report(path, report):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(dir=path.parent, prefix=".report-", suffix=".json")
    try:
        with os.fdopen(fd, "w") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
        os.replace(name, path)
    finally:
        Path(name).unlink(missing_ok=True)


def assert_arrived(trial, case):
    assert trial["outcome"] == "arrived", trial
    assert trial["joint_error_rad"] <= case["joint_tolerance_rad"], trial
    assert trial["tcp_error_m"] <= case["tcp_tolerance_m"], trial
    assert trial["displacement_m"] >= case["min_displacement_m"], trial
    if "minimum_orientation_rad" in case:
        assert (
            trial["orientation_displacement_rad"] >= case["minimum_orientation_rad"]
        ), trial
