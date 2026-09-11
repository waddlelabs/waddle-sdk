"""Explicit local bench inputs. Loading these files never opens a device."""

import json
import math
import os
from pathlib import Path


def load(path):
    path = Path(path).resolve()
    config = json.loads(path.read_text())
    if config.get("schema") != "waddle.live-bench/v1":
        raise ValueError("Expected waddle.live-bench/v1")
    for key in ("site", "evidence_directory"):
        value = Path(config[key]).expanduser()
        config[key] = str(value if value.is_absolute() else path.parent / value)
    config.setdefault("parts", [])
    config.setdefault("cameras", [])
    config.setdefault("cases", [])
    seen = set()
    for case in config["cases"]:
        identity = case["part"], case["case_id"]
        if identity in seen or case["part"] not in config["parts"]:
            raise ValueError("Duplicate case or unselected part")
        seen.add(identity)
        start, target = case["start_rad"], case["target_rad"]
        names = case["joint_names"]
        if len(names) != len(start) or len(set(names)) != len(names):
            raise ValueError("Joint names must uniquely identify each target axis")
        if not start or len(start) != len(target):
            raise ValueError("Target and reference joint widths must match")
        if not all(math.isfinite(v) for v in start + target):
            raise ValueError("Joint targets must be finite")
        for key in (
            "velocity_rad_s",
            "acceleration_rad_s2",
            "jerk_rad_s3",
            "joint_tolerance_rad",
            "tcp_tolerance_m",
            "settle_s",
        ):
            if not math.isfinite(case[key]) or case[key] <= 0:
                raise ValueError(f"{key} must be positive and finite")
        displacement = case["min_displacement_m"]
        if not math.isfinite(displacement) or displacement < 0:
            raise ValueError("min_displacement_m must be nonnegative and finite")
    if config["cases"]:
        tracking = config["max_tracking_error_rad"]
        if not math.isfinite(tracking) or tracking <= 0:
            raise ValueError("max_tracking_error_rad must be positive and finite")
    config["config_path"] = str(path)
    return config


def reject_ci():
    if any(
        os.environ.get(key, "").lower() not in ("", "0", "false")
        for key in ("CI", "GITHUB_ACTIONS")
    ):
        raise ValueError(
            "Live device/service tests are disabled in CI and release jobs"
        )
