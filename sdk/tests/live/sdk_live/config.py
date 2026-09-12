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
    rests = config.get("rest_positions", {})
    if not isinstance(rests, dict):
        raise TypeError("rest_positions must map selected parts to named positions")
    for part, rest in rests.items():
        names = rest.get("joint_names") if isinstance(rest, dict) else None
        positions = rest.get("position_rad") if isinstance(rest, dict) else None
        if (
            part not in config["parts"]
            or not isinstance(names, list)
            or not names
            or not all(isinstance(name, str) and name for name in names)
            or len(set(names)) != len(names)
            or not isinstance(positions, list)
            or len(names) != len(positions)
            or not all(
                isinstance(value, (int, float))
                and not isinstance(value, bool)
                and math.isfinite(value)
                for value in positions
            )
        ):
            raise ValueError(
                f"rest_positions.{part} needs unique joint_names and matching finite position_rad"
            )
    seen = set()
    for case in config["cases"]:
        identity = case["part"], case["case_id"]
        if identity in seen or case["part"] not in config["parts"]:
            raise ValueError("Duplicate case or unselected part")
        seen.add(identity)
        start, target = case["start_rad"], case["target_rad"]
        names = case["joint_names"]
        if case["part"] in rests and set(rests[case["part"]]["joint_names"]) != set(
            names
        ):
            raise ValueError(
                f"rest_positions.{case['part']} must name the same joints as its cases"
            )
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
        if "minimum_orientation_rad" in case:
            angle = case["minimum_orientation_rad"]
            if not math.isfinite(angle) or angle < 0:
                raise ValueError(
                    "minimum_orientation_rad must be nonnegative and finite"
                )
    if config["cases"]:
        tracking = config["max_tracking_error_rad"]
        if not math.isfinite(tracking) or tracking <= 0:
            raise ValueError("max_tracking_error_rad must be positive and finite")
    comparison = config.get("comparison")
    if comparison is not None:
        if comparison.get("vendor") != "i2rt":
            raise ValueError("The available raw comparison adapter is i2rt")
        for key in (
            "initial_tolerance_rad",
            "joint_error_margin_rad",
            "tcp_error_margin_m",
            "settling_margin_s",
        ):
            if not math.isfinite(comparison[key]) or comparison[key] < 0:
                raise ValueError(f"comparison.{key} must be nonnegative and finite")
    config["config_path"] = str(path)
    from .named_config import validate

    validate(config)
    return config


def reject_ci():
    if any(
        os.environ.get(key, "").lower() not in ("", "0", "false")
        for key in ("CI", "GITHUB_ACTIONS")
    ):
        raise ValueError(
            "Live device/service tests are disabled in CI and release jobs"
        )
