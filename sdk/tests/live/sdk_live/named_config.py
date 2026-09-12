"""Reviewed inputs for optional two-part live acceptance; no device access."""

import math

from .metrics import duration


def positive(row, *keys):
    for key in keys:
        value = row.get(key)
        if (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"named_parts.{key} must be positive and finite")


def cases(config):
    indexed = {(c["part"], c["case_id"]): c for c in config["cases"]}
    return [
        indexed[(ref["part"], ref["case_id"])]
        for ref in config["named_parts"].get("motion_cases", [])
    ]


def validate(config):
    profile = config.get("named_parts")
    if profile is None:
        return
    parts = profile.get("parts", [])
    if (
        len(parts) != 2
        or len(set(parts)) != 2
        or not set(parts) <= set(config["parts"])
    ):
        raise ValueError("named_parts.parts must select exactly two distinct parts")
    positive(
        profile,
        "observation_s",
        "max_observation_latency_s",
        "max_feedback_gap_s",
        "mapping_tolerance_rad",
    )
    if profile["observation_s"] < 2 * profile["max_feedback_gap_s"]:
        raise ValueError("named_parts.observation_s needs two feedback windows")
    for part, factory in profile.get("feedback_probes", {}).items():
        if part not in parts or not isinstance(factory, str) or ":" not in factory:
            raise ValueError("feedback_probes maps selected parts to module:factory")
    if profile.get("motion_cases"):
        try:
            first, neighbor, repeat = cases(config)
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                "motion_cases needs three existing part/case_id references"
            ) from error
        if [first["part"], neighbor["part"]] != parts or repeat["part"] != parts[0]:
            raise ValueError("motion_cases must address first, neighbor, first parts")
        first_target = dict(zip(first["joint_names"], first["target_rad"], strict=True))
        repeat_start = dict(
            zip(repeat["joint_names"], repeat["start_rad"], strict=True)
        )
        if first_target != repeat_start:
            raise ValueError("repeat start must equal the first target by joint name")
        positive(
            profile, "max_command_latency_s", "max_command_gap_s", "min_progress_rad"
        )
        if not set(parts) <= set(config.get("rest_positions", {})):
            raise ValueError(
                "named motion requires reviewed rest_positions for both parts"
            )
        first_duration = duration(first["start_rad"], first["target_rad"], first)
        neighbor_duration = duration(
            neighbor["start_rad"], neighbor["target_rad"], neighbor
        )
        if first_duration + first["settle_s"] >= neighbor_duration:
            raise ValueError(
                "neighbor duration must exceed the first trajectory plus settling budget"
            )
    for stop in profile.get("stops", []):
        if not profile.get("motion_cases"):
            raise ValueError("named stops require motion_cases")
        if (
            stop.get("method") not in ("hold", "estop")
            or stop.get("supported") is not True
        ):
            raise ValueError("stop needs hold/estop and explicit supported=true")
        positive(
            stop,
            "after_s",
            "response_s",
            "observe_s",
            "max_velocity_rad_s",
            "max_drift_rad",
            "min_velocity_rad_s",
        )
        if stop["observe_s"] <= stop["response_s"]:
            raise ValueError("stop observe_s must exceed response_s")
        if stop["min_velocity_rad_s"] <= stop["max_velocity_rad_s"]:
            raise ValueError(
                "stop minimum moving velocity must exceed stopped velocity"
            )
        if stop["after_s"] >= min(
            duration(c["start_rad"], c["target_rad"], c) for c in cases(config)[:2]
        ):
            raise ValueError("stop after_s must interrupt both trajectories")
    envelope = profile.get("envelope")
    if envelope is not None:
        if envelope.get("part") not in parts or not envelope.get("joint_name"):
            raise ValueError("envelope needs a selected part and joint_name")
        target = envelope.get("rejected_position_rad")
        if (
            isinstance(target, bool)
            or not isinstance(target, (int, float))
            or not math.isfinite(target)
        ):
            raise ValueError("envelope rejected_position_rad must be finite")
        positive(envelope, "observe_s", "max_drift_rad")
