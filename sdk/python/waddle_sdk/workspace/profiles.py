"""Editable configuration suggestions from explicit public SDK profiles."""

from __future__ import annotations

import hashlib
import re
from importlib.resources import files
from pathlib import Path

from .sources import InitializationError, _write

YAM_DRIVER = "waddle_sdk.robots.yam:arm"

_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
SIMULATION_DRIVER = "waddle_sdk.robots.mujoco:arm"
SIMULATION_JOINTS = ("shoulder", "elbow")


def checked_name(value: str, label: str = "name") -> str:
    if not isinstance(value, str) or not _NAME.fullmatch(value):
        raise ValueError(
            f"{label} must use 1–128 letters, digits, dots, underscores or hyphens"
        )
    return value


def simulation_directory(part: str) -> str:
    return (
        "models/simulation-"
        + hashlib.sha256(checked_name(part).encode()).hexdigest()[:16]
    )


def part_defaults(profile: str, name: str) -> dict:
    """Return an editable SDK part row; this never constructs a driver."""
    checked_name(name, "part name")
    frame = checked_name(name + "_base", "base frame")
    if profile == "yam":
        from waddle_sdk.robots.yam import (
            DEFAULT_ARM_GAINS,
            DEFAULT_CONTROL_JOINT_LIMITS,
            DEFAULT_MAX_JOINT_POSITION_ERROR_RAD,
            DEFAULT_RATE_HZ,
            GRIPPER_MAX_OPENING_M,
            JOINT_NAMES,
        )

        return {
            "driver": YAM_DRIVER,
            "posture": "supervised",
            "base_frame": frame,
            "connection": {
                "channel": "",
                "configure_can": True,
                "can_bitrate": 1_000_000,
            },
            "joint_limits": {
                joint: list(bounds)
                for joint, bounds in zip(
                    JOINT_NAMES, DEFAULT_CONTROL_JOINT_LIMITS, strict=True
                )
            },
            "gripper": {
                "joint": "gripper",
                "closed_m": 0.0,
                "open_m": GRIPPER_MAX_OPENING_M,
                "closed_action": 0.0,
                "open_action": 1.0,
            },
            "options": {
                "velocity_feedforward": True,
                "rate_hz": DEFAULT_RATE_HZ,
                "arm_gains": {
                    key: list(values) for key, values in DEFAULT_ARM_GAINS.items()
                },
                "max_joint_position_error_rad": list(
                    DEFAULT_MAX_JOINT_POSITION_ERROR_RAD
                ),
            },
        }
    if profile == "simulation":
        return {
            "driver": SIMULATION_DRIVER,
            "posture": "supervised",
            "base_frame": frame,
            "connection": {"model": simulation_directory(name) + "/model.xml"},
            "joint_limits": {joint: [-1.0, 1.0] for joint in SIMULATION_JOINTS},
            "options": {
                "actuator_names": ["shoulder_motor", "elbow_motor"],
                "home": [0.0, 0.0],
                "rate_hz": 20.0,
                "max_joint_speed_per_s": 1.0,
                "tool_site": "tcp",
                "collision_bodies": [
                    {
                        "name": "upper",
                        "body": "upper",
                        "center_m": [0.04, 0.0, 0.0],
                        "radius_m": 0.048,
                    },
                    {
                        "name": "forearm",
                        "body": "forearm",
                        "center_m": [0.04, 0.0, 0.0],
                        "radius_m": 0.048,
                    },
                ],
            },
        }
    raise ValueError("part profile must be yam or simulation")


def camera_defaults(profile: str, name: str) -> dict:
    """Return a camera row for explicit mount and connection review."""
    checked_name(name, "camera name")
    row = {
        "mount": {"kind": "scene"},
        "connection": {},
        "stream": {"width": 640, "height": 480, "fps": 30.0},
        "frame_id": checked_name(name + "_optical", "camera frame"),
    }
    if profile == "realsense":
        row.update(driver="waddle_sdk.cameras.realsense", connection={"serial": ""})
    elif profile == "usb":
        row.update(driver="waddle_sdk.cameras.usb", connection={"device": ""})
    elif profile == "orbbec":
        row.update(driver="waddle_sdk.cameras.orbbec")
    elif profile == "simulation":
        row.update(
            driver="waddle_sdk.cameras.mock:MockDriver",
            stream={"width": 64, "height": 48, "fps": 20.0},
            intrinsics={
                "fx": 60.0,
                "fy": 60.0,
                "cx": 31.5,
                "cy": 23.5,
                "depth_scale_mm": 1.0,
            },
            options={"has_depth": True, "object_radius_px": 8},
        )
    else:
        raise ValueError("camera profile must be realsense, usb, orbbec or simulation")
    return row


def review_profile_connections(document: dict):
    for part, row in document["parts"].items():
        if not row.get("base_frame") and not row.get("options", {}).get("base_frame"):
            raise InitializationError(f"Part {part} needs its explicit SDK base_frame")
        if row.get("driver") == YAM_DRIVER:
            options = row.get("options", {})
            connection = row["connection"]
            if not options.get("sim", False):
                channel = connection.get("channel")
                if not isinstance(channel, str) or not channel.strip():
                    raise InitializationError(
                        f"Select the actual YAM CAN channel for {part}"
                    )
            envelope = document.get("envelope", {})
            if envelope.get("static_keepouts") or envelope.get("self_collision"):
                raise InitializationError(
                    "The selected SDK YAM arm factory cannot accept static keepouts/self-collision rules"
                )
    for camera, row in document["cameras"].items():
        if not row.get("mount") or not row.get("frame_id"):
            raise InitializationError(
                f"Camera {camera} needs its explicit mount and optical frame"
            )
        for driver, field in (
            ("waddle_sdk.cameras.realsense", "serial"),
            ("waddle_sdk.cameras.usb", "device"),
        ):
            if row.get("driver") == driver and not row.get("connection", {}).get(field):
                raise InitializationError(f"Camera {camera} needs its selected {field}")


def prepare_profile_assets(stage: Path, part: str, row: dict) -> dict | None:
    """Stage the bundled synthetic profile only when its declaration matches."""
    directory = simulation_directory(part)
    if (
        row.get("driver") != SIMULATION_DRIVER
        or row.get("connection", {}).get("model") != directory + "/model.xml"
        or tuple(row.get("joint_limits", {})) != SIMULATION_JOINTS
    ):
        return None
    package = files("waddle_sdk.workspace") / "assets"
    hashes = {}
    for filename in ("model.xml", "actuators.xml"):
        content = (package / filename).read_bytes()
        target = stage / directory / filename
        if target.exists():
            if target.read_bytes() != content:
                return None
        else:
            _write(target, content)
        hashes[filename] = hashlib.sha256(content).hexdigest()
    return hashes
