"""Shared, metre/radian reference scenes and hardware-matched declarations.

Joint origins/limits and jaw travel come from the same sources as the live
adapters. Engines load the same pinned manufacturer URDF and mesh assemblies.
"""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from functools import lru_cache
from importlib.resources import files
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from ..robots import xarm, yam
from ..robots.site import PartConfig

BACKENDS = ("mujoco", "isaac", "sapien")
ROBOTS = ("yam", "xarm7")
ENVIRONMENTS = ("two_cubes", "bottle_cap", "drawer")


def rotation(rpy: Any) -> np.ndarray:
    r, p, y = map(float, rpy)
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def transform(xyz: Any = (0, 0, 0), rpy: Any = (0, 0, 0)) -> np.ndarray:
    value = np.eye(4)
    value[:3, :3] = rotation(rpy)
    value[:3, 3] = xyz
    return value


def quaternion(matrix: Any) -> tuple[float, ...]:
    """Rotation matrix to unit wxyz, including rotations close to pi."""
    m = np.asarray(matrix)
    k = (
        np.array(
            [
                [
                    m[0, 0] - m[1, 1] - m[2, 2],
                    m[1, 0] + m[0, 1],
                    m[2, 0] + m[0, 2],
                    m[2, 1] - m[1, 2],
                ],
                [
                    m[1, 0] + m[0, 1],
                    m[1, 1] - m[0, 0] - m[2, 2],
                    m[2, 1] + m[1, 2],
                    m[0, 2] - m[2, 0],
                ],
                [
                    m[2, 0] + m[0, 2],
                    m[2, 1] + m[1, 2],
                    m[2, 2] - m[0, 0] - m[1, 1],
                    m[1, 0] - m[0, 1],
                ],
                [m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1], np.trace(m)],
            ]
        )
        / 3
    )
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1][[3, 0, 1, 2]]
    return tuple(q if q[0] >= 0 else -q)


@dataclass(frozen=True)
class Profile:
    name: str
    names: tuple[str, ...]
    limits: tuple[tuple[float, float], ...]
    origins: tuple[tuple[float, ...], ...]
    rpys: tuple[tuple[float, ...], ...]
    tool_xyz: tuple[float, ...]
    tool_rpy: tuple[float, ...]
    home: tuple[float, ...]
    opening: float
    frame: str
    closing_axis: tuple[float, ...]
    pinch_offset: tuple[float, ...]
    pointing_down: tuple[float, ...]

    @property
    def dof(self) -> int:
        return len(self.names) - 1

    def poses(self, q: Any) -> list[np.ndarray]:
        values = np.asarray(q, dtype=float)
        if values.shape != (len(self.names),) or not np.isfinite(values).all():
            raise ValueError(
                "joint vector must have the declared width and finite values"
            )
        pose = np.eye(4)
        result = [pose.copy()]
        for i in range(self.dof):
            pose = (
                pose
                @ transform(self.origins[i], self.rpys[i])
                @ transform(rpy=(0, 0, values[i]))
            )
            result.append(pose.copy())
        result.append(pose @ transform(self.tool_xyz, self.tool_rpy))
        return result


@lru_cache(maxsize=2)
def profile(name: str) -> Profile:
    if name == "yam":
        hand = ET.fromstring(
            files(__package__).joinpath("data/yam/linear_4310.xml").read_text()
        )
        pinch = np.fromstring(
            hand.find(".//site[@name='grasp_site']").get("pos"), sep=" "
        )
        # Native hand -> SDK TCP: Rx(pi), then subtract the declared tool offset.
        pinch = pinch * [1, -1, -1] - yam.TOOL_ORIGIN_XYZ_M
        return Profile(
            name,
            yam.JOINT_NAMES,
            yam.JOINT_LIMITS,
            yam.CHAIN_ORIGIN_XYZ_M,
            yam.CHAIN_ORIGIN_RPY_RAD,
            yam.TOOL_ORIGIN_XYZ_M,
            yam.TOOL_ORIGIN_RPY_RAD,
            tuple(yam.DEFAULT_SIM_HOME[0][:-1]) + (1.0,),
            yam.GRIPPER_MAX_OPENING_M,
            yam.BASE_FRAME,
            (0.0, 1.0, 0.0),
            tuple(map(float, pinch)),
            (0.0, 0.0, 1.0, 0.0),
        )
    if name != "xarm7":
        raise ValueError(f"robot must be one of {ROBOTS}")
    # Reuse the live factory's public action descriptor without opening hardware.
    declaration = xarm.arm(
        config=PartConfig(
            name=name,
            posture="supervised",
            connection={},
            joint_limits={},
            workspace_bounds={},
            envelope={},
            options={"model": name},
        )
    ).robot()
    joints = declaration.action_space.joints
    data = yaml.safe_load(
        files(__package__).joinpath("data/xarm7-kinematics.yaml").read_text()
    )["kinematics"]
    rows = [data[f"joint{i + 1}"] for i in range(7)]
    return Profile(
        name,
        tuple(j.name for j in joints),
        tuple((j.min_position, j.max_position) for j in joints),
        tuple(tuple(row[k] for k in ("x", "y", "z")) for row in rows),
        tuple(tuple(row[k] for k in ("roll", "pitch", "yaw")) for row in rows),
        (0.0, 0.0, 0.172),
        (0.0, 0.0, 0.0),
        (0.0, -0.65, 0.0, 1.0, 0.0, 1.2, 0.0, 1.0),
        0.084,
        "xarm_base",
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
    )


def look_at(position: Any, target: Any) -> np.ndarray:
    """World-from-camera transform for optical +X right, +Y down, +Z forward."""
    z = np.asarray(target, dtype=float) - position
    z /= np.linalg.norm(z)
    x = np.cross(z, (0, 0, 1))
    x /= np.linalg.norm(x)
    value = np.eye(4)
    value[:3, :3] = np.column_stack((x, np.cross(z, x), z))
    value[:3, 3] = position
    return value


def make_site(
    site_id: str,
    *,
    backend: str,
    robot: str,
    environment: str,
    width: int = 640,
    height: int = 480,
    worker_python: str | None = None,
) -> tuple[dict, dict]:
    """Return a strict site declaration and its separate simulator configuration.

    This is non-opening and performs no downloads. Names and stream calibration
    can be edited together in the returned documents before validation/publication.
    """
    if backend not in BACKENDS or environment not in ENVIRONMENTS:
        raise ValueError(f"choose backend {BACKENDS} and environment {ENVIRONMENTS}")
    if (
        type(width) is not int
        or type(height) is not int
        or not (16 <= width <= 4096 and 16 <= height <= 4096)
    ):
        raise ValueError("camera width and height must be integers in [16, 4096]")
    if worker_python is not None and (
        not isinstance(worker_python, str) or not Path(worker_python).is_absolute()
    ):
        raise ValueError("worker_python must be an absolute interpreter path")
    p = profile(robot)
    intrinsic = dict(
        fx=width * 0.9,
        fy=width * 0.9,
        cx=(width - 1) / 2,
        cy=(height - 1) / 2,
        depth_scale_mm=1.0,
    )
    connection = {"simulation": "simulation.json"}
    cameras = {}
    mounts = {
        "scene": look_at((0.75, -0.9, 0.95), (0.2, 0, 0.25)).tolist(),
        # View through the jaw gap, perpendicular to the closing axis.
        "wrist": look_at((-0.10, 0, -0.08), (0, 0, 0.07)).tolist(),
    }
    if robot == "yam":
        # I2RT's optical frame for the LINEAR_4310 D405 bracket, expressed
        # relative to the unchanged SDK TCP rather than the native hand frame.
        mount = json.loads(
            files(__package__).joinpath("data/yam/wrist_camera.json").read_text()
        )
        mounts["wrist"] = (
            transform(-np.asarray(p.tool_xyz), (math.pi, 0, 0))
            @ transform(mount["xyz"], mount["rpy"])
        ).tolist()
    for name in mounts:
        cameras[name] = dict(
            world="cell",
            connection={},
            stream=dict(width=width, height=height, fps=15),
            frame_id=f"cam_{name}",
            intrinsics=intrinsic.copy(),
            mount={"kind": name, **({"part": "arm"} if name == "wrist" else {})},
        )
    simulation = dict(
        api_version="waddle.simulation/v1",
        backend=backend,
        robot=robot,
        environment=environment,
        timestep=0.002,
        cameras={
            name: dict(**row, transform=mounts[name]) for name, row in cameras.items()
        },
    )
    if worker_python is not None:
        simulation["worker_python"] = worker_python
    site = dict(
        api_version="waddle.site/v1",
        kind="Site",
        metadata={"id": site_id},
        worlds={
            "cell": {
                "driver": "waddle_sdk.simulators.adapters:backend",
                "connection": connection,
            }
        },
        parts={
            "arm": dict(
                world="cell",
                posture="supervised",
                connection={},
                base_frame=p.frame,
                joint_limits=dict(zip(p.names, p.limits)),
                gripper=dict(
                    joint="gripper",
                    closed_m=0.0,
                    open_m=p.opening,
                    closed_action=0.0,
                    open_action=1.0,
                    closing_axis_tcp=list(p.closing_axis),
                    pinch_offset_tcp_m=list(p.pinch_offset),
                    pointing_down_wxyz=list(p.pointing_down),
                ),
                options=dict(
                    rate_hz=50.0,
                    max_joint_speed_rad_s=0.5,
                    max_gripper_speed_per_s=(
                        yam.DEFAULT_MAX_GRIPPER_SPEED_PER_S if robot == "yam" else 1.0
                    ),
                ),
            )
        },
        cameras=cameras,
        frames={},
        calibration={"artifacts": "calib/"},
        workspace_bounds={"min": [-1.0, -1.0, -0.12], "max": [1.0, 1.0, 1.5]},
        envelope={"static_keepouts": [], "self_collision": {}},
        recording={"root": "recordings/", "format": "mcap"},
    )
    return site, simulation


def load_scene(root: Path, relative: Any) -> tuple[Path, dict]:
    if (
        not isinstance(relative, str)
        or not relative
        or "\\" in relative
        or "\x00" in relative
    ):
        raise ValueError("connection.simulation must be a portable relative path")
    path = Path(relative)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError("connection.simulation must stay beneath the site root")
    resolved = (root / path).resolve(strict=True)
    resolved.relative_to(root.resolve())
    value = json.loads(resolved.read_text())
    if (
        not isinstance(value, dict)
        or value.get("api_version") != "waddle.simulation/v1"
    ):
        raise ValueError("expected waddle.simulation/v1 configuration")
    allowed = {
        "api_version",
        "backend",
        "robot",
        "environment",
        "timestep",
        "cameras",
        "worker_python",
    }
    if value.keys() - allowed:
        raise ValueError(
            f"unknown simulation settings: {sorted(value.keys() - allowed)}"
        )
    if (
        value.get("backend") not in BACKENDS
        or value.get("environment") not in ENVIRONMENTS
    ):
        raise ValueError("unknown simulation backend or environment")
    profile(value.get("robot"))
    dt = value.get("timestep")
    if (
        isinstance(dt, bool)
        or not isinstance(dt, (int, float))
        or not 0.0001 <= dt <= 0.01
    ):
        raise ValueError("simulation timestep must be in [.0001, .01] seconds")
    cameras = value.get("cameras")
    if not isinstance(cameras, dict) or not cameras:
        raise ValueError("simulation requires explicit camera profiles")
    worker = value.get("worker_python")
    if worker is not None and (
        not isinstance(worker, str) or not Path(worker).is_absolute()
    ):
        raise ValueError("worker_python must be an absolute interpreter path")
    for name, row in cameras.items():
        if not isinstance(name, str) or not name or not isinstance(row, dict):
            raise ValueError("camera profiles must be named objects")
        stream, intr, mount = row.get("stream"), row.get("intrinsics"), row.get("mount")
        if not all(isinstance(v, dict) for v in (stream, intr, mount)):
            raise ValueError(
                f"camera {name} needs stream, intrinsics, and mount objects"
            )
        for key in ("width", "height"):
            if type(stream.get(key)) is not int or not 16 <= stream[key] <= 4096:
                raise ValueError(
                    f"camera {name} {key} must be an integer in [16, 4096]"
                )
        fps = stream.get("fps")
        if (
            type(fps) not in (int, float)
            or not math.isfinite(fps)
            or not 0 < fps <= 120
        ):
            raise ValueError(f"camera {name} fps must be in (0, 120]")
        wrist = (
            set(mount) == {"kind", "part"}
            and mount["kind"] == "wrist"
            and isinstance(mount["part"], str)
            and bool(mount["part"].strip())
        )
        if mount != {"kind": "scene"} and not wrist:
            raise ValueError(
                f"camera {name} requires a scene mount or wrist mount on a named part"
            )
        if not isinstance(row.get("frame_id"), str) or not row["frame_id"]:
            raise ValueError(f"camera {name} requires an optical frame id")
        for key in ("fx", "fy", "cx", "cy", "depth_scale_mm"):
            if type(intr.get(key)) not in (int, float) or not math.isfinite(intr[key]):
                raise ValueError(f"camera {name} needs finite numeric {key}")
        matrix = np.asarray(row["transform"], dtype=float)
        if (
            matrix.shape != (4, 4)
            or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1])
        ):
            raise ValueError(f"camera {name} requires a finite rigid transform")
        r = matrix[:3, :3]
        if not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or not np.isclose(
            np.linalg.det(r), 1
        ):
            raise ValueError(f"camera {name} rotation must be orthonormal")
        intr = row["intrinsics"]
        if any(intr.get("distortion", ())):
            raise ValueError(
                "reference simulation cameras require rectified intrinsics"
            )
        for key in ("fx", "fy", "depth_scale_mm"):
            if not math.isfinite(intr[key]) or intr[key] <= 0:
                raise ValueError(f"camera {name} needs positive {key}")
    return resolved, value


def depth_z16(depth_m: Any, scale_mm: float) -> np.ndarray:
    depth = np.asarray(depth_m, dtype=float)
    raw = np.rint(depth * (1000.0 / scale_mm))
    valid = np.isfinite(raw) & (raw > 0) & (raw <= 65535)
    return np.where(valid, raw, 0).astype(np.uint16)
