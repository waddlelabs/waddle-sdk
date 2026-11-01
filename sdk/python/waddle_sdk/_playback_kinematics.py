"""Numeric selected-tool FK for the existing native worker's feedback callback."""

import math

import numpy as np

from .robots.base import quaternion_wxyz
from .runtime import PlaybackKinematicJoint, PlaybackKinematics, Pose


def vector(value, width):
    result = np.asarray(value, dtype=float)
    if result.shape != (width,) or not np.isfinite(result).all():
        raise ValueError(f"kinematics requires {width} finite coordinates")
    return result


def rotation(axis, angle):
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


class Kinematics:
    def __init__(self, spec, *, width, frame):
        if not isinstance(spec, PlaybackKinematics) or len(spec.joints) > 128:
            raise ValueError("provide PlaybackKinematics with at most 128 joints")
        pose = spec.reference_pose
        if not isinstance(pose, Pose) or pose.frame_id != frame:
            raise ValueError("kinematic reference pose must use the part's base frame")
        self.position = vector(pose.position_m, 3)
        w, x, y, z = quaternion = vector(pose.quaternion_wxyz, 4)
        if abs(np.linalg.norm(quaternion) - 1) > 1e-6:
            raise ValueError("kinematic reference quaternion must have unit length")
        self.orientation = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
        self.frame = frame
        self.joints = []
        for joint in spec.joints:
            if (
                not isinstance(joint, PlaybackKinematicJoint)
                or type(joint.index) is not int
                or not 0 <= joint.index < width
                or joint.kind not in ("hinge", "slide")
            ):
                raise ValueError(
                    "kinematic joints require a local index and hinge/slide kind"
                )
            axis, origin = vector(joint.axis, 3), vector(joint.origin_m, 3)
            if abs(np.linalg.norm(axis) - 1) > 1e-6:
                raise ValueError("kinematic joint axis must have unit length")
            scale, offset = vector((joint.scale, joint.offset), 2)
            self.joints.append((joint.index, joint.kind, axis, origin, scale, offset))

    def __call__(self, positions):
        transform, translation = np.eye(3), np.zeros(3)
        for index, kind, axis, origin, scale, offset in self.joints:
            value = scale * positions[index] + offset
            if kind == "slide":
                translation += transform @ (axis * value)
            else:
                turn = rotation(axis, value)
                translation += transform @ (origin - turn @ origin)
                transform = transform @ turn
        return Pose(
            tuple(translation + transform @ self.position),
            quaternion_wxyz(transform @ self.orientation),
            self.frame,
        )
