"""Optional robot feedback acquired with an exact camera sample."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .. import SessionStamp
    from ..runtime import PartObservation


@dataclass(frozen=True)
class CameraCaptureState:
    """Frozen measured feedback for image geometry, never current control state.

    The driver calls the supplied native ``Session.stamp`` at acquisition,
    before deferred processing/rendering. ``revision`` identifies the source
    generation; a driver changing ``capture_revision`` withdraws old samples
    from the latest-camera slot. It does not rewrite retained historical data.
    """

    stamp: SessionStamp
    parts: Mapping[str, PartObservation]
    revision: str

    def __post_init__(self):
        from ..runtime import PartObservation

        if (
            type(getattr(self.stamp, "session_ns", None)) is not int
            or self.stamp.session_ns < 0
            or type(getattr(self.stamp, "unix_ns", None)) is not int
            or self.stamp.unix_ns <= 0
        ):
            raise TypeError("camera capture state needs a paired SessionStamp")
        if not isinstance(self.revision, str) or not 1 <= len(self.revision) <= 128:
            raise ValueError(
                "camera capture revision must be a bounded nonempty string"
            )

        def vector(value, *, size=None):
            array = np.asarray(value, dtype=np.float64)
            if (
                array.ndim != 1
                or not np.isfinite(array).all()
                or (size is not None and len(array) != size)
            ):
                raise ValueError(
                    "camera capture feedback must contain finite matching vectors"
                )
            return np.frombuffer(array.tobytes(), dtype=np.float64)

        parts = {}
        for name, part in self.parts.items():
            if (
                not isinstance(name, str)
                or not name
                or not isinstance(part, PartObservation)
            ):
                raise TypeError("camera capture parts must be named PartObservations")
            q = vector(part.joint_position)
            parts[name] = PartObservation(
                q,
                vector(part.joint_velocity, size=len(q)),
                None
                if part.ee_pose_wxyz is None
                else vector(part.ee_pose_wxyz, size=7),
                part.frame_id,
                part.gripper_force,
            )
        object.__setattr__(self, "parts", MappingProxyType(parts))
