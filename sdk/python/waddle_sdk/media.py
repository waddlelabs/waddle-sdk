"""Camera-only publishing for separately owned presentation processes.

This publisher consumes copied RGB samples. It opens no Site, camera, robot,
run, control transport or actuation callback. Its caller owns sampling and
process/resource isolation. Source timestamps remain caller-supplied.
"""

from __future__ import annotations

from collections.abc import Mapping

from ._native import FEATURES, core
from .transport import LiveKit


class PreviewPublisher:
    """Demand-driven RGB publisher with explicit presentation bounds.

    Use in a presentation process, outside a hardware owner. ``cameras`` maps
    source camera IDs to (width, height). Construction/publication can wait for
    signaling; neither belongs on a control or capture thread. Missing media
    support raises before dialing. Call :meth:`close` during process drain.
    """

    def __init__(self, cameras: Mapping[str, tuple[int, int]], media: LiveKit):
        if "livekit" not in FEATURES:
            raise RuntimeError("Camera preview requires waddle-sdk[media]")
        if (
            not isinstance(media, LiveKit)
            or not media.video_only
            or media.depth_preview
            or not media.demand_driven
            or any(
                value is None
                for value in (
                    media.preview_width,
                    media.preview_fps,
                    media.preview_max_kbps,
                )
            )
        ):
            raise ValueError(
                "Camera preview requires bounded, demand-driven RGB-only video"
            )
        self._native = core.CameraPublisher(
            media.url,
            media.token,
            dict(cameras),
            media.preview_width,
            media.preview_fps,
            media.preview_max_kbps,
        )

    def publish_track(self, camera: str) -> None:
        """Publish a declared RGB track; signaling failures retain native scope."""
        self._native.publish_track(camera)

    def wants_frame(self, camera: str) -> bool:
        """Whether a connected viewer currently requests this track."""
        return self._native.wants_frame(camera)

    def push_rgb(self, camera: str, data: bytes, *, session_ns: int) -> None:
        """Publish original-grid RGB8 bytes with the acquired sample timestamp."""
        self._native.push_rgb(camera, data, session_ns)

    def close(self) -> None:
        """Close this media transport only; owns no hardware shutdown."""
        self._native.close()
