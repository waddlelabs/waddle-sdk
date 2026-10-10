"""Declarative SDK transport selection.

These values contain no connection state. The native core owns dialing,
feature negotiation, reconnect behavior, and connection-scoped safety.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Grpc:
    """Select the waddle.v0 control transport.

    Connector bindings are all-or-none. authorization_only is reserved for
    the Site lifecycle pre-open authorization probe.
    """

    url: str
    token: str | None = field(default=None, repr=False)
    customer_id: str | None = None
    project_id: str | None = None
    workspace_id: str | None = None
    authorization_only: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.url, str) or not self.url:
            raise ValueError("Grpc.url must be a non-empty str")
        if self.token is not None and (
            not isinstance(self.token, str) or not self.token
        ):
            raise ValueError("Grpc.token must be a non-empty str or None")
        binding = (self.customer_id, self.project_id, self.workspace_id)
        if any(value is not None for value in binding):
            if not all(isinstance(value, str) and bool(value) for value in binding):
                raise ValueError(
                    "Grpc customer_id, project_id, and workspace_id must all be "
                    "non-empty strings or all be omitted"
                )
        elif self.authorization_only:
            raise ValueError("Grpc.authorization_only requires a connector binding")
        if not isinstance(self.authorization_only, bool):
            raise TypeError("Grpc.authorization_only must be bool")


@dataclass(frozen=True)
class LiveKit:
    """Select the optional LiveKit media transport.

    The companion waddle-sdk-media wheel supplies the native feature. Optional
    preview_width/fps/max_kbps ceilings affect presentation only; depth_preview
    and demand_driven control colorization and publisher subscriber demand.
    video_only excludes teleoperation/telemetry topics and their intake worker.
    isolated selects a Linux RGB-only child with nonblocking shared-memory
    slots. worker_cpu_ids places the child before native initialization;
    callers reserve disjoint owner CPUs. worker_memory_mb bounds its address
    space, including libraries (not just pixel buffers), to 2048 MiB by default.
    """

    url: str
    token: str = field(repr=False)
    preview_width: int | None = None
    preview_fps: float | None = None
    preview_max_kbps: int | None = None
    depth_preview: bool = True
    demand_driven: bool = False
    video_only: bool = False
    isolated: bool = False
    worker_cpu_ids: tuple[int, ...] | None = None
    worker_memory_mb: int = 2048

    def __post_init__(self) -> None:
        if not isinstance(self.url, str) or not self.url:
            raise ValueError("LiveKit.url must be a non-empty str")
        if not isinstance(self.token, str) or not self.token:
            raise ValueError("LiveKit.token must be a non-empty str")
        for name in ("preview_width", "preview_max_kbps"):
            value = getattr(self, name)
            if value is not None and (type(value) is not int or value <= 0):
                raise ValueError(f"LiveKit.{name} must be a positive integer or None")
        if self.preview_fps is not None and (
            isinstance(self.preview_fps, bool)
            or not isinstance(self.preview_fps, (int, float))
            or not math.isfinite(self.preview_fps)
            or self.preview_fps <= 0
        ):
            raise ValueError("LiveKit.preview_fps must be finite and positive or None")
        for name in ("depth_preview", "demand_driven", "video_only", "isolated"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"LiveKit.{name} must be a bool")
        if self.isolated and (
            not self.video_only or self.depth_preview or self.preview_fps is None
        ):
            raise ValueError(
                "isolated preview requires video_only, no depth preview and an explicit fps ceiling"
            )
        if self.worker_cpu_ids is not None and (
            not self.isolated
            or not isinstance(self.worker_cpu_ids, tuple)
            or not self.worker_cpu_ids
            or any(type(cpu) is not int or cpu < 0 for cpu in self.worker_cpu_ids)
            or len(set(self.worker_cpu_ids)) != len(self.worker_cpu_ids)
        ):
            raise ValueError(
                "LiveKit.worker_cpu_ids requires isolated preview and distinct nonnegative CPU IDs"
            )
        if type(self.worker_memory_mb) is not int or self.worker_memory_mb <= 0:
            raise ValueError("LiveKit.worker_memory_mb must be a positive integer")


__all__ = ["Grpc", "LiveKit"]
