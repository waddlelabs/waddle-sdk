"""Manifest-facing camera driver extension contract."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class CameraMount:
    """Physical camera mobility declared by the site operator."""

    kind: str
    part: str | None = None

    def __post_init__(self) -> None:
        if self.kind not in {"scene", "wrist"}:
            raise ValueError("camera mount kind must be 'scene' or 'wrist'")
        if self.kind == "scene" and self.part is not None:
            raise ValueError("a scene camera mount must not name a robot part")
        if self.kind == "wrist" and not self.part:
            raise ValueError("a wrist camera mount must name its owning robot part")


@dataclass(frozen=True)
class CameraConfig:
    name: str
    connection: Mapping[str, Any]
    stream: Mapping[str, Any]
    frame_id: str | None
    intrinsics: Mapping[str, Any] | None
    mount: CameraMount | None = None
    options: Mapping[str, Any] = field(default_factory=dict)
    site_root: Path = Path(".")


__all__ = ["CameraConfig", "CameraMount"]
