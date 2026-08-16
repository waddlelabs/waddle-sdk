"""Manifest-facing camera driver extension contract."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class CameraConfig:
    name: str
    connection: Mapping[str, Any]
    stream: Mapping[str, Any]
    frame_id: str | None
    intrinsics: Mapping[str, Any] | None
    options: Mapping[str, Any] = field(default_factory=dict)
    site_root: Path = Path(".")


__all__ = ["CameraConfig"]
