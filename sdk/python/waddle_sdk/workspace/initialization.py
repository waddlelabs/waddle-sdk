"""Publish reviewed SDK sites without opening hardware or selecting planners."""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path

import yaml

from waddle_sdk import load_site

from .files import _rename_absent
from .profiles import checked_name, prepare_profile_assets, review_profile_connections
from .sources import InitializationError, _relative, _SourceFiles, _write


def _plain(value):
    try:
        return json.loads(json.dumps(value, allow_nan=False))
    except (TypeError, ValueError) as error:
        raise InitializationError(
            "Configuration must contain finite JSON values"
        ) from error


def make_site(
    *,
    site_id: str,
    parts: dict,
    cameras: dict,
    workspace_bounds: dict,
    static_keepouts=(),
    self_collision=None,
) -> dict:
    """Compose explicit editable SDK rows and reviewed owner-envelope values."""
    checked_name(site_id, "site ID")
    if not isinstance(parts, dict) or not parts:
        raise InitializationError(
            "A hardware site needs at least one explicit robot part"
        )
    if not isinstance(cameras, dict):
        raise InitializationError("Cameras must map names to SDK camera rows")
    for key in (*parts, *cameras):
        checked_name(key)
    return _plain(
        {
            "api_version": "waddle.site/v1",
            "kind": "Site",
            "metadata": {"id": site_id},
            "parts": parts,
            "cameras": cameras,
            "frames": {},
            "calibration": {"artifacts": "calibration/"},
            "workspace_bounds": workspace_bounds,
            "envelope": {
                "static_keepouts": list(static_keepouts),
                "self_collision": self_collision or {},
            },
            "recording": {"root": "recordings/", "format": "mcap"},
        }
    )


def _yaml(value) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode()


def build_site(
    destination,
    site=None,
    *,
    source_site=None,
    site_files=None,
    asset_paths=(),
    configure=None,
):
    """Atomically create a reviewed site and its selected source assets.

    An optional trusted application callback receives ``(stage, document)`` and
    may write its configuration before publication. It must not open hardware.
    Failure or a changed source leaves no published destination. Existing sites
    and their calibration remain unchanged.
    """
    destination = Path(destination).expanduser().absolute()
    if (
        destination.exists()
        or destination.is_symlink()
        or destination.name in {"", ".", ".."}
    ):
        raise InitializationError("Destination must name a new workspace directory")
    parent = destination.parent
    if parent.is_symlink():
        raise InitializationError("Destination parent must be a real directory")
    parent.mkdir(parents=True, exist_ok=True)
    parent = parent.resolve(strict=True)
    destination = parent / destination.name
    stage = Path(tempfile.mkdtemp(prefix=f".{destination.name}.init-", dir=parent))
    source = None
    try:
        document = None if site is None else _plain(site)
        if source_site is not None:
            selected = Path(source_site).expanduser().absolute()
            if not selected.is_symlink() and selected.is_dir():
                selected /= "site.yaml"
            if selected.is_symlink() or not selected.is_file():
                raise InitializationError(
                    "Source must be a real SDK site file or directory"
                )
            source = _SourceFiles(selected.parent.resolve(strict=True), stage)
            _write(stage / "site.yaml", source.read(selected.name))
            inherited = _plain(dict(load_site(stage / "site.yaml").manifest))
            (stage / "site.yaml").unlink()
            if document is None:
                document = inherited
            elif document.get("metadata", {}).get("id") != inherited["metadata"]["id"]:
                raise InitializationError(
                    "Imported site ID must preserve physical-owner identity"
                )
        if document is None:
            raise InitializationError(
                "Select a reviewed site or an explicit source_site"
            )
        sidecars = site_files or {}
        if (
            not isinstance(sidecars, dict)
            or len(sidecars) > 16
            or any(
                not isinstance(content, bytes) or len(content) > 1024 * 1024
                for content in sidecars.values()
            )
            or sum(map(len, sidecars.values())) > 4 * 1024 * 1024
        ):
            raise InitializationError(
                "Site sidecars allow 16 files, 1 MiB each and 4 MiB total"
            )
        for name, content in sidecars.items():
            relative = _relative(name)
            if relative in {"site.yaml", "workspace.yaml"}:
                raise InitializationError(
                    "A site sidecar cannot replace a generated manifest"
                )
            _write(stage / relative, content)
        _write(stage / "site.yaml", _yaml(document))
        load_site(stage / "site.yaml")
        review_profile_connections(document)
        for directory in ("calibration", "operator/poses", "recordings"):
            (stage / directory).mkdir(parents=True, exist_ok=True)
        if source is not None:
            for section in ("parts", "worlds"):
                for row in document.get(section, {}).values():
                    connection = row.get("connection", {})
                    if row.get("driver") in {
                        "waddle_sdk.robots.mujoco:arm",
                        "waddle_sdk.robots.mujoco:backend",
                    }:
                        source.model(connection.get("model", ""))
                    elif row.get("driver") == "waddle_sdk.simulators.adapters:backend":
                        source.copy(connection["simulation"])
            source.artifacts(document.get("calibration", {}).get("artifacts", "calib/"))
            for relative in asset_paths:
                source.copy(relative)
        elif asset_paths:
            raise InitializationError("asset_paths require an explicit source_site")
        for part, row in document["parts"].items():
            prepare_profile_assets(stage, part, row)
        if configure is not None:
            configure(stage, document)
        inventory = {}
        for current, _directories, filenames in os.walk(stage, topdown=False):
            for filename in filenames:
                path = Path(current) / filename
                inventory[path.relative_to(stage).as_posix()] = hashlib.sha256(
                    path.read_bytes()
                ).hexdigest()
            fd = os.open(current, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        parent_fd = os.open(parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            if source is not None:
                source.recheck()
            _rename_absent(parent_fd, stage.name, parent_fd, destination.name)
            os.fsync(parent_fd)
        finally:
            os.close(parent_fd)
        return {
            "site": str(destination / "site.yaml"),
            "site_id": document["metadata"]["id"],
            "files": inventory,
        }
    except Exception:
        shutil.rmtree(stage, ignore_errors=True)
        raise
