"""Bounded source snapshots and dependency copying for the workspace builder."""

from __future__ import annotations

import os
import stat
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath

MAX_FILES = 1024
MAX_FILE_BYTES = 32 * 1024 * 1024
MAX_TOTAL_BYTES = 256 * 1024 * 1024


class InitializationError(ValueError):
    """A reviewed configuration cannot produce a complete new workspace."""


def _relative(value: str) -> str:
    if not isinstance(value, str) or not value or "\\" in value or "\x00" in value:
        raise InitializationError(
            "Asset paths must be explicit workspace-relative paths"
        )
    path = PurePosixPath(value)
    if (
        path.is_absolute()
        or any(part in {"", ".", ".."} for part in path.parts)
        or not path.parts
    ):
        raise InitializationError("Asset paths must stay inside the source workspace")
    return str(path)


def _read(path: Path, *, root: Path) -> bytes:
    relative = path.relative_to(root)
    parent_fd = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        for component in relative.parts[:-1]:
            child_fd = os.open(
                component,
                os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
                dir_fd=parent_fd,
            )
            os.close(parent_fd)
            parent_fd = child_fd
        fd = os.open(
            relative.name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=parent_fd
        )
        with os.fdopen(fd, "rb") as stream:
            before = os.fstat(stream.fileno())
            if not stat.S_ISREG(before.st_mode) or before.st_size > MAX_FILE_BYTES:
                raise InitializationError(
                    "Source files must be regular files at most 32 MiB"
                )
            content = stream.read(MAX_FILE_BYTES + 1)
            after = os.fstat(stream.fileno())
    except OSError as error:
        raise InitializationError(
            f"Required source asset is unavailable: {relative}"
        ) from error
    finally:
        os.close(parent_fd)
    if len(content) > MAX_FILE_BYTES or (before.st_size, before.st_mtime_ns) != (
        after.st_size,
        after.st_mtime_ns,
    ):
        raise InitializationError("Source asset changed while it was being read")
    return content


def _write(path: Path, content: bytes):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    path.chmod(0o600)


class _SourceFiles:
    def __init__(self, root: Path, stage: Path):
        self.root, self.stage = root, stage
        self.snapshots: dict[str, bytes] = {}
        self.total = 0
        self.visited: set[str] = set()

    def read(self, relative: str) -> bytes:
        relative = _relative(relative)
        if relative not in self.snapshots:
            content = _read(self.root / relative, root=self.root)
            self.total += len(content)
            if len(self.snapshots) >= MAX_FILES or self.total > MAX_TOTAL_BYTES:
                raise InitializationError(
                    "Imported files exceed the 1024-file/256-MiB workspace bound"
                )
            self.snapshots[relative] = content
        return self.snapshots[relative]

    def copy(self, relative: str):
        relative = _relative(relative)
        if relative in {"workspace.yaml", "site.yaml"}:
            raise InitializationError("An asset cannot replace a generated manifest")
        content = self.read(relative)
        target = self.stage / relative
        if target.exists():
            if target.read_bytes() != content:
                raise InitializationError(
                    "Two selected sources disagree on one asset path"
                )
        else:
            _write(target, content)

    def model(self, relative: str, depth=0):
        relative = _relative(relative)
        if relative in self.visited:
            return
        if depth > 16:
            raise InitializationError("Model includes exceed 16 levels")
        self.visited.add(relative)
        self.copy(relative)
        if PurePosixPath(relative).suffix.lower() not in {".xml", ".urdf"}:
            return
        data = self.read(relative)
        if b"<!DOCTYPE" in data or b"<!ENTITY" in data:
            raise InitializationError(
                "Model XML must not contain external entities or DTDs"
            )
        try:
            tree = ET.fromstring(data)
        except ET.ParseError as error:
            raise InitializationError("Referenced model XML is invalid") from error
        base = PurePosixPath(relative).parent
        compiler = tree.find("compiler")
        for element in tree.iter():
            resource = element.get("file") or element.get("filename")
            if not resource:
                continue
            folder = ""
            if compiler is not None and element.tag in {"mesh", "texture"}:
                folder = compiler.get(element.tag + "dir", compiler.get("assetdir", ""))
            selected = _relative(str(base / folder / _relative(resource)))
            if element.tag == "include":
                self.model(selected, depth + 1)
            else:
                self.copy(selected)

    def artifacts(self, relative: str):
        relative = _relative(relative)
        path = self.root / relative
        if not path.exists():
            return
        if path.is_symlink() or not path.is_dir():
            raise InitializationError(
                "Calibration artifacts must be a regular directory"
            )
        for current, directories, filenames in os.walk(path, followlinks=False):
            if any((Path(current) / child).is_symlink() for child in directories):
                raise InitializationError(
                    "Calibration artifacts must not contain symlinks"
                )
            for filename in sorted(filenames):
                item = Path(current) / filename
                if item.suffix.lower() not in {
                    ".json",
                    ".npz",
                    ".npy",
                    ".png",
                    ".jpg",
                    ".jpeg",
                }:
                    raise InitializationError(
                        "Calibration directory contains an unsupported artifact file"
                    )
                self.copy(item.relative_to(self.root).as_posix())

    def recheck(self):
        for relative, content in self.snapshots.items():
            if _read(self.root / relative, root=self.root) != content:
                raise InitializationError(
                    "Source configuration or assets changed before publication"
                )
