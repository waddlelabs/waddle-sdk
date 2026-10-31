"""Validate saved connection settings and explicitly selected credentials."""

import ipaddress
import json
import os
import re
import stat
from dataclasses import dataclass, field
from pathlib import Path
from urllib.parse import urlsplit

from .errors import RelayError

RIG_PATTERN = "^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$"

_RIG = re.compile(RIG_PATTERN)


def valid_rig(name):
    """A rig name registered with an account key; the gateway assigns its workspace."""
    if name is not None and (not isinstance(name, str) or _RIG.fullmatch(name) is None):
        raise RelayError(
            "connector_rig",
            "Rig names use 1 to 64 letters, digits, '.', '_' or '-' and start "
            "with a letter or digit",
        )
    return name


@dataclass(frozen=True)
class SavedConnection:
    endpoint: str
    key_file: Path
    allow_loopback_ws: bool
    launcher: tuple[str, ...] = ()
    rig: str | None = None


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate connection field")
        result[key] = value
    return result


def load_connection(path: Path, *, root: Path | None = None) -> SavedConnection | None:
    """Read bounded private operator config; never execute it while inspecting."""
    path = Path(path)
    directory = path.parent
    root = directory if root is None else Path(root)
    if not directory.exists() and not directory.is_symlink():
        return None
    try:
        info = directory.lstat()
        if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid():
            raise ValueError("connection directory must belong to the current user")
        try:
            descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        except FileNotFoundError:
            return None
        with os.fdopen(descriptor, "rb") as source:
            info = os.fstat(source.fileno())
            if (
                not stat.S_ISREG(info.st_mode)
                or info.st_uid != os.getuid()
                or info.st_mode & 0o077
                or info.st_size > 16384
            ):
                raise ValueError("connection config must be a private regular file")
            data = source.read(16385)
        row = json.loads(data, object_pairs_hook=_unique_pairs)
        required = {"version", "endpoint", "key_file", "allow_loopback_ws"}
        if (
            not isinstance(row, dict)
            or not required <= row.keys()
            or row.keys() - required - {"launcher", "rig"}
            or type(row["version"]) is not int
            or row["version"] != 1
            or type(row["allow_loopback_ws"]) is not bool
        ):
            raise ValueError("connection config must match version 1")
        for key in ("endpoint", "key_file"):
            if not isinstance(row[key], str) or not row[key] or "\0" in row[key]:
                raise ValueError("invalid connection endpoint or key path")
        key_file = Path(row["key_file"]).expanduser()
        if not key_file.is_absolute():
            key_file = root / key_file
        launcher = row.get("launcher", [])
        if not isinstance(launcher, list) or len(launcher) > 32:
            raise ValueError("invalid connection launcher")
        if any(
            not isinstance(arg, str) or not arg or "\0" in arg or len(arg) > 4096
            for arg in launcher
        ) or (launcher and not Path(launcher[0]).is_absolute()):
            raise ValueError(
                "launcher requires an absolute executable and bounded arguments"
            )
        rig = row.get("rig")
        if rig is not None and (
            not isinstance(rig, str) or _RIG.fullmatch(rig) is None
        ):
            raise ValueError("invalid connection rig name")
        return SavedConnection(
            row["endpoint"], key_file, row["allow_loopback_ws"], tuple(launcher), rig
        )
    except (OSError, ValueError, TypeError) as error:
        raise RelayError(
            "connection_configuration",
            "Invalid saved connection configuration; review its fields and file permissions",
        ) from error


@dataclass(frozen=True)
class ConnectionConfig:
    endpoint: str
    key: str = field(repr=False)
    allow_loopback_ws: bool = False
    # Set with an account key: hello names this rig and no workspace (rig mode).
    rig: str | None = None

    def __post_init__(self):
        valid_rig(self.rig)
        try:
            parsed = urlsplit(self.endpoint)
            port = parsed.port
            loopback = ipaddress.ip_address(parsed.hostname or "").is_loopback
        except ValueError:
            loopback = False
            try:
                parsed, port = urlsplit(self.endpoint), urlsplit(self.endpoint).port
            except ValueError:
                raise RelayError(
                    "connector_configuration", "Invalid connector endpoint"
                ) from None
        if (
            not parsed.hostname
            or parsed.username is not None
            or parsed.password is not None
            or parsed.fragment
            or parsed.query
            or port == 0
            or not (
                parsed.scheme == "wss"
                or (parsed.scheme == "ws" and self.allow_loopback_ws and loopback)
            )
        ):
            raise RelayError(
                "connector_configuration",
                "Use a credential-free wss endpoint, or explicit development ws on a loopback IP",
            )
        if (
            not isinstance(self.key, str)
            or not 16 <= len(self.key) <= 4096
            or any(not 0x21 <= ord(c) <= 0x7E for c in self.key)
        ):
            raise RelayError(
                "connector_key", "Connector key must be a bounded visible ASCII token"
            )


def read_key(key_file):
    """Read a bounded credential from one explicitly selected private file."""
    try:
        fd = os.open(Path(key_file).expanduser(), os.O_RDONLY | os.O_NOFOLLOW)
        with os.fdopen(fd, "rb") as stream:
            info = os.fstat(stream.fileno())
            if (
                not stat.S_ISREG(info.st_mode)
                or info.st_uid != os.getuid()
                or info.st_mode & 0o077
            ):
                raise RelayError(
                    "connector_key",
                    "Connector key file must be private to this OS user",
                )
            value = stream.read(4098)
        if len(value) > 4097:
            raise RelayError(
                "connector_key", "Connector key file exceeds its size bound"
            )
        return value.decode("ascii").strip()
    except (OSError, UnicodeError):
        raise RelayError(
            "connector_key", "Connector key file could not be read"
        ) from None
