"""Small, explicit SocketCAN link lifecycle helpers for robot adapters.

The caller supplies one exact interface and bitrate.  This module never scans,
guesses which robot is attached, or changes an interface that is already up.
It exists so vendor adapters can share one fail-closed implementation instead
of each shelling out differently.
"""

from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import sys
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class SocketCanState:
    """The bounded link facts needed before a robot opens its bus."""

    interface: str
    up: bool
    bitrate: int | None


def _interface(value: str) -> str:
    if not isinstance(value, str) or not value or value.strip() != value:
        raise ValueError("SocketCAN interface must be a non-empty exact name")
    if "\0" in value or len(value.encode("utf-8")) > 15:
        raise ValueError(
            f"SocketCAN interface {value!r} exceeds Linux's 15-byte name limit"
        )
    return value


def _bitrate(value: int) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 1 <= value <= 10_000_000
    ):
        raise ValueError(
            "SocketCAN bitrate must be an integer from 1 to 10000000 bit/s"
        )
    return value


def _restart_ms(value: int) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= 60_000
    ):
        raise ValueError("SocketCAN restart_ms must be an integer from 0 to 60000 ms")
    return value


def _detail(completed: subprocess.CompletedProcess[str]) -> str:
    raw = (
        completed.stderr.strip()
        or completed.stdout.strip()
        or f"exit {completed.returncode}"
    )
    return raw.splitlines()[0][:240]


def _run(
    command: Sequence[str], *, timeout: float = 5.0
) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            list(command),
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise RuntimeError(
            f"could not run {shlex.join(command)}: {type(error).__name__}: {error}"
        ) from error


def _ip_command() -> str:
    command = shutil.which("ip")
    if command is None:
        raise RuntimeError("SocketCAN setup needs the ip command from iproute2")
    return command


def read_socketcan_state(interface: str) -> SocketCanState:
    """Read one declared interface without changing it."""

    interface = _interface(interface)
    completed = _run(
        [_ip_command(), "-details", "-json", "link", "show", "dev", interface]
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"SocketCAN {interface}: interface not found ({_detail(completed)})"
        )
    try:
        payload: Any = json.loads(completed.stdout)
        row = payload[0] if isinstance(payload, list) and len(payload) == 1 else None
    except (IndexError, json.JSONDecodeError) as error:
        raise RuntimeError(
            f"SocketCAN {interface}: ip returned invalid link JSON"
        ) from error
    if not isinstance(row, dict):
        raise RuntimeError(  # noqa: TRY004 - malformed external process output
            f"SocketCAN {interface}: ip returned no exact link"
        )
    linkinfo = row.get("linkinfo")
    kind = linkinfo.get("info_kind") if isinstance(linkinfo, dict) else None
    if kind != "can":
        raise RuntimeError(
            f"SocketCAN {interface}: interface is {kind!r}, not SocketCAN"
        )
    info_data = linkinfo.get("info_data") if isinstance(linkinfo, dict) else None
    timing = info_data.get("bittiming") if isinstance(info_data, dict) else None
    raw_bitrate = timing.get("bitrate") if isinstance(timing, dict) else None
    bitrate = (
        int(raw_bitrate)
        if isinstance(raw_bitrate, (int, float)) and not isinstance(raw_bitrate, bool)
        else None
    )
    flags = row.get("flags")
    up = isinstance(flags, list) and "UP" in flags
    return SocketCanState(interface=interface, up=up, bitrate=bitrate)


def _permission_failure(completed: subprocess.CompletedProcess[str]) -> bool:
    detail = (completed.stderr + "\n" + completed.stdout).lower()
    return "operation not permitted" in detail or "permission denied" in detail


def _change(interface: str, ip_args: Sequence[str], *, action: str) -> None:
    ip = _ip_command()
    command = [ip, *ip_args]
    completed = _run(command)
    if completed.returncode == 0:
        return

    elevated: subprocess.CompletedProcess[str] | None = None
    sudo = shutil.which("sudo")
    get_euid = getattr(os, "geteuid", None)
    is_root = callable(get_euid) and get_euid() == 0
    if _permission_failure(completed) and sudo is not None and not is_root:
        try:
            interactive = bool(sys.stdin.isatty())
        except (AttributeError, OSError):
            interactive = False
        elevated_command = [sudo, *(() if interactive else ("-n",)), *command]
        elevated = _run(elevated_command, timeout=60.0 if interactive else 5.0)
        if elevated.returncode == 0:
            return

    failure = elevated if elevated is not None else completed
    remedy = shlex.join(["sudo", "ip", *ip_args])
    raise RuntimeError(
        f"SocketCAN {interface}: could not {action} ({_detail(failure)}); run: {remedy}"
    )


def ensure_socketcan_up(
    interface: str,
    *,
    bitrate: int,
    restart_ms: int = 100,
    report: Callable[[str], None] = print,
) -> bool:
    """Bring one declared, currently-down SocketCAN interface up.

    ``False`` means it was already up at the declared bitrate.  An already-up
    link at another bitrate is refused rather than disrupted.  A down link is
    configured, activated, and read back before this function returns ``True``.
    Privileged changes first use the process's own capability, then the exact
    same argv through ``sudo`` (interactive on a terminal, ``-n`` otherwise).
    """

    interface = _interface(interface)
    bitrate = _bitrate(bitrate)
    restart_ms = _restart_ms(restart_ms)
    before = read_socketcan_state(interface)
    if before.up:
        if before.bitrate is None:
            raise RuntimeError(
                f"SocketCAN {interface}: interface is up but its bitrate is unavailable"
            )
        if before.bitrate != bitrate:
            raise RuntimeError(
                f"SocketCAN {interface}: interface is already up at {before.bitrate} bit/s; "
                f"the manifest expects {bitrate}; stop it before reconfiguration or fix "
                "the manifest"
            )
        return False

    _change(
        interface,
        [
            "link",
            "set",
            "dev",
            interface,
            "type",
            "can",
            "bitrate",
            str(bitrate),
            "restart-ms",
            str(restart_ms),
        ],
        action=f"configure {bitrate} bit/s",
    )
    _change(
        interface,
        ["link", "set", "dev", interface, "up"],
        action="bring the interface up",
    )
    after = read_socketcan_state(interface)
    if not after.up or after.bitrate != bitrate:
        raise RuntimeError(
            f"SocketCAN {interface}: activation did not verify "
            f"(up={after.up}, bitrate={after.bitrate!r}, expected={bitrate})"
        )
    report(f"SocketCAN {interface}: activated at {bitrate} bit/s")
    return True


__all__ = ["SocketCanState", "ensure_socketcan_up", "read_socketcan_state"]
