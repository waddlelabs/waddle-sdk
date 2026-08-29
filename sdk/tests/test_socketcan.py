from __future__ import annotations

import json
import subprocess

import pytest
from waddle_sdk.robots import socketcan


def _link(*, up: bool, bitrate: int | None = None, kind: str = "can") -> str:
    info_data: dict[str, object] = {"state": "ERROR-ACTIVE" if up else "STOPPED"}
    if bitrate is not None:
        info_data["bittiming"] = {"bitrate": bitrate}
    return json.dumps(
        [
            {
                "ifname": "can_left",
                "flags": ["NOARP", "UP"] if up else ["NOARP"],
                "linkinfo": {"info_kind": kind, "info_data": info_data},
            }
        ]
    )


def _completed(
    command: list[str], *, returncode: int = 0, stdout: str = "", stderr: str = ""
) -> subprocess.CompletedProcess[str]:
    return subprocess.CompletedProcess(command, returncode, stdout, stderr)


def test_a_down_declared_socketcan_link_is_configured_then_brought_up(monkeypatch):
    calls: list[list[str]] = []
    inspections = iter((_link(up=False), _link(up=True, bitrate=1_000_000)))

    def run(command, **_kwargs):
        row = list(command)
        calls.append(row)
        if "show" in row:
            return _completed(row, stdout=next(inspections))
        return _completed(row)

    monkeypatch.setattr(socketcan.subprocess, "run", run)
    monkeypatch.setattr(socketcan.shutil, "which", lambda name: f"/usr/bin/{name}")
    reports: list[str] = []

    changed = socketcan.ensure_socketcan_up(
        "can_left", bitrate=1_000_000, report=reports.append
    )

    assert changed is True
    assert calls == [
        ["/usr/bin/ip", "-details", "-json", "link", "show", "dev", "can_left"],
        [
            "/usr/bin/ip",
            "link",
            "set",
            "dev",
            "can_left",
            "type",
            "can",
            "bitrate",
            "1000000",
            "restart-ms",
            "100",
        ],
        ["/usr/bin/ip", "link", "set", "dev", "can_left", "up"],
        ["/usr/bin/ip", "-details", "-json", "link", "show", "dev", "can_left"],
    ]
    assert reports == ["SocketCAN can_left: activated at 1000000 bit/s"]


def test_an_already_up_matching_link_is_left_untouched(monkeypatch):
    calls: list[list[str]] = []

    def run(command, **_kwargs):
        row = list(command)
        calls.append(row)
        return _completed(row, stdout=_link(up=True, bitrate=1_000_000))

    monkeypatch.setattr(socketcan.subprocess, "run", run)
    monkeypatch.setattr(socketcan.shutil, "which", lambda name: f"/usr/bin/{name}")

    changed = socketcan.ensure_socketcan_up("can_left", bitrate=1_000_000)

    assert changed is False
    assert len(calls) == 1


def test_an_up_link_at_another_bitrate_is_refused_not_reconfigured(monkeypatch):
    calls: list[list[str]] = []

    def run(command, **_kwargs):
        row = list(command)
        calls.append(row)
        return _completed(row, stdout=_link(up=True, bitrate=500_000))

    monkeypatch.setattr(socketcan.subprocess, "run", run)
    monkeypatch.setattr(socketcan.shutil, "which", lambda name: f"/usr/bin/{name}")

    with pytest.raises(
        RuntimeError, match="already up at 500000 bit/s.*expects 1000000"
    ):
        socketcan.ensure_socketcan_up("can_left", bitrate=1_000_000)

    assert len(calls) == 1


def test_a_non_can_interface_is_refused_without_mutation(monkeypatch):
    calls: list[list[str]] = []

    def run(command, **_kwargs):
        row = list(command)
        calls.append(row)
        return _completed(row, stdout=_link(up=False, kind="ether"))

    monkeypatch.setattr(socketcan.subprocess, "run", run)
    monkeypatch.setattr(socketcan.shutil, "which", lambda name: f"/usr/bin/{name}")

    with pytest.raises(RuntimeError, match="is 'ether', not SocketCAN"):
        socketcan.ensure_socketcan_up("can_left", bitrate=1_000_000)

    assert len(calls) == 1


def test_permission_failure_uses_sudo_for_only_the_declared_link(monkeypatch):
    calls: list[list[str]] = []
    inspections = iter((_link(up=False), _link(up=True, bitrate=1_000_000)))

    def run(command, **_kwargs):
        row = list(command)
        calls.append(row)
        if "show" in row:
            return _completed(row, stdout=next(inspections))
        if row[0] == "/usr/bin/ip":
            return _completed(
                row, returncode=2, stderr="RTNETLINK answers: Operation not permitted"
            )
        return _completed(row)

    monkeypatch.setattr(socketcan.subprocess, "run", run)
    monkeypatch.setattr(socketcan.shutil, "which", lambda name: f"/usr/bin/{name}")
    monkeypatch.setattr(socketcan.sys.stdin, "isatty", lambda: False)

    socketcan.ensure_socketcan_up("can_left", bitrate=1_000_000)

    sudo_calls = [row for row in calls if row[0] == "/usr/bin/sudo"]
    assert sudo_calls == [
        [
            "/usr/bin/sudo",
            "-n",
            "/usr/bin/ip",
            "link",
            "set",
            "dev",
            "can_left",
            "type",
            "can",
            "bitrate",
            "1000000",
            "restart-ms",
            "100",
        ],
        [
            "/usr/bin/sudo",
            "-n",
            "/usr/bin/ip",
            "link",
            "set",
            "dev",
            "can_left",
            "up",
        ],
    ]
