from __future__ import annotations

from pathlib import Path

from waddle_sdk.discovery import HardwareCandidate, discover_hardware


def _write(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding="utf-8")


def test_linux_scan_reports_transports_and_deduplicated_cameras_without_opening(
    tmp_path: Path,
) -> None:
    sys_root = tmp_path / "sys"
    dev_root = tmp_path / "dev"
    dev_root.mkdir()

    _write(sys_root / "class/net/can0/type", "280\n")
    _write(sys_root / "class/net/can0/operstate", "up\n")
    _write(sys_root / "class/net/eth0/type", "1\n")
    _write(sys_root / "class/video4linux/video0/name", "Intel RealSense D455\n")
    _write(sys_root / "class/video4linux/video0/device/serial", "12345\n")
    _write(sys_root / "class/video4linux/video0/device/idVendor", "8086\n")
    _write(sys_root / "class/video4linux/video1/name", "Intel RealSense D455\n")
    _write(sys_root / "class/video4linux/video1/device/serial", "12345\n")
    _write(sys_root / "class/video4linux/video2/name", "Desk camera\n")
    _write(dev_root / "ttyACM0", "")

    report = discover_hardware(
        include_plugins=False,
        sys_root=sys_root,
        dev_root=dev_root,
    )
    by_id = {candidate.identifier: candidate for candidate in report.candidates}

    assert by_id["linux-can:can0"].connection == {"channel": "can0"}
    realsense = by_id["linux-camera:realsense:12345"]
    assert realsense.driver == "waddle_sdk.cameras.realsense"
    assert realsense.connection == {"serial": "12345"}
    assert len([row for row in report.candidates if "realsense" in row.identifier]) == 1
    assert any(row.driver == "waddle_sdk.cameras.usb" for row in report.candidates)
    assert any(row.identifier == "linux-serial:ttyACM0" for row in report.candidates)
    assert report.warnings == ()


def test_custom_provider_candidates_are_immutable_and_failures_are_isolated(
    tmp_path: Path,
) -> None:
    connection = {"ip": "192.0.2.10"}

    def openarm():
        return (
            HardwareCandidate(
                identifier="openarm:left",
                kind="robot",
                label="OpenArm left",
                driver="customer_openarm:arm",
                connection=connection,
            ),
        )

    def broken():
        raise RuntimeError("vendor probe unavailable")

    report = discover_hardware(
        providers=(openarm, broken),
        include_plugins=False,
        sys_root=tmp_path / "missing-sys",
        dev_root=tmp_path / "missing-dev",
    )
    assert report.candidates[0].driver == "customer_openarm:arm"
    connection["ip"] = "changed"
    assert report.candidates[0].connection["ip"] == "192.0.2.10"
    assert "vendor probe unavailable" in report.warnings[0]


def test_duplicate_candidate_identifiers_are_reported_not_overwritten(tmp_path: Path) -> None:
    first = HardwareCandidate("same", "transport", "first")
    second = HardwareCandidate("same", "transport", "second")
    report = discover_hardware(
        providers=(lambda: (first,), lambda: (second,)),
        include_plugins=False,
        sys_root=tmp_path / "missing-sys",
        dev_root=tmp_path / "missing-dev",
    )
    assert report.candidates == (first,)
    assert report.warnings == ("duplicate hardware candidate 'same' ignored",)


def test_connector_default_uses_connection_preserving_hostname(monkeypatch) -> None:
    from waddle_sdk.cli import _parser

    monkeypatch.delenv("WADDLE_CONNECTOR_TARGET", raising=False)
    args = _parser().parse_args(
        [
            "connect",
            "--site",
            "site.yaml",
            "--customer",
            "customer",
            "--project",
            "project",
            "--workspace",
            "workspace",
        ]
    )
    assert args.target == "https://connect.waddlelabs.ai:443"
    assert args.api_url == "https://api.waddlelabs.ai"
