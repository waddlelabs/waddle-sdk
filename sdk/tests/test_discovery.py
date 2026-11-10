from __future__ import annotations

from pathlib import Path

from waddle_sdk import discovery
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
        include_vendor_enumeration=False,
        sys_root=sys_root,
        dev_root=dev_root,
    )
    by_id = {candidate.identifier: candidate for candidate in report.candidates}

    assert by_id["linux-can:can0"].connection == {"channel": "can0"}
    realsense = by_id["linux-camera:realsense:12345"]
    assert realsense.driver is None
    assert realsense.connection == {}
    assert realsense.metadata["serial"] == ""
    assert realsense.metadata["usb_serial_evidence"] == "12345"
    assert realsense.confidence == "possible"
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
        raise RuntimeError("token=vendor-probe-secret")

    report = discover_hardware(
        providers=(openarm, broken),
        include_plugins=False,
        include_vendor_enumeration=False,
        sys_root=tmp_path / "missing-sys",
        dev_root=tmp_path / "missing-dev",
    )
    assert report.candidates[0].driver == "customer_openarm:arm"
    connection["ip"] = "changed"
    assert report.candidates[0].connection["ip"] == "192.0.2.10"
    assert report.warnings == (
        "hardware discovery provider 'broken' failed (RuntimeError)",
    )
    assert "vendor-probe-secret" not in report.warnings[0]


def test_duplicate_candidate_identifiers_are_reported_not_overwritten(
    tmp_path: Path,
) -> None:
    first = HardwareCandidate("same", "transport", "first")
    second = HardwareCandidate("same", "transport", "second")
    report = discover_hardware(
        providers=(lambda: (first,), lambda: (second,)),
        include_plugins=False,
        include_vendor_enumeration=False,
        sys_root=tmp_path / "missing-sys",
        dev_root=tmp_path / "missing-dev",
    )
    assert report.candidates == (first,)
    assert report.warnings == ("duplicate hardware candidate 'same' ignored",)


def test_vendor_realsense_serial_replaces_non_driver_sysfs_identity(
    tmp_path: Path, monkeypatch
) -> None:
    sys_root = tmp_path / "sys"
    dev_root = tmp_path / "dev"
    dev_root.mkdir()
    _write(sys_root / "class/video4linux/video0/name", "Intel RealSense D435\n")
    _write(sys_root / "class/video4linux/video0/device/serial", "usb-interface-id\n")
    _write(sys_root / "class/video4linux/video0/device/idVendor", "8086\n")
    physical = sys_root / "devices/1-2"
    physical.mkdir(parents=True)
    (physical / "serial").write_text("usb-interface-id")
    node = sys_root / "class/video4linux/video1"
    node.mkdir()
    (node / "name").write_text("Intel RealSense D435")
    (node / "device").symlink_to(physical)
    # The old fixture has no physical path: keep it as a second unresolved camera.
    exact = HardwareCandidate(
        identifier="linux-camera:realsense:207522070634",
        kind="camera",
        label="RealSense D435 207522070634",
        driver="waddle_sdk.cameras.realsense",
        connection={"serial": "207522070634"},
        metadata={
            "family": "realsense",
            "serial": "207522070634",
            "usb_path": "usb:1-2",
        },
    )
    monkeypatch.setattr(discovery, "_realsense_vendor_candidates", lambda: (exact,))

    report = discover_hardware(
        include_plugins=False,
        sys_root=sys_root,
        dev_root=dev_root,
    )

    cameras = [row for row in report.candidates if row.kind == "camera"]
    assert exact in cameras
    assert len(cameras) == 2
    assert any(row.driver is None for row in cameras)


def test_connector_default_uses_connection_preserving_hostname(monkeypatch) -> None:
    from waddle_sdk.cli import _parser

    monkeypatch.delenv("WADDLE_CONNECTOR_TARGET", raising=False)
    args = _parser().parse_args(["connect", "--site", "site.yaml"])
    assert args.target == "https://connect.waddlelabs.ai:443"
    assert args.api_url == "https://api.waddlelabs.ai"


def test_missing_vendor_reports_original_reason_for_detected_camera(
    tmp_path, monkeypatch
):
    node = tmp_path / "class/video4linux/video0"
    _write(node / "name", "Intel RealSense D435")

    def missing():
        raise ModuleNotFoundError("No module named 'pyrealsense2'")

    monkeypatch.setattr(discovery, "_realsense_vendor_candidates", missing)
    report = discover_hardware(
        include_plugins=False, sys_root=tmp_path, dev_root=tmp_path
    )
    assert len(report.candidates) == 1
    assert "No module named 'pyrealsense2'" in report.warnings[0]


def test_realsense_usb_backends_bind_the_same_physical_port():
    assert discovery._usb_path(Path("2-3.4-4")) == "usb:2-3.4"
    assert discovery._usb_path(Path("2-3.4-21")) == "usb:2-3.4"
    assert (
        discovery._usb_path(
            Path("/sys/devices/usb2/2-3/2-3.4/2-3.4:1.0/video4linux/video0")
        )
        == "usb:2-3.4"
    )


def test_can_usb_is_visible_without_driver_and_keeps_identity_when_bound(tmp_path):
    sys_root = tmp_path / "sys"
    port = sys_root / "bus/usb/devices/1-2.2"
    _write(port / "idVendor", "1d50")
    _write(port / "idProduct", "606f")
    _write(port / "serial", "adapter-one")
    _write(port / "product", "CANable 2.5 Candlelight")

    def scan():
        return discover_hardware(
            include_plugins=False,
            include_vendor_enumeration=False,
            sys_root=sys_root,
            dev_root=tmp_path / "dev",
        ).candidates

    (unbound,) = scan()
    assert unbound.metadata["setup_error"]
    assert unbound.connection == {}
    assert unbound.metadata["attachment"] == "usb"
    assert "adapter-one" in unbound.label
    _write(sys_root / "class/net/can2/type", "280")
    _write(sys_root / "class/net/can2/operstate", "down")
    (sys_root / "class/net/can2/device").symlink_to(port)
    (bound,) = scan()
    assert bound.identifier == unbound.identifier
    assert bound.connection == {"channel": "can2"}
    assert not bound.metadata.get("setup_error")
    assert bound.metadata["state"] == "down"
    # USB removal removes the candidate; no stale channel is reported.
    (sys_root / "class/net/can2/type").unlink()
    (port / "idVendor").unlink()
    assert scan() == ()


def test_platform_can_is_identified_as_builtin_not_a_connected_arm(tmp_path):
    interface = tmp_path / "class/net/can0"
    _write(interface / "type", "280")
    device = tmp_path / "devices/platform/c310000.mttcan"
    device.mkdir(parents=True)
    (device / "subsystem").symlink_to(tmp_path / "bus/platform")
    (interface / "device").symlink_to(device)
    report = discover_hardware(
        include_plugins=False,
        include_vendor_enumeration=False,
        sys_root=tmp_path,
        dev_root=tmp_path / "dev",
    )
    assert report.candidates[0].metadata["attachment"] == "built-in"
    assert "Built-in" in report.candidates[0].label


def test_orbbec_discovery_retains_serial_for_site_connection(tmp_path):
    root = tmp_path / "sys"
    for node in ["video0", "video1"]:
        _write(root / f"class/video4linux/{node}/name", "Orbbec Gemini 305")
        _write(root / f"class/video4linux/{node}/device/serial", "CV2L761000XJ")
    report = discover_hardware(
        include_plugins=False,
        include_vendor_enumeration=False,
        sys_root=root,
        dev_root=tmp_path / "dev",
    )
    cameras = [c for c in report.candidates if c.driver == "waddle_sdk.cameras.orbbec"]
    assert len(cameras) == 1
    assert cameras[0].connection == {"serial": "CV2L761000XJ"}
