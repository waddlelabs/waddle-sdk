"""Discovery selects possible behavior without opening devices or altering normal tests."""

from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from live import discovery
from test_site_api import _write_site


class Config:
    def __init__(self, *, live):
        self.option = SimpleNamespace(maxfail=0)
        self.options = {"--live": live, "--live-config": None, "--live-site": None}

    def getoption(self, name, default=None):
        return self.options.get(name, default)

    def addinivalue_line(self, *args):
        pass


@pytest.mark.parametrize(
    "connection,metadata_serial,driver,eligible",
    [
        ({}, "camera-1", "waddle_sdk.cameras.orbbec", True),
        ({}, "wrong-camera", "waddle_sdk.cameras.orbbec", False),
        ({"serial": "wrong-camera"}, "camera-1", "waddle_sdk.cameras.orbbec", False),
        ({"serial": "camera-1"}, "wrong-camera", "waddle_sdk.cameras.orbbec", True),
        ({}, "camera-1", "different.camera", False),
    ],
)
def test_explicit_camera_serial_can_match_metadata_but_connection_remains_authoritative(
    tmp_path, monkeypatch, connection, metadata_serial, driver, eligible
):
    from waddle_sdk.discovery import DiscoveryReport, HardwareCandidate

    path = _write_site(tmp_path)
    document = yaml.safe_load(path.read_text())
    document["cameras"]["overhead"]["driver"] = "waddle_sdk.cameras.orbbec"
    document["cameras"]["overhead"]["connection"] = {"serial": "camera-1"}
    path.write_text(yaml.safe_dump(document))
    candidate = HardwareCandidate(
        identifier="camera-evidence",
        kind="camera",
        label="camera",
        driver=driver,
        connection=connection,
        metadata={"serial": metadata_serial},
    )
    monkeypatch.setattr(
        discovery, "discover_hardware", lambda: DiscoveryReport((candidate,))
    )
    for key in ("CI", "GITHUB_ACTIONS", "WADDLE_SDK_LIVE_CONFIG", "WADDLE_LIVE_SITE"):
        monkeypatch.delenv(key, raising=False)
    config = Config(live=True)
    config.options["--live-site"] = str(path)
    discovery.pytest_configure(config)
    item = SimpleNamespace(
        config=config, callspec=SimpleNamespace(params={"camera": "overhead"})
    )
    assert (discovery.reason(item) is None) is eligible


def test_default_collection_does_not_discover_devices_or_skip_ordinary_tests(
    monkeypatch,
):
    def forbidden():
        raise AssertionError("ordinary collection queried hardware")

    monkeypatch.setattr(discovery, "discover_hardware", forbidden)
    config = Config(live=False)
    discovery.pytest_configure(config)
    root = Path(discovery.__file__).parent
    ordinary = SimpleNamespace(
        path=root.parent / "test_site_api.py", config=config, markers=[]
    )
    live = SimpleNamespace(path=root / "test_02_motion.py", config=config, markers=[])
    for item in (ordinary, live):
        item.add_marker = item.markers.append
    discovery.pytest_collection_modifyitems(config, [ordinary, live])
    assert not ordinary.markers
    assert any(marker.name == "skip" for marker in live.markers)


def test_camera_can_be_selected_without_motion_site_or_robot(monkeypatch):
    from waddle_sdk.discovery import DiscoveryReport, HardwareCandidate

    camera = HardwareCandidate(
        identifier="usb-test",
        kind="camera",
        label="camera",
        driver="camera_vendor",
        connection={"serial": "123"},
    )
    monkeypatch.setattr(
        discovery, "discover_hardware", lambda: DiscoveryReport((camera,))
    )
    monkeypatch.delenv("CI", raising=False)
    monkeypatch.delenv("GITHUB_ACTIONS", raising=False)
    monkeypatch.delenv("WADDLE_SDK_LIVE_CONFIG", raising=False)
    monkeypatch.delenv("WADDLE_LIVE_SITE", raising=False)
    config = Config(live=True)
    discovery.pytest_configure(config)

    def item(params):
        return SimpleNamespace(config=config, callspec=SimpleNamespace(params=params))

    assert discovery.reason(item({"camera": "usb-test"})) is None
    assert "no configured site" in discovery.reason(item({"case": None}))


def test_release_environment_cannot_enable_live_devices(monkeypatch):
    monkeypatch.setenv("GITHUB_ACTIONS", "true")
    with pytest.raises(pytest.UsageError, match="CI"):
        discovery.pytest_configure(Config(live=True))


def test_passive_openarm_selection_needs_two_declared_monitors_but_no_torque_release(
    monkeypatch,
):
    config = Config(live=True)
    config.live_bench = {"site": "test-site", "parts": ["bus_a", "bus_b"]}
    config.live_missing = {"shutdown": "torque release not authorized"}
    rows = {
        name: {"driver": "waddle_sdk.robots.openarm:arm", "posture": "monitor"}
        for name in config.live_bench["parts"]
    }
    monkeypatch.setattr(
        discovery, "load_site", lambda _: SimpleNamespace(manifest={"parts": rows})
    )
    item = SimpleNamespace(
        config=config, get_closest_marker=lambda name: name == "openarm_monitor"
    )
    assert discovery.reason(item) is None
    config.live_missing["parts:bus_b"] = "second bus is absent"
    assert discovery.reason(item) == "second bus is absent"
    del config.live_missing["parts:bus_b"]
    rows["bus_b"]["posture"] = "supervised"
    assert "monitor-only" in discovery.reason(item)
    config.live_bench["parts"] = ["bus_a"]
    assert "exactly two" in discovery.reason(item)


def test_missing_arm_only_disables_its_own_robot_cases():
    config = Config(live=True)
    config.live_bench = {"parts": ["left", "right"]}
    config.live_missing = {"parts:right": "right CAN interface is absent"}

    def item(params):
        return SimpleNamespace(config=config, callspec=SimpleNamespace(params=params))

    assert discovery.reason(item({"part": "left"})) is None
    assert discovery.reason(item({"case": {"part": "left"}})) is None
    assert discovery.reason(item({"part": "right"})) == "right CAN interface is absent"


def test_camera_service_requires_both_credentials_and_available_camera(monkeypatch):
    config = Config(live=True)
    config.live_missing = {"cameras:scene": "scene camera is absent"}
    item = SimpleNamespace(
        config=config,
        callspec=SimpleNamespace(params={"camera": "scene"}),
        iter_markers=lambda name: [SimpleNamespace(args=("TEST_MEDIA_GRANT",))],
        get_closest_marker=lambda name: None,
    )
    monkeypatch.setenv("TEST_MEDIA_GRANT", "scoped-test-value")
    assert discovery.reason(item) == "scene camera is absent"
    config.live_missing = {}
    assert discovery.reason(item) is None
    monkeypatch.delenv("TEST_MEDIA_GRANT")
    assert discovery.reason(item) == "missing TEST_MEDIA_GRANT"
