"""Discovery selects possible behavior without opening devices or altering normal tests."""

from pathlib import Path
from types import SimpleNamespace

import pytest
from live import discovery


class Config:
    def __init__(self, *, live):
        self.option = SimpleNamespace(maxfail=0)
        self.options = {"--live": live, "--live-config": None, "--live-site": None}

    def getoption(self, name, default=None):
        return self.options.get(name, default)

    def addinivalue_line(self, *args):
        pass


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
