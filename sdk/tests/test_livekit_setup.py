"""Selected media owns its isolated service resources; no API requests in tests."""

import json
from types import SimpleNamespace

import pytest
from conftest import livekit_configuration
from live import discovery


def test_provisionable_media_still_requires_camera_and_live_execution(monkeypatch):
    for name in ("URL", "API_KEY", "API_SECRET"):
        monkeypatch.setenv("WADDLE_TEST_LIVEKIT_" + name, "configured")
    for name in ("PUBLISHER_TOKEN", "VIEWER_TOKEN"):
        monkeypatch.delenv("WADDLE_TEST_LIVEKIT_" + name, raising=False)
    config = SimpleNamespace(
        getoption=lambda name: True, live_missing={"cameras:scene": "scene absent"}
    )
    item = SimpleNamespace(
        config=config,
        callspec=SimpleNamespace(params={"camera": "scene"}),
        iter_markers=lambda name: [
            SimpleNamespace(
                args=(
                    "WADDLE_TEST_LIVEKIT_PUBLISHER_TOKEN",
                    "WADDLE_TEST_LIVEKIT_VIEWER_TOKEN",
                )
            )
        ],
        get_closest_marker=lambda name: None,
    )
    assert discovery.reason(item) == "scene absent"
    config.live_missing.clear()
    assert discovery.reason(item) is None
    config.getoption = lambda name: False
    assert discovery.reason(item) == "live tests require --live"


@pytest.mark.parametrize("deletion_fails", [False, True])
def test_selected_media_fixture_owns_only_its_room_and_reports_cleanup(
    monkeypatch, tmp_path, deletion_fails
):
    api = pytest.importorskip("livekit.api")
    actions = []

    class Client:
        def __init__(self, *args):
            self.room = self

        async def create_room(self, request):
            actions.append(("create", request.name))

        async def delete_room(self, request):
            actions.append(("delete", request.room))
            if deletion_fails:
                raise RuntimeError("room cleanup rejected")

        async def aclose(self):
            actions.append(("close", None))

    monkeypatch.setattr(api, "LiveKitAPI", Client)
    for name, value in (
        ("URL", "wss://example.invalid"),
        ("API_KEY", "fixture-key"),
        ("API_SECRET", "fixture-secret-value-with-32-bytes"),
    ):
        monkeypatch.setenv("WADDLE_TEST_LIVEKIT_" + name, value)
    for name in ("PUBLISHER_TOKEN", "VIEWER_TOKEN"):
        monkeypatch.delenv("WADDLE_TEST_LIVEKIT_" + name, raising=False)
    request = SimpleNamespace(
        config=SimpleNamespace(
            getoption=lambda name: True,
            live_bench={"evidence_directory": str(tmp_path)},
        )
    )
    fixture = livekit_configuration.__wrapped__(request)
    assert not actions
    url, publisher, viewer, identity = next(fixture)
    assert url == "wss://example.invalid"
    assert publisher != viewer and identity.startswith("sdk-media-test-")
    if deletion_fails:
        with pytest.raises(Exception, match="room cleanup rejected"):
            next(fixture)
    else:
        with pytest.raises(StopIteration):
            next(fixture)
    assert [name for name, _ in actions] == ["create", "close", "delete", "close"]
    assert actions[0][1] == actions[2][1]
    evidence = json.loads(next(tmp_path.glob("*.json")).read_text())
    assert evidence["created"] and evidence["deleted"] != deletion_fails
    assert "fixture-secret" not in json.dumps(evidence)
