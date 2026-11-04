"""Camera selection survives support installation and USB hotplug."""

import asyncio
import threading

from rich.text import Text
from textual.widgets import OptionList
from waddle_sdk.discovery import DiscoveryReport, HardwareCandidate
from waddle_sdk.workspace import tui


def camera(serial="sdk-one", *, ready=True):
    return HardwareCandidate(
        identifier=serial,
        kind="camera",
        label="Desk camera " + serial,
        driver="waddle_sdk.cameras.realsense" if ready else None,
        connection={"serial": serial} if ready else {},
        metadata={"family": "realsense", "usb_path": "/sys/devices/1-2"},
    )


def test_install_then_add_uses_sdk_identity(tmp_path, monkeypatch):
    async def check():
        installed = []
        unresolved = camera("usb-one", ready=False)
        resolved = camera()
        monkeypatch.setattr(
            tui, "discover_hardware", lambda: DiscoveryReport((resolved,))
        )
        app = tui.WorkspaceInitApp(
            tmp_path,
            DiscoveryReport((unresolved,)),
            prepare_drivers=lambda drivers: installed.extend(drivers),
        )
        async with app.run_test(size=(110, 44)) as pilot:
            app.selected_camera = unresolved.identifier
            await app._prepare_camera()
            app._add_camera()
            assert installed == ["waddle_sdk.cameras.realsense"]
            assert app.cameras["camera"]["connection"] == {"serial": "sdk-one"}
            await pilot.pause()
            assert app._input("camera-connection") == '{"serial": "sdk-one"}'

    asyncio.run(check())


def test_hotplug_keeps_unavailable_row_and_selection(tmp_path, monkeypatch):
    async def check():
        row = camera()
        current = DiscoveryReport((row,))
        monkeypatch.setattr(tui, "discover_hardware", lambda: current)
        app = tui.WorkspaceInitApp(tmp_path, current)
        async with app.run_test(size=(110, 44)):
            app.selected_camera = row.identifier
            await app._scan_cameras()
            current = DiscoveryReport(())
            await app._scan_cameras()
            widget = app.query_one("#camera-candidate", OptionList)
            assert widget.get_option(row.identifier).disabled
            assert app.selected_camera == row.identifier
            assert row.identifier not in app.available_cameras
            current = DiscoveryReport((row,))
            await app._scan_cameras()
            assert not widget.get_option(row.identifier).disabled
            assert app.selected_camera == row.identifier

    asyncio.run(check())


def test_slow_discovery_is_single_flight_and_does_not_block_ui(tmp_path, monkeypatch):
    async def check():
        entered, release = threading.Event(), threading.Event()
        calls = []

        def discover():
            calls.append(1)
            entered.set()
            assert release.wait(5)
            return DiscoveryReport(())

        monkeypatch.setattr(tui, "discover_hardware", discover)
        app = tui.WorkspaceInitApp(tmp_path, DiscoveryReport(()))
        async with app.run_test(size=(110, 44)):
            app.page = app.PAGES.index("cameras")
            app._poll_cameras()
            try:
                assert await asyncio.to_thread(entered.wait, 5)
                app._poll_cameras()
                app._set_input("camera-name", "still-responsive")
                assert app._input("camera-name") == "still-responsive"
                assert calls == [1]
            finally:
                release.set()
                await app.workers.wait_for_complete()

    asyncio.run(check())


def test_preview_renders_rgb_without_display_and_closes(monkeypatch):
    from waddle_sdk.workspace.camera_preview import snapshot

    monkeypatch.delenv("DISPLAY", raising=False)
    preview = snapshot(
        "waddle_sdk.cameras.mock", "{}", {"width": 64, "height": 48, "fps": 20}
    )
    assert isinstance(preview, Text)
    assert "▀" in preview.plain
    assert preview.spans
    assert "camera closed" in preview.plain
