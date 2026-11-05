"""Camera selection survives support installation and USB hotplug."""

import asyncio
import threading

import pytest
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


@pytest.mark.parametrize("page", ["robots", "cameras"])
def test_slow_discovery_is_single_flight_and_does_not_block_ui(
    tmp_path, monkeypatch, page
):
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
            app.page = app.PAGES.index(page)
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


def test_refresh_keeps_visually_selected_camera_and_connection(tmp_path, monkeypatch):
    async def check():
        row = camera()
        report = DiscoveryReport((row,))
        monkeypatch.setattr(tui, "discover_hardware", lambda: report)
        app = tui.WorkspaceInitApp(tmp_path, report)
        async with app.run_test(size=(110, 44)) as pilot:
            widget = app.query_one("#camera-candidate", OptionList)
            widget.highlighted = widget.get_option_index(row.identifier)
            widget.action_select()
            await pilot.pause()
            for _ in range(3):
                await app._scan_cameras()
                await pilot.pause()
                assert (
                    widget.get_option_at_index(widget.highlighted).id == row.identifier
                )
                assert app.selected_camera == row.identifier
                assert app._input("camera-connection") == '{"serial": "sdk-one"}'

    asyncio.run(check())


def test_arm_transport_hotplug_preserves_identity_and_blocks_disconnected_add(
    tmp_path, monkeypatch
):
    async def check():
        rows = tuple(
            HardwareCandidate(
                identifier=f"linux-can:can{i}",
                kind="transport",
                label=f"CAN interface can{i}",
                connection={"channel": f"can{i}"},
                metadata={"transport": "socketcan"},
            )
            for i in range(2)
        )
        current = DiscoveryReport(rows)
        monkeypatch.setattr(tui, "discover_hardware", lambda: current)
        app = tui.WorkspaceInitApp(tmp_path, current)
        async with app.run_test(size=(110, 44)) as pilot:
            widget = app.query_one("#transport", OptionList)
            widget.highlighted = widget.get_option_index(rows[1].identifier)
            widget.action_select()
            await pilot.pause()
            connection = app._input("part-connection")
            assert "can1" in connection
            await app._scan_cameras()
            assert (
                widget.get_option_at_index(widget.highlighted).id == rows[1].identifier
            )
            current = DiscoveryReport((rows[0],))
            await app._scan_cameras()
            assert widget.get_option(rows[1].identifier).disabled
            assert app._input("part-connection") == connection
            with pytest.raises(ValueError, match="disconnected"):
                app._add_part()
            current = DiscoveryReport(rows)
            await app._scan_cameras()
            await pilot.pause()
            assert not widget.get_option(rows[1].identifier).disabled
            assert (
                widget.get_option_at_index(widget.highlighted).id == rows[1].identifier
            )
            app._add_part()
            assert app.parts["arm"]["connection"]["channel"] == "can1"

    asyncio.run(check())


def test_usb_driver_recovery_updates_selected_channel_and_blocks_missing_driver(
    tmp_path,
    monkeypatch,
):
    async def check():
        unresolved = HardwareCandidate(
            "linux-can-usb:one",
            "transport",
            "USB CAN one",
            metadata={"transport": "socketcan", "setup_error": "Install gs_usb"},
        )
        resolved = HardwareCandidate(
            unresolved.identifier,
            "transport",
            "USB CAN one · can2",
            connection={"channel": "can2"},
            metadata={"transport": "socketcan", "state": "down"},
        )
        current = DiscoveryReport((unresolved,))
        monkeypatch.setattr(tui, "discover_hardware", lambda: current)
        app = tui.WorkspaceInitApp(tmp_path, current)
        async with app.run_test(size=(110, 44)) as pilot:
            widget = app.query_one("#transport", OptionList)
            widget.highlighted = widget.get_option_index(unresolved.identifier)
            widget.action_select()
            await pilot.pause()
            with pytest.raises(ValueError, match="Install gs_usb"):
                app._add_part()
            current = DiscoveryReport((resolved,))
            await app._scan_cameras()
            assert "can2" in app._input("part-connection")
            assert "arm connection unknown" in str(
                widget.get_option(resolved.identifier).prompt
            )
            current = DiscoveryReport(())
            await app._scan_cameras()
            assert widget.get_option(resolved.identifier).disabled
            reconnected = HardwareCandidate(
                resolved.identifier,
                "transport",
                "USB CAN one · can4",
                connection={"channel": "can4"},
                metadata=resolved.metadata,
            )
            current = DiscoveryReport((reconnected,))
            await app._scan_cameras()
            assert "can4" in app._input("part-connection")
            app._add_part()
            assert app.parts["arm"]["connection"]["channel"] == "can4"

    asyncio.run(check())
