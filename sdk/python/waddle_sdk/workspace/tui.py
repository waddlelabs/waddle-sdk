"""Keyboard workspace setup, using public SDK discovery and reviewed presets."""

import asyncio
import json
import re
from collections.abc import Mapping
from pathlib import Path
from typing import ClassVar

import yaml
from textual import on
from textual.app import App, ComposeResult
from textual.containers import Horizontal, VerticalScroll
from textual.widgets import (
    Button,
    Checkbox,
    Collapsible,
    ContentSwitcher,
    Input,
    Select,
    SelectionList,
    Static,
)

from waddle_sdk import load_site
from waddle_sdk.discovery import DiscoveryReport, discover_hardware
from waddle_sdk.robots import safety_presets_for_driver

from .initialization import build_site, make_site
from .profiles import YAM_DRIVER, camera_defaults, part_defaults


def _plain(value):
    if isinstance(value, Mapping):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return value


def _json(raw, kind, label):
    value = json.loads(raw)
    if not isinstance(value, kind):
        raise ValueError(f"{label} must be a JSON {kind.__name__}")  # noqa: TRY004 -- terminal input validation
    return value


class WorkspaceInitApp(App):
    """Retained robot/camera/safety/review workflow over the new local contract."""

    TITLE = "Waddle SDK · New site"
    ENABLE_COMMAND_PALETTE = False
    BINDINGS: ClassVar = [
        ("ctrl+q", "cancel", "Cancel"),
        ("ctrl+r", "review", "Review"),
    ]
    CSS = """
    Screen { background: $surface; }
    #title { height: 3; padding: 1 2 0 2; text-style: bold; }
    #steps { height: 2; padding: 0 2; color: $text-muted; }
    #pages { height: 1fr; }
    .page { padding: 0 2 1 2; }
    .field { margin-top: 1; height: auto; }
    .help { color: $text-muted; height: auto; margin-bottom: 1; }
    Input, Select { margin-bottom: 0; }
    SelectionList { height: 5; margin-top: 1; }
    .actions { height: 3; margin-top: 1; }
    .actions Button { margin-right: 1; }
    #navigation { height: 3; padding: 0 2; }
    #status { height: auto; max-height: 4; padding: 0 2; color: $error; }
    #footer { height: 1; padding: 0 2; color: $text-muted; }
    #review { height: auto; }
    """
    PAGES = ("workspace", "robots", "cameras", "safety", "review")

    def __init__(
        self,
        destination: Path,
        report: DiscoveryReport,
        *,
        source_site: Path | None = None,
        host: str | None = None,
        hosts=(),
        builder=build_site,
        prepare_drivers=None,
    ):
        super().__init__()
        self.destination = destination
        self.report = report
        self.source_site = source_site
        self.hosts = tuple(hosts)
        self.host = host or (self.hosts[0] if len(self.hosts) == 1 else "none")
        self.builder = builder
        self.prepare_drivers = prepare_drivers
        self.parts = {}
        self.cameras = {}
        self.presets = {}
        self.page = 0
        self.building = False
        self.created = None
        self.transports = {
            row.identifier: row
            for row in report.candidates
            if row.kind in {"transport", "robot"}
        }
        self.camera_candidates = {
            row.identifier: row for row in report.candidates if row.kind == "camera"
        }

    def compose(self) -> ComposeResult:
        yield Static(self.TITLE, id="title")
        yield Static("", id="steps")
        with ContentSwitcher(initial="page-workspace", id="pages"):
            with VerticalScroll(id="page-workspace", classes="page"):
                yield Static("Workspace directory", classes="field")
                yield Input(str(self.destination), id="destination")
                yield Static("Site name", classes="field")
                yield Input(self.destination.name, id="site-name")
                yield Static("Control interface", classes="field")
                yield Select(
                    [("Local site", "none")]
                    + [
                        (name.replace("-", " ").title() + " browser", name)
                        for name in self.hosts
                    ],
                    value=self.host,
                    allow_blank=False,
                    id="host",
                )
                yield Static("Existing SDK site to reuse (optional)", classes="field")
                yield Input(str(self.source_site or ""), id="source-site")
                yield Static(
                    "Use a site.yaml file or its directory. Hardware settings and local "
                    "calibration are copied; the source is preserved.",
                    classes="help",
                )
                if self.report.warnings:
                    yield Static(
                        "\n".join(self.report.warnings), markup=False, classes="help"
                    )
            with VerticalScroll(id="page-robots", classes="page"):
                yield Static("Add each arm to this workspace", classes="help")
                yield Select(
                    [("YAM · SocketCAN", "yam"), ("Simulation", "simulation")],
                    value="yam" if self.transports else "simulation",
                    allow_blank=False,
                    id="profile",
                )
                yield Select(
                    [("Enter connection manually", "manual")]
                    + [(row.label, key) for key, row in self.transports.items()],
                    value="manual",
                    allow_blank=False,
                    id="transport",
                )
                with Horizontal(classes="actions"):
                    yield Button("Add arm", id="add-part", variant="primary")
                    yield Button("Remove selected", id="remove-parts")
                yield SelectionList(id="parts")
                with Collapsible(title="Advanced arm settings"):
                    yield Static("Arm name", classes="field")
                    yield Input("arm", id="part-name")
                    yield Static("Connection settings", classes="field")
                    yield Input("{}", id="part-connection")
                    yield Static("Base frame", classes="field")
                    yield Input("arm_base", id="part-frame")
                    yield Static("Joint limits", classes="field")
                    yield Input("{}", id="part-limits")
                    yield Static("Driver options", classes="field")
                    yield Input("{}", id="part-options")
                yield Static(
                    "Review the SDK defaults for this unit, including motor gripper "
                    "endpoints and gains. CAN detection identifies a transport, not an arm.",
                    classes="help",
                )
            with VerticalScroll(id="page-cameras", classes="page"):
                yield Static(
                    "Select a camera and its mount, then choose Add camera.",
                    classes="help",
                )
                yield Select(
                    [("Enter camera manually", "manual")]
                    + [(row.label, key) for key, row in self.camera_candidates.items()],
                    value="manual",
                    allow_blank=False,
                    id="camera-candidate",
                )
                yield Select(
                    [("Scene · fixed camera", "scene")],
                    value="scene",
                    allow_blank=False,
                    id="camera-mount",
                )
                with Horizontal(classes="actions"):
                    yield Button("Add camera", id="add-camera", variant="primary")
                    yield Button("Remove selected", id="remove-cameras")
                yield SelectionList(id="cameras")
                yield Button("Preview selected camera", id="preview-camera")
                yield Static(
                    "Preview opens only this camera, captures a snapshot, then closes it.",
                    id="camera-preview",
                    markup=False,
                )
                with Collapsible(title="Advanced camera settings"):
                    yield Static("Camera name", classes="field")
                    yield Input("camera", id="camera-name")
                    yield Static("Camera type", classes="field")
                    yield Select(
                        [
                            ("RealSense", "realsense"),
                            ("USB camera", "usb"),
                            ("Orbbec", "orbbec"),
                            ("Simulation", "simulation"),
                        ],
                        value="realsense",
                        allow_blank=False,
                        id="camera-profile",
                    )
                    yield Static("Connection settings", classes="field")
                    yield Input("{}", id="camera-connection")
                    yield Static("Camera frame", classes="field")
                    yield Input("camera_optical", id="camera-frame")
                    yield Static(
                        "Stream · width x height @ frames per second", classes="field"
                    )
                    yield Input("640x480@30", id="camera-stream")
            with VerticalScroll(id="page-safety", classes="page"):
                yield Static("Workspace bounds", classes="field")
                yield Select(
                    [("Enter measured bounds", "manual")],
                    value="manual",
                    allow_blank=False,
                    id="safety-preset",
                )
                yield Button("Use selected starting bounds", id="apply-preset")
                yield Static("", id="preset-note", classes="help", markup=False)
                yield Input("{}", id="bounds")
                yield Static("Static keep-outs", classes="field")
                yield Input("[]", id="keepouts")
                yield Static("Self/cross-arm collision rules", classes="field")
                yield Input("{}", id="self-collision")
                yield Static(
                    "Workspace bounds and "
                    "clearance from the table, other arms and payloads need site review. "
                    "Creating a workspace does not move hardware or calibrate it.",
                    classes="help",
                )
                yield Checkbox(
                    "I reviewed the hardware settings and physical workspace",
                    id="safety-reviewed",
                )
            with VerticalScroll(id="page-review", classes="page"):
                yield Static("", id="review", markup=False)
                yield Button("Create workspace", id="create", variant="success")
        with Horizontal(id="navigation"):
            yield Button("Back", id="back")
            yield Button("Next", id="next", variant="primary")
            yield Button("Cancel", id="cancel")
        yield Static("", id="status", markup=False)
        yield Static(
            "Tab / Shift+Tab fields · Enter select · Ctrl+R review · Ctrl+Q cancel",
            id="footer",
        )

    def on_mount(self):
        self._part_defaults()
        self._camera_defaults()
        self._show_page()

    def _input(self, name):
        return self.query_one("#" + name, Input).value.strip()

    def _set_input(self, name, value):
        self.query_one("#" + name, Input).value = value

    def _select(self, name):
        return str(self.query_one("#" + name, Select).value)

    def _status(self, message=""):
        self.query_one("#status", Static).update(message)

    async def _prepare(self, drivers):
        if self.prepare_drivers is None or not drivers:
            return
        self.building = True
        self._status("Installing support for the selected hardware…")
        try:
            await asyncio.to_thread(self.prepare_drivers, drivers)
        finally:
            self.building = False
        self._status()

    async def _discover_cameras(self):
        if self.prepare_drivers is None:
            return
        # RealSense USB evidence can exist before its SDK is installed. Install
        # the detected camera adapters, then enumerate driver-valid selectors.
        drivers = []
        for row in self.camera_candidates.values():
            family = row.metadata.get("family")
            driver = row.driver
            if driver is None and family == "realsense":
                driver = "waddle_sdk.cameras.realsense"
            if driver:
                drivers.append(driver)
        await self._prepare(drivers)
        self.report = await asyncio.to_thread(discover_hardware)
        self.camera_candidates = {
            row.identifier: row
            for row in self.report.candidates
            if row.kind == "camera"
        }
        selector = self.query_one("#camera-candidate", Select)
        selected = selector.value
        selector.set_options(
            [("Enter camera manually", "manual")]
            + [(row.label, key) for key, row in self.camera_candidates.items()]
        )
        selector.value = selected if selected in self.camera_candidates else "manual"
        if self.report.warnings:
            self._status("\n".join(self.report.warnings))

    def _part_defaults(self):
        row = part_defaults(self._select("profile"), self._input("part-name") or "arm")
        for field, key in (
            ("part-connection", "connection"),
            ("part-limits", "joint_limits"),
            ("part-options", "options"),
        ):
            self._set_input(field, json.dumps(row.get(key, {})))
        self._set_input("part-frame", row["base_frame"])

    def _camera_defaults(self):
        row = camera_defaults(
            self._select("camera-profile"), self._input("camera-name") or "camera"
        )
        self._set_input("camera-connection", json.dumps(row.get("connection", {})))
        self._set_input("camera-frame", row["frame_id"])
        stream = row["stream"]
        self._set_input(
            "camera-stream", f"{stream['width']}x{stream['height']}@{stream['fps']:g}"
        )

    @on(Select.Changed, "#profile")
    def profile_changed(self):
        if self.is_mounted:
            self._part_defaults()

    @on(Select.Changed, "#camera-profile")
    def camera_profile_changed(self):
        if self.is_mounted:
            self._camera_defaults()

    @on(Select.Changed, "#transport")
    def transport_changed(self):
        selected = self.transports.get(self._select("transport"))
        if selected is not None:
            defaults = part_defaults(
                self._select("profile"), self._input("part-name") or "arm"
            )
            connection = {**defaults["connection"], **dict(selected.connection)}
            self._set_input("part-connection", json.dumps(connection))

    @on(Select.Changed, "#camera-candidate")
    async def camera_candidate_changed(self):
        selected = self.camera_candidates.get(self._select("camera-candidate"))
        if selected is None:
            return
        profile = {
            "waddle_sdk.cameras.realsense": "realsense",
            "waddle_sdk.cameras.usb": "usb",
            "waddle_sdk.cameras.orbbec": "orbbec",
        }.get(selected.driver)
        if profile is None:
            self._status(
                "This candidate needs an explicit installed SDK adapter and valid connection settings."
            )
            return
        self.query_one("#camera-profile", Select).value = profile
        self.query_one("#camera-preview", Static).update(
            "Orbbec's adapter uses its default device; exact serial selection is unavailable."
            if profile == "orbbec"
            else "Preview the selected camera to identify its view."
        )
        await asyncio.sleep(0)
        self.call_after_refresh(
            self._set_input, "camera-connection", json.dumps(dict(selected.connection))
        )

    def _refresh_parts(self):
        widget = self.query_one("#parts", SelectionList)
        widget.clear_options()
        widget.add_options(
            [
                (f"{name} · {row['driver']} · {row['base_frame']}", name)
                for name, row in self.parts.items()
            ]
        )
        self.query_one("#camera-mount", Select).set_options(
            [("Scene · fixed camera", "scene")]
            + [(f"Wrist · {name}", "wrist:" + name) for name in self.parts]
        )
        self.query_one("#camera-mount", Select).value = "scene"
        self.presets = {}
        for row in self.parts.values():
            report = safety_presets_for_driver(
                row["driver"], options=row.get("options", {})
            )
            for preset in report.presets:
                self.presets.setdefault(preset.identifier, preset)
        self.query_one("#safety-preset", Select).set_options(
            [("Enter measured bounds", "manual")]
            + [(row.label, key) for key, row in self.presets.items()]
        )
        self.query_one("#safety-preset", Select).value = next(
            iter(self.presets), "manual"
        )
        if (
            self.parts
            and all(row["driver"] == YAM_DRIVER for row in self.parts.values())
            and self._input("bounds") == "{}"
        ):
            preset = self.presets.get("yam-tabletop")
            if preset is not None:
                self._set_input("bounds", json.dumps(_plain(preset.workspace_bounds)))
                self.query_one("#preset-note", Static).update(preset.review)
        self.query_one("#safety-reviewed", Checkbox).value = False

    def _add_part(self):
        name = self._input("part-name")
        if not name or name in self.parts:
            raise ValueError("Choose a unique arm name")
        row = part_defaults(self._select("profile"), name)
        row["connection"] = _json(self._input("part-connection"), dict, "Connection")
        row["options"] = _json(self._input("part-options"), dict, "Driver options")
        limits = json.loads(self._input("part-limits"))
        if not isinstance(limits, (dict, list)):
            raise ValueError("Joint limits must be an object or array")  # noqa: TRY004 -- terminal input validation
        row["joint_limits"] = limits
        row["base_frame"] = self._input("part-frame")
        if self._select("profile") == "yam" and not row["connection"].get("channel"):
            raise ValueError("Select or enter the CAN interface for this YAM")
        self.parts[name] = row
        self._refresh_parts()
        self._set_input("part-name", f"arm{len(self.parts) + 1}")
        self._part_defaults()
        self._status(f"Added {name}")

    def _add_camera(self):
        name = self._input("camera-name")
        if not name or name in self.cameras:
            raise ValueError("Choose a unique camera name")
        row = camera_defaults(self._select("camera-profile"), name)
        row["connection"] = _json(
            self._input("camera-connection"), dict, "Camera connection"
        )
        row["frame_id"] = self._input("camera-frame")
        stream = re.fullmatch(
            r"(\d+)x(\d+)@(\d+(?:\.\d+)?)", self._input("camera-stream")
        )
        if stream is None:
            raise ValueError("Use a stream such as 640x480@30")
        row["stream"] = dict(
            zip(
                ("width", "height", "fps"),
                (int(stream[1]), int(stream[2]), float(stream[3])),
                strict=True,
            )
        )
        mount = self._select("camera-mount")
        row["mount"] = (
            {"kind": "scene"}
            if mount == "scene"
            else {"kind": "wrist", "part": mount.removeprefix("wrist:")}
        )
        if self._select("camera-profile") == "realsense" and not row["connection"].get(
            "serial"
        ):
            raise ValueError("Select a RealSense device or enter its SDK serial")
        self.cameras[name] = row
        self._refresh_cameras()
        self._set_input("camera-name", f"camera{len(self.cameras) + 1}")
        self._set_input("camera-frame", f"camera{len(self.cameras) + 1}_optical")
        self._status(f"Added {name}")

    def _refresh_cameras(self):
        widget = self.query_one("#cameras", SelectionList)
        widget.clear_options()
        widget.add_options(
            [
                (f"{name} · {row['mount']['kind']} · {row['frame_id']}", name)
                for name, row in self.cameras.items()
            ]
        )

    def _site(self):
        source = self._input("source-site")
        if source:
            path = Path(source).expanduser()
            if path.is_dir():
                path /= "site.yaml"
            load_site(path)
            return yaml.safe_load(path.read_text()), path
        if not self.parts:
            raise ValueError("Add at least one arm, or select an existing site")
        return make_site(
            site_id=self._input("site-name"),
            parts=self.parts,
            cameras=self.cameras,
            workspace_bounds=_json(self._input("bounds"), dict, "Workspace bounds"),
            static_keepouts=_json(self._input("keepouts"), list, "Static keep-outs"),
            self_collision=_json(
                self._input("self-collision"), dict, "Collision rules"
            ),
        ), None

    def _review(self):
        site, source = self._site()
        self.host = self._select("host")
        lines = [
            f"Create {Path(self._input('destination')).expanduser()}",
            f"Site: {site['metadata']['id']}",
            f"Interface: {self.host}",
            "",
        ]
        if source:
            lines += [
                f"Copy settings from {source}",
                "Source files and hardware ownership are preserved.",
                "",
            ]
        lines += ["Arms:"] + [
            f"  {name} · {row['driver']} · {row.get('base_frame', 'frame required')}"
            for name, row in site["parts"].items()
        ]
        lines += ["", "Cameras:"] + [
            f"  {name} · {row.get('mount', {})}"
            for name, row in site.get("cameras", {}).items()
        ]
        if not site.get("cameras"):
            lines.append("  None selected")
        lines += [
            "",
            "Workspace bounds: " + json.dumps(site.get("workspace_bounds", {})),
            "",
            "The workspace includes site settings and local artifact directories.",
            "Capture Home/Parking and perform guided calibration after connecting.",
        ]
        if self.host != "none":
            lines.append(
                "The selected interface will save its connection settings after workspace preparation."
            )
        self.query_one("#review", Static).update("\n".join(lines))

    def _show_page(self):
        self.query_one("#pages", ContentSwitcher).current = (
            "page-" + self.PAGES[self.page]
        )
        self.query_one("#steps", Static).update(
            "   ".join(
                ("● " if i == self.page else "") + name.title()
                for i, name in enumerate(self.PAGES)
            )
        )
        self.query_one("#back", Button).disabled = self.page == 0
        self.query_one("#next", Button).disabled = self.page == len(self.PAGES) - 1

    async def _create(self):
        site, source = self._site()
        physical = any(
            row.get("driver") != "waddle_sdk.robots.mujoco:arm"
            for row in site["parts"].values()
        )
        if (
            source is None
            and physical
            and not self.query_one("#safety-reviewed", Checkbox).value
        ):
            self.page = self.PAGES.index("safety")
            self._show_page()
            raise ValueError(
                "Review the hardware settings and workspace bounds before creating this site"
            )
        self.building = True
        self.query_one("#create", Button).disabled = True
        self._status("Preparing and validating the workspace…")
        try:
            arguments = {"source_site": source} if source else {"site": site}
            self.created = await asyncio.to_thread(
                self.builder, Path(self._input("destination")).expanduser(), **arguments
            )
        finally:
            self.building = False
            self.query_one("#create", Button).disabled = False
        self.host = self._select("host")
        self.exit(self.created)

    @on(Button.Pressed)
    async def pressed(self, event):
        if self.building:
            return
        self._status()
        try:
            action = event.button.id
            if action == "add-part":
                row = part_defaults(
                    self._select("profile"), self._input("part-name") or "arm"
                )
                await self._prepare([row["driver"]])
                self._add_part()
            elif action == "preview-camera":
                from .camera_preview import snapshot

                row = camera_defaults(self._select("camera-profile"), "setup-preview")
                await self._prepare([row["driver"]])
                match = re.fullmatch(r"(\d+)x(\d+)@(\d+)", self._input("camera-stream"))
                if match is None:
                    raise ValueError(
                        "Preview requires an integer stream such as 640x480@30"
                    )
                stream = dict(
                    zip(
                        ("width", "height", "fps"),
                        map(int, match.groups()),
                        strict=True,
                    )
                )
                self.building = True
                self._status("Opening the selected camera for one snapshot…")
                connection = self._input("camera-connection")
                profile = self._select("camera-profile")
                try:
                    preview = await asyncio.to_thread(
                        snapshot,
                        row["driver"],
                        connection,
                        stream,
                    )
                    if (connection, profile) == (
                        self._input("camera-connection"),
                        self._select("camera-profile"),
                    ):
                        self.query_one("#camera-preview", Static).update(preview)
                        self._status("Camera preview captured and camera closed.")
                    else:
                        self._status(
                            "Camera selection changed; preview the selected camera again."
                        )
                finally:
                    self.building = False
            elif action == "add-camera":
                row = camera_defaults(
                    self._select("camera-profile"),
                    self._input("camera-name") or "camera",
                )
                await self._prepare([row["driver"]])
                self._add_camera()
            elif action == "remove-parts":
                for name in tuple(self.query_one("#parts", SelectionList).selected):
                    if any(
                        row["mount"].get("part") == name
                        for row in self.cameras.values()
                    ):
                        raise ValueError(
                            "Remove or reassign wrist cameras before removing their arm"
                        )
                    self.parts.pop(name)
                self._refresh_parts()
            elif action == "remove-cameras":
                for name in tuple(self.query_one("#cameras", SelectionList).selected):
                    self.cameras.pop(name)
                self._refresh_cameras()
            elif action == "apply-preset":
                preset = self.presets.get(self._select("safety-preset"))
                if preset is not None:
                    for field, value in (
                        ("bounds", preset.workspace_bounds),
                        ("keepouts", preset.static_keepouts),
                        ("self-collision", preset.self_collision),
                    ):
                        self._set_input(field, json.dumps(_plain(value)))
                    self.query_one("#preset-note", Static).update(preset.review)
                    self.query_one("#safety-reviewed", Checkbox).value = False
            elif action in {"next", "back"}:
                target = self.page + (1 if action == "next" else -1)
                if action == "next" and self.page == 0 and self._input("source-site"):
                    target = self.PAGES.index("review")
                if target == self.PAGES.index("review"):
                    self._review()
                self.page = max(0, min(len(self.PAGES) - 1, target))
                self._show_page()
                if self.page == self.PAGES.index("cameras"):
                    await self._discover_cameras()
            elif action == "create":
                await self._create()
            elif action == "cancel":
                self.action_cancel()
        except (ValueError, OSError, RuntimeError) as error:
            self._status(str(error))

    def action_review(self):
        if self.building:
            return
        try:
            self._review()
        except (ValueError, OSError) as error:
            self._status(str(error))
            return
        self.page = self.PAGES.index("review")
        self._show_page()

    def action_cancel(self):
        if not self.building:
            self.exit(None)


def run_initializer(
    destination: Path,
    *,
    source_site=None,
    host=None,
    hosts=(),
    builder=build_site,
    prepare_drivers=None,
):
    report = DiscoveryReport(()) if source_site else discover_hardware()
    app = WorkspaceInitApp(
        destination,
        report,
        source_site=source_site,
        host=host,
        hosts=hosts,
        builder=builder,
        prepare_drivers=prepare_drivers,
    )
    result = app.run()
    return result, app.host
