"""Drift guards for the strict public SDK contract."""

from __future__ import annotations

import ast
from pathlib import Path

import waddle_sdk

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib  # type: ignore[no-redef]


SDK = Path(__file__).resolve().parents[1]
README = (SDK / "README.md").read_text(encoding="utf-8")
EXAMPLES = (SDK / "examples" / "README.md").read_text(encoding="utf-8")


def test_examples_publish_only_the_site_lifecycle():
    program = (SDK / "examples" / "run_site.py").read_text(encoding="utf-8")
    ast.parse(program)
    assert "waddle_sdk.load_site(" in program
    assert "with site.open(" in program
    assert "with session.run(" in program
    for removed in ("waddle_sdk.init(", "waddle_sdk.agent(", "waddle_sdk.ui()"):
        assert removed not in program
        assert removed not in EXAMPLES


def test_camera_install_commands_are_held_to_package_metadata():
    with (SDK / "pyproject.toml").open("rb") as stream:
        extras = tomllib.load(stream)["project"]["optional-dependencies"]

    assert extras["depthai"] == ["depthai>=3,<4"]
    assert extras["orbbec"] == ["pyorbbecsdk2"]
    assert extras["realsense"] == ["pyrealsense2"]
    assert extras["usb"] == ["opencv-python-headless>=4.8"]
    assert set(extras["cameras"]) == set(
        extras["depthai"] + extras["orbbec"] + extras["realsense"] + extras["usb"]
    )
    for extra in ("depthai", "orbbec", "realsense", "usb", "cameras"):
        assert f"waddle-sdk[{extra}]" in README
    assert "waddle-sdk[cameras,teleop]" in README


def test_readme_holds_the_site_and_runtime_boundaries():
    prose = " ".join(README.split())
    assert 'waddle_sdk.load_site("site.yaml")' in README
    assert "SdkRuntimePort" in README
    assert "Guided calibration orchestration belongs to Metal" in prose
    assert "hold-first" in prose
    assert "waddle.execution.v1" not in README
    assert "waddle_sdk.ui()" not in README
    assert "waddle_sdk.agent(" not in README
    assert "waddle_sdk.init(" not in README
    assert set(waddle_sdk.__all__) == {
        "Grpc",
        "LiveKit",
        "ManifestError",
        "ManifestPathError",
        "ManifestSyntaxError",
        "ManifestValidationError",
        "Outcome",
        "Run",
        "Site",
        "SiteSession",
        "load_site",
    }
    for removed in (
        "Control",
        "Handoff",
        "init",
        "shutdown",
        "rollout",
        "agent",
        "ui",
        "task_session",
        "calibration_click",
        "calibration_updates",
        "request_workspace_artifact",
        "execution_backends",
    ):
        assert not hasattr(waddle_sdk, removed)
    assert not (SDK / "python" / "waddle_sdk" / "_ui.py").exists()
    assert not (SDK / "python" / "waddle_sdk" / "_services.py").exists()
    assert not (SDK / "python" / "waddle_sdk" / "_testing.py").exists()
