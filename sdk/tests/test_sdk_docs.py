"""Drift guards for the strict public SDK contract."""

from __future__ import annotations

import ast
from pathlib import Path

import waddle_sdk
import yaml

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib  # type: ignore[no-redef]


SDK = Path(__file__).resolve().parents[1]
REPO = SDK.parent
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

    assert extras["orbbec"] == ["pyorbbecsdk2"]
    assert extras["realsense"] == ["pyrealsense2"]
    assert extras["usb"] == ["opencv-python-headless>=4.8"]
    assert set(extras["cameras"]) == set(
        extras["orbbec"] + extras["realsense"] + extras["usb"]
    )
    for extra in ("orbbec", "realsense", "usb", "cameras"):
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
    assert "ConnectorRegistrationError" in README
    assert "ConnectorCompatibilityWarning" in README
    assert set(waddle_sdk.__all__) == {
        "ConnectorCompatibilityWarning",
        "ConnectorRegistrationError",
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


def test_release_is_gated_and_the_distribution_pair_is_atomic():
    ci_path = REPO / ".github" / "workflows" / "ci.yml"
    release_path = REPO / ".github" / "workflows" / "release.yml"
    ci = ci_path.read_text(encoding="utf-8")
    release_text = release_path.read_text(encoding="utf-8")
    jobs = yaml.safe_load(release_text)["jobs"]

    assert jobs["quality"]["uses"] == "./.github/workflows/ci.yml"
    assert jobs["wheels"]["needs"] == ["quality"]
    assert jobs["teleop-wheel"]["needs"] == ["quality"]
    assert jobs["publish-sdk"]["needs"] == ["wheels", "teleop-wheel"]
    assert jobs["publish-teleop"]["needs"] == ["wheels", "teleop-wheel"]
    assert "continue-on-error" not in str(jobs)

    for gate in (
        "uv run --no-sync pytest",
        "cargo test --workspace --locked",
        "cargo clippy --workspace --all-targets --locked -- -D warnings",
        "cargo test -p waddle-controlplane --features tonic-transport --locked",
        "cargo test -p waddle-media --features livekit --locked",
        "cargo clippy -p waddle-runtime --features grpc,livekit --all-targets --locked -- -D warnings",
    ):
        assert gate in ci


def test_generated_extension_ignore_uses_the_current_package_name():
    ignore = (REPO / ".gitignore").read_text(encoding="utf-8")
    assert "sdk/python/waddle_sdk/_core*.so" in ignore
    assert "sdk/python/waddle/_core*.so" not in ignore
