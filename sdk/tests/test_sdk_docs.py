"""Drift guards for new public SDK documentation.

These are deliberately exact where a customer copies code or an install
command. API prose can evolve freely, but package extras, the optional
integration boundary, and the managed-rig examples must move with code.
"""

from __future__ import annotations

import ast
from pathlib import Path

from waddle_sdk import _services

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python 3.10 only
    import tomli as tomllib  # type: ignore[no-redef]


SDK = Path(__file__).resolve().parents[1]
README = (SDK / "README.md").read_text(encoding="utf-8")
EXAMPLES = (SDK / "examples" / "README.md").read_text(encoding="utf-8")


MANAGED_RIG_EXAMPLE = '''rig = yam.bimanual(
    workspace=WORKSPACE_M,
    gripper_limits=GRIPPER_LIMITS_MOTOR_RAD,
    cross_arm=CROSS_ARM,
    sim=True,
)
waddle_sdk.init(
    "waddle-yam-bimanual",
    rig=rig,
    transport=waddle_sdk.Grpc(url, token),
)
try:
    dashboard = waddle_sdk.ui()
    result = waddle_sdk.agent("move each arm to its taught home, one at a time")
finally:
    waddle_sdk.shutdown()'''


def test_examples_publish_the_process_owned_rig_lifecycle_verbatim():
    assert MANAGED_RIG_EXAMPLE in EXAMPLES
    ast.parse(MANAGED_RIG_EXAMPLE)
    assert "Take Local Control" in EXAMPLES
    assert "bounded 3-D measurement" in EXAMPLES


def test_camera_install_commands_are_held_to_package_metadata():
    with (SDK / "pyproject.toml").open("rb") as stream:
        extras = tomllib.load(stream)["project"]["optional-dependencies"]

    assert extras["orbbec"] == ["pyorbbecsdk2"]
    assert extras["realsense"] == ["pyrealsense2"]
    assert set(extras["cameras"]) == set(extras["orbbec"] + extras["realsense"])
    for extra in ("orbbec", "realsense", "cameras"):
        assert f"pip install 'waddle-sdk[{extra}]'" in README
    assert "pip install 'waddle-sdk[cameras,teleop]'" in README


def test_readme_holds_the_public_service_and_integration_boundaries():
    prose = " ".join(README.split())
    assert "waddle_sdk.init(rig=...)" in README
    assert "waddle_sdk.task_session(name, task_session_id=...)" in README
    assert "waddle_sdk.request_workspace_artifact(" in README
    assert "Calibration clicks carry the frame sequence" in prose
    assert "exclusive remote-to-local handoff" in prose
    assert _services._EXECUTION_GROUP == "waddle.execution.v1"
    assert _services._EXECUTION_GROUP in README
