from __future__ import annotations

import sys
from types import ModuleType

import pytest
from waddle_sdk.robots import SafetyPreset, safety_presets_for_driver


def test_yam_publishes_a_non_opening_tabletop_workspace_preset() -> None:
    report = safety_presets_for_driver(
        "waddle_sdk.robots.yam:arm",
        options={"velocity_feedforward": True},
    )

    assert report.warnings == ()
    assert len(report.presets) == 1
    preset = report.presets[0]
    assert preset.identifier == "yam-tabletop"
    assert dict(preset.workspace_bounds) == {
        "min": (0.05, -0.45, 0.05),
        "max": (0.60, 0.45, 0.70),
    }
    assert preset.static_keepouts == ()
    assert dict(preset.self_collision) == {}
    assert "table" in preset.review
    with pytest.raises(TypeError):
        preset.workspace_bounds["min"] = (0.0, 0.0, 0.0)  # type: ignore[index]


def test_mock_preset_is_derived_from_selected_hardware_options() -> None:
    [preset] = safety_presets_for_driver(
        "waddle_sdk.robots.mock:arm",
        options={"joint_count": 2, "link_length_m": 0.25, "body_radius_m": 0.03},
    ).presets

    assert dict(preset.workspace_bounds) == {
        "min": (-0.53, -0.53, -0.03),
        "max": (0.53, 0.53, 0.03),
    }


def test_custom_robot_module_can_publish_presets_without_an_sdk_registry(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = ModuleType("customer_openarm")
    seen: dict[str, object] = {}

    def presets(*, factory: str, options: object) -> tuple[SafetyPreset, ...]:
        seen.update(factory=factory, options=options)
        return (
            SafetyPreset(
                identifier="openarm-bench",
                label="OpenArm bench",
                workspace_bounds={"min": [-0.4, -0.4, 0.0], "max": [0.4, 0.4, 0.6]},
                review="Measure the actual bench before use.",
            ),
        )

    module.safety_presets = presets  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, module.__name__, module)

    report = safety_presets_for_driver(
        "customer_openarm:bimanual",
        options={"model": "v1"},
    )

    assert [preset.identifier for preset in report.presets] == ["openarm-bench"]
    assert seen["factory"] == "bimanual"
    assert dict(seen["options"]) == {"model": "v1"}  # type: ignore[arg-type]


def test_broken_optional_preset_provider_degrades_to_a_warning(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = ModuleType("broken_robot")

    def presets(*, factory: str, options: object) -> tuple[SafetyPreset, ...]:
        del factory, options
        raise RuntimeError("configuration bug")

    module.safety_presets = presets  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, module.__name__, module)

    report = safety_presets_for_driver("broken_robot:arm")

    assert report.presets == ()
    assert report.warnings == (
        "robot safety presets failed: RuntimeError: configuration bug",
    )


def test_safety_preset_rejects_malformed_bounds() -> None:
    with pytest.raises(ValueError, match="min must not exceed max"):
        SafetyPreset(
            identifier="bad",
            label="Bad",
            workspace_bounds={"min": [1, 0, 0], "max": [0, 1, 1]},
        )
