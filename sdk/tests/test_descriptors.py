"""Descriptor compilation: canonical proto3 JSON dicts, checked against the
reference outputs and round-tripped through the shim's core-side validator.
Fixtures are compared semantically (defaults omitted), not textually."""

import json

import pytest

import waddle
from waddle import _core


def test_joint_space_reference_output():
    space = waddle.JointSpace(joints=["j0", "j1", "j2"], rate_hz=50)
    assert space._compile_space() == {
        "jointPosition": {"joints": [{"name": "j0"}, {"name": "j1"}, {"name": "j2"}]},
        "rateHz": 50.0,
    }


def test_bimanual_composite_reference_output():
    space = waddle.Composite(
        left=waddle.JointSpace(joints=[f"l{i}" for i in range(7)]),
        right=waddle.JointSpace(joints=[f"r{i}" for i in range(7)]),
        rate_hz=50,
        chunking=waddle.Chunking(horizon=20, replan="IMMEDIATE", interp="linear"),
    )
    compiled = space._compile_space()
    assert compiled["rateHz"] == 50.0
    assert compiled["chunking"] == {
        "horizonSteps": 20,
        "replan": "REPLAN_POLICY_IMMEDIATE",
        "interpolation": "INTERPOLATION_LINEAR",
    }
    parts = compiled["composite"]["parts"]
    # Kwargs insertion order is the normative part order.
    assert [p["name"] for p in parts] == ["left", "right"]
    # Nested part rateHz stays omitted (core validates rate at the top level).
    assert "rateHz" not in parts[0]["space"]
    assert parts[0]["space"]["jointPosition"]["joints"][0] == {"name": "l0"}


def test_compiled_robot_round_trips_through_core_validation():
    robot = waddle.Robot(
        name="bimanual",
        robot_id="bot-1",
        cell_id="cell-a",
        action_space=waddle.Composite(
            left=waddle.JointSpace(joints=[f"l{i}" for i in range(7)]),
            right=waddle.JointSpace(
                joints=[f"r{i}" for i in range(7)],
                gripper=waddle.Gripper.parallel(dim=-1, open=1.0),
            ),
            rate_hz=50,
            chunking=waddle.Chunking(horizon=20, replan="immediate", interp="linear"),
        ),
        cameras={"wrist": waddle.Camera(width=640, height=480, fps=30, encoding="rgb8")},
    )
    control = waddle.Control(
        send=lambda chunk: None,
        hold=lambda: None,
        estop=lambda: None,
        estop_hardware=True,
        estop_latency_bound_ms=15,
    )
    grants = waddle._derive_grants(control, robot.action_space)
    assert {"verb": "VERB_SEND", "sendInterfaces": ["SPACE_KIND_COMPOSITE"]} in grants
    assert {"verb": "VERB_HOLD"} in grants
    estop = next(g for g in grants if g["verb"] == "VERB_ESTOP")
    # int64 crosses canonical proto3 JSON as a decimal string.
    assert estop == {
        "verb": "VERB_ESTOP",
        "hardware": True,
        "declaredLatencyBoundNs": "15000000",
    }
    # The compiled declaration is valid canonical JSON for waddle-core.
    _core.validate_robot_json(json.dumps(robot._compile(grants)))


def test_ee_delta_must_declare_conventions():
    space = waddle.EEDelta(frame_id="base", rotation="rotvec", delta_frame="base")
    assert space._compile_kind() == {
        "eeDelta": {
            "frameId": "base",
            "rotationEncoding": "ROTATION_ENCODING_ROTVEC",
            "deltaFrame": "DELTA_FRAME_BASE",
        }
    }
    with pytest.raises(ValueError, match="rotation"):
        waddle.EEDelta(frame_id="base", rotation="spin", delta_frame="base")._compile_kind()


def test_invalid_declarations_raise():
    with pytest.raises(ValueError, match="radians"):
        waddle.JointSpace(joints=["j0"], units="deg")
    with pytest.raises(ValueError, match="at least one joint"):
        waddle.JointSpace(joints=[])
    with pytest.raises(ValueError, match="at least one part"):
        waddle.Composite(rate_hz=50)
    with pytest.raises(ValueError, match="replan"):
        waddle.Chunking(horizon=1, replan="never")._compile()
    with pytest.raises(ValueError):
        _core.validate_robot_json("{not json}")
    with pytest.raises(ValueError):
        # Parses, but has no action space: rejected by core validation.
        _core.validate_robot_json(json.dumps({"name": "empty"}))


def test_control_rejects_send_dict():
    with pytest.raises(TypeError, match="ONE callable"):
        waddle.Control(send={"joint_position": lambda chunk: None})
