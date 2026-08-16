"""Descriptor compilation: canonical proto3 JSON dicts, checked against the
reference outputs and round-tripped through the shim's core-side validator.
Fixtures are compared semantically (defaults omitted), not textually."""

import base64
import json

import pytest

import waddle_sdk
from waddle_sdk import descriptors
from waddle_sdk._session import Control, _derive_grants, create_core_session
from waddle_sdk import _core


def test_joint_space_reference_output():
    space = descriptors.JointSpace(joints=["j0", "j1", "j2"], rate_hz=50)
    assert space._compile_space() == {
        "jointPosition": {"joints": [{"name": "j0"}, {"name": "j1"}, {"name": "j2"}]},
        "rateHz": 50.0,
    }


def test_bimanual_composite_reference_output():
    space = descriptors.Composite(
        left=descriptors.JointSpace(joints=[f"l{i}" for i in range(7)]),
        right=descriptors.JointSpace(joints=[f"r{i}" for i in range(7)]),
        rate_hz=50,
        chunking=descriptors.Chunking(horizon=20, replan="IMMEDIATE", interp="linear"),
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
    robot = descriptors.Robot(
        name="bimanual",
        robot_id="bot-1",
        cell_id="cell-a",
        action_space=descriptors.Composite(
            left=descriptors.JointSpace(joints=[f"l{i}" for i in range(7)]),
            right=descriptors.JointSpace(
                joints=[f"r{i}" for i in range(7)],
                gripper=descriptors.Gripper.parallel(dim=-1, open=1.0),
            ),
            rate_hz=50,
            chunking=descriptors.Chunking(horizon=20, replan="immediate", interp="linear"),
        ),
        cameras={"wrist": descriptors.Camera(width=640, height=480, fps=30, encoding="rgb8")},
    )
    control = Control(
        send=lambda chunk: None,
        hold=lambda: None,
        estop=lambda: None,
        estop_hardware=True,
        estop_latency_bound_ms=15,
    )
    grants = _derive_grants(control, robot.action_space)
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
    space = descriptors.EEDelta(frame_id="base", rotation="rotvec", delta_frame="base")
    assert space._compile_kind() == {
        "eeDelta": {
            "frameId": "base",
            "rotationEncoding": "ROTATION_ENCODING_ROTVEC",
            "deltaFrame": "DELTA_FRAME_BASE",
        }
    }
    with pytest.raises(ValueError, match="rotation"):
        descriptors.EEDelta(frame_id="base", rotation="spin", delta_frame="base")._compile_kind()


def test_invalid_declarations_raise():
    with pytest.raises(ValueError, match="radians"):
        descriptors.JointSpace(joints=["j0"], units="deg")
    with pytest.raises(ValueError, match="at least one joint"):
        descriptors.JointSpace(joints=[])
    with pytest.raises(ValueError, match="at least one part"):
        descriptors.Composite(rate_hz=50)
    with pytest.raises(ValueError, match="replan"):
        descriptors.Chunking(horizon=1, replan="never")._compile()
    with pytest.raises(ValueError):
        _core.validate_robot_json("{not json}")
    with pytest.raises(ValueError):
        # Parses, but has no action space: rejected by core validation.
        _core.validate_robot_json(json.dumps({"name": "empty"}))


def test_control_rejects_send_dict():
    with pytest.raises(TypeError, match="ONE callable"):
        Control(send={"joint_position": lambda chunk: None})


# ---------------------------------------------------------------------------
# Back-compat golden asserts: descriptors that set none of the new fields
# compile byte-identical (as a dict) to the pre-widening shape.
# ---------------------------------------------------------------------------


def test_camera_without_new_fields_compiles_unchanged():
    cam = descriptors.Camera(width=640, height=480, fps=30, encoding="rgb8")
    assert cam._compile("wrist") == {
        "name": "wrist",
        "width": 640,
        "height": 480,
        "fps": 30.0,
        "encoding": "CAMERA_ENCODING_RGB8",
    }


def test_robot_without_new_fields_compiles_unchanged():
    robot = descriptors.Robot(
        name="bimanual",
        robot_id="bot-1",
        cell_id="cell-a",
        action_space=descriptors.JointSpace(joints=["j0"], rate_hz=50),
    )
    assert robot._compile([]) == {
        "name": "bimanual",
        "actionSpace": {
            "jointPosition": {"joints": [{"name": "j0"}]},
            "rateHz": 50.0,
        },
        "robotId": "bot-1",
        "cellId": "cell-a",
    }


# ---------------------------------------------------------------------------
# Rich descriptors: intrinsics, StreamPolicy, URDF, FrameGraph, joint limits.
# ---------------------------------------------------------------------------


def test_intrinsics_compiled_json_keys():
    intr = descriptors.Intrinsics(
        fx=605.1,
        fy=605.2,
        cx=320.0,
        cy=240.0,
        distortion_model="plumb_bob",
        distortion=(0.1, -0.2, 0.001, 0.002, 0.0),
        depth_scale_mm=0.1,
    )
    assert intr._compile() == {
        "fx": 605.1,
        "fy": 605.2,
        "cx": 320.0,
        "cy": 240.0,
        "model": "DISTORTION_MODEL_PLUMB_BOB",
        "distortion": [0.1, -0.2, 0.001, 0.002, 0.0],
        "depthScaleMm": 0.1,
    }


def test_intrinsics_omits_default_distortion_model_and_empty_fields():
    intr = descriptors.Intrinsics(fx=1.0, fy=1.0, cx=1.0, cy=1.0)
    compiled = intr._compile()
    assert "model" not in compiled
    assert "distortion" not in compiled
    assert "depthScaleMm" not in compiled


def test_intrinsics_validates_positive_depth_scale():
    with pytest.raises(ValueError, match="depth_scale_mm"):
        descriptors.Intrinsics(fx=1.0, fy=1.0, cx=1.0, cy=1.0, depth_scale_mm=-1.0)


def test_camera_stream_policy_and_vendor_compiled_json_keys():
    cam = descriptors.Camera(
        width=1280,
        height=720,
        fps=30,
        stream_policy=descriptors.StreamPolicy(
            local_full_rate=True,
            uplink=descriptors.Uplink(fps=15, encoding="h264", max_kbps=2000),
        ),
        vendor={"serial": "SN-1"},
    )
    compiled = cam._compile("overhead")
    assert compiled["stream"] == {
        "localFullRate": True,
        "uplink": {
            "fps": 15.0,
            "encoding": "CAMERA_ENCODING_H264",
            "maxKbps": 2000,
        },
    }
    assert compiled["vendor"] == {"serial": "SN-1"}


def test_uplink_validates_positive_fps_and_kbps():
    with pytest.raises(ValueError, match="fps"):
        descriptors.Uplink(fps=-1.0, encoding="h264")
    with pytest.raises(ValueError, match="max_kbps"):
        descriptors.Uplink(fps=15.0, encoding="h264", max_kbps=0)


def test_stream_policy_still_fps_compiles_and_is_absent_by_default():
    # The control-plane stills declaration (flag `waddle.v0.obs.stills`):
    # a distinct key from the media plane's `uplink.fps`, and absent
    # entirely unless declared — an undeclared camera must not start
    # putting pictures on the control plane.
    assert descriptors.StreamPolicy(local_full_rate=True)._compile() == {"localFullRate": True}
    assert descriptors.StreamPolicy(still_fps=2)._compile() == {"stillFps": 2.0}
    both = descriptors.StreamPolicy(
        still_fps=2, uplink=descriptors.Uplink(fps=15, encoding="rgb8")
    )._compile()
    assert both == {
        "stillFps": 2.0,
        "uplink": {"fps": 15.0, "encoding": "CAMERA_ENCODING_RGB8"},
    }
    with pytest.raises(ValueError, match="still_fps"):
        descriptors.StreamPolicy(still_fps=-1.0)


def test_still_fps_survives_the_round_trip_into_core():
    # The compiled key has to be a field core actually knows, and only a
    # ROUND TRIP can say so: decoding tolerates unknown fields on purpose
    # (append-only evolution), so a misspelled key validates perfectly and
    # is dropped in silence — the declaration would then be honored by
    # nobody, and the first symptom would be a customer's connected session
    # sending no stills. `robot_json_roundtrip` hands back core's own
    # canonical JSON of what it understood.
    robot = descriptors.Robot(
        name="stills-bot",
        action_space=descriptors.JointSpace(joints=["j0"], rate_hz=20),
        cameras={
            "wrist": descriptors.Camera(
                width=320,
                height=240,
                fps=20,
                stream_policy=descriptors.StreamPolicy(still_fps=2.0),
            )
        },
    )
    compiled = robot._compile([])
    _core.validate_robot_json(json.dumps(compiled))
    decoded = json.loads(_core.robot_json_roundtrip(json.dumps(compiled)))
    # The declared rate, read back out of the field it landed in.
    assert decoded["cameras"][0]["stream"]["stillFps"] == 2.0

    # And the check has teeth: the spelling this test exists to catch
    # disappears on the way in, which is exactly why "it validated" proves
    # nothing.
    misspelled = json.loads(json.dumps(compiled))
    misspelled["cameras"][0]["stream"] = {"stillfps": 2.0}
    _core.validate_robot_json(json.dumps(misspelled))  # still valid!
    survivors = json.loads(_core.robot_json_roundtrip(json.dumps(misspelled)))
    assert "stillFps" not in survivors["cameras"][0].get("stream", {})


def test_frame_transform_pins_wxyz_order():
    # A quarter-turn about y: distinct w/x/y/z values so a transposition
    # (e.g. xyzw written into wxyz slots) is caught, not just a symmetric
    # identity quaternion.
    ft = descriptors.FrameTransform(
        parent="base_link",
        child="cam_overhead",
        position=(0.1, 0.2, 0.3),
        quaternion=(0.9238795, 0.0, 0.3826834, 0.0),
    )
    assert ft._compile() == {
        "parent": "base_link",
        "child": "cam_overhead",
        "transform": {
            "position": {"x": 0.1, "y": 0.2, "z": 0.3},
            "rotation": {"w": 0.9238795, "x": 0.0, "y": 0.3826834, "z": 0.0},
            "frameId": "base_link",
        },
    }


def test_frame_transform_shape_validation():
    with pytest.raises(ValueError, match="parent"):
        descriptors.FrameTransform(parent="", child="cam")
    with pytest.raises(ValueError, match="position"):
        descriptors.FrameTransform(parent="base", child="cam", position=(1.0, 2.0))
    with pytest.raises(ValueError, match="quaternion"):
        descriptors.FrameTransform(parent="base", child="cam", quaternion=(1.0, 0.0, 0.0))


def test_robot_frames_compile_to_frame_graph():
    robot = descriptors.Robot(
        name="arm",
        action_space=descriptors.JointSpace(joints=["j0"], rate_hz=50),
        frames=(
            descriptors.FrameTransform(parent="base_link", child="cam_overhead"),
            descriptors.FrameTransform(parent="base_link", child="cam_wrist"),
        ),
    )
    compiled = robot._compile([])
    assert compiled["frames"] == {
        "transforms": [
            {
                "parent": "base_link",
                "child": "cam_overhead",
                "transform": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                    "frameId": "base_link",
                },
            },
            {
                "parent": "base_link",
                "child": "cam_wrist",
                "transform": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                    "frameId": "base_link",
                },
            },
        ]
    }


def test_joint_limits_compile_and_names_only_form_still_works():
    space = descriptors.JointSpace(
        joints=[
            "j0",  # names-only form, unchanged
            descriptors.Joint(name="j1", min_position=-1.5, max_position=1.5, max_velocity=2.0,
                         max_effort=10.0),
        ],
        rate_hz=50,
    )
    assert space._compile_kind() == {
        "jointPosition": {
            "joints": [
                {"name": "j0"},
                {
                    "name": "j1",
                    "minPosition": -1.5,
                    "maxPosition": 1.5,
                    "maxVelocity": 2.0,
                    "maxEffort": 10.0,
                },
            ]
        }
    }


def test_joint_limits_validate_min_le_max_and_nonnegative_ceilings():
    with pytest.raises(ValueError, match="min_position"):
        descriptors.Joint(name="j0", min_position=1.0, max_position=0.5)
    with pytest.raises(ValueError, match="max_velocity"):
        descriptors.Joint(name="j0", max_velocity=-1.0)
    with pytest.raises(ValueError, match="max_effort"):
        descriptors.Joint(name="j0", max_effort=-1.0)


def test_gripper_dexterous_compiles_joints():
    gripper = descriptors.Gripper.dexterous(["f0", descriptors.Joint(name="f1", max_effort=3.0)])
    assert gripper._compile() == {
        "dexterous": {
            "joints": [
                {"name": "f0"},
                {"name": "f1", "maxEffort": 3.0},
            ]
        }
    }
    with pytest.raises(ValueError, match="at least one joint"):
        descriptors.Gripper.dexterous([])


def test_robot_kinematics_urdf_bytes_passthrough():
    robot = descriptors.Robot(
        name="arm",
        action_space=descriptors.JointSpace(joints=["j0"], rate_hz=50),
        kinematics_urdf=b"<robot name='arm'/>",
    )
    compiled = robot._compile([])
    assert compiled["kinematicsUrdf"] == base64.b64encode(b"<robot name='arm'/>").decode("ascii")


def test_robot_kinematics_urdf_path_is_read_at_compile_time(tmp_path):
    urdf_path = tmp_path / "arm.urdf"
    urdf_path.write_bytes(b"<robot name='arm-from-file'/>")
    robot = descriptors.Robot(
        name="arm",
        action_space=descriptors.JointSpace(joints=["j0"], rate_hz=50),
        kinematics_urdf=urdf_path,
    )
    compiled = robot._compile([])
    assert compiled["kinematicsUrdf"] == base64.b64encode(
        b"<robot name='arm-from-file'/>"
    ).decode("ascii")
    # str path works identically to a Path.
    robot_str = descriptors.Robot(
        name="arm",
        action_space=descriptors.JointSpace(joints=["j0"], rate_hz=50),
        kinematics_urdf=str(urdf_path),
    )
    assert robot_str._compile([])["kinematicsUrdf"] == compiled["kinematicsUrdf"]


def test_robot_series_compiles_time_series_description():
    robot = descriptors.Robot(
        name="arm",
        action_space=descriptors.JointSpace(joints=["j0"], rate_hz=50),
        series={
            "/robot/ft_wrist": descriptors.TimeSeries(
                dtype="f64", shape=(6,), units="N,Nm", rate_hz=200.0
            ),
        },
    )
    compiled = robot._compile([])
    assert compiled["series"] == [
        {
            "name": "/robot/ft_wrist",
            "dtype": "DTYPE_F64",
            "shape": [6],
            "units": "N,Nm",
            "rateHz": 200.0,
        }
    ]


def test_full_robot_round_trips_through_session_creation(tmp_path):
    """URDF bytes + frames + intrinsics + stream policy + joint limits +
    dexterous gripper + series, all together, must build a real core
    session without error (session teardown so other tests aren't
    affected)."""
    robot = descriptors.Robot(
        name="full-widened-bot",
        robot_id="bot-full",
        cell_id="cell-full",
        action_space=descriptors.Composite(
            left=descriptors.JointSpace(
                joints=[
                    descriptors.Joint(name="l0", min_position=-3.0, max_position=3.0,
                                 max_velocity=2.0, max_effort=50.0),
                    "l1",
                ],
                gripper=descriptors.Gripper.dexterous(
                    ["f0", descriptors.Joint(name="f1", max_effort=1.0)]
                ),
            ),
            rate_hz=50,
        ),
        cameras={
            "overhead": descriptors.Camera(
                width=1280,
                height=720,
                fps=30,
                frame_id="cam_overhead",
                intrinsics=descriptors.Intrinsics(
                    fx=605.0, fy=605.0, cx=320.0, cy=240.0, depth_scale_mm=0.1
                ),
                stream_policy=descriptors.StreamPolicy(
                    local_full_rate=True,
                    uplink=descriptors.Uplink(fps=15, encoding="h264", max_kbps=2000),
                ),
                vendor={"serial": "SN-1"},
            ),
        },
        kinematics_urdf=b"<robot name='full-widened-bot'/>",
        frames=(descriptors.FrameTransform(parent="base_link", child="cam_overhead"),),
        series={"/robot/ft_wrist": descriptors.TimeSeries(shape=(6,), rate_hz=200.0)},
    )
    control = Control(send=lambda chunk: None, hold=lambda: None, resume=lambda: None)
    try:
        session = create_core_session(
            "py-descriptor-widening", robot, control, recording_dir=tmp_path
        )
    finally:
        session.shutdown()
