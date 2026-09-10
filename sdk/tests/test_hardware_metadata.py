"""Driver-neutral metadata: arbitrary topology and scoped optional support."""

import math

import pytest
from waddle_sdk.cameras.metadata import (
    CameraMetadataError,
    camera_declarations,
    camera_intrinsics,
)
from waddle_sdk.robots.metadata import gripper_mapping, part_action_spaces


def mapping(**changes):
    return {
        "joint": "jaw",
        "closed_m": 0.01,
        "open_m": 0.09,
        "closed_action": 2,
        "open_action": -2,
        **changes,
    }


def joints():
    return [{"name": "axis"}, {"name": "jaw", "minPosition": -2, "maxPosition": 2}]


def test_reversed_mapping_roundtrip_and_unclamped_measurement():
    physical = gripper_mapping(mapping(), joints())
    assert physical is not None
    for opening in (0.01, 0.03, 0.05, 0.09):
        assert physical.opening(physical.action(opening)) == pytest.approx(opening)
    assert physical.metres_per_action == pytest.approx(-0.02)
    assert physical.opening(3) < physical.closed_m
    for invalid in (True, math.nan, math.inf, 0, 0.1):
        with pytest.raises(ValueError):
            physical.action(invalid)


@pytest.mark.parametrize(
    "changes",
    [
        {"joint": "absent"},
        {"open_m": 0.01},
        {"closed_action": -2},
        {"open_action": 3},
        {"closed_m": True},
        {"open_m": math.inf},
        {"open_m": 10**400},
        {"closed_action": -1e308, "open_action": 1e308},
    ],
)
def test_bad_mapping_is_optional(changes):
    assert gripper_mapping(mapping(**changes), joints()) is None
    assert gripper_mapping(mapping(), joints()) is not None


def test_ambiguous_joint_and_bad_limits_are_unavailable():
    assert gripper_mapping(mapping(), joints() + [joints()[1]]) is None
    assert (
        gripper_mapping(
            mapping(), [{"name": "jaw", "minPosition": False, "maxPosition": 2}]
        )
        is None
    )
    assert gripper_mapping(None, joints()) is None


@pytest.mark.parametrize("width", [1, 5, 9])
def test_action_spaces_preserve_names_and_width(width):
    space = {"jointPosition": {"joints": [{"name": f"axis_{i}"} for i in range(width)]}}
    assert part_action_spaces(
        {"parts": {"custom": {}}, "robot": {"actionSpace": space}}
    ) == {"custom": space}
    assert (
        part_action_spaces(
            {"parts": {"a": {}, "b": {}}, "robot": {"actionSpace": space}}
        )
        == {}
    )
    description = {
        "robot": {
            "actionSpace": {
                "composite": {
                    "parts": [
                        {"name": "custom", "space": space},
                        {"name": "sibling", "space": {"customAction": {}}},
                    ]
                }
            }
        }
    }
    assert part_action_spaces(description) == {
        "custom": space,
        "sibling": {"customAction": {}},
    }


def intrinsics(**changes):
    return {"fx": 400, "fy": 401, "cx": 12, "cy": 13, **changes}


def test_effective_camera_metadata_preserves_mount_and_distortion_identity():
    configured = intrinsics(depth_scale_mm=1)
    live = intrinsics(
        depthScaleMm=0.25, model="DISTORTION_MODEL_PLUMB_BOB", distortion=[0.1]
    )
    description = {
        "cameras": {
            "scene": {
                "frame_id": "old",
                "intrinsics": configured,
                "mount": {"kind": "scene"},
                "stream": {"width": 100},
            },
            "rgb": {"frame_id": "rgb_frame", "intrinsics": intrinsics()},
            "missing": {"frame_id": "unconfigured"},
        },
        "robot": {
            "cameras": [
                {
                    "name": "scene",
                    "frameId": "effective",
                    "width": 200,
                    "intrinsics": live,
                }
            ]
        },
    }
    rows = camera_declarations(description)
    assert rows["scene"]["frame_id"] == "effective"
    assert rows["scene"]["width"] == 200
    assert rows["scene"]["mount"] == {"kind": "scene"}
    result = camera_intrinsics(rows["scene"]["intrinsics"], require_depth_scale=True)
    assert result.depth_scale_mm == 0.25
    assert result.distortion == (0.1,)
    assert result.distortion_model == "DISTORTION_MODEL_PLUMB_BOB"
    assert camera_intrinsics(rows["rgb"]["intrinsics"]).depth_scale_mm is None
    with pytest.raises(CameraMetadataError) as missing:
        camera_intrinsics(rows["missing"]["intrinsics"])
    assert missing.value.code == "intrinsics_missing"
    assert description["cameras"]["scene"]["intrinsics"] == configured


@pytest.mark.parametrize(
    "changes",
    [
        {"fx": True},
        {"fx": 10**400},
        {"fy": 0},
        {"cx": math.nan},
        {"depth_scale_mm": -1},
        {"distortion": [math.nan]},
        {"distortion": [False]},
        {"distortion": [0] * 17},
        {"model": "unknown"},
    ],
)
def test_bad_camera_intrinsics_do_not_disable_healthy_camera(changes):
    rows = camera_declarations(
        {
            "cameras": {
                "bad": {"intrinsics": intrinsics(**changes)},
                "good": {"intrinsics": intrinsics(depth_scale_mm=1)},
            }
        }
    )
    with pytest.raises(CameraMetadataError) as error:
        camera_intrinsics(rows["bad"]["intrinsics"])
    assert error.value.code == "intrinsics_invalid"
    assert (
        camera_intrinsics(rows["good"]["intrinsics"], require_depth_scale=True).fx
        == 400
    )


def test_depth_scale_is_never_invented():
    with pytest.raises(CameraMetadataError) as error:
        camera_intrinsics(intrinsics(), require_depth_scale=True)
    assert error.value.code == "intrinsics_invalid"
    assert camera_intrinsics(intrinsics()).depth_scale_mm is None


def test_metadata_imports_need_no_optional_driver_or_simulator():
    import subprocess
    import sys

    source = """
import importlib.abc
import sys
class Block(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'i2rt', 'mujoco', 'pyrealsense2', 'cv2'}:
            raise AssertionError('Optional dependency imported: ' + fullname)
sys.meta_path.insert(0, Block())
from waddle_sdk.robots.metadata import part_action_spaces
from waddle_sdk.cameras.metadata import camera_declarations
from waddle_sdk.robots import yam
assert part_action_spaces({}) == {}
assert camera_declarations({}) == {}
assert 'waddle_sdk.robots.yam_model' not in sys.modules
"""
    subprocess.run([sys.executable, "-c", source], check=True, timeout=15)
