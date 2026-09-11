"""Commands must produce measured arrival and encoder-derived TCP displacement."""

import pytest

from .sdk_live.metrics import assert_arrived

pytestmark = pytest.mark.motion


def test_joint_trajectory_reaches_measured_target(bench, case):
    part = case["part"]
    reference_case = {
        **case,
        "case_id": case["case_id"] + "-reference",
        "min_displacement_m": 0,
        "target_rad": case["start_rad"],
    }
    reference = bench.move(part, case["start_rad"], reference_case)
    assert_arrived(reference, reference_case)
    result = bench.move(part, case["target_rad"], case)
    assert_arrived(result, case)
    returned = bench.move(part, case["start_rad"], reference_case)
    assert_arrived(returned, reference_case)
