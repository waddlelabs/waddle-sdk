"""Commands must produce measured arrival and encoder-derived TCP displacement."""

import pytest

from .sdk_live.metrics import assert_arrived
from .sdk_live.paired import paired, reference
from .sdk_live.session import Bench

pytestmark = pytest.mark.motion


def test_joint_trajectory_reaches_measured_target(request, case):
    config = request.config.live_bench
    if config.get("comparison", {}).get("vendor") == "i2rt":
        result = paired(config, case)
        assert result["verdict"]["passed"], result["verdict"]
        return
    with Bench(config, case["part"]) as bench:
        measured_case(bench, case)


def measured_case(bench, case):
    part = case["part"]
    reference_case = reference(case)
    initial = bench.move(part, case["start_rad"], reference_case)
    assert_arrived(initial, reference_case)
    result = bench.move(part, case["target_rad"], case)
    assert_arrived(result, case)
    returned = bench.move(part, case["start_rad"], reference_case)
    assert_arrived(returned, reference_case)
