"""SDK-owned two-arm acquisition, streaming, envelopes and measured stop behavior."""

import pytest

from .sdk_live import named_trials
from .sdk_live.named import NamedBench
from .sdk_live.named_config import cases

pytestmark = pytest.mark.named_parts


def test_named_observations_follow_device_feedback(request):
    with NamedBench(request.config.live_bench) as bench:
        bench.require()
        named_trials.observations(bench)


@pytest.mark.motion
@pytest.mark.named_motion
def test_independent_streams_reuse_arrived_arm_with_bounded_latency(request):
    config = request.config.live_bench
    with NamedBench(config) as bench:
        bench.require(motion=True)
        named_trials.parallel(bench, cases(config))


@pytest.mark.motion
@pytest.mark.named_envelope
def test_named_envelope_refuses_target_without_measured_motion(request):
    config = request.config.live_bench
    with NamedBench(config) as bench:
        bench.require(action=True)
        named_trials.envelope(bench, bench.profile["envelope"])


@pytest.mark.motion
def test_explicit_stop_settles_both_moving_arms(request, named_stop):
    config = request.config.live_bench
    with NamedBench(config) as bench:
        bench.require(motion=True, stop=named_stop["method"])
        named_trials.stop(bench, cases(config), named_stop)
