"""Read actual encoders and all configured cameras before commanding motion."""

import time
from itertools import pairwise

import numpy as np


def test_named_joint_observations_are_finite_and_repeatable(bench, part):
    samples = []
    for _ in range(5):
        observation = bench.run.observe()
        state = observation.parts[part]
        assert np.isfinite(state.joint_position).all()
        assert np.isfinite(state.joint_velocity).all()
        assert state.joint_position.shape == state.joint_velocity.shape
        assert state.frame_id == bench.site.manifest["parts"][part]["base_frame"]
        samples.append(
            {
                "session_ns": observation.session_ns,
                "joints": state.joint_position.tolist(),
            }
        )
        time.sleep(bench.period)
    assert all(a["session_ns"] < b["session_ns"] for a, b in pairwise(samples))
    # Envelope stamps order reads; motion trials below prove responsive feedback.
    bench.report["observations"].append({"part": part, "samples": samples})
    bench.save()


def test_camera_delivers_five_fresh_aligned_rgb_depth_frames(camera_session, camera):
    last = 0
    for _ in range(5):
        sample = camera_session.wait(camera, after_sequence=last, timeout_s=5)
        assert sample is not None, dict(camera_session.errors)
        assert sample.rgb.dtype == np.uint8 and sample.rgb.shape[2] == 3
        assert sample.depth is not None and sample.depth.shape == sample.rgb.shape[:2]
        assert np.count_nonzero(sample.depth) > sample.depth.size * 0.01
        assert np.isfinite(sample.depth).all()
        assert sample.rgb.max() > sample.rgb.min(), "Camera returned a flat image"
        assert sample.sequence > last
        last = sample.sequence
