"""Invalid optional camera declarations refuse before signaling or device open."""

import pytest
from waddle_sdk import FEATURES, LiveKit
from waddle_sdk.media import PreviewPublisher


def settings(**overrides):
    return LiveKit(
        "wss://media.invalid",
        "synthetic-test-token",
        preview_width=320,
        preview_fps=1.0,
        preview_max_kbps=128,
        depth_preview=False,
        demand_driven=True,
        video_only=True,
        **overrides,
    )


@pytest.mark.skipif("livekit" not in FEATURES, reason="requires matching media core")
@pytest.mark.parametrize("cameras", [{}, {"scene": (0, 48)}, {"scene": (10000, 10000)}])
def test_invalid_source_grid_refuses_without_signaling(cameras):
    with pytest.raises(ValueError, match="declaration"):
        PreviewPublisher(cameras, settings())


def test_unbounded_publisher_or_missing_media_refuses():
    error = ValueError if "livekit" in FEATURES else RuntimeError
    with pytest.raises(error):
        PreviewPublisher(
            {"scene": (64, 48)}, LiveKit("wss://media.invalid", "synthetic")
        )
