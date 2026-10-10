from __future__ import annotations

import waddle_sdk


def test_transport_credentials_are_redacted_from_representations():
    grpc = waddle_sdk.Grpc("https://api.example.test", token="grpc-secret")
    media = waddle_sdk.LiveKit("wss://media.example.test", token="media-secret")

    assert "grpc-secret" not in repr(grpc)
    assert "media-secret" not in repr(media)
    assert "api.example.test" in repr(grpc)
    assert "media.example.test" in repr(media)


def test_preview_ceilings_refuse_invalid_values():
    import pytest

    for row in (
        {"preview_width": 0},
        {"preview_width": True},
        {"preview_max_kbps": -1},
        {"preview_fps": float("nan")},
        {"preview_fps": float("inf")},
        {"preview_fps": True},
        {"depth_preview": 0},
        {"demand_driven": "yes"},
    ):
        with pytest.raises(ValueError):
            waddle_sdk.LiveKit("wss://media.example.test", "secret", **row)
