from __future__ import annotations

import waddle_sdk


def test_transport_credentials_are_redacted_from_representations():
    grpc = waddle_sdk.Grpc("https://api.example.test", token="grpc-secret")
    media = waddle_sdk.LiveKit("wss://media.example.test", token="media-secret")

    assert "grpc-secret" not in repr(grpc)
    assert "media-secret" not in repr(media)
    assert "api.example.test" in repr(grpc)
    assert "media.example.test" in repr(media)
