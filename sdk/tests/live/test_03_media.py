"""Real camera RGB/depth through the SDK publisher and configured LiveKit."""

import asyncio
from uuid import uuid4

import pytest
from test_livekit_public import _ENV, _acceptance, _site
from waddle_sdk import load_site

from .sdk_live.metrics import write_report

pytestmark = [pytest.mark.requires_env(*_ENV), pytest.mark.requires_site]


def test_real_camera_publication_and_reconnect(
    request, tmp_path, camera, livekit_configuration
):
    config = request.config.live_bench
    original = load_site(config["site"])
    row = original.describe()["cameras"][camera]
    site = _site(tmp_path / "site.yaml", camera_row=row, site_id=original.id)
    rtc = pytest.importorskip("livekit.rtc")
    pytest.importorskip("websockets.asyncio.server")
    evidence = asyncio.run(_acceptance(tmp_path, rtc, livekit_configuration, site=site))
    write_report(
        config["evidence_directory"] + "/camera-media-" + uuid4().hex + ".json",
        {"camera": camera, "site_id": original.id, **evidence},
    )
