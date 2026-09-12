"""Opt-in real media acceptance using public Site and synthetic camera APIs.

The selected media fixture supplies isolated publisher/viewer grants, using
caller-provided grants or explicitly configured test signing credentials.
"""

from __future__ import annotations

import asyncio
import base64
import json
import logging
import os
from collections import Counter
from contextlib import asynccontextmanager
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit
from uuid import uuid4

import numpy as np
import pytest
from waddle_sdk import LiveKit, load_site

_ENV = (
    "WADDLE_TEST_LIVEKIT_URL",
    "WADDLE_TEST_LIVEKIT_PUBLISHER_TOKEN",
    "WADDLE_TEST_LIVEKIT_VIEWER_TOKEN",
)
pytestmark = [pytest.mark.live, pytest.mark.requires_env(*_ENV)]

_LOG = logging.getLogger("sdk.media.acceptance.websocket")
_LOG.disabled = True  # Signaling headers may contain the scoped token.


def _claims(token):
    payload = token.split(".")[1]
    return json.loads(base64.urlsafe_b64decode(payload + "=" * (-len(payload) % 4)))


def _configuration(values=None):
    values = os.environ if values is None else values
    if not all(values.get(name) for name in _ENV):
        pytest.skip("requires explicit isolated LiveKit publisher/viewer test grants")
    url, publisher, viewer = (values[name] for name in _ENV)
    published, viewed = _claims(publisher), _claims(viewer)
    room = published["video"]["room"]
    assert room.startswith("sdk-media-test-"), "Use a fresh isolated test room"
    assert viewed["video"]["room"] == room
    assert published["sub"] != viewed["sub"]
    assert published["video"]["canPublish"]
    assert not published["video"]["canSubscribe"]
    assert not viewed["video"]["canPublish"]
    assert viewed["video"]["canSubscribe"]
    return url, publisher, viewer, published["sub"]


def _site(path: Path, *, camera_row=None, site_id=None):
    manifest = {
        "api_version": "waddle.site/v1",
        "kind": "Site",
        "metadata": {"id": "media-acceptance-" + uuid4().hex},
        "parts": {
            "arm": {
                "driver": "waddle_sdk.robots.mock:arm",
                "posture": "supervised",
                "base_frame": "arm_base",
                "connection": {},
                "joint_limits": {"joint": [-1.0, 1.0]},
                "options": {
                    "rate_hz": 20.0,
                    "step_caps": [0.05],
                    "home": [0.0],
                    "link_length_m": 0.08,
                    "body_radius_m": 0.01,
                },
            }
        },
        "cameras": {
            "scene": {
                "driver": "waddle_sdk.cameras.mock:MockDriver",
                "connection": {},
                "mount": {"kind": "scene"},
                "frame_id": "scene_optical",
                "stream": {"width": 96, "height": 64, "fps": 10.0},
                "intrinsics": {
                    "fx": 90.0,
                    "fy": 90.0,
                    "cx": 47.5,
                    "cy": 31.5,
                    "depth_scale_mm": 1.0,
                },
                "options": {"has_depth": True, "object_radius_px": 12},
            }
        },
        "frames": {},
        "calibration": {"artifacts": "calibration"},
        "workspace_bounds": {"min": [-0.2, -0.2, -0.02], "max": [0.2, 0.2, 0.02]},
        "envelope": {"static_keepouts": [], "self_collision": {}},
        "recording": {"root": "recordings", "format": "mcap"},
    }
    if camera_row is not None:
        # A camera-only transport projection uses a mock motion context. Camera
        # mounting is irrelevant to transport; never open its physical arm.
        manifest["cameras"]["scene"] = {**camera_row, "mount": {"kind": "scene"}}
        manifest["metadata"]["id"] = site_id
    path.write_text(json.dumps(manifest))
    return load_site(path)


@asynccontextmanager
async def _signaling_proxy(upstream_url):
    """Drop only this test publisher's signaling sockets, never its SDK owner."""
    from websockets.asyncio.client import connect
    from websockets.asyncio.server import serve
    from websockets.exceptions import ConnectionClosed

    connected = asyncio.Queue()
    parsed = urlsplit(upstream_url)
    scheme = {"https": "wss", "http": "ws"}.get(parsed.scheme, parsed.scheme)
    upstream_url = urlunsplit((scheme, parsed.netloc, parsed.path, "", ""))

    async def relay(source, destination):
        async for message in source:
            await destination.send(message)

    async def handler(downstream):
        # The SDK supplies the ordinary LiveKit path/query. This listener and
        # every socket it touches belong solely to the synthetic test publisher.
        target = upstream_url.rstrip("/") + downstream.request.path
        tasks = []
        try:
            async with connect(
                target,
                additional_headers={
                    "Authorization": downstream.request.headers["Authorization"]
                },
                proxy=None,
                compression=None,
                logger=_LOG,
                open_timeout=5,
                max_size=4 * 1024 * 1024,
            ) as upstream:
                await connected.put((downstream, upstream))
                tasks = [
                    asyncio.create_task(relay(downstream, upstream)),
                    asyncio.create_task(relay(upstream, downstream)),
                ]
                done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                for task in done:
                    task.result()
        except ConnectionClosed:
            pass
        except Exception as error:  # noqa: BLE001 -- grant-bearing errors must stay private.
            # Never emit a failing handshake URL/query or raw grant-bearing error.
            await connected.put(type(error).__name__)
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    async with serve(
        handler, "127.0.0.1", 0, logger=_LOG, compression=None, max_size=4 * 1024 * 1024
    ) as server:
        endpoint = f"ws://127.0.0.1:{server.sockets[0].getsockname()[1]}"
        yield endpoint, connected


class _Viewer:
    def __init__(self, rtc, publisher, size=(96, 64)):
        self.rtc, self.publisher = rtc, publisher
        self.size = size
        self.room = rtc.Room()
        self.frames, self.images, self.publications = Counter(), {}, {}
        self.received_sizes, self.publication_sizes = {}, {}
        self.changed = asyncio.Event()
        self.tasks, self.streams = [], []

        @self.room.on("track_subscribed")
        def subscribed(track, publication, participant):
            assert participant.identity == publisher
            assert publication.source == rtc.TrackSource.SOURCE_CAMERA
            self.publications[publication.name] = publication.sid
            self.publication_sizes[publication.name] = (
                publication.width,
                publication.height,
            )
            if publication.simulcasted:
                publication.set_video_quality(rtc.VideoQuality.VIDEO_QUALITY_HIGH)
            stream = rtc.VideoStream(
                track, capacity=2, format=rtc.VideoBufferType.RGB24
            )
            self.streams.append(stream)
            self.tasks.append(
                asyncio.create_task(self.consume(publication.name, stream))
            )

    async def consume(self, name, stream):
        try:
            async for event in stream:
                frame = event.frame
                assert self.publication_sizes[name] == self.size
                # WebRTC can adapt encoded dimensions to available bandwidth.
                # Validate the source dimensions separately from received pixels.
                width, height = self.size
                assert 0 < frame.width <= width and 0 < frame.height <= height
                assert frame.width * height == frame.height * width
                self.received_sizes.setdefault(name, set()).add(
                    (frame.width, frame.height)
                )
                self.images[name] = (
                    np.frombuffer(frame.data, dtype=np.uint8)
                    .reshape(frame.height, frame.width, 3)
                    .copy()
                )
                self.frames[name] += 1
                self.changed.set()
        finally:
            self.changed.set()

    async def received(self, minimum, *, timeout=20):
        async def wait():
            while not all(
                self.frames[name] >= count for name, count in minimum.items()
            ):
                self.changed.clear()
                for task in self.tasks:
                    if task.done():
                        task.result()  # Preserve the actual frame validation error.
                        raise AssertionError(
                            "Video stream ended before enough frames arrived"
                        )
                await self.changed.wait()

        await asyncio.wait_for(wait(), timeout)

    async def close(self):
        await self.room.disconnect()
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        await asyncio.gather(*(stream.aclose() for stream in self.streams))


def test_public_camera_rgb_depth_and_signaling_reconnect_keep_one_sdk_run(
    tmp_path, livekit_configuration
):
    rtc = pytest.importorskip("livekit.rtc")
    pytest.importorskip("websockets.asyncio.server")
    asyncio.run(_acceptance(tmp_path, rtc, livekit_configuration))


async def _acceptance(tmp_path, rtc, config, *, site=None):
    url, publisher_token, viewer_token, publisher = config
    synthetic = site is None
    site = site or _site(tmp_path / "site.yaml")
    stream = site.manifest["cameras"]["scene"]["stream"]
    size = (int(stream["width"]), int(stream["height"]))
    viewer = _Viewer(rtc, publisher, size)
    evidence = {"width": size[0], "height": size[1], "synthetic": synthetic}
    sdk = run = None
    try:
        await viewer.room.connect(
            url, viewer_token, rtc.RoomOptions(connect_timeout=10)
        )
        async with _signaling_proxy(url) as (endpoint, connections):
            unopened = site.open(
                media=LiveKit(endpoint, publisher_token), console=False
            )
            sdk = await asyncio.to_thread(unopened.__enter__)
            run = sdk.run(task="synthetic camera media acceptance", actor="sdk-test")
            await asyncio.to_thread(run.__enter__)
            run_id = run.id
            first = await asyncio.wait_for(connections.get(), 5)
            assert not isinstance(first, str), (
                "Publisher signaling proxy did not connect"
            )
            await viewer.received({"scene": 3, "scene/depth": 3})
            assert set(viewer.publications) == {"scene", "scene/depth"}
            rgb, depth = viewer.images["scene"], viewer.images["scene/depth"]
            if synthetic:
                assert int(rgb[32, 48, 1]) > int(rgb[32, 48, 0]) + 60
            else:
                assert rgb.max() > rgb.min(), "Remote RGB image is flat"
                assert depth.max() > depth.min(), "Remote depth preview is flat"
            assert not np.array_equal(rgb, depth), (
                "Depth must be its paired colorized preview"
            )
            sample = (await asyncio.to_thread(sdk.observe)).cameras["scene"]
            assert sample.depth.dtype == np.uint16
            assert sample.depth.shape == (size[1], size[0])
            if synthetic:
                assert int(sample.depth[32, 48]) == 400
            else:
                assert np.count_nonzero(sample.depth) > sample.depth.size * 0.01
            evidence["initial_frames"] = dict(viewer.frames)
            before = viewer.frames.copy()
            for socket in first:
                socket.transport.abort()
            resumed = await asyncio.wait_for(connections.get(), 15)
            assert not isinstance(resumed, str), "Publisher signaling did not reconnect"
            assert resumed[0] is not first[0]
            await viewer.received({name: count + 3 for name, count in before.items()})
            evidence["after_publisher_reconnect"] = dict(viewer.frames)
            evidence["received_sizes"] = {
                name: sorted(sizes) for name, sizes in viewer.received_sizes.items()
            }
            assert run.id == run_id and not run.done
            assert not any(event.kind == "run.step" for event in sdk.events())
            # A new viewer session discovers the same live publisher and names.
            await viewer.close()
            viewer = _Viewer(rtc, publisher, size)
            await viewer.room.connect(
                url, viewer_token, rtc.RoomOptions(connect_timeout=10)
            )
            await viewer.received({"scene": 3, "scene/depth": 3})
            assert set(viewer.publications) == {"scene", "scene/depth"}
            evidence["viewer_rejoin_frames"] = dict(viewer.frames)
            assert run.id == run_id and not run.done
            await asyncio.to_thread(run.__exit__, None, None, None)
            assert run.done and run.outcome == "abort"
            run = None
            await asyncio.to_thread(unopened.__exit__, None, None, None)
            sdk = None
    finally:
        if run is not None:
            await asyncio.to_thread(run.__exit__, None, None, None)
        if sdk is not None:
            await asyncio.to_thread(sdk.__exit__, None, None, None)
        await viewer.close()
    return evidence
