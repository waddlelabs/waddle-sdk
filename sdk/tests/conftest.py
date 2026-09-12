"""Shared fixtures.

The terminal fixture exists because every test that touches
`waddle_sdk.robots.base`'s console recovery has to obey: **no test may take the
developer's own terminal.** The reader starts only when stdin is a foreground
TTY — which it IS under `pytest -s` — and a thread reading the real one would
sit in `for line in sys.stdin` eating keystrokes for the rest of the run. So a
test that wants a console says so and gets one of its own.
"""

from __future__ import annotations

import queue
import sys

import pytest

from waddle_sdk.robots import base


class Terminal:
    """A terminal the test types at, one line at a time.

    Blocking between lines the way a real one is: a reader of this is a thread
    parked mid-read, which is the state everything about retiring a reader has
    to survive. A `StringIO` cannot show that — its reader reaches
    end-of-input immediately and dies, which is the one case the lifecycle
    handles for free.
    """

    def __init__(self) -> None:
        self._typed: queue.Queue[str | None] = queue.Queue()

    def type(self, line: str) -> None:
        """Type one line, as a site operator would."""
        self._typed.put(line)

    def end(self) -> None:
        """End of input — the reader thread ends here, observably."""
        self._typed.put(None)

    def isatty(self) -> bool:
        return True

    def __iter__(self) -> Terminal:
        return self

    def __next__(self) -> str:
        line = self._typed.get()
        if line is None:
            raise StopIteration
        return line


@pytest.fixture
def terminal(monkeypatch):
    """A foreground terminal of this test's own, as `sys.stdin`.

    The predicate is decided here too (`console_is_at_the_machine`), so no
    test inherits an answer from however the suite happened to be invoked, and
    the input is ended at teardown so the reader thread this test started ends
    with it rather than outliving it.
    """
    stream = Terminal()
    monkeypatch.setattr(base, "console_is_at_the_machine", lambda: True)
    monkeypatch.setattr(sys, "stdin", stream)
    yield stream
    stream.end()


@pytest.fixture(scope="session")
def livekit_configuration(request):
    """Only a selected media test creates a room; collection never requests this."""
    import asyncio
    import json
    import os
    from datetime import timedelta
    from pathlib import Path
    from uuid import uuid4

    from test_livekit_public import _ENV, _configuration
    from waddle_sdk.runtime import RuntimeFault

    assert request.config.getoption("--live")
    if all(os.environ.get(name) for name in _ENV):
        yield _configuration()
        return
    names = (_ENV[0], "WADDLE_TEST_LIVEKIT_API_KEY", "WADDLE_TEST_LIVEKIT_API_SECRET")
    if not all(os.environ.get(name) for name in names):
        pytest.skip(
            "requires scoped grants or explicit LiveKit test signing credentials"
        )
    api = pytest.importorskip("livekit.api")
    url, key, secret = (os.environ[name] for name in names)
    room = "sdk-media-test-" + uuid4().hex
    values = {_ENV[0]: url}
    for role, name in zip(("publisher", "viewer"), _ENV[1:], strict=True):
        values[name] = (
            api.AccessToken(key, secret)
            .with_identity(room + "-" + role)
            .with_ttl(timedelta(hours=2))
            .with_grants(
                api.VideoGrants(
                    room_join=True,
                    room=room,
                    can_publish=role == "publisher",
                    can_subscribe=role == "viewer",
                    can_publish_data=False,
                )
            )
            .to_jwt()
        )
    private_values = (key, secret, values[_ENV[1]], values[_ENV[2]])

    def safe_error(error):
        fault = RuntimeFault.from_exception(error)
        for value in private_values:
            fault.detail = fault.detail.replace(value, "[REDACTED]")
        return fault

    async def room_operation(create):
        client = api.LiveKitAPI(url, key, secret)
        try:
            if create:
                await client.room.create_room(
                    api.CreateRoomRequest(
                        name=room,
                        empty_timeout=7200,
                        departure_timeout=7200,
                    )
                )
                evidence["created"] = True
            else:
                await client.room.delete_room(api.DeleteRoomRequest(room=room))
                evidence["deleted"] = True
        finally:
            await client.aclose()

    directory = Path(
        request.config.live_bench.get("evidence_directory", ".pytest_cache/live")
    )
    directory.mkdir(parents=True, exist_ok=True)
    evidence = {"room": room, "created": False, "deleted": False}
    try:
        try:
            asyncio.run(room_operation(True))
        except Exception as error:  # noqa: BLE001 -- retain service diagnostics without credential text
            fault = safe_error(error)
            evidence["error"] = fault.as_dict()
            raise fault from None
        yield _configuration(values)
    finally:
        try:
            if evidence["created"]:
                asyncio.run(room_operation(False))
        except Exception as error:  # noqa: BLE001 -- report cleanup independently of test failures
            fault = safe_error(error)
            evidence["cleanup_error"] = fault.as_dict()
            raise fault from None
        finally:
            serialized = json.dumps(evidence, indent=2)
            for value in private_values:
                serialized = serialized.replace(value, "[REDACTED]")
            (directory / (room + ".json")).write_text(serialized + "\n")
