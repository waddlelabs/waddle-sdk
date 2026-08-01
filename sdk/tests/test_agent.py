"""The PyO3 shim's connected surface: the `FEATURES` probe, the connected
transport kwargs on `create_session`, and `Session.agent` (flag
`waddle.v0.agent`).

Drives `waddle._core` directly, exactly as `test_reset.py` does: these
tests end at the `_core` module surface. The friendly Python surface built
on top of it (`waddle.agent`, `waddle.init(transport=...)`) is
`test_agent_api.py`'s and `test_features.py`'s.
"""

from __future__ import annotations

import json
import signal
import threading
import time

import pytest

import waddle
import waddle._core as _core


def _robot() -> waddle.Robot:
    return waddle.Robot(
        name="pytest-agent-bot",
        robot_id="py-agent-01",
        cell_id="cell-py-agent",
        action_space=waddle.JointSpace(joints=["j0", "j1", "j2"], rate_hz=50),
    )


def _session(**kwargs) -> _core.Session:
    control = waddle.Control(
        send=lambda chunk: None, hold=lambda: None, resume=lambda: None
    )
    robot = _robot()
    robot_json = json.dumps(
        robot._compile(waddle._derive_grants(control, robot.action_space))
    )
    return _core.create_session(
        project="pytest-agent",
        robot_json=robot_json,
        send=control.send,
        hold=control.hold,
        resume=control.resume,
        **kwargs,
    )


# --- The feature probe -----------------------------------------------------


def test_features_probe_names_only_known_transports():
    """`FEATURES` is the ONE thing Python may branch on to know what this
    build can do; its shape is part of the surface."""
    assert isinstance(_core.FEATURES, frozenset)
    assert _core.FEATURES <= {"grpc", "livekit"}
    assert isinstance(_core.__version__, str) and _core.__version__


# --- Connected kwargs on an offline build ----------------------------------


@pytest.mark.skipif(
    "grpc" in _core.FEATURES, reason="this build carries the grpc transport"
)
def test_transport_url_without_the_feature_refuses_loudly():
    """A supervision session that silently ran offline because a URL was
    ignored is the failure this layer exists to prevent — so an offline
    build refuses the kwarg instead of degrading."""
    with pytest.raises(RuntimeError, match="grpc"):
        _session(transport_url="http://localhost:50051")


@pytest.mark.skipif(
    "livekit" in _core.FEATURES, reason="this build carries the livekit media plane"
)
def test_media_url_without_the_feature_refuses_loudly():
    with pytest.raises(RuntimeError, match="livekit"):
        _session(media_url="wss://example.invalid", media_token="tok")


def test_tokens_without_their_url_are_rejected():
    with pytest.raises(ValueError, match="transport_token"):
        _session(transport_token="tok")
    with pytest.raises(ValueError, match="media_token"):
        _session(media_token="tok")


# --- Session.agent ---------------------------------------------------------


def test_agent_invite_timeout_returns_an_abort_result():
    """Nothing is connected, so nobody can ever engage: the invite deadline
    (FSM.md E25) aborts the episode and `agent()` reports that as the run's
    outcome, not as an error."""
    session = _session()
    try:
        started = time.monotonic()
        result = session.agent("nobody home", 150_000_000)
        elapsed = time.monotonic() - started
        assert result.outcome == "abort"
        assert result.episode_id
        assert result.recording_ref is None
        assert result.detail == ""
        assert "AgentResult(" in repr(result)
        assert elapsed < 5.0, "the invite deadline, not a hang"
    finally:
        session.shutdown()


def test_agent_is_interruptible_and_ends_the_run():
    """Ctrl-C during an agent run must be heard long before the invite
    deadline, AND must leave nothing driving the robot: the shim asks the
    core to abort the live agent-invited episode, then raises.

    Runs on the main thread on purpose — CPython only runs signal handlers
    there, which is exactly the thread `agent()` keeps reattaching to.
    """
    session = _session()
    try:
        threading.Timer(0.2, lambda: signal.raise_signal(signal.SIGINT)).start()
        started = time.monotonic()
        # A 60 s invite deadline: only the interrupt can end this quickly.
        with pytest.raises(KeyboardInterrupt):
            session.agent("wait for a Ctrl-C", 60_000_000_000)
        elapsed = time.monotonic() - started
        assert elapsed < 10.0, "the interrupt was not heard promptly"
        # The run really ended (aborted), rather than being abandoned mid-flight.
        status_ep = session.start_episode("after the interrupt")
        status_ep.terminate("abort")
    finally:
        session.shutdown()
