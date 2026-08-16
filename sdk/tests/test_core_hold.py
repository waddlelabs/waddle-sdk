"""The core-owned local hold seam exposed by the PyO3 session."""

from __future__ import annotations

import threading
from pathlib import Path

import pytest

import waddle_sdk._core as _core
from waddle_sdk import descriptors
from waddle_sdk._session import Control, create_core_session


def _session(hold) -> _core.Session:
    control = Control(send=lambda chunk: None, hold=hold)
    robot = descriptors.Robot(
        name="pytest-hold-bot",
        robot_id="py-hold-01",
        cell_id="cell-py-hold",
        action_space=descriptors.JointSpace(joints=["j0"], rate_hz=50),
    )
    return create_core_session("pytest-hold", robot, control)


def test_request_hold_dispatches_off_caller_thread_and_validates_reason():
    called = threading.Event()
    callback_threads: list[int] = []

    def hold() -> None:
        callback_threads.append(threading.get_ident())
        called.set()

    session = _session(hold)
    try:
        caller_thread = threading.get_ident()
        assert session.request_hold("connector disconnected") == "requested"
        assert called.wait(2)
        assert callback_threads == [callback_threads[0]]
        assert callback_threads[0] != caller_thread

        with pytest.raises(RuntimeError, match="reason must be non-empty"):
            session.request_hold(" ")
        with pytest.raises(RuntimeError, match="at most 1024"):
            session.request_hold("x" * 1025)
    finally:
        session.shutdown()


def test_type_stub_declares_request_hold():
    stub = Path(_core.__file__).with_name("_core.pyi").read_text()
    assert "def request_hold(self, reason: str) -> str: ..." in stub
