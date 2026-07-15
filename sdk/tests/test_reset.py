"""Task 11 (PyO3 shim): reset kwargs, PyResetHook normalization, and the
`_testing` reset-window hooks.

Drives `waddle._core` directly rather than `waddle.init`/`waddle.rollout`:
the Python-side sugar (`TeleopReset`/`AgentReset`, `rollout(pre_reset=...)`)
is a later task, and this one's own scope statement is "this task ends at
the `_core` module surface" — so these tests exercise exactly that surface.
"""

from __future__ import annotations

import json
import threading
import time

import pytest

import waddle
import waddle._core as _core


def _robot(n_joints: int = 3) -> waddle.Robot:
    return waddle.Robot(
        name="pytest-reset-bot",
        robot_id="py-reset-01",
        cell_id="cell-py-reset",
        action_space=waddle.JointSpace(
            joints=[f"j{i}" for i in range(n_joints)], rate_hz=50
        ),
    )


def _control() -> waddle.Control:
    return waddle.Control(send=lambda chunk: None, hold=lambda: None, resume=lambda: None)


def _session(testing_loopback: bool = False, **reset_kwargs) -> _core.Session:
    """Build a session directly against `_core` (bypassing `waddle.init`,
    which does not thread reset kwargs through yet)."""
    control = _control()
    robot = _robot()
    robot_json = json.dumps(robot._compile(waddle._derive_grants(control, robot.action_space)))
    return _core.create_session(
        project="pytest-reset",
        robot_json=robot_json,
        send=control.send,
        hold=control.hold,
        resume=control.resume,
        testing_loopback=testing_loopback,
        **reset_kwargs,
    )


def _call_with_timeout(fn, timeout: float = 5.0):
    """Run a blocking call on a background thread; fail cleanly instead of
    hanging pytest forever if it never returns (a broken reset path is
    exactly a scenario this suite must be able to report, not freeze on)."""
    box: dict = {}

    def target():
        try:
            box["value"] = fn()
        except BaseException as exc:  # noqa: BLE001 - re-raised on the caller
            box["error"] = exc

    t = threading.Thread(target=target, daemon=True)
    t.start()
    t.join(timeout)
    if t.is_alive():
        pytest.fail(f"operation did not complete within {timeout}s (possible hang)")
    if "error" in box:
        raise box["error"]
    return box.get("value")


def _background(fn) -> threading.Thread:
    t = threading.Thread(target=fn, daemon=True)
    t.start()
    return t


# --- create_session reset kwargs + PyResetHook -----------------------------


def test_pre_reset_hook_runs_and_reaches_ready():
    """Basic happy path: the hook sees the task string, and a bare `True`
    return normalizes to (true, Some(true)) so the default Blocking
    verification mode reaches READY instead of hanging in RESETTING."""
    seen = []

    def hook(task):
        seen.append(task)
        return True

    session = _session(pre_reset_kind="hook", pre_reset_hook=hook)
    try:
        ep = _call_with_timeout(lambda: session.start_episode("stack the blocks"))
        assert seen == ["stack the blocks"]
        assert not ep.done
        assert ep.outcome is None
        ep.terminate("success")
    finally:
        session.shutdown()


def test_pre_reset_hook_failure_raises_reset_failed():
    def hook(task):
        return False

    session = _session(pre_reset_kind="hook", pre_reset_hook=hook)
    try:
        with pytest.raises(RuntimeError, match="reset failed"):
            _call_with_timeout(lambda: session.start_episode("nope"))
    finally:
        session.shutdown()


@pytest.mark.parametrize("verified", [False, None])
def test_pre_reset_hook_tuple_optimistic_reaches_ready(verified):
    """An explicit (True, verified) tuple, under "optimistic" verification,
    always reaches READY regardless of `verified` (OptimisticAsync enters
    READY immediately per FSM.md E3); also proves a `None` second element
    extracts cleanly (no special-casing needed for that shape)."""

    def hook(task):
        return (True, verified)

    session = _session(
        pre_reset_kind="hook", pre_reset_hook=hook, reset_verification="optimistic"
    )
    try:
        ep = _call_with_timeout(lambda: session.start_episode("towel"))
        assert not ep.done
        ep.terminate("success")
    finally:
        session.shutdown()


def test_pre_reset_hook_exception_degrades_to_failure_without_crashing():
    """A raised exception must never unwind into Rust — it normalizes to
    (false, None) (reset fails cleanly) rather than aborting the process."""

    def hook(task):
        raise ValueError("boom")

    session = _session(pre_reset_kind="hook", pre_reset_hook=hook)
    try:
        with pytest.raises(RuntimeError, match="reset failed"):
            _call_with_timeout(lambda: session.start_episode("nope"))
    finally:
        session.shutdown()


def test_pre_reset_hook_invalid_return_degrades_to_failure_without_crashing():
    """A return value that is neither bool nor (bool, Optional[bool]) is
    the "invalid return" case the brief calls out: normalize to
    (false, None), never panic."""

    def hook(task):
        return "not a bool"

    session = _session(pre_reset_kind="hook", pre_reset_hook=hook)
    try:
        with pytest.raises(RuntimeError, match="reset failed"):
            _call_with_timeout(lambda: session.start_episode("nope"))
    finally:
        session.shutdown()


def test_reset_kind_hook_without_callable_raises_value_error():
    with pytest.raises(ValueError, match="pre_reset_hook"):
        _session(pre_reset_kind="hook")


def test_reset_kind_unknown_raises_value_error():
    with pytest.raises(ValueError, match="pre_reset_kind"):
        _session(pre_reset_kind="bogus")


def test_start_episode_override_disables_session_default_hook():
    """`start_episode`'s per-episode kwargs mirror the session ones;
    `pre_reset_kind="none"` disables the session's declared hook for one
    episode only (EpisodeOptions' inherit/disable contract)."""
    calls = []

    def hook(task):
        calls.append(task)
        return True

    session = _session(pre_reset_kind="hook", pre_reset_hook=hook)
    try:
        ep = session.start_episode("uses default")
        assert calls == ["uses default"]
        ep.terminate("success")

        ep2 = session.start_episode("overridden", pre_reset_kind="none")
        assert calls == ["uses default"]  # the hook did not run again
        ep2.terminate("success")
    finally:
        session.shutdown()


# --- post-reset hook + post_reset_failed / done ----------------------------


def test_post_reset_hook_and_post_reset_failed_getter():
    proceed = threading.Event()
    seen = []

    def post_hook(task):
        seen.append(task)
        proceed.wait(timeout=5)
        return False

    session = _session(post_reset_kind="hook", post_reset_hook=post_hook)
    try:
        ep = session.start_episode("towel")
        action = [0.0, 0.0, 0.0]
        ep.gate(action)
        time.sleep(0.05)  # let the reducer process the READY->RUNNING tick

        term_thread = _background(lambda: ep.terminate("success"))

        deadline = time.monotonic() + 5.0
        while not seen and time.monotonic() < deadline:
            time.sleep(0.005)
        assert seen == ["towel"], "post-reset hook never ran"

        # POST_RESET already counts as done; the outcome is pinned, and the
        # post-reset failure hasn't happened yet (the hook is still blocked).
        assert ep.done
        assert ep.outcome == "success"
        assert not ep.post_reset_failed

        proceed.set()
        term_thread.join(timeout=5.0)
        assert not term_thread.is_alive(), "terminate() never unblocked"

        assert ep.post_reset_failed
        assert ep.outcome == "success"  # never altered by the post-reset failure
    finally:
        session.shutdown()


# --- _testing reset-window hooks -------------------------------------------


def test_testing_reset_window_engage_and_complete():
    session = _session(
        post_reset_kind="teleop",
        post_reset_prompt="reset the scene",
        testing_loopback=True,
    )
    try:
        ep = session.start_episode("towel")
        action = [0.0, 0.0, 0.0]
        ep.gate(action)
        time.sleep(0.05)

        term_thread = _background(lambda: ep.terminate("success"))

        deadline = time.monotonic() + 5.0
        while not ep.done and time.monotonic() < deadline:
            time.sleep(0.005)
        assert ep.done, "episode never reached POST_RESET"

        session._testing_reset_window_engage("claim-1", "teleop")

        deadline = time.monotonic() + 5.0
        noop_seen = False
        while time.monotonic() < deadline:
            out = ep.gate(action)
            if out is None and ep.last_gate.kind == "noop":
                noop_seen = True
                break
            time.sleep(0.005)
        assert noop_seen, "gate() never went Noop (RESET_ACTIVE) during the engaged window"

        session._testing_reset_window_complete("claim-1", True)
        term_thread.join(timeout=5.0)
        assert not term_thread.is_alive(), "terminate() never unblocked after window complete"
        assert ep.outcome == "success"
    finally:
        session.shutdown()
