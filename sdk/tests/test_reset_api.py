"""The `waddle.init`/`waddle.rollout`-level reset API —
`TeleopReset`/`AgentReset`, the `pre_reset`/`post_reset` kwargs, and its
e2e pytest suite. Complements `test_reset.py` (which
drives `waddle._core` directly) one layer up: everything here goes through
the public `waddle.init`/`waddle.rollout` surface.
"""

from __future__ import annotations

import dataclasses
import json
import sys
import threading
import time

import pytest
from mcap.reader import make_reader

import waddle
import waddle._testing


@pytest.fixture(autouse=True)
def _clean_session():
    yield
    waddle.shutdown()


def _robot(n_joints: int = 3) -> waddle.Robot:
    return waddle.Robot(
        name="pytest-reset-api-bot",
        robot_id="py-reset-api-01",
        cell_id="cell-py-reset-api",
        action_space=waddle.JointSpace(
            joints=[f"j{i}" for i in range(n_joints)], rate_hz=50
        ),
    )


def _control() -> waddle.Control:
    return waddle.Control(send=lambda chunk: None, hold=lambda: None, resume=lambda: None)


def _sidecar(tmp_path, episode_id: str) -> dict:
    return json.loads((tmp_path / f"{episode_id}.sidecar.json").read_text())


def _events(sidecar: dict) -> list[dict]:
    return sidecar.get("events", [])


def _topic_counts(mcap_path) -> dict[str, int]:
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        counts: dict[str, int] = {}
        for _, channel, _ in reader.iter_messages():
            counts[channel.topic] = counts.get(channel.topic, 0) + 1
    return counts


# --- TeleopReset / AgentReset ------------------------------------------------


def test_teleop_reset_and_agent_reset_are_frozen_and_repr_friendly():
    t = waddle.TeleopReset("reset the scene")
    assert t.prompt == "reset the scene"
    assert t.timeout_s == 600.0
    assert repr(t) == "TeleopReset(prompt='reset the scene', timeout_s=600.0)"
    with pytest.raises(dataclasses.FrozenInstanceError):
        t.prompt = "nope"  # type: ignore[misc]

    a = waddle.AgentReset("clear the table", timeout_s=30.0)
    assert a.prompt == "clear the table"
    assert a.timeout_s == 30.0
    assert repr(a) == "AgentReset(prompt='clear the table', timeout_s=30.0)"

    # timeout_s is keyword-only.
    with pytest.raises(TypeError):
        waddle.TeleopReset("go", 30.0)  # type: ignore[misc]


@pytest.mark.parametrize("cls", [waddle.TeleopReset, waddle.AgentReset])
def test_teleop_and_agent_reset_reject_bad_values(cls):
    with pytest.raises(TypeError, match="prompt"):
        cls(123)  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="timeout_s"):
        cls("go", timeout_s="nope")  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="timeout_s"):
        cls("go", timeout_s=0)
    with pytest.raises(ValueError, match="timeout_s"):
        cls("go", timeout_s=-1.0)


# --- marker -> FFI kwarg mapping (unit-level, no session needed) -----------


def test_reset_spec_kwargs_maps_markers_to_ffi_kwargs():
    assert waddle._reset_spec_kwargs("pre_reset", None) == {"pre_reset_kind": "none"}

    kwargs = waddle._reset_spec_kwargs("pre_reset", waddle.TeleopReset("go", timeout_s=1.5))
    assert kwargs == {
        "pre_reset_kind": "teleop",
        "pre_reset_prompt": "go",
        "pre_reset_timeout_ns": 1_500_000_000,
    }

    kwargs = waddle._reset_spec_kwargs("post_reset", waddle.AgentReset("go2"))
    assert kwargs == {
        "post_reset_kind": "agent",
        "post_reset_prompt": "go2",
        "post_reset_timeout_ns": 600_000_000_000,
    }

    hook_kwargs = waddle._reset_spec_kwargs("pre_reset", lambda task: True)
    assert hook_kwargs["pre_reset_kind"] == "hook"
    assert callable(hook_kwargs["pre_reset_hook"])

    with pytest.raises(TypeError, match="pre_reset"):
        waddle._reset_spec_kwargs("pre_reset", 123)


def test_reset_override_kwargs_unset_means_inherit():
    assert waddle._reset_override_kwargs("pre_reset", waddle._UNSET) == {}
    assert waddle._reset_override_kwargs("pre_reset", None) == {"pre_reset_kind": "none"}


def test_normalize_reset_hook_passes_through_valid_shapes():
    assert waddle._normalize_reset_hook(lambda task: True)("t") == (True, True)
    assert waddle._normalize_reset_hook(lambda task: False)("t") == (False, False)
    assert waddle._normalize_reset_hook(lambda task: (True, None))("t") == (True, None)
    assert waddle._normalize_reset_hook(lambda task: (False, True))("t") == (False, True)


def test_normalize_reset_hook_raises_typeerror_naming_the_contract():
    wrapped = waddle._normalize_reset_hook(lambda task: "not a bool")
    with pytest.raises(TypeError, match=r"bool.*Optional\[bool\]"):
        wrapped("task")


def test_init_rejects_bad_reset_marker():
    with pytest.raises(TypeError, match="pre_reset"):
        waddle.init("py-bad-reset", _robot(), _control(), pre_reset=123)  # type: ignore[arg-type]


# --- scripted pre+post reset happy path -------------------------------------


def test_scripted_pre_and_post_reset_happy_path(tmp_path):
    calls: list[tuple[str, str]] = []

    def pre_hook(task):
        calls.append(("pre", task))
        return True

    def post_hook(task):
        calls.append(("post", task))
        return True

    waddle.init(
        "py-reset-happy",
        _robot(),
        _control(),
        recording_dir=tmp_path,
        pre_reset=pre_hook,
        post_reset=post_hook,
    )
    with waddle.rollout(task="fold the towel") as ep:
        episode_id = ep.id
        ep.gate([0.0, 0.0, 0.0])
        ep.terminate("success")
    assert ep.done
    assert ep.outcome == waddle.Outcome.SUCCESS
    assert not ep.post_reset_failed
    # Order proves the pre-reset ran (blocking, before the loop body) before
    # the post-reset (after `terminate()`, which itself blocks through it).
    assert calls == [("pre", "fold the towel"), ("post", "fold the towel")]
    waddle.shutdown()

    sidecar = _sidecar(tmp_path, episode_id)
    assert sidecar["outcome"] == "TERMINAL_OUTCOME_SUCCESS"
    assert sidecar["postResetDeclared"] is True
    assert sidecar.get("postResetFailed", False) is False
    assert sidecar["postResetResult"]["ok"] is True
    bounds = sidecar["postResetBounds"]
    assert int(bounds["tEndNs"]) > int(bounds["tStartNs"])

    counts = _topic_counts(tmp_path / f"{episode_id}.mcap")
    assert counts.get("/waddle/events", 0) >= 1


# --- per-rollout override ----------------------------------------------------


def test_rollout_override_disables_or_replaces_session_reset(tmp_path):
    pre_calls: list[tuple[str, str]] = []
    post_calls: list[tuple[str, str]] = []

    def default_pre(task):
        pre_calls.append(("default_pre", task))
        return True

    def default_post(task):
        post_calls.append(("default_post", task))
        return True

    def override_post(task):
        post_calls.append(("override_post", task))
        return True

    waddle.init(
        "py-reset-override",
        _robot(),
        _control(),
        recording_dir=tmp_path,
        pre_reset=default_pre,
        post_reset=default_post,
    )

    # Episode 1: post disabled for this episode only (None = disable, not
    # inherit — inheriting is `_UNSET`, the default).
    with waddle.rollout(task="ep1", post_reset=None) as ep1:
        ep1.gate([0.0, 0.0, 0.0])
        ep1.terminate("success")
    assert pre_calls == [("default_pre", "ep1")]
    assert post_calls == []
    assert not ep1.post_reset_failed

    # Episode 2: pre_reset inherited (unset); post_reset overridden to a
    # fresh hook, so the session default post hook never runs for it.
    with waddle.rollout(task="ep2", post_reset=override_post) as ep2:
        ep2.gate([0.0, 0.0, 0.0])
        ep2.terminate("success")
    assert pre_calls == [("default_pre", "ep1"), ("default_pre", "ep2")]
    assert post_calls == [("override_post", "ep2")]


# --- failing hooks ------------------------------------------------------------


def test_failing_pre_reset_hook_raises(tmp_path):
    waddle.init(
        "py-reset-pre-fail",
        _robot(),
        _control(),
        recording_dir=tmp_path,
        pre_reset=lambda task: False,
    )
    with pytest.raises(RuntimeError, match="reset failed"):
        waddle.rollout(task="nope")


def test_malformed_pre_reset_hook_return_is_indistinguishable_from_a_legitimate_failure(tmp_path):
    """Regression for a corrected claim in `_normalize_reset_hook`'s
    docstring/CHANGELOG: a hook whose return value violates the
    `bool | (bool, Optional[bool])` contract does NOT raise a
    distinguishable, actionable error to the `rollout()` caller.
    `PyResetHook::call` (`sdk/rust/src/verbs.rs`) catches every exception
    the Python callable raises -- including `_normalize_reset_hook`'s own
    `TypeError` -- and reports it only via `sys.unraisablehook` before
    normalizing to `(False, None)`, the exact same outcome a hook that
    legitimately returns `False` produces. This test proves both the
    diagnostic-only delivery (captured via a chained `sys.unraisablehook`)
    and the caller-visible indistinguishability (byte-identical
    `RuntimeError` text), through the public `waddle.init`/`waddle.rollout`
    API rather than the private `_normalize_reset_hook` helper alone."""
    captured_unraisable = []
    previous_hook = sys.unraisablehook

    def _chained_hook(unraisable):
        captured_unraisable.append(unraisable)
        previous_hook(unraisable)

    sys.unraisablehook = _chained_hook
    try:
        waddle.init(
            "py-reset-pre-malformed",
            _robot(),
            _control(),
            recording_dir=tmp_path,
            pre_reset=lambda task: "not a bool",
        )
        with pytest.raises(RuntimeError, match="reset failed") as malformed_exc:
            waddle.rollout(task="nope")
    finally:
        sys.unraisablehook = previous_hook
    waddle.shutdown()

    # The TypeError naming the contract really was raised -- but only
    # ever delivered to the unraisable-hook diagnostic channel.
    assert len(captured_unraisable) == 1
    assert captured_unraisable[0].exc_type is TypeError
    assert "Optional[bool]" in str(captured_unraisable[0].exc_value)

    waddle.init(
        "py-reset-pre-legit-false",
        _robot(),
        _control(),
        recording_dir=tmp_path,
        pre_reset=lambda task: False,
    )
    with pytest.raises(RuntimeError, match="reset failed") as legit_exc:
        waddle.rollout(task="nope")

    # A malformed return and a legitimate `False` are byte-for-byte the
    # same exception from the caller's point of view.
    assert str(malformed_exc.value) == str(legit_exc.value)


def test_failing_post_reset_hook_pins_success_and_sets_post_reset_failed(tmp_path):
    waddle.init(
        "py-reset-post-fail",
        _robot(),
        _control(),
        recording_dir=tmp_path,
        post_reset=lambda task: False,
    )
    with waddle.rollout(task="towel") as ep:
        episode_id = ep.id
        ep.gate([0.0, 0.0, 0.0])
        ep.terminate("success")
    assert ep.outcome == waddle.Outcome.SUCCESS
    assert ep.post_reset_failed
    waddle.shutdown()

    sidecar = _sidecar(tmp_path, episode_id)
    assert sidecar["outcome"] == "TERMINAL_OUTCOME_SUCCESS"  # pinned, unchanged
    assert sidecar["postResetFailed"] is True


# --- with-exit during POST_RESET is a no-op ----------------------------------


def test_with_exit_during_post_reset_does_not_abort_it(tmp_path):
    proceed = threading.Event()
    hook_calls: list[str] = []

    def slow_post_hook(task):
        hook_calls.append(task)
        proceed.wait(timeout=5)
        return True

    waddle.init(
        "py-reset-exit-noop",
        _robot(),
        _control(),
        recording_dir=tmp_path,
        post_reset=slow_post_hook,
    )
    term_thread = None
    with waddle.rollout(task="towel") as ep:
        episode_id = ep.id
        ep.gate([0.0, 0.0, 0.0])
        term_thread = threading.Thread(target=lambda: ep.terminate("success"), daemon=True)
        term_thread.start()

        deadline = time.monotonic() + 5.0
        while not ep.done and time.monotonic() < deadline:
            time.sleep(0.005)
        assert ep.done, "episode never reached POST_RESET"
        assert ep.outcome == waddle.Outcome.SUCCESS
        # Exit here, with the post-reset hook still blocked on `proceed`:
        # `__exit__` must see `done` already True and do nothing.

    assert hook_calls == ["towel"]
    assert ep.outcome == waddle.Outcome.SUCCESS, "the no-op __exit__ must not flip this to ABORT"
    assert not ep.post_reset_failed, "the hook has not returned yet"
    assert term_thread.is_alive(), "terminate() must still be blocked in POST_RESET"

    proceed.set()
    term_thread.join(timeout=5.0)
    assert not term_thread.is_alive(), "terminate() never unblocked"
    assert ep.outcome == waddle.Outcome.SUCCESS  # never altered by post-reset completion
    waddle.shutdown()

    sidecar = _sidecar(tmp_path, episode_id)
    assert sidecar["outcome"] == "TERMINAL_OUTCOME_SUCCESS"


# --- remote reset window via `_testing` hooks --------------------------------


def test_remote_post_reset_window_via_testing_hooks(tmp_path):
    session = waddle.init(
        "py-reset-remote",
        _robot(n_joints=6),
        _control(),
        recording_dir=tmp_path,
        post_reset=waddle.TeleopReset("reset the scene"),
        _testing=True,
    )
    episode_id = None
    with waddle.rollout(task="towel") as ep:
        episode_id = ep.id
        a = [0.0] * 6
        ep.gate(a)

        term_thread = threading.Thread(target=lambda: ep.terminate("success"), daemon=True)
        term_thread.start()

        deadline = time.monotonic() + 5.0
        while not ep.done and time.monotonic() < deadline:
            time.sleep(0.005)
        assert ep.done, "episode never reached POST_RESET"

        waddle._testing.reset_window_engage(session, "claim-remote-1", "teleop")

        deadline = time.monotonic() + 5.0
        noop_seen = False
        while time.monotonic() < deadline:
            waddle._testing.push_teleop(session, [0.5, 0.0, 0.0])
            out = ep.gate(a)
            if out is None and ep.last_gate.kind == "noop":
                assert ep.last_gate.provenance == "teleop"
                noop_seen = True
                break
            time.sleep(0.005)
        assert noop_seen, "gate() never went Noop (RESET_ACTIVE) during the engaged window"

        waddle._testing.reset_window_complete(session, "claim-remote-1", True)
        term_thread.join(timeout=5.0)
        assert not term_thread.is_alive(), "terminate() never unblocked after window complete"

    assert ep.outcome == waddle.Outcome.SUCCESS
    assert not ep.post_reset_failed
    waddle.shutdown()

    sidecar = _sidecar(tmp_path, episode_id)
    kinds = [e["resetWindow"]["kind"] for e in _events(sidecar) if "resetWindow" in e]
    assert "RESET_WINDOW_EVENT_KIND_ENGAGED" in kinds
    assert "RESET_WINDOW_EVENT_KIND_COMPLETED" in kinds


def test_remote_post_reset_window_timeout_sets_post_reset_failed(tmp_path):
    # Real elapsed time, short and deliberate (matches the Rust suite's own
    # `post_reset_window_timeout_pins_outcome_and_flags_failure`, which uses
    # the same 150ms to steer clear of timer-granularity flakiness).
    waddle.init(
        "py-reset-timeout",
        _robot(),
        _control(),
        recording_dir=tmp_path,
        post_reset=waddle.TeleopReset("reset the scene", timeout_s=0.15),
        _testing=True,
    )
    with waddle.rollout(task="towel") as ep:
        episode_id = ep.id
        ep.gate([0.0, 0.0, 0.0])
        ep.terminate("success")  # blocks until the window times out for real
    assert ep.outcome == waddle.Outcome.SUCCESS  # pinned, unchanged by the timeout
    assert ep.post_reset_failed
    waddle.shutdown()

    sidecar = _sidecar(tmp_path, episode_id)
    assert sidecar["outcome"] == "TERMINAL_OUTCOME_SUCCESS"
    assert sidecar["postResetFailed"] is True
    kinds = [e["resetWindow"]["kind"] for e in _events(sidecar) if "resetWindow" in e]
    assert "RESET_WINDOW_EVENT_KIND_TIMED_OUT" in kinds


# --- dim-mismatch intervention rejected with fault (Python surface) --------


def test_dims_mismatch_intervention_rejected_with_fault(tmp_path):
    # A 3-joint robot: `_testing.push_teleop`'s Twist always flattens to 6
    # raw values (linear xyz + angular xyz) — media intake's dims
    # validation must drop every one of these as a mismatch (3
    # declared vs. 6 incoming), never handing them to the gate, and raise
    # exactly one deduplicated Fault.
    session = waddle.init(
        "py-reset-dims-mismatch",
        _robot(n_joints=3),
        _control(),
        recording_dir=tmp_path,
        _testing=True,
    )
    with waddle.rollout(task="towel") as ep:
        episode_id = ep.id
        a = [0.0, 0.0, 0.0]
        for _ in range(5):
            assert ep.gate(a, a) is a

        waddle._testing.engage(session, "claim-dims-1", "teleop")

        # Once claimed, HOLD_FIRST holds (`ep.gate()` returns `None`) until a
        # *valid* intervenor action arrives — dims-mismatched packets must
        # never count as one, so this must never see a substituted/blended
        # action, only the pre-claim `pass` (brief mirror lag) or `hold`.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            waddle._testing.push_teleop(session, [0.7, 0.0, 0.0])
            out = ep.gate(a, a)
            assert out is a or out is None, (
                "a dims-mismatched teleop packet must never reach the gate as a "
                f"substituted/blended action; got {out!r} (kind={ep.last_gate.kind!r})"
            )
            assert ep.last_gate.kind in ("pass", "hold"), (
                f"unexpected gate decision {ep.last_gate.kind!r} for a dims-mismatched stream"
            )
            time.sleep(0.005)
        assert ep.last_gate.kind == "hold", "claim never settled into Hold (nothing ever engaged?)"

        # `terminate()` fires from RUNNING *or* INTERVENTION (E10) — no need
        # to settle/release a claim that never received a valid action first.
        ep.terminate("success")
    waddle.shutdown()

    sidecar = _sidecar(tmp_path, episode_id)
    faults = [e["fault"] for e in _events(sidecar) if "fault" in e]
    assert any(
        f.get("source") == "media-intake" and f.get("kind") == "FAULT_KIND_VALIDATION_ERROR"
        for f in faults
    ), f"expected a media-intake dims-mismatch Fault event, got {faults}"
