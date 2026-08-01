"""Two-distribution packaging, from the Python side.

`waddle-sdk` and its `waddle-sdk-teleop` companion are one source tree
built twice; `waddle._native` decides which of the two compiled cores this
process runs on, and `waddle.init` keys its refusals off what that core was
BUILT with (`FEATURES`) rather than trying an import and hoping. These
tests pin all three: the default build's features, the selection rules, and
the refusals — plus one offline smoke over the real gRPC transport, because
"the transport is compiled in" is only worth anything if a session that
declares an unreachable plane still starts, runs, and shuts down.
"""

from __future__ import annotations

import sys
import time
import types

import pytest

import waddle
import waddle._core as _core
import waddle._native as _native


@pytest.fixture(autouse=True)
def _clean_session():
    yield
    waddle.shutdown()


def _robot() -> waddle.Robot:
    return waddle.Robot(
        name="pytest-features-bot",
        robot_id="py-features-01",
        cell_id="cell-py-features",
        action_space=waddle.JointSpace(joints=["j0", "j1", "j2"], rate_hz=50),
    )


def _control() -> waddle.Control:
    return waddle.Control(
        send=lambda chunk: None, hold=lambda: None, resume=lambda: None
    )


def _camera_robot() -> waddle.Robot:
    return waddle.Robot(
        name="pytest-features-cam",
        robot_id="py-features-cam-01",
        cell_id="cell-py-features",
        action_space=waddle.JointSpace(joints=["j0", "j1", "j2"], rate_hz=50),
        cameras={"overhead": waddle.Camera(width=4, height=4, fps=30)},
    )


# --- What this build is ----------------------------------------------------


def test_the_default_build_carries_the_control_transport():
    """`sdk/pyproject.toml`'s `[tool.maturin] features` — what `uv sync
    --dev` builds and what the published `waddle-sdk` wheel ships. A
    supervision SDK whose default install cannot reach the plane would be a
    strange thing to ship."""
    assert "grpc" in _core.FEATURES
    assert _core.FEATURES <= {"grpc", "livekit"}
    # Whatever `_native` selected, its FEATURES are that core's own — never
    # a Python-side idea of what should be available.
    assert _native.FEATURES == _native.core.FEATURES
    assert "grpc" in _native.FEATURES


@pytest.mark.skipif(
    "livekit" in _core.FEATURES,
    reason="locally built with the teleop feature (the companion wheel's flavour)",
)
def test_the_default_build_leaves_livekit_to_the_companion_wheel():
    assert _core.FEATURES == frozenset({"grpc"})


def test_version_is_the_cores_own():
    """One Cargo.toml, one version: the Python surface and the shim ship
    together, and the teleop companion is built from the same manifest —
    which is what makes `_native`'s version check meaningful."""
    assert waddle.__version__ == _core.__version__
    assert isinstance(waddle.__version__, str) and waddle.__version__


# --- Choosing a core -------------------------------------------------------


def _fake_teleop_core(version: str) -> types.ModuleType:
    module = types.ModuleType("waddle_teleop._core")
    module.__version__ = version
    module.FEATURES = frozenset({"grpc", "livekit"})
    return module


def _install_fake_teleop(monkeypatch, core_module: types.ModuleType) -> None:
    package = types.ModuleType("waddle_teleop")
    package._core = core_module
    monkeypatch.setitem(sys.modules, "waddle_teleop", package)
    monkeypatch.setitem(sys.modules, "waddle_teleop._core", core_module)


def test_select_core_without_the_companion_returns_the_bundled_core(monkeypatch):
    # `None` in sys.modules makes the import fail — the shape of an
    # environment that only ever installed `waddle-sdk`.
    monkeypatch.setitem(sys.modules, "waddle_teleop", None)
    assert _native._select_core() is _core


def test_select_core_prefers_a_matching_companion(monkeypatch):
    teleop = _fake_teleop_core(_core.__version__)
    _install_fake_teleop(monkeypatch, teleop)
    assert _native._select_core() is teleop


def test_select_core_falls_back_on_a_version_mismatch(monkeypatch):
    """A mismatched pair is a half-upgraded environment: loading a core
    built from other sources than this Python surface expects is not a risk
    worth taking for a media transport. Warn, naming both versions and the
    one command that fixes it, and keep the bundled core."""
    teleop = _fake_teleop_core("9.9.9")
    _install_fake_teleop(monkeypatch, teleop)
    with pytest.warns(RuntimeWarning) as record:
        assert _native._select_core() is _core
    message = str(record[0].message)
    assert "9.9.9" in message and _core.__version__ in message
    assert "pip install" in message


def test_select_core_honors_the_opt_out(monkeypatch):
    teleop = _fake_teleop_core(_core.__version__)
    _install_fake_teleop(monkeypatch, teleop)
    monkeypatch.setenv("WADDLE_NO_TELEOP", "1")
    assert _native._select_core() is _core


def test_bundled_core_stays_reachable_by_name():
    """`import waddle._core` must keep meaning the BUNDLED module whatever
    `_native` selected — the selection changes which core the package
    *uses*, never what a submodule name resolves to."""
    from waddle import _core as by_name

    assert by_name is sys.modules["waddle._core"]


# --- Refusals, keyed on what the core was built with -----------------------


def test_media_without_livekit_names_the_teleop_extra(monkeypatch):
    monkeypatch.setattr(_native, "FEATURES", frozenset({"grpc"}))
    with pytest.raises(RuntimeError, match=r"waddle-sdk\[teleop\]") as excinfo:
        waddle.init(
            "py-features-media",
            _camera_robot(),
            _control(),
            media=waddle.LiveKit(url="wss://example.invalid", token="tok"),
        )
    assert "not compiled" in str(excinfo.value)


def test_transport_without_grpc_names_the_from_source_build(monkeypatch):
    monkeypatch.setattr(_native, "FEATURES", frozenset())
    with pytest.raises(RuntimeError, match="grpc") as excinfo:
        waddle.init(
            "py-features-transport",
            _robot(),
            _control(),
            transport=waddle.Grpc("http://127.0.0.1:9"),
        )
    assert "maturin develop --features grpc" in str(excinfo.value)


def test_transport_and_testing_are_mutually_exclusive():
    with pytest.raises(ValueError, match="transport and _testing"):
        waddle.init(
            "py-features-both",
            _robot(),
            _control(),
            transport=waddle.Grpc("http://127.0.0.1:9"),
            _testing=True,
        )


def test_transport_declaration_validates_its_shape():
    with pytest.raises(ValueError, match="Grpc.url"):
        waddle.Grpc("")
    with pytest.raises(ValueError, match="Grpc.token"):
        waddle.Grpc("http://127.0.0.1:9", token="")
    with pytest.raises(TypeError, match="transport must be"):
        waddle.init(
            "py-features-shape",
            _robot(),
            _control(),
            transport="http://127.0.0.1:9",
        )


# --- The real transport, with nothing at the other end ---------------------


@pytest.mark.skipif(
    "grpc" not in _core.FEATURES, reason="this build has no control transport"
)
def test_grpc_session_runs_and_shuts_down_while_the_plane_is_unreachable(tmp_path):
    """Port 9 (discard) refuses instantly, so this exercises the partition
    path from the first connect attempt: constructing the transport dials
    nothing, the core client owns connect/backoff/replay on its own thread,
    and a rollout must run locally throughout. Shutdown then has to JOIN
    that thread mid-backoff — a supervision session must never take a
    reconnect timer's worth of time to exit."""
    session = waddle.init(
        "py-grpc-offline",
        _robot(),
        _control(),
        recording_dir=tmp_path,
        transport=waddle.Grpc("http://127.0.0.1:9", token="not-a-real-token"),
    )
    assert session is not None

    with waddle.rollout(task="the plane is not home") as ep:
        action = [0.1, 0.2, 0.3]
        assert ep.gate(action, [0.0, 0.0, 0.0]) is action
        episode_id = ep.id
        ep.terminate("success")

    started = time.monotonic()
    waddle.shutdown()
    elapsed = time.monotonic() - started
    assert elapsed < 10.0, f"shutdown stalled on the transport backoff ({elapsed:.1f}s)"
    assert (tmp_path / f"{episode_id}.sidecar.json").exists()
