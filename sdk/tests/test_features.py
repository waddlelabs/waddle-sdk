"""Two-distribution packaging, from the Python side.

`waddle-sdk` and its `waddle-sdk-teleop` companion are one source tree
built twice; `waddle_sdk._native` decides which of the two compiled cores this
process runs on, and `waddle_sdk.init` keys its refusals off what that core was
BUILT with (`FEATURES`) rather than trying an import and hoping. These
tests pin that: the two projects' metadata held to each other, the default
build's features, the selection rules, and the refusals — plus one smoke
over the real gRPC transport, which watches a session dial a declared plane
and then keep running locally once that plane vanishes, because "the
transport is compiled in" is only worth anything if it is also wired up and
a partition is survivable.
"""

from __future__ import annotations

import socket
import sys
import time
import types
from pathlib import Path

import pytest

import waddle_sdk
from waddle_sdk import descriptors
from waddle_sdk._session import Control, _derive_grants, create_core_session
import waddle_sdk._core as _core
import waddle_sdk._native as _native

try:  # 3.11+
    import tomllib
except ModuleNotFoundError:  # 3.10 (see the `dev` group in pyproject.toml)
    import tomli as tomllib  # type: ignore[no-redef]

_SDK_DIR = Path(__file__).resolve().parents[1]


def _pyproject(*parts: str) -> dict:
    with (_SDK_DIR.joinpath(*parts)).open("rb") as fh:
        return tomllib.load(fh)




def _robot() -> descriptors.Robot:
    return descriptors.Robot(
        name="pytest-features-bot",
        robot_id="py-features-01",
        cell_id="cell-py-features",
        action_space=descriptors.JointSpace(joints=["j0", "j1", "j2"], rate_hz=50),
    )


def _control() -> Control:
    return Control(
        send=lambda chunk: None, hold=lambda: None, resume=lambda: None
    )


def _camera_robot() -> descriptors.Robot:
    return descriptors.Robot(
        name="pytest-features-cam",
        robot_id="py-features-cam-01",
        cell_id="cell-py-features",
        action_space=descriptors.JointSpace(joints=["j0", "j1", "j2"], rate_hz=50),
        cameras={"overhead": descriptors.Camera(width=4, height=4, fps=30)},
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
    assert waddle_sdk.__version__ == _core.__version__
    assert isinstance(waddle_sdk.__version__, str) and waddle_sdk.__version__


def test_python_and_selected_native_core_share_the_binding_api():
    assert _core.BINDING_API_VERSION == _native._REQUIRED_BINDING_API_VERSION
    assert _native.core.BINDING_API_VERSION == _native._REQUIRED_BINDING_API_VERSION


# --- The two projects' metadata, held to each other ------------------------


def test_the_teleop_extra_pins_this_builds_version():
    """`teleop = ["waddle-sdk-teleop==X"]` is the one version in this project
    that maturin does NOT derive from `rust/Cargo.toml`, so it is the one
    that can drift: bump the crate, ship two 0.2.0 wheels, and an extra
    still pinned to 0.1.0 either fails to resolve or installs last
    release's companion — whereupon `_native` sees the mismatch, warns, and
    falls back to the bundled core, i.e. `pip install 'waddle-sdk[teleop]'`
    silently yields no LiveKit. That is exactly what the exact pin exists to
    prevent, so the pin is checked here rather than left to whoever
    remembers to edit two files."""
    extras = _pyproject("pyproject.toml")["project"]["optional-dependencies"]
    assert extras["teleop"] == [f"waddle-sdk-teleop=={waddle_sdk.__version__}"]


def test_both_distributions_are_one_build_of_one_manifest():
    """What makes the exact pin (and `_native`'s version check) mean
    anything: the companion is not a separate project that happens to
    share a version, it is this project's shim built from the SAME
    Cargo.toml with one feature added."""
    default = _pyproject("pyproject.toml")["tool"]["maturin"]
    companion_project = _pyproject("teleop", "pyproject.toml")
    companion = companion_project["tool"]["maturin"]

    assert companion_project["project"]["name"] == "waddle-sdk-teleop"
    assert (_SDK_DIR / default["manifest-path"]).resolve() == (
        _SDK_DIR / "teleop" / companion["manifest-path"]
    ).resolve()

    # A strict superset, and the extra is exactly the media plane: the
    # default wheel must never grow libwebrtc, and the companion must never
    # lose the control transport it is a superset of.
    assert set(default["features"]) < set(companion["features"])
    assert set(companion["features"]) - set(default["features"]) == {"livekit"}
    assert "livekit" not in default["features"]


# --- Choosing a core -------------------------------------------------------


def _fake_teleop_core(version: str) -> types.ModuleType:
    module = types.ModuleType("waddle_teleop._core")
    module.__version__ = version
    module.BINDING_API_VERSION = _native._REQUIRED_BINDING_API_VERSION
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


def test_select_core_falls_back_on_a_binding_api_mismatch(monkeypatch):
    teleop = _fake_teleop_core(_core.__version__)
    teleop.BINDING_API_VERSION -= 1
    _install_fake_teleop(monkeypatch, teleop)
    with pytest.warns(RuntimeWarning, match="native binding"):
        assert _native._select_core() is _core


def test_select_core_honors_the_opt_out(monkeypatch):
    teleop = _fake_teleop_core(_core.__version__)
    _install_fake_teleop(monkeypatch, teleop)
    monkeypatch.setenv("WADDLE_NO_TELEOP", "1")
    assert _native._select_core() is _core


def test_bundled_core_stays_reachable_by_name():
    """`import waddle_sdk._core` must keep meaning the BUNDLED module whatever
    `_native` selected — the selection changes which core the package
    *uses*, never what a submodule name resolves to."""
    from waddle_sdk import _core as by_name

    assert by_name is sys.modules["waddle_sdk._core"]


# --- Refusals, keyed on what the core was built with -----------------------


def test_media_without_livekit_names_the_teleop_extra(monkeypatch):
    monkeypatch.setattr(_native, "FEATURES", frozenset({"grpc"}))
    with pytest.raises(RuntimeError, match=r"waddle-sdk\[teleop\]") as excinfo:
        create_core_session(
            "py-features-media",
            _camera_robot(),
            _control(),
            media=waddle_sdk.LiveKit(url="wss://example.invalid", token="tok"),
        )
    assert "not compiled" in str(excinfo.value)


def test_transport_without_grpc_names_the_from_source_build(monkeypatch):
    monkeypatch.setattr(_native, "FEATURES", frozenset())
    with pytest.raises(RuntimeError, match="grpc") as excinfo:
        create_core_session(
            "py-features-transport",
            _robot(),
            _control(),
            transport=waddle_sdk.Grpc("http://127.0.0.1:9"),
        )
    assert "maturin develop --features grpc" in str(excinfo.value)


def test_transport_and_testing_are_mutually_exclusive():
    with pytest.raises(ValueError, match="transport and _testing"):
        create_core_session(
            "py-features-both",
            _robot(),
            _control(),
            transport=waddle_sdk.Grpc("http://127.0.0.1:9"),
            _testing=True,
        )


def test_transport_declaration_validates_its_shape():
    with pytest.raises(ValueError, match="Grpc.url"):
        waddle_sdk.Grpc("")
    with pytest.raises(ValueError, match="Grpc.token"):
        waddle_sdk.Grpc("http://127.0.0.1:9", token="")
    with pytest.raises(TypeError, match="transport must be"):
        create_core_session(
            "py-features-shape",
            _robot(),
            _control(),
            transport="http://127.0.0.1:9",
        )


# --- The real transport, with nothing at the other end ---------------------

#: What an HTTP/2 client writes before anything else (RFC 9113 §3.4). Seeing
#: it proves the accepted connection came from the gRPC client, not from
#: something that merely opened a socket.
_H2_PREFACE = b"PRI * HTTP/2.0\r\n\r\nSM\r\n\r\n"


def _read_exactly(conn: socket.socket, count: int, timeout_s: float) -> bytes:
    conn.settimeout(timeout_s)
    buf = b""
    while len(buf) < count:
        chunk = conn.recv(count - len(buf))
        if not chunk:
            break
        buf += chunk
    return buf


@pytest.mark.skipif(
    "grpc" not in _native.FEATURES,
    reason="the selected core has no control transport",
)
def test_grpc_session_dials_the_plane_and_then_survives_losing_it(tmp_path):
    """The one test that proves `init(transport=...)` reaches the real
    transport: a listener stands in for the plane just long enough to
    observe the dial, then goes away.

    The accept() is the load-bearing assertion. Everything else here — a
    rollout that gates locally, a prompt shutdown, a sidecar on disk — is
    equally true of a session with no transport at all, so a regression
    that dropped the wiring and quietly ran unsupervised (the failure
    `init`'s own FEATURES refusals exist to prevent) would sail past them.

    Once the listener closes, the port refuses and the rest is the
    partition path: the core client owns connect/backoff/replay on its own
    thread, the rollout runs locally throughout, and shutdown has to JOIN
    that thread mid-backoff — a supervision session must never take a
    reconnect timer's worth of time to exit."""
    plane = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    plane.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    plane.bind(("127.0.0.1", 0))
    plane.listen(1)
    plane.settimeout(15.0)
    port = plane.getsockname()[1]

    try:
        session = create_core_session(
            "py-grpc-offline",
            _robot(),
            _control(),
            recording_dir=tmp_path,
            transport=waddle_sdk.Grpc(f"http://127.0.0.1:{port}", token="not-a-real-token"),
        )
        assert session is not None

        try:
            conn, _ = plane.accept()
        except TimeoutError:
            pytest.fail(
                "the session never dialed the declared plane: the gRPC "
                "transport is not wired into init(transport=...)"
            )
        with conn:
            assert _read_exactly(conn, len(_H2_PREFACE), 15.0) == _H2_PREFACE
        # Hanging up here (and closing the listener below) is what puts the
        # client into the connect → backoff → retry loop the rest of this
        # test rides on.
    finally:
        plane.close()

    ep = session.start_episode("the plane is not home")
    try:
        action = [0.1, 0.2, 0.3]
        assert ep.gate(action, [0.0, 0.0, 0.0]) is action
        episode_id = ep.id
        ep.terminate("success")
    finally:
        if not ep.done:
            ep.terminate("abort")

    started = time.monotonic()
    session.shutdown()
    elapsed = time.monotonic() - started
    assert elapsed < 10.0, f"shutdown stalled on the transport backoff ({elapsed:.1f}s)"
    assert (tmp_path / f"{episode_id}.sidecar.json").exists()
