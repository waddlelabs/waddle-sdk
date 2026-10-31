"""The optional bridge uses only public SDK contracts and caller-owned transport."""

from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("fcntl")
pytest.importorskip("msgpack")

from waddle_sdk.runtime import Observation, PartObservation
from waddle_sdk_relay.device import Device, checked_path
from waddle_sdk_relay.errors import RelayError
from waddle_sdk_relay.publisher import publisher_binding
from waddle_sdk_relay.wire import decode, encode


def test_protocol_and_publisher_imports_do_not_load_the_sdk_runtime():
    import subprocess
    import sys

    subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import sys; import waddle_sdk_relay.errors; "
                "import waddle_sdk_relay.protocol; import waddle_sdk_relay.publisher; "
                "assert not any(n == 'waddle_sdk' or n.startswith('waddle_sdk.') for n in sys.modules); "
                "assert 'msgpack' not in sys.modules"
            ),
        ],
        check=True,
    )


def test_bridge_constructs_without_opening_hardware(tmp_path):
    def refuse_open(**kwargs):
        pytest.fail("Construction and file operations must not open hardware")

    device = Device(SimpleNamespace(open=refuse_open), tmp_path)
    device.dispatch(
        "workspace",
        {
            "operation": "write",
            "path": "site.yaml",
            "content": "site: example\n",
            "expected_revision": None,
        },
    )
    assert [row["path"] for row in device.inventory()] == ["site.yaml"]
    assert device.sdk is None and not device.opened_once


@pytest.mark.parametrize(
    "path",
    [".env", "config/.secrets/key", "../file", "a//b", "/file", "recordings/run"],
)
def test_bridge_refuses_private_and_unconfined_paths(path):
    with pytest.raises(RelayError) as error:
        checked_path(path)
    assert error.value.code == "sdk_workspace_path"


def test_inventory_omits_hidden_files_and_directories(tmp_path):
    (tmp_path / "site.yaml").write_text("site: example\n")
    (tmp_path / ".env").write_text("private")
    (tmp_path / ".credentials").mkdir()
    (tmp_path / ".credentials" / "key").write_text("private")
    (tmp_path / "recordings").mkdir()
    (tmp_path / "recordings" / "run").write_text("private")
    device = Device(None, tmp_path)
    assert [row["path"] for row in device.inventory()] == ["site.yaml"]
    with pytest.raises(RelayError):
        device.dispatch("workspace", {"operation": "read", "path": ".env"})


def test_wire_copies_measured_values_without_writable_aliases():
    source = Observation(
        7, 11, {"arm": PartObservation(np.array([0.5]), np.array([0.0]))}, {}, {}
    )
    copied = decode(encode(source))
    assert (copied.session_ns, copied.unix_ns) == (7, 11)
    assert np.array_equal(copied.parts["arm"].joint_position, [0.5])
    assert not copied.parts["arm"].joint_position.flags.writeable
    source.parts["arm"].joint_position[0] = 0.75
    assert copied.parts["arm"].joint_position[0] == 0.5


@pytest.mark.parametrize(
    "url,allowed",
    [
        ("wss://media.example.com", True),
        ("https://media.example.com", True),
        ("ws://media.example.com", False),
        ("wss://user:password@media.example.com", False),
        ("wss://media.example.com?token=secret", False),
        ("ws://127.0.0.1:7880", False),
    ],
)
def test_publisher_requires_tls_without_url_credentials(url, allowed):
    publisher = {
        "url": url,
        "token": "scoped-token",
        "room": "room",
        "publisher_identity": "camera",
    }
    if allowed:
        assert publisher_binding(publisher, allow_loopback_ws=False) == (
            url,
            "room",
            "camera",
        )
    else:
        with pytest.raises(RelayError):
            publisher_binding(publisher, allow_loopback_ws=False)


def test_publisher_loopback_requires_explicit_opt_in():
    publisher = {
        "url": "ws://127.0.0.1:7880",
        "token": "scoped-token",
        "room": "room",
        "publisher_identity": "camera",
    }
    assert publisher_binding(publisher, allow_loopback_ws=True) == (
        publisher["url"],
        "room",
        "camera",
    )


def test_native_playback_uses_the_site_run_and_retains_its_receipt(tmp_path):
    import site_fixtures
    from test_finite_playback import motion
    from test_site_api import _write_site
    from waddle_sdk import load_site

    site_fixtures.reset()
    site = load_site(_write_site(tmp_path))
    # The only opened drivers are this suite's mock arm and synthetic camera.
    local = SimpleNamespace(open=lambda **kw: site.open(_testing=True, **kw))
    device = Device(local, tmp_path, torque_release_authorized=True)
    try:
        assert device.dispatch("open", {}) == {"safety_slots": 2}
        run_id = device.dispatch(
            "run_enter", {"task": "local references", "actor": "test"}
        )
        admitted = device.dispatch(
            "playback_start",
            {
                "run_id": run_id,
                "motion": decode(encode(motion((0, 0), (0.02, -0.01)))),
            },
        )
        handle = device._playbacks[(run_id, admitted["id"])]
        completed = handle.wait()
        receipt = device.dispatch(
            "playback_status", {"run_id": run_id, "id": admitted["id"]}
        )
        assert receipt == completed
        assert receipt["status"] == "arrived"
        assert receipt["dispatched"] == 2
        with pytest.raises(RelayError) as wrong_run:
            device.dispatch(
                "playback_status", {"run_id": "another-run", "id": admitted["id"]}
            )
        assert wrong_run.value.code == "sdk_playback_expired"
    finally:
        device.disconnect()
        assert site_fixtures.opened == site_fixtures.closed
        site_fixtures.reset()


def test_playback_bounds_require_active_run_and_roundtrip_public_values(tmp_path):
    from waddle_sdk.runtime import JointPositionBounds

    calls = []
    device = Device(None, tmp_path)
    device.sdk = object()
    device.run = SimpleNamespace(id="current", set_playback_bounds=calls.append)
    bounds = {"left": JointPositionBounds((-0.2, 0), (0.2, 1))}
    copied = decode(encode(bounds))
    assert copied == bounds
    with pytest.raises(RelayError) as caught:
        device.dispatch("playback_bounds", {"run_id": "old", "bounds": copied})
    assert caught.value.code == "sdk_run_binding"
    assert not calls
    device.dispatch("playback_bounds", {"run_id": "current", "bounds": copied})
    assert calls == [bounds]
