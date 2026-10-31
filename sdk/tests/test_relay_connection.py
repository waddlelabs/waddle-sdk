"""Private connection settings stay independent of runtime or application imports."""

import json
import subprocess
import sys

import pytest

pytest.importorskip("fcntl")

from waddle_sdk_relay.connection import ConnectionConfig, load_connection, read_key
from waddle_sdk_relay.errors import RelayError


def test_connection_contract_import_does_not_load_runtime():
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import waddle_sdk_relay.connection; assert 'waddle_sdk' not in sys.modules",
        ],
        check=True,
    )


def test_private_config_resolves_relative_key_without_reading_it(tmp_path):
    private = tmp_path / ".connection"
    private.mkdir()
    path = private / "connection.json"
    path.write_text(
        json.dumps(
            {
                "version": 1,
                "endpoint": "wss://example.com/device",
                "key_file": "key",
                "allow_loopback_ws": False,
                "rig": "my-rig",
            }
        )
    )
    path.chmod(0o600)
    saved = load_connection(path, root=tmp_path)
    assert saved.rig == "my-rig" and saved.key_file == tmp_path / "key"
    assert not saved.key_file.exists()
    assert load_connection(private / "absent.json") is None
    path.chmod(0o644)
    with pytest.raises(RelayError):
        load_connection(path)


def test_key_requires_private_regular_file(tmp_path):
    path = tmp_path / "key"
    path.write_text("synthetic-scoped-token\n")
    path.chmod(0o600)
    assert read_key(path) == "synthetic-scoped-token"
    link = tmp_path / "alias"
    link.symlink_to(path)
    with pytest.raises(RelayError):
        read_key(link)
    path.chmod(0o640)
    with pytest.raises(RelayError):
        read_key(path)


@pytest.mark.parametrize(
    "url",
    ["ws://example.com", "wss://user:key@example.com", "wss://example.com?key=secret"],
)
def test_connection_refuses_plaintext_or_url_credentials(url):
    with pytest.raises(RelayError):
        ConnectionConfig(url, "synthetic-scoped-token", rig="rig")


def test_loopback_is_explicit_and_tokens_do_not_appear_in_repr():
    token = "synthetic-scoped-token"
    config = ConnectionConfig(
        "ws://127.0.0.1:9000", token, allow_loopback_ws=True, rig="rig"
    )
    assert token not in repr(config)
    with pytest.raises(RelayError):
        ConnectionConfig(config.endpoint, token, rig="rig")


def test_connection_parser_does_not_follow_private_directory_symlinks(tmp_path):
    target = tmp_path / "private"
    target.mkdir()
    alias = tmp_path / "alias"
    alias.symlink_to(target, target_is_directory=True)
    with pytest.raises(RelayError):
        load_connection(alias / "connection.json")
