"""Applications extend device setup without entering the default import path."""

from types import SimpleNamespace

import pytest
from waddle_sdk.connect import extensions


def provider(value, loaded):
    def load():
        loaded.append(value)
        return value

    return SimpleNamespace(load=load)


def test_selecting_execution_loads_only_the_requested_installed_extension(monkeypatch):
    loaded = []
    selected = SimpleNamespace(main=lambda argv, **kw: (argv, kw))

    def discover(*, group, name):
        assert group == "waddle_sdk.connect_execution" and name == "local"
        return [provider(selected, loaded)]

    monkeypatch.setattr(extensions, "entry_points", discover)
    assert not loaded
    assert extensions.execution("local").main(["start"], prog="example") == (
        ["start"],
        {"prog": "example"},
    )
    assert loaded == [selected]


@pytest.mark.parametrize("count", [0, 2])
def test_unavailable_or_ambiguous_execution_never_starts_an_extension(
    monkeypatch, count
):
    loaded = []
    monkeypatch.setattr(
        extensions, "entry_points", lambda **kw: [provider(object(), loaded)] * count
    )
    with pytest.raises(ValueError, match="exactly one installed"):
        extensions.execution("local")
    assert not loaded


def test_saved_layout_is_reused_without_rewriting_credentials(tmp_path, monkeypatch):
    existing = tmp_path / "existing-application"
    existing.mkdir()
    credential = existing / "connection.json"
    credential.write_bytes(b"original bytes")
    loaded = []
    monkeypatch.setattr(
        extensions,
        "entry_points",
        lambda **kw: [provider(lambda workspace: existing, loaded)],
    )
    assert extensions.connection_directory(tmp_path / "workspace.yaml") == existing
    assert credential.read_bytes() == b"original bytes"
    assert not (tmp_path / ".waddle").exists()


def test_conflicting_layouts_refuse_before_changing_either(tmp_path, monkeypatch):
    current = tmp_path / ".waddle"
    current.mkdir()
    (current / "connection.json").write_bytes(b"current")
    existing = tmp_path / "existing"
    monkeypatch.setattr(
        extensions,
        "entry_points",
        lambda **kw: [provider(lambda workspace: existing, [])],
    )
    with pytest.raises(ValueError, match="Multiple saved connection layouts"):
        extensions.connection_directory(tmp_path / "workspace.yaml")
    assert (current / "connection.json").read_bytes() == b"current"
    assert not existing.exists()
