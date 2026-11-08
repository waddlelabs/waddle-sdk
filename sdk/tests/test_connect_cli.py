"""Hosted commands ship with the SDK without importing private planners."""

import subprocess
import sys

from waddle_sdk import cli


def test_hosted_subcommands_delegate_without_legacy_site(monkeypatch):
    from waddle_sdk.connect import bootstrap

    calls = []
    monkeypatch.setattr(bootstrap, "main", lambda argv: calls.append(argv) or 0)
    for name in ("setup", "start", "status"):
        assert cli.main(["connect", name, "--directory", "/tmp/rig"]) == 0
        assert calls[-1] == [name, "--directory", "/tmp/rig"]


def test_legacy_connect_keeps_existing_flags(monkeypatch):
    calls = []
    monkeypatch.setattr(cli, "_connect", lambda args: calls.append(args) or 0)
    assert cli.main(["connect", "--site", "site.yaml"]) == 0
    assert calls[0].site == "site.yaml"


def test_hosted_help_does_not_import_private_planner_or_hardware():
    script = """
import importlib.abc, sys
class Reject(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'waddle_metal', 'torch', 'pyrealsense2', 'i2rt'}:
            raise AssertionError('unexpected dependency: ' + fullname)
sys.meta_path.insert(0, Reject())
from waddle_sdk.cli import main
main(['connect', 'setup', '--help'])
"""
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr
    assert "waddle-sdk connect setup" in result.stdout
    assert "--endpoint" in result.stdout


def test_all_hosted_commands_have_real_help(capsys):
    import pytest

    for command in ("setup", "start", "status", "check"):
        with pytest.raises(SystemExit) as exit:
            cli.main(["connect", command, "--help"])
        assert exit.value.code == 0
        assert "--directory" in capsys.readouterr().out
