"""The shipped Site API example is an executable customer program."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import waddle_sdk

SDK = Path(__file__).resolve().parents[1]


def test_example_manifest_is_strict_and_uses_simulation():
    site = waddle_sdk.load_site(SDK / "examples" / "site.yaml")
    assert site.id == "simulated-yam"
    assert site.manifest["parts"]["arm"]["options"]["sim"] is True
    assert site.manifest["parts"]["arm"]["gripper"] == {
        "joint": "gripper",
        "closed_m": 0.0,
        "open_m": 0.095,
        "closed_action": 0.0,
        "open_action": 1.0,
    }
    assert "graph" not in site.manifest
    assert "skills" not in site.manifest


def test_example_program_runs_as_a_subprocess(tmp_path):
    source = SDK / "examples" / "site.yaml"
    manifest = tmp_path / "site.yaml"
    manifest.write_bytes(source.read_bytes())
    program = (SDK / "examples" / "run_site.py").read_text()
    program = program.replace(
        'Path(__file__).with_name("site.yaml")',
        repr(str(manifest)),
    )
    command = [
        sys.executable,
        "-c",
        program,
    ]
    result = subprocess.run(
        command,
        cwd=SDK,
        text=True,
        capture_output=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    assert list((tmp_path / "recordings").glob("*.mcap"))
