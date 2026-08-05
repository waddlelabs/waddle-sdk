"""The published YAM example (`examples/yam_bimanual.py`) has to keep working.

It is the file a customer with this rig copies, and it is a PROGRAM — five
Waddle-facing lines around a table of site numbers — so it is tested the way
it is run: as a subprocess, with nothing configured. What that reaches is the
factory: the site numbers it passes have to still be arguments `yam.bimanual`
accepts, and building the rig has to still be declaration only. The session
those lines open is covered in `test_yam_session.py`, against a rig this test
does not need a plane to build.
"""

import os
import subprocess
import sys
from pathlib import Path

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "yam_bimanual.py"


def _run(**overrides):
    env = dict(os.environ)
    # Never inherit a developer's plane, or their bench.
    for leaked in (
        "WADDLE_YAM_TRANSPORT",
        "WADDLE_YAM_TOKEN",
        "WADDLE_YAM_SIM",
        "WADDLE_YAM_CAN_LEFT",
        "WADDLE_YAM_CAN_RIGHT",
    ):
        env.pop(leaked, None)
    env.update(overrides)
    return subprocess.run(
        [sys.executable, str(EXAMPLE)],
        capture_output=True,
        text=True,
        timeout=120,
        env=env,
    )


def test_the_example_builds_its_rig_and_says_what_it_still_needs():
    # With no plane there is nobody to invite, and the program says so
    # instead of opening a session that can only record. Everything before
    # that line still ran: the factory took the example's own workspace box,
    # gripper limits and cross-arm mounting, which is the half of this file
    # that a change to `yam.bimanual`'s signature would break.
    done = _run()
    assert done.returncode == 2, done.stderr
    assert "WADDLE_YAM_TRANSPORT" in done.stdout


def test_the_example_opens_nothing_until_it_is_asked_to():
    # A factory call is declaration only — no bus, no thread — which is what
    # makes it safe to write above the decision to run. On a machine with no
    # CAN interfaces and no vendor package, a live rig must therefore still
    # BUILD, and fail (if at all) only where the arms open.
    done = _run(WADDLE_YAM_SIM="0", WADDLE_YAM_CAN_LEFT="can_left", WADDLE_YAM_CAN_RIGHT="can_right")
    assert done.returncode == 2, done.stderr
    assert "WADDLE_YAM_TRANSPORT" in done.stdout
