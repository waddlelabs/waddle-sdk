"""SDK-only site ownership, including independent processes and failed teardown."""

import subprocess
import sys
from pathlib import Path
from uuid import uuid4

import pytest
import site_fixtures
from test_site_api import _write_site
from waddle_sdk import load_site
from waddle_sdk.ownership import SiteOwnershipError
from waddle_sdk.runtime import MediaRuntimePort


def site(tmp_path):
    path = _write_site(tmp_path)
    path.write_text(path.read_text().replace("test-cell", "owner-" + uuid4().hex))
    return load_site(path)


def test_same_site_id_is_exclusive_before_factory_and_across_processes(
    tmp_path, monkeypatch
):
    selected = site(tmp_path)
    with selected.open(console=False, _testing=True) as sdk:

        def forbidden(**kwargs):
            pytest.fail("Duplicate site reached its hardware factory")

        monkeypatch.setattr(site_fixtures, "part", forbidden)
        with (
            pytest.raises(SiteOwnershipError) as error,
            selected.open(console=False, _testing=True),
        ):
            pass
        assert error.value.code == "site_owned"
        program = """import sys
from waddle_sdk import load_site
from waddle_sdk.ownership import SiteOwnershipError
try:
    with load_site(sys.argv[1]).open(console=False, _testing=True):
        raise AssertionError("Duplicate site opened")
except SiteOwnershipError as error:
    print(error.code)
    sys.exit(23)
"""
        result = subprocess.run(
            [sys.executable, "-c", program, str(selected.path)],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert result.returncode == 23, result.stderr
        assert result.stdout.strip() == "site_owned"
        assert isinstance(sdk, MediaRuntimePort)
    monkeypatch.undo()
    with selected.open(console=False, _testing=True):
        pass


def test_failure_before_hardware_construction_releases_owner(tmp_path, monkeypatch):
    selected = site(tmp_path)
    original = site_fixtures.part

    def fail(**kwargs):
        raise ValueError("invalid fixture")

    monkeypatch.setattr(site_fixtures, "part", fail)
    with (
        pytest.raises(ValueError, match="invalid fixture"),
        selected.open(console=False, _testing=True),
    ):
        pass
    monkeypatch.setattr(site_fixtures, "part", original)
    with selected.open(console=False, _testing=True):
        pass


@pytest.mark.parametrize("device", ["arm", "camera"])
def test_failed_device_close_retains_owner_for_process_lifetime(tmp_path, device):
    selected = site(tmp_path)
    # A child intentionally retains an uncertain owner; no test cleanup can
    # pretend that its failed physical teardown became confirmed.
    program = """import sys
sys.path.insert(0, sys.argv[2])
import site_fixtures
from waddle_sdk import load_site
from waddle_sdk.ownership import SiteOwnershipError
selected = load_site(sys.argv[1])
sdk = selected.open(console=False, _testing=True).__enter__()
kind = sys.argv[3]
original = site_fixtures._Driver.close if kind == "arm" else site_fixtures._Camera.close
def fail(self):
    original(self)
    raise RuntimeError("injected teardown failure")
if kind == "arm":
    site_fixtures._Driver.close = fail
else:
    site_fixtures._Camera.close = fail
try:
    sdk.__exit__(None, None, None)
    raise AssertionError("Teardown uncertainty was hidden")
except SiteOwnershipError as error:
    assert error.code == "site_close_unknown"
try:
    selected.open(console=False, _testing=True).__enter__()
    raise AssertionError("Uncertain owner released")
except SiteOwnershipError as error:
    assert error.code == "site_owned"
from waddle_sdk.runtime import RuntimeFault
try:
    sdk.observe()
    raise AssertionError("Closing session remained available")
except RuntimeFault:
    pass
print("retained")
"""
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            program,
            str(selected.path),
            str(Path(site_fixtures.__file__).parent),
            device,
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip().splitlines()[-1] == "retained"
    # Process exit releases the kernel resource, without claiming a safe robot state.
    with selected.open(console=False, _testing=True):
        pass


def test_no_media_returns_no_tracks_and_closed_session_refuses(tmp_path):
    from waddle_sdk.runtime import RuntimeFault

    with site(tmp_path).open(console=False) as sdk:
        assert sdk.media_tracks() == []
    with pytest.raises(RuntimeFault):
        sdk.media_tracks()
