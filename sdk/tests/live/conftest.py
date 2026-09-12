"""Resources are opened only by selected live tests, never by discovery."""

from pathlib import Path

import pytest
from waddle_sdk import load_site
from waddle_sdk.cameras.inspection import CameraInspection, CameraInspectionSpec

from .sdk_live.session import Bench


@pytest.fixture
def bench(request):
    assert request.config.getoption("--live")
    params = request.node.callspec.params
    part = params.get("part") or params["case"]["part"]
    with Bench(request.config.live_bench, part) as owner:
        yield owner


@pytest.fixture
def camera_session(request, camera):
    assert request.config.getoption("--live")
    config = request.config.live_bench
    if camera in config.get("camera_specs", {}):
        with CameraInspection([config["camera_specs"][camera]]) as session:
            yield session
        return
    site = load_site(config["site"])
    row = site.manifest["cameras"][camera]
    spec = CameraInspectionSpec(
        name=camera,
        driver=row["driver"],
        connection=row["connection"],
        stream={key: int(value) for key, value in row["stream"].items()},
        options=row.get("options", {}),
        site_root=Path(site.path).parent,
    )
    with CameraInspection([spec]) as session:
        yield session
