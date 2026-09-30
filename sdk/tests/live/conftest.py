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
    with CameraInspection([_site_camera_spec(config, camera)]) as session:
        yield session


def _site_camera_spec(config, camera):
    site = load_site(config["site"])
    row = site.manifest["cameras"][camera]
    return CameraInspectionSpec(
        name=camera,
        driver=row["driver"],
        connection=row["connection"],
        stream={key: int(value) for key, value in row["stream"].items()},
        options=row.get("options", {}),
        site_root=Path(site.path).parent,
    )


@pytest.fixture
def camera_pair_session(request, camera):
    assert request.config.getoption("--live")
    config = request.config.live_bench
    names = config["cameras"]
    if len(names) < 2:
        pytest.skip("simultaneous capture needs two configured cameras")
    peer = names[(names.index(camera) + 1) % len(names)]
    missing = request.config.live_missing.get(f"cameras:{peer}")
    if missing:
        pytest.skip(missing)
    specs = [
        config["camera_specs"][name]
        if name in config.get("camera_specs", {})
        else _site_camera_spec(config, name)
        for name in (camera, peer)
    ]
    with CameraInspection(specs) as session:
        yield session
