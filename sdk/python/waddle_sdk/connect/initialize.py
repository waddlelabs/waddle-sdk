"""Reviewed device-site setup without a local planner or application owner."""

import json
import sys
from pathlib import Path
from uuid import uuid4

import yaml

from waddle_sdk import load_site
from waddle_sdk.workspace.initialization import build_site


def build_workspace(destination, site=None, *, source_site=None, site_files=None):
    identity = str(uuid4())

    def configure(stage, document):
        path = stage / "workspace.yaml"
        path.write_text(
            yaml.safe_dump(
                {
                    "version": 1,
                    "id": identity,
                    "site": "site.yaml",
                    "motion": {"prepare_models": True},
                },
                sort_keys=False,
            )
        )
        path.chmod(0o600)

    built = build_site(
        destination,
        site,
        source_site=source_site,
        site_files=site_files,
        configure=configure,
    )
    return {
        **built,
        "workspace": str(Path(built["site"]).with_name("workspace.yaml")),
        "workspace_id": identity,
    }


def initialize(
    directory, *, source_site=None, simulation=False, prepare_drivers=None, site_id=None
):
    def build(destination, site=None, *, source_site=None, **kwargs):
        if prepare_drivers is not None:
            if source_site:
                selected = Path(source_site).expanduser()
                manifest = load_site(
                    selected / "site.yaml" if selected.is_dir() else selected
                ).manifest
            else:
                manifest = site or {}
            prepare_drivers(
                [
                    row["driver"]
                    for section in ("parts", "cameras")
                    for row in manifest.get(section, {}).values()
                    if "driver" in row
                ]
            )
        return build_workspace(destination, site, source_site=source_site, **kwargs)

    if simulation:
        from waddle_sdk.simulators import make_site

        site, scene = make_site(
            directory.name, backend="mujoco", robot="yam", environment="two_cubes"
        )
        return build(
            directory,
            site,
            site_files={
                "simulation.json": (json.dumps(scene, indent=2) + "\n").encode()
            },
        )
    if source_site and not sys.stdin.isatty():
        return build(directory, source_site=source_site)
    if not sys.stdin.isatty():
        raise ValueError(
            "Run setup in a terminal, or explicitly select --site or --simulation"
        )
    from waddle_sdk.workspace.tui import run_initializer

    result, _ = run_initializer(
        directory,
        source_site=source_site,
        host="hosted",
        hosts=("hosted",),
        builder=build,
        prepare_drivers=prepare_drivers,
        site_id=site_id,
    )
    return result
