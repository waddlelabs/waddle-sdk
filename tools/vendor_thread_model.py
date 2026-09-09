"""Rebuild the external collision surfaces of the native MuJoCo thread.

Use MuJoCo 3.11.0, coacd 1.0.14, trimesh 5.1.0 and numpy, with this SDK on
PYTHONPATH and a C++17 compiler. No geometry or source is downloaded. --output
can generate a separate directory for comparison before replacing the assets.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import coacd
import mujoco
import numpy as np
import trimesh
from waddle_sdk.simulators.thread import _build_plugin

DATA = (
    Path(__file__).resolve().parents[1] / "sdk/python/waddle_sdk/simulators/data/thread"
)
PARAMETERS = {
    "threshold": 0.015,
    "resolution": 1500,
    "mcts_nodes": 15,
    "mcts_iterations": 100,
    "max_convex_hull": 32,
    "seed": 0,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DATA)
    destination = parser.parse_args().output.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    _build_plugin(DATA / "metric_thread.cc")
    root = ET.parse(DATA / "cap.xml").getroot()
    asset = root.find("asset")
    external = {m.get("name") for m in asset.findall("mesh[@file]")}
    for mesh in list(asset):
        if mesh.get("name") in external:
            asset.remove(mesh)
    for body in root.findall("worldbody/body"):
        for geom in body.findall("geom"):
            if geom.get("mesh") in external:
                body.remove(geom)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    coacd.set_log_level("error")
    geometry = {}
    for name, body_name, mask in [("nut", "cap", "4"), ("bolt", "bottle_thread", "8")]:
        mesh = model.mesh(name)
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, model.mesh_quat[mesh.id])
        va, vn = model.mesh_vertadr[mesh.id], model.mesh_vertnum[mesh.id]
        fa, fn = model.mesh_faceadr[mesh.id], model.mesh_facenum[mesh.id]
        vertices = model.mesh_vert[va : va + vn] @ rotation.reshape(3, 3).T
        vertices += model.mesh_pos[mesh.id]
        source = trimesh.Trimesh(vertices, model.mesh_face[fa : fa + fn], process=True)
        parts = coacd.run_coacd(coacd.Mesh(source.vertices, source.faces), **PARAMETERS)
        body = root.find(f"worldbody/body[@name='{body_name}']")
        sdf = body.find("geom[@type='sdf']")
        outputs = []
        for old in destination.glob(f"{name}-[0-9]*.stl"):
            old.unlink()
        for index, (points, faces) in enumerate(parts):
            path = destination / f"{name}-{index}.stl"
            path.write_bytes(
                trimesh.Trimesh(points, faces, process=False).export(file_type="stl")
            )
            outputs.append(
                {
                    "path": path.name,
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                }
            )
            mesh_name = f"{name}_convex_{index}"
            ET.SubElement(asset, "mesh", name=mesh_name, file=path.name)
            ET.SubElement(
                body,
                "geom",
                name=mesh_name,
                type="mesh",
                mesh=mesh_name,
                contype=mask,
                conaffinity="1",
                group="3",
                mass="0",
                **{key: sdf.get(key) for key in ("solref", "solimp", "friction")},
            )
        geometry[name] = {
            "source_sha256": hashlib.sha256(source.export(file_type="stl")).hexdigest(),
            "source_bounds": source.bounds.tolist(),
            "source_watertight": source.is_watertight,
            "parts": outputs,
        }
        print(f"{name}: {len(parts)} native convex pieces", flush=True)
    ET.indent(root)
    (destination / "cap.xml").write_text(ET.tostring(root, encoding="unicode") + "\n")
    if destination != DATA:
        for name in ("metric_thread.cc", "LICENSE", "README.md"):
            if (DATA / name).exists():
                shutil.copy2(DATA / name, destination / name)
    manifest = {
        "source": f"MuJoCo {mujoco.__version__} first-party nut/bolt SDFs, dimensional scale 0.05",
        "nominal_pitch_m_per_turn": 0.05 / 12,
        "native_cap_mass_kg": 0.025,
        "collision_decomposition": {"coacd": "1.0.14", **PARAMETERS},
        "geometry": geometry,
        "files": {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(destination.iterdir())
            if path.is_file() and path.name not in ("manifest.json", "README.md")
        },
    }
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
