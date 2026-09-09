"""Freely removable cap built from MuJoCo's first-party thread geometry.

Only dimensional scaling lives in the small compiled plugin. MuJoCo supplies
the SDFs and contact solver; no callback applies forces or changes constraints.
SAPIEN and Isaac use offline surface meshes with native GPU PhysX SDF contacts.
"""

from __future__ import annotations

import hashlib
import json
import os
import shlex
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from functools import lru_cache
from importlib.resources import files
from pathlib import Path


@lru_cache(maxsize=1)
def assets() -> Path:
    root = Path(str(files(__package__).joinpath("data/thread")))
    manifest = json.loads((root / "manifest.json").read_text())
    for relative, digest in manifest["files"].items():
        path = (root / relative).resolve()
        path.relative_to(root.resolve())
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f"reference thread asset hash mismatch: {relative}")
    return root


def _build_plugin(source: Path) -> None:
    """Compile against this interpreter's installed native headers and library."""
    import mujoco

    package = Path(mujoco.__file__).parent
    libraries = list(
        package.glob(
            "libmujoco.*.dylib" if sys.platform == "darwin" else "libmujoco.so.*"
        )
    )
    if len(libraries) != 1:
        raise RuntimeError("the MuJoCo installation must provide one native library")
    compiler = shlex.split(os.environ.get("CXX", "c++"))
    if not compiler:
        raise RuntimeError(
            "CXX must name a C++17 compiler for the native thread plugin"
        )
    # Workers are POSIX processes. The mapped library remains loaded after this
    # private build directory is removed; no shared installation is overwritten.
    with tempfile.TemporaryDirectory(prefix="waddle-thread-") as temporary:
        library = Path(temporary) / "metric_thread.so"
        command = [
            *compiler,
            "-std=c++17",
            "-O2",
            "-dynamiclib" if sys.platform == "darwin" else "-shared",
            "-fPIC",
            str(source),
            "-I" + str(package / "include"),
            str(libraries[0]),
            "-o",
            str(library),
        ]
        try:
            result = subprocess.run(
                command, capture_output=True, text=True, timeout=60, check=False
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            raise RuntimeError(
                "MuJoCo bottle_cap requires a C++17 compiler (c++ or CXX)"
            ) from error
        if result.returncode:
            raise RuntimeError(
                "MuJoCo thread plugin compilation failed; use a C++17 compiler "
                f"and matching native headers: {result.stderr[-2000:]}"
            )
        mujoco.mj_loadPluginLibrary(str(library))


@lru_cache(maxsize=1)
def load_plugin() -> None:
    """Load the dimensional wrapper once per worker, only for a threaded cap."""
    _build_plugin(assets() / "metric_thread.cc")


def append_mjcf(root: ET.Element) -> None:
    load_plugin()
    template = ET.parse(assets() / "cap.xml").getroot()
    for mesh in template.findall("asset/mesh[@file]"):
        mesh.set("file", str(assets() / mesh.get("file")))
    root.append(template.find("extension"))
    root.find("asset").extend(template.find("asset"))
    root.find("worldbody").extend(template.find("worldbody"))
    root.find("option").set("sdf_initpoints", "40")
    root.find("option").set("sdf_iterations", "20")


def append_isaac(stage, path):
    """Reference the self-contained USD assembly; return its free cap prim."""
    from pxr import UsdGeom

    root = UsdGeom.Xform.Define(stage, path)
    root.GetPrim().GetReferences().AddReference(str(assets() / "cap.usdc"))
    return stage.GetPrimAtPath(f"{path}/cap")


def append_sapien(scene):
    """Load the same free-body assembly through native PhysX mesh cooking."""
    import numpy as np
    import sapien

    px = sapien.physx
    # 256 samples across the nominal 57.3 mm cap, with sparse subgrids.
    px.set_sdf_config(
        spacing=0.05730736 / 256, subgrid_size=6, num_threads_for_construction=4
    )
    material = px.PhysxMaterial(1.2, 1.0, 0.0)
    template = ET.parse(assets() / "cap.xml")

    def values(element, attribute):
        return np.fromstring(element.get(attribute), sep=" ")

    entities = []
    for name, mesh_name in (("bottle_thread", "bolt"), ("cap", "nut")):
        source = template.find(f".//body[@name='{name}']")
        path = str(assets() / f"{mesh_name}-physx.stl")
        builder = scene.create_actor_builder()
        builder.set_initial_pose(
            sapien.Pose(values(source, "pos"), values(source, "quat"))
        )
        color = tuple(values(source.find("geom[@type='sdf']"), "rgba"))
        builder.add_visual_from_file(path, material=color)
        if name == "cap":
            roof = source.find("geom[@name='cap_roof']")
            radius, half_length = values(roof, "size")
            # Native cylinders point along +X; the assembly's roof uses +Z.
            roof_pose = sapien.Pose(values(roof, "pos"), [2**-0.5, 0, 2**-0.5, 0])
            builder.add_cylinder_visual(
                roof_pose, radius, half_length, material=tuple(values(roof, "rgba"))
            )
            inertial = source.find("inertial")
            builder.set_mass_and_inertia(
                float(inertial.get("mass")),
                sapien.Pose(values(inertial, "pos"), values(inertial, "quat")),
                values(inertial, "diaginertia"),
            )
            entity = builder.build(name=name)
            body = entity.find_component_by_type(px.PhysxRigidDynamicComponent)
            roof_shape = px.PhysxCollisionShapeCylinder(radius, half_length, material)
            roof_shape.local_pose = roof_pose
            roof_shape.contact_offset = 0.001
            body.attach(roof_shape)
            body.set_solver_position_iterations(15)
            body.set_solver_velocity_iterations(1)
        else:
            entity = builder.build_static(name=name)
            body = entity.find_component_by_type(px.PhysxRigidStaticComponent)
        shape = px.PhysxCollisionShapeTriangleMesh(
            path, [1, 1, 1], material, sdf=name == "cap"
        )
        shape.contact_offset = 0.001
        body.attach(shape)
        entities.append(entity)
    return entities
