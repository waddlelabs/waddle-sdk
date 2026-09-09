"""Rebuild native PhysX thread meshes using standard offline geometry tools.

Linux, C++17, MuJoCo 3.11.0, scikit-image 0.26.0, manifold3d 3.5.3,
pymeshlab 2025.7.post1, trimesh 5.1.0 and numpy are required. Use --output
to inspect generated meshes before replacing packaged assets. No contact
solver or thread formula is implemented here: sampling calls MuJoCo's SDFs.
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import os
import shlex
import subprocess
import tempfile
from importlib.metadata import version
from pathlib import Path

import manifold3d
import mujoco
import numpy as np
import pymeshlab
import trimesh
from skimage.measure import marching_cubes
from waddle_sdk.simulators.thread import load_plugin

DATA = (
    Path(__file__).resolve().parents[1] / "sdk/python/waddle_sdk/simulators/data/thread"
)
SAMPLER = """
#include <mujoco/mujoco.h>
extern "C" int sample_grid(const char* name, const double* attributes,
                          const double* origin, const int* shape,
                          double spacing, float* output) {
  const mjpPlugin* plugin = mjp_getPlugin(name, nullptr);
  if (!plugin || !plugin->sdf_staticdistance) return -1;
  size_t index = 0;
  for (int x = 0; x < shape[0]; ++x)
    for (int y = 0; y < shape[1]; ++y)
      for (int z = 0; z < shape[2]; ++z) {
        const double point[] = {origin[0] + x * spacing,
                                origin[1] + y * spacing,
                                origin[2] + z * spacing};
        output[index++] = plugin->sdf_staticdistance(point, attributes);
      }
  return 0;
}
"""


def generate(destination, sample):
    manifest = json.loads((DATA / "manifest.json").read_text())
    provenance = {
        "source": "MuJoCo 3.11.0 first-party SDFs; uniform scale 0.05",
        "versions": {
            name: version(name)
            for name in (
                "mujoco",
                "numpy",
                "scikit-image",
                "manifold3d",
                "pymeshlab",
                "trimesh",
            )
        },
        "sampling_spacing_m": 0.00015,
        "simplification_tolerance_m": 0.000025,
        "cleanup": "MeshLab T-vertex Edge Flip, then Edge Collapse threshold 1e6",
        "native_sdf": {"spacing_m": 0.05730736 / 256, "subgrid_size": 6},
        "geometry": {},
    }
    spacing = provenance["sampling_spacing_m"]
    for name, radius, zmin in (("nut", 0.26, -0.026), ("bolt", 0.255, -0.051)):
        origin = np.array([-0.026, -0.030, zmin], dtype=np.float64)
        shape = (
            np.ceil((np.array([0.026, 0.030, 0.001]) - origin) / spacing).astype(
                np.int32
            )
            + 1
        )
        grid = np.empty(shape, dtype=np.float32)
        code = sample(
            f"waddle_sdk.sdf.metric_{name}".encode(),
            np.array([radius, 0.05], dtype=np.float64),
            origin,
            shape,
            spacing,
            grid,
        )
        if code:
            raise RuntimeError(f"native SDF sampling failed: {name}")
        assert all(
            np.min(face) > 0
            for face in (
                grid[0],
                grid[-1],
                grid[:, 0],
                grid[:, -1],
                grid[:, :, 0],
                grid[:, :, -1],
            )
        )
        vertices, faces, _, _ = marching_cubes(
            grid,
            level=0,
            spacing=(spacing,) * 3,
            method="lewiner",
            gradient_direction="ascent",
            allow_degenerate=False,
        )
        del grid
        fine = trimesh.Trimesh(
            vertices.astype(np.float64) + origin, faces, process=False
        )
        if fine.volume < 0:
            fine.invert()
        native = manifold3d.Manifold(
            manifold3d.Mesh64(
                np.ascontiguousarray(fine.vertices, dtype=np.float64),
                np.ascontiguousarray(fine.faces, dtype=np.uint64),
            )
        )
        assert native.status() == manifold3d.Error.NoError
        reduced = native.simplify(provenance["simplification_tolerance_m"])
        assert reduced.status() == manifold3d.Error.NoError
        raw = reduced.to_mesh64()
        meshes = pymeshlab.MeshSet()
        meshes.add_mesh(
            pymeshlab.Mesh(
                raw.vert_properties[:, :3].astype(np.float32).astype(np.float64),
                raw.tri_verts.astype(np.int32),
            )
        )
        meshes.meshing_remove_t_vertices(method="Edge Flip")
        meshes.meshing_remove_t_vertices(method="Edge Collapse", threshold=1e6)
        meshes.meshing_remove_unreferenced_vertices()
        raw = meshes.current_mesh()
        mesh = trimesh.Trimesh(
            raw.vertex_matrix().astype(np.float32), raw.face_matrix(), process=False
        )
        assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume > 0
        assert mesh.unique_faces().all() and (mesh.area_faces > 0).all()
        path = destination / f"{name}-physx.stl"
        path.write_bytes(mesh.export(file_type="stl"))
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        manifest["files"][path.name] = digest
        provenance["geometry"][name] = {
            "radius": radius,
            "vertices": len(mesh.vertices),
            "faces": len(mesh.faces),
            "sha256": digest,
            "watertight": True,
            "winding_consistent": True,
            "zero_area_faces": 0,
            "duplicate_faces": 0,
        }
        print(name, provenance["geometry"][name], flush=True)
    manifest["physx_geometry"] = provenance
    if destination == DATA.resolve():
        (destination / "manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n"
        )
    else:
        (destination / "physx-manifest.json").write_text(
            json.dumps(provenance, indent=2) + "\n"
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DATA)
    destination = parser.parse_args().output.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    if mujoco.__version__ != "3.11.0":
        raise RuntimeError("reference geometry generation requires MuJoCo 3.11.0")
    load_plugin()
    package = Path(mujoco.__file__).parent
    with tempfile.TemporaryDirectory(prefix="waddle-thread-mesh-") as temporary:
        source = Path(temporary) / "sample.cc"
        source.write_text(SAMPLER)
        library = Path(temporary) / "sample.so"
        subprocess.run(
            [
                *shlex.split(os.environ.get("CXX", "c++")),
                "-std=c++17",
                "-O2",
                "-shared",
                "-fPIC",
                str(source),
                "-I" + str(package / "include"),
                str(next(package.glob("libmujoco.so.*"))),
                "-o",
                str(library),
            ],
            check=True,
        )
        sampler = ctypes.CDLL(str(library))
        double_array = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
        sampler.sample_grid.argtypes = [
            ctypes.c_char_p,
            double_array,
            double_array,
            np.ctypeslib.ndpointer(dtype=np.int32, flags="C_CONTIGUOUS"),
            ctypes.c_double,
            np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS"),
        ]
        generate(destination, sampler.sample_grid)


if __name__ == "__main__":
    main()
