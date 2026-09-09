"""Native, freely removable cap using MuJoCo's first-party thread geometry.

Only dimensional scaling lives in the small compiled plugin. MuJoCo supplies
the SDFs and contact solver; no callback applies forces or changes constraints.
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
