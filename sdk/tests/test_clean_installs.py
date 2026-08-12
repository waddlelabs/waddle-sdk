"""Deterministic clean-install coverage for every published SDK extra.

The test wheelhouse is entirely local: the SDK wheel contains this checkout's
Python package and already-built extension, while tiny dependency wheels make
pip exercise the real optional-dependency metadata without a package index or
camera hardware.  The vendor-module stubs raise on import, making successful
``waddle`` and adapter-module imports a proof that camera SDK loading stays
lazy.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import venv
import zipfile
from collections.abc import Iterable, Mapping
from pathlib import Path

import pytest
import waddle

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python 3.10 only
    import tomli as tomllib  # type: ignore[no-redef]


SDK = Path(__file__).resolve().parents[1]
PACKAGE = SDK / "python" / "waddle"
VERSION = waddle.__version__


def _wheel_name(distribution: str) -> str:
    return distribution.replace("-", "_").replace(".", "_")


def _build_wheel(
    wheelhouse: Path,
    distribution: str,
    version: str,
    *,
    files: Mapping[str, bytes],
    requirements: Iterable[str] = (),
    extras: Mapping[str, Iterable[str]] | None = None,
) -> Path:
    normalized = _wheel_name(distribution)
    dist_info = f"{normalized}-{version}.dist-info"
    metadata = [
        "Metadata-Version: 2.4",
        f"Name: {distribution}",
        f"Version: {version}",
    ]
    metadata.extend(f"Requires-Dist: {requirement}" for requirement in requirements)
    for extra, dependencies in (extras or {}).items():
        metadata.append(f"Provides-Extra: {extra}")
        metadata.extend(
            f'Requires-Dist: {dependency} ; extra == "{extra}"'
            for dependency in dependencies
        )
    wheel = wheelhouse / f"{normalized}-{version}-py3-none-any.whl"
    with zipfile.ZipFile(wheel, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in files.items():
            archive.writestr(name, data)
        archive.writestr(f"{dist_info}/METADATA", "\n".join(metadata) + "\n")
        archive.writestr(
            f"{dist_info}/WHEEL",
            "Wheel-Version: 1.0\n"
            "Generator: waddle-sdk clean-install test\n"
            "Root-Is-Purelib: true\n"
            "Tag: py3-none-any\n",
        )
        archive.writestr(f"{dist_info}/RECORD", "")
    return wheel


def _package_files() -> dict[str, bytes]:
    files: dict[str, bytes] = {}
    for path in PACKAGE.rglob("*"):
        if not path.is_file() or "__pycache__" in path.parts:
            continue
        files[path.relative_to(SDK / "python").as_posix()] = path.read_bytes()
    if not any(name.startswith("waddle/_core.") for name in files):
        raise AssertionError(
            "the clean-install test needs the extension built by uv sync or "
            "maturin develop"
        )
    return files


@pytest.fixture(scope="module")
def wheelhouse(tmp_path_factory: pytest.TempPathFactory) -> Path:
    directory = tmp_path_factory.mktemp("clean-install-wheelhouse")
    with (SDK / "pyproject.toml").open("rb") as stream:
        project = tomllib.load(stream)["project"]
    extras = project["optional-dependencies"]

    _build_wheel(
        directory,
        "waddle-sdk",
        VERSION,
        files=_package_files(),
        requirements=project["dependencies"],
        extras=extras,
    )
    _build_wheel(
        directory,
        "numpy",
        "2.0.0",
        files={"numpy/__init__.py": b'__version__ = "2.0.0"\n'},
    )
    _build_wheel(
        directory,
        "pyorbbecsdk2",
        "0.0.0",
        files={
            "pyorbbecsdk.py": (
                b'raise RuntimeError("pyorbbecsdk must remain lazily imported")\n'
            )
        },
    )
    _build_wheel(
        directory,
        "pyrealsense2",
        "0.0.0",
        files={
            "pyrealsense2.py": (
                b'raise RuntimeError("pyrealsense2 must remain lazily imported")\n'
            )
        },
    )
    _build_wheel(
        directory,
        "waddle-sdk-teleop",
        VERSION,
        requirements=("numpy>=1.24",),
        files={
            "waddle_teleop/__init__.py": b"",
            "waddle_teleop/_core.py": (
                f'__version__ = {VERSION!r}\n'
                'FEATURES = frozenset({"grpc", "livekit"})\n'
                "class SessionStamp:\n"
                "    pass\n"
            ).encode(),
        },
    )
    return directory


CASES = (
    ("base", "", frozenset()),
    ("orbbec", "orbbec", frozenset({"pyorbbecsdk2"})),
    ("realsense", "realsense", frozenset({"pyrealsense2"})),
    ("cameras", "cameras", frozenset({"pyorbbecsdk2", "pyrealsense2"})),
    ("teleop", "teleop", frozenset({"waddle-sdk-teleop"})),
    (
        "teleop-orbbec",
        "teleop,orbbec",
        frozenset({"waddle-sdk-teleop", "pyorbbecsdk2"}),
    ),
    (
        "teleop-realsense",
        "teleop,realsense",
        frozenset({"waddle-sdk-teleop", "pyrealsense2"}),
    ),
    (
        "teleop-cameras",
        "teleop,cameras",
        frozenset({"waddle-sdk-teleop", "pyorbbecsdk2", "pyrealsense2"}),
    ),
)


@pytest.mark.parametrize(("case", "selected", "optional"), CASES)
def test_clean_install_extra_matrix(
    tmp_path: Path,
    wheelhouse: Path,
    case: str,
    selected: str,
    optional: frozenset[str],
):
    environment = tmp_path / case
    venv.EnvBuilder(with_pip=True, clear=True).create(environment)
    python = environment / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    requirement = f"waddle-sdk=={VERSION}"
    if selected:
        requirement = f"waddle-sdk[{selected}]=={VERSION}"
    subprocess.run(
        [
            str(python),
            "-m",
            "pip",
            "install",
            "--disable-pip-version-check",
            "--no-index",
            "--find-links",
            str(wheelhouse),
            requirement,
        ],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    probe = r'''
import importlib.metadata
import json
import sys

import waddle
from waddle.cameras import orbbec, realsense

names = {
    dist.metadata["Name"].lower()
    for dist in importlib.metadata.distributions()
    if dist.metadata["Name"]
}
print(json.dumps({
    "names": sorted(names),
    "core": waddle._native.core.__name__,
    "features": sorted(waddle._native.FEATURES),
    "requirements": importlib.metadata.requires("waddle-sdk"),
    "vendor_modules": sorted(
        name for name in sys.modules
        if name in {"pyorbbecsdk", "pyrealsense2"}
    ),
}))
'''
    completed = subprocess.run(
        [str(python), "-I", "-c", probe],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    result = json.loads(completed.stdout)
    expected = {"waddle-sdk", "numpy", *optional}
    assert expected <= set(result["names"])
    assert not ({"pyorbbecsdk2", "pyrealsense2", "waddle-sdk-teleop"} - optional) & set(
        result["names"]
    )
    assert result["vendor_modules"] == []
    assert result["requirements"]
    if "waddle-sdk-teleop" in optional:
        assert result["core"] == "waddle_teleop._core"
        assert result["features"] == ["grpc", "livekit"]
    else:
        assert result["core"] == "waddle._core"
