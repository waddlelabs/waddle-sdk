"""Deterministic clean-install coverage for every published SDK extra.

The test wheelhouse is entirely local: the SDK wheel contains this checkout's
Python package and already-built extension, while tiny dependency wheels make
pip exercise the real optional-dependency metadata without a package index or
camera hardware.  The vendor-module stubs raise on import, making successful
``waddle_sdk`` and adapter-module imports a proof that camera SDK loading stays
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
import waddle_sdk

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python 3.10 only
    import tomli as tomllib  # type: ignore[no-redef]


SDK = Path(__file__).resolve().parents[1]
PACKAGE = SDK / "python" / "waddle_sdk"
VERSION = waddle_sdk.__version__
PYTHON_311_ONLY = frozenset(
    {"alicia-m-sdk", "alicia-d-sdk", "synriard", "synria-robocore"}
)


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
        for dependency in dependencies:
            requirement, separator, marker = dependency.partition(";")
            condition = f'extra == "{extra}"'
            if separator:
                condition = f"({marker.strip()}) and {condition}"
            metadata.append(f"Requires-Dist: {requirement.strip()} ; {condition}")
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
    if not any(name.startswith("waddle_sdk/_core.") for name in files):
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
        files={
            "numpy/__init__.py": b'__version__ = "2.0.0"\n',
            "numpy/typing.py": b"NDArray = object\n",
        },
    )
    _build_wheel(
        directory,
        "PyYAML",
        "6.0.0",
        files={"yaml/__init__.py": b""},
    )
    _build_wheel(
        directory,
        "jsonschema",
        "4.22.0",
        files={"jsonschema/__init__.py": b""},
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
        "opencv-python-headless",
        "4.8.0",
        files={
            "cv2/__init__.py": b'raise RuntimeError("cv2 must remain lazily imported")\n'
        },
    )
    _build_wheel(
        directory,
        "mujoco",
        "3.1.0",
        files={
            "mujoco/__init__.py": (
                b'raise RuntimeError("mujoco must remain lazily imported")\n'
            )
        },
    )
    _build_wheel(
        directory,
        "xarm-python-sdk",
        "1.16.0",
        files={
            "xarm/__init__.py": b"",
            "xarm/wrapper.py": (
                b'raise RuntimeError("xarm must remain lazily imported")\n'
            ),
        },
    )
    _build_wheel(
        directory,
        "alicia-m-sdk",
        "1.1.1rc2",
        files={
            "alicia_m_sdk/__init__.py": (
                b'raise RuntimeError("alicia_m_sdk must remain lazily imported")\n'
            )
        },
    )
    _build_wheel(
        directory,
        "alicia-d-sdk",
        "6.1.0",
        files={
            "alicia_d_sdk/__init__.py": (
                b'raise RuntimeError("alicia_d_sdk must remain lazily imported")\n'
            )
        },
    )
    _build_wheel(
        directory,
        "synriard",
        "1.2.2",
        files={"synriard/__init__.py": b""},
    )
    _build_wheel(
        directory,
        "synria-robocore",
        "2.5.0rc4",
        files={"synria_robocore/__init__.py": b""},
    )
    _build_wheel(
        directory,
        "waddle-sdk-media",
        VERSION,
        requirements=("numpy>=1.24",),
        files={
            "waddle_media/__init__.py": b"",
            "waddle_media/_core.py": (
                f"__version__ = {VERSION!r}\n"
                "BINDING_API_VERSION = 3\n"
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
    ("usb", "usb", frozenset({"opencv-python-headless"})),
    ("mujoco", "mujoco", frozenset({"mujoco"})),
    ("xarm", "xarm", frozenset({"xarm-python-sdk"})),
    (
        "alicia",
        "alicia",
        frozenset({"alicia-m-sdk", "synria-robocore", "synriard"}),
    ),
    ("alicia-d", "alicia-d", frozenset({"alicia-d-sdk", "synriard"})),
    (
        "robots",
        "robots",
        frozenset(
            {
                "xarm-python-sdk",
                "alicia-m-sdk",
                "alicia-d-sdk",
                "synria-robocore",
                "synriard",
            }
        ),
    ),
    (
        "cameras",
        "cameras",
        frozenset({"opencv-python-headless", "pyorbbecsdk2", "pyrealsense2"}),
    ),
    ("media", "media", frozenset({"waddle-sdk-media"})),
    (
        "media-orbbec",
        "media,orbbec",
        frozenset({"waddle-sdk-media", "pyorbbecsdk2"}),
    ),
    (
        "media-realsense",
        "media,realsense",
        frozenset({"waddle-sdk-media", "pyrealsense2"}),
    ),
    (
        "media-usb",
        "media,usb",
        frozenset({"opencv-python-headless", "waddle-sdk-media"}),
    ),
    (
        "media-cameras",
        "media,cameras",
        frozenset(
            {
                "opencv-python-headless",
                "waddle-sdk-media",
                "pyorbbecsdk2",
                "pyrealsense2",
            }
        ),
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

    probe = r"""
import importlib.metadata
import json
import sys
import tempfile
from pathlib import Path

import waddle_sdk
from waddle_sdk.agent_skills import bundled_skills, export_skill
from waddle_sdk.cameras import mock, orbbec, realsense, usb
from waddle_sdk.robots import alicia, alicia_d, mujoco
from waddle_sdk.robots import xarm as xarm_adapter

names = {
    dist.metadata["Name"].lower()
    for dist in importlib.metadata.distributions()
    if dist.metadata["Name"]
}
with tempfile.TemporaryDirectory() as directory:
    exported = export_skill("waddle-sdk-contracts", directory)
    export_ok = (
        exported.name == "waddle-sdk-contracts"
        and (Path(exported) / "SKILL.md").read_text().startswith("---\n")
    )
print(json.dumps({
    "names": sorted(names),
    "core": waddle_sdk._native.core.__name__,
    "features": sorted(waddle_sdk._native.FEATURES),
    "requirements": importlib.metadata.requires("waddle-sdk"),
    "skills": [skill.name for skill in bundled_skills()],
    "skill_export_ok": export_ok,
    "vendor_modules": sorted(
        name for name in sys.modules
        if name in {"alicia_d_sdk", "alicia_m_sdk", "cv2", "mujoco", "pyorbbecsdk", "pyrealsense2", "xarm"}
    ),
}))
"""
    completed = subprocess.run(
        [str(python), "-I", "-c", probe],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    result = json.loads(completed.stdout)
    effective_optional = optional
    if sys.version_info < (3, 11):
        effective_optional -= PYTHON_311_ONLY

    expected = {"waddle-sdk", "numpy", *effective_optional}
    assert expected <= set(result["names"])
    assert not (
        {
            "opencv-python-headless",
            "mujoco",
            "pyorbbecsdk2",
            "pyrealsense2",
            "waddle-sdk-media",
            "waddle-sdk-teleop",
            "xarm-python-sdk",
            "alicia-m-sdk",
            "alicia-d-sdk",
            "synriard",
            "synria-robocore",
        }
        - effective_optional
    ) & set(result["names"])
    assert result["vendor_modules"] == []
    assert result["requirements"]
    assert result["skills"] == ["port-waddle-hardware", "waddle-sdk-contracts"]
    assert result["skill_export_ok"] is True
    if "waddle-sdk-media" in effective_optional:
        assert result["core"] == "waddle_media._core"
        assert result["features"] == ["grpc", "livekit"]
    else:
        assert result["core"] == "waddle_sdk._core"
