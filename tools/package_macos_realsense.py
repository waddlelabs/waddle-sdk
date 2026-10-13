"""Package a reviewed native RealSense build without opening hardware.

Run with CPython on arm64 macOS and wheel==0.46.3, delocate==0.13.0.
The source/build recipe and qualification limits live in docs/porting/camera.md.
This creates a new artifact directory; it never installs or publishes a package.
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import importlib.metadata
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import sysconfig
import tarfile
import tempfile
import zipfile
from pathlib import Path

REALSENSE_COMMIT = "34d6c778e1134d8505adcd56bb57acbe7598a459"
VERSION = "2.58.4.post2"
LIBUSB_SHA256 = "fea36f34f9156400209595e300840767ab1a385ede1dc7ee893015aea9c6dbaf"
MINIMUM_MACOS = "15.0"
ROOT = Path(__file__).resolve().parents[1]
PATCHES = [
    ROOT / "docs/porting/patches/librealsense-2.58.4-align-gil.patch",
    ROOT / "docs/porting/patches/librealsense-2.58.4-cleanup-ownership.patch",
]
PATCHED_FILES = [
    "third-party/rsutils/include/rsutils/deferred.h",
    "wrappers/python/pyrs_processing.cpp",
]


def run(*args: str | Path) -> str:
    return subprocess.check_output([str(arg) for arg in args], text=True).strip()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(reason)


def check_source(source: Path) -> None:
    run("git", "-C", source, "merge-base", "--is-ancestor", REALSENSE_COMMIT, "HEAD")
    require(
        run("git", "-C", source, "diff", "--name-only", REALSENSE_COMMIT).splitlines()
        == PATCHED_FILES,
        "RealSense source must contain exactly the reviewed changes",
    )
    require(
        not run("git", "-C", source, "ls-files", "--others", "--exclude-standard"),
        "RealSense source contains unreviewed files",
    )
    with tempfile.TemporaryDirectory(prefix="realsense-source-check-") as folder:
        expected = Path(folder)
        for name in PATCHED_FILES:
            target = expected / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(
                subprocess.check_output(
                    ["git", "-C", str(source), "show", f"{REALSENSE_COMMIT}:{name}"]
                )
            )
        for patch in PATCHES:
            run("patch", "--batch", "-p1", "-d", expected, "-i", patch)
        for name in PATCHED_FILES:
            require(
                (source / name).read_bytes() == (expected / name).read_bytes(),
                f"RealSense source does not match its reviewed patches: {name}",
            )


def cache_values(build: Path) -> dict[str, str]:
    values = {}
    for line in (build / "CMakeCache.txt").read_text().splitlines():
        if line and not line.startswith(("#", "//")) and "=" in line:
            field, value = line.split("=", 1)
            name, _kind = field.split(":", 1)
            values[name] = value
    return values


def check_binary(binary: Path) -> None:
    require(
        run("lipo", "-archs", binary) == "arm64",
        f"Unexpected architecture: {binary.name}",
    )
    minimum = re.findall(r"\bminos (\d+\.\d+(?:\.\d+)?)", run("otool", "-l", binary))
    require(
        minimum == [MINIMUM_MACOS],
        f"Unexpected deployment floor: {binary.name}: {minimum}",
    )


def check_usb(binary: Path) -> None:
    class Version(ctypes.Structure):
        _fields_ = [
            (name, ctypes.c_uint16) for name in ("major", "minor", "micro", "nano")
        ]

    library = ctypes.CDLL(str(binary))
    library.libusb_get_version.restype = ctypes.POINTER(Version)
    version = library.libusb_get_version().contents
    require(
        (version.major, version.minor, version.micro) == (1, 0, 30),
        "Shared USB runtime is not libusb 1.0.30",
    )
    require(
        "_pipe2" not in run("nm", "-u", binary),
        "Shared USB runtime depends on pipe2 beyond its deployment floor",
    )


def license_files(directory: Path):
    for path in sorted(directory.rglob("*")):
        if path.is_file() and path.name.lower() in {
            "license",
            "license.txt",
            "license.md",
            "copying",
            "copying.txt",
            "notice",
        }:
            yield path


def package(source: Path, build: Path, usb_archive: Path, output: Path) -> Path:
    require(
        platform.system() == "Darwin" and platform.machine() == "arm64",
        "Use native arm64 macOS",
    )
    require(sys.implementation.name == "cpython", "The binding requires CPython")
    for name, version in {"wheel": "0.46.3", "delocate": "0.13.0"}.items():
        require(importlib.metadata.version(name) == version, f"Use {name}=={version}")
    require(not output.exists(), "Refusing to overwrite an artifact directory")
    check_source(source)
    require(
        sha256(usb_archive) == LIBUSB_SHA256, "libusb source archive digest mismatch"
    )
    cache = cache_values(build)
    expected = {
        "CMAKE_OSX_ARCHITECTURES": "arm64",
        "CMAKE_OSX_DEPLOYMENT_TARGET": MINIMUM_MACOS,
        "CMAKE_BUILD_TYPE": "Release",
        "FORCE_RSUSB_BACKEND": "ON",
        "BUILD_SHARED_LIBS": "OFF",
        "BUILD_PYTHON_BINDINGS": "ON",
        "BUILD_LEGACY_PYBACKEND": "OFF",
        "BUILD_GLSL_EXTENSIONS": "OFF",
        "BUILD_WITH_CUDA": "OFF",
        "BUILD_WITH_DDS": "OFF",
        "USE_EXTERNAL_USB": "OFF",
    }
    for name, value in expected.items():
        require(cache.get(name) == value, f"Build configuration mismatch: {name}")
    require(
        Path(cache["CMAKE_HOME_DIRECTORY"]).resolve() == source,
        "Build belongs to a different source tree",
    )
    require(
        Path(cache["PYTHON_EXECUTABLE"]).resolve() == Path(sys.executable).resolve(),
        "Build belongs to another interpreter",
    )
    extension = "pyrealsense2" + sysconfig.get_config_var("EXT_SUFFIX")
    binary = build / "Release" / extension
    usb = Path(cache["LIBUSB_LIB"])
    require(usb.name == "libusb-1.0.0.dylib", "The runtime must use shared libusb")
    check_binary(binary)
    check_binary(usb)
    check_usb(usb)
    require(
        str(usb) in run("otool", "-L", binary),
        "The binding does not link the declared shared USB library",
    )
    py_tag = f"cp{sys.version_info.major}{sys.version_info.minor}"
    tag = f"{py_tag}-{py_tag}-macosx_15_0_arm64"
    dist_info = f"pyrealsense2-{VERSION}.dist-info"
    name = f"pyrealsense2-{VERSION}-{tag}.whl"
    with tempfile.TemporaryDirectory(
        prefix="realsense-package-", dir=build
    ) as temporary:
        stage = Path(temporary)
        tree = stage / "tree"
        module = tree / "pyrealsense2"
        metadata = tree / dist_info
        module.mkdir(parents=True)
        metadata.mkdir()
        shutil.copy2(
            source / "wrappers/python/pyrealsense2/__init__.py", module / "__init__.py"
        )
        shutil.copy2(binary, module / extension)
        licenses = metadata / "licenses"
        license_names = []
        for prefix, directory in [
            ("librealsense", source),
            ("build-dependencies", build / "third-party"),
        ]:
            for path in license_files(directory):
                relative = Path(prefix) / path.relative_to(directory)
                target = licenses / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, target)
                license_names.append("licenses/" + relative.as_posix())
        with tarfile.open(usb_archive, "r:bz2") as archive:
            member = archive.extractfile("libusb-1.0.30/COPYING")
            require(member is not None, "libusb license is missing")
            target = licenses / "libusb/COPYING"
            target.parent.mkdir(parents=True)
            target.write_bytes(member.read())
            license_names.append("licenses/libusb/COPYING")
        sources = metadata / "source"
        sources.mkdir()
        shutil.copy2(usb_archive, sources / "libusb-1.0.30.tar.bz2")
        for patch in PATCHES:
            shutil.copy2(patch, sources / patch.name)
        shutil.copy2(Path(__file__), sources / Path(__file__).name)
        manifest = {
            "distribution": {"name": "pyrealsense2", "version": VERSION, "tag": tag},
            "native_version": "2.58.4",
            "source": {
                "url": "https://github.com/realsenseai/librealsense",
                "commit": REALSENSE_COMMIT,
                "derived_head": run("git", "-C", source, "rev-parse", "HEAD"),
                "patches": {patch.name: sha256(patch) for patch in PATCHES},
            },
            "usb": {
                "version": "1.0.30",
                "source_url": "https://github.com/libusb/libusb/releases/download/v1.0.30/libusb-1.0.30.tar.bz2",
                "source_sha256": LIBUSB_SHA256,
                "linkage": "shared",
                "configure_cache": {"ac_cv_func_pipe2": "no"},
                "input_binary_sha256": sha256(usb),
            },
            "build": {
                **expected,
                "interpreter": platform.python_version(),
                "host_macos": platform.mac_ver()[0],
                "input_binding_sha256": sha256(binary),
                "packaging": {"wheel": "0.46.3", "delocate": "0.13.0"},
            },
            "qualification": {
                "hardware": False,
                "camera_access": False,
                "control_load": False,
            },
        }
        (module / "native_build.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n"
        )
        (metadata / "METADATA").write_text(
            "Metadata-Version: 2.4\nName: pyrealsense2\n"
            f"Version: {VERSION}\nSummary: Reviewed native macOS RealSense RSUSB candidate\n"
            f"Requires-Python: >={sys.version_info.major}.{sys.version_info.minor},<{sys.version_info.major}.{sys.version_info.minor + 1}\n"
            + "".join(f"License-File: {license}\n" for license in sorted(license_names))
            + "\nDerived from librealsense 2.58.4 with the bundled cleanup ownership and alignment GIL patches.\n"
            "Shared libusb source and notices are included. Hardware acceptance remains separate.\n"
        )
        (metadata / "WHEEL").write_text(
            f"Wheel-Version: 1.0\nGenerator: package_macos_realsense\nRoot-Is-Purelib: false\nTag: {tag}\n"
        )
        from wheel.wheelfile import WheelFile

        wheel = stage / name
        with WheelFile(str(wheel), "w") as archive:
            for path in sorted(tree.rglob("*")):
                if path.is_file():
                    archive.write(str(path), path.relative_to(tree).as_posix())
        environment = os.environ.copy()
        environment["MACOSX_DEPLOYMENT_TARGET"] = MINIMUM_MACOS
        subprocess.run(
            [
                sys.executable,
                "-m",
                "delocate.cmd.delocate_wheel",
                "--require-archs",
                "arm64",
                str(wheel),
            ],
            env=environment,
            check=True,
        )
        unpacked = stage / "verified"
        with zipfile.ZipFile(wheel) as archive:
            archive.extractall(unpacked)
        bundled_usb = list(unpacked.rglob("libusb*.dylib"))
        require(
            len(bundled_usb) == 1, "The wheel must bundle exactly one shared libusb"
        )
        check_binary(bundled_usb[0])
        packaged_binding = unpacked / "pyrealsense2" / extension
        check_binary(packaged_binding)
        for native in [packaged_binding, bundled_usb[0]]:
            for line in run("otool", "-L", native).splitlines()[2:]:
                dependency = line.strip().split(" (", 1)[0]
                require(
                    dependency.startswith(
                        ("@loader_path/", "/System/Library/", "/usr/lib/")
                    ),
                    f"Nonportable native dependency: {dependency}",
                )
        output.mkdir(parents=True)
        result = output / name
        shutil.copy2(wheel, result)
        (output / "artifact.json").write_text(
            json.dumps(
                {"wheel": name, "sha256": sha256(result), "build": manifest},
                indent=2,
                sort_keys=True,
            )
            + "\n"
        )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "build", "libusb-archive", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    args = parser.parse_args()
    try:
        result = package(
            *(
                path.resolve()
                for path in [args.source, args.build, args.libusb_archive, args.output]
            )
        )
    except (
        ValueError,
        OSError,
        subprocess.CalledProcessError,
        importlib.metadata.PackageNotFoundError,
    ) as error:
        parser.exit(1, f"Native RealSense packaging refused: {error}\n")
    print(result)


if __name__ == "__main__":
    main()
