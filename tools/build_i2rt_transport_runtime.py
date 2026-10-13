"""Build the reviewed I2RT candidate from immutable upstream bytes and SDK patches.

Does not modify a checkout, installed vendor, hardware or installation metadata.
The output directory must be new. Source and wheel files are verified against
the SDK's reviewed manifest; archive hashes are reported for each actual build.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
import subprocess
import zipfile
from pathlib import Path, PurePosixPath

_ROOT = Path(__file__).resolve().parents[1]
_DATA = _ROOT / "sdk/python/waddle_sdk/robots/yam_data"
_PATCH = _ROOT / "docs/porting/patches/i2rt-570ef666-transport-injection.patch"


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _verify(actual, expected, scope):
    if set(actual) != set(expected):
        raise ValueError(f"{scope} file inventory differs from the reviewed source")
    for name, digest in expected.items():
        if _sha(actual[name]) != digest:
            raise ValueError(f"{scope} file differs from the reviewed source: {name}")


def build(source_repo: Path, output: Path, uv: str):
    """Build only the source/metadata identity reviewed by the SDK."""
    manifest = json.loads((_DATA / "i2rt-transport-runtime.json").read_text())
    if _sha(_PATCH.read_bytes()) != manifest["transport_patch_sha256"]:
        raise ValueError("The SDK transport patch differs from its reviewed identity")
    env = dict(os.environ)
    for name in ("GIT_DIR", "GIT_WORK_TREE", "GIT_INDEX_FILE", "GIT_COMMON_DIR"):
        env.pop(name, None)
    archive = subprocess.run(
        [
            "git",
            "-C",
            str(source_repo.resolve()),
            "archive",
            "--format=zip",
            manifest["upstream_commit"],
        ],
        env=env,
        check=True,
        capture_output=True,
    ).stdout
    output.mkdir(parents=True)
    stage = output / "source"
    stage.mkdir()
    with zipfile.ZipFile(io.BytesIO(archive)) as source:
        for entry in source.infolist():
            name = PurePosixPath(entry.filename)
            if (
                name.is_absolute()
                or ".." in name.parts
                or (entry.external_attr >> 16) & 0o170000 == 0o120000
            ):
                raise ValueError("The pinned source archive contains an unsafe path")
            target = stage.joinpath(*name.parts)
            if entry.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                with target.open("xb") as handle:
                    handle.write(source.read(entry))
    # Prevent discovering a parent SDK repository. Plain git apply works on this
    # isolated source directory without creating or modifying a Git repository.
    env["GIT_CEILING_DIRECTORIES"] = str(output.resolve())
    subprocess.run(
        ["git", "apply", "--check", str(_PATCH)], cwd=stage, env=env, check=True
    )
    subprocess.run(["git", "apply", str(_PATCH)], cwd=stage, env=env, check=True)
    pyproject = stage / "pyproject.toml"
    text = pyproject.read_text()
    replacements = (
        ('version = "1.1.2"', f'version = "{manifest["version"]}"'),
        (
            '"numpy==2.2.6"',
            (
                "\"numpy==2.2.6; sys_platform != 'darwin'\",\n"
                "    \"numpy>=2.3.1; sys_platform == 'darwin'\""
            ),
        ),
    )
    for before, after in replacements:
        if text.count(before) != 1:
            raise ValueError("The pinned vendor package metadata changed")
        text = text.replace(before, after)
    pyproject.write_text(text)
    (stage / "i2rt/_transport_provenance.json").write_bytes(
        (_DATA / "i2rt-transport-provenance.json").read_bytes()
    )
    source_files = {
        path.relative_to(stage).as_posix(): path.read_bytes()
        for path in (stage / "i2rt").rglob("*")
        if path.is_file()
    }
    _verify(source_files, manifest["package_files"], "Patched I2RT source")
    wheels = output / "wheels"
    build_constraints = output / "build-constraints.txt"
    build_constraints.write_text("flit_core==4.1.0\n")
    env["SOURCE_DATE_EPOCH"] = subprocess.run(
        [
            "git",
            "-C",
            str(source_repo.resolve()),
            "show",
            "-s",
            "--format=%ct",
            manifest["upstream_commit"],
        ],
        env=env,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    subprocess.run(
        [
            uv,
            "build",
            "--wheel",
            "--build-constraints",
            str(build_constraints.resolve()),
            "--out-dir",
            str(wheels.resolve()),
            str(stage.resolve()),
        ],
        env=env,
        check=True,
    )
    candidates = list(wheels.glob("*.whl"))
    if len(candidates) != 1:
        raise ValueError("The candidate build must produce exactly one wheel")
    wheel = candidates[0]
    with zipfile.ZipFile(wheel) as built:
        _verify(
            {
                name: built.read(name)
                for name in built.namelist()
                if name.startswith("i2rt/") and not name.endswith("/")
            },
            manifest["package_files"],
            "Built I2RT package",
        )
        _verify(
            {name: built.read(name) for name in manifest["metadata"]},
            manifest["metadata"],
            "Built I2RT metadata/license",
        )
    evidence = {
        "version": manifest["version"],
        "upstream_commit": manifest["upstream_commit"],
        "transport_patch_sha256": manifest["transport_patch_sha256"],
        "wheel": str(wheel.resolve()),
        "wheel_sha256": _sha(wheel.read_bytes()),
        "verified_package_files": len(source_files),
        "qualification": manifest["qualification"],
    }
    with (output / "build-evidence.json").open("x") as handle:
        json.dump(evidence, handle, indent=2)
        handle.write("\n")
    return evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-repo", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--uv", default="uv")
    args = parser.parse_args()
    print(json.dumps(build(args.source_repo, args.output, args.uv), indent=2))


if __name__ == "__main__":
    main()
