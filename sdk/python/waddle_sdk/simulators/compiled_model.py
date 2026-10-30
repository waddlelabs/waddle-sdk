"""Optional private cache of compiled MuJoCo models, never runtime state.

Callers supply a complete, confined, verified source inventory. Every call hashes
those bytes again. A hit loads a fresh native model; no mutable model, MjData,
camera sample, ownership or outcome is shared. Ordinary native source errors
remain authoritative when disposable cache evidence is absent or invalid.
"""

from __future__ import annotations

import hashlib
import json
import os
import platform
import tempfile
import time
from pathlib import Path, PurePosixPath

CACHE_ENV = "WADDLE_SIMULATION_MODEL_CACHE"
_MAX_BYTES = 256 * 1024 * 1024
_MAX_ENTRIES = 16


def _hash(data):
    return hashlib.sha256(data).hexdigest()


def load_model(model_name, contents, *, cache=None, timings=None):
    """Compile complete verified VFS inputs or load a verified private binary.

    ``cache`` is an absolute private directory, or defaults to
    ``WADDLE_SIMULATION_MODEL_CACHE``. Omission keeps compilation uncached. The
    optional timings dict reports actual compilation and cache load separately.
    The directory must stay outside untrusted code/input mounts. Cache failure
    degrades to real compilation; source/model failures are never suppressed.
    """
    import mujoco as mj

    timings = timings if timings is not None else {}
    inputs = dict(contents)
    for name, data in inputs.items():
        if not isinstance(name, str) or not isinstance(data, bytes):
            raise TypeError(
                "simulation.model_inputs: expected names and immutable bytes"
            )
        path = PurePosixPath(name)
        if path.is_absolute() or ".." in path.parts or str(path) != name:
            raise ValueError(
                "simulation.model_inputs: source names must be confined relative paths"
            )
    if model_name not in inputs:
        raise ValueError(
            "simulation.model_inputs: root XML is absent from the inventory"
        )
    # Portable copies may add a directory prefix or rename just the root XML.
    # Normalize both while keeping every relative asset reference intact.
    parent = str(PurePosixPath(model_name).parent)
    if parent != "." and all(name.startswith(parent + "/") for name in inputs):
        prefix = len(parent) + 1
        inputs = {name[prefix:]: data for name, data in inputs.items()}
        model_name = model_name[prefix:]
    canonical = "__waddle_root__.xml"
    if canonical not in inputs:
        inputs[canonical] = inputs.pop(model_name)
        model_name = canonical
    hashes = {name: _hash(data) for name, data in inputs.items()}
    # Namespace VFS assets by bytes, independent of the separately parsed XML.
    namespace = _hash(
        json.dumps(
            {k: v for k, v in hashes.items() if k != model_name}, sort_keys=True
        ).encode()
    )
    assets = {f"{namespace}/{name}": data for name, data in inputs.items()}
    identity = {
        "files": hashes,
        "root": model_name,
        "mujoco": mj.__version__,
        "platform": [platform.system(), platform.machine()],
    }
    key = _hash(json.dumps(identity, sort_keys=True).encode())

    def compile():
        began = time.monotonic()
        result = mj.MjModel.from_xml_path(f"{namespace}/{model_name}", assets=assets)
        timings.update(cache_hit=False, native_compile_s=time.monotonic() - began)
        return result

    selected = cache if cache is not None else os.environ.get(CACHE_ENV)
    if not selected:
        return compile()
    root = Path(selected)
    if not root.is_absolute():
        raise ValueError("simulation.model_cache: select an absolute private directory")
    try:
        import fcntl

        root.mkdir(parents=True, exist_ok=True, mode=0o700)
        if root.is_symlink():
            raise OSError("simulation.model_cache: cache directory cannot be a symlink")
        fd = os.open(root / ".lock", os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    except (OSError, ImportError):
        return compile()
    with os.fdopen(fd, "a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        binary, marker = root / (key + ".mjb"), root / (key + ".json")
        try:
            if binary.is_symlink() or marker.is_symlink():
                raise ValueError("simulation.model_cache: invalid cache member")
            record = json.loads(marker.read_text())
            if (
                record["key"] != key
                or binary.stat().st_size > _MAX_BYTES
                or _hash(binary.read_bytes()) != record["sha256"]
            ):
                raise ValueError("simulation.model_cache: compiled bytes changed")
            began = time.monotonic()
            model = mj.MjModel.from_binary_path(str(binary))
            timings.update(
                cache_hit=True,
                native_compile_s=0.0,
                cache_load_s=time.monotonic() - began,
            )
            binary.touch()
            return model
        except Exception:  # noqa: BLE001 -- disposable cache cannot replace source validation
            timings["cache_hit"] = False
        model = (
            compile()
        )  # Keep original native source failures, outside cache recovery.
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                dir=root, suffix=".mjb", delete=False
            ) as handle:
                temporary = Path(handle.name)
            mj.mj_saveModel(model, str(temporary))
            if temporary.stat().st_size <= _MAX_BYTES:
                digest = _hash(temporary.read_bytes())
                temporary.replace(binary)
                with tempfile.NamedTemporaryFile(
                    dir=root, suffix=".json", mode="w", delete=False
                ) as handle:
                    json.dump({"key": key, "sha256": digest}, handle)
                    temporary_marker = Path(handle.name)
                temporary_marker.replace(marker)
                entries = sorted(
                    (
                        p
                        for p in root.glob("*.mjb")
                        if not p.is_symlink() and len(p.stem) == 64
                    ),
                    key=lambda p: p.stat().st_mtime_ns,
                    reverse=True,
                )
                total = 0
                for index, entry in enumerate(entries):
                    total += entry.stat().st_size
                    if index >= _MAX_ENTRIES or total > _MAX_BYTES:
                        entry.unlink()
                        entry.with_suffix(".json").unlink(missing_ok=True)
        except OSError:
            pass
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        return model


def load_native(root, config, *, cache=None, timings=None):
    """Resolve the ordinary native-scene contract before using compiled bytes."""
    from .native_scene import resolve

    root = Path(root)
    resolve(root, config)
    native = config["native_scene"]
    contents = {name: (root / name).read_bytes() for name in native["files"]}
    if any(
        _hash(contents[name]) != expected for name, expected in native["files"].items()
    ):
        raise ValueError(
            "simulation.native_source_changed: files changed after resolution"
        )
    return load_model(native["model"], contents, cache=cache, timings=timings)
