"""Bounded immutable model handoffs on trusted, process-private owner pipes.

The owning host validates the ordinary source inventory before using a matching
preparation. Linux sealed descriptors carry the compiled bytes without copying
large Python buffers through each pipe. Every consumer loads a separate model;
no live data is retained. Unsupported systems keep ordinary source/cache loading.
No descriptor or native payload is accepted through authoring JSON or files.
"""

import json
import os
import platform
import threading
import weakref
from collections import OrderedDict
from dataclasses import dataclass, replace

from .compiled_model import _canonical_inputs


def _identity(model_name, files):
    name, entries = _canonical_inputs(model_name, files)
    return name, tuple(sorted(entries.items()))


def _seals():
    import fcntl

    return (
        fcntl.F_SEAL_WRITE | fcntl.F_SEAL_GROW | fcntl.F_SEAL_SHRINK | fcntl.F_SEAL_SEAL
    )


@dataclass(frozen=True)
class PreparedModel:
    identity: tuple
    runtime: tuple
    xml: bytes
    fd: int
    size: int
    contract: bytes = b""
    bodies: bytes = b""

    def __post_init__(self):
        object.__setattr__(
            self, "_finalizer", weakref.finalize(self, os.close, self.fd)
        )

    def descriptor(self):
        if not self._finalizer.alive:
            raise ValueError("simulation.prepared_closed: preparation was released")
        return self.fd

    def close(self):
        self._finalizer()

    def metadata(self):
        values = self.identity, self.runtime, self.xml, self.size, self.contract
        return (*values, self.bodies) if self.bodies else values

    def body_comparison(self, model_name, files, objects):
        """Map private same-source body analysis to the verified object bindings."""
        if not self.bodies or self.identity != _identity(model_name, files):
            return None
        by_joint = json.loads(self.bodies)
        if any(joint not in by_joint for joint in objects.values()):
            return None
        return {name: by_joint[joint] for name, joint in objects.items()}

    def replacement_contract(self, config):
        """Read a same-source robot comparison after fresh source validation."""
        native = config["native_scene"]
        if not self.contract or self.identity != _identity(
            native["model"], native["files"]
        ):
            return None
        result = json.loads(self.contract)
        # The inventory alone does not bind the selected robot/camera mapping.
        # Changed selection must run the ordinary comparison against that XML.
        if result[:2] != [native["prefixes"], native["camera_names"]]:
            return None
        return result

    def duplicate(self):
        return PreparedModel(
            self.identity,
            self.runtime,
            self.xml,
            os.dup(self.descriptor()),
            self.size,
            self.contract,
            self.bodies,
        )

    def __reduce__(self):
        from multiprocessing.reduction import DupFd

        values = (
            DupFd(self.descriptor()),
            self.identity,
            self.runtime,
            self.xml,
            self.size,
            self.contract,
        )
        return _receive, (*values, self.bodies) if self.bodies else values


def _receive(descriptor, identity, runtime, xml, size, contract, bodies=b""):
    return PreparedModel(
        identity, runtime, xml, descriptor.detach(), size, contract, bodies
    )


def receive(connection, metadata):
    """Receive a descriptor on an existing private pipe to a subprocess owner."""
    from multiprocessing.reduction import recv_handle

    if len(metadata) == 5:
        metadata = (*metadata, b"")
    identity, runtime, xml, size, contract, bodies = metadata
    return PreparedModel(
        identity, runtime, xml, recv_handle(connection), size, contract, bodies
    )


def _runtime():
    import mujoco

    return mujoco.__version__, platform.system(), platform.machine()


def seal(model, *, identity, xml=b"", _contract=None):
    """Capture the supplied current model, without retaining live state or a cache."""
    import mujoco as mj

    fd = None
    try:
        import fcntl

        fd = os.memfd_create(
            "waddle-prepared-model", os.MFD_CLOEXEC | os.MFD_ALLOW_SEALING
        )
        mj.mj_saveModel(model, f"/proc/self/fd/{fd}")
        if os.fstat(fd).st_size != mj.mj_sizeModel(model):
            raise OSError("compiled model export was incomplete")
        fcntl.fcntl(fd, fcntl.F_ADD_SEALS, _seals())
        contract = b"" if _contract is None else json.dumps(_contract).encode()
        size = (
            os.fstat(fd).st_size
            + len(xml)
            + len(repr(identity).encode())
            + len(contract)
        )
        payload = PreparedModel(identity, _runtime(), xml, fd, size, contract)
        fd = None
        return payload
    except (OSError, AttributeError, ImportError, mj.FatalError):
        return None
    finally:
        if fd is not None:
            os.close(fd)


class PreparedModelCache:
    def __init__(self, *, max_bytes=256 * 1024 * 1024):
        self.max_bytes = max_bytes
        self.entries = OrderedDict()
        self.lock = threading.Lock()

    def remember(self, model_name, files, xml, model, *, _contract=None):
        payload = seal(
            model, identity=_identity(model_name, files), xml=xml, _contract=_contract
        )
        if payload is None:
            return False
        identity = payload.identity
        with self.lock:
            previous = self.entries.pop(identity, None)
            if previous is not None:
                previous.close()
            self.entries[identity] = payload
            while (
                len(self.entries) > 2
                or sum(v.size for v in self.entries.values()) > self.max_bytes
            ):
                self.entries.popitem(last=False)[1].close()
            return identity in self.entries

    def get(self, model_name, files):
        identity = _identity(model_name, files)
        with self.lock:
            entry = self.entries.get(identity)
            if entry is None:
                return None
            self.entries.move_to_end(identity)
            # A handoff outlives cache eviction. Its private descriptor closes
            # explicitly or when this short-lived handle is released.
            return entry.duplicate()

    def annotate_bodies(self, model_name, files, bodies):
        """Complete a private preparation from its trusted same-call author tree.

        Do not accept this analysis from scene files or authoring JSON. Existing
        handed-out descriptors retain their own immutable metadata and lifetime.
        Oversized optional analysis leaves the compiled preparation available.
        """
        identity = _identity(model_name, files)
        encoded = json.dumps(bodies).encode()
        with self.lock:
            previous = self.entries.get(identity)
            if previous is None:
                return False
            size = previous.size - len(previous.bodies) + len(encoded)
            if size > self.max_bytes:
                return False
            try:
                descriptor = os.dup(previous.descriptor())
            except OSError:
                return False
            entry = replace(previous, fd=descriptor, size=size, bodies=encoded)
            self.entries[identity] = entry
            self.entries.move_to_end(identity)
            previous.close()
            while sum(value.size for value in self.entries.values()) > self.max_bytes:
                self.entries.popitem(last=False)[1].close()
            return True

    def close(self):
        with self.lock:
            while self.entries:
                self.entries.popitem()[1].close()


def restore(model_name, files, prepared, *, _preparation=None):
    """Load a matching sealed native model after ordinary source validation."""
    if not isinstance(prepared, PreparedModel) or prepared.identity != _identity(
        model_name, files
    ):
        raise ValueError(
            "simulation.prepared_source: preparation describes different sources"
        )
    if _preparation is not None:
        model = _preparation.take(prepared)
        if model is not None:
            return model
    return load_sealed(prepared)


def load_sealed(prepared):
    """Load after the private consumer has checked its source or snapshot identity."""
    if prepared.runtime != _runtime():
        return None
    import fcntl

    import mujoco as mj

    fd = prepared.descriptor()
    if fcntl.fcntl(fd, fcntl.F_GET_SEALS) & _seals() != _seals():
        raise ValueError("simulation.prepared_mutable: preparation is not sealed")
    return mj.MjModel.from_binary_path(f"/proc/self/fd/{fd}")


cache = PreparedModelCache()
