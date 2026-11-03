"""Bounded immutable model handoffs on trusted, process-private owner pipes.

The owning host validates the ordinary source inventory before using a matching
preparation. Linux sealed descriptors carry the compiled bytes without copying
large Python buffers through each pipe. Every consumer loads a separate model;
no live data is retained. Unsupported systems keep ordinary source/cache loading.
No descriptor or native payload is accepted through authoring JSON or files.
"""

import os
import platform
import threading
import weakref
from collections import OrderedDict
from dataclasses import dataclass

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
        return self.identity, self.runtime, self.xml, self.size

    def duplicate(self):
        return PreparedModel(
            self.identity, self.runtime, self.xml, os.dup(self.descriptor()), self.size
        )

    def __reduce__(self):
        from multiprocessing.reduction import DupFd

        return _receive, (
            DupFd(self.descriptor()),
            self.identity,
            self.runtime,
            self.xml,
            self.size,
        )


def _receive(descriptor, identity, runtime, xml, size):
    return PreparedModel(identity, runtime, xml, descriptor.detach(), size)


def receive(connection, metadata):
    """Receive a descriptor on an existing private pipe to a subprocess owner."""
    from multiprocessing.reduction import recv_handle

    identity, runtime, xml, size = metadata
    return PreparedModel(identity, runtime, xml, recv_handle(connection), size)


def _runtime():
    import mujoco

    return mujoco.__version__, platform.system(), platform.machine()


class PreparedModelCache:
    def __init__(self, *, max_bytes=256 * 1024 * 1024):
        self.max_bytes = max_bytes
        self.entries = OrderedDict()
        self.lock = threading.Lock()

    def remember(self, model_name, files, xml, model):
        import mujoco as mj

        identity = _identity(model_name, files)
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
            size = os.fstat(fd).st_size + len(xml) + len(repr(identity).encode())
            payload = PreparedModel(identity, _runtime(), xml, fd, size)
            fd = None
        except (OSError, AttributeError, ImportError, mj.FatalError):
            return False
        finally:
            if fd is not None:
                os.close(fd)
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

    def close(self):
        with self.lock:
            while self.entries:
                self.entries.popitem()[1].close()


def restore(model_name, files, prepared):
    """Load a matching sealed native model after ordinary source validation."""
    if not isinstance(prepared, PreparedModel) or prepared.identity != _identity(
        model_name, files
    ):
        raise ValueError(
            "simulation.prepared_source: preparation describes different sources"
        )
    if prepared.runtime != _runtime():
        return None
    import fcntl

    import mujoco as mj

    fd = prepared.descriptor()
    if fcntl.fcntl(fd, fcntl.F_GET_SEALS) & _seals() != _seals():
        raise ValueError("simulation.prepared_mutable: preparation is not sealed")
    return mj.MjModel.from_binary_path(f"/proc/self/fd/{fd}")


cache = PreparedModelCache()
