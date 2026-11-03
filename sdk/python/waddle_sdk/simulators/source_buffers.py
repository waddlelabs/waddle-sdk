"""Bounded private virtual assets, keyed by freshly hashed source bytes.

Native file buffers survive a call. Trusted authoring may also retain a private
specification for supported authored edits. Models and running state stay independent.
"""

import os
import threading
from collections import OrderedDict


class SourceBuffers:
    def __init__(self, *, max_bytes=256 * 1024 * 1024, max_entries=2):
        self.max_bytes, self.max_entries = max_bytes, max_entries
        self.pid = os.getpid()
        self.lock = threading.RLock()
        self.entries = OrderedDict()
        self.prepared = {}

    def close(self):
        with self.lock:
            self.prepared.clear()
            for _, vfs in self.entries.values():
                vfs.close()
            self.entries.clear()

    def compile(self, root, assets, *, key=None, authored=False):
        import mujoco as mj

        if self.pid != os.getpid():
            self.lock = threading.RLock()
            self.close()
            self.pid = os.getpid()
        size = sum(len(name.encode()) + len(data) for name, data in assets.items())
        key = (key, root) if key is not None and size <= self.max_bytes else None
        with self.lock:
            entry = self.entries.pop(key, None)
            prepared = self.prepared.pop(key, None)
            vfs = entry[1] if entry is not None else mj.MjVfs()
            try:
                if entry is None:
                    for name, data in assets.items():
                        if name != root:
                            vfs[name] = data
                vfs[root] = assets[root]
                spec = None
                edits = 0
                # Native attachment can retain empty frames after body removal.
                # Bound each lineage as well as the number/bytes of source sets.
                if authored and prepared is not None and prepared[2] < 32:
                    from .authored_preparation import revise

                    try:
                        spec = revise(prepared[0], prepared[1], assets[root])
                        model = None if spec is None else spec.compile(vfs=vfs)
                        if spec is not None:
                            edits = prepared[2] + 1
                    except Exception:  # noqa: BLE001 -- disposable preparation falls back to the source parser
                        spec = None
                if spec is None:
                    spec = mj.MjSpec.from_file(root, vfs=vfs)
                    model = spec.compile(vfs=vfs)
                del vfs[root]
                if key is not None:
                    self.entries[key] = size, vfs
                    if authored:
                        self.prepared[key] = spec, assets[root], edits
                    vfs = None
                    while (
                        len(self.entries) > self.max_entries
                        or sum(size for size, _ in self.entries.values())
                        > self.max_bytes
                    ):
                        evicted, (_, obsolete) = self.entries.popitem(last=False)
                        self.prepared.pop(evicted, None)
                        obsolete.close()
                return model
            finally:
                if vfs is not None:
                    vfs.close()


cache = SourceBuffers()
