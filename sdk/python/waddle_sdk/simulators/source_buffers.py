"""Bounded private virtual assets, keyed by freshly hashed source bytes.

Only native file buffers survive a call. Each new root is parsed and compiled
into an independent model; no spec, model or running state is retained here.
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

    def close(self):
        with self.lock:
            for _, vfs in self.entries.values():
                vfs.close()
            self.entries.clear()

    def compile(self, root, assets, *, key=None):
        import mujoco as mj

        if self.pid != os.getpid():
            self.lock = threading.RLock()
            self.close()
            self.pid = os.getpid()
        size = sum(len(name.encode()) + len(data) for name, data in assets.items())
        key = (key, root) if key is not None and size <= self.max_bytes else None
        with self.lock:
            entry = self.entries.pop(key, None)
            vfs = entry[1] if entry is not None else mj.MjVfs()
            try:
                if entry is None:
                    for name, data in assets.items():
                        if name != root:
                            vfs[name] = data
                vfs[root] = assets[root]
                spec = mj.MjSpec.from_file(root, vfs=vfs)
                model = spec.compile(vfs=vfs)
                del vfs[root]
                if key is not None:
                    self.entries[key] = size, vfs
                    vfs = None
                    while (
                        len(self.entries) > self.max_entries
                        or sum(size for size, _ in self.entries.values())
                        > self.max_bytes
                    ):
                        self.entries.popitem(last=False)[1][1].close()
                return model
            finally:
                if vfs is not None:
                    vfs.close()


cache = SourceBuffers()
