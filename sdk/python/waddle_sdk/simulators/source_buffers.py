"""Bounded private virtual assets, keyed by freshly hashed source bytes.

Native file buffers survive a call. Trusted authoring may also retain a private
specification for supported authored edits. Models and running state stay independent.
"""

import os
import threading
from collections import OrderedDict
from copy import copy


class SourceBuffers:
    def __init__(self, *, max_bytes=256 * 1024 * 1024, max_entries=2):
        from .prepared_fragments import FragmentCache

        self.max_bytes, self.max_entries = max_bytes, max_entries
        self.pid = os.getpid()
        self.lock = threading.RLock()
        self.entries = OrderedDict()
        self.prepared = {}
        self.fragments = FragmentCache()

    def close(self):
        with self.lock:
            self.prepared.clear()
            self.fragments.close()
            for _, vfs in self.entries.values():
                vfs.close()
            self.entries.clear()

    def compile(
        self,
        root,
        assets,
        *,
        key=None,
        authored=False,
        _authored_tree=None,
        _authored_contacts=None,
        timings=None,
    ):
        import mujoco as mj

        timings = {} if timings is None else timings
        timings["placement_reused"] = False

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
                source = None
                edits = 0
                if authored and key is not None:
                    from .authored_preparation import parse

                    try:
                        source = parse(
                            assets[root],
                            _tree=_authored_tree,
                            _contacts=_authored_contacts,
                        )
                    except Exception:  # noqa: BLE001 -- native parsing reports source errors
                        source = None
                # Native attachment can retain empty frames after body removal.
                # Bound each lineage as well as the number/bytes of source sets.
                if authored and prepared is not None and prepared[2] < 32:
                    from .authored_preparation import revise

                    try:
                        spec = revise(
                            prepared[0], prepared[1], source, _fragments=self.fragments
                        )
                        model = None
                        if spec is not None:
                            from .prepared_placement import translate

                            model = translate(prepared[3], prepared[1], source)
                            timings["placement_reused"] = model is not None
                            if model is None:
                                model = spec.compile(vfs=vfs)
                            edits = prepared[2] + 1
                    except Exception:  # noqa: BLE001 -- disposable preparation falls back to the source parser
                        spec = None
                if spec is None:
                    spec = mj.MjSpec.from_file(root, vfs=vfs)
                    model = spec.compile(vfs=vfs)
                del vfs[root]
                if key is not None:
                    template = None
                    if authored and source is not None:
                        model_size = mj.mj_sizeModel(model)
                        if size + model_size <= self.max_bytes:
                            template = copy(model)
                            size += model_size
                    self.entries[key] = size, vfs
                    if authored:
                        self.prepared[key] = spec, source, edits, template
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
