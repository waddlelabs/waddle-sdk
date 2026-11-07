"""Bounded XML reference analysis, keyed only by freshly verified source bytes."""

import os
import threading
import xml.etree.ElementTree as ET
from collections import OrderedDict
from pathlib import Path


def references(data):
    return _references(ET.fromstring(data))


def _references(xml):
    compiler = xml.find("compiler")
    directories = {} if compiler is None else compiler.attrib
    result = []
    for node in xml.iter():
        for key in ("file", "meshdir", "texturedir", "assetdir"):
            value = node.get(key)
            if not value:
                continue
            path = Path(value)
            if path.is_absolute() or ".." in path.parts or "\\" in value:
                raise ValueError("native XML references must remain portable")
            if key == "file":
                directory = directories.get("assetdir", "")
                if node.tag in ("mesh", "texture"):
                    directory = directories.get(node.tag + "dir", directory)
                result.append((Path(directory) / path).as_posix())
    return tuple(result)


class SourceReferences:
    """Retain reference names, never resolved paths, file validity or XML trees.

    Callers freshly hash the supplied bytes before using this private cache.
    Two input documents totaling at most 32 MiB may retain their reference lists;
    Python object overhead is excluded. Parsing failures and fork discard records.
    """

    def __init__(self, *, max_bytes=32 * 1024 * 1024, max_entries=2):
        self.max_bytes, self.max_entries = max_bytes, max_entries
        self.pid = os.getpid()
        self.lock = threading.Lock()
        self.entries = OrderedDict()

    def read(self, data, digest, *, _tree=None):
        # Trusted authoring can supply the exact tree it just serialized into
        # these bytes. Ordinary validators always derive analysis from bytes.
        if self.pid != os.getpid():
            self.lock = threading.Lock()
            self.entries = OrderedDict()
            self.pid = os.getpid()
        key = digest, len(data)
        with self.lock:
            existing = self.entries.pop(key, None)
            if existing is not None:
                self.entries[key] = existing
                return existing
            try:
                result = references(data) if _tree is None else _references(_tree)
            except BaseException:
                self.entries.clear()
                raise
            self.entries[key] = result
            while (
                len(self.entries) > self.max_entries
                or sum(key[1] for key in self.entries) > self.max_bytes
            ):
                self.entries.popitem(last=False)
            return result


cache = SourceReferences()
