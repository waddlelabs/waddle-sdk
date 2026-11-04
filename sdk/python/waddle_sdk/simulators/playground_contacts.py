"""Private prepared contact sections for the trusted Playground author.

The key contains every base contact byte, ordered geometry name and builder.
Returned trees are detached; only the author may reuse their serialized fragment
during the same call, before making any contact changes. No native state is kept.
"""

import os
import threading
import xml.etree.ElementTree as ET
from collections import OrderedDict
from copy import copy, deepcopy
from dataclasses import dataclass


@dataclass(frozen=True)
class ContactFragment:
    node: ET.Element
    xml: bytes


class ContactCache:
    def __init__(self, *, max_bytes=32 * 1024 * 1024, max_entries=2):
        self.max_bytes, self.max_entries = max_bytes, max_entries
        self.pid = os.getpid()
        self.lock = threading.Lock()
        self.entries = OrderedDict()

    def prepare(self, contact, fingers, props, build):
        if self.pid != os.getpid():
            self.lock = threading.Lock()
            self.entries = OrderedDict()
            self.pid = os.getpid()
        base = ET.tostring(contact, encoding="unicode").encode()
        key = base, tuple(fingers), tuple(props), build
        with self.lock:
            entry = self.entries.pop(key, None)
            try:
                if entry is None:
                    node = deepcopy(contact)
                    build(node, fingers, props)
                    xml = ET.tostring(node, encoding="unicode").encode()
                    size = (
                        len(base)
                        + len(xml)
                        + sum(len(name.encode()) for name in (*fingers, *props))
                    )
                    entry = node, xml, size
                self.entries[key] = entry
                while (
                    len(self.entries) > self.max_entries
                    or sum(row[2] for row in self.entries.values()) > self.max_bytes
                ):
                    self.entries.popitem(last=False)
                return ContactFragment(deepcopy(entry[0]), entry[1])
            except BaseException:
                self.entries.clear()
                raise


def scene_xml(root, fragment=None):
    """Serialize the current author tree using its unchanged contact fragment.

    The shallow root is private. Reset keys and rewritten asset paths are always
    serialized anew; only the exact same-call contact node can use the fragment.
    Unrecognized roots/namespaces retain ordinary ElementTree serialization.
    """
    if fragment is not None and root.find("contact") is fragment.node:
        shell = copy(root)
        marker = ET.Element("_waddle_contact_fragment_")
        shell[list(shell).index(fragment.node)] = marker
        data = ET.tostring(shell, encoding="unicode").encode()
        token = b"<_waddle_contact_fragment_ />"
        if (
            data.count(token) == 1
            and b"xmlns" not in data
            and b"xmlns" not in fragment.xml
        ):
            return data.replace(token, fragment.xml)
    return ET.tostring(root, encoding="unicode").encode()


cache = ContactCache()
