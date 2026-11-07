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
    comparison: object = None


def _layout(base, node, fingers, props):
    """Admit the trusted builder's plain ordered Cartesian-product layout."""
    if (
        base.tag != "contact"
        or base.attrib
        or base.text
        or base.tail
        or node.attrib != base.attrib
        or node.text != base.text
        or node.tail != base.tail
        or [ET.tostring(child) for child in list(node)[: len(base)]]
        != [ET.tostring(child) for child in base]
        or len(set(fingers)) != len(fingers)
        or len(set(props)) != len(props)
        or set(fingers) & set(props)
        or any(
            child.tag not in {"pair", "exclude"}
            or len(child)
            or child.text
            or child.tail
            for child in base
        )
        or any({child.get("geom1"), child.get("geom2")} & set(props) for child in base)
        or len(node) != len(base) + len(fingers) * len(props)
    ):
        return False
    return all(
        child.tag == "pair"
        and child.attrib
        == {"geom1": finger, "geom2": prop, "class": "playground_contact"}
        and not len(child)
        and child.text is None
        and child.tail is None
        for child, (finger, prop) in zip(
            list(node)[len(base) :],
            ((finger, prop) for finger in fingers for prop in props),
            strict=True,
        )
    )


def _xml(pieces):
    body = b"".join(pieces)
    return b"<contact>" + body + b"</contact>" if body else b"<contact />"


def _pieces(node, base_count, width, rows):
    children = list(node)

    def encode(selected):
        if not selected:
            return b""
        shell = ET.Element("contact")
        shell.extend(selected)
        return ET.tostring(shell, encoding="unicode").encode()[9:-10]

    return (
        encode(children[:base_count]),
        *(
            encode(children[base_count + row * width : base_count + (row + 1) * width])
            for row in range(rows)
        ),
    )


def _append(base, fingers, props, build, entries):
    """Extend one exact earlier layout, keeping no chain of prior preparations."""
    from .authored_preparation import ContactIndex, contact_index

    for key, entry in reversed(entries.items()):
        old_props = key[2]
        if (
            key[:2] != (ET.tostring(base, encoding="unicode").encode(), tuple(fingers))
            or key[3] is not build
            or not key[4]
            or entry[4] is None
            or len(old_props) >= len(props)
            or tuple(props[: len(old_props)]) != old_props
            or len(set(props)) != len(props)
            or set(fingers) & set(props)
            or any(
                {child.get("geom1"), child.get("geom2")} & set(props) for child in base
            )
        ):
            continue
        added_props = props[len(old_props) :]
        delta = deepcopy(base)
        build(delta, fingers, added_props)
        if not _layout(base, delta, fingers, added_props):
            continue
        normalized = contact_index(delta)
        node, indexed = ET.Element("contact"), ET.Element("contact")
        old_index = entry[3]
        n = len(base)
        node.extend(list(entry[0])[:n])
        indexed.extend(list(old_index.node)[:n])
        pieces, records = [entry[4][0]], list(old_index.entries[:n])
        old_nodes = list(entry[0])
        delta_nodes = list(delta)
        old_width, added_width = len(old_props), len(added_props)
        added_pieces = _pieces(delta, n, added_width, len(fingers))
        for row in range(len(fingers)):
            a = n + row * old_width
            b = n + row * added_width
            node.extend(old_nodes[a : a + old_width])
            node.extend(delta_nodes[b : b + added_width])
            old_records = old_index.entries[a : a + old_width]
            new_records = normalized.entries[b : b + added_width]
            indexed.extend(child for _, child in (*old_records, *new_records))
            records.extend((*old_records, *new_records))
            pieces.append(entry[4][row + 1] + added_pieces[row + 1])
        comparison = ContactIndex(
            indexed,
            tuple(records),
            object(),
            old_index.token,
            tuple(child for _, child in normalized.entries[n:]),
        )
        return node, _xml(pieces), comparison, tuple(pieces)
    return None


class ContactCache:
    def __init__(self, *, max_bytes=32 * 1024 * 1024, max_entries=2):
        self.max_bytes, self.max_entries = max_bytes, max_entries
        self.pid = os.getpid()
        self.lock = threading.Lock()
        self.entries = OrderedDict()

    def prepare(self, contact, fingers, props, build, *, _incremental=False):
        if self.pid != os.getpid():
            self.lock = threading.Lock()
            self.entries = OrderedDict()
            self.pid = os.getpid()
        base = ET.tostring(contact, encoding="unicode").encode()
        key = base, tuple(fingers), tuple(props), build, _incremental
        with self.lock:
            entry = self.entries.pop(key, None)
            try:
                if entry is None:
                    prepared = (
                        _append(contact, fingers, props, build, self.entries)
                        if _incremental
                        else None
                    )
                    if prepared is None:
                        node = deepcopy(contact)
                        build(node, fingers, props)
                        comparison = pieces = None
                        if _incremental:
                            from .authored_preparation import contact_index

                            comparison = contact_index(node)
                            if _layout(contact, node, fingers, props):
                                pieces = _pieces(
                                    node, len(contact), len(props), len(fingers)
                                )
                        xml = (
                            _xml(pieces)
                            if pieces is not None
                            else ET.tostring(node, encoding="unicode").encode()
                        )
                    else:
                        node, xml, comparison, pieces = prepared
                    size = (
                        len(base)
                        + len(xml)
                        + sum(len(piece) for piece in pieces or ())
                        + sum(len(name.encode()) for name in (*fingers, *props))
                    )
                    entry = node, xml, size, comparison, pieces
                self.entries[key] = entry
                while (
                    len(self.entries) > self.max_entries
                    or sum(row[2] for row in self.entries.values()) > self.max_bytes
                ):
                    self.entries.popitem(last=False)
                return ContactFragment(deepcopy(entry[0]), entry[1], entry[3])
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
