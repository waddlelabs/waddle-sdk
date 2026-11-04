"""Bounded private authoring templates for placement after geometry preparation.

Only the trusted scene author supplies templates. Consumers must verify the
published files again before using the result; this carries no live authority.
"""

import hashlib
import os
import xml.etree.ElementTree as ET
from copy import copy, deepcopy

from .model import numbers
from .playground import ScenePreparation, validate_scene


def position_changes(before, after):
    """Return authored translations, or None if any other scene input changed."""
    following = deepcopy(after)
    try:
        old, new = before["objects"], following["objects"]
        if len(old) != len(new):
            return None
        changes = {}
        for a, b in zip(old, new, strict=True):
            if a["position"] != b["position"]:
                changes[a["id"]] = b["position"]
                b["position"] = a["position"]
        return changes if following == before else None
    except (KeyError, TypeError):
        return None


class PlacementTemplate:
    """Own one pristine native model, source bytes and private authored tree.

    Retained bytes include the native model, source payload and another XML-size
    allowance for its tree; Python container overhead is excluded. Returned models
    are independent. Unchanged tree nodes are shared read-only by trusted consumers.
    """

    @classmethod
    def capture(cls, preparation, scene, *, max_bytes=128 * 1024 * 1024):
        import mujoco as mj

        root = preparation._tree
        if root is None or not os.environ.get("WADDLE_SIMULATION_MODEL_CACHE"):
            return None
        model = preparation.model
        size = (
            mj.mj_sizeModel(model)
            + sum(map(len, preparation.contents.values()))
            + len(preparation.contents["world.xml"])
        )
        if (
            size > max_bytes
            or root.find("statistic") is not None
            or any(getattr(model, name) for name in ("nplugin", "nflex", "nskin"))
            or model.nkey != 1
        ):
            return None
        self = cls()
        self.bytes = size
        self.scene = validate_scene(scene)
        self.source = ScenePreparation(
            deepcopy(preparation.config),
            copy(model),
            dict(preparation.contents),
            _tree=deepcopy(root),
        )
        self.bodies = {}
        owners, names = {}, {}
        world = self.source._tree.find("worldbody")
        for obj in self.scene["objects"]:
            name = "prop_" + obj["id"]
            body = world.find(f"body[@name='{name}']")
            index = model.body(name).id
            if (
                body is None
                or set(body.attrib) != {"name", "pos", "quat"}
                or model.body_parentid[index] != 0
                or model.body_mocapid[index] != -1
                or any(
                    node.tag not in {"freejoint", "geom"} or len(node) for node in body
                )
                or body.get("pos") != numbers(obj["position"])
            ):
                continue
            joint = int(model.body_jntadr[index])
            qpos = None
            if obj["kind"] == "target":
                if model.body_jntnum[index] or body.find("freejoint") is not None:
                    continue
            elif (
                model.body_jntnum[index] != 1
                or len(body.findall("freejoint")) != 1
                or model.jnt_type[joint] != mj.mjtJoint.mjJNT_FREE
            ):
                continue
            else:
                qpos = int(model.jnt_qposadr[joint])
            encoded = ET.tostring(body, encoding="unicode").encode()
            if self.source.contents["world.xml"].count(encoded) != 1:
                continue
            self.bodies[obj["id"]] = index, qpos, encoded
            for node in body.iter():
                owners[id(node)] = obj["id"]
                if node.get("name"):
                    names[node.get("name")] = obj["id"]
        # Native initialization of external constraints/sensors/cameras can
        # depend on a body's pose. Contact declarations retain ordinary runtime
        # contact semantics and are the sole permitted external references.
        contacts = self.source._tree.find("contact")
        contact_nodes = set() if contacts is None else {id(n) for n in contacts.iter()}
        for node in self.source._tree.iter():
            if id(node) in contact_nodes:
                continue
            values = [v for k, v in node.attrib.items() if k != "name"]
            values.append(node.text or "")
            for value in values:
                if "prop_" not in value:
                    continue
                for token in value.split():
                    owner = names.get(token)
                    if owner is not None and owner != owners.get(id(node)):
                        self.bodies.pop(owner, None)
        from .native_scene import replacement_signature

        self.contract = replacement_signature(
            self.source.config, _tree=self.source._tree
        )
        return self

    def close(self):
        self.source = None
        self.bodies.clear()

    def prepare(self, scene):
        """Produce exact final authoring bytes without compiling the geometry."""
        from .prepared_placement import translated_model

        scene = validate_scene(scene)
        changes = position_changes(self.scene, scene)
        if (
            self.source is None
            or changes is None
            or not changes.keys() <= self.bodies.keys()
        ):
            return None
        source = self.source
        model = translated_model(
            source.model,
            [
                (
                    self.bodies[name][0],
                    self.bodies[name][1],
                    list(map(float, numbers(pos).split())),
                )
                for name, pos in changes.items()
            ],
        )
        root = copy(source._tree)
        old_world = root.find("worldbody")
        world = copy(old_world)
        root[list(root).index(old_world)] = world
        xml = source.contents["world.xml"]
        for name, position in changes.items():
            old = world.find(f"body[@name='prop_{name}']")
            body = deepcopy(old)
            body.set("pos", numbers(position))
            world[list(world).index(old)] = body
            xml = xml.replace(
                self.bodies[name][2], ET.tostring(body, encoding="unicode").encode(), 1
            )
        old_keys = root.find("keyframe")
        keys = deepcopy(old_keys)
        root[list(root).index(old_keys)] = keys
        initial = keys.find("key[@name='initial']")
        previous = ET.tostring(initial, encoding="unicode").encode()
        if xml.count(previous) != 1:
            return None
        initial.set("qpos", numbers(model.key_qpos[model.key("initial").id]))
        xml = xml.replace(
            previous, ET.tostring(initial, encoding="unicode").encode(), 1
        )
        config = deepcopy(source.config)
        config["native_scene"]["files"]["world.xml"] = hashlib.sha256(xml).hexdigest()
        from .prepared_model import cache
        from .source_references import cache as references

        native = config["native_scene"]
        references.read(xml, native["files"]["world.xml"], _tree=root)
        cache.remember(
            "world.xml", native["files"], xml, model, _contract=self.contract
        )
        return ScenePreparation(
            config, model, {**source.contents, "world.xml": xml}, _tree=root
        )
