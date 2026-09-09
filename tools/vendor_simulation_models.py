"""Rebuild the pinned reference robot descriptions (offline at runtime).

Run with Python 3.12, xacro==2.1.1, trimesh==5.1.0, pycollada==0.9.3,
coacd==1.0.14, numpy and scipy. Downloads only public manufacturer files.
Visual conversion preserves metres and triangles; collision decomposition
approximates concave surfaces with convex pieces.
"""

from __future__ import annotations

import copy
import hashlib
import json
import tempfile
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

import coacd
import numpy as np
import trimesh
import xacro
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "sdk/python/waddle_sdk/simulators/data"
I2RT = "570ef66681ff12bd8298aba34084307cfecc9f05"
I2RT_HAND = "5b72c47239bd056d0fa6c1a39edeb0537c89443c"
XARM = "aad7e1611c9c46eb719045414394bfdd42dcb0f8"
SOURCES: dict[str, str] = {}


def fetch(repo: str, revision: str, path: str) -> bytes:
    url = f"https://raw.githubusercontent.com/{repo}/{revision}/{path}"
    data = urllib.request.urlopen(url, timeout=60).read()
    SOURCES[url] = hashlib.sha256(data).hexdigest()
    return data


def numbers(values) -> str:
    return " ".join(str(float(x)) for x in values)


def rpy(q: str) -> str:
    values = np.fromstring(q, sep=" ")
    return numbers(Rotation.from_quat(values[[1, 2, 3, 0]]).as_euler("xyz"))


def write(root: ET.Element, path: Path) -> None:
    ET.indent(root)
    path.write_text(ET.tostring(root, encoding="unicode") + "\n")


def collision_parts(robot: ET.Element, dest: Path) -> None:
    """Retain concave collision geometry with offline decomposition shared by engines."""
    coacd.set_log_level("error")
    for link in robot.findall("link"):
        name = link.get("name")
        collision = link.find("collision")
        if collision is None:
            continue
        source = dest / collision.find("geometry/mesh").get("filename")
        mesh = trimesh.load_mesh(source)
        if mesh.is_convex:
            continue
        parts = coacd.run_coacd(
            coacd.Mesh(mesh.vertices, mesh.faces),
            threshold=0.015,
            resolution=1500,
            mcts_nodes=15,
            mcts_iterations=100,
            max_convex_hull=32,
            seed=0,
        )
        directory = dest / "assets/collision" / name
        directory.mkdir(parents=True, exist_ok=True)
        for old in directory.glob("*.stl"):
            old.unlink()
        link.remove(collision)
        for index, (vertices, faces) in enumerate(parts):
            path = directory / f"{name}-{index}.stl"
            path.write_bytes(
                trimesh.Trimesh(vertices, faces, process=False).export(file_type="stl")
            )
            element = copy.deepcopy(collision)
            element.find("geometry/mesh").set("filename", str(path.relative_to(dest)))
            link.append(element)


def yam() -> None:
    dest = DATA / "yam"
    (dest / "assets").mkdir(parents=True, exist_ok=True)
    source = ROOT / "sdk/python/waddle_sdk/robots/yam_data/yam.urdf"
    robot = ET.parse(source).getroot()
    # The shipped arm contract includes the previous hand's housing at link_6.
    # Replace that housing with the live adapter's LINEAR_4310 assembly. Keep
    # all six arm joints and the public grasp_link frame byte-for-byte numeric.
    flange = robot.find("link[@name='link_6']")
    for child in list(flange):
        flange.remove(child)
    for mesh in robot.iter("mesh"):
        relative = mesh.get("filename")
        (dest / relative).write_bytes(
            fetch("i2rt-robotics/i2rt", I2RT, "i2rt/robot_models/arm/yam/" + relative)
        )
    hand_data = fetch(
        "i2rt-robotics/i2rt",
        I2RT_HAND,
        "i2rt/robot_models/gripper/linear_4310/linear_4310.xml",
    )
    (dest / "linear_4310.xml").write_bytes(hand_data)
    hand = ET.fromstring(hand_data)
    for mesh in hand.findall("asset/mesh"):
        relative = "assets/" + mesh.get("file")
        (dest / relative).write_bytes(
            fetch(
                "i2rt-robotics/i2rt",
                I2RT_HAND,
                "i2rt/robot_models/gripper/linear_4310/" + relative,
            )
        )
    bodies = hand.findall(".//body")
    for body in bodies:
        name = body.get("name")
        link = ET.SubElement(robot, "link", name=name)
        raw = body.find("inertial")
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz=raw.get("pos"), rpy=rpy(raw.get("quat")))
        ET.SubElement(inertial, "mass", value=raw.get("mass"))
        diagonal = raw.get("diaginertia").split()
        ET.SubElement(
            inertial,
            "inertia",
            ixx=diagonal[0],
            iyy=diagonal[1],
            izz=diagonal[2],
            ixy="0",
            ixz="0",
            iyz="0",
        )
        for geom in body.findall("geom"):
            for kind in ("visual", "collision"):
                element = ET.SubElement(link, kind)
                ET.SubElement(
                    element, "origin", xyz=geom.get("pos"), rpy=rpy(geom.get("quat"))
                )
                ET.SubElement(
                    ET.SubElement(element, "geometry"),
                    "mesh",
                    filename=f"assets/{geom.get('mesh')}.stl",
                )
                if kind == "visual":
                    material = ET.SubElement(element, "material", name=name)
                    # Neutral colors separate robot geometry from task objects.
                    ET.SubElement(material, "color", rgba="0.25 0.27 0.3 1")
        raw_joint = body.find("joint")
        joint = ET.SubElement(
            robot,
            "joint",
            name="linear_mount" if raw_joint is None else raw_joint.get("name"),
            type="fixed" if raw_joint is None else "prismatic",
        )
        ET.SubElement(
            joint, "parent", link="link_6" if raw_joint is None else "gripper"
        )
        ET.SubElement(joint, "child", link=name)
        if raw_joint is None:
            # Preserve the SDK flange/TCP contract while mounting the current
            # linear hand. Its physical pinch offset is read from grasp_site.
            ET.SubElement(joint, "origin", xyz="0 0 0", rpy=f"{np.pi} 0 -1.5708")
        else:
            ET.SubElement(
                joint, "origin", xyz=body.get("pos"), rpy=rpy(body.get("quat"))
            )
            ET.SubElement(joint, "axis", xyz=raw_joint.get("axis"))
            lo, hi = raw_joint.get("range").split()
            ET.SubElement(
                joint, "limit", lower=lo, upper=hi, effort="20", velocity="0.1"
            )
            if raw_joint.get("name") == "joint8":
                ET.SubElement(
                    joint, "mimic", joint="joint7", multiplier="1", offset="0"
                )
    collision_parts(robot, dest)
    write(robot, dest / "robot.urdf")
    station = ET.fromstring(
        fetch(
            "i2rt-robotics/i2rt",
            I2RT_HAND,
            "i2rt/robot_models/station/yam_station_linear_4310_d405/"
            "yam_station_linear_4310_d405.urdf",
        )
    )
    origin = station.find("joint[@name='left_camera_joint']/origin")
    (dest / "wrist_camera.json").write_text(
        json.dumps(
            {key: list(map(float, origin.get(key).split())) for key in ("xyz", "rpy")},
            indent=2,
        )
        + "\n"
    )
    (dest / "LICENSE").write_bytes(fetch("i2rt-robotics/i2rt", I2RT, "LICENSE"))


def xarm7() -> None:
    dest = DATA / "xarm7"
    dest.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as temporary:
        package = Path(temporary)
        paths = [
            "urdf/common/common.link.xacro",
            "urdf/common/common.material.xacro",
            "urdf/xarm7/xarm7.urdf.xacro",
            "urdf/gripper/xarm_gripper.urdf.xacro",
            "config/link_inertial/xarm7_type7_HT_BR2.yaml",
            "config/kinematics/default/xarm7_default_kinematics.yaml",
        ]
        for relative in paths:
            path = package / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            raw = fetch("xArm-Developer/xarm_ros", XARM, "xarm_description/" + relative)
            path.write_text(
                raw.decode().replace("$(find xarm_description)", str(package))
            )
        wrapper = package / "model.xacro"
        includes = "".join(
            f'<xacro:include filename="{package / p}"/>' for p in paths[:4]
        )
        wrapper.write_text(f'''<robot
  xmlns:xacro="http://ros.org/wiki/xacro" name="xarm7_g2">
<xacro:property name="mesh_suffix" value="stl"/>
<xacro:property name="mesh_path" value="assets"/>
<xacro:property name="use_xacro_load_yaml" value="true"/>
{includes}
<xacro:xarm7_urdf prefix="" kinematics_params_filename="{package / paths[-1]}"/>
<xacro:xarm_gripper_urdf attach_to="link7" gripper_version="G2"/>
</robot>''')
        robot = ET.fromstring(xacro.process_file(str(wrapper)).toxml())
        for relative in sorted({m.get("filename") for m in robot.iter("mesh")}):
            target = dest / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            raw = fetch(
                "xArm-Developer/xarm_ros",
                XARM,
                "xarm_description/meshes/" + relative.removeprefix("assets/"),
            )
            if target.suffix == ".dae":
                source = package / target.name
                source.write_bytes(raw)
                # Scene.to_mesh applies COLLADA node transforms without changing units.
                mesh = trimesh.load_scene(source).to_mesh()
                target = target.with_suffix(".stl")
                target.write_bytes(mesh.export(file_type="stl"))
                for node in robot.iter("mesh"):
                    if node.get("filename") == relative:
                        node.set("filename", str(Path(relative).with_suffix(".stl")))
            else:
                target.write_bytes(raw)
        # G2's housing mesh includes its long cable. A single convex hull
        # fills the empty space between cable and housing, blocking the arm.
        collision_parts(robot, dest)
        write(robot, dest / "robot.urdf")
    (dest / "LICENSE").write_bytes(fetch("xArm-Developer/xarm_ros", XARM, "LICENSE"))


if __name__ == "__main__":
    yam()
    xarm7()
    outputs = {
        str(p.relative_to(DATA)): hashlib.sha256(p.read_bytes()).hexdigest()
        for name in ("yam", "xarm7")
        for p in sorted((DATA / name).rglob("*"))
        if p.is_file()
    }
    (DATA / "models.json").write_text(
        json.dumps(
            {
                "sources": SOURCES,
                "assembly_inputs": {
                    "sdk/python/waddle_sdk/robots/yam_data/yam.urdf": hashlib.sha256(
                        (
                            ROOT / "sdk/python/waddle_sdk/robots/yam_data/yam.urdf"
                        ).read_bytes()
                    ).hexdigest()
                },
                "files": outputs,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n"
    )
