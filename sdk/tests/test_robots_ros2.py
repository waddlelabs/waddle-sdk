"""ROS 2 shared-world backend tests without a ROS installation."""

from __future__ import annotations

import threading
from types import SimpleNamespace

import numpy as np
import pytest
import waddle_sdk
import yaml
from waddle_sdk.cameras.site import CameraConfig, CameraMount
from waddle_sdk.robots import ros2
from waddle_sdk.robots.site import PartConfig
from waddle_sdk.simulation import WorldConfig


class Message:
    pass


class Float64MultiArray:
    def __init__(self):
        self.data = []


class Empty:
    class Request:
        pass


class Future:
    def add_done_callback(self, callback):
        callback(self)

    def exception(self):
        return None


class Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        if hasattr(message, "data"):
            self.messages.append(tuple(message.data))
        else:
            self.messages.append((tuple(message.name), tuple(message.position)))


class Client:
    def __init__(self, ready=True):
        self.ready = ready
        self.called = 0

    def wait_for_service(self, *, timeout_sec):
        assert timeout_sec > 0
        return self.ready

    def call_async(self, _request):
        self.called += 1
        return Future()


class Node:
    def __init__(self, messages):
        self.messages = messages
        self.publishers = []
        self.subscriptions = []
        self.clients = []
        self.destroyed = False

    def create_publisher(self, _type, topic, _qos):
        publisher = Publisher(topic)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, _type, topic, callback, _qos):
        subscription = SimpleNamespace(topic=topic, callback=callback)
        self.subscriptions.append(subscription)
        for message in self.messages.get(topic, ()):
            callback(message)
        return subscription

    def create_client(self, _type, service):
        client = Client()
        client.service = service
        self.clients.append(client)
        return client

    def destroy_subscription(self, subscription):
        if subscription in self.subscriptions:
            self.subscriptions.remove(subscription)

    def destroy_publisher(self, publisher):
        if publisher in self.publishers:
            self.publishers.remove(publisher)

    def destroy_client(self, client):
        if client in self.clients:
            self.clients.remove(client)

    def destroy_node(self):
        self.destroyed = True


class Context:
    def __init__(self):
        self.initialized = False
        self.closed = False

    def init(self, *, args, domain_id):
        assert args == []
        self.initialized = True
        self.domain_id = domain_id

    def try_shutdown(self):
        self.closed = True


class Executor:
    def __init__(self, *, context):
        self.context = context
        self.stopping = threading.Event()

    def add_node(self, node):
        self.node = node

    def spin(self):
        self.stopping.wait()

    def shutdown(self, *, timeout_sec):
        assert timeout_sec > 0
        self.stopping.set()
        return True


def _stamp(seconds=2, nanoseconds=10):
    return SimpleNamespace(sec=seconds, nanosec=nanoseconds)


def _joint_state():
    return SimpleNamespace(
        name=["extra", "shoulder", "elbow"],
        position=[9.0, 0.1, -0.2],
        velocity=[0.0, 0.3, -0.4],
    )


def _image(*, encoding, shape, data, step=None, stamp=None):
    height, width = shape
    bytes_per_pixel = {"rgb8": 3, "bgr8": 3, "16UC1": 2, "32FC1": 4}[encoding]
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp or _stamp()),
        height=height,
        width=width,
        encoding=encoding,
        is_bigendian=0,
        step=step or width * bytes_per_pixel,
        data=data,
    )


@pytest.fixture
def fake_ros(monkeypatch):
    rgb = np.asarray(
        [[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]], dtype=np.uint8
    )
    depth = np.asarray([[100, 200], [300, 400]], dtype="<u2")
    messages = {
        "/joint_states": [_joint_state()],
        "/rgb": [_image(encoding="bgr8", shape=(2, 2), data=rgb.tobytes())],
        "/depth": [_image(encoding="16UC1", shape=(2, 2), data=depth.tobytes())],
        "/info": [
            SimpleNamespace(
                k=[100.0, 0.0, 0.5, 0.0, 101.0, 0.5, 0.0, 0.0, 1.0],
                d=[],
            )
        ],
    }
    nodes = []

    def create_node(_name, **kwargs):
        assert kwargs["use_global_arguments"] is False
        node = Node(messages)
        nodes.append(node)
        return node

    modules = SimpleNamespace(
        rclpy=SimpleNamespace(create_node=create_node),
        Context=Context,
        SingleThreadedExecutor=Executor,
        qos_sensor_data=object(),
        JointState=Message,
        Image=Message,
        CameraInfo=Message,
        Float64MultiArray=Float64MultiArray,
        Empty=Empty,
    )
    monkeypatch.setattr(
        ros2,
        "_ros_modules",
        lambda *, needs_reset_service: modules,
    )
    return nodes


def _world(tmp_path):
    return ros2.backend(
        config=WorldConfig(
            name="cell",
            connection={
                "node_name": "waddle_test",
                "namespace": "/cell",
                "domain_id": 7,
                "reset_service": "/reset_simulation",
            },
            options={"reset_timeout_s": 1.0},
            site_root=tmp_path,
        )
    )


def _part(tmp_path):
    return PartConfig(
        name="arm",
        posture="supervised",
        connection={
            "joint_state_topic": "/joint_states",
            "command_topic": "/position_controller/commands",
        },
        joint_limits={"shoulder": [-1.0, 1.0], "elbow": [-1.5, 1.5]},
        workspace_bounds={},
        envelope={"static_keepouts": [], "self_collision": {}},
        base_frame="world",
        options={
            "home": [0.0, 0.0],
            "rate_hz": 100,
            "max_joint_speed_per_s": 1.0,
            "state_timeout_s": 1.0,
        },
        site_root=tmp_path,
        world="cell",
    )


def _camera(tmp_path):
    return CameraConfig(
        name="scene",
        connection={
            "rgb_topic": "/rgb",
            "depth_topic": "/depth",
            "camera_info_topic": "/info",
        },
        stream={"width": 2, "height": 2, "fps": 20},
        frame_id="scene_optical",
        intrinsics=None,
        mount=CameraMount(kind="scene"),
        options={"depth": True, "depth_scale_mm": 1.0, "sync_tolerance_ms": 1.0},
        site_root=tmp_path,
        world="cell",
    )


def test_ros2_world_is_lazy_and_normalizes_joint_and_rgbd_topics(tmp_path, fake_ros):
    world = _world(tmp_path)
    rig = world.part(config=_part(tmp_path))
    assert fake_ros == []
    world.open()
    driver = rig.arms()[""].driver
    assert driver.read()[0].tolist() == [0.1, -0.2]
    assert driver.read()[1].tolist() == [0.3, -0.4]
    driver.write(np.asarray([0.4, 0.5]))
    publisher = fake_ros[0].publishers[0]
    assert publisher.topic == "/position_controller/commands"
    assert publisher.messages[-1] == (0.4, 0.5)

    camera = world.camera(config=_camera(tmp_path))
    frame = camera.capture()
    # The incoming message is BGR8 and must become ordinary SDK RGB8.
    assert frame.rgb.tolist() == [
        [[3, 2, 1], [6, 5, 4]],
        [[9, 8, 7], [12, 11, 10]],
    ]
    assert frame.depth is not None
    assert frame.depth.tolist() == [[100, 200], [300, 400]]
    intrinsics = camera.intrinsics()
    assert (intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy) == (
        100.0,
        101.0,
        0.5,
        0.5,
    )
    assert world.reset()
    assert world.step(0.01) is None

    camera.close()
    driver.close()
    world.close()
    assert fake_ros[0].destroyed


def test_ros_float_depth_is_converted_to_declared_z16_units():
    metres = np.asarray([[0.5, np.nan], [1.25, 70.0]], dtype="<f4")
    message = _image(
        encoding="32FC1",
        shape=(2, 2),
        data=metres.tobytes(),
    )
    depth = ros2._depth_image(message, depth_scale_mm=1.0)
    assert depth.tolist() == [[500, 0], [1250, 0]]


def test_ros_joint_state_command_mode_matches_isaac_subscriber(tmp_path, fake_ros):
    world = _world(tmp_path)
    config = _part(tmp_path)
    config = PartConfig(
        **{
            **config.__dict__,
            "options": {**config.options, "command_message": "joint_state"},
        }
    )
    world.open()
    driver = world.part(config=config).arms()[""].driver
    driver.write(np.asarray([0.25, -0.5]))
    assert fake_ros[0].publishers[0].messages[-1] == (
        ("shoulder", "elbow"),
        (0.25, -0.5),
    )
    driver.close()
    world.close()


def test_ros_world_refuses_invalid_lifecycle_values_without_importing_ros(tmp_path):
    with pytest.raises(ValueError, match="domain_id"):
        ros2.backend(
            config=WorldConfig(
                name="cell",
                connection={"domain_id": -1},
                site_root=tmp_path,
            )
        )
    world = ros2.backend(
        config=WorldConfig(name="cell", connection={}, site_root=tmp_path)
    )
    with pytest.raises(RuntimeError, match="not open"):
        world.step(0.01)


def test_ros2_backend_reaches_metal_through_the_ordinary_site_port(tmp_path, fake_ros):
    document = {
        "api_version": "waddle.site/v1",
        "kind": "Site",
        "metadata": {"id": "ros-cell"},
        "worlds": {
            "cell": {
                "driver": "waddle_sdk.robots.ros2:backend",
                "connection": {"node_name": "waddle_site_test"},
                "options": {},
            }
        },
        "parts": {
            "arm": {
                "world": "cell",
                "posture": "supervised",
                "base_frame": "world",
                "connection": {
                    "joint_state_topic": "/joint_states",
                    "command_topic": "/position_controller/commands",
                },
                "joint_limits": {
                    "shoulder": [-1.0, 1.0],
                    "elbow": [-1.5, 1.5],
                },
                "options": {
                    "home": [0.0, 0.0],
                    "state_timeout_s": 1.0,
                },
            }
        },
        "cameras": {
            "scene": {
                "world": "cell",
                "connection": {
                    "rgb_topic": "/rgb",
                    "depth_topic": "/depth",
                    "camera_info_topic": "/info",
                },
                "stream": {"width": 2, "height": 2, "fps": 20},
                "frame_id": "scene_optical",
                "mount": {"kind": "scene"},
                "options": {"depth": True, "depth_scale_mm": 1.0},
            }
        },
        "frames": {},
        "calibration": {"artifacts": "calibration/"},
        "workspace_bounds": {},
        "envelope": {"static_keepouts": [], "self_collision": {}},
        "recording": {"root": "recordings/", "format": "mcap"},
    }
    path = tmp_path / "site.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    with waddle_sdk.load_site(path).open(console=False, _testing=True) as session:
        sample = session._require().wait_camera("scene", timeout_s=1.0)
        assert sample is not None
        assert sample.depth is not None
        assert set(session.observe().parts) == {"arm"}
        assert session.support().contract_version == "waddle.sdk.support/v1"
    assert fake_ros[0].destroyed
