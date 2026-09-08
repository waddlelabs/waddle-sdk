"""ROS 2 shared-world backend for simulator graphs.

The module is dependency-free until :meth:`Ros2Backend.open`; ROS distributions
provide ``rclpy`` and the standard message packages.  One backend owns one
dedicated ROS context, node, executor thread, joint-state subscriptions,
position-controller publishers, and paired RGB-D subscriptions.  Metal still
sees the ordinary SDK runtime and never imports ROS.
"""

from __future__ import annotations

import importlib
import math
import threading
from collections import deque
from collections.abc import Mapping, Sequence
from types import SimpleNamespace
from typing import Any

import numpy as np

from .. import descriptors
from ..cameras import CameraFrame
from ..cameras.site import CameraConfig
from ..simulation import WorldConfig
from . import base
from .site import PartConfig

__all__ = ["Ros2Backend", "Ros2CameraDriver", "Ros2Driver", "backend"]


def _ros_modules(*, needs_reset_service: bool) -> SimpleNamespace:
    """Import the ROS-owned Python surface only while a world opens."""

    try:
        rclpy = importlib.import_module("rclpy")
        context = importlib.import_module("rclpy.context")
        executors = importlib.import_module("rclpy.executors")
        qos = importlib.import_module("rclpy.qos")
        sensor_msgs = importlib.import_module("sensor_msgs.msg")
        std_msgs = importlib.import_module("std_msgs.msg")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "ROS 2 simulation needs a sourced ROS environment containing rclpy, "
            "sensor_msgs, and std_msgs"
        ) from exc
    empty = None
    if needs_reset_service:
        try:
            empty = importlib.import_module("std_srvs.srv").Empty
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "ROS 2 connection.reset_service needs std_srvs in the sourced "
                "ROS environment"
            ) from exc
    return SimpleNamespace(
        rclpy=rclpy,
        Context=context.Context,
        SingleThreadedExecutor=executors.SingleThreadedExecutor,
        qos_sensor_data=qos.qos_profile_sensor_data,
        JointState=sensor_msgs.JointState,
        Image=sensor_msgs.Image,
        CameraInfo=sensor_msgs.CameraInfo,
        Float64MultiArray=std_msgs.Float64MultiArray,
        Empty=empty,
    )


def _positive(value: object, *, field: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{field} must be numeric") from exc
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{field} must be finite and positive")
    return number


def _topic(value: object, *, field: str) -> str:
    if not isinstance(value, str) or not value or "\x00" in value:
        raise ValueError(f"{field} must be a non-empty ROS topic name")
    return value


class Ros2Backend:
    """One lazily opened ROS 2 graph behind the simulation-world contract."""

    def __init__(self, *, config: WorldConfig) -> None:
        self._config = config
        self._lock = threading.RLock()
        self._opened = False
        self._closed = False
        self._modules = None
        self._context = None
        self._node = None
        self._executor = None
        self._thread: threading.Thread | None = None
        node_name = config.connection.get("node_name", f"waddle_{config.name}")
        if not isinstance(node_name, str) or not node_name:
            raise ValueError("ROS 2 connection.node_name must be non-empty")
        self._node_name = node_name
        namespace = config.connection.get("namespace", "")
        if not isinstance(namespace, str):
            raise TypeError("ROS 2 connection.namespace must be a string")
        self._namespace = namespace
        domain_id = config.connection.get("domain_id")
        if domain_id is not None and (
            isinstance(domain_id, bool)
            or not isinstance(domain_id, int)
            or domain_id < 0
        ):
            raise ValueError(
                "ROS 2 connection.domain_id must be a non-negative integer"
            )
        self._domain_id = domain_id
        reset_service = config.connection.get("reset_service")
        self._reset_service = (
            None
            if reset_service is None
            else _topic(reset_service, field="ROS 2 connection.reset_service")
        )
        self._reset_timeout = _positive(
            config.options.get("reset_timeout_s", 5.0),
            field="ROS 2 options.reset_timeout_s",
        )

    @property
    def modules(self):
        self._require_open()
        return self._modules

    @property
    def node(self):
        self._require_open()
        return self._node

    def _require_open(self) -> None:
        if not self._opened or self._closed:
            raise RuntimeError("ROS 2 world is not open")

    def open(self) -> None:
        with self._lock:
            if self._opened or self._closed:
                raise RuntimeError("ROS 2 world instances may be opened only once")
            modules = _ros_modules(needs_reset_service=self._reset_service is not None)
            context = modules.Context()
            executor = None
            node = None
            try:
                context.init(args=[], domain_id=self._domain_id)
                node = modules.rclpy.create_node(
                    self._node_name,
                    context=context,
                    namespace=self._namespace,
                    use_global_arguments=False,
                    enable_rosout=False,
                    start_parameter_services=False,
                )
                executor = modules.SingleThreadedExecutor(context=context)
                executor.add_node(node)
                thread = threading.Thread(
                    target=executor.spin,
                    name=f"waddle-ros2-{self._config.name}",
                    daemon=True,
                )
                self._modules = modules
                self._context = context
                self._node = node
                self._executor = executor
                self._thread = thread
                self._opened = True
                thread.start()
            except BaseException:
                if executor is not None:
                    executor.shutdown(timeout_sec=1.0)
                if node is not None:
                    node.destroy_node()
                context.try_shutdown()
                raise

    def part(self, *, config: PartConfig) -> base.Rig:
        return _ros2_rig(self, config)

    def camera(self, *, config: CameraConfig) -> Ros2CameraDriver:
        self._require_open()
        return Ros2CameraDriver(world=self, config=config)

    def step(self, dt: float) -> None:
        self._require_open()
        duration = float(dt)
        if not math.isfinite(duration) or duration < 0.0:
            raise ValueError("ROS 2 step duration must be finite and non-negative")
        # The ROS graph owns its clock. This notification deliberately does not
        # publish /clock or call a simulator-specific stepping service.

    def reset(self) -> bool:
        self._require_open()
        if self._reset_service is None:
            return True
        client = self.node.create_client(self.modules.Empty, self._reset_service)
        try:
            if not client.wait_for_service(timeout_sec=self._reset_timeout):
                return False
            future = client.call_async(self.modules.Empty.Request())
            finished = threading.Event()
            future.add_done_callback(lambda _future: finished.set())
            if not finished.wait(self._reset_timeout):
                return False
            return future.exception() is None
        finally:
            self.node.destroy_client(client)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            executor = self._executor
            thread = self._thread
            node = self._node
            context = self._context
            self._opened = False
        if executor is not None:
            executor.shutdown(timeout_sec=5.0)
        if thread is not None and threading.current_thread() is not thread:
            thread.join(timeout=5.0)
        if node is not None:
            node.destroy_node()
        if context is not None:
            context.try_shutdown()
        with self._lock:
            self._executor = None
            self._thread = None
            self._node = None
            self._context = None
            self._modules = None


class Ros2Driver:
    """One ROS joint-state/position-controller pair behind the Driver contract."""

    kind = "sim"

    def __init__(
        self,
        *,
        world: Ros2Backend,
        joint_names: Sequence[str],
        state_topic: str,
        command_topic: str,
        command_message: str,
        initial_timeout_s: float,
        home: Sequence[float],
    ) -> None:
        self._world = world
        self._joint_names = tuple(joint_names)
        self._indices = {name: index for index, name in enumerate(self._joint_names)}
        self._condition = threading.Condition()
        self._position: np.ndarray | None = None
        self._velocity: np.ndarray | None = None
        self._closed = False
        self._estopped = False
        if command_message not in {"float64_multi_array", "joint_state"}:
            raise ValueError(
                "ROS 2 command_message must be 'float64_multi_array' or 'joint_state'"
            )
        self._command_message = command_message
        message_type = (
            world.modules.Float64MultiArray
            if command_message == "float64_multi_array"
            else world.modules.JointState
        )
        self._publisher = world.node.create_publisher(
            message_type,
            command_topic,
            1,
        )
        self._subscription = world.node.create_subscription(
            world.modules.JointState,
            state_topic,
            self._state,
            world.modules.qos_sensor_data,
        )
        with self._condition:
            if self._position is None:
                self._condition.wait_for(
                    lambda: self._position is not None or self._closed,
                    timeout=initial_timeout_s,
                )
            if self._position is None:
                self.close()
                raise RuntimeError(
                    f"ROS 2 joint state {state_topic!r} did not provide all "
                    "declared joints"
                )
        self.home(home)

    def _state(self, message: Any) -> None:
        names = tuple(str(value) for value in message.name)
        positions = tuple(float(value) for value in message.position)
        velocities = tuple(float(value) for value in message.velocity)
        if len(names) != len(positions) or len(set(names)) != len(names):
            return
        rows = {name: index for index, name in enumerate(names)}
        if any(name not in rows for name in self._joint_names):
            return
        selected_position = np.asarray(
            [positions[rows[name]] for name in self._joint_names], dtype=float
        )
        selected_velocity = np.zeros(len(self._joint_names), dtype=float)
        if len(velocities) == len(names):
            selected_velocity = np.asarray(
                [velocities[rows[name]] for name in self._joint_names], dtype=float
            )
        if not np.all(np.isfinite(selected_position)) or not np.all(
            np.isfinite(selected_velocity)
        ):
            return
        with self._condition:
            if self._closed:
                return
            self._position = selected_position
            self._velocity = selected_velocity
            self._condition.notify_all()

    @property
    def estopped(self) -> bool:
        with self._condition:
            return self._estopped

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        with self._condition:
            if self._closed:
                raise RuntimeError("ROS 2 driver is closed")
            if self._position is None or self._velocity is None:
                raise RuntimeError("ROS 2 joint state is not ready")
            return self._position.copy(), self._velocity.copy()

    def _publish(self, target: Sequence[float]) -> None:
        values = np.asarray(target, dtype=float).reshape(-1)
        if values.size != len(self._joint_names) or not np.all(np.isfinite(values)):
            raise ValueError("ROS 2 target must have one finite value per joint")
        if self._command_message == "float64_multi_array":
            message = self._world.modules.Float64MultiArray()
            message.data = [float(value) for value in values]
        else:
            message = self._world.modules.JointState()
            message.name = list(self._joint_names)
            message.position = [float(value) for value in values]
            message.velocity = []
            message.effort = []
        self._publisher.publish(message)

    def write(self, target: np.ndarray) -> None:
        with self._condition:
            if self._closed:
                raise RuntimeError("ROS 2 driver is closed")
            if self._estopped:
                raise RuntimeError(
                    "ROS 2 driver is e-stopped; re-enable it at the site"
                )
        self._publish(target)

    def hold(self) -> None:
        with self._condition:
            if self._closed or self._position is None:
                return
            position = self._position.copy()
        self._publish(position)

    def estop(self) -> None:
        with self._condition:
            self._estopped = True
        self.hold()

    def re_enable(self) -> None:
        self.hold()
        with self._condition:
            if self._closed:
                raise RuntimeError("ROS 2 driver is closed")
            self._estopped = False

    def step(self, dt: float) -> None:
        duration = float(dt)
        if not math.isfinite(duration) or duration < 0.0:
            raise ValueError("ROS 2 step duration must be finite and non-negative")

    def home(self, values: Sequence[float]) -> bool:
        with self._condition:
            if self._closed or self._estopped:
                return False
        self._publish(values)
        return True

    def close(self) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._condition.notify_all()
        self._world.node.destroy_subscription(self._subscription)
        self._world.node.destroy_publisher(self._publisher)


def _stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    second = int(stamp.sec)
    nanosecond = int(stamp.nanosec)
    if second < 0 or nanosecond < 0 or nanosecond >= 1_000_000_000:
        raise ValueError("ROS image has an invalid acquisition timestamp")
    return second * 1_000_000_000 + nanosecond


def _image_rows(message: Any, *, bytes_per_pixel: int) -> np.ndarray:
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if height <= 0 or width <= 0 or step < width * bytes_per_pixel:
        raise ValueError("ROS image dimensions or row step are invalid")
    data = bytes(message.data)
    if len(data) != height * step:
        raise ValueError("ROS image data length does not match height*step")
    return np.frombuffer(data, dtype=np.uint8).reshape(height, step)


def _rgb_image(message: Any) -> np.ndarray:
    encoding = str(message.encoding).lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported ROS RGB encoding {message.encoding!r}")
    rows = _image_rows(message, bytes_per_pixel=channels)
    image = rows[:, : int(message.width) * channels].reshape(
        int(message.height), int(message.width), channels
    )
    if encoding in {"bgr8", "bgra8"}:
        image = image[..., [2, 1, 0, 3] if channels == 4 else [2, 1, 0]]
    if channels == 4:
        image = image[..., :3]
    return np.ascontiguousarray(image, dtype=np.uint8)


def _depth_image(message: Any, *, depth_scale_mm: float) -> np.ndarray:
    encoding = str(message.encoding).lower()
    if encoding in {"16uc1", "mono16"}:
        rows = _image_rows(message, bytes_per_pixel=2)
        packed = rows[:, : int(message.width) * 2].copy()
        order = ">u2" if bool(message.is_bigendian) else "<u2"
        return np.ascontiguousarray(
            packed.reshape(-1)
            .view(order)
            .reshape(int(message.height), int(message.width)),
            dtype=np.uint16,
        )
    if encoding == "32fc1":
        rows = _image_rows(message, bytes_per_pixel=4)
        packed = rows[:, : int(message.width) * 4].copy()
        order = ">f4" if bool(message.is_bigendian) else "<f4"
        metres = (
            packed.reshape(-1)
            .view(order)
            .reshape(int(message.height), int(message.width))
        )
        valid = np.isfinite(metres) & (metres > 0.0)
        raw = np.zeros(metres.shape, dtype=np.uint16)
        scaled = np.rint(metres[valid] * (1000.0 / depth_scale_mm))
        within = scaled <= np.iinfo(np.uint16).max
        values = np.zeros(scaled.shape, dtype=np.uint16)
        values[within] = scaled[within].astype(np.uint16)
        raw[valid] = values
        return raw
    raise ValueError(f"unsupported ROS depth encoding {message.encoding!r}")


class Ros2CameraDriver:
    """Paired ROS Image topics normalized to one aligned SDK RGB-D frame."""

    def __init__(self, *, world: Ros2Backend, config: CameraConfig) -> None:
        self._world = world
        self._width = int(config.stream["width"])
        self._height = int(config.stream["height"])
        self._condition = threading.Condition()
        self._closing = False
        self._rgb: deque[tuple[int, np.ndarray]] = deque(maxlen=8)
        self._depth: deque[tuple[int, np.ndarray]] = deque(maxlen=8)
        self._paired: deque[CameraFrame] = deque(maxlen=2)
        self._intrinsics: descriptors.Intrinsics | None = None
        self._intrinsics_timeout = _positive(
            config.options.get("camera_info_timeout_s", 5.0),
            field="ROS 2 camera options.camera_info_timeout_s",
        )
        self._depth_enabled = bool(config.options.get("depth", True))
        self._depth_scale_mm = _positive(
            config.options.get("depth_scale_mm", 1.0),
            field="ROS 2 camera options.depth_scale_mm",
        )
        tolerance_ms = float(config.options.get("sync_tolerance_ms", 5.0))
        if not math.isfinite(tolerance_ms) or tolerance_ms < 0.0:
            raise ValueError("ROS 2 camera sync_tolerance_ms must be non-negative")
        self._tolerance_ns = int(tolerance_ms * 1_000_000)
        rgb_topic = _topic(
            config.connection.get("rgb_topic"),
            field="ROS 2 camera connection.rgb_topic",
        )
        self._subscriptions = [
            world.node.create_subscription(
                world.modules.Image,
                rgb_topic,
                self._on_rgb,
                world.modules.qos_sensor_data,
            )
        ]
        if self._depth_enabled:
            depth_topic = _topic(
                config.connection.get("depth_topic"),
                field="ROS 2 camera connection.depth_topic",
            )
            self._subscriptions.append(
                world.node.create_subscription(
                    world.modules.Image,
                    depth_topic,
                    self._on_depth,
                    world.modules.qos_sensor_data,
                )
            )
        info_topic = config.connection.get("camera_info_topic")
        if info_topic is None and config.intrinsics is None:
            raise ValueError(
                "ROS 2 cameras need connection.camera_info_topic or explicit "
                "site intrinsics"
            )
        if info_topic is not None:
            self._subscriptions.append(
                world.node.create_subscription(
                    world.modules.CameraInfo,
                    _topic(
                        info_topic,
                        field="ROS 2 camera connection.camera_info_topic",
                    ),
                    self._on_info,
                    world.modules.qos_sensor_data,
                )
            )

    def _on_rgb(self, message: Any) -> None:
        try:
            stamp = _stamp_ns(message)
            image = _rgb_image(message)
            if image.shape != (self._height, self._width, 3):
                return
        except (TypeError, ValueError):
            return
        with self._condition:
            if self._closing:
                return
            self._rgb.append((stamp, image))
            self._match()

    def _on_depth(self, message: Any) -> None:
        try:
            stamp = _stamp_ns(message)
            image = _depth_image(message, depth_scale_mm=self._depth_scale_mm)
            if image.shape != (self._height, self._width):
                return
        except (TypeError, ValueError):
            return
        with self._condition:
            if self._closing:
                return
            self._depth.append((stamp, image))
            self._match()

    def _on_info(self, message: Any) -> None:
        try:
            matrix = tuple(float(value) for value in message.k)
            if len(matrix) != 9:
                return
            intrinsics = descriptors.Intrinsics(
                fx=matrix[0],
                fy=matrix[4],
                cx=matrix[2],
                cy=matrix[5],
                distortion=tuple(float(value) for value in message.d),
                depth_scale_mm=self._depth_scale_mm,
            )
        except (TypeError, ValueError):
            return
        with self._condition:
            if not self._closing:
                self._intrinsics = intrinsics
                self._condition.notify_all()

    def _match(self) -> None:
        if not self._rgb:
            return
        if not self._depth_enabled:
            _stamp, rgb = self._rgb.popleft()
            self._paired.append(CameraFrame(rgb=rgb))
            self._condition.notify_all()
            return
        if not self._depth:
            return
        best = None
        for rgb_index, (rgb_stamp, _rgb) in enumerate(self._rgb):
            for depth_index, (depth_stamp, _depth) in enumerate(self._depth):
                delta = abs(rgb_stamp - depth_stamp)
                if best is None or delta < best[0]:
                    best = (delta, rgb_index, depth_index)
        assert best is not None
        if best[0] > self._tolerance_ns:
            if self._rgb[0][0] < self._depth[0][0]:
                self._rgb.popleft()
            else:
                self._depth.popleft()
            return
        _delta, rgb_index, depth_index = best
        _rgb_stamp, rgb = self._rgb[rgb_index]
        _depth_stamp, depth = self._depth[depth_index]
        for _ in range(rgb_index + 1):
            self._rgb.popleft()
        for _ in range(depth_index + 1):
            self._depth.popleft()
        self._paired.append(CameraFrame(rgb=rgb, depth=depth))
        self._condition.notify_all()

    def intrinsics(self) -> descriptors.Intrinsics:
        with self._condition:
            self._condition.wait_for(
                lambda: self._intrinsics is not None or self._closing,
                timeout=self._intrinsics_timeout,
            )
            if self._intrinsics is None:
                raise RuntimeError(
                    "ROS CameraInfo did not arrive before camera_info_timeout_s"
                )
            return self._intrinsics

    def capture(self) -> CameraFrame:
        with self._condition:
            self._condition.wait_for(lambda: bool(self._paired) or self._closing)
            if self._closing:
                raise RuntimeError("ROS 2 camera is closed")
            return self._paired.popleft()

    def close(self) -> None:
        with self._condition:
            if self._closing:
                return
            self._closing = True
            self._condition.notify_all()
        for subscription in self._subscriptions:
            self._world.node.destroy_subscription(subscription)
        self._subscriptions.clear()


def _limits_and_names(
    config: PartConfig,
) -> tuple[tuple[str, ...], tuple[tuple[float, float], ...]]:
    raw = config.joint_limits
    if isinstance(raw, Mapping) and raw:
        names = tuple(str(name) for name in raw)
        rows = tuple(raw.values())
    elif isinstance(raw, Sequence) and not isinstance(raw, (str, bytes)) and raw:
        names = tuple(str(name) for name in config.options.get("joint_names", ()))
        rows = tuple(raw)
    else:
        raise ValueError("ROS 2 parts require explicit joint_limits")
    limits = tuple((float(row[0]), float(row[1])) for row in rows)
    if not names or len(names) != len(limits):
        raise ValueError("ROS 2 joint_names and joint_limits must have equal width")
    return names, limits


def _ros2_rig(world: Ros2Backend, config: PartConfig) -> base.Rig:
    names, limits = _limits_and_names(config)
    rate_hz = _positive(config.options.get("rate_hz", 100.0), field="ROS 2 rate_hz")
    max_speed = _positive(
        config.options.get("max_joint_speed_per_s", 1.0),
        field="ROS 2 max_joint_speed_per_s",
    )
    home = tuple(
        float(value)
        for value in config.options.get(
            "home", tuple((lower + upper) / 2.0 for lower, upper in limits)
        )
    )
    if len(home) != len(names):
        raise ValueError("ROS 2 home must have one value per joint")
    state_topic = _topic(
        config.connection.get("joint_state_topic", "/joint_states"),
        field="ROS 2 part connection.joint_state_topic",
    )
    command_topic = _topic(
        config.connection.get("command_topic", "/position_controller/commands"),
        field="ROS 2 part connection.command_topic",
    )
    command_message = str(config.options.get("command_message", "float64_multi_array"))
    initial_timeout = _positive(
        config.options.get("state_timeout_s", 5.0),
        field="ROS 2 state_timeout_s",
    )

    def build_arms() -> dict[str, base.Arm]:
        driver = Ros2Driver(
            world=world,
            joint_names=names,
            state_topic=state_topic,
            command_topic=command_topic,
            command_message=command_message,
            initial_timeout_s=initial_timeout,
            home=home,
        )
        return {
            "": base.Arm(
                part="",
                driver=driver,
                joint_names=names,
                joint_limits=limits,
                step_caps=tuple(max_speed / rate_hz for _ in names),
                base_frame=config.base_frame or "",
                home_values=home,
                rate_hz=rate_hz,
            )
        }

    return base.Rig(
        declaration=descriptors.Robot(
            name=config.name,
            action_space=descriptors.JointSpace(
                joints=tuple(
                    descriptors.Joint(
                        name=name,
                        min_position=lower,
                        max_position=upper,
                        max_velocity=max_speed,
                    )
                    for name, (lower, upper) in zip(names, limits, strict=True)
                ),
                rate_hz=rate_hz,
            ),
        ),
        build_arms=build_arms,
        rate_hz=rate_hz,
        posture=config.posture,
    )


def backend(*, config: WorldConfig) -> Ros2Backend:
    """Build one unopened ROS 2 shared world selected by ``site.yaml``."""

    return Ros2Backend(config=config)
