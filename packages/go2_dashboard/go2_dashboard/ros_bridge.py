"""ROS 2 subscriptions feeding the read-only dashboard state.

ROS imports are intentionally delayed until the observer thread starts.  The
web UI can therefore report a useful dependency error instead of crashing when
it is opened on a host where the ROS environment was not sourced.
"""

from __future__ import annotations

import io
import math
import os
import threading
import time
from dataclasses import dataclass, replace
from typing import Any

from .parsers import (
    cloud_to_points,
    goal_status_label,
    image_to_rgb_bytes,
    occupancy_to_luma,
    quaternion_to_yaw,
    scan_to_points,
)
from .state import DEFAULT_TOPIC_SPECS, DashboardState, TopicSpec


def _environment_name(name: str, default: str, *, ros_topic: bool) -> str:
    value = os.environ.get(name, default).strip()
    if not value or len(value) > 200 or any(character.isspace() for character in value):
        raise ValueError(f"invalid {name}")
    if ros_topic and not value.startswith("/"):
        raise ValueError(f"{name} must be an absolute ROS topic")
    return value


@dataclass(frozen=True)
class RosConfig:
    camera_topic: str = "/camera/color/image_raw"
    camera_status_topic: str = "/go2/sensors/status"
    scan_topic: str = "/scanner/scan"
    cloud_topic: str = "/scanner/cloud"
    map_topic: str = "/map"
    odom_topic: str = "/odom"
    nav_status_topic: str = "/navigate_to_pose/_action/status"
    map_frame: str = "map"
    base_frame: str = "base_link"
    camera_period_s: float = 0.2
    lidar_period_s: float = 0.2

    @classmethod
    def from_environment(cls) -> "RosConfig":
        return cls(
            camera_topic=_environment_name(
                "GO2_DASHBOARD_CAMERA_TOPIC", cls.camera_topic, ros_topic=True
            ),
            camera_status_topic=_environment_name(
                "GO2_DASHBOARD_CAMERA_STATUS_TOPIC",
                cls.camera_status_topic,
                ros_topic=True,
            ),
            scan_topic=_environment_name(
                "GO2_DASHBOARD_SCAN_TOPIC", cls.scan_topic, ros_topic=True
            ),
            cloud_topic=_environment_name(
                "GO2_DASHBOARD_CLOUD_TOPIC", cls.cloud_topic, ros_topic=True
            ),
            map_topic=_environment_name(
                "GO2_DASHBOARD_MAP_TOPIC", cls.map_topic, ros_topic=True
            ),
            odom_topic=_environment_name(
                "GO2_DASHBOARD_ODOM_TOPIC", cls.odom_topic, ros_topic=True
            ),
            nav_status_topic=_environment_name(
                "GO2_DASHBOARD_NAV_STATUS_TOPIC",
                cls.nav_status_topic,
                ros_topic=True,
            ),
            map_frame=_environment_name(
                "GO2_DASHBOARD_MAP_FRAME", cls.map_frame, ros_topic=False
            ),
            base_frame=_environment_name(
                "GO2_DASHBOARD_BASE_FRAME", cls.base_frame, ros_topic=False
            ),
        )

    def topic_specs(self) -> dict[str, TopicSpec]:
        specs = dict(DEFAULT_TOPIC_SPECS)
        specs["camera"] = replace(specs["camera"], topic=self.camera_topic)
        specs["laser_scan"] = replace(specs["laser_scan"], topic=self.scan_topic)
        specs["point_cloud"] = replace(specs["point_cloud"], topic=self.cloud_topic)
        specs["map"] = replace(specs["map"], topic=self.map_topic)
        specs["odom"] = replace(specs["odom"], topic=self.odom_topic)
        specs["pose_map"] = replace(
            specs["pose_map"], topic=f"TF {self.map_frame} -> {self.base_frame}"
        )
        specs["nav_status"] = replace(
            specs["nav_status"], topic=self.nav_status_topic
        )
        return specs


def _stamp_payload(stamp: Any) -> dict[str, int | float]:
    seconds = int(stamp.sec)
    nanoseconds = int(stamp.nanosec)
    return {
        "sec": seconds,
        "nanosec": nanoseconds,
        "unix_s": seconds + nanoseconds / 1_000_000_000.0,
    }


def _finite_float(value: Any, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} is not finite")
    return result


def _position_payload(position: Any) -> dict[str, float]:
    return {
        "x": round(_finite_float(position.x, "position.x"), 5),
        "y": round(_finite_float(position.y, "position.y"), 5),
        "z": round(_finite_float(position.z, "position.z"), 5),
    }


def _quaternion_payload(rotation: Any) -> dict[str, float]:
    return {
        "x": round(_finite_float(rotation.x, "orientation.x"), 7),
        "y": round(_finite_float(rotation.y, "orientation.y"), 7),
        "z": round(_finite_float(rotation.z, "orientation.z"), 7),
        "w": round(_finite_float(rotation.w, "orientation.w"), 7),
    }


class RosBridge:
    """Own a single ROS observer thread and no command-side interfaces."""

    def __init__(
        self, state: DashboardState, config: RosConfig | None = None
    ) -> None:
        self._state = state
        self._config = config or RosConfig.from_environment()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._state.set_bridge(
            running=True, connected=False, error="正在初始化 ROS 只读观察线程"
        )
        self._thread = threading.Thread(
            target=self._run,
            name="go2-dashboard-ros-observer",
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout_s: float = 5.0) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.0, float(timeout_s)))

    def _run(self) -> None:
        context = None
        node = None
        executor = None
        try:
            import rclpy
            from action_msgs.msg import GoalStatusArray
            from diagnostic_msgs.msg import DiagnosticArray
            from nav_msgs.msg import OccupancyGrid, Odometry
            from PIL import Image
            from rclpy.context import Context
            from rclpy.duration import Duration
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
                qos_profile_sensor_data,
            )
            from rclpy.time import Time
            from sensor_msgs.msg import Image as RosImage
            from sensor_msgs.msg import LaserScan, PointCloud2
            from tf2_ros import Buffer, TransformListener

            state = self._state
            config = self._config
            stop_event = self._stop_event

            map_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            action_status_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )

            class _TelemetryNode(Node):
                def __init__(self, ros_context: Context) -> None:
                    super().__init__("go2_readonly_dashboard", context=ros_context)
                    self._last_camera = 0.0
                    self._last_scan = 0.0
                    self._last_cloud = 0.0
                    self._tf_buffer = Buffer()
                    self._tf_listener = TransformListener(
                        self._tf_buffer, self, spin_thread=False
                    )
                    self._subscriptions = [
                        self.create_subscription(
                            RosImage,
                            config.camera_topic,
                            self._on_camera,
                            qos_profile_sensor_data,
                        ),
                        self.create_subscription(
                            DiagnosticArray,
                            config.camera_status_topic,
                            self._on_camera_status,
                            10,
                        ),
                        self.create_subscription(
                            LaserScan,
                            config.scan_topic,
                            self._on_scan,
                            qos_profile_sensor_data,
                        ),
                        self.create_subscription(
                            PointCloud2,
                            config.cloud_topic,
                            self._on_cloud,
                            qos_profile_sensor_data,
                        ),
                        self.create_subscription(
                            OccupancyGrid,
                            config.map_topic,
                            self._on_map,
                            map_qos,
                        ),
                        self.create_subscription(
                            Odometry,
                            config.odom_topic,
                            self._on_odom,
                            qos_profile_sensor_data,
                        ),
                        self.create_subscription(
                            GoalStatusArray,
                            config.nav_status_topic,
                            self._on_nav_status,
                            action_status_qos,
                        ),
                    ]
                    self._tf_timer = self.create_timer(0.5, self._read_map_pose)

                def _on_camera(self, message: RosImage) -> None:
                    now = time.monotonic()
                    if now - self._last_camera < config.camera_period_s:
                        return
                    self._last_camera = now
                    try:
                        rgb = image_to_rgb_bytes(
                            message.data,
                            int(message.width),
                            int(message.height),
                            int(message.step),
                            str(message.encoding),
                        )
                        image = Image.frombytes(
                            "RGB", (int(message.width), int(message.height)), rgb
                        )
                        output = io.BytesIO()
                        image.save(output, format="JPEG", quality=72)
                        state.set_camera(
                            output.getvalue(),
                            {
                                "width": int(message.width),
                                "height": int(message.height),
                                "encoding": str(message.encoding),
                                "stamp": _stamp_payload(message.header.stamp),
                            },
                            frame_id=str(message.header.frame_id),
                        )
                    except Exception as error:
                        state.note_error("camera", error)

                def _on_camera_status(self, message: DiagnosticArray) -> None:
                    try:
                        selected = next(
                            entry
                            for entry in message.status
                            if str(entry.name) == "go2_sensors/camera"
                        )
                    except StopIteration:
                        return
                    try:
                        values = {
                            str(entry.key): str(entry.value)
                            for entry in selected.values
                        }
                        numeric_keys = (
                            "rate_hz",
                            "quality_error_ratio",
                            "quality_window_attempts",
                            "quality_window_failures",
                            "reconnect_count",
                            "rejected_count",
                            "daemon_source_rejected_count",
                            "daemon_api_error_count",
                            "daemon_last_api_code",
                        )
                        numeric = {
                            key: _finite_float(values[key], key)
                            for key in numeric_keys
                            if key in values
                        }
                        state.set_camera_quality(
                            {
                                **numeric,
                                "level": int(selected.level),
                                "message": str(selected.message),
                                "healthy": values.get("healthy") == "true",
                                "ready": values.get("quality_ready") == "true",
                                "api_code_semantics": (
                                    "opaque vendor return code; not interpreted"
                                ),
                            }
                        )
                    except Exception as error:
                        state.note_error("camera", error)

                def _on_scan(self, message: LaserScan) -> None:
                    now = time.monotonic()
                    if now - self._last_scan < config.lidar_period_s:
                        return
                    self._last_scan = now
                    try:
                        points = scan_to_points(
                            message.ranges,
                            float(message.angle_min),
                            float(message.angle_increment),
                            float(message.range_min),
                            float(message.range_max),
                        )
                        state.observe(
                            "laser_scan",
                            {
                                "kind": "LaserScan",
                                "points": points,
                                "point_count": len(points),
                                "range_max": round(float(message.range_max), 3),
                                "stamp": _stamp_payload(message.header.stamp),
                            },
                            frame_id=str(message.header.frame_id),
                        )
                    except Exception as error:
                        state.note_error("laser_scan", error)

                def _on_cloud(self, message: PointCloud2) -> None:
                    now = time.monotonic()
                    if now - self._last_cloud < config.lidar_period_s:
                        return
                    self._last_cloud = now
                    try:
                        points = cloud_to_points(
                            message.data,
                            message.fields,
                            int(message.point_step),
                            int(message.width),
                            int(message.height),
                            bool(message.is_bigendian),
                            row_step=int(message.row_step),
                        )
                        state.observe(
                            "point_cloud",
                            {
                                "kind": "PointCloud2",
                                "points": points,
                                "point_count": len(points),
                                "stamp": _stamp_payload(message.header.stamp),
                            },
                            frame_id=str(message.header.frame_id),
                        )
                    except Exception as error:
                        state.note_error("point_cloud", error)

                def _on_map(self, message: OccupancyGrid) -> None:
                    try:
                        width = int(message.info.width)
                        height = int(message.info.height)
                        luma = occupancy_to_luma(message.data, width, height)
                        image = Image.frombytes("L", (width, height), luma)
                        output = io.BytesIO()
                        image.save(output, format="PNG", compress_level=3)
                        origin = message.info.origin
                        origin_yaw = quaternion_to_yaw(
                            origin.orientation.x,
                            origin.orientation.y,
                            origin.orientation.z,
                            origin.orientation.w,
                        )
                        resolution = _finite_float(
                            message.info.resolution, "map resolution"
                        )
                        if resolution <= 0.0:
                            raise ValueError("map resolution must be positive")
                        state.set_map(
                            output.getvalue(),
                            {
                                "width": width,
                                "height": height,
                                "resolution": resolution,
                                "origin": {
                                    **_position_payload(origin.position),
                                    "yaw": round(origin_yaw, 7),
                                },
                                "stamp": _stamp_payload(message.header.stamp),
                                "load_time": _stamp_payload(message.info.map_load_time),
                            },
                            frame_id=str(message.header.frame_id),
                        )
                    except Exception as error:
                        state.note_error("map", error)

                def _on_odom(self, message: Odometry) -> None:
                    try:
                        pose = message.pose.pose
                        twist = message.twist.twist
                        yaw = quaternion_to_yaw(
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        )
                        state.observe(
                            "odom",
                            {
                                "frame_id": str(message.header.frame_id),
                                "child_frame_id": str(message.child_frame_id),
                                "stamp": _stamp_payload(message.header.stamp),
                                "position": _position_payload(pose.position),
                                "orientation": _quaternion_payload(pose.orientation),
                                "yaw": round(yaw, 7),
                                "linear": {
                                    "x": round(
                                        _finite_float(twist.linear.x, "linear.x"), 5
                                    ),
                                    "y": round(
                                        _finite_float(twist.linear.y, "linear.y"), 5
                                    ),
                                    "z": round(
                                        _finite_float(twist.linear.z, "linear.z"), 5
                                    ),
                                },
                                "angular": {
                                    "x": round(
                                        _finite_float(twist.angular.x, "angular.x"), 5
                                    ),
                                    "y": round(
                                        _finite_float(twist.angular.y, "angular.y"), 5
                                    ),
                                    "z": round(
                                        _finite_float(twist.angular.z, "angular.z"), 5
                                    ),
                                },
                            },
                            frame_id=str(message.header.frame_id),
                        )
                    except Exception as error:
                        state.note_error("odom", error)

                def _on_nav_status(self, message: GoalStatusArray) -> None:
                    try:
                        if not message.status_list:
                            payload = {
                                "status": "idle",
                                "status_code": 0,
                                "goal_id": "",
                                "stamp": None,
                            }
                        else:
                            selected = max(
                                message.status_list,
                                key=lambda entry: (
                                    int(entry.goal_info.stamp.sec),
                                    int(entry.goal_info.stamp.nanosec),
                                ),
                            )
                            status_code = int(selected.status)
                            payload = {
                                "status": goal_status_label(status_code),
                                "status_code": status_code,
                                "goal_id": bytes(
                                    selected.goal_info.goal_id.uuid
                                ).hex(),
                                "stamp": _stamp_payload(
                                    selected.goal_info.stamp
                                ),
                                "tracked_goals": len(message.status_list),
                            }
                        state.observe("nav_status", payload)
                    except Exception as error:
                        state.note_error("nav_status", error)

                def _read_map_pose(self) -> None:
                    try:
                        transform = self._tf_buffer.lookup_transform(
                            config.map_frame,
                            config.base_frame,
                            Time(),
                            timeout=Duration(seconds=0.03),
                        )
                        translation = transform.transform.translation
                        rotation = transform.transform.rotation
                        yaw = quaternion_to_yaw(
                            rotation.x, rotation.y, rotation.z, rotation.w
                        )
                        state.observe(
                            "pose_map",
                            {
                                "parent_frame": config.map_frame,
                                "child_frame": config.base_frame,
                                "stamp": _stamp_payload(transform.header.stamp),
                                "position": _position_payload(translation),
                                "orientation": _quaternion_payload(rotation),
                                "yaw": round(yaw, 7),
                            },
                            frame_id=config.map_frame,
                        )
                    except Exception as error:
                        state.note_error("pose_map", error)

            context = Context()
            rclpy.init(args=None, context=context)
            node = _TelemetryNode(context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            state.set_bridge(running=True, connected=True, error="")
            while not stop_event.is_set() and context.ok():
                executor.spin_once(timeout_sec=0.1)
        except Exception as error:
            self._state.set_bridge(running=False, connected=False, error=str(error))
        finally:
            if executor is not None:
                try:
                    executor.shutdown(timeout_sec=1.0)
                except Exception:
                    pass
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if context is not None:
                try:
                    context.shutdown()
                except Exception:
                    pass
            if self._stop_event.is_set():
                self._state.set_bridge(
                    running=False, connected=False, error="ROS 观察线程已停止"
                )
            elif self._state.snapshot()["bridge"]["connected"]:
                self._state.set_bridge(
                    running=False, connected=False, error="ROS 上下文意外结束"
                )
