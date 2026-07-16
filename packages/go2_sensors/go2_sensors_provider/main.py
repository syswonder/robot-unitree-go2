#!/usr/bin/env python3
"""Register the standard Go2 sensor streams with Robonix Atlas.

This module owns process lifecycle and endpoint metadata only. Sensor bytes are
handled by the C++ ROS nodes and the isolated, read-only camera daemon.
"""

from __future__ import annotations

import os
from pathlib import Path
import signal
import socket
import threading
import time
from typing import Any

from robonix_api import Deferred, Err, Ok, Primitive


ROOT = Path(__file__).resolve().parents[1]
ROS_BUILD = Path(os.environ.get("GO2_SENSORS_BUILD_ROOT", ROOT / ".build" / "ros"))
CAMERA_BUILD = Path(
    os.environ.get("GO2_CAMERA_DAEMON_BUILD_ROOT", ROOT / ".build" / "camera-daemon")
)
CAMERA_INSTALL = CAMERA_BUILD / "install"
CAMERA_LIBRARY_DIR = CAMERA_INSTALL / "lib"
SENSOR_RELAY = ROS_BUILD / "install" / "lib" / "go2_sensors" / "go2_sensor_relay"
CAMERA_BRIDGE = ROS_BUILD / "install" / "lib" / "go2_sensors" / "go2_camera_bridge"
CAMERA_DAEMON = CAMERA_INSTALL / "bin" / "go2_camera_daemon"
CAMERA_DDS_LIBRARIES = tuple(
    CAMERA_LIBRARY_DIR / name
    for name in ("libddsc.so", "libddsc.so.0", "libddscxx.so", "libddscxx.so.0")
)
PARAMETERS = ROOT / "config" / "go2_sensors.yaml"

provider = Primitive(id="go2_sensors", namespace="robonix/primitive/lidar")

# Keep this table literal and unconditional: package_manifest.yaml is a promise
# that every listed data-plane capability is declared after successful startup.
# CameraInfo remains a ROS diagnostic stream until a per-deployment calibration
# exists; an uncalibrated K[0] == 0 must not be advertised as intrinsics.
DATA_CAPABILITIES = (
    ("robonix/primitive/lidar/lidar3d", "lidar_output", "best_effort"),
    ("robonix/primitive/imu/imu", "imu_output", "best_effort"),
    ("robonix/primitive/camera/rgb", "camera_image", "best_effort"),
)

_lock = threading.Lock()
_config: dict[str, Any] = {}
_children: list[Any] = []
_declared = False


def _absolute_topic(value: Any, name: str, default: str) -> str:
    topic = str(value or default)
    if not topic.startswith("/") or topic == "/":
        raise ValueError(f"{name} must be an absolute ROS topic")
    return topic


def _positive_float(value: Any, name: str, default: float, maximum: float) -> float:
    result = float(default if value is None else value)
    if not 0.0 < result <= maximum:
        raise ValueError(f"{name} must be in (0, {maximum}]")
    return result


def _normalize_config(cfg: dict[str, Any]) -> dict[str, Any]:
    interface = str(cfg.get("network_interface") or "")
    if not interface or "/" in interface or interface in {".", ".."} or len(interface) >= 16:
        raise ValueError("network_interface must be an explicit Linux interface name")
    if socket.if_nametoindex(interface) == 0:
        raise ValueError(f"network interface does not exist: {interface}")

    socket_path = Path(
        str(cfg.get("camera_ipc_socket") or f"/run/user/{os.getuid()}/robonix-go2/camera.sock")
    )
    if not socket_path.is_absolute() or ".." in socket_path.parts or len(str(socket_path)) >= 100:
        raise ValueError("camera_ipc_socket must be a short absolute path")

    domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if not 0 <= domain_id <= 232:
        raise ValueError("ROS_DOMAIN_ID must be in 0..232")

    normalized = {
        "network_interface": interface,
        "domain_id": domain_id,
        "lidar_input": _absolute_topic(
            cfg.get("lidar_input_topic"), "lidar_input_topic", "/utlidar/cloud"
        ),
        "lidar_output": _absolute_topic(
            cfg.get("lidar_output_topic"), "lidar_output_topic", "/scanner/cloud"
        ),
        "imu_input": _absolute_topic(cfg.get("imu_input_topic"), "imu_input_topic", "/imu/data"),
        "imu_output": _absolute_topic(
            cfg.get("imu_output_topic"), "imu_output_topic", "/sensors/imu/data"
        ),
        "camera_image": _absolute_topic(
            cfg.get("camera_topic"), "camera_topic", "/camera/color/image_raw"
        ),
        "camera_info": _absolute_topic(
            cfg.get("camera_info_topic"),
            "camera_info_topic",
            "/camera/color/camera_info",
        ),
        "camera_frame": str(cfg.get("camera_frame") or "front_camera"),
        "camera_fps": _positive_float(cfg.get("camera_fps"), "camera_fps", 10.0, 30.0),
        "camera_socket": str(socket_path),
        "sentinel_timeout": _positive_float(
            cfg.get("sentinel_timeout_s"), "sentinel_timeout_s", 30.0, 60.0
        ),
    }
    if normalized["lidar_input"] == normalized["lidar_output"]:
        raise ValueError("lidar input and output topics must differ")
    if normalized["imu_input"] == normalized["imu_output"]:
        raise ValueError("IMU input and output topics must differ")
    if not normalized["camera_frame"] or normalized["camera_frame"].startswith("/"):
        raise ValueError("camera_frame must be a valid TF frame without a leading slash")
    return normalized


def _stop_children() -> None:
    global _children
    with _lock:
        children = _children
        _children = []
    for process in children:
        if process.poll() is not None:
            continue
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except OSError:
            continue
    deadline = time.monotonic() + 3.0
    for process in children:
        if process.poll() is not None:
            continue
        try:
            process.wait(timeout=max(0.0, deadline - time.monotonic()))
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=1.0)
            except Exception:
                pass


def _spawn(
    argv: list[str], log_name: str, *, child_env: dict[str, str] | None = None
) -> Any:
    process = provider.spawn(argv, env=child_env, log=log_name, cwd=ROOT)
    with _lock:
        _children.append(process)
    return process


@provider.on_init
def initialize(cfg: dict[str, Any]):
    global _config
    if not isinstance(cfg, dict):
        return Err("sensor config must be a JSON object")
    with _lock:
        if _children:
            return Err("cannot reinitialize while sensor child processes are active")
    try:
        normalized = _normalize_config(cfg)
    except (TypeError, ValueError, OSError) as error:
        return Err(str(error))
    executable_artifacts = [SENSOR_RELAY, CAMERA_BRIDGE, CAMERA_DAEMON]
    required_artifacts = [*executable_artifacts, *CAMERA_DDS_LIBRARIES, PARAMETERS]
    missing = [str(path) for path in required_artifacts if not path.is_file()]
    if missing:
        return Err("missing built sensor artifact(s): " + ", ".join(missing))
    not_executable = [str(path) for path in executable_artifacts if not os.access(path, os.X_OK)]
    if not_executable:
        return Err("sensor artifact(s) are not executable: " + ", ".join(not_executable))
    if _declared and _config and normalized != _config:
        return Err("sensor endpoints were already declared; restart the provider to change config")
    _config = normalized
    return Ok()


@provider.on_activate
def activate():
    global _declared
    if not _config:
        return Err("sensor provider was not initialized")
    if _children:
        return Deferred("sensor child processes are already active")

    cfg = _config
    ros_common = ["--ros-args", "--params-file", str(PARAMETERS)]
    relay_arguments = [
        str(SENSOR_RELAY),
        *ros_common,
        "-p",
        f"lidar.input_topic:={cfg['lidar_input']}",
        "-p",
        f"lidar.output_topic:={cfg['lidar_output']}",
        "-p",
        f"imu.input_topic:={cfg['imu_input']}",
        "-p",
        f"imu.output_topic:={cfg['imu_output']}",
    ]
    camera_arguments = [
        str(CAMERA_BRIDGE),
        *ros_common,
        "-p",
        f"camera.socket_path:={cfg['camera_socket']}",
        "-p",
        f"camera.frame_id:={cfg['camera_frame']}",
        "-p",
        f"camera.image_topic:={cfg['camera_image']}",
        "-p",
        f"camera.camera_info_topic:={cfg['camera_info']}",
    ]
    daemon_arguments = [
        str(CAMERA_DAEMON),
        "--interface",
        cfg["network_interface"],
        "--domain-id",
        str(cfg["domain_id"]),
        "--fps",
        str(cfg["camera_fps"]),
        "--socket",
        cfg["camera_socket"],
    ]

    try:
        _spawn(relay_arguments, "sensor-relay.log")
        # Override the inherited ROS LD_LIBRARY_PATH for this child only.  The
        # installed daemon also has $ORIGIN/../lib RPATH; this explicit child
        # environment prevents ROS's libddsc from winning loader precedence.
        _spawn(
            daemon_arguments,
            "camera-daemon.log",
            child_env={"LD_LIBRARY_PATH": str(CAMERA_LIBRARY_DIR)},
        )
        _spawn(camera_arguments, "camera-bridge.log")

        timeout = cfg["sentinel_timeout"]
        required = [
            (cfg["lidar_output"], "PointCloud2"),
            (cfg["imu_output"], "Imu"),
            (cfg["camera_image"], "Image"),
            # CameraInfo is intentionally still required as a truthful ROS
            # companion stream, even while K[0] == 0 means uncalibrated.
            (cfg["camera_info"], "CameraInfo"),
        ]
        deadline = time.monotonic() + timeout
        for topic, message_type in required:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not provider.wait_for_topic(topic, message_type, remaining):
                _stop_children()
                return Err(
                    f"no {message_type} sample on {topic} within the "
                    f"{timeout:.1f}s activation deadline"
                )

        if not _declared:
            for contract, topic_key, qos in DATA_CAPABILITIES:
                provider.declare_ros2_topic(contract, cfg[topic_key], qos=qos)
            _declared = True
    except Exception as error:
        _stop_children()
        return Err(f"sensor activation failed: {type(error).__name__}: {error}")
    return Ok()


@provider.on_deactivate
def deactivate():
    _stop_children()
    return Ok()


@provider.on_shutdown
def shutdown():
    _stop_children()
    return Ok()


if __name__ == "__main__":
    provider.run()
