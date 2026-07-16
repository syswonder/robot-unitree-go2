#!/usr/bin/env python3
"""Offline check that the provider fulfils every manifest data capability."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import types


ROOT = Path(__file__).resolve().parents[1]
PROVIDER_PATH = ROOT / "go2_sensors_provider" / "main.py"


class FakeProcess:
    _next_pid = 100000

    def __init__(self) -> None:
        self.pid = FakeProcess._next_pid
        FakeProcess._next_pid += 1

    def poll(self):
        return None


class FakePrimitive:
    def __init__(self, *, id: str, namespace: str) -> None:
        self.id = id
        self.namespace = namespace
        self.spawned: list[tuple[list[str], dict[str, str] | None, str, Path]] = []
        self.waited: list[tuple[str, str, float]] = []
        self.declared: list[tuple[str, str, str]] = []

    def on_init(self, callback):
        self.init_callback = callback
        return callback

    def on_activate(self, callback):
        self.activate_callback = callback
        return callback

    def on_deactivate(self, callback):
        self.deactivate_callback = callback
        return callback

    def on_shutdown(self, callback):
        self.shutdown_callback = callback
        return callback

    def spawn(self, argv, *, env=None, log: str, cwd: Path):
        self.spawned.append((list(argv), env, log, cwd))
        return FakeProcess()

    def wait_for_topic(self, topic: str, message_type: str, timeout: float) -> bool:
        self.waited.append((topic, message_type, timeout))
        return True

    def declare_ros2_topic(self, contract: str, topic: str, *, qos: str) -> None:
        self.declared.append((contract, topic, qos))

    def run(self) -> None:
        raise AssertionError("offline contract test must not run the provider")


fake_api = types.ModuleType("robonix_api")
fake_api.Primitive = FakePrimitive
fake_api.Ok = lambda: ("ok", None)
fake_api.Err = lambda message: ("err", message)
fake_api.Deferred = lambda message: ("deferred", message)
sys.modules["robonix_api"] = fake_api

spec = importlib.util.spec_from_file_location("go2_sensors_provider_contract", PROVIDER_PATH)
if spec is None or spec.loader is None:
    raise SystemExit("could not load provider module")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

artifacts = tempfile.TemporaryDirectory(prefix="go2-sensors-contract-")
artifact_root = Path(artifacts.name)
module.SENSOR_RELAY = artifact_root / "ros" / "go2_sensor_relay"
module.CAMERA_BRIDGE = artifact_root / "ros" / "go2_camera_bridge"
module.CAMERA_INSTALL = artifact_root / "camera"
module.CAMERA_LIBRARY_DIR = module.CAMERA_INSTALL / "lib"
module.CAMERA_DAEMON = module.CAMERA_INSTALL / "bin" / "go2_camera_daemon"
module.CAMERA_DDS_LIBRARIES = tuple(
    module.CAMERA_LIBRARY_DIR / name
    for name in ("libddsc.so", "libddsc.so.0", "libddscxx.so", "libddscxx.so.0")
)
module.PARAMETERS = artifact_root / "go2_sensors.yaml"
for executable in (module.SENSOR_RELAY, module.CAMERA_BRIDGE, module.CAMERA_DAEMON):
    executable.parent.mkdir(parents=True, exist_ok=True)
    executable.write_text("offline placeholder\n", encoding="utf-8")
    executable.chmod(0o700)
for library in module.CAMERA_DDS_LIBRARIES:
    library.parent.mkdir(parents=True, exist_ok=True)
    library.write_bytes(b"offline library placeholder\n")
    library.chmod(0o600)
module.PARAMETERS.write_text("offline: true\n", encoding="utf-8")

config = {
    "network_interface": "offline0",
    "lidar_input_topic": "/vendor/cloud",
    "lidar_output_topic": "/scanner/cloud",
    "imu_input_topic": "/vendor/imu",
    "imu_output_topic": "/scanner/imu",
    "camera_topic": "/camera/color/image_raw",
    "camera_info_topic": "/camera/color/camera_info",
    "camera_frame": "front_camera",
    "camera_fps": 10.0,
    "camera_ipc_socket": "/tmp/offline-go2-camera.sock",
    "sentinel_timeout_s": 30.0,
}
original_if_nametoindex = module.socket.if_nametoindex
module.socket.if_nametoindex = lambda interface: 1 if interface == "offline0" else 0
try:
    init_result = module.initialize(config)
finally:
    module.socket.if_nametoindex = original_if_nametoindex
if init_result != ("ok", None):
    raise SystemExit(f"provider initialization unexpectedly failed: {init_result!r}")
if any(os.access(path, os.X_OK) for path in module.CAMERA_DDS_LIBRARIES):
    raise SystemExit("offline test fixture unexpectedly made a DDS library executable")

result = module.activate()
if result != ("ok", None):
    raise SystemExit(f"provider activation unexpectedly failed: {result!r}")

provider = module.provider
if [entry[2] for entry in provider.spawned] != [
    "sensor-relay.log",
    "camera-daemon.log",
    "camera-bridge.log",
]:
    raise SystemExit("provider did not start all three mandatory read-only processes")

relay_env, daemon_env, bridge_env = [entry[1] for entry in provider.spawned]
if relay_env is not None or bridge_env is not None:
    raise SystemExit("vendor DDS environment leaked into a ROS child process")
expected_daemon_env = {"LD_LIBRARY_PATH": str(module.CAMERA_LIBRARY_DIR)}
if daemon_env != expected_daemon_env:
    raise SystemExit(f"camera daemon did not receive its private DDS path: {daemon_env!r}")

waited_streams = {(topic, message_type) for topic, message_type, _ in provider.waited}
expected_streams = {
    ("/scanner/cloud", "PointCloud2"),
    ("/scanner/imu", "Imu"),
    ("/camera/color/image_raw", "Image"),
    ("/camera/color/camera_info", "CameraInfo"),
}
if waited_streams != expected_streams:
    raise SystemExit(f"unexpected activation sentinels: {sorted(waited_streams)}")

expected_declarations = {
    ("robonix/primitive/lidar/lidar3d", "/scanner/cloud", "best_effort"),
    ("robonix/primitive/imu/imu", "/scanner/imu", "best_effort"),
    ("robonix/primitive/camera/rgb", "/camera/color/image_raw", "best_effort"),
}
if set(provider.declared) != expected_declarations:
    raise SystemExit(f"unexpected runtime capability declarations: {provider.declared!r}")
if any("intrinsics" in contract for contract, _, _ in provider.declared):
    raise SystemExit("uncalibrated CameraInfo was exposed as an intrinsics capability")

print("provider manifest/runtime contract test passed")
