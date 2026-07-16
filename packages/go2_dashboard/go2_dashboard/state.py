"""Thread-safe state shared by the ROS observer and HTTP server."""

from __future__ import annotations

import copy
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping


SEMANTIC_STATUSES = frozenset(
    {
        "idle",
        "received",
        "resolving",
        "resolved",
        "navigating",
        "succeeded",
        "canceled",
        "failed",
    }
)


@dataclass(frozen=True)
class TopicSpec:
    label: str
    topic: str
    stale_after_s: float


DEFAULT_TOPIC_SPECS = {
    "camera": TopicSpec("相机", "/camera/color/image_raw", 2.0),
    "laser_scan": TopicSpec("二维雷达", "/scanner/scan", 2.0),
    "point_cloud": TopicSpec("三维雷达", "/scanner/cloud", 2.0),
    "map": TopicSpec("地图", "/map", 15.0),
    "odom": TopicSpec("里程计", "/odom", 1.5),
    "pose_map": TopicSpec("机器人地图位姿", "TF map -> base_link", 1.5),
    "nav_status": TopicSpec(
        "导航任务", "/navigate_to_pose/_action/status", 8.0
    ),
}


def _bounded_text(value: Any, limit: int) -> str:
    return str(value or "").strip()[:limit]


class DashboardState:
    """Own the last bounded preview and health metadata for every input."""

    def __init__(
        self,
        topic_specs: Mapping[str, TopicSpec] | None = None,
        monotonic_fn: Callable[[], float] = time.monotonic,
        wall_fn: Callable[[], float] = time.time,
    ) -> None:
        self._lock = threading.RLock()
        self._monotonic = monotonic_fn
        self._wall = wall_fn
        self._started_monotonic = float(monotonic_fn())
        self._topic_specs = dict(topic_specs or DEFAULT_TOPIC_SPECS)
        self._topic_health = {
            key: {
                "last_seen_monotonic": None,
                "last_seen_at": None,
                "sequence": 0,
                "frame_id": "",
                "error": "",
            }
            for key in self._topic_specs
        }
        self._payload: dict[str, Any] = {
            "camera": None,
            "lidar": None,
            "map": None,
            "pose_map": None,
            "odom": None,
            "navigation": {
                "status": "idle",
                "status_code": 0,
                "goal_id": "",
                "stamp": None,
            },
        }
        self._last_lidar_key: str | None = None
        self._camera_jpeg: bytes | None = None
        self._map_png: bytes | None = None
        self._bridge = {
            "running": False,
            "connected": False,
            "error": "尚未启动 ROS 观察线程",
        }
        self._semantic_task = {
            "revision": 0,
            "task_id": "",
            "target_name": "",
            "status": "idle",
            "message": "",
            "pose": None,
            "updated_at": float(wall_fn()),
            "source": "status-api",
            "read_only_effect": True,
        }

    def set_bridge(
        self, *, running: bool, connected: bool, error: str = ""
    ) -> None:
        with self._lock:
            self._bridge = {
                "running": bool(running),
                "connected": bool(connected),
                "error": _bounded_text(error, 400),
            }

    def observe(
        self,
        key: str,
        payload: Any,
        *,
        frame_id: str = "",
        monotonic_now: float | None = None,
        wall_now: float | None = None,
    ) -> int:
        with self._lock:
            if key not in self._topic_health:
                raise KeyError(f"unknown telemetry key: {key}")
            health = self._topic_health[key]
            health["last_seen_monotonic"] = float(
                self._monotonic() if monotonic_now is None else monotonic_now
            )
            health["last_seen_at"] = float(
                self._wall() if wall_now is None else wall_now
            )
            health["sequence"] += 1
            health["frame_id"] = _bounded_text(frame_id, 160)
            health["error"] = ""
            if key in {"laser_scan", "point_cloud"}:
                self._last_lidar_key = key
                self._payload["lidar"] = copy.deepcopy(payload)
            elif key == "nav_status":
                self._payload["navigation"] = copy.deepcopy(payload)
            else:
                self._payload[key] = copy.deepcopy(payload)
            return int(health["sequence"])

    def note_error(self, key: str, error: Any) -> None:
        with self._lock:
            if key not in self._topic_health:
                raise KeyError(f"unknown telemetry key: {key}")
            self._topic_health[key]["error"] = _bounded_text(error, 400)

    def set_camera(
        self, jpeg: bytes, metadata: Mapping[str, Any], *, frame_id: str
    ) -> int:
        if not jpeg:
            raise ValueError("camera preview is empty")
        with self._lock:
            sequence = self.observe("camera", dict(metadata), frame_id=frame_id)
            self._camera_jpeg = bytes(jpeg)
            self._payload["camera"]["sequence"] = sequence
            return sequence

    def set_map(
        self, png: bytes, metadata: Mapping[str, Any], *, frame_id: str
    ) -> int:
        if not png:
            raise ValueError("map preview is empty")
        with self._lock:
            sequence = self.observe("map", dict(metadata), frame_id=frame_id)
            self._map_png = bytes(png)
            self._payload["map"]["sequence"] = sequence
            return sequence

    def camera_image(self) -> tuple[bytes | None, int]:
        with self._lock:
            sequence = int(self._topic_health["camera"]["sequence"])
            return self._camera_jpeg, sequence

    def map_image(self) -> tuple[bytes | None, int]:
        with self._lock:
            sequence = int(self._topic_health["map"]["sequence"])
            return self._map_png, sequence

    def semantic_task(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._semantic_task)

    def update_semantic_task(self, update: Mapping[str, Any]) -> dict[str, Any]:
        status = _bounded_text(update.get("status"), 32).lower()
        if status not in SEMANTIC_STATUSES:
            raise ValueError(f"unsupported semantic task status: {status}")
        pose = update.get("pose")
        normalized_pose = None
        if pose is not None:
            normalized_pose = {
                "frame_id": _bounded_text(pose.get("frame_id", "map"), 80)
                or "map",
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "yaw": float(pose["yaw"]),
            }
            if not all(
                math.isfinite(value)
                for key, value in normalized_pose.items()
                if key != "frame_id"
            ):
                raise ValueError("semantic pose contains a non-finite value")
        with self._lock:
            self._semantic_task = {
                "revision": int(self._semantic_task["revision"]) + 1,
                "task_id": _bounded_text(update.get("task_id"), 128),
                "target_name": _bounded_text(update.get("target_name"), 128),
                "status": status,
                "message": _bounded_text(update.get("message"), 500),
                "pose": normalized_pose,
                "updated_at": float(self._wall()),
                "source": "status-api",
                "read_only_effect": True,
            }
            return copy.deepcopy(self._semantic_task)

    def snapshot(self) -> dict[str, Any]:
        now = float(self._monotonic())
        wall_now = float(self._wall())
        with self._lock:
            topics: dict[str, Any] = {}
            for key, spec in self._topic_specs.items():
                health = self._topic_health[key]
                last_seen = health["last_seen_monotonic"]
                age = None if last_seen is None else max(0.0, now - float(last_seen))
                if health["error"]:
                    state = "error"
                elif age is None:
                    state = "missing"
                elif age <= spec.stale_after_s:
                    state = "fresh"
                else:
                    state = "stale"
                topics[key] = {
                    "label": spec.label,
                    "topic": spec.topic,
                    "state": state,
                    "age_s": None if age is None else round(age, 3),
                    "stale_after_s": spec.stale_after_s,
                    "last_seen_at": health["last_seen_at"],
                    "sequence": health["sequence"],
                    "frame_id": health["frame_id"],
                    "error": health["error"],
                }
            result = {
                "schema_version": 1,
                "generated_at": wall_now,
                "uptime_s": round(max(0.0, now - self._started_monotonic), 3),
                "read_only": True,
                "bridge": copy.deepcopy(self._bridge),
                "topics": topics,
                "camera": copy.deepcopy(self._payload["camera"]),
                "lidar": copy.deepcopy(self._payload["lidar"]),
                "lidar_source": self._last_lidar_key,
                "map": copy.deepcopy(self._payload["map"]),
                "pose_map": copy.deepcopy(self._payload["pose_map"]),
                "odom": copy.deepcopy(self._payload["odom"]),
                "navigation": copy.deepcopy(self._payload["navigation"]),
                "semantic_task": copy.deepcopy(self._semantic_task),
            }
        return result
