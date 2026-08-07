"""ROS-independent RobotTrack protocol and command-plan core."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Mapping, Sequence
from urllib.parse import urlparse


WAYPOINT_COUNT = 8
WAYPOINT_DIMENSIONS = 3
WAYPOINT_STRATEGY = "first"
CONTROL_DT = 0.1
MAX_VX = 0.15
MAX_WZ = 0.30
# The package default stays at the first physical-test value above. Explicit
# live profiles may request a slightly stronger forward command, but never
# exceed the already-established downstream smoother/chassis contract.
MAX_CONFIGURED_VX = 0.50
MAX_CONFIGURED_WZ = 0.40
MAX_PLAN_AGE_S = 1.5
DEFAULT_DISPATCH_HZ = 50.0
MODEL_INPUT_MODE = "center_crop_height"
MODEL_CROP_SIZE = 384


class ProtocolError(ValueError):
    """The inference server returned a response outside the official contract."""


def _finite_float(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _absolute_topic(value: Any, name: str, default: str) -> str:
    topic = str(value or default)
    if (
        not topic.startswith("/")
        or topic == "/"
        or topic.endswith("/")
        or "//" in topic
        or any(character.isspace() for character in topic)
        or len(topic) > 255
    ):
        raise ValueError(f"{name} must be one normalized absolute ROS topic")
    return topic


def _positive_float(value: Any, name: str, default: float, maximum: float) -> float:
    result = _finite_float(default if value is None else value, name)
    if not 0.0 < result <= maximum:
        raise ValueError(f"{name} must be in (0, {maximum}]")
    return result


@dataclass(frozen=True)
class RuntimeConfig:
    """Normalized runtime settings shared by ROS and the Robonix provider."""

    mode: str = "dry-run"
    rgb_topic: str = "/go2/d435i/color/image_raw"
    command_topic: str = "/go2/robottrack/cmd_vel_raw"
    server_url: str = "http://127.0.0.1:5801/eval_dual"
    instruction: str = "Follow the person ahead"
    model_input_mode: str = MODEL_INPUT_MODE
    model_crop_size: int = MODEL_CROP_SIZE
    jpeg_quality: int = 60
    http_timeout_s: float = 30.0
    waypoint_strategy: str = WAYPOINT_STRATEGY
    control_dt: float = CONTROL_DT
    dispatch_hz: float = DEFAULT_DISPATCH_HZ
    max_plan_age_s: float = MAX_PLAN_AGE_S
    max_vx: float = MAX_VX
    max_wz: float = MAX_WZ
    nav_raw_topic: str = "/go2/robottrack/nav_cmd_vel_raw"
    robottrack_raw_topic: str = "/go2/robottrack/cmd_vel_raw"
    selected_output_topic: str = "/cmd_vel_nav"
    selected_source: str = "robottrack"
    source_max_age_s: float = 0.25

    @classmethod
    def from_mapping(cls, source: Mapping[str, Any] | None) -> "RuntimeConfig":
        cfg = dict(source or {})
        mux_value = cfg.get("source_mux", {})
        if mux_value is None:
            mux_value = {}
        if not isinstance(mux_value, Mapping):
            raise ValueError("source_mux must be a JSON object")
        mux = dict(mux_value)

        mode = str(cfg.get("mode") or "dry-run").strip().lower()
        if mode not in {"dry-run", "live"}:
            raise ValueError("mode must be exactly 'dry-run' or 'live'")

        server_url = str(cfg.get("server_url") or cls.server_url).strip()
        parsed = urlparse(server_url)
        if parsed.scheme not in {"http", "https"} or not parsed.hostname:
            raise ValueError("server_url must be an HTTP(S) URL")
        if parsed.username or parsed.password:
            raise ValueError("server_url must not contain credentials")
        if not parsed.path.endswith("/eval_dual"):
            raise ValueError("server_url must select the official /eval_dual endpoint")

        instruction = str(cfg.get("instruction") or cls.instruction).strip()
        if not instruction or len(instruction) > 1000:
            raise ValueError("instruction must contain 1..1000 characters")

        model_input_mode = str(
            cfg.get("model_input_mode") or MODEL_INPUT_MODE
        ).strip().lower()
        if model_input_mode != MODEL_INPUT_MODE:
            raise ValueError(
                "model_input_mode must be the official 'center_crop_height'"
            )
        model_crop_size = int(cfg.get("model_crop_size", MODEL_CROP_SIZE))
        if model_crop_size != MODEL_CROP_SIZE:
            raise ValueError("model_crop_size must be the official 384 pixels")

        jpeg_quality = int(cfg.get("jpeg_quality", cls.jpeg_quality))
        if not 1 <= jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in [1, 100]")

        strategy = str(cfg.get("waypoint_strategy") or WAYPOINT_STRATEGY).strip().lower()
        if strategy != WAYPOINT_STRATEGY:
            raise ValueError("waypoint_strategy must be 'first' for this integration")
        control_dt = _finite_float(cfg.get("control_dt", CONTROL_DT), "control_dt")
        if not math.isclose(control_dt, CONTROL_DT, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("control_dt must be 0.1 seconds")
        max_plan_age_s = _finite_float(
            cfg.get("max_plan_age_s", MAX_PLAN_AGE_S), "max_plan_age_s"
        )
        if not math.isclose(max_plan_age_s, MAX_PLAN_AGE_S, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("max_plan_age_s must be 1.5 seconds")

        max_vx = _positive_float(
            cfg.get("max_vx"),
            "max_vx",
            MAX_VX,
            MAX_CONFIGURED_VX,
        )
        max_wz = _positive_float(
            cfg.get("max_wz"),
            "max_wz",
            MAX_WZ,
            MAX_CONFIGURED_WZ,
        )
        selected_source = str(
            mux.get("selected_source", cfg.get("selected_source", "robottrack"))
        ).strip().lower()
        if selected_source not in {"navigation", "robottrack"}:
            raise ValueError("selected_source must be 'navigation' or 'robottrack'")

        robottrack_raw_topic = _absolute_topic(
            mux.get(
                "robottrack_input_topic",
                cfg.get("robottrack_raw_topic", cls.robottrack_raw_topic),
            ),
            "robottrack_raw_topic",
            cls.robottrack_raw_topic,
        )
        command_topic = _absolute_topic(
            cfg.get("command_topic"), "command_topic", cls.command_topic
        )
        if command_topic != robottrack_raw_topic:
            raise ValueError(
                "command_topic and source_mux.robottrack_input_topic must match"
            )

        return cls(
            mode=mode,
            rgb_topic=_absolute_topic(
                cfg.get("rgb_topic"), "rgb_topic", cls.rgb_topic
            ),
            command_topic=command_topic,
            server_url=server_url,
            instruction=instruction,
            model_input_mode=model_input_mode,
            model_crop_size=model_crop_size,
            jpeg_quality=jpeg_quality,
            http_timeout_s=_positive_float(
                cfg.get("http_timeout_s"), "http_timeout_s", cls.http_timeout_s, 120.0
            ),
            waypoint_strategy=strategy,
            control_dt=control_dt,
            dispatch_hz=_positive_float(
                cfg.get("dispatch_hz"), "dispatch_hz", DEFAULT_DISPATCH_HZ, 100.0
            ),
            max_plan_age_s=max_plan_age_s,
            max_vx=max_vx,
            max_wz=max_wz,
            nav_raw_topic=_absolute_topic(
                mux.get(
                    "nav_input_topic",
                    cfg.get("nav_raw_topic", cls.nav_raw_topic),
                ),
                "nav_raw_topic",
                cls.nav_raw_topic,
            ),
            robottrack_raw_topic=robottrack_raw_topic,
            selected_output_topic=_absolute_topic(
                mux.get(
                    "output_topic",
                    cfg.get("selected_output_topic", cls.selected_output_topic),
                ),
                "selected_output_topic",
                cls.selected_output_topic,
            ),
            selected_source=selected_source,
            source_max_age_s=_positive_float(
                mux.get("max_age_s", cfg.get("source_max_age_s")),
                "source_max_age_s",
                cls.source_max_age_s,
                MAX_PLAN_AGE_S,
            ),
        )

    def as_ros_parameters(self) -> dict[str, Any]:
        """Return the flat parameter map consumed by ``RobotTrackNode``."""

        return {
            field: getattr(self, field)
            for field in self.__dataclass_fields__
        }


@dataclass(frozen=True)
class VelocityCommand:
    vx: float = 0.0
    wz: float = 0.0

    def clipped(self, max_vx: float = MAX_VX, max_wz: float = MAX_WZ) -> "VelocityCommand":
        vx = _finite_float(self.vx, "vx")
        wz = _finite_float(self.wz, "wz")
        return VelocityCommand(
            vx=max(-float(max_vx), min(float(max_vx), vx)),
            wz=max(-float(max_wz), min(float(max_wz), wz)),
        )


ZERO_COMMAND = VelocityCommand()


@dataclass(frozen=True)
class EncodedFrame:
    sequence: int
    jpeg: bytes
    source_timestamp: float
    received_monotonic: float


class LatestFrameMailbox:
    """A one-slot mailbox: every incoming frame atomically replaces the prior one."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._latest: EncodedFrame | None = None
        self._sequence = 0

    def put(
        self,
        jpeg: bytes,
        *,
        source_timestamp: float,
        received_monotonic: float | None = None,
    ) -> EncodedFrame:
        payload = bytes(jpeg)
        if not payload:
            raise ValueError("jpeg frame must not be empty")
        stamp = _finite_float(source_timestamp, "source_timestamp")
        received = (
            time.monotonic()
            if received_monotonic is None
            else _finite_float(received_monotonic, "received_monotonic")
        )
        with self._condition:
            self._sequence += 1
            frame = EncodedFrame(self._sequence, payload, stamp, received)
            self._latest = frame
            self._condition.notify_all()
            return frame

    def latest(self) -> EncodedFrame | None:
        with self._condition:
            return self._latest

    def wait_after(
        self,
        sequence: int,
        *,
        timeout_s: float,
        stop_event: threading.Event | None = None,
    ) -> EncodedFrame | None:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            while self._latest is None or self._latest.sequence <= sequence:
                if stop_event is not None and stop_event.is_set():
                    return None
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(min(remaining, 0.1))
            return self._latest

    @property
    def retained_count(self) -> int:
        with self._condition:
            return 0 if self._latest is None else 1


Waypoint = tuple[float, float, float]


@dataclass(frozen=True)
class InferencePlan:
    command: VelocityCommand
    waypoints: tuple[Waypoint, ...] | None
    control_dt: float = CONTROL_DT
    velocity_source: str = "base_velocity"


def _parse_vector(value: Any, name: str, length: int) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ProtocolError(f"{name} must be an array of length {length}")
    if len(value) != length:
        raise ProtocolError(f"{name} must be an array of length {length}")
    try:
        return tuple(_finite_float(item, f"{name}[{index}]") for index, item in enumerate(value))
    except (TypeError, ValueError) as error:
        raise ProtocolError(str(error)) from error


def _parse_waypoints(value: Any) -> tuple[Waypoint, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ProtocolError("waypoints must be an 8x3 array")
    if len(value) != WAYPOINT_COUNT:
        raise ProtocolError("waypoints must contain exactly 8 rows")
    rows: list[Waypoint] = []
    for index, row in enumerate(value):
        vector = _parse_vector(row, f"waypoints[{index}]", WAYPOINT_DIMENSIONS)
        rows.append((vector[0], vector[1], vector[2]))
    return tuple(rows)


def parse_inference_response(
    response: Mapping[str, Any],
    *,
    max_vx: float = MAX_VX,
    max_wz: float = MAX_WZ,
) -> InferencePlan:
    """Parse official ``/eval_dual`` responses and return a bounded command.

    The full server response carries both ``waypoints`` and ``base_velocity``.
    The official server also supports a velocity-only trimmed response, so the
    bridge accepts either representation while validating every representation
    that is present. If velocity is omitted, the direct controller uses waypoint
    index 1 over the official 0.1 second horizon.
    """

    if not isinstance(response, Mapping):
        raise ProtocolError("inference response must be a JSON object")
    if response.get("error"):
        raise ProtocolError(f"inference server error: {response['error']}")

    waypoints_value = response.get("waypoints")
    if waypoints_value is None:
        waypoints_value = response.get("trajectory")
    waypoints = None if waypoints_value is None else _parse_waypoints(waypoints_value)

    velocity_value = response.get("base_velocity")
    velocity_name = "base_velocity"
    if velocity_value is None:
        velocity_value = response.get("velocity")
        velocity_name = "velocity"

    if velocity_value is not None:
        velocity = _parse_vector(velocity_value, velocity_name, 3)
        command = VelocityCommand(velocity[0], velocity[2]).clipped(max_vx, max_wz)
        source = velocity_name
    elif waypoints is not None:
        first_motion_waypoint = waypoints[1]
        command = VelocityCommand(
            first_motion_waypoint[0] / CONTROL_DT,
            first_motion_waypoint[2] / CONTROL_DT,
        ).clipped(max_vx, max_wz)
        source = "waypoints[1]"
    else:
        raise ProtocolError("response must contain waypoints/trajectory or velocity")

    if "control_dt" in response:
        try:
            server_dt = _finite_float(response["control_dt"], "control_dt")
        except (TypeError, ValueError) as error:
            raise ProtocolError(str(error)) from error
        if not math.isclose(server_dt, CONTROL_DT, rel_tol=0.0, abs_tol=1e-6):
            raise ProtocolError("server control_dt does not match the official 0.1 seconds")

    return InferencePlan(
        command=command,
        waypoints=waypoints,
        control_dt=CONTROL_DT,
        velocity_source=source,
    )


@dataclass(frozen=True)
class DispatchState:
    command: VelocityCommand
    reason: str
    age_s: float | None
    version: int


class PlanStore:
    """Thread-safe latest inference plan with the official 1.5 second expiry."""

    def __init__(
        self,
        *,
        max_plan_age_s: float = MAX_PLAN_AGE_S,
        max_vx: float = MAX_VX,
        max_wz: float = MAX_WZ,
    ) -> None:
        self._max_plan_age_s = float(max_plan_age_s)
        self._max_vx = float(max_vx)
        self._max_wz = float(max_wz)
        self._lock = threading.Lock()
        self._plan: InferencePlan | None = None
        self._arrival_monotonic = 0.0
        self._version = 0

    def update(self, plan: InferencePlan, *, now: float | None = None) -> int:
        arrival = time.monotonic() if now is None else _finite_float(now, "now")
        bounded = InferencePlan(
            command=plan.command.clipped(self._max_vx, self._max_wz),
            waypoints=plan.waypoints,
            control_dt=plan.control_dt,
            velocity_source=plan.velocity_source,
        )
        with self._lock:
            self._plan = bounded
            self._arrival_monotonic = arrival
            self._version += 1
            return self._version

    def clear(self) -> None:
        with self._lock:
            self._plan = None
            self._arrival_monotonic = 0.0
            self._version += 1

    def dispatch(self, *, now: float | None = None) -> DispatchState:
        current = time.monotonic() if now is None else _finite_float(now, "now")
        with self._lock:
            plan = self._plan
            arrival = self._arrival_monotonic
            version = self._version
        if plan is None:
            return DispatchState(ZERO_COMMAND, "no_plan", None, version)
        age = max(0.0, current - arrival)
        if age > self._max_plan_age_s:
            return DispatchState(ZERO_COMMAND, "stale_plan", age, version)
        return DispatchState(plan.command, "active_plan", age, version)
