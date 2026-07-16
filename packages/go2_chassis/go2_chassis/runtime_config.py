"""Strict, side-effect-free runtime configuration for the Go2 chassis.

Robonix delivers per-instance configuration only through Driver(CMD_INIT).
Keeping normalization in this dependency-free module makes the fail-closed
rules testable without ROS, Robonix, or Unitree SDK2 installed.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
import os
from pathlib import Path
import re
import stat
from typing import Mapping


SAFETY_ACK = "I_UNDERSTAND_GO2_CAN_MOVE"
UNKNOWN_MODE = 255
_TOPIC_RE = re.compile(r"^/[A-Za-z0-9_/]+$")
_FRAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_/]*$")
_INTERFACE_RE = re.compile(r"^[A-Za-z0-9_.:-]+$")

_KNOWN_KEYS = {
    "state_topic",
    "state_fallback_topic",
    "twist_in_topic",
    "odom_topic",
    "imu_topic",
    "status_topic",
    "diagnostics_topic",
    "arm_service",
    "odom_frame",
    "base_frame",
    "imu_frame",
    "velocity_frame",
    "network_interface",
    "ipc_socket",
    "allow_motion",
    "operator_present",
    "safety_ack",
    "allowed_modes",
    "max_linear_x_mps",
    "max_linear_y_mps",
    "max_angular_z_rps",
    "max_linear_accel_mps2",
    "max_angular_accel_rps2",
    "command_timeout_s",
    "state_timeout_s",
    "zero_preamble_s",
    "control_rate_hz",
    "max_position_jump_m",
    "startup_timeout_s",
}


class ConfigError(ValueError):
    """The delivered config is unsafe, ambiguous, or unsupported."""


def prepare_private_directory(path: Path) -> None:
    """Create or validate an owned, real directory and set it to mode 0700.

    ``Path.chmod`` follows symbolic links. The runtime socket parent is a
    security boundary, so validate with ``lstat`` before opening it with
    ``O_NOFOLLOW``, compare the opened inode, and chmod only that descriptor.
    """
    directory = Path(path)
    try:
        path_stat = os.lstat(directory)
    except FileNotFoundError:
        try:
            directory.mkdir(mode=0o700, parents=True, exist_ok=False)
        except FileExistsError:
            # Another actor may have installed an entry between lstat and
            # mkdir. The mandatory lstat below decides whether it is safe.
            pass
        path_stat = os.lstat(directory)

    current_uid = os.geteuid()
    if stat.S_ISLNK(path_stat.st_mode):
        raise ConfigError(f"private directory must not be a symlink: {directory}")
    if not stat.S_ISDIR(path_stat.st_mode):
        raise ConfigError(f"private directory is not a directory: {directory}")
    if path_stat.st_uid != current_uid:
        raise ConfigError(
            f"private directory is not owned by the current UID: {directory}"
        )

    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
    descriptor = os.open(directory, flags)
    try:
        opened_stat = os.fstat(descriptor)
        if (
            not stat.S_ISDIR(opened_stat.st_mode)
            or opened_stat.st_uid != current_uid
            or opened_stat.st_dev != path_stat.st_dev
            or opened_stat.st_ino != path_stat.st_ino
        ):
            raise ConfigError(
                f"private directory changed during validation: {directory}"
            )
        os.fchmod(descriptor, 0o700)
        secured_stat = os.fstat(descriptor)
        if stat.S_IMODE(secured_stat.st_mode) != 0o700:
            raise ConfigError(
                f"private directory mode could not be set to 0700: {directory}"
            )
    finally:
        os.close(descriptor)

    final_stat = os.lstat(directory)
    if (
        not stat.S_ISDIR(final_stat.st_mode)
        or final_stat.st_uid != current_uid
        or final_stat.st_dev != path_stat.st_dev
        or final_stat.st_ino != path_stat.st_ino
        or stat.S_IMODE(final_stat.st_mode) != 0o700
    ):
        raise ConfigError(
            f"private directory changed after it was secured: {directory}"
        )


def _boolean(value, name: str, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
    raise ConfigError(f"{name} must be an explicit boolean")


def _text(value, name: str, default: str = "", *, allow_empty: bool = False) -> str:
    if value is None:
        value = default
    if not isinstance(value, str):
        raise ConfigError(f"{name} must be a string")
    result = value.strip()
    if not result and not allow_empty:
        raise ConfigError(f"{name} must not be empty")
    return result


def _topic(value, name: str, default: str, *, allow_empty: bool = False) -> str:
    result = _text(value, name, default, allow_empty=allow_empty)
    if result and (_TOPIC_RE.fullmatch(result) is None or "//" in result):
        raise ConfigError(f"{name} is not a canonical absolute ROS topic")
    return result


def _frame(value, name: str, default: str) -> str:
    result = _text(value, name, default)
    if _FRAME_RE.fullmatch(result) is None or "//" in result:
        raise ConfigError(f"{name} is not a valid frame id")
    return result


def _number(
    value,
    name: str,
    default: float,
    *,
    minimum: float,
    maximum: float,
    include_minimum: bool = True,
) -> float:
    if value is None:
        value = default
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{name} must be a number")
    result = float(value)
    minimum_ok = result >= minimum if include_minimum else result > minimum
    if not math.isfinite(result) or not minimum_ok or result > maximum:
        boundary = "at least" if include_minimum else "greater than"
        raise ConfigError(
            f"{name} must be finite, {boundary} {minimum}, and at most {maximum}"
        )
    return result


def _mode_list(value, name: str) -> tuple[int, ...]:
    if not isinstance(value, (list, tuple)) or not value:
        raise ConfigError(f"{name} must be a non-empty list")
    modes: list[int] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, int) or not 0 <= item <= 255:
            raise ConfigError(f"{name} entries must be uint8 integers")
        if item not in modes:
            modes.append(item)
    return tuple(modes)


def parse_audited_modes(environ: Mapping[str, str]) -> tuple[int, ...]:
    """Parse the operator-supplied modes observed during read-only auditing."""
    raw = environ.get("GO2_ALLOWED_MODES", "").strip()
    if not raw:
        raise ConfigError(
            "motion requires explicit GO2_ALLOWED_MODES from the read-only audit"
        )
    tokens = [token.strip() for token in raw.replace(";", ",").split(",")]
    if any(not token for token in tokens):
        raise ConfigError("GO2_ALLOWED_MODES contains an empty entry")
    modes: list[int] = []
    for token in tokens:
        if not token.isdecimal():
            raise ConfigError("GO2_ALLOWED_MODES entries must be decimal uint8 values")
        mode = int(token, 10)
        if not 0 <= mode < UNKNOWN_MODE:
            raise ConfigError("GO2_ALLOWED_MODES entries must be between 0 and 254")
        if mode not in modes:
            modes.append(mode)
    return tuple(modes)


@dataclass(frozen=True, slots=True)
class RuntimeConfig:
    state_topic: str
    state_fallback_topic: str
    twist_in_topic: str
    odom_topic: str
    imu_topic: str
    status_topic: str
    diagnostics_topic: str
    arm_service: str
    odom_frame: str
    base_frame: str
    imu_frame: str
    velocity_frame: str
    network_interface: str
    ipc_socket: Path
    allow_motion: bool
    allowed_modes: tuple[int, ...]
    max_linear_x_mps: float
    max_linear_y_mps: float
    max_angular_z_rps: float
    max_linear_accel_mps2: float
    max_angular_accel_rps2: float
    command_timeout_s: float
    state_timeout_s: float
    zero_preamble_s: float
    control_rate_hz: float
    max_position_jump_m: float
    startup_timeout_s: float

    @property
    def starts_sdk_daemon(self) -> bool:
        return self.allow_motion

    def process_env(self) -> dict[str, str]:
        return {
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "GO2_ALLOW_MOTION": "1" if self.allow_motion else "0",
            "GO2_NETWORK_INTERFACE": self.network_interface,
            "GO2_SDK_SOCKET": str(self.ipc_socket),
            "GO2_ALLOWED_MODES": ",".join(str(mode) for mode in self.allowed_modes),
        }

    def adapter_argv(self, executable: Path, params_file: Path) -> list[str]:
        modes = "[" + ",".join(str(mode) for mode in self.allowed_modes) + "]"
        parameters = {
            "allow_motion": "true" if self.allow_motion else "false",
            "sport_state_topic": self.state_topic,
            "state_fallback_topic": self.state_fallback_topic,
            "cmd_vel_topic": self.twist_in_topic,
            "odom_topic": self.odom_topic,
            "imu_topic": self.imu_topic,
            "status_topic": self.status_topic,
            "diagnostics_topic": self.diagnostics_topic,
            "arm_service": self.arm_service,
            "odom_frame": self.odom_frame,
            "base_frame": self.base_frame,
            "imu_frame": self.imu_frame,
            "state_velocity_frame": self.velocity_frame,
            "sdk_socket": str(self.ipc_socket),
            "allowed_modes": modes,
            "state_timeout_sec": str(self.state_timeout_s),
            "command_timeout_sec": str(self.command_timeout_s),
            "zero_preparation_sec": str(self.zero_preamble_s),
            "control_rate_hz": str(self.control_rate_hz),
            "max_vx": str(self.max_linear_x_mps),
            "max_vy": str(self.max_linear_y_mps),
            "max_wz": str(self.max_angular_z_rps),
            "max_linear_acceleration": str(self.max_linear_accel_mps2),
            "max_angular_acceleration": str(self.max_angular_accel_rps2),
            "max_position_jump_m": str(self.max_position_jump_m),
        }
        argv = [str(executable), "--ros-args", "--params-file", str(params_file)]
        for name, value in parameters.items():
            argv.extend(("-p", f"{name}:={value}"))
        return argv

    def daemon_argv(self, executable: Path) -> list[str]:
        if not self.allow_motion:
            raise ConfigError("SDK daemon must not start while motion is disabled")
        return [
            str(executable),
            "--socket",
            str(self.ipc_socket),
            "--watchdog-ms",
            "300",
            "--allow-motion",
            "--interface",
            self.network_interface,
            "--motion-ack",
            "GO2_PHYSICAL_MOTION_APPROVED",
        ]


def normalize_config(
    config: Mapping[str, object] | None,
    environ: Mapping[str, str] | None,
    package_root: Path,
) -> RuntimeConfig:
    if config is None:
        config = {}
    if not isinstance(config, Mapping):
        raise ConfigError("Driver config must be a mapping")
    unknown = sorted(set(config) - _KNOWN_KEYS)
    if unknown:
        raise ConfigError(f"unknown config keys: {', '.join(unknown)}")
    env = os.environ if environ is None else environ

    allow_motion = _boolean(config.get("allow_motion"), "allow_motion", False)
    operator_present = _boolean(
        config.get("operator_present"), "operator_present", False
    )
    safety_ack = _text(
        config.get("safety_ack"), "safety_ack", "", allow_empty=True
    )
    configured_modes = _mode_list(
        config.get("allowed_modes", [UNKNOWN_MODE]), "allowed_modes"
    )
    if configured_modes != (UNKNOWN_MODE,):
        raise ConfigError(
            "Driver allowed_modes must remain [255]; use GO2_ALLOWED_MODES "
            "only after a read-only audit"
        )

    network_interface = _text(
        config.get("network_interface"),
        "network_interface",
        "",
        allow_empty=not allow_motion,
    )
    if network_interface and _INTERFACE_RE.fullmatch(network_interface) is None:
        raise ConfigError("network_interface contains unsupported characters")

    if allow_motion:
        if not operator_present:
            raise ConfigError("motion requires operator_present=true")
        if safety_ack != SAFETY_ACK:
            raise ConfigError("motion requires the exact safety_ack phrase")
        if not network_interface:
            raise ConfigError("motion requires a dedicated network_interface")
        allowed_modes = parse_audited_modes(env)
    else:
        # A stale manifest list can never authorize motion. Until an operator
        # enables all gates and supplies audited modes, only the impossible
        # uint8 sentinel is injected into the ROS guard.
        allowed_modes = (UNKNOWN_MODE,)

    package_hash = hashlib.sha256(str(package_root.resolve()).encode()).hexdigest()[:12]
    socket_default = (
        Path("/tmp")
        / f"robonix-go2-{os.getuid()}-{package_hash}"
        / "sport.sock"
    )
    socket_text = _text(
        config.get("ipc_socket"), "ipc_socket", str(socket_default)
    )
    ipc_socket = Path(socket_text)
    if not ipc_socket.is_absolute() or len(str(ipc_socket)) >= 108:
        raise ConfigError("ipc_socket must be an absolute Unix socket path under 108 bytes")

    state_topic = _topic(
        config.get("state_topic"), "state_topic", "/sportmodestate"
    )
    state_fallback_topic = _topic(
        config.get("state_fallback_topic"),
        "state_fallback_topic",
        "/lf/sportmodestate",
        allow_empty=True,
    )
    if state_fallback_topic == state_topic:
        state_fallback_topic = ""

    velocity_frame = _text(
        config.get("velocity_frame"), "velocity_frame", "odom"
    )
    if velocity_frame not in {"odom", "base_link"}:
        raise ConfigError("velocity_frame must be 'odom' or 'base_link'")

    return RuntimeConfig(
        state_topic=state_topic,
        state_fallback_topic=state_fallback_topic,
        twist_in_topic=_topic(
            config.get("twist_in_topic"), "twist_in_topic", "/cmd_vel"
        ),
        odom_topic=_topic(config.get("odom_topic"), "odom_topic", "/odom"),
        imu_topic=_topic(config.get("imu_topic"), "imu_topic", "/imu/data"),
        status_topic=_topic(
            config.get("status_topic"), "status_topic", "/go2_chassis/status"
        ),
        diagnostics_topic=_topic(
            config.get("diagnostics_topic"), "diagnostics_topic", "/diagnostics"
        ),
        arm_service=_topic(
            config.get("arm_service"), "arm_service", "/go2_chassis/arm"
        ),
        odom_frame=_frame(config.get("odom_frame"), "odom_frame", "odom"),
        base_frame=_frame(
            config.get("base_frame"), "base_frame", "base_link"
        ),
        imu_frame=_frame(config.get("imu_frame"), "imu_frame", "imu"),
        velocity_frame=velocity_frame,
        network_interface=network_interface,
        ipc_socket=ipc_socket,
        allow_motion=allow_motion,
        allowed_modes=allowed_modes,
        max_linear_x_mps=_number(
            config.get("max_linear_x_mps"),
            "max_linear_x_mps",
            0.25,
            minimum=0.0,
            maximum=0.25,
            include_minimum=False,
        ),
        max_linear_y_mps=_number(
            config.get("max_linear_y_mps"),
            "max_linear_y_mps",
            0.0,
            minimum=0.0,
            maximum=0.0,
        ),
        max_angular_z_rps=_number(
            config.get("max_angular_z_rps"),
            "max_angular_z_rps",
            0.40,
            minimum=0.0,
            maximum=0.40,
            include_minimum=False,
        ),
        max_linear_accel_mps2=_number(
            config.get("max_linear_accel_mps2"),
            "max_linear_accel_mps2",
            0.30,
            minimum=0.0,
            maximum=0.30,
            include_minimum=False,
        ),
        max_angular_accel_rps2=_number(
            config.get("max_angular_accel_rps2"),
            "max_angular_accel_rps2",
            0.80,
            minimum=0.0,
            maximum=0.80,
            include_minimum=False,
        ),
        command_timeout_s=_number(
            config.get("command_timeout_s"),
            "command_timeout_s",
            0.25,
            minimum=0.05,
            maximum=0.25,
        ),
        state_timeout_s=_number(
            config.get("state_timeout_s"),
            "state_timeout_s",
            0.20,
            minimum=0.05,
            maximum=0.50,
        ),
        zero_preamble_s=_number(
            config.get("zero_preamble_s"),
            "zero_preamble_s",
            0.50,
            minimum=0.50,
            maximum=5.0,
        ),
        control_rate_hz=_number(
            config.get("control_rate_hz"),
            "control_rate_hz",
            50.0,
            minimum=10.0,
            maximum=100.0,
        ),
        max_position_jump_m=_number(
            config.get("max_position_jump_m"),
            "max_position_jump_m",
            1.0,
            minimum=0.10,
            maximum=2.0,
        ),
        startup_timeout_s=_number(
            config.get("startup_timeout_s"),
            "startup_timeout_s",
            15.0,
            minimum=1.0,
            maximum=60.0,
        ),
    )
