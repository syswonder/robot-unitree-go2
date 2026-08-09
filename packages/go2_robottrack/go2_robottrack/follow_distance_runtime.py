"""Thread-safe runtime state for optional fixed-distance RobotTrack following.

The ordinary RobotTrack profile leaves this runtime disabled, so commands pass
through unchanged.  A fixed-distance profile configures the shared instance
before the ROS worker starts; Robonix RPC callbacks may then update the target
while the worker applies the controller and publishes diagnostics.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
from typing import Any

from .core import RuntimeConfig, VelocityCommand
from .distance_control import (
    DistanceControlConfig,
    DistanceControlResult,
    DistanceMeasurement,
    FixedDistanceController,
)


MIN_TARGET_DISTANCE_M = 0.5
MAX_TARGET_DISTANCE_M = 7.0


def _finite_float(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


@dataclass(frozen=True)
class FollowDistanceSnapshot:
    """One immutable view shared by the provider, ROS node, and UI."""

    enabled: bool
    active: bool
    target_distance_m: float
    measured_distance_m: float | None
    status: str
    detail: str
    measurement_confidence: float | None = None
    measurement_source: str = ""
    revision: int = 0


@dataclass(frozen=True)
class FollowDistanceCommandResult:
    """Result of a Robonix ``set``, ``adjust``, or ``get`` operation."""

    accepted: bool
    snapshot: FollowDistanceSnapshot
    detail: str


class FollowDistanceRuntime:
    """Own a fixed-distance controller plus live state under one lock."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._controller = FixedDistanceController()
        self._min_target_m = MIN_TARGET_DISTANCE_M
        self._max_target_m = MAX_TARGET_DISTANCE_M
        self._active = False
        self._measurement: DistanceMeasurement | None = None
        self._measurement_source = ""
        self._status = "disabled"
        self._detail = "fixed-distance control is disabled"
        self._revision = 0

    def configure(
        self,
        config: DistanceControlConfig,
        *,
        min_target_m: float = MIN_TARGET_DISTANCE_M,
        max_target_m: float = MAX_TARGET_DISTANCE_M,
    ) -> FollowDistanceSnapshot:
        """Replace configuration and clear transient measurement state."""

        minimum = _finite_float(min_target_m, "min_target_m")
        maximum = _finite_float(max_target_m, "max_target_m")
        if minimum <= 0.0:
            raise ValueError("min_target_m must be greater than zero")
        if maximum < minimum:
            raise ValueError("max_target_m must be at least min_target_m")
        target = _finite_float(config.target_distance_m, "target_distance_m")
        if not minimum <= target <= maximum:
            raise ValueError(
                f"target_distance_m must be in [{minimum:.2f}, {maximum:.2f}]"
            )

        with self._lock:
            self._controller = FixedDistanceController(config)
            self._min_target_m = minimum
            self._max_target_m = maximum
            self._active = False
            self._measurement = None
            self._measurement_source = ""
            if config.enabled:
                self._status = "ready"
                self._detail = "fixed-distance control is configured"
            else:
                self._status = "disabled"
                self._detail = "fixed-distance control is disabled"
            self._revision += 1
            return self._snapshot_locked()

    def set_active(self, active: bool, detail: str = "") -> FollowDistanceSnapshot:
        """Record whether the owning RobotTrack lifecycle is active."""

        with self._lock:
            enabled = self._controller.config.enabled
            self._active = bool(active) and enabled
            if not enabled:
                self._status = "disabled"
                self._detail = "fixed-distance control is disabled"
            elif self._active:
                self._status = "waiting_for_measurement"
                self._detail = detail or "waiting for a target distance measurement"
            else:
                self._status = "inactive"
                self._detail = detail or "RobotTrack runtime is inactive"
            self._revision += 1
            return self._snapshot_locked()

    def update_measurement(
        self,
        measurement: DistanceMeasurement | None,
        *,
        status: str = "measurement_updated",
        source: str = "",
        detail: str = "",
    ) -> FollowDistanceSnapshot:
        """Publish the most recent target observation for control and status."""

        if measurement is not None and not isinstance(
            measurement, DistanceMeasurement
        ):
            raise TypeError("measurement must be DistanceMeasurement or None")
        with self._lock:
            self._measurement = measurement
            self._measurement_source = str(source).strip()
            self._status = str(status).strip() or "measurement_updated"
            self._detail = str(detail).strip()
            self._revision += 1
            return self._snapshot_locked()

    def apply(
        self,
        model_command: VelocityCommand,
        measurement: DistanceMeasurement | None = None,
        *,
        now_monotonic: float | None = None,
    ) -> DistanceControlResult:
        """Apply the current target while retaining an inspectable snapshot."""

        with self._lock:
            controller = self._controller
            enabled = controller.config.enabled
            active = self._active
            if measurement is not None:
                self._measurement = measurement
                self._revision += 1
            selected_measurement = (
                measurement if measurement is not None else self._measurement
            )

        if enabled and not active:
            result = DistanceControlResult(
                command=VelocityCommand(vx=0.0, wz=model_command.wz),
                reason="inactive",
                target_distance_m=controller.target_distance_m,
            )
        else:
            result = controller.apply(
                model_command,
                selected_measurement,
                now_monotonic=now_monotonic,
            )

        with self._lock:
            # Ignore diagnostics from a controller replaced concurrently by a
            # lifecycle reinitialization; the new configuration already owns
            # the authoritative snapshot.
            if controller is self._controller:
                self._status = result.reason
                self._detail = self._detail_for_result(result)
                self._revision += 1
        return result

    def snapshot(self) -> FollowDistanceSnapshot:
        with self._lock:
            return self._snapshot_locked()

    def set_target_distance(self, distance_m: float) -> FollowDistanceSnapshot:
        target = _finite_float(distance_m, "distance_m")
        with self._lock:
            self._require_enabled_locked()
            self._validate_target_locked(target)
            self._controller.set_target_distance(target)
            self._status = "target_updated"
            self._detail = f"target distance set to {target:.2f} m"
            self._revision += 1
            return self._snapshot_locked()

    def adjust_target_distance(self, delta_m: float) -> FollowDistanceSnapshot:
        delta = _finite_float(delta_m, "distance_m")
        with self._lock:
            self._require_enabled_locked()
            target = self._controller.target_distance_m + delta
            self._validate_target_locked(target)
            self._controller.set_target_distance(target)
            self._status = "target_updated"
            self._detail = (
                f"target distance adjusted by {delta:+.2f} m to {target:.2f} m"
            )
            self._revision += 1
            return self._snapshot_locked()

    def handle(self, operation: str, distance_m: float = 0.0) -> FollowDistanceCommandResult:
        """Handle a stable, transport-neutral runtime operation."""

        normalized = str(operation).strip().casefold()
        if normalized == "get":
            snapshot = self.snapshot()
            return FollowDistanceCommandResult(True, snapshot, snapshot.detail)
        try:
            if normalized == "set":
                snapshot = self.set_target_distance(distance_m)
            elif normalized == "adjust":
                snapshot = self.adjust_target_distance(distance_m)
            else:
                raise ValueError("operation must be one of: set, adjust, get")
        except (TypeError, ValueError) as error:
            snapshot = self.snapshot()
            return FollowDistanceCommandResult(False, snapshot, str(error))
        return FollowDistanceCommandResult(True, snapshot, snapshot.detail)

    def _require_enabled_locked(self) -> None:
        if not self._controller.config.enabled:
            raise ValueError("fixed-distance control is disabled")

    def _validate_target_locked(self, target: float) -> None:
        if not self._min_target_m <= target <= self._max_target_m:
            raise ValueError(
                "target distance must be in "
                f"[{self._min_target_m:.2f}, {self._max_target_m:.2f}] m"
            )

    def _snapshot_locked(self) -> FollowDistanceSnapshot:
        measurement = self._measurement
        measured: float | None = None
        confidence: float | None = None
        if measurement is not None:
            try:
                candidate = float(measurement.distance_m)
                measured = candidate if math.isfinite(candidate) and candidate > 0.0 else None
            except (TypeError, ValueError):
                measured = None
            try:
                candidate_confidence = float(measurement.confidence)
                confidence = (
                    candidate_confidence
                    if math.isfinite(candidate_confidence)
                    else None
                )
            except (TypeError, ValueError):
                confidence = None
        return FollowDistanceSnapshot(
            enabled=self._controller.config.enabled,
            active=self._active,
            target_distance_m=self._controller.target_distance_m,
            measured_distance_m=measured,
            status=self._status,
            detail=self._detail,
            measurement_confidence=confidence,
            measurement_source=self._measurement_source,
            revision=self._revision,
        )

    @staticmethod
    def _detail_for_result(result: DistanceControlResult) -> str:
        if result.measured_distance_m is None:
            return result.reason.replace("_", " ")
        return (
            f"{result.reason.replace('_', ' ')}: measured "
            f"{result.measured_distance_m:.2f} m, target "
            f"{result.target_distance_m:.2f} m"
        )


distance_runtime = FollowDistanceRuntime()


def configure_distance_runtime(
    config: RuntimeConfig,
    *,
    runtime: FollowDistanceRuntime = distance_runtime,
) -> FollowDistanceSnapshot:
    """Configure one shared runtime from the normalized ROS/provider settings."""

    if not isinstance(config, RuntimeConfig):
        raise TypeError("config must be a RuntimeConfig")
    return runtime.configure(
        DistanceControlConfig(
            enabled=config.distance_enabled,
            target_distance_m=config.target_distance_m,
            deadband_m=config.distance_deadband_m,
            kp=config.distance_kp,
            max_forward_mps=config.distance_max_forward_mps,
            max_reverse_mps=config.distance_max_reverse_mps,
            max_measurement_age_s=config.distance_measurement_max_age_s,
            min_confidence=config.distance_min_confidence,
        ),
        min_target_m=config.min_target_distance_m,
        max_target_m=config.max_target_distance_m,
    )


__all__ = [
    "FollowDistanceCommandResult",
    "FollowDistanceRuntime",
    "FollowDistanceSnapshot",
    "MAX_TARGET_DISTANCE_M",
    "MIN_TARGET_DISTANCE_M",
    "configure_distance_runtime",
    "distance_runtime",
]
