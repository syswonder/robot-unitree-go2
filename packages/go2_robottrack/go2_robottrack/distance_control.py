"""ROS-independent fixed-distance control for RobotTrack commands.

The controller is deliberately separate from the inference protocol core.  A
caller may leave it disabled and retain the exact model command, or enable it
and replace only longitudinal velocity while keeping RobotTrack's yaw command.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any

from .core import VelocityCommand


def _finite_float(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


@dataclass(frozen=True)
class DistanceControlConfig:
    """Tuning for longitudinal fixed-distance control.

    ``enabled`` defaults to false so adding the controller to an existing
    RobotTrack command path does not alter its behaviour.  Reverse velocity is
    represented as a positive magnitude and applied with a negative sign when
    the measured target is closer than the requested distance.
    """

    enabled: bool = False
    target_distance_m: float = 5.0
    deadband_m: float = 0.25
    kp: float = 0.25
    max_forward_mps: float = 0.50
    max_reverse_mps: float = 0.0
    max_measurement_age_s: float = 0.50
    min_confidence: float = 0.50

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise ValueError("enabled must be a bool")

        target = _finite_float(self.target_distance_m, "target_distance_m")
        deadband = _finite_float(self.deadband_m, "deadband_m")
        kp = _finite_float(self.kp, "kp")
        max_forward = _finite_float(self.max_forward_mps, "max_forward_mps")
        max_reverse = _finite_float(self.max_reverse_mps, "max_reverse_mps")
        max_age = _finite_float(
            self.max_measurement_age_s, "max_measurement_age_s"
        )
        min_confidence = _finite_float(self.min_confidence, "min_confidence")

        if target <= 0.0:
            raise ValueError("target_distance_m must be greater than zero")
        if deadband < 0.0:
            raise ValueError("deadband_m must be non-negative")
        if kp <= 0.0:
            raise ValueError("kp must be greater than zero")
        if max_forward < 0.0:
            raise ValueError("max_forward_mps must be non-negative")
        if max_reverse < 0.0:
            raise ValueError("max_reverse_mps must be non-negative")
        if max_age <= 0.0:
            raise ValueError("max_measurement_age_s must be greater than zero")
        if not 0.0 <= min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")


@dataclass(frozen=True)
class DistanceMeasurement:
    """One target-distance observation using the process monotonic clock."""

    distance_m: float
    confidence: float
    received_monotonic: float


@dataclass(frozen=True)
class DistanceControlResult:
    """Command and diagnostics suitable for a ROS node or dashboard."""

    command: VelocityCommand
    reason: str
    target_distance_m: float
    measured_distance_m: float | None = None
    distance_error_m: float | None = None
    measurement_age_s: float | None = None
    requested_vx: float | None = None


class FixedDistanceController:
    """Apply a proportional distance loop to RobotTrack longitudinal speed.

    Target updates are thread-safe so a speech callback may call
    :meth:`set_target_distance` or :meth:`adjust_target_distance` while a
    camera/control callback calls :meth:`apply`.
    """

    def __init__(self, config: DistanceControlConfig | None = None) -> None:
        self._config = config or DistanceControlConfig()
        self._target_lock = threading.Lock()
        self._target_distance_m = float(self._config.target_distance_m)

    @property
    def config(self) -> DistanceControlConfig:
        return self._config

    @property
    def target_distance_m(self) -> float:
        with self._target_lock:
            return self._target_distance_m

    def set_target_distance(self, distance_m: float) -> float:
        """Set and return a new positive target distance."""

        target = _finite_float(distance_m, "distance_m")
        if target <= 0.0:
            raise ValueError("distance_m must be greater than zero")
        with self._target_lock:
            self._target_distance_m = target
            return target

    def adjust_target_distance(self, delta_m: float) -> float:
        """Atomically add ``delta_m`` and return the updated target distance."""

        delta = _finite_float(delta_m, "delta_m")
        with self._target_lock:
            target = self._target_distance_m + delta
            if target <= 0.0:
                raise ValueError("adjusted target distance must be greater than zero")
            self._target_distance_m = target
            return target

    def apply(
        self,
        model_command: VelocityCommand,
        measurement: DistanceMeasurement | None,
        *,
        now_monotonic: float | None = None,
    ) -> DistanceControlResult:
        """Return the command selected for the current model/measurement pair.

        Disabled control is an exact passthrough.  When enabled, a missing,
        malformed, stale, future-dated, or low-confidence measurement sets
        longitudinal velocity to zero while preserving the model yaw command.
        A valid measurement drives proportional speed outside the configured
        deadband and is clipped independently in the forward and reverse
        directions.
        """

        target = self.target_distance_m
        if not self._config.enabled:
            return DistanceControlResult(
                command=model_command,
                reason="disabled_passthrough",
                target_distance_m=target,
            )

        stopped = VelocityCommand(vx=0.0, wz=model_command.wz)
        if measurement is None:
            return DistanceControlResult(
                command=stopped,
                reason="no_measurement",
                target_distance_m=target,
            )

        try:
            distance = _finite_float(measurement.distance_m, "distance_m")
        except (TypeError, ValueError):
            return DistanceControlResult(
                command=stopped,
                reason="invalid_distance",
                target_distance_m=target,
            )
        if distance <= 0.0:
            return DistanceControlResult(
                command=stopped,
                reason="invalid_distance",
                target_distance_m=target,
                measured_distance_m=distance,
            )

        try:
            confidence = _finite_float(measurement.confidence, "confidence")
        except (TypeError, ValueError):
            return DistanceControlResult(
                command=stopped,
                reason="invalid_confidence",
                target_distance_m=target,
                measured_distance_m=distance,
            )
        if not 0.0 <= confidence <= 1.0:
            return DistanceControlResult(
                command=stopped,
                reason="invalid_confidence",
                target_distance_m=target,
                measured_distance_m=distance,
            )
        if confidence < self._config.min_confidence:
            return DistanceControlResult(
                command=stopped,
                reason="low_confidence",
                target_distance_m=target,
                measured_distance_m=distance,
            )

        try:
            received = _finite_float(
                measurement.received_monotonic, "received_monotonic"
            )
            current = (
                time.monotonic()
                if now_monotonic is None
                else _finite_float(now_monotonic, "now_monotonic")
            )
        except (TypeError, ValueError):
            return DistanceControlResult(
                command=stopped,
                reason="invalid_timestamp",
                target_distance_m=target,
                measured_distance_m=distance,
            )

        age = current - received
        if age < 0.0:
            return DistanceControlResult(
                command=stopped,
                reason="future_measurement",
                target_distance_m=target,
                measured_distance_m=distance,
                measurement_age_s=age,
            )
        if age > self._config.max_measurement_age_s:
            return DistanceControlResult(
                command=stopped,
                reason="stale_measurement",
                target_distance_m=target,
                measured_distance_m=distance,
                measurement_age_s=age,
            )

        error = distance - target
        if abs(error) <= self._config.deadband_m or math.isclose(
            abs(error),
            self._config.deadband_m,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            return DistanceControlResult(
                command=stopped,
                reason="within_deadband",
                target_distance_m=target,
                measured_distance_m=distance,
                distance_error_m=error,
                measurement_age_s=age,
                requested_vx=0.0,
            )

        # Remove the deadband width from the controlled error so velocity does
        # not jump discontinuously when a measurement crosses its edge.
        controlled_error = math.copysign(
            abs(error) - self._config.deadband_m,
            error,
        )
        requested_vx = self._config.kp * controlled_error
        if requested_vx < 0.0 and self._config.max_reverse_mps == 0.0:
            return DistanceControlResult(
                command=stopped,
                reason="too_close_forward_hold",
                target_distance_m=target,
                measured_distance_m=distance,
                distance_error_m=error,
                measurement_age_s=age,
                requested_vx=requested_vx,
            )
        vx = max(
            -self._config.max_reverse_mps,
            min(self._config.max_forward_mps, requested_vx),
        )
        return DistanceControlResult(
            command=VelocityCommand(vx=vx, wz=model_command.wz),
            reason="tracking_distance",
            target_distance_m=target,
            measured_distance_m=distance,
            distance_error_m=error,
            measurement_age_s=age,
            requested_vx=requested_vx,
        )


__all__ = [
    "DistanceControlConfig",
    "DistanceControlResult",
    "DistanceMeasurement",
    "FixedDistanceController",
]
