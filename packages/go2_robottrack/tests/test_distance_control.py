from __future__ import annotations

import math
import unittest

from go2_robottrack.core import VelocityCommand
from go2_robottrack.distance_control import (
    DistanceControlConfig,
    DistanceMeasurement,
    FixedDistanceController,
)


def measurement(
    distance_m: float,
    *,
    confidence: float = 0.9,
    received_monotonic: float = 10.0,
) -> DistanceMeasurement:
    return DistanceMeasurement(distance_m, confidence, received_monotonic)


class FixedDistanceControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.model_command = VelocityCommand(vx=0.31, wz=-0.27)
        self.config = DistanceControlConfig(
            enabled=True,
            target_distance_m=5.0,
            deadband_m=0.2,
            kp=0.5,
            max_forward_mps=0.5,
            max_reverse_mps=0.15,
            max_measurement_age_s=0.5,
            min_confidence=0.6,
        )
        self.controller = FixedDistanceController(self.config)

    def test_disabled_default_preserves_exact_model_command(self) -> None:
        controller = FixedDistanceController()
        result = controller.apply(self.model_command, None, now_monotonic=10.0)
        self.assertIs(result.command, self.model_command)
        self.assertEqual(result.reason, "disabled_passthrough")

    def test_far_target_commands_forward_and_preserves_model_yaw(self) -> None:
        result = self.controller.apply(
            self.model_command,
            measurement(6.0),
            now_monotonic=10.1,
        )
        self.assertAlmostEqual(result.command.vx, 0.4)
        self.assertEqual(result.command.wz, self.model_command.wz)
        self.assertEqual(result.reason, "tracking_distance")
        self.assertAlmostEqual(result.distance_error_m or 0.0, 1.0)

    def test_forward_and_reverse_speeds_have_independent_limits(self) -> None:
        forward = self.controller.apply(
            self.model_command,
            measurement(8.0),
            now_monotonic=10.0,
        )
        reverse = self.controller.apply(
            self.model_command,
            measurement(3.0),
            now_monotonic=10.0,
        )
        self.assertEqual(forward.command, VelocityCommand(0.5, -0.27))
        self.assertEqual(reverse.command, VelocityCommand(-0.15, -0.27))

    def test_reverse_disabled_holds_forward_position_with_explicit_reason(self) -> None:
        controller = FixedDistanceController(
            DistanceControlConfig(
                enabled=True,
                target_distance_m=5.0,
                deadband_m=0.2,
                kp=0.5,
                max_forward_mps=0.5,
                max_reverse_mps=0.0,
            )
        )
        result = controller.apply(
            VelocityCommand(0.3, 0.2),
            DistanceMeasurement(4.0, 0.9, 10.0),
            now_monotonic=10.1,
        )
        self.assertEqual(result.command, VelocityCommand(0.0, 0.2))
        self.assertEqual(result.reason, "too_close_forward_hold")
        self.assertLess(result.requested_vx, 0.0)

    def test_measurements_at_deadband_edges_stop_longitudinal_motion(self) -> None:
        for distance in (4.8, 5.0, 5.2):
            with self.subTest(distance=distance):
                result = self.controller.apply(
                    self.model_command,
                    measurement(distance),
                    now_monotonic=10.0,
                )
                self.assertEqual(result.command, VelocityCommand(0.0, -0.27))
                self.assertEqual(result.reason, "within_deadband")

    def test_missing_stale_and_low_confidence_measurements_zero_only_vx(self) -> None:
        cases = (
            (None, 10.0, "no_measurement"),
            (measurement(6.0, received_monotonic=9.49), 10.0, "stale_measurement"),
            (measurement(6.0, confidence=0.59), 10.0, "low_confidence"),
        )
        for sample, now, reason in cases:
            with self.subTest(reason=reason):
                result = self.controller.apply(
                    self.model_command,
                    sample,
                    now_monotonic=now,
                )
                self.assertEqual(result.command, VelocityCommand(0.0, -0.27))
                self.assertEqual(result.reason, reason)

    def test_invalid_measurement_values_fail_closed_longitudinally(self) -> None:
        cases = (
            (measurement(math.nan), "invalid_distance"),
            (measurement(0.0), "invalid_distance"),
            (measurement(6.0, confidence=math.nan), "invalid_confidence"),
            (measurement(6.0, confidence=1.1), "invalid_confidence"),
            (
                measurement(6.0, received_monotonic=math.nan),
                "invalid_timestamp",
            ),
            (measurement(6.0, received_monotonic=10.1), "future_measurement"),
        )
        for sample, reason in cases:
            with self.subTest(reason=reason):
                result = self.controller.apply(
                    self.model_command,
                    sample,
                    now_monotonic=10.0,
                )
                self.assertEqual(result.command, VelocityCommand(0.0, -0.27))
                self.assertEqual(result.reason, reason)

    def test_measurement_at_maximum_age_is_still_fresh(self) -> None:
        result = self.controller.apply(
            self.model_command,
            measurement(6.0, received_monotonic=9.5),
            now_monotonic=10.0,
        )
        self.assertEqual(result.reason, "tracking_distance")
        self.assertAlmostEqual(result.command.vx, 0.4)

    def test_target_distance_can_be_set_and_adjusted_atomically(self) -> None:
        self.assertEqual(self.controller.set_target_distance(6.0), 6.0)
        self.assertEqual(self.controller.adjust_target_distance(1.0), 7.0)
        self.assertEqual(self.controller.adjust_target_distance(-2.0), 5.0)
        self.assertEqual(self.controller.target_distance_m, 5.0)
        with self.assertRaisesRegex(ValueError, "greater than zero"):
            self.controller.adjust_target_distance(-5.0)
        self.assertEqual(self.controller.target_distance_m, 5.0)

    def test_invalid_configuration_is_rejected(self) -> None:
        invalid = (
            {"target_distance_m": 0.0},
            {"deadband_m": -0.1},
            {"kp": 0.0},
            {"max_forward_mps": -0.1},
            {"max_reverse_mps": -0.1},
            {"max_measurement_age_s": 0.0},
            {"min_confidence": 1.01},
        )
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                DistanceControlConfig(**values)


if __name__ == "__main__":
    unittest.main()
