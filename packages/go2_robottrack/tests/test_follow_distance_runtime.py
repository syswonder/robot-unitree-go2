from __future__ import annotations

import math
import threading
import unittest

from go2_robottrack.core import RuntimeConfig, VelocityCommand
from go2_robottrack.distance_control import (
    DistanceControlConfig,
    DistanceMeasurement,
)
from go2_robottrack.follow_distance_runtime import (
    FollowDistanceRuntime,
    configure_distance_runtime,
)


class FollowDistanceRuntimeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.runtime = FollowDistanceRuntime()

    def configure_enabled(self, target: float = 5.0) -> None:
        self.runtime.configure(
            DistanceControlConfig(
                enabled=True,
                target_distance_m=target,
                deadband_m=0.2,
                kp=0.5,
                max_forward_mps=0.5,
                max_reverse_mps=0.2,
                max_measurement_age_s=0.5,
                min_confidence=0.5,
            ),
            min_target_m=0.5,
            max_target_m=7.0,
        )

    def test_default_is_disabled_inactive_and_passthrough(self) -> None:
        snapshot = self.runtime.snapshot()
        self.assertFalse(snapshot.enabled)
        self.assertFalse(snapshot.active)
        command = VelocityCommand(0.31, -0.2)
        result = self.runtime.apply(command, now_monotonic=10.0)
        self.assertIs(result.command, command)
        self.assertEqual(result.reason, "disabled_passthrough")

    def test_enabled_runtime_controls_and_reports_latest_measurement(self) -> None:
        self.configure_enabled()
        self.runtime.set_active(True)
        sample = DistanceMeasurement(6.0, 0.9, 10.0)
        self.runtime.update_measurement(sample, source="person_bbox")
        result = self.runtime.apply(
            VelocityCommand(0.1, -0.3),
            now_monotonic=10.1,
        )
        self.assertEqual(result.reason, "tracking_distance")
        self.assertAlmostEqual(result.command.vx, 0.4)
        self.assertEqual(result.command.wz, -0.3)
        snapshot = self.runtime.snapshot()
        self.assertTrue(snapshot.active)
        self.assertEqual(snapshot.measured_distance_m, 6.0)
        self.assertEqual(snapshot.measurement_confidence, 0.9)
        self.assertEqual(snapshot.measurement_source, "person_bbox")
        self.assertEqual(snapshot.status, "tracking_distance")

    def test_inactive_enabled_runtime_zeros_longitudinal_command(self) -> None:
        self.configure_enabled()
        result = self.runtime.apply(VelocityCommand(0.4, 0.2))
        self.assertEqual(result.reason, "inactive")
        self.assertEqual(result.command, VelocityCommand(0.0, 0.2))

    def test_set_adjust_get_and_bounds(self) -> None:
        self.configure_enabled()
        set_result = self.runtime.handle("set", 6.0)
        self.assertTrue(set_result.accepted)
        self.assertEqual(set_result.snapshot.target_distance_m, 6.0)
        adjust_result = self.runtime.handle("adjust", -1.0)
        self.assertTrue(adjust_result.accepted)
        self.assertEqual(adjust_result.snapshot.target_distance_m, 5.0)
        get_result = self.runtime.handle("get", math.nan)
        self.assertTrue(get_result.accepted)
        self.assertEqual(get_result.snapshot.target_distance_m, 5.0)

        rejected = self.runtime.handle("adjust", 3.0)
        self.assertFalse(rejected.accepted)
        self.assertIn("[0.50, 7.00]", rejected.detail)
        self.assertEqual(rejected.snapshot.target_distance_m, 5.0)
        unknown = self.runtime.handle("replace", 3.0)
        self.assertFalse(unknown.accepted)
        self.assertIn("set, adjust, get", unknown.detail)

    def test_disabled_runtime_rejects_mutation_but_allows_get(self) -> None:
        rejected = self.runtime.handle("set", 4.0)
        self.assertFalse(rejected.accepted)
        self.assertIn("disabled", rejected.detail)
        self.assertTrue(self.runtime.handle("get").accepted)

    def test_adjust_is_atomic_across_threads(self) -> None:
        self.configure_enabled(target=3.5)

        def adjust(delta: float) -> None:
            for _ in range(100):
                result = self.runtime.handle("adjust", delta)
                self.assertTrue(result.accepted)

        threads = [
            threading.Thread(target=adjust, args=(0.001,)) for _ in range(4)
        ] + [
            threading.Thread(target=adjust, args=(-0.001,)) for _ in range(4)
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertAlmostEqual(
            self.runtime.snapshot().target_distance_m,
            3.5,
            places=9,
        )

    def test_reconfigure_clears_transient_state(self) -> None:
        self.configure_enabled()
        self.runtime.set_active(True)
        self.runtime.update_measurement(DistanceMeasurement(5.5, 0.8, 1.0))
        self.runtime.configure(DistanceControlConfig())
        snapshot = self.runtime.snapshot()
        self.assertFalse(snapshot.enabled)
        self.assertFalse(snapshot.active)
        self.assertIsNone(snapshot.measured_distance_m)
        self.assertEqual(snapshot.status, "disabled")

    def test_normalized_runtime_config_has_one_shared_configuration_path(self) -> None:
        config = RuntimeConfig.from_mapping(
            {
                "mode": "live",
                "max_vx": 0.5,
                "distance_enabled": True,
                "target_distance_m": 6.0,
                "distance_max_forward_mps": 0.5,
            }
        )

        snapshot = configure_distance_runtime(config, runtime=self.runtime)

        self.assertTrue(snapshot.enabled)
        self.assertFalse(snapshot.active)
        self.assertEqual(snapshot.target_distance_m, 6.0)
        self.runtime.set_active(True)
        result = self.runtime.apply(
            VelocityCommand(0.1, -0.2),
            DistanceMeasurement(10.0, 0.9, 1.0),
            now_monotonic=1.0,
        )
        self.assertEqual(result.command, VelocityCommand(0.5, -0.2))


if __name__ == "__main__":
    unittest.main()
