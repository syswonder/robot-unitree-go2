from __future__ import annotations

import unittest

from go2_robottrack.core import DispatchState, VelocityCommand, ZERO_COMMAND
from go2_robottrack.distance_control import DistanceControlResult
from go2_robottrack.distance_dispatch import select_distance_dispatch


class DistanceDispatchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.calls: list[tuple[VelocityCommand, float]] = []

    def _apply(
        self,
        command: VelocityCommand,
        now_monotonic: float,
    ) -> DistanceControlResult:
        self.calls.append((command, now_monotonic))
        return DistanceControlResult(
            command=VelocityCommand(vx=0.42, wz=command.wz),
            reason="tracking_distance",
            target_distance_m=5.0,
        )

    def test_disabled_active_plan_is_exact_passthrough(self) -> None:
        model = VelocityCommand(vx=0.2, wz=-0.3)
        selected = select_distance_dispatch(
            DispatchState(model, "active_plan", 0.1, 3),
            distance_enabled=False,
            apply_distance=self._apply,
            now_monotonic=10.0,
        )
        self.assertIs(selected.command, model)
        self.assertEqual(selected.reason, "active_plan")
        self.assertEqual(self.calls, [])

    def test_enabled_active_plan_applies_longitudinal_control_and_keeps_yaw(self) -> None:
        model = VelocityCommand(vx=0.2, wz=-0.3)
        selected = select_distance_dispatch(
            DispatchState(model, "active_plan", 0.1, 3),
            distance_enabled=True,
            apply_distance=self._apply,
            now_monotonic=10.0,
        )
        self.assertEqual(selected.command, VelocityCommand(vx=0.42, wz=-0.3))
        self.assertEqual(selected.reason, "tracking_distance")
        self.assertEqual(self.calls, [(model, 10.0)])

    def test_no_plan_and_stale_plan_are_strict_zero_without_controller_call(self) -> None:
        for reason in ("no_plan", "stale_plan"):
            with self.subTest(reason=reason):
                selected = select_distance_dispatch(
                    DispatchState(
                        VelocityCommand(vx=0.5, wz=0.3),
                        reason,
                        None,
                        1,
                    ),
                    distance_enabled=True,
                    apply_distance=self._apply,
                    now_monotonic=10.0,
                )
                self.assertEqual(selected.command, ZERO_COMMAND)
                self.assertEqual(selected.reason, reason)
        self.assertEqual(self.calls, [])


if __name__ == "__main__":
    unittest.main()
