from __future__ import annotations

import unittest

from go2_robottrack.source_mux import CommandSourceMux, TwistCommand, ZERO_TWIST


class SourceMuxTests(unittest.TestCase):
    def test_only_selected_source_can_reach_output(self) -> None:
        mux = CommandSourceMux("robottrack", max_age_s=0.25)
        nav = TwistCommand(linear_x=0.8, angular_z=0.9)
        follow = TwistCommand(linear_x=0.1, angular_z=-0.2)
        mux.update("navigation", nav, received_monotonic=10.0)
        mux.update("robottrack", follow, received_monotonic=10.0)

        selected = mux.output(now=10.1)
        self.assertEqual(selected.command, follow)
        self.assertEqual(selected.selected_source, "robottrack")
        self.assertEqual(selected.reason, "selected_source_fresh")
        self.assertNotEqual(selected.command, nav)

    def test_old_selected_command_becomes_zero(self) -> None:
        mux = CommandSourceMux("navigation", max_age_s=0.25)
        mux.update(
            "navigation",
            TwistCommand(linear_x=0.4),
            received_monotonic=1.0,
        )
        selected = mux.output(now=1.250001)
        self.assertEqual(selected.command, ZERO_TWIST)
        self.assertEqual(selected.reason, "selected_source_stale")

    def test_out_of_order_command_is_discarded(self) -> None:
        mux = CommandSourceMux("robottrack", max_age_s=1.0)
        newest = TwistCommand(linear_x=0.1)
        old = TwistCommand(linear_x=-0.1)
        self.assertTrue(
            mux.update("robottrack", newest, received_monotonic=5.0)
        )
        self.assertFalse(
            mux.update("robottrack", old, received_monotonic=4.0)
        )
        self.assertEqual(mux.output(now=5.1).command, newest)

    def test_source_switch_cannot_replay_pre_switch_command(self) -> None:
        mux = CommandSourceMux("navigation", max_age_s=10.0)
        mux.update("robottrack", TwistCommand(linear_x=0.1), received_monotonic=2.0)
        mux.select_source("robottrack")
        selected = mux.output(now=2.1)
        self.assertEqual(selected.command, ZERO_TWIST)
        self.assertEqual(selected.reason, "selected_source_missing")

    def test_nonfinite_twist_is_rejected(self) -> None:
        mux = CommandSourceMux("robottrack", max_age_s=1.0)
        with self.assertRaisesRegex(ValueError, "finite"):
            mux.update(
                "robottrack",
                TwistCommand(linear_x=float("nan")),
                received_monotonic=1.0,
            )


if __name__ == "__main__":
    unittest.main()
