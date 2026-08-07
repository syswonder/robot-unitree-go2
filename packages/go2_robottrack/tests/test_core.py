from __future__ import annotations

import math
import threading
import unittest

from go2_robottrack.core import (
    CONTROL_DT,
    InferencePlan,
    LatestFrameMailbox,
    PlanStore,
    ProtocolError,
    RuntimeConfig,
    VelocityCommand,
    parse_inference_response,
)


def waypoints() -> list[list[float]]:
    return [[index * 0.01, 0.0, index * 0.005] for index in range(8)]


class RuntimeConfigTests(unittest.TestCase):
    def test_defaults_are_d435i_dry_run_and_official_tuning(self) -> None:
        config = RuntimeConfig.from_mapping({})
        self.assertEqual(config.mode, "dry-run")
        self.assertEqual(config.rgb_topic, "/go2/d435i/color/image_raw")
        self.assertEqual(config.command_topic, "/go2/robottrack/cmd_vel_raw")
        self.assertEqual(config.model_input_mode, "center_crop_height")
        self.assertEqual(config.model_crop_size, 384)
        self.assertEqual(config.waypoint_strategy, "first")
        self.assertEqual(config.control_dt, 0.1)
        self.assertEqual(config.dispatch_hz, 50.0)
        self.assertEqual(config.max_plan_age_s, 1.5)
        self.assertEqual(config.max_vx, 0.15)
        self.assertEqual(config.max_wz, 0.30)

    def test_live_and_nested_mux_config_are_accepted(self) -> None:
        config = RuntimeConfig.from_mapping(
            {
                "mode": "live",
                "source_mux": {
                    "nav_input_topic": "/custom/nav_raw",
                    "robottrack_input_topic": "/custom/follow_raw",
                    "output_topic": "/custom/selected",
                    "selected_source": "robottrack",
                },
                "command_topic": "/custom/follow_raw",
                "camera_info_topic": "/unused/info",
                "asset_manifest": "/unused/assets.json",
            }
        )
        self.assertEqual(config.mode, "live")
        self.assertEqual(config.nav_raw_topic, "/custom/nav_raw")
        self.assertEqual(config.robottrack_raw_topic, "/custom/follow_raw")
        self.assertEqual(config.selected_output_topic, "/custom/selected")

    def test_live_forward_limit_can_use_robottrack_chassis_contract(self) -> None:
        config = RuntimeConfig.from_mapping({"mode": "live", "max_vx": 0.50})
        self.assertEqual(config.max_vx, 0.50)
        with self.assertRaisesRegex(ValueError, "max_vx"):
            RuntimeConfig.from_mapping({"max_vx": 0.51})
        yaw = RuntimeConfig.from_mapping({"mode": "live", "max_wz": 0.40})
        self.assertEqual(yaw.max_wz, 0.40)
        with self.assertRaisesRegex(ValueError, "max_wz"):
            RuntimeConfig.from_mapping({"max_wz": 0.41})

    def test_official_model_input_geometry_is_fixed(self) -> None:
        with self.assertRaisesRegex(ValueError, "model_input_mode"):
            RuntimeConfig.from_mapping({"model_input_mode": "aspect_resize"})
        with self.assertRaisesRegex(ValueError, "model_crop_size"):
            RuntimeConfig.from_mapping({"model_crop_size": 512})


class ResponseParserTests(unittest.TestCase):
    def test_full_response_is_finite_and_bounded(self) -> None:
        plan = parse_inference_response(
            {
                "waypoints": waypoints(),
                "base_velocity": [0.8, 0.4, -1.2],
                "control_dt": CONTROL_DT,
            }
        )
        self.assertEqual(len(plan.waypoints or ()), 8)
        self.assertEqual(plan.command, VelocityCommand(0.15, -0.30))
        self.assertEqual(plan.velocity_source, "base_velocity")

    def test_velocity_only_trimmed_response_is_supported(self) -> None:
        plan = parse_inference_response({"velocity": [-0.08, 0.0, 0.1]})
        self.assertEqual(plan.command, VelocityCommand(-0.08, 0.1))
        self.assertIsNone(plan.waypoints)

    def test_waypoint_only_response_uses_index_one_and_point_one_seconds(self) -> None:
        points = waypoints()
        points[1] = [0.01, 99.0, -0.02]
        plan = parse_inference_response({"trajectory": points})
        self.assertAlmostEqual(plan.command.vx, 0.1)
        self.assertAlmostEqual(plan.command.wz, -0.2)
        self.assertEqual(plan.velocity_source, "waypoints[1]")

    def test_wrong_shape_and_nonfinite_values_are_rejected(self) -> None:
        with self.assertRaisesRegex(ProtocolError, "exactly 8"):
            parse_inference_response({"waypoints": waypoints()[:7]})
        broken = waypoints()
        broken[3][1] = math.nan
        with self.assertRaisesRegex(ProtocolError, "finite"):
            parse_inference_response({"waypoints": broken})
        with self.assertRaisesRegex(ProtocolError, "finite"):
            parse_inference_response({"base_velocity": [0.1, 0.0, math.inf]})
        with self.assertRaisesRegex(ProtocolError, "control_dt"):
            parse_inference_response(
                {"base_velocity": [0.1, 0.0, 0.1], "control_dt": 0.2}
            )


class MailboxAndPlanTests(unittest.TestCase):
    def test_mailbox_retains_only_latest_frame(self) -> None:
        mailbox = LatestFrameMailbox()
        first = mailbox.put(b"first", source_timestamp=1.0, received_monotonic=2.0)
        second = mailbox.put(b"second", source_timestamp=2.0, received_monotonic=3.0)
        self.assertEqual(mailbox.retained_count, 1)
        self.assertEqual(mailbox.latest(), second)
        self.assertEqual(mailbox.wait_after(first.sequence, timeout_s=0.0), second)

    def test_plan_is_zero_only_after_one_point_five_seconds(self) -> None:
        store = PlanStore()
        plan = InferencePlan(VelocityCommand(0.1, -0.2), None)
        store.update(plan, now=10.0)
        self.assertEqual(store.dispatch(now=11.5).reason, "active_plan")
        stale = store.dispatch(now=11.500001)
        self.assertEqual(stale.reason, "stale_plan")
        self.assertEqual(stale.command, VelocityCommand())


if __name__ == "__main__":
    unittest.main()
