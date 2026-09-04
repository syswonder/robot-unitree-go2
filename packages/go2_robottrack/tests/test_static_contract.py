from __future__ import annotations

import ast
from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class StaticContractTests(unittest.TestCase):
    def test_default_config_and_topics_are_exact(self) -> None:
        config = (ROOT / "config" / "go2_robottrack.yaml").read_text(encoding="utf-8")
        for expected in (
            "mode: dry-run",
            "rgb_topic: /go2/d435i/color/image_raw",
            "depth_topic: /go2/d435i/aligned_depth_to_color/image_raw",
            "command_topic: /go2/robottrack/cmd_vel_raw",
            "model_input_mode: center_crop_height",
            "model_crop_size: 384",
            "waypoint_strategy: first",
            "control_dt: 0.1",
            "dispatch_hz: 50.0",
            "max_plan_age_s: 1.5",
            "max_vx: 0.15",
            "max_wz: 0.30",
            "nav_raw_topic: /go2/robottrack/nav_cmd_vel_raw",
            "robottrack_raw_topic: /go2/robottrack/cmd_vel_raw",
            "selected_output_topic: /cmd_vel_nav",
            "selected_source: robottrack",
            "distance_enabled: false",
            "target_distance_m: 5.0",
        ):
            with self.subTest(expected=expected):
                self.assertIn(expected, config)

    def test_dry_run_branch_contains_all_velocity_publisher_creation(self) -> None:
        source = (ROOT / "go2_robottrack" / "ros_node.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        publisher_calls = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                if node.func.attr == "create_publisher":
                    publisher_calls.append(node)
        self.assertEqual(len(publisher_calls), 2)
        live_block = re.search(
            r'if self\._config\.mode == "live":\n(?P<body>(?: {12}.*\n|\n)+)',
            source,
        )
        self.assertIsNotNone(live_block)
        self.assertEqual(live_block.group("body").count("create_publisher"), 2)
        self.assertEqual(source.count("self._selected_publisher = self.create_publisher"), 1)

    def test_node_close_zeroes_both_live_velocity_outputs(self) -> None:
        source = (ROOT / "go2_robottrack" / "ros_node.py").read_text(encoding="utf-8")
        self.assertIn("self._raw_publisher.publish(Twist())", source)
        self.assertIn("self._selected_publisher.publish(Twist())", source)
        self.assertIn("self._http.close()", source)
        self.assertIn("prepare_center_crop_height", source)
        self.assertIn("CameraFrameUploadWorker", source)
        self.assertIn("self._camera_preview.close()", source)
        self.assertIn("self._camera_preview.submit(", source)
        self.assertIn("self._offer_camera_preview(bgr)", source)
        self.assertIn("model_bgr,", source)
        self.assertIn("full_bgr,", source)
        provider = (ROOT / "go2_robottrack" / "provider.py").read_text(encoding="utf-8")
        self.assertEqual(provider.count("ok, detail = _stop_runtime()"), 2)

    def test_distance_estimation_is_async_and_opt_in(self) -> None:
        source = (ROOT / "go2_robottrack" / "ros_node.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        distance_worker_calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "RgbdDistanceWorker"
        ]
        self.assertEqual(len(distance_worker_calls), 1)
        guarded_calls = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.If):
                continue
            if ast.unparse(node.test) != "self._config.distance_enabled":
                continue
            guarded_calls.extend(
                child
                for child in ast.walk(node)
                if isinstance(child, ast.Call)
                and isinstance(child.func, ast.Name)
                and child.func.id == "RgbdDistanceWorker"
            )
        self.assertEqual(guarded_calls, distance_worker_calls)
        self.assertNotIn("self._distance_estimator.estimate(", source)
        self.assertIn("worker.submit(decision.pair)", source)
        self.assertIn("worker.invalidate()", source)
        self.assertIn("worker.commit_if_current(epoch, commit)", source)
        self.assertIn("received_monotonic=pair.received_monotonic", source)
        self.assertIn(
            "self._distance_measurement_received_monotonic = (\n"
            "                    pair.received_monotonic\n"
            "                )",
            source,
        )
        self.assertLess(
            source.index("distance_worker.request_stop()"),
            source.index("self._raw_publisher.publish(Twist())"),
        )

    def test_no_unitree_or_posture_control_surface_exists(self) -> None:
        runtime_sources = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (ROOT / "go2_robottrack").glob("*.py")
        )
        for forbidden in (
            "SportClient",
            "StopMove",
            "/api/sport/request",
            "/lowcmd",
            "unitree_sdk",
            "StandUp",
            "StandDown",
            "RecoveryStand",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, runtime_sources)

    def test_build_start_and_tests_do_not_install_or_change_networking(self) -> None:
        scripts = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (ROOT / "build.sh", ROOT / "start.sh", ROOT / "tests" / "run_offline_tests.sh")
        )
        for forbidden in (
            r"\bsudo\b",
            r"\bapt(?:-get)?\b",
            r"\bnmcli\b",
            r"\bip\s+(?:addr|route|link)\b",
            r"\bsystemctl\b",
            r"\bdocker\b",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertIsNone(re.search(forbidden, scripts))

    def test_manifest_and_provider_identity_are_present(self) -> None:
        manifest = (ROOT / "package_manifest.yaml").read_text(encoding="utf-8")
        provider = (ROOT / "go2_robottrack" / "provider.py").read_text(encoding="utf-8")
        self.assertIn("name: robonix/primitive/follow/driver", manifest)
        self.assertIn("name: robonix/primitive/follow/distance", manifest)
        self.assertIn('Primitive(id="go2_robottrack", namespace="robonix/primitive/follow")', provider)
        self.assertIn("@provider.mcp(DISTANCE_CONTRACT)", provider)
        for callback in ("on_init", "on_activate", "on_deactivate", "on_shutdown"):
            self.assertIn(f"@provider.{callback}", provider)

    def test_build_generates_and_start_loads_mcp_bindings(self) -> None:
        build = (ROOT / "build.sh").read_text(encoding="utf-8")
        start = (ROOT / "start.sh").read_text(encoding="utf-8")
        self.assertRegex(build, r"codegen\s+-p\s+.*--mcp\s+--ros2")
        self.assertIn("robonix_mcp_types/go2_robottrack_control_mcp.py", start)
        self.assertIn("codegen/robonix_mcp_types", start)


if __name__ == "__main__":
    unittest.main()
