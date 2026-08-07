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
        self.assertIn('Primitive(id="go2_robottrack", namespace="robonix/primitive/follow")', provider)
        for callback in ("on_init", "on_activate", "on_deactivate", "on_shutdown"):
            self.assertIn(f"@provider.{callback}", provider)


if __name__ == "__main__":
    unittest.main()
