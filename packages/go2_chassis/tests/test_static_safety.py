from pathlib import Path
import re
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]


class StaticSafetyTest(unittest.TestCase):
    def test_public_configuration_is_motion_disabled(self) -> None:
        config = yaml.safe_load((ROOT / "config" / "adapter.yaml").read_text())
        parameters = config["go2_chassis_adapter"]["ros__parameters"]
        self.assertIs(parameters["allow_motion"], False)
        self.assertEqual(parameters["allowed_modes"], [255])
        self.assertEqual(parameters["max_source_stamp_age_sec"], 0.20)
        self.assertEqual(parameters["max_source_stamp_future_skew_sec"], 0.05)

        deployment = yaml.safe_load(
            (ROOT.parents[1] / "robonix_manifest.yaml").read_text()
        )
        chassis = next(
            entry["config"]
            for entry in deployment["primitive"]
            if entry["name"] == "go2_chassis"
        )
        self.assertEqual(chassis["max_source_stamp_age_s"], 0.20)
        self.assertEqual(chassis["max_source_stamp_future_skew_s"], 0.05)

    def test_cpp_defaults_are_motion_disabled_with_unknown_mode(self) -> None:
        node = (
            ROOT
            / "ros2_ws"
            / "src"
            / "go2_chassis_adapter"
            / "src"
            / "go2_chassis_adapter_node.cpp"
        ).read_text()
        self.assertIn('"allowed_modes", {255}', node)
        self.assertIn('"state_fallback_topic", "/lf/sportmodestate"', node)
        self.assertIn("ValidateSourceStamp(", node)
        self.assertIn("stamp_tracker.Accept(source_stamp, allow_motion_)", node)
        self.assertLess(
            node.index("ValidateSourceStamp("),
            node.index("stamp_tracker.Accept(source_stamp, allow_motion_)"),
        )
        self.assertLess(
            node.index("stamp_tracker.Accept(source_stamp, allow_motion_)"),
            node.index("last_state_receipt_sec_ = receipt_sec"),
        )
        self.assertLess(
            node.index("ValidateSourceStamp("),
            node.index("guard_->UpdateRobotState(receipt_sec"),
        )
        self.assertIn("odometry.header.stamp.sec = message.stamp.sec", node)
        self.assertIn(
            "odometry.header.stamp.nanosec = message.stamp.nanosec", node
        )
        self.assertIn("imu.header.stamp = odometry.header.stamp", node)
        self.assertNotIn("odometry.header.stamp = stamp", node)
        self.assertNotIn("imu.header.stamp = stamp", node)
        self.assertIn("state == GuardState::kFault || source_stamp_fault", node)
        self.assertIn('"source timestamp rejected: "', node)
        self.assertIn("source_stamp_diagnostics_.fault_since_last_report()", node)
        self.assertIn("source_stamp_diagnostics_.MarkReported()", node)

    def test_motion_state_requires_progress_and_reverse_is_not_forwarded(self) -> None:
        guard = (ROOT / "include" / "go2_chassis" / "safety_guard.hpp").read_text()
        daemon = (ROOT / "include" / "go2_chassis" / "daemon_core.hpp").read_text()
        self.assertIn("source_stamp_ns <= last_source_stamp_ns_", guard)
        self.assertIn("source_stamp_ns == 0U", guard)
        self.assertIn(
            "message.error_code == 0U",
            (
                ROOT
                / "ros2_ws"
                / "src"
                / "go2_chassis_adapter"
                / "src"
                / "go2_chassis_adapter_node.cpp"
            ).read_text(),
        )
        self.assertIn("config_.allowed_modes.count(state_mode_)", guard)
        self.assertIn("std::clamp(velocity.vx, 0.0, config_.max_vx)", guard)
        self.assertIn("packet.vx < 0.0F || packet.vx > config_.max_vx", daemon)

    def test_ros_process_does_not_link_sdk2(self) -> None:
        cmake = (
            ROOT / "ros2_ws" / "src" / "go2_chassis_adapter" / "CMakeLists.txt"
        ).read_text()
        self.assertNotIn("unitree_sdk2", cmake)
        self.assertIn("unitree_go", cmake)

    def test_sdk_process_does_not_link_ros(self) -> None:
        cmake = (ROOT / "sdk_daemon" / "CMakeLists.txt").read_text()
        self.assertIn("unitree_sdk2", cmake)
        self.assertIn('INSTALL_RPATH "\\$ORIGIN/../lib"', cmake)
        self.assertIn("libddsc.so libddsc.so.0 libddscxx.so libddscxx.so.0", cmake)
        for forbidden in ("rclcpp", "ament", "geometry_msgs", "nav_msgs"):
            self.assertNotIn(forbidden, cmake)

    def test_runtime_has_no_posture_or_low_level_api(self) -> None:
        runtime_files = list((ROOT / "include").rglob("*.hpp"))
        runtime_files += list((ROOT / "sdk_daemon").rglob("*.cpp"))
        runtime_files += list((ROOT / "ros2_ws").rglob("*.cpp"))
        source = "\n".join(path.read_text() for path in runtime_files)
        for forbidden in (
            "StandUp",
            "StandDown",
            "RecoveryStand",
            "BalanceStand",
            "lowcmd",
            "/lowcmd",
            "/api/sport/request",
        ):
            self.assertNotIn(forbidden, source)

    def test_wrappers_never_publish_with_ros2_cli(self) -> None:
        for path in (ROOT / "scripts").glob("*.sh"):
            self.assertIsNone(
                re.search(r"ros2\s+topic\s+pub", path.read_text()), path
            )

    def test_robonix_registration_does_not_create_cmd_vel_publisher(self) -> None:
        provider = (ROOT / "go2_chassis" / "main.py").read_text()
        self.assertNotIn("create_publisher", provider)
        self.assertIn("declare_ros2_topic", provider)
        self.assertIn("normalize_config", provider)
        self.assertIn("prepare_private_directory(runtime.ipc_socket.parent)", provider)
        self.assertNotIn("ipc_socket.parent.chmod", provider)
        self.assertIn(".spawn(", provider)

    def test_package_manifest_uses_canonical_contract_names(self) -> None:
        manifest = yaml.safe_load((ROOT / "package_manifest.yaml").read_text())
        capabilities = manifest["capabilities"]
        self.assertEqual(
            {entry["name"] for entry in capabilities},
            {
                "robonix/primitive/chassis/driver",
                "robonix/primitive/chassis/twist_in",
                "robonix/primitive/chassis/odom",
            },
        )
        self.assertTrue(all(set(entry) == {"name"} for entry in capabilities))

    def test_package_entrypoint_only_starts_provider(self) -> None:
        start = (ROOT / "scripts" / "start.sh").read_text()
        for forbidden in ("start_adapter.sh", "start_daemon.sh", "ros2 run"):
            self.assertNotIn(forbidden, start)
        self.assertIn("RBNX_PACKAGE_ROOT", start)
        self.assertIn("export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp", start)
        self.assertIn("CYCLONEDDS_URI must be bound", start)
        self.assertIn("rbnx-build/codegen/ros2_idl/install/setup.bash", start)

    def test_build_checks_all_runtime_outputs(self) -> None:
        build = (ROOT / "scripts" / "build.sh").read_text()
        self.assertIn('rbnx codegen -p "$PKG" --ros2', build)
        for output in (
            "go2_chassis_adapter_node",
            "go2_sport_daemon",
            "libddsc.so.0",
            "libddscxx.so.0",
        ):
            self.assertIn(output, build)

    def test_manual_wrappers_cannot_enable_motion(self) -> None:
        for name in ("start_adapter.sh", "start_daemon.sh"):
            wrapper = (ROOT / "scripts" / name).read_text()
            self.assertIn("Refusing direct motion-enabled", wrapper)
        daemon_wrapper = (ROOT / "scripts" / "start_daemon.sh").read_text()
        self.assertNotIn("--allow-motion", daemon_wrapper)

    def test_capability_frontmatter_has_only_description(self) -> None:
        capability = (ROOT / "CAPABILITY.md").read_text()
        match = re.match(r"\A---\n(.*?)\n---\n", capability, re.DOTALL)
        self.assertIsNotNone(match)
        self.assertEqual(set(yaml.safe_load(match.group(1))), {"description"})


if __name__ == "__main__":
    unittest.main()
