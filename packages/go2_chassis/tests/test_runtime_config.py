import os
from pathlib import Path
import stat
import sys
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from go2_chassis.runtime_config import (  # noqa: E402
    ConfigError,
    UNKNOWN_MODE,
    normalize_config,
    prepare_private_directory,
)


class RuntimeConfigTest(unittest.TestCase):
    def test_private_directory_is_created_and_secured(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / "runtime" / "socket"
            prepare_private_directory(directory)
            info = os.lstat(directory)
            self.assertTrue(stat.S_ISDIR(info.st_mode))
            self.assertEqual(info.st_uid, os.geteuid())
            self.assertEqual(stat.S_IMODE(info.st_mode), 0o700)

    def test_private_directory_rejects_symlink_without_chmod_target(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            target = root / "target"
            target.mkdir()
            os.chmod(target, 0o755)
            link = root / "runtime"
            link.symlink_to(target, target_is_directory=True)

            with self.assertRaisesRegex(ConfigError, "symlink"):
                prepare_private_directory(link)
            self.assertEqual(stat.S_IMODE(os.lstat(target).st_mode), 0o755)

    def test_private_directory_rejects_foreign_uid_without_chmod(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / "runtime"
            directory.mkdir()
            os.chmod(directory, 0o755)
            simulated_foreign_uid = os.geteuid() + 1

            with mock.patch(
                "go2_chassis.runtime_config.os.geteuid",
                return_value=simulated_foreign_uid,
            ), self.assertRaisesRegex(ConfigError, "current UID"):
                prepare_private_directory(directory)
            self.assertEqual(stat.S_IMODE(os.lstat(directory).st_mode), 0o755)

    def test_default_is_read_only_with_impossible_mode(self) -> None:
        runtime = normalize_config({}, {}, ROOT)
        self.assertFalse(runtime.allow_motion)
        self.assertFalse(runtime.starts_sdk_daemon)
        self.assertEqual(runtime.allowed_modes, (UNKNOWN_MODE,))
        self.assertEqual(runtime.max_source_stamp_age_s, 0.20)
        self.assertEqual(runtime.max_source_stamp_future_skew_s, 0.05)
        self.assertTrue(str(runtime.ipc_socket).startswith(
            f"/tmp/robonix-go2-"
        ))
        self.assertLess(len(str(runtime.ipc_socket)), 108)
        with self.assertRaises(ConfigError):
            runtime.daemon_argv(Path("/tmp/daemon"))

    def test_driver_config_reaches_every_adapter_parameter(self) -> None:
        config = {
            "state_topic": "/go2/high_state",
            "state_fallback_topic": "/go2/low_state",
            "twist_in_topic": "/nav/cmd_vel",
            "odom_topic": "/go2/odom",
            "imu_topic": "/go2/imu",
            "odom_frame": "go2_odom",
            "base_frame": "go2_base",
            "imu_frame": "go2_imu",
            "velocity_frame": "base_link",
            "ipc_socket": "/tmp/go2-test.sock",
            "max_linear_x_mps": 0.1,
            "max_linear_y_mps": 0.0,
            "max_angular_z_rps": 0.2,
            "max_linear_accel_mps2": 0.2,
            "max_angular_accel_rps2": 0.4,
            "command_timeout_s": 0.2,
            "state_timeout_s": 0.3,
            "max_source_stamp_age_s": 0.15,
            "max_source_stamp_future_skew_s": 0.02,
            "zero_preamble_s": 1.0,
        }
        runtime = normalize_config(config, {}, ROOT)
        argv = runtime.adapter_argv(Path("/tmp/adapter"), Path("/tmp/p.yaml"))
        rendered = "\n".join(argv)
        for expected in (
            "sport_state_topic:=/go2/high_state",
            "state_fallback_topic:=/go2/low_state",
            "cmd_vel_topic:=/nav/cmd_vel",
            "odom_topic:=/go2/odom",
            "imu_topic:=/go2/imu",
            "odom_frame:=go2_odom",
            "base_frame:=go2_base",
            "imu_frame:=go2_imu",
            "state_velocity_frame:=base_link",
            "sdk_socket:=/tmp/go2-test.sock",
            "max_vx:=0.1",
            "max_vy:=0.0",
            "max_wz:=0.2",
            "max_source_stamp_age_sec:=0.15",
            "max_source_stamp_future_skew_sec:=0.02",
            "allowed_modes:=[255]",
        ):
            self.assertIn(expected, rendered)

    def test_manifest_modes_cannot_enable_motion_without_audit_env(self) -> None:
        dangerous = {
            "allow_motion": True,
            "operator_present": True,
            "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
            "network_interface": "enp1s0",
            "allowed_modes": [0, 1, 3],
            "ipc_socket": "/tmp/go2-test.sock",
        }
        with self.assertRaisesRegex(ConfigError, "allowed_modes"):
            normalize_config(dangerous, {}, ROOT)

        dangerous["allowed_modes"] = [UNKNOWN_MODE]
        with self.assertRaisesRegex(ConfigError, "GO2_ALLOWED_MODES"):
            normalize_config(dangerous, {}, ROOT)

    def test_all_motion_gates_and_audited_modes_are_required(self) -> None:
        base = {
            "allow_motion": True,
            "network_interface": "enp1s0",
            "ipc_socket": "/tmp/go2-test.sock",
        }
        with self.assertRaisesRegex(ConfigError, "operator_present"):
            normalize_config(base, {"GO2_ALLOWED_MODES": "0"}, ROOT)
        base["operator_present"] = True
        with self.assertRaisesRegex(ConfigError, "safety_ack"):
            normalize_config(base, {"GO2_ALLOWED_MODES": "0"}, ROOT)
        base["safety_ack"] = "I_UNDERSTAND_GO2_CAN_MOVE"
        runtime = normalize_config(
            base, {"GO2_ALLOWED_MODES": "3, 1,3"}, ROOT
        )
        self.assertTrue(runtime.allow_motion)
        self.assertTrue(runtime.starts_sdk_daemon)
        self.assertEqual(runtime.allowed_modes, (3, 1))
        self.assertIn("--allow-motion", runtime.daemon_argv(Path("/tmp/daemon")))
        self.assertIn("allowed_modes:=[3,1]", "\n".join(
            runtime.adapter_argv(Path("/tmp/adapter"), Path("/tmp/p.yaml"))
        ))

    def test_unknown_and_dangerous_config_fails_closed(self) -> None:
        bad_configs = (
            {"max_linear_x_mps": 0.26},
            {"max_linear_y_mps": 0.01},
            {"zero_preamble_s": 0.49},
            {"command_timeout_s": float("nan")},
            {"max_source_stamp_age_s": 0.0},
            {"max_source_stamp_age_s": 0.51},
            {"max_source_stamp_future_skew_s": -0.01},
            {"max_source_stamp_future_skew_s": 0.11},
            {"velocity_frame": "map"},
            {"twist_in_topic": "cmd_vel"},
            {"surprise_motion_flag": True},
        )
        for config in bad_configs:
            with self.subTest(config=config), self.assertRaises(ConfigError):
                normalize_config(config, {}, ROOT)

    def test_invalid_audited_modes_fail_closed(self) -> None:
        config = {
            "allow_motion": True,
            "operator_present": True,
            "safety_ack": "I_UNDERSTAND_GO2_CAN_MOVE",
            "network_interface": "enp1s0",
            "ipc_socket": "/tmp/go2-test.sock",
        }
        for value in ("", "255", "-1", "1,,3", "standing"):
            with self.subTest(value=value), self.assertRaises(ConfigError):
                normalize_config(config, {"GO2_ALLOWED_MODES": value}, ROOT)


if __name__ == "__main__":
    unittest.main()
