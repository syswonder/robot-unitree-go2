from __future__ import annotations

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
CHECKER = ROOT / "scripts" / "check_runtime_ownership.sh"


class RuntimeOwnershipTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.fake_bin = Path(self.temporary.name) / "bin"
        self.fake_bin.mkdir()
        fake_ros2 = self.fake_bin / "ros2"
        fake_ros2.write_text(
            "#!/usr/bin/env bash\n"
            "topic=\"${@: -1}\"\n"
            "case \"$topic\" in\n"
            "  /camera/color/image_raw|/camera/color/camera_info) count=${CAMERA_COUNT:-0} ;;\n"
            "  /scanner/cloud|/scanner/imu) count=${SENSOR_COUNT:-0} ;;\n"
            "  /odom) count=${ODOM_COUNT:-0} ;;\n"
            "  /tf_static) count=${TF_STATIC_COUNT:-0} ;;\n"
            "  *) printf \"Unknown topic '%s'\\n\" \"$topic\" >&2; exit 1 ;;\n"
            "esac\n"
            "printf 'Type: offline/Fake\\nPublisher count: %s\\nSubscription count: 0\\n' \"$count\"\n",
            encoding="utf-8",
        )
        fake_ros2.chmod(0o755)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def run_checker(
        self, profile: str, phase: str = "pre", **counts: str
    ) -> subprocess.CompletedProcess[str]:
        env = os.environ.copy()
        env.update(
            {
                "PATH": f"{self.fake_bin}:{env['PATH']}",
                "GO2_OWNERSHIP_DISCOVERY_TIMEOUT_S": "3",
                "GO2_OWNERSHIP_STABILITY_SAMPLES": "1",
                "GO2_OWNERSHIP_QUERY_TIMEOUT_S": "1",
                **counts,
            }
        )
        return subprocess.run(
            [str(CHECKER), profile, phase],
            check=False,
            capture_output=True,
            text=True,
            env=env,
            timeout=10,
        )

    def test_local_preflight_requires_an_empty_owned_graph(self) -> None:
        result = self.run_checker("workstation-local")
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_nx_sensor_owner_has_exactly_one_sensor_publisher(self) -> None:
        result = self.run_checker(
            "workstation-full-nx-sensors",
            CAMERA_COUNT="1",
            SENSOR_COUNT="1",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("/odom publishers=0", result.stdout)
        self.assertIn("/tf_static publishers=0", result.stdout)

    def test_nx_full_owner_has_exactly_one_publisher_for_every_stream(self) -> None:
        result = self.run_checker(
            "workstation-ui-nx-full",
            CAMERA_COUNT="1",
            SENSOR_COUNT="1",
            ODOM_COUNT="1",
            TF_STATIC_COUNT="1",
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_duplicate_camera_publisher_fails_immediately(self) -> None:
        result = self.run_checker(
            "workstation-full-nx-sensors",
            CAMERA_COUNT="2",
            SENSOR_COUNT="1",
        )
        self.assertEqual(result.returncode, 5)
        self.assertIn("publisher ownership violation", result.stderr)

    def test_unknown_profile_is_rejected(self) -> None:
        result = self.run_checker("automatic")
        self.assertEqual(result.returncode, 2)

    def test_post_start_requires_exactly_one_final_owner(self) -> None:
        passed = self.run_checker(
            "workstation-local",
            "post",
            CAMERA_COUNT="1",
            SENSOR_COUNT="1",
            ODOM_COUNT="1",
            TF_STATIC_COUNT="1",
        )
        self.assertEqual(passed.returncode, 0, passed.stderr)
        missing = self.run_checker(
            "workstation-local",
            "post",
            CAMERA_COUNT="1",
            SENSOR_COUNT="1",
            ODOM_COUNT="0",
            TF_STATIC_COUNT="1",
        )
        self.assertEqual(missing.returncode, 6)

    def test_nx_sensors_only_post_start_omits_odom_and_tf(self) -> None:
        result = self.run_checker(
            "nx-sensors-only",
            "post",
            CAMERA_COUNT="1",
            SENSOR_COUNT="1",
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_unknown_phase_is_rejected(self) -> None:
        result = self.run_checker("workstation-local", "automatic")
        self.assertEqual(result.returncode, 2)


if __name__ == "__main__":
    unittest.main()
