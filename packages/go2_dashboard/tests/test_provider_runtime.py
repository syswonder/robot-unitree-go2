from __future__ import annotations

import json
import unittest

from go2_dashboard.provider_runtime import (
    DashboardConfig,
    DashboardProcess,
    status_json,
)


class _Clock:
    def __init__(self) -> None:
        self.value = 10.0

    def monotonic(self) -> float:
        return self.value

    def sleep(self, seconds: float) -> None:
        self.value += seconds


class _FakeProcess:
    def __init__(self, pid: int = 4242) -> None:
        self.pid = pid
        self.return_code = None
        self.terminate_calls = 0
        self.kill_calls = 0
        self.wait_calls = []

    def poll(self):
        return self.return_code

    def terminate(self) -> None:
        self.terminate_calls += 1
        self.return_code = 0

    def kill(self) -> None:
        self.kill_calls += 1
        self.return_code = -9

    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        return self.return_code


class ProviderRuntimeTests(unittest.TestCase):
    def test_config_maps_every_topic_host_and_port_to_child_environment(self) -> None:
        config = DashboardConfig.from_mapping(
            {
                "host": "127.0.0.1",
                "port": "9102",
                "public_url": "https://go2.example.test/telemetry",
                "image_topic": "/front/image",
                "scan_topic": "/lidar/scan",
                "cloud_topic": "/lidar/cloud",
                "map_topic": "/lab/map",
                "odom_topic": "/go2/odom",
                "nav_status_topic": "/nav/status",
                "map_frame": "lab_map",
                "base_frame": "go2_base",
                "log_level": "warning",
            }
        )
        environment = config.child_environment(
            {
                "SAFE_PARENT": "kept",
                "VLM_API_KEY": "must-not-reach-child",
                "HTTPS_PROXY": "http://user:password@example.test",
            }
        )
        self.assertEqual(environment["SAFE_PARENT"], "kept")
        self.assertNotIn("VLM_API_KEY", environment)
        self.assertNotIn("HTTPS_PROXY", environment)
        self.assertEqual(environment["GO2_DASHBOARD_HOST"], "127.0.0.1")
        self.assertEqual(environment["GO2_DASHBOARD_PORT"], "9102")
        self.assertEqual(environment["GO2_DASHBOARD_CAMERA_TOPIC"], "/front/image")
        self.assertEqual(environment["GO2_DASHBOARD_SCAN_TOPIC"], "/lidar/scan")
        self.assertEqual(environment["GO2_DASHBOARD_CLOUD_TOPIC"], "/lidar/cloud")
        self.assertEqual(environment["GO2_DASHBOARD_MAP_TOPIC"], "/lab/map")
        self.assertEqual(environment["GO2_DASHBOARD_ODOM_TOPIC"], "/go2/odom")
        self.assertEqual(environment["GO2_DASHBOARD_NAV_STATUS_TOPIC"], "/nav/status")
        self.assertEqual(environment["GO2_DASHBOARD_MAP_FRAME"], "lab_map")
        self.assertEqual(environment["GO2_DASHBOARD_BASE_FRAME"], "go2_base")
        self.assertEqual(config.url, "https://go2.example.test/telemetry")
        self.assertEqual(config.health_url, "http://127.0.0.1:9102/healthz")

    def test_config_rejects_unknown_or_unsafe_values(self) -> None:
        with self.assertRaisesRegex(ValueError, "unknown"):
            DashboardConfig.from_mapping({"secret": "must-not-be-accepted"})
        with self.assertRaisesRegex(ValueError, "disagree"):
            DashboardConfig.from_mapping(
                {"camera_topic": "/camera/a", "image_topic": "/camera/b"}
            )
        with self.assertRaisesRegex(ValueError, "absolute"):
            DashboardConfig.from_mapping({"camera_topic": "relative/image"})
        with self.assertRaisesRegex(ValueError, "between"):
            DashboardConfig.from_mapping({"port": 70000})
        with self.assertRaisesRegex(ValueError, "without credentials"):
            DashboardConfig.from_mapping(
                {"public_url": "https://user:password@example.test"}
            )
        with self.assertRaisesRegex(ValueError, "127.0.0.1"):
            DashboardConfig.from_mapping({"host": "0.0.0.0"})

    def test_start_and_stop_signal_only_the_created_child(self) -> None:
        clock = _Clock()
        child = _FakeProcess()
        unrelated = _FakeProcess(pid=9999)
        calls = []

        def popen(command, **kwargs):
            calls.append((command, kwargs))
            return child

        process = DashboardProcess(
            popen_factory=popen,
            health_reader=lambda _: {"ok": True, "ros_connected": True},
            monotonic_fn=clock.monotonic,
            sleep_fn=clock.sleep,
        )
        process.configure({"host": "127.0.0.1", "port": 8092})
        process.start()
        self.assertEqual(calls[0][0][-2:], ["-m", "go2_dashboard.main"])
        self.assertFalse(calls[0][1]["start_new_session"])
        self.assertEqual(process.status()["status"]["pid"], 4242)
        process.stop()
        self.assertEqual(child.terminate_calls, 1)
        self.assertEqual(child.kill_calls, 0)
        self.assertEqual(unrelated.terminate_calls, 0)
        self.assertEqual(unrelated.kill_calls, 0)

    def test_start_failure_terminates_the_owned_child(self) -> None:
        clock = _Clock()
        child = _FakeProcess()
        process = DashboardProcess(
            popen_factory=lambda *args, **kwargs: child,
            health_reader=lambda _: (_ for _ in ()).throw(ConnectionError("offline")),
            monotonic_fn=clock.monotonic,
            sleep_fn=clock.sleep,
        )
        process.configure({"startup_timeout_s": 0.5})
        with self.assertRaisesRegex(RuntimeError, "offline"):
            process.start()
        self.assertEqual(child.terminate_calls, 1)

    def test_status_is_read_only_json_and_requires_ros_connection_for_ok(self) -> None:
        child = _FakeProcess()
        process = DashboardProcess(
            popen_factory=lambda *args, **kwargs: child,
            health_reader=lambda _: {"ok": True, "ros_connected": False},
        )
        process.configure({})
        process.start()
        snapshot = process.status()
        self.assertFalse(snapshot["ok"])
        self.assertIn("ROS observer is disconnected", snapshot["detail"])
        parsed = json.loads(status_json(snapshot))
        self.assertTrue(parsed["read_only"])
        self.assertTrue(parsed["process_running"])


if __name__ == "__main__":
    unittest.main()
