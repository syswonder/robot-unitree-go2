from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts" / "stack_readiness.py"
SPEC = importlib.util.spec_from_file_location("stack_readiness", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
gate = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = gate
SPEC.loader.exec_module(gate)


def provider(provider_id: str, caps: set[str], state: str = "ACTIVE") -> dict:
    return {
        "provider_id": provider_id,
        "namespace": "robonix/test",
        "state": state,
        "state_detail": "",
        "capabilities": [
            {
                "contract_id": contract_id,
                "transport": "grpc",
                "namespace_mismatch": False,
            }
            for contract_id in sorted(caps)
        ],
    }


class ReadinessUnitTests(unittest.TestCase):
    def test_all_required_active_providers_pass(self) -> None:
        payload = [
            provider(provider_id, set(capabilities))
            for provider_id, capabilities in gate.REQUIRED_CAPABILITIES.items()
        ]
        result = gate.validate_providers(payload)
        self.assertEqual(result.status, gate.PASS)

    def test_missing_capability_and_error_provider_fail(self) -> None:
        payload = [
            provider(provider_id, set(capabilities))
            for provider_id, capabilities in gate.REQUIRED_CAPABILITIES.items()
        ]
        payload[0]["state"] = "ERROR"
        payload[1]["capabilities"] = []
        result = gate.validate_providers(payload)
        self.assertEqual(result.status, gate.FAIL)
        self.assertTrue(result.evidence["problems"])

    def test_topic_header_requires_exact_fresh_timestamp_and_frame(self) -> None:
        requirement = gate.TOPICS[0]
        now = time.time()
        sec = int(now)
        header = {
            "stamp": {"sec": sec, "nanosec": int((now - sec) * 1_000_000_000)},
            "frame_id": "front_camera",
        }
        self.assertEqual(
            gate.validate_topic_header(requirement, header, now).status, gate.PASS
        )
        header["frame_id"] = "wrong"
        self.assertEqual(
            gate.validate_topic_header(requirement, header, now).status, gate.FAIL
        )
        header["frame_id"] = "front_camera"
        header["stamp"] = {"sec": int(now) - 20, "nanosec": 0}
        self.assertEqual(
            gate.validate_topic_header(requirement, header, now).status, gate.FAIL
        )

    def test_map_lifecycle_requires_exact_tuple(self) -> None:
        expected = {"map_id": "lab_go2", "mode": "localization", "generation": 8}
        self.assertEqual(
            gate.validate_map_lifecycle_payload(dict(expected), expected).status,
            gate.PASS,
        )
        actual = dict(expected)
        actual["generation"] = 9
        self.assertEqual(
            gate.validate_map_lifecycle_payload(actual, expected).status,
            gate.FAIL,
        )
        actual["generation"] = True
        self.assertEqual(
            gate.validate_map_lifecycle_payload(actual, expected).status,
            gate.UNKNOWN,
        )

    def test_landmark_binding_must_be_verified_and_measured(self) -> None:
        document = {
            "schema_version": 2,
            "map_id": "lab_go2",
            "map_generation": 8,
            "frame_id": "map",
            "landmarks": [
                {
                    "id": "vending_machine_front",
                    "verified": True,
                    "pose": {"x": 1.0, "y": 2.0, "yaw": 0.5},
                }
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "landmarks.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            check, binding = gate.load_landmark_binding(path)
            self.assertEqual(check.status, gate.PASS)
            self.assertEqual(binding["generation"], 8)
            document["landmarks"][0]["verified"] = False
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            check, binding = gate.load_landmark_binding(path)
            self.assertEqual(check.status, gate.FAIL)
            self.assertIsNone(binding)

    def test_action_requires_one_server_and_exact_type(self) -> None:
        output = """Action: /navigate_to_pose
Action clients: 1
    /nav_client [nav2_msgs/action/NavigateToPose]
Action servers: 1
    /bt_navigator [nav2_msgs/action/NavigateToPose]
"""
        result = gate.CommandResult((), 0, output, "")
        self.assertEqual(
            gate.validate_action_output(
                "/navigate_to_pose", "nav2_msgs/action/NavigateToPose", result
            ).status,
            gate.PASS,
        )
        output = output.replace("Action servers: 1", "Action servers: 2")
        result = gate.CommandResult((), 0, output, "")
        self.assertEqual(
            gate.validate_action_output(
                "/navigate_to_pose", "nav2_msgs/action/NavigateToPose", result
            ).status,
            gate.FAIL,
        )

    def test_dynamic_tf_requires_fresh_time(self) -> None:
        now = time.time()
        output = f"At time {now - 0.1:.6f}\n- Translation: [0, 0, 0]\n- Rotation: in Quaternion [0, 0, 0, 1]\n"
        result = gate.CommandResult((), 124, output, "", True)
        self.assertEqual(
            gate.validate_tf_output(
                "map_to_odom", result, dynamic=True, now_s=now
            ).status,
            gate.PASS,
        )
        stale = output.replace(f"{now - 0.1:.6f}", f"{now - 20:.6f}")
        result = gate.CommandResult((), 124, stale, "", True)
        self.assertEqual(
            gate.validate_tf_output(
                "map_to_odom", result, dynamic=True, now_s=now
            ).status,
            gate.FAIL,
        )

    def test_ros_runner_always_prefixes_external_timeout(self) -> None:
        runner = gate.CommandRunner(4.2)
        with mock.patch.object(runner, "run") as run:
            run.return_value = gate.CommandResult((), 0, "", "")
            runner.ros(("ros2", "topic", "echo", "/safe", "--once"))
        argv = run.call_args.args[0]
        self.assertEqual(argv[:3], ("timeout", "--signal=TERM", "5s"))
        self.assertEqual(argv[3], "ros2")

    def test_dashboard_requires_connected_and_fresh_topics(self) -> None:
        topics = {
            name: {"state": "fresh"}
            for name in ("camera", "point_cloud", "map", "odom", "pose_map")
        }
        status = {
            "bridge": {"connected": True},
            "telemetry_read_only": True,
            "topics": topics,
        }
        health = {"ros_connected": True, "telemetry_read_only": True}
        with mock.patch.object(
            gate, "get_json_loopback", side_effect=[(status, "status"), (health, "health")]
        ):
            self.assertEqual(gate.validate_dashboard(8092).status, gate.PASS)
        status["topics"]["camera"]["state"] = "stale"
        with mock.patch.object(
            gate, "get_json_loopback", side_effect=[(status, "status"), (health, "health")]
        ):
            self.assertEqual(gate.validate_dashboard(8092).status, gate.FAIL)

    def test_speech_log_requires_non_mock_streaming_asr_ok(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "speech.log"
            record = {
                "ts": "2026-07-17 00:00:00.000000000",
                "level": "info",
                "tag": "speech",
                "msg": "Backend status: mode=local asr=Whisper (UNAVAILABLE) asr_stream=FunASR streaming ASR (OK) tts=Edge (OK)",
            }
            path.write_text(json.dumps(record) + "\n", encoding="utf-8")
            self.assertEqual(gate.validate_speech_backend(path).status, gate.PASS)
            record["msg"] = record["msg"].replace("mode=local", "mode=mock")
            path.write_text(json.dumps(record) + "\n", encoding="utf-8")
            self.assertEqual(gate.validate_speech_backend(path).status, gate.FAIL)


class ReadinessStaticSafetyTests(unittest.TestCase):
    def test_scripts_contain_no_write_or_motion_cli(self) -> None:
        source = MODULE_PATH.read_text(encoding="utf-8")
        wrapper = (ROOT / "scripts" / "check_stack_readiness.sh").read_text(
            encoding="utf-8"
        )
        combined = source + wrapper
        forbidden = (
            "ros2 topic pub",
            "ros2 action send_goal",
            "ros2 service call",
            "SportClient",
            "sport_mode_ctrl",
            "go2_sport_client",
            "low_level_ctrl",
        )
        for needle in forbidden:
            self.assertNotIn(needle, combined)

    def test_every_ros_command_is_routed_through_bounded_runner(self) -> None:
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertNotIn('subprocess.run(("ros2"', source)
        self.assertGreaterEqual(source.count("runner.ros("), 5)
        self.assertIn('("timeout", "--signal=TERM"', source)

    def test_wrapper_is_read_only_and_executable_after_install_step(self) -> None:
        wrapper = (ROOT / "scripts" / "check_stack_readiness.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("READ-ONLY", wrapper)
        self.assertNotIn("sudo", wrapper)
        self.assertNotIn("apt ", wrapper)
        self.assertNotIn("nmcli con", wrapper)


if __name__ == "__main__":
    unittest.main()
