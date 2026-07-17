from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
TIME_DIR = ROOT / "deploy" / "time-sync"

import sys

sys.path.insert(0, str(TIME_DIR))

import go2_time_core


def load_script(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / filename)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


probe = load_script("time_probe_signal", "probe_go2_time_readonly.py")
recovery = load_script("time_probe_recovery", "recover_go2_time_summary.py")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class TimeProbeRecoveryTest(unittest.TestCase):
    def make_evidence(self, directory: Path) -> tuple[Path, Path]:
        metadata = {
            "schema_version": 1,
            "mode": "read-only-no-adjust",
            "clock_adjustment_requested": False,
            "ros_publishers_created": False,
            "unitree_clients_created": False,
            "started_realtime_ns": 100_000_000_000,
            "started_monotonic_ns": 5_000_000_000,
            "duration_seconds": 60,
            "max_samples": 1000,
            "retained_offsets_per_stream": 100,
            "topics": {"sport_primary": "/sportmodestate"},
        }
        metadata_path = directory / "metadata.json"
        metadata_path.write_text(json.dumps(metadata) + "\n", encoding="utf-8")
        tracker = go2_time_core.StreamTracker("sport_primary", "/sportmodestate")
        records = []
        for index in range(3):
            records.append(
                tracker.observe(
                    100 + index,
                    0,
                    100_500_000_000 + index * 1_000_000_000,
                    5_500_000_000 + index * 1_000_000_000,
                    200 + index,
                ).as_dict()
            )
        samples_path = directory / "samples.jsonl"
        samples_path.write_text(
            "".join(json.dumps(record, sort_keys=True) + "\n" for record in records),
            encoding="utf-8",
        )
        return metadata_path, samples_path

    def test_recovery_validates_raw_records_without_modifying_them(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as temporary:
            directory = Path(temporary)
            metadata_path, samples_path = self.make_evidence(directory)
            before = (digest(metadata_path), digest(samples_path))
            output, summary = recovery.recover_summary(
                directory, "summary.recovered.json"
            )
            self.assertEqual(before, (digest(metadata_path), digest(samples_path)))
            self.assertEqual(summary["total_samples"], 3)
            self.assertEqual(
                summary["exit_reason"], "incomplete_before_duration_or_sample_cap"
            )
            self.assertEqual(summary["termination_cause"], "unknown")
            self.assertFalse(summary["duration_boundary_observed"])
            self.assertFalse(summary["sample_cap_observed"])
            self.assertFalse(summary["safe_for_clock_discipline"])
            self.assertEqual(summary["raw_observation_validation_errors"], 0)
            self.assertEqual(output.stat().st_mode & 0o777, 0o600)
            with self.assertRaises(FileExistsError):
                recovery.recover_summary(directory, "summary.recovered.json")

    def test_recovery_rejects_edited_observation(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT / "logs") as temporary:
            directory = Path(temporary)
            _metadata_path, samples_path = self.make_evidence(directory)
            records = [
                json.loads(line)
                for line in samples_path.read_text(encoding="utf-8").splitlines()
            ]
            records[1]["source_delta_ns"] = 7
            samples_path.write_text(
                "".join(json.dumps(record) + "\n" for record in records),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "does not match reconstruction"):
                recovery.recover_summary(directory, "summary.recovered.json")
            self.assertFalse((directory / "summary.recovered.json").exists())

    def test_recovery_rejects_paths_outside_repository_logs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            with self.assertRaisesRegex(ValueError, "must resolve below"):
                recovery.recover_summary(directory, "summary.recovered.json")
            with tempfile.TemporaryDirectory(dir=ROOT / "logs") as logs_temporary:
                symlink = Path(logs_temporary) / "outside-link"
                symlink.symlink_to(directory, target_is_directory=True)
                with self.assertRaisesRegex(ValueError, "must resolve below"):
                    recovery._validated_evidence_directory(symlink)

    def test_recovery_output_cannot_impersonate_canonical_evidence(self) -> None:
        for invalid in (
            "summary.json",
            "metadata.json",
            "samples.jsonl",
            "../summary.recovered.json",
            "summary.recovered.json.tmp",
            "summary.recovered-.json",
        ):
            with self.subTest(invalid=invalid):
                with self.assertRaisesRegex(ValueError, "independent"):
                    recovery._validated_output_name(invalid)
        for valid in (
            "summary.recovered.json",
            "summary.recovered-canonical.json",
            "summary.recovered.audit-2.json",
        ):
            self.assertEqual(recovery._validated_output_name(valid), valid)

    def test_sigterm_is_deferred_to_auditable_exit_reason(self) -> None:
        request = probe.TerminationRequest()
        request.handler(signal.SIGTERM, None)
        request.handler(signal.SIGINT, None)
        self.assertTrue(request.requested)
        self.assertEqual(request.exit_reason, "signal_sigterm")
        self.assertEqual(request.exit_status, 143)

    def test_offline_fake_ros_sigterm_still_writes_summary(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            temporary_path = Path(temporary)
            fake_modules = temporary_path / "fake-modules"
            output = temporary_path / "evidence"
            ready = temporary_path / "fake-ros-ready"
            files = {
                "rclpy/__init__.py": (
                    "import time\n"
                    "_ok = True\n"
                    "def init(args=None):\n    pass\n"
                    "def ok():\n    return _ok\n"
                    "def spin_once(node, timeout_sec=0.1):\n"
                    "    time.sleep(min(timeout_sec, 0.05))\n"
                    "def shutdown():\n"
                    "    global _ok\n    _ok = False\n"
                ),
                "rclpy/node.py": (
                    "import os\nfrom pathlib import Path\n"
                    "class Node:\n"
                    "    def __init__(self, name):\n"
                    "        Path(os.environ['FAKE_ROS_READY']).touch()\n"
                    "    def create_subscription(self, *args, **kwargs):\n"
                    "        return object()\n"
                    "    def destroy_node(self):\n        pass\n"
                ),
                "rclpy/qos.py": (
                    "class DurabilityPolicy:\n    VOLATILE = 1\n"
                    "class HistoryPolicy:\n    KEEP_LAST = 1\n"
                    "class ReliabilityPolicy:\n    BEST_EFFORT = 1\n"
                    "class QoSProfile:\n"
                    "    def __init__(self, **kwargs):\n        pass\n"
                ),
                "sensor_msgs/__init__.py": "",
                "sensor_msgs/msg/__init__.py": (
                    "class Imu:\n    pass\nclass PointCloud2:\n    pass\n"
                ),
                "unitree_go/__init__.py": "",
                "unitree_go/msg/__init__.py": "class SportModeState:\n    pass\n",
            }
            for relative, source in files.items():
                path = fake_modules / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(source, encoding="utf-8")
            environment = os.environ.copy()
            environment["PYTHONPATH"] = str(fake_modules)
            environment["FAKE_ROS_READY"] = str(ready)
            process = subprocess.Popen(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "probe_go2_time_readonly.py"),
                    "--output-dir",
                    str(output),
                    "--duration-seconds",
                    "30",
                    "--max-samples",
                    "100",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=environment,
            )
            deadline = time.monotonic() + 5
            while not ready.exists() and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(ready.exists(), "fake ROS probe did not become ready")
            process.send_signal(signal.SIGTERM)
            stdout, stderr = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 143, (stdout, stderr))
            summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["exit_reason"], "signal_sigterm")
            self.assertEqual(summary["total_samples"], 0)
            self.assertEqual(summary["cleanup_errors"], [])
            self.assertFalse(summary["safe_for_clock_discipline"])
            self.assertEqual((output / "summary.json").stat().st_mode & 0o777, 0o600)

    def test_wrapper_requests_graceful_term_and_preserves_status(self) -> None:
        source = (ROOT / "scripts" / "probe_go2_time_readonly.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("timeout --signal=TERM --kill-after=15s --preserve-status", source)
        self.assertIn('[[ -s "${OUTPUT_DIR}/summary.json" ]]', source)
        self.assertNotIn("timeout --signal=INT --kill-after=5s", source)

    def test_sigterm_handler_restoration_covers_entire_probe(self) -> None:
        source = (ROOT / "scripts" / "probe_go2_time_readonly.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("return execute_probe()", source)
        self.assertIn(
            "signal.signal(signal.SIGTERM, previous_sigterm_handler)", source
        )
        self.assertIn("This outer finally covers normal return", source)

    def test_recovery_source_has_no_ros_network_or_publish_path(self) -> None:
        source = (ROOT / "scripts" / "recover_go2_time_summary.py").read_text(
            encoding="utf-8"
        )
        for forbidden in (
            "import rclpy",
            "create_publisher",
            "create_subscription",
            "SportClient",
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
            "subprocess",
            "socket",
        ):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
