from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[1]
SMOKE = ROOT / "scripts" / "robottrack_checkpoint_smoke.py"
LAUNCHER = ROOT / "scripts" / "start_robottrack_inference_server.sh"
SPEC = importlib.util.spec_from_file_location("robottrack_checkpoint_smoke", SMOKE)
assert SPEC is not None and SPEC.loader is not None
smoke = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = smoke
SPEC.loader.exec_module(smoke)


class RobotTrackCheckpointSmokeTests(unittest.TestCase):
    def test_defaults_are_workspace_local_and_support_cpu_or_gpu(self) -> None:
        parser = smoke._parser({})
        args = parser.parse_args([])
        self.assertEqual(args.device, "auto")
        self.assertEqual(args.warmup, 1)
        self.assertEqual(args.iterations, 5)
        self.assertTrue(args.checkpoint.is_relative_to(WORKSPACE))

        custom = parser.parse_args(["--device", "cpu", "--iterations", "2"])
        self.assertEqual(custom.device, "cpu")
        self.assertEqual(custom.iterations, 2)

    def test_upstream_environment_resolves_relative_to_workspace(self) -> None:
        expected = WORKSPACE / "somewhere" / smoke.CHECKPOINT_RELATIVE
        actual = smoke._checkpoint_default(
            {"ROBOTTRACK_UPSTREAM_ROOT": "somewhere"}
        )
        self.assertEqual(actual, expected.resolve())

    def test_smoke_is_offline_non_ros_and_reports_required_json_sections(self) -> None:
        source = SMOKE.read_text(encoding="utf-8")
        for required in (
            '"TRANSFORMERS_OFFLINE"] = "1"',
            '"HF_HUB_OFFLINE"] = "1"',
            "local_files_only=True",
            '"checkpoint_provenance"',
            '"revision_source"',
            '"sha256_source"',
            "_sha256_file",
            '"versions"',
            '"load"',
            '"shape"',
            '"finite"',
            '"warm_latency_ms"',
            '"vram"',
            "coarse_tokens",
            "fine_tokens",
        ):
            with self.subTest(required=required):
                self.assertIn(required, source)
        for forbidden in (
            "import requests",
            "import urllib",
            "rclpy",
            "ros2 topic",
            "unitree_sdk",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)

    def test_incomplete_checkpoint_returns_json_without_importing_ml_stack(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            completed = subprocess.run(
                [sys.executable, str(SMOKE), "--checkpoint", directory],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=10,
            )
        self.assertEqual(completed.returncode, 1)
        result = json.loads(completed.stdout)
        self.assertFalse(result["ok"])
        self.assertTrue(result["offline"])
        self.assertEqual(result["error"]["type"], "FileNotFoundError")


class RobotTrackServerLauncherTests(unittest.TestCase):
    @staticmethod
    def _fake_python(directory: Path) -> tuple[Path, Path]:
        executable = directory / "fake-python"
        trace = directory / "trace.jsonl"
        executable.write_text(
            """#!/usr/bin/env python3
import json
import os
from pathlib import Path
import sys

record = {
    "argv": sys.argv[1:],
    "camera_source": os.environ.get("ROBOTTRACK_CAMERA_SOURCE"),
    "offline": os.environ.get("TRANSFORMERS_OFFLINE"),
    "pythonpath": os.environ.get("PYTHONPATH"),
}
with Path(os.environ["FAKE_TRACE"]).open("a", encoding="utf-8") as handle:
    handle.write(json.dumps(record) + "\\n")
if sys.argv[1].endswith("verify_robottrack_assets.py"):
    sys.exit(int(os.environ.get("FAKE_VERIFY_STATUS", "0")))
""",
            encoding="utf-8",
        )
        executable.chmod(0o755)
        return executable, trace

    def test_launcher_verifies_then_executes_only_official_local_server(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as directory_name:
            directory = Path(directory_name)
            fake_python, trace = self._fake_python(directory)
            environment = os.environ.copy()
            environment.update(
                {
                    "PYTHON": str(fake_python),
                    "FAKE_TRACE": str(trace),
                    "PYTHONPATH": "preserved-entry",
                }
            )
            completed = subprocess.run(
                ["bash", str(LAUNCHER)],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=environment,
                timeout=10,
            )
            records = [
                json.loads(line)
                for line in trace.read_text(encoding="utf-8").splitlines()
            ]
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertEqual(len(records), 2)
        self.assertTrue(records[0]["argv"][0].endswith("verify_robottrack_assets.py"))
        server_args = records[1]["argv"]
        self.assertTrue(
            server_args[1].endswith("http_minicpm_robot_track_server.py")
        )
        self.assertEqual(server_args[server_args.index("--host") + 1], "127.0.0.1")
        self.assertEqual(server_args[server_args.index("--dino_backend") + 1], "torch")
        self.assertEqual(server_args[server_args.index("--siglip_backend") + 1], "torch")
        self.assertEqual(
            server_args[server_args.index("--velocity-controller") + 1],
            "lookahead",
        )
        self.assertEqual(
            server_args[server_args.index("--controller-lookahead-idx") + 1],
            "2",
        )
        self.assertEqual(
            server_args[
                server_args.index("--controller-feedforward-points") + 1
            ],
            "3",
        )
        self.assertEqual(
            server_args[server_args.index("--controller-ff-blend") + 1],
            "0.65",
        )
        self.assertNotIn("--allow-reverse", server_args)
        self.assertEqual(records[1]["camera_source"], "d435i")
        self.assertEqual(records[1]["offline"], "1")
        self.assertIn(".tools/robottrack-python", records[1]["pythonpath"])
        self.assertIn("preserved-entry", records[1]["pythonpath"])

    def test_controller_can_be_explicitly_restored_to_direct(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as directory_name:
            directory = Path(directory_name)
            fake_python, trace = self._fake_python(directory)
            environment = os.environ.copy()
            environment.update(
                {
                    "PYTHON": str(fake_python),
                    "FAKE_TRACE": str(trace),
                    "ROBOTTRACK_VELOCITY_CONTROLLER": "direct",
                }
            )
            completed = subprocess.run(
                ["bash", str(LAUNCHER)],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=environment,
                timeout=10,
            )
            records = [
                json.loads(line)
                for line in trace.read_text(encoding="utf-8").splitlines()
            ]
        self.assertEqual(completed.returncode, 0, completed.stderr)
        server_args = records[1]["argv"]
        self.assertEqual(
            server_args[server_args.index("--velocity-controller") + 1],
            "direct",
        )

    def test_asset_failure_exits_before_server_without_downloading(self) -> None:
        with tempfile.TemporaryDirectory(dir=ROOT) as directory_name:
            directory = Path(directory_name)
            fake_python, trace = self._fake_python(directory)
            environment = os.environ.copy()
            environment.update(
                {
                    "PYTHON": str(fake_python),
                    "FAKE_TRACE": str(trace),
                    "FAKE_VERIFY_STATUS": "1",
                }
            )
            completed = subprocess.run(
                ["bash", str(LAUNCHER)],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=environment,
                timeout=10,
            )
            records = trace.read_text(encoding="utf-8").splitlines()
        self.assertEqual(completed.returncode, 1)
        self.assertEqual(len(records), 1)
        self.assertIn("asset verification failed", completed.stderr)
        self.assertNotIn("download", completed.stdout.lower())

    def test_launcher_contains_no_download_install_ros_or_unitree_commands(self) -> None:
        source = LAUNCHER.read_text(encoding="utf-8")
        for forbidden in (
            "curl ",
            "wget ",
            "pip install",
            "apt ",
            "sudo ",
            "ros2 ",
            "unitree_sdk",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
