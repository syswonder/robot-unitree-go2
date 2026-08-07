from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC = ROOT / "deploy" / "time-sync"
ROBOTTRACK_PACKAGE = ROOT / "packages" / "go2_robottrack"
sys.path.insert(0, str(TIME_SYNC))
sys.path.insert(0, str(ROBOTTRACK_PACKAGE))

from go2_robottrack.core import RuntimeConfig  # noqa: E402


def _load(name: str, filename: str):
    path = TIME_SYNC / filename
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load renderer {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RobotTrackManifestTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.renderer = _load(
            "robottrack_renderer",
            "render_workstation_robottrack_manifest.py",
        )
        cls.persistent_renderer = _load(
            "persistent_nav2_renderer_for_robottrack",
            "render_workstation_persistent_nav2_manifest.py",
        )
        cls.base = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )

    def render(self):
        return self.renderer.render(
            self.base,
            state_marker=100,
            passive_state_markers=[100, 1002],
        )

    def persistent(self):
        return self.persistent_renderer.render(
            self.base,
            state_marker=100,
            passive_state_markers=[100, 1002],
        )

    @staticmethod
    def config(manifest, section, name):
        return next(
            item["config"] for item in manifest[section] if item["name"] == name
        )

    @staticmethod
    def without_named(entries, name):
        return [item for item in entries if item.get("name") != name]

    def test_follow_route_has_one_pre_smoother_mux_output(self) -> None:
        manifest = self.render()
        nav = self.config(manifest, "service", "nav2")
        chassis = self.config(manifest, "primitive", "go2_chassis")
        robottrack = self.config(
            manifest, "primitive", self.renderer.ROBOTTRACK_PROVIDER_ID
        )
        mux = robottrack["source_mux"]

        self.assertEqual(
            nav["controller_velocity_output_topic"],
            self.renderer.NAV_RAW_COMMAND_TOPIC,
        )
        self.assertEqual(
            mux,
            {
                "nav_input_topic": self.renderer.NAV_RAW_COMMAND_TOPIC,
                "robottrack_input_topic":
                    self.renderer.ROBOTTRACK_RAW_COMMAND_TOPIC,
                "output_topic": self.renderer.SMOOTHER_INPUT_TOPIC,
                "selected_source": "robottrack",
            },
        )
        self.assertEqual(
            robottrack["command_topic"],
            self.renderer.ROBOTTRACK_RAW_COMMAND_TOPIC,
        )
        self.assertEqual(
            nav["velocity_output_topic"],
            self.renderer.CHASSIS_COMMAND_TOPIC,
        )
        self.assertEqual(
            chassis["twist_in_topic"], self.renderer.CHASSIS_COMMAND_TOPIC
        )
        self.assertEqual(
            len(
                {
                    mux["nav_input_topic"],
                    mux["robottrack_input_topic"],
                    mux["output_topic"],
                    chassis["twist_in_topic"],
                }
            ),
            4,
        )

    def test_robottrack_uses_d435i_and_official_runtime_defaults(self) -> None:
        robottrack = self.config(
            self.render(), "primitive", self.renderer.ROBOTTRACK_PROVIDER_ID
        )
        self.assertEqual(robottrack["mode"], "live")
        self.assertEqual(robottrack["rgb_topic"], "/go2/d435i/color/image_raw")
        self.assertEqual(
            robottrack["server_url"], "http://127.0.0.1:5801/eval_dual"
        )
        self.assertEqual(robottrack["instruction"], "Follow the person ahead")
        self.assertEqual(robottrack["model_input_mode"], "center_crop_height")
        self.assertEqual(robottrack["model_crop_size"], 384)
        self.assertEqual(robottrack["waypoint_strategy"], "first")
        self.assertEqual(robottrack["control_dt"], 0.1)
        self.assertEqual(robottrack["dispatch_hz"], 50.0)
        self.assertEqual(robottrack["max_plan_age_s"], 1.5)
        self.assertEqual(
            robottrack["max_vx"], self.renderer.ROBOTTRACK_LIVE_MAX_VX
        )
        self.assertEqual(robottrack["max_vx"], 0.50)
        self.assertEqual(robottrack["max_wz"], 0.30)

    def test_runtime_endpoint_and_instruction_are_explicitly_overridable(self) -> None:
        manifest = self.renderer.render(
            self.base,
            state_marker=2010,
            passive_state_markers=[100, 1002, 2010],
            server_url="http://127.0.0.1:5901/eval_dual",
            instruction="Follow the person wearing a blue shirt",
        )
        robottrack = self.config(
            manifest, "primitive", self.renderer.ROBOTTRACK_PROVIDER_ID
        )
        self.assertEqual(
            robottrack["server_url"],
            "http://127.0.0.1:5901/eval_dual",
        )
        self.assertEqual(
            robottrack["instruction"],
            "Follow the person wearing a blue shirt",
        )
        self.assertEqual(robottrack["mode"], "live")
        self.assertEqual(
            robottrack["source_mux"]["selected_source"], "robottrack"
        )

        for field in ("server_url", "instruction"):
            kwargs = {field: "  "}
            with self.subTest(field=field), self.assertRaises(
                self.renderer.ManifestError
            ):
                self.renderer.render(
                    self.base,
                    state_marker=100,
                    passive_state_markers=[100, 1002, 2010],
                    **kwargs,
                )
        with self.assertRaises(self.renderer.ManifestError):
            self.renderer.render(
                self.base,
                state_marker=100,
                passive_state_markers=[100, 1002, 2010],
                server_url="http://127.0.0.1:5901/not_the_official_endpoint",
            )

    def test_rendered_config_is_accepted_by_the_robottrack_provider_core(self) -> None:
        robottrack = self.config(
            self.render(), "primitive", self.renderer.ROBOTTRACK_PROVIDER_ID
        )
        runtime = RuntimeConfig.from_mapping(robottrack)
        self.assertEqual(runtime.mode, "live")
        self.assertEqual(runtime.rgb_topic, self.renderer.D435I_RGB_TOPIC)
        self.assertEqual(
            runtime.nav_raw_topic, self.renderer.NAV_RAW_COMMAND_TOPIC
        )
        self.assertEqual(
            runtime.robottrack_raw_topic,
            self.renderer.ROBOTTRACK_RAW_COMMAND_TOPIC,
        )
        self.assertEqual(
            runtime.selected_output_topic,
            self.renderer.SMOOTHER_INPUT_TOPIC,
        )
        self.assertEqual(runtime.selected_source, "robottrack")

    def test_generation1_map_nav_baseline_has_only_robottrack_runtime_overrides(
        self,
    ) -> None:
        persistent = self.persistent()
        follow = self.render()

        self.assertEqual(follow["system"], persistent["system"])
        self.assertEqual(follow["skill"], persistent["skill"])
        follow_primitives = copy.deepcopy(
            self.without_named(
                follow["primitive"], self.renderer.ROBOTTRACK_PROVIDER_ID
            )
        )
        follow_chassis = next(
            item["config"]
            for item in follow_primitives
            if item["name"] == "go2_chassis"
        )
        self.assertIs(follow_chassis["preserve_classic_walk"], False)
        follow_chassis["preserve_classic_walk"] = True
        persistent_chassis = self.config(
            persistent, "primitive", "go2_chassis"
        )
        self.assertEqual(follow_chassis["max_linear_x_mps"], 0.50)
        follow_chassis["max_linear_x_mps"] = persistent_chassis[
            "max_linear_x_mps"
        ]
        self.assertEqual(follow_primitives, persistent["primitive"])

        follow_services = copy.deepcopy(follow["service"])
        follow_nav = next(
            item["config"] for item in follow_services if item["name"] == "nav2"
        )
        follow_nav.pop("controller_velocity_output_topic")
        self.assertEqual(follow_services, persistent["service"])

        follow_env = copy.deepcopy(follow["env"])
        follow_env.pop("ROBOTTRACK_UPSTREAM_ROOT")
        self.assertEqual(follow_env, persistent["env"])

        mapping = self.config(follow, "service", "mapping")
        persistent_mapping = self.config(persistent, "service", "mapping")
        self.assertEqual(mapping, persistent_mapping)
        self.assertEqual(mapping["map_id"], "${GO2_MAP_ID}")
        self.assertEqual(mapping["map_mode"], "localization")
        self.assertIs(mapping["reset_map"], False)

        chassis = self.config(follow, "primitive", "go2_chassis")
        persistent_chassis = self.config(
            persistent, "primitive", "go2_chassis"
        )
        self.assertIs(chassis["preserve_classic_walk"], False)
        self.assertIs(persistent_chassis["preserve_classic_walk"], True)
        self.assertEqual(chassis["motion_profile"], self.renderer.PROFILE)

    def test_persistent_validator_is_not_relaxed_for_robottrack_override(self) -> None:
        persistent = self.persistent()
        self.config(persistent, "primitive", "go2_chassis").update(
            {"preserve_classic_walk": False}
        )
        with self.assertRaises(self.persistent_renderer.ManifestError):
            self.persistent_renderer.validate_rendered(persistent)

    def test_persistent_validation_copy_restores_ordinary_speed(self) -> None:
        manifest = self.render()

        def validate_persistent_copy(persistent_view):
            chassis = self.config(
                persistent_view, "primitive", "go2_chassis"
            )
            self.assertEqual(chassis["max_linear_x_mps"], 0.30)

        with mock.patch.object(
            self.renderer,
            "validate_persistent",
            side_effect=validate_persistent_copy,
        ):
            self.renderer.validate_rendered(manifest)

    def test_renderer_rejects_mux_map_or_existing_route_drift(self) -> None:
        mutations = (
            lambda manifest: self.config(
                manifest,
                "primitive",
                self.renderer.ROBOTTRACK_PROVIDER_ID,
            )["source_mux"].update({"output_topic": "/cmd_vel"}),
            lambda manifest: self.config(
                manifest, "service", "nav2"
            ).update({"controller_velocity_output_topic": "/cmd_vel_nav"}),
            lambda manifest: self.config(
                manifest, "primitive", "go2_chassis"
            ).update({"preserve_classic_walk": True}),
            lambda manifest: self.config(
                manifest, "service", "mapping"
            ).update({"reset_map": True}),
        )
        for mutate in mutations:
            manifest = self.render()
            mutate(manifest)
            with self.assertRaises(self.renderer.ManifestError):
                self.renderer.validate_rendered(manifest)

    def test_render_does_not_mutate_generic_manifest(self) -> None:
        original = copy.deepcopy(self.base)
        self.render()
        self.assertEqual(self.base, original)

    def test_cli_writes_the_same_valid_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "robottrack.yaml"
            argv = [
                "render_workstation_robottrack_manifest.py",
                "--base",
                str(ROOT / "robonix_manifest.yaml"),
                "--output",
                str(output),
                "--state-marker",
                "100",
                "--passive-state-markers",
                "100,1002",
            ]
            previous = sys.argv
            try:
                sys.argv = argv
                self.assertEqual(self.renderer.main(), 0)
            finally:
                sys.argv = previous
            written = yaml.safe_load(output.read_text(encoding="utf-8"))
        self.renderer.validate_rendered(written)
        self.assertEqual(written, self.render())


if __name__ == "__main__":
    unittest.main()
