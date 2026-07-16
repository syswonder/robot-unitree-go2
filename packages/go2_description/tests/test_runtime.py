from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

import yaml

from go2_description_provider.runtime import (
    require_pinned_urdf,
    validate_urdf,
    write_robot_state_publisher_params,
)


ROOT = Path(__file__).resolve().parents[1]
PINNED = ROOT / "urdf" / "go2_robonix.urdf"


class DescriptionRuntimeTests(unittest.TestCase):
    def test_pinned_model_has_required_single_root(self) -> None:
        root, links, joints = validate_urdf(PINNED.read_text(encoding="utf-8"))
        self.assertEqual(root, "base_link")
        self.assertGreater(links, 20)
        self.assertGreater(joints, 20)

    def test_exact_pinned_model_is_required(self) -> None:
        text = PINNED.read_text(encoding="utf-8")
        digest = require_pinned_urdf(text, PINNED)
        self.assertEqual(len(digest), 64)
        with self.assertRaisesRegex(ValueError, "does not match"):
            require_pinned_urdf(
                text.replace(
                    '<robot name="go2_description">',
                    '<robot name="go2_changed">',
                    1,
                ),
                PINNED,
            )

    def test_external_entities_and_duplicate_links_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "DTD"):
            validate_urdf('<!DOCTYPE robot><robot name="x"><link name="base_link"/></robot>')
        with self.assertRaisesRegex(ValueError, "duplicate"):
            validate_urdf('<robot name="x"><link name="base_link"/><link name="base_link"/></robot>')

    def test_parameter_file_preserves_pinned_urdf(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "params.yaml"
            text = PINNED.read_text(encoding="utf-8")
            write_robot_state_publisher_params(target, text)
            data = yaml.safe_load(target.read_text(encoding="utf-8"))
            self.assertEqual(data["/**"]["ros__parameters"]["robot_description"], text)

    def test_manifest_exposes_only_description_driver(self) -> None:
        manifest = yaml.safe_load((ROOT / "package_manifest.yaml").read_text(encoding="utf-8"))
        self.assertEqual(
            manifest["capabilities"],
            [{"name": "robonix/primitive/robot_description/driver"}],
        )
        source = (ROOT / "go2_description_provider" / "main.py").read_text(encoding="utf-8")
        for forbidden in ("create_publisher", "SportClient", "/cmd_vel", "/lowcmd"):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
