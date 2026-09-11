# SPDX-License-Identifier: Apache-2.0
"""Network-free regression tests for the source-only onboarding tools."""

import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import prepare
import inspect_model


class PathTests(unittest.TestCase):
    def test_reject_unsafe_or_noncanonical_paths(self):
        for value in ("../outside", "/absolute", "a/../b", "a\\b", "", ".", "a//b", "a/./b", "a/"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                prepare.relative_path(value)

    def test_reject_unrelated_upstream_file(self):
        with self.assertRaises(ValueError):
            prepare.destination("simulate_python/config.py")

    def test_expected_destinations(self):
        self.assertEqual(prepare.destination("LICENSE"), "vendor/LICENSE")
        self.assertEqual(prepare.destination("unitree_robots/go2/go2.xml"), "vendor/go2.xml")

    def test_lock_has_exact_model_assets_and_license(self):
        lock = prepare.load_lock()
        self.assertEqual(len(lock["files"]), 18)
        self.assertEqual(len([e for e in lock["files"] if e["path"].endswith(".obj")]), 16)
        self.assertEqual(lock["license"], "BSD-3-Clause")


class PreparationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_idempotent_write(self):
        path = self.root / "asset"
        prepare.write_unchanged(path, b"original")
        prepare.write_unchanged(path, b"original")
        self.assertEqual(path.read_bytes(), b"original")

    def test_preserves_modified_asset(self):
        path = self.root / "asset"
        path.write_bytes(b"user change")
        with self.assertRaises(ValueError):
            prepare.write_unchanged(path, b"upstream")
        self.assertEqual(path.read_bytes(), b"user change")

    def test_rejects_symlink_escape(self):
        root = self.root / "bundle"
        root.mkdir()
        (root / "vendor").symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(ValueError):
            prepare.checked_path(root, "vendor/outside")

    def fake_lock(self):
        return {"repository": "unitreerobotics/unitree_mujoco", "commit": "a" * 40,
                "files": [{"path": "unitree_robots/go2/go2.xml",
                           "sha256": hashlib.sha256(b"model").hexdigest()}]}

    def test_offline_copy_and_reuse_never_use_network(self):
        source = self.root / "source"
        model = source / "unitree_robots/go2/go2.xml"
        model.parent.mkdir(parents=True)
        model.write_bytes(b"model")
        output = self.root / "out"
        with patch.object(prepare, "load_lock", return_value=self.fake_lock()), \
             patch.object(prepare.urllib.request, "urlopen", side_effect=AssertionError("network")):
            prepare.prepare(output, source)
            result = prepare.prepare(output)
        self.assertEqual(result["files_verified"], 1)
        self.assertEqual((output / "vendor/go2.xml").read_bytes(), b"model")
        self.assertEqual(json.loads((output / "index.json").read_text()),
                         ["vendor/go2.xml", "vendor/scene.xml"])

    def test_rejects_corrupt_source_before_copy(self):
        source = self.root / "source"
        model = source / "unitree_robots/go2/go2.xml"
        model.parent.mkdir(parents=True)
        model.write_bytes(b"corrupt")
        output = self.root / "out"
        with patch.object(prepare, "load_lock", return_value=self.fake_lock()), self.assertRaises(ValueError):
            prepare.prepare(output, source)
        self.assertFalse((output / "vendor/go2.xml").exists())

    def test_rejects_modified_output_without_network(self):
        output = self.root / "out"
        path = output / "vendor/go2.xml"
        path.parent.mkdir(parents=True)
        path.write_bytes(b"user change")
        with patch.object(prepare, "load_lock", return_value=self.fake_lock()), \
             patch.object(prepare.urllib.request, "urlopen", side_effect=AssertionError("network")), \
             self.assertRaises(ValueError):
            prepare.prepare(output)
        self.assertEqual(path.read_bytes(), b"user change")


class InspectionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        vendor = self.root / "vendor"
        vendor.mkdir()
        model = ET.Element("mujoco")
        ET.SubElement(model, "asset")
        body = ET.SubElement(ET.SubElement(model, "worldbody"), "body", name="base_link")
        ET.SubElement(body, "freejoint")
        ET.SubElement(body, "inertial", mass="1.5")
        for leg in ("FL", "FR", "RL", "RR"):
            for joint in ("hip", "thigh", "calf"):
                ET.SubElement(body, "joint", name=f"{leg}_{joint}_joint")
        actuators = ET.SubElement(model, "actuator")
        for name in inspect_model.SDK_JOINTS:
            ET.SubElement(actuators, "motor", name=name.removesuffix("_joint"), joint=name)
        data = ET.tostring(model)
        (vendor / "go2.xml").write_bytes(data)
        (vendor / "scene.xml").write_text(prepare.SCENE)
        (self.root / "index.json").write_text(json.dumps(["vendor/go2.xml", "vendor/scene.xml"]))
        (self.root / "upstream.lock.json").write_bytes((prepare.HERE / "upstream.lock.json").read_bytes())
        lock = {"commit": "a" * 40, "files": [{"path": "unitree_robots/go2/go2.xml",
                                                  "sha256": hashlib.sha256(data).hexdigest()}]}
        mock = patch.object(inspect_model, "load_lock", return_value=lock)
        mock.start()
        self.addCleanup(mock.stop)

    def test_named_mapping_not_xml_position(self):
        report = inspect_model.inspect_bundle(self.root)
        self.assertEqual(report["joint_mapping"][0]["xml_joint_order"], 3)
        self.assertEqual(report["joint_mapping"][3]["xml_joint_order"], 0)
        self.assertEqual(report["model_mass_sum_kg"], 1.5)

    def test_static_result_never_claims_runtime_or_walking(self):
        report = inspect_model.inspect_bundle(self.root)
        self.assertFalse(report["runtime_validated"])
        self.assertFalse(report["walking_controller_included"])

    def test_rejects_modified_model(self):
        (self.root / "vendor/go2.xml").write_text("<mujoco/>")
        with self.assertRaisesRegex(ValueError, "SHA256"):
            inspect_model.inspect_bundle(self.root)

    def test_rejects_modified_scene(self):
        (self.root / "vendor/scene.xml").write_text("<mujoco/>")
        with self.assertRaisesRegex(ValueError, "scene"):
            inspect_model.inspect_bundle(self.root)

    def test_rejects_changed_provenance(self):
        (self.root / "upstream.lock.json").write_text("{}")
        with self.assertRaisesRegex(ValueError, "provenance"):
            inspect_model.inspect_bundle(self.root)

    def test_rejects_incomplete_or_duplicate_index(self):
        for index in (["vendor/go2.xml"], ["vendor/go2.xml", "vendor/scene.xml", "vendor/go2.xml"]):
            (self.root / "index.json").write_text(json.dumps(index))
            with self.subTest(index=index), self.assertRaisesRegex(ValueError, "index"):
                inspect_model.inspect_bundle(self.root)

    def test_rejects_external_index_entry(self):
        (self.root / "index.json").write_text(json.dumps(["../outside"]))
        with self.assertRaisesRegex(ValueError, "index"):
            inspect_model.inspect_bundle(self.root)


if __name__ == "__main__":
    unittest.main()
