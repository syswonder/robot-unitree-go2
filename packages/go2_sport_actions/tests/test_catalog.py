# SPDX-License-Identifier: Apache-2.0
import ast
import json
from pathlib import Path
import unittest

from go2_sport_actions.catalog import ACTIONS, catalog, dumps, preview
from go2_sport_actions.runtime import ActionManager


class CatalogTests(unittest.TestCase):
    def test_only_accepted_actions_are_published(self):
        for name in ("backflip", "front_jump", "sway", "bow"):
            self.assertFalse(preview(name)["known"])

    def test_handstand_has_explicit_bool_directions(self):
        self.assertEqual(preview("handstand")["sdk_arguments"], (True, False))

    def test_handstand_walk_is_not_guessed(self):
        self.assertFalse(preview("handstand_walk")["sdk_declared"])
        self.assertTrue(preview("handstand_walk")["operator_reported_manual_support"])

    def test_removed_bow_is_not_executable(self):
        self.assertFalse(preview("bow")["known"])
        self.assertFalse(ActionManager().execute("bow")["accepted"])

    def test_sdk_declaration_never_implies_hardware_readiness(self):
        for action in catalog()["actions"]:
            with self.subTest(action=action["name"]):
                self.assertFalse(action["physical_plan_prepared"])
                verified = action["name"] in {
                    "dance_1", "dance_2", "hello", "stretch", "new_year_greeting", "crouch",
                    "handstand", "handstand_walk",
                }
                self.assertEqual(action["firmware_verified"], verified)
                self.assertEqual(action["physical_tested"], verified)

    def test_every_execute_rejects_without_fake_task(self):
        for action in ACTIONS:
            with self.subTest(action=action.name):
                result = ActionManager().execute(action.name, "request-1")
                self.assertFalse(result["accepted"])
                self.assertFalse(result["motion_sent"])
                self.assertEqual(result["run_id"], "")

    def test_no_fuzzy_or_raw_sdk_dispatch(self):
        for name in ("BackFlip", "backflip; Move", "", "做后空翻", None, [], {}):
            with self.subTest(name=name):
                self.assertFalse(preview(name)["known"])
                self.assertFalse(ActionManager().execute(name)["accepted"])

    def test_no_false_completion(self):
        self.assertFalse(ActionManager().status("arbitrary")["known"])
        self.assertEqual(ActionManager().status("arbitrary")["state"], "UNKNOWN")

    def test_cancel_does_not_claim_robot_stopped(self):
        self.assertFalse(ActionManager().cancel("arbitrary")["ok"])
        self.assertFalse(ActionManager().cancel("arbitrary")["stop_acknowledged"])

    def test_json_roundtrip(self):
        listing = json.loads(dumps(catalog()))
        self.assertTrue(listing["physical_validation_complete"])
        self.assertIn("one Go2 EDU", listing["validation_scope"])

    def test_names_unique(self):
        self.assertEqual(len(ACTIONS), len({a.name for a in ACTIONS}))

    def test_catalog_has_no_sdk_or_network_imports(self):
        source = Path(__file__).resolve().parents[1] / "go2_sport_actions/catalog.py"
        tree = ast.parse(source.read_text())
        imports = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                imports.add(node.module)
        self.assertEqual(imports, {"dataclasses", "json"})


if __name__ == "__main__":
    unittest.main()
