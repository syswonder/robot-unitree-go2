# SPDX-License-Identifier: Apache-2.0
import json
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock

from go2_sport_actions.runtime import ActionManager, PLANS
from go2_sport_actions.skills import BY_ACTION, DEFINITIONS
import test_runtime as fixtures
FakeDaemon = fixtures.FakeDaemon


class SplitSkillTests(unittest.TestCase):
    def test_seven_unique_named_identities_in_one_package(self):
        self.assertEqual(set(BY_ACTION), {"hello", "stretch", "new_year_greeting",
                         "crouch", "dance", "handstand", "handstand_walk"})
        self.assertEqual(len({d["provider_id"] for d in DEFINITIONS}), 7)
        root = Path(__file__).resolve().parents[1]
        for d in DEFINITIONS:
            self.assertTrue(d["provider_id"].startswith("unitree_go2_"))
            self.assertEqual(d["init"]["action"], d["action"])
            self.assertIn("宇树 Go2", d["description"])
            manifest = (root / f'package_manifest.{d["action"]}.yaml').read_text()
            self.assertIn(f'start: bash start.sh {d["action"]}', manifest)
            self.assertIn(f'{d["namespace"]}/execute', manifest)
            doc = (root / "docs" / "skills" / f'{d["action"]}.md').read_text()
            self.assertIn(d["sdk"], doc)

    def test_model_never_supplies_action_name_or_dance_number(self):
        root = Path(__file__).resolve().parents[1]
        request = (root / "capabilities/lib/go2_sport_actions/srv/PerformAction.srv").read_text().split("---")[0]
        fields = [l for l in request.splitlines() if l and not l.startswith("#")]
        self.assertEqual(fields, ["string request_id"])
        demo = json.loads((root / "examples/demo_sequence.json").read_text())
        self.assertEqual(len(demo["steps"]), 7)
        self.assertEqual(demo["steps"][4]["utterance"], "跳个舞吧")

    def test_random_both_branches_keep_accepted_plans(self):
        for selected, action_id, duration in (("dance_1", 3, 45), ("dance_2", 4, 55)):
            chooser = Mock(return_value=selected)
            FakeDaemon.calls = []
            manager = ActionManager("/fake", client_factory=FakeDaemon, chooser=chooser)
            manager._hold = lambda *_: None
            result = manager.execute("dance", "same-id")
            state = fixtures.RuntimeTests.finished(self, manager, result["run_id"])
            repeated = manager.execute("dance", "same-id")
            self.assertEqual(result["run_id"], repeated["run_id"])
            self.assertEqual(state["name"], selected)
            self.assertEqual(state["requested_name"], "dance")
            chooser.assert_called_once_with(("dance_1", "dance_2"))
            self.assertEqual(PLANS[selected][0].duration_s, duration)
            self.assertEqual(FakeDaemon.calls[1][1]["action_id"], action_id)
            self.assertFalse(manager.execute("hello", "same-id")["accepted"])

    def test_dance_configuration_cannot_dispatch_unaccepted_sdk_actions(self):
        for variants in ([], ["dance_1", "dance_1"], ["backflip"], ["hello"], "dance_1"):
            with self.assertRaises(ValueError):
                ActionManager(dance_variants=variants)

    def test_unconfigured_or_busy_does_not_sample_or_start_another_action(self):
        chooser = Mock(return_value="dance_2")
        self.assertFalse(ActionManager(chooser=chooser).execute("dance")["accepted"])
        chooser.assert_not_called()
        ready, release = threading.Event(), threading.Event()
        manager = ActionManager("/fake", client_factory=FakeDaemon, chooser=chooser)
        def hold(*_):
            ready.set()
            release.wait(2)
        manager._hold = hold
        first = manager.execute("hello", "one")
        try:
            self.assertTrue(ready.wait(1))
            self.assertEqual(manager.execute("dance", "two")["reason"], "action_in_progress")
            chooser.assert_not_called()
        finally:
            release.set()
            fixtures.RuntimeTests.finished(self, manager, first["run_id"])


if __name__ == "__main__":
    unittest.main()
