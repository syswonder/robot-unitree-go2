# SPDX-License-Identifier: Apache-2.0
import json
import unittest

from go2_sport_actions.intent_router import CAPABILITY, FEEDBACK_PREFIX, decide


class SportRouterTest(unittest.TestCase):
    def test_one_action_leaf(self):
        result = decide([{"role": "user", "content": "请跳舞"}])
        leaf = result["rtdl"]["children"][0]
        self.assertEqual(leaf["cap"], CAPABILITY)
        self.assertEqual(leaf["args"]["text"], "请跳舞")

    def test_executor_feedback_does_not_reissue(self):
        feedback = {"leaf_result": {"contract_id":
                    "robonix/skill/go2_sport_actions/execute_utterance",
                    "success": True, "output": '{"accepted": false}'}}
        result = decide([{"role": "user", "content": "请跳舞"},
                         {"role": "user", "content": FEEDBACK_PREFIX +
                          json.dumps(feedback)}])
        self.assertEqual(result["rtdl"]["children"], [])
        self.assertIn("未接受", result["content"])

    def test_cancel_is_routed(self):
        result = decide([{"role": "user", "content": "停止动作"}])
        self.assertEqual(result["rtdl"]["children"][0]["cap"], CAPABILITY)

    def test_backflip_and_questions_have_no_motion_leaf(self):
        for text in ("后空翻", "倒立能做吗", "不要跳舞", "去地图上的办公室"):
            with self.subTest(text=text):
                self.assertEqual(decide([{"role": "user", "content": text}])["rtdl"]["children"], [])


if __name__ == "__main__":
    unittest.main()
