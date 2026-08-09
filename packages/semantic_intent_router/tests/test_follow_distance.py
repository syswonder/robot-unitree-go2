from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "packages" / "semantic_intent_router"))

from semantic_intent_router.follow_distance import (  # noqa: E402
    DEFAULT_DISTANCE_STEP_M,
    FollowDistanceIntent,
    parse_follow_distance_intent,
)


class FollowDistanceIntentTests(unittest.TestCase):
    def test_relative_farther_is_one_positive_step(self) -> None:
        expected = FollowDistanceIntent("adjust", DEFAULT_DISTANCE_STEP_M)
        for utterance in ("离我远一点", "请离我远点", "再离我远一点儿"):
            with self.subTest(utterance=utterance):
                self.assertEqual(parse_follow_distance_intent(utterance), expected)

    def test_relative_closer_is_one_negative_step(self) -> None:
        expected = FollowDistanceIntent("adjust", -DEFAULT_DISTANCE_STEP_M)
        for utterance in ("靠近一点", "靠近我一点", "请向我靠近点", "再靠我近一点儿"):
            with self.subTest(utterance=utterance):
                self.assertEqual(parse_follow_distance_intent(utterance), expected)

    def test_absolute_arabic_distance(self) -> None:
        expected = FollowDistanceIntent("set", 5.0)
        for utterance in (
            "保持5米",
            "设为5米",
            "跟随距离设为5米",
            "请把距离调整到 5 米。",
            "跟我保持5米距离",
        ):
            with self.subTest(utterance=utterance):
                self.assertEqual(parse_follow_distance_intent(utterance), expected)

    def test_absolute_chinese_and_decimal_distance(self) -> None:
        cases = {
            "保持五米": 5.0,
            "设置为六点五米": 6.5,
            "距离调到十二米": 12.0,
            "离我保持两米的距离": 2.0,
            "保持0.5米": 0.5,
        }
        for utterance, meters in cases.items():
            with self.subTest(utterance=utterance):
                self.assertEqual(
                    parse_follow_distance_intent(utterance),
                    FollowDistanceIntent("set", meters),
                )

    def test_repeating_relative_command_keeps_one_step_semantics(self) -> None:
        first = parse_follow_distance_intent("离我远一点")
        second = parse_follow_distance_intent("离我远一点")
        self.assertEqual(first, FollowDistanceIntent("adjust", 1.0))
        self.assertEqual(second, FollowDistanceIntent("adjust", 1.0))

    def test_nonpositive_or_malformed_distance_is_rejected(self) -> None:
        for utterance in ("保持0米", "设为-1米", "保持五点米", "保持无限米"):
            with self.subTest(utterance=utterance):
                self.assertIsNone(parse_follow_distance_intent(utterance))

    def test_navigation_and_unrelated_text_do_not_match(self) -> None:
        for utterance in (
            "导航到5米外的自动售货机",
            "走到前面自动售货机那里",
            "离自动售货机远一点",
            "靠近实验室大门",
            "保持经典步态",
            "停止导航",
        ):
            with self.subTest(utterance=utterance):
                self.assertIsNone(parse_follow_distance_intent(utterance))


if __name__ == "__main__":
    unittest.main()
