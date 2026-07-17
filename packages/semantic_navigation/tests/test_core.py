from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from semantic_navigation.core import LandmarkError, LandmarkStore, normalize_text


def document(
    *, verified: bool = True, map_id: str = "lab_go2", map_generation: int = 7
) -> dict:
    return {
        "schema_version": 2,
        "map_id": map_id,
        "map_generation": map_generation,
        "frame_id": "map",
        "landmarks": [
            {
                "id": "vending_machine_front",
                "name": "自动售货机",
                "aliases": ["售货机", "自动贩卖机"],
                "verified": verified,
                "pose": {"x": 1.25, "y": -0.75, "yaw": 7.0},
            }
        ],
    }


class LandmarkStoreTest(unittest.TestCase):
    def test_required_acceptance_phrase_resolves(self) -> None:
        store = LandmarkStore.from_mapping(document())
        landmark = store.resolve(
            "走到前面自动售货机那里",
            expected_map_id="lab_go2",
            expected_generation=7,
        )
        self.assertEqual(landmark.id, "vending_machine_front")
        self.assertEqual(landmark.name, "自动售货机")
        self.assertTrue(-math.pi <= landmark.yaw <= math.pi)

    def test_asr_spacing_and_punctuation_resolve(self) -> None:
        store = LandmarkStore.from_mapping(document())
        landmark = store.resolve(
            "请走到，自动 售货机！那里。",
            expected_map_id="lab_go2",
            expected_generation=7,
        )
        self.assertEqual(landmark.name, "自动售货机")

    def test_unverified_pose_fails_closed(self) -> None:
        store = LandmarkStore.from_mapping(document(verified=False))
        with self.assertRaisesRegex(LandmarkError, "verified"):
            store.resolve(
                "自动售货机", expected_map_id="lab_go2", expected_generation=7
            )

    def test_quoted_verified_value_is_rejected(self) -> None:
        raw = document()
        raw["landmarks"][0]["verified"] = "false"
        with self.assertRaisesRegex(LandmarkError, "YAML boolean"):
            LandmarkStore.from_mapping(raw)

    def test_wrong_active_map_fails_closed(self) -> None:
        store = LandmarkStore.from_mapping(document(map_id="floor_1"))
        with self.assertRaisesRegex(LandmarkError, "does not match"):
            store.resolve(
                "自动售货机", expected_map_id="floor_2", expected_generation=7
            )

    def test_wrong_map_generation_fails_closed(self) -> None:
        store = LandmarkStore.from_mapping(document(map_generation=12))
        with self.assertRaisesRegex(LandmarkError, "generation"):
            store.resolve(
                "自动售货机", expected_map_id="lab_go2", expected_generation=13
            )

    def test_generation_must_be_real_uint64(self) -> None:
        for invalid in (True, "7", -1, float("nan")):
            raw = document()
            raw["map_generation"] = invalid
            with self.subTest(invalid=invalid):
                with self.assertRaisesRegex(LandmarkError, "uint64|within"):
                    LandmarkStore.from_mapping(raw)

    def test_unknown_name_fails_closed(self) -> None:
        store = LandmarkStore.from_mapping(document())
        with self.assertRaisesRegex(LandmarkError, "unknown"):
            store.resolve(
                "实验室大门", expected_map_id="lab_go2", expected_generation=7
            )

    def test_two_distinct_landmarks_in_one_utterance_are_rejected(self) -> None:
        raw = document()
        raw["landmarks"].append(
            {
                "id": "lab_door",
                "name": "实验室大门",
                "aliases": ["大门"],
                "verified": True,
                "pose": {"x": 0.0, "y": 1.0, "yaw": 0.0},
            }
        )
        store = LandmarkStore.from_mapping(raw)
        with self.assertRaisesRegex(LandmarkError, "ambiguous"):
            store.resolve(
                "先去实验室大门再去自动售货机",
                expected_map_id="lab_go2",
                expected_generation=7,
            )

    def test_non_finite_pose_is_rejected(self) -> None:
        raw = document()
        raw["landmarks"][0]["pose"]["x"] = float("nan")
        with self.assertRaisesRegex(LandmarkError, "finite"):
            LandmarkStore.from_mapping(raw)

    def test_normalization_does_not_translate_or_guess(self) -> None:
        self.assertEqual(normalize_text("自动 售货机！？"), "自动售货机")


if __name__ == "__main__":
    unittest.main()
