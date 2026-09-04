from __future__ import annotations

import unittest

from go2_robottrack.rgbd_sync import LatestRgbdPairer


class LatestRgbdPairerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.pairer = LatestRgbdPairer(
            max_source_delta_s=0.20,
            max_frame_age_s=0.60,
        )

    def test_rgb_without_depth_invalidates_measurement(self) -> None:
        decision = self.pairer.offer_rgb(
            "rgb-1",
            source_timestamp_s=10.0,
            received_monotonic=100.0,
        )
        self.assertIsNone(decision.pair)
        self.assertEqual(decision.status, "missing_depth")
        self.assertTrue(decision.invalidate_measurement)

    def test_depth_callback_can_complete_latest_rgb_pair(self) -> None:
        self.pairer.offer_rgb(
            "rgb-1",
            source_timestamp_s=10.0,
            received_monotonic=100.0,
        )
        decision = self.pairer.offer_depth(
            "depth-1",
            source_timestamp_s=10.1,
            received_monotonic=100.1,
        )
        self.assertIsNotNone(decision.pair)
        assert decision.pair is not None
        self.assertEqual(decision.pair.rgb.image, "rgb-1")
        self.assertEqual(decision.pair.depth.image, "depth-1")
        self.assertAlmostEqual(decision.pair.source_delta_s, 0.1)

    def test_pair_freshness_uses_older_receive_time(self) -> None:
        self.pairer.offer_rgb(
            "rgb-1",
            source_timestamp_s=10.0,
            received_monotonic=100.0,
        )
        decision = self.pairer.offer_depth(
            "depth-1",
            source_timestamp_s=10.1,
            received_monotonic=100.4,
        )
        self.assertIsNotNone(decision.pair)
        assert decision.pair is not None
        self.assertEqual(decision.pair.received_monotonic, 100.0)
        self.assertEqual(decision.pair.completed_monotonic, 100.4)

    def test_one_rgb_is_emitted_at_most_once(self) -> None:
        self.pairer.offer_depth(
            "depth-1",
            source_timestamp_s=10.0,
            received_monotonic=100.0,
        )
        first = self.pairer.offer_rgb(
            "rgb-1",
            source_timestamp_s=10.0,
            received_monotonic=100.01,
        )
        self.assertIsNotNone(first.pair)
        second = self.pairer.offer_depth(
            "depth-2",
            source_timestamp_s=10.01,
            received_monotonic=100.02,
        )
        self.assertIsNone(second.pair)
        self.assertEqual(second.status, "awaiting_new_rgb")
        self.assertFalse(second.invalidate_measurement)

    def test_source_delta_at_limit_is_accepted(self) -> None:
        self.pairer.offer_depth(
            "depth-1",
            source_timestamp_s=10.0,
            received_monotonic=100.0,
        )
        decision = self.pairer.offer_rgb(
            "rgb-1",
            source_timestamp_s=10.2,
            received_monotonic=100.1,
        )
        self.assertIsNotNone(decision.pair)

    def test_new_rgb_waits_when_latest_depth_was_already_consumed(self) -> None:
        self.pairer.offer_depth(
            "depth-1",
            source_timestamp_s=8.0,
            received_monotonic=100.0,
        )
        first = self.pairer.offer_rgb(
            "rgb-1",
            source_timestamp_s=8.0,
            received_monotonic=100.05,
        )
        self.assertIsNotNone(first.pair)

        mismatch = self.pairer.offer_rgb(
            "rgb-2",
            source_timestamp_s=10.0,
            received_monotonic=100.1,
        )
        self.assertEqual(mismatch.status, "awaiting_new_depth")
        self.assertFalse(mismatch.invalidate_measurement)

        repaired = self.pairer.offer_depth(
            "depth-2",
            source_timestamp_s=10.05,
            received_monotonic=100.2,
        )
        self.assertIsNotNone(repaired.pair)
        assert repaired.pair is not None
        self.assertEqual(repaired.pair.rgb.image, "rgb-2")
        self.assertEqual(repaired.pair.depth.image, "depth-2")

    def test_stamp_mismatch_waits_for_matching_depth_without_invalidation(self) -> None:
        self.pairer.offer_depth(
            "depth-0",
            source_timestamp_s=7.0,
            received_monotonic=99.8,
        )
        first = self.pairer.offer_rgb(
            "rgb-0",
            source_timestamp_s=7.0,
            received_monotonic=99.9,
        )
        self.assertIsNotNone(first.pair)
        self.pairer.offer_depth(
            "old-depth",
            source_timestamp_s=8.0,
            received_monotonic=100.0,
        )

        mismatch = self.pairer.offer_rgb(
            "rgb-1",
            source_timestamp_s=10.0,
            received_monotonic=100.1,
        )
        self.assertEqual(mismatch.status, "awaiting_matching_depth")
        self.assertFalse(mismatch.invalidate_measurement)
        self.assertIn("waiting for matching depth frame", mismatch.detail)

        repaired = self.pairer.offer_depth(
            "depth-1",
            source_timestamp_s=10.05,
            received_monotonic=100.2,
        )
        self.assertIsNotNone(repaired.pair)
        assert repaired.pair is not None
        self.assertEqual(repaired.pair.rgb.image, "rgb-1")
        self.assertEqual(repaired.pair.depth.image, "depth-1")

    def test_new_depth_waits_for_matching_rgb_without_invalidation(self) -> None:
        self.pairer.offer_rgb(
            "old-rgb",
            source_timestamp_s=8.0,
            received_monotonic=100.0,
        )
        mismatch = self.pairer.offer_depth(
            "depth-1",
            source_timestamp_s=10.0,
            received_monotonic=100.1,
        )
        self.assertEqual(mismatch.status, "awaiting_matching_rgb")
        self.assertFalse(mismatch.invalidate_measurement)
        self.assertIn("waiting for matching RGB frame", mismatch.detail)

        repaired = self.pairer.offer_rgb(
            "rgb-1",
            source_timestamp_s=10.05,
            received_monotonic=100.2,
        )
        self.assertIsNotNone(repaired.pair)
        assert repaired.pair is not None
        self.assertEqual(repaired.pair.rgb.image, "rgb-1")
        self.assertEqual(repaired.pair.depth.image, "depth-1")

    def test_five_hz_rgb_then_depth_pairs_each_frame_once(self) -> None:
        for index in range(4):
            source = 10.0 + index * 0.2
            received = 100.0 + index * 0.2
            rgb = self.pairer.offer_rgb(
                f"rgb-{index}",
                source_timestamp_s=source,
                received_monotonic=received,
            )
            self.assertIsNone(rgb.pair)
            if index == 0:
                self.assertEqual(rgb.status, "missing_depth")
            else:
                self.assertEqual(rgb.status, "awaiting_new_depth")
                self.assertFalse(rgb.invalidate_measurement)

            depth = self.pairer.offer_depth(
                f"depth-{index}",
                source_timestamp_s=source,
                received_monotonic=received + 0.01,
            )
            self.assertIsNotNone(depth.pair)
            assert depth.pair is not None
            self.assertEqual(depth.pair.rgb.image, f"rgb-{index}")
            self.assertEqual(depth.pair.depth.image, f"depth-{index}")
            self.assertEqual(depth.pair.source_delta_s, 0.0)

    def test_stale_latest_frame_is_not_paired(self) -> None:
        self.pairer.offer_depth(
            "depth-1",
            source_timestamp_s=10.0,
            received_monotonic=100.0,
        )
        decision = self.pairer.offer_rgb(
            "rgb-1",
            source_timestamp_s=10.0,
            received_monotonic=101.0,
        )
        self.assertIsNone(decision.pair)
        self.assertEqual(decision.status, "stale_rgbd_pair")
        self.assertTrue(decision.invalidate_measurement)

    def test_clear_removes_both_streams(self) -> None:
        self.pairer.offer_depth(
            "depth-1",
            source_timestamp_s=10.0,
            received_monotonic=100.0,
        )
        self.pairer.clear()
        decision = self.pairer.offer_rgb(
            "rgb-2",
            source_timestamp_s=20.0,
            received_monotonic=200.0,
        )
        self.assertEqual(decision.status, "missing_depth")


if __name__ == "__main__":
    unittest.main()
