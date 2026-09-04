from __future__ import annotations

import unittest

import numpy as np

from go2_robottrack.rgbd_distance import (
    BoundingBox,
    OpenCvHogPersonDetector,
    PersonDetection,
    RgbdDistanceConfig,
    RgbdPersonDistanceEstimator,
    bbox_iou,
)


class SequenceDetector:
    def __init__(self, *responses):
        self._responses = list(responses)
        self.calls = 0

    def detect(self, _bgr):
        self.calls += 1
        if not self._responses:
            return ()
        return self._responses.pop(0)


class HogStub:
    def __init__(self, rectangle, weight=1.0):
        self.rectangle = rectangle
        self.weight = weight
        self.shapes = []

    def detectMultiScale(self, bgr, **_kwargs):
        self.shapes.append(bgr.shape)
        return np.asarray([self.rectangle]), np.asarray([self.weight])


def config(**overrides):
    values = {
        "min_valid_samples": 4,
        "min_valid_fraction": 0.10,
        "smoothing_window": 1,
        "detection_confirmation_frames": 1,
    }
    values.update(overrides)
    return RgbdDistanceConfig(**values)


class RgbdPersonDistanceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.bgr = np.zeros((120, 160, 3), dtype=np.uint8)

    def test_robust_depth_uses_inner_person_roi_and_ignores_invalid_samples(self) -> None:
        detector = SequenceDetector(
            (PersonDetection(BoundingBox(40, 10, 80, 100), 0.9),)
        )
        depth = np.full((120, 160), 7000, dtype=np.uint16)
        depth[30:90, 60:100] = 2500
        depth[30:35, 60:100] = 0
        depth[40, 70] = 500
        depth[41, 70] = 7900
        estimator = RgbdPersonDistanceEstimator(detector, config())

        result = estimator.estimate(
            self.bgr, depth, frame_timestamp_s=10.0, now_s=10.1
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.status, "ok")
        self.assertEqual(result.source, "hog")
        self.assertEqual(result.depth_roi, BoundingBox(60, 30, 40, 60))
        self.assertAlmostEqual(result.distance_m, 2.5, places=3)
        self.assertGreater(result.valid_fraction, 0.9)
        self.assertAlmostEqual(result.confidence, 0.9)

    def test_previous_box_association_does_not_switch_to_higher_score_bystander(self) -> None:
        first = PersonDetection(BoundingBox(10, 10, 45, 100), 0.8)
        moved_target = PersonDetection(BoundingBox(15, 10, 45, 100), 0.6)
        bystander = PersonDetection(BoundingBox(105, 10, 45, 100), 0.99)
        detector = SequenceDetector((first,), (bystander, moved_target))
        estimator = RgbdPersonDistanceEstimator(detector, config())
        first_depth = np.full((120, 160), 2000, dtype=np.uint16)
        second_depth = np.full((120, 160), 5000, dtype=np.uint16)
        second_depth[:, :80] = 2200

        initial = estimator.estimate(
            self.bgr, first_depth, frame_timestamp_s=1.0, now_s=1.0
        )
        associated = estimator.estimate(
            self.bgr, second_depth, frame_timestamp_s=1.2, now_s=1.2
        )

        self.assertTrue(initial.valid)
        self.assertTrue(associated.valid)
        self.assertEqual(associated.source, "hog-associated")
        self.assertEqual(associated.bbox, moved_target.bbox)
        self.assertAlmostEqual(associated.raw_distance_m, 2.2, places=3)

    def test_optional_center_fallback_is_explicit_and_low_confidence(self) -> None:
        detector = SequenceDetector(())
        estimator = RgbdPersonDistanceEstimator(
            detector,
            config(
                enable_center_fallback=True,
                center_fallback_confidence=0.2,
            ),
        )
        depth = np.full((120, 160), 3000, dtype=np.uint16)

        result = estimator.estimate(
            self.bgr, depth, frame_timestamp_s=2.0, now_s=2.0
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.source, "center-fallback")
        self.assertAlmostEqual(result.confidence, 0.2)
        self.assertAlmostEqual(result.distance_m, 3.0, places=5)

    def test_center_fallback_is_disabled_by_default(self) -> None:
        estimator = RgbdPersonDistanceEstimator(SequenceDetector(()), config())
        result = estimator.estimate(
            self.bgr,
            np.full((120, 160), 3000, dtype=np.uint16),
            frame_timestamp_s=2.0,
            now_s=2.0,
        )
        self.assertFalse(result.valid)
        self.assertEqual(result.status, "target_not_detected")
        self.assertEqual(result.source, "none")

    def test_short_tracking_bridges_only_bounded_hog_misses(self) -> None:
        person = PersonDetection(BoundingBox(40, 10, 80, 100), 0.9)
        detector = SequenceDetector((person,), (), (), ())
        estimator = RgbdPersonDistanceEstimator(
            detector,
            config(tracking_max_frames=2),
        )
        random = np.random.RandomState(7)
        texture = random.randint(0, 256, (100, 80, 3), dtype=np.uint8)

        def shifted_frame(offset: int) -> np.ndarray:
            frame = np.zeros_like(self.bgr)
            frame[10:110, 40 + offset:120 + offset] = texture
            return frame

        depth = np.full((120, 160), 3000, dtype=np.uint16)
        acquired = estimator.estimate(
            shifted_frame(0), depth, frame_timestamp_s=1.0, now_s=1.0
        )
        tracked_once = estimator.estimate(
            shifted_frame(4), depth, frame_timestamp_s=1.2, now_s=1.2
        )
        tracked_twice = estimator.estimate(
            shifted_frame(8), depth, frame_timestamp_s=1.4, now_s=1.4
        )
        expired = estimator.estimate(
            shifted_frame(12), depth, frame_timestamp_s=1.6, now_s=1.6
        )

        self.assertTrue(acquired.valid)
        self.assertEqual(acquired.source, "hog")
        self.assertTrue(tracked_once.valid)
        self.assertEqual(tracked_once.source, "short-track")
        self.assertGreaterEqual(tracked_once.bbox.x, 42)
        self.assertTrue(tracked_twice.valid)
        self.assertEqual(tracked_twice.source, "short-track")
        self.assertGreaterEqual(tracked_twice.bbox.x, 46)
        self.assertFalse(expired.valid)
        self.assertEqual(expired.status, "target_not_detected")
        self.assertEqual(expired.source, "none")

    def test_short_tracking_never_acquires_without_hog(self) -> None:
        random = np.random.RandomState(11)
        textured = random.randint(0, 256, self.bgr.shape, dtype=np.uint8)
        estimator = RgbdPersonDistanceEstimator(
            SequenceDetector((), ()),
            config(tracking_max_frames=3),
        )
        depth = np.full((120, 160), 3000, dtype=np.uint16)

        first = estimator.estimate(
            textured, depth, frame_timestamp_s=2.0, now_s=2.0
        )
        second = estimator.estimate(
            np.roll(textured, 3, axis=1),
            depth,
            frame_timestamp_s=2.2,
            now_s=2.2,
        )

        self.assertFalse(first.valid)
        self.assertFalse(second.valid)
        self.assertEqual(first.source, "none")
        self.assertEqual(second.source, "none")

    def test_default_requires_two_consistent_hog_frames_before_acquisition(self) -> None:
        person = PersonDetection(BoundingBox(40, 10, 80, 100), 0.9)
        detector = SequenceDetector((person,), (person,))
        estimator = RgbdPersonDistanceEstimator(
            detector,
            RgbdDistanceConfig(
                min_valid_samples=4,
                min_valid_fraction=0.10,
                smoothing_window=1,
            ),
        )
        depth = np.full((120, 160), 3000, dtype=np.uint16)

        pending = estimator.estimate(
            self.bgr, depth, frame_timestamp_s=3.0, now_s=3.0
        )
        confirmed = estimator.estimate(
            self.bgr, depth, frame_timestamp_s=3.2, now_s=3.2
        )

        self.assertFalse(pending.valid)
        self.assertEqual(pending.status, "target_not_detected")
        self.assertTrue(confirmed.valid)
        self.assertEqual(confirmed.source, "hog")

    def test_single_false_hog_box_cannot_acquire_the_target(self) -> None:
        false_box = PersonDetection(BoundingBox(115, 10, 40, 90), 0.95)
        person_one = PersonDetection(BoundingBox(10, 10, 45, 100), 0.8)
        person_two = PersonDetection(BoundingBox(14, 10, 45, 100), 0.8)
        estimator = RgbdPersonDistanceEstimator(
            SequenceDetector((false_box,), (person_one,), (person_two,)),
            RgbdDistanceConfig(
                min_valid_samples=4,
                min_valid_fraction=0.10,
                smoothing_window=1,
            ),
        )
        depth = np.full((120, 160), 3000, dtype=np.uint16)

        false_pending = estimator.estimate(
            self.bgr, depth, frame_timestamp_s=4.0, now_s=4.0
        )
        person_pending = estimator.estimate(
            self.bgr, depth, frame_timestamp_s=4.2, now_s=4.2
        )
        person_confirmed = estimator.estimate(
            self.bgr, depth, frame_timestamp_s=4.4, now_s=4.4
        )

        self.assertFalse(false_pending.valid)
        self.assertFalse(person_pending.valid)
        self.assertTrue(person_confirmed.valid)
        self.assertEqual(person_confirmed.bbox, person_two.bbox)

    def test_hog_stride_requires_an_opencv_compatible_multiple(self) -> None:
        with self.assertRaisesRegex(ValueError, "multiple of 4"):
            OpenCvHogPersonDetector(win_stride=3)

    def test_hog_width_bounds_are_validated(self) -> None:
        with self.assertRaisesRegex(ValueError, "min_width"):
            OpenCvHogPersonDetector(min_width=63)
        with self.assertRaisesRegex(ValueError, "max_width"):
            OpenCvHogPersonDetector(max_width=512.5)
        with self.assertRaisesRegex(ValueError, "below min_width"):
            OpenCvHogPersonDetector(min_width=384, max_width=320)

    def test_hog_upscales_small_input_and_maps_box_back(self) -> None:
        detector = OpenCvHogPersonDetector(min_width=384, max_width=512)
        stub = HogStub((48, 24, 96, 192))
        detector._hog = stub

        detections = detector.detect(np.zeros((240, 320, 3), dtype=np.uint8))

        self.assertEqual(stub.shapes, [(288, 384, 3)])
        self.assertEqual(detections[0].bbox, BoundingBox(40, 20, 80, 160))

    def test_hog_keeps_mid_width_and_downscales_large_input(self) -> None:
        mid_detector = OpenCvHogPersonDetector(min_width=384, max_width=512)
        mid_stub = HogStub((32, 16, 64, 128))
        mid_detector._hog = mid_stub
        mid = mid_detector.detect(np.zeros((300, 400, 3), dtype=np.uint8))

        large_detector = OpenCvHogPersonDetector(min_width=384, max_width=512)
        large_stub = HogStub((64, 32, 128, 256))
        large_detector._hog = large_stub
        large = large_detector.detect(np.zeros((600, 800, 3), dtype=np.uint8))

        self.assertEqual(mid_stub.shapes, [(300, 400, 3)])
        self.assertEqual(mid[0].bbox, BoundingBox(32, 16, 64, 128))
        self.assertEqual(large_stub.shapes, [(384, 512, 3)])
        self.assertEqual(large[0].bbox, BoundingBox(100, 50, 200, 400))

    def test_stale_and_out_of_order_frames_are_rejected_before_detection(self) -> None:
        person = PersonDetection(BoundingBox(40, 10, 80, 100), 0.9)
        detector = SequenceDetector((person,), (person,))
        estimator = RgbdPersonDistanceEstimator(
            detector, config(max_frame_age_s=0.5)
        )
        depth = np.full((120, 160), 2000, dtype=np.uint16)

        stale = estimator.estimate(
            self.bgr, depth, frame_timestamp_s=1.0, now_s=1.6
        )
        accepted = estimator.estimate(
            self.bgr, depth, frame_timestamp_s=2.0, now_s=2.0
        )
        duplicate = estimator.estimate(
            self.bgr, depth, frame_timestamp_s=2.0, now_s=2.1
        )

        self.assertEqual(stale.status, "stale_frame")
        self.assertTrue(accepted.valid)
        self.assertEqual(duplicate.status, "out_of_order_frame")
        self.assertEqual(detector.calls, 1)

    def test_insufficient_or_dispersed_depth_is_rejected_with_metrics(self) -> None:
        person = PersonDetection(BoundingBox(40, 10, 80, 100), 0.9)
        detector = SequenceDetector((person,), (person,))
        estimator = RgbdPersonDistanceEstimator(
            detector,
            config(min_valid_fraction=0.5, max_depth_mad_m=0.5),
        )
        empty_depth = np.zeros((120, 160), dtype=np.uint16)
        dispersed_depth = np.full((120, 160), 1000, dtype=np.uint16)
        # The estimator's inner ROI is 40x60.  Equal near/far halves produce a
        # large median absolute deviation even after percentile trimming.
        dispersed_depth[60:90, 60:100] = 5000

        empty = estimator.estimate(
            self.bgr, empty_depth, frame_timestamp_s=3.0, now_s=3.0
        )
        dispersed = estimator.estimate(
            self.bgr, dispersed_depth, frame_timestamp_s=3.2, now_s=3.2
        )

        self.assertEqual(empty.status, "insufficient_depth")
        self.assertEqual(empty.valid_samples, 0)
        self.assertEqual(dispersed.status, "dispersed_depth")
        self.assertGreater(dispersed.depth_mad_m, 0.5)

    def test_large_jump_requires_a_second_consistent_frame(self) -> None:
        person = PersonDetection(BoundingBox(40, 10, 80, 100), 0.9)
        detector = SequenceDetector((person,), (person,), (person,))
        estimator = RgbdPersonDistanceEstimator(
            detector,
            config(
                max_distance_jump_m=1.0,
                jump_confirmation_frames=2,
                jump_consistency_m=0.2,
            ),
        )

        baseline = estimator.estimate(
            self.bgr,
            np.full((120, 160), 2000, dtype=np.uint16),
            frame_timestamp_s=4.0,
            now_s=4.0,
        )
        jump = estimator.estimate(
            self.bgr,
            np.full((120, 160), 5000, dtype=np.uint16),
            frame_timestamp_s=4.2,
            now_s=4.2,
        )
        confirmed = estimator.estimate(
            self.bgr,
            np.full((120, 160), 5100, dtype=np.uint16),
            frame_timestamp_s=4.4,
            now_s=4.4,
        )

        self.assertTrue(baseline.valid)
        self.assertFalse(jump.valid)
        self.assertEqual(jump.status, "distance_jump")
        self.assertAlmostEqual(jump.raw_distance_m, 5.0)
        self.assertTrue(confirmed.valid)
        self.assertEqual(confirmed.status, "jump_confirmed")
        self.assertAlmostEqual(confirmed.distance_m, 5.05, places=2)

    def test_invalid_image_and_depth_contracts_raise(self) -> None:
        estimator = RgbdPersonDistanceEstimator(SequenceDetector(()), config())
        with self.assertRaisesRegex(ValueError, "uint8"):
            estimator.estimate(
                self.bgr.astype(np.float32),
                np.zeros((120, 160), dtype=np.uint16),
            )
        with self.assertRaisesRegex(ValueError, "uint16"):
            estimator.estimate(
                self.bgr,
                np.zeros((120, 160), dtype=np.float32),
            )
        with self.assertRaisesRegex(ValueError, "match"):
            estimator.estimate(
                self.bgr,
                np.zeros((60, 80), dtype=np.uint16),
            )

    def test_bbox_iou(self) -> None:
        self.assertAlmostEqual(
            bbox_iou(BoundingBox(0, 0, 10, 10), BoundingBox(5, 0, 10, 10)),
            1.0 / 3.0,
        )
        self.assertEqual(
            bbox_iou(BoundingBox(0, 0, 1, 1), BoundingBox(2, 2, 1, 1)),
            0.0,
        )


if __name__ == "__main__":
    unittest.main()
