from __future__ import annotations

import unittest

import numpy as np

from go2_robottrack.image_preprocess import prepare_center_crop_height


class OfficialImagePreprocessTests(unittest.TestCase):
    def test_d435i_four_by_three_frame_becomes_centered_384_square(self) -> None:
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        crop, geometry = prepare_center_crop_height(image)
        self.assertEqual(geometry, (512, 384, 64, 0))
        self.assertEqual(crop.shape, (384, 384, 3))
        self.assertTrue(crop.flags.c_contiguous)

    def test_portrait_frame_scales_width_first_then_crops_height(self) -> None:
        image = np.zeros((480, 320, 3), dtype=np.uint8)
        crop, geometry = prepare_center_crop_height(image)
        self.assertEqual(geometry, (384, 576, 0, 96))
        self.assertEqual(crop.shape, (384, 384, 3))

    def test_invalid_frame_or_crop_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "HxWx3"):
            prepare_center_crop_height(np.zeros((480, 640), dtype=np.uint8))
        with self.assertRaisesRegex(ValueError, "positive"):
            prepare_center_crop_height(
                np.zeros((10, 10, 3), dtype=np.uint8),
                crop_size=0,
            )


if __name__ == "__main__":
    unittest.main()
