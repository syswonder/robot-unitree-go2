"""Official MiniCPM-RobotTrack D435i model-input geometry."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from .core import MODEL_CROP_SIZE


def prepare_center_crop_height(
    image: Any,
    *,
    crop_size: int = MODEL_CROP_SIZE,
) -> tuple[Any, tuple[int, int, int, int]]:
    """Resize to ``crop_size`` high, then center-crop a square.

    This mirrors OpenBMB's Go2 client preprocessing. Color order is irrelevant
    to the geometry, so the ROS bridge can apply it directly to a BGR frame
    before JPEG encoding.
    """

    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("expected an HxWx3 image")
    source_height, source_width = image.shape[:2]
    size = int(crop_size)
    if source_height <= 0 or source_width <= 0:
        raise ValueError("image dimensions must be positive")
    if size <= 0:
        raise ValueError("crop size must be positive")

    target_height = size
    target_width = max(
        1,
        int(round(source_width * (target_height / float(source_height)))),
    )
    if target_width < size:
        target_width = size
        target_height = max(
            1,
            int(round(source_height * (target_width / float(source_width)))),
        )
    resized = cv2.resize(
        image,
        (target_width, target_height),
        interpolation=cv2.INTER_AREA,
    )
    if target_width < size or target_height < size:
        raise ValueError(
            f"resized frame {target_width}x{target_height} is smaller than "
            f"crop {size}x{size}"
        )
    x_offset = (target_width - size) // 2
    y_offset = (target_height - size) // 2
    crop = np.ascontiguousarray(
        resized[
            y_offset : y_offset + size,
            x_offset : x_offset + size,
        ]
    )
    return crop, (target_width, target_height, x_offset, y_offset)
