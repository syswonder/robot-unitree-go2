"""ROS-independent RGB-D distance estimation for a single followed person.

The estimator deliberately does not publish commands or alter RobotTrack's
normal RGB-only path.  It turns an aligned BGR/depth frame pair into a typed
distance observation which a higher-level controller may choose to consume.
The bundled HOG detector normalizes input width to 384..512 pixels and maps
confirmed or short-term tracked person boxes back to aligned RGB-D pixels.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Protocol, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class BoundingBox:
    """Pixel-space ``x, y, width, height`` rectangle."""

    x: int
    y: int
    width: int
    height: int

    @property
    def area(self) -> int:
        return max(0, self.width) * max(0, self.height)

    @property
    def center(self) -> tuple[float, float]:
        return (self.x + self.width * 0.5, self.y + self.height * 0.5)

    def clipped(self, image_width: int, image_height: int) -> "BoundingBox | None":
        x0 = max(0, min(image_width, int(self.x)))
        y0 = max(0, min(image_height, int(self.y)))
        x1 = max(0, min(image_width, int(self.x + self.width)))
        y1 = max(0, min(image_height, int(self.y + self.height)))
        if x1 <= x0 or y1 <= y0:
            return None
        return BoundingBox(x0, y0, x1 - x0, y1 - y0)


@dataclass(frozen=True)
class PersonDetection:
    """One person candidate and the detector's uncalibrated confidence."""

    bbox: BoundingBox
    confidence: float


class PersonDetector(Protocol):
    def detect(self, bgr: np.ndarray) -> Sequence[PersonDetection]:
        """Return person candidates in the input image coordinate system."""


def bbox_iou(left: BoundingBox, right: BoundingBox) -> float:
    """Return intersection over union for two pixel rectangles."""

    x0 = max(left.x, right.x)
    y0 = max(left.y, right.y)
    x1 = min(left.x + left.width, right.x + right.width)
    y1 = min(left.y + left.height, right.y + right.height)
    intersection = max(0, x1 - x0) * max(0, y1 - y0)
    union = left.area + right.area - intersection
    return float(intersection) / float(union) if union > 0 else 0.0


def _non_maximum_suppression(
    detections: Sequence[PersonDetection],
    iou_threshold: float,
) -> tuple[PersonDetection, ...]:
    ordered = sorted(detections, key=lambda item: item.confidence, reverse=True)
    kept: list[PersonDetection] = []
    for candidate in ordered:
        if all(bbox_iou(candidate.bbox, item.bbox) <= iou_threshold for item in kept):
            kept.append(candidate)
    return tuple(kept)


class OpenCvHogPersonDetector:
    """Dependency-free CPU person detector using OpenCV's bundled HOG SVM.

    The returned confidence is a normalized ranking score, not a calibrated
    probability.  Inputs narrower than ``min_width`` are enlarged so a distant
    person is less likely to fall below HOG's 64x128 window; inputs wider than
    ``max_width`` are reduced to bound CPU cost.  Every detection is mapped
    back into the original aligned RGB-D coordinate system.
    """

    def __init__(
        self,
        *,
        min_width: int = 384,
        max_width: int = 512,
        hit_threshold: float = 0.0,
        pyramid_scale: float = 1.025,
        win_stride: int = 4,
        nms_iou_threshold: float = 0.45,
    ) -> None:
        if int(min_width) != min_width or min_width < 64:
            raise ValueError("min_width must be an integer of at least 64 pixels")
        if int(max_width) != max_width or max_width < 64:
            raise ValueError("max_width must be an integer of at least 64 pixels")
        if max_width < min_width:
            raise ValueError("max_width must not be below min_width")
        if not 1.0 < pyramid_scale <= 2.0:
            raise ValueError("pyramid_scale must be in (1, 2]")
        if int(win_stride) != win_stride or win_stride < 4 or win_stride % 4:
            raise ValueError("win_stride must be a positive multiple of 4")
        if not 0.0 <= nms_iou_threshold <= 1.0:
            raise ValueError("nms_iou_threshold must be in [0, 1]")
        self._min_width = int(min_width)
        self._max_width = int(max_width)
        self._hit_threshold = float(hit_threshold)
        self._pyramid_scale = float(pyramid_scale)
        self._win_stride = int(win_stride)
        self._nms_iou_threshold = float(nms_iou_threshold)
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    @staticmethod
    def _confidence(weight: float) -> float:
        # HOG's SVM margin is unbounded.  A sigmoid retains ranking and gives
        # callers a stable 0..1 quality field without claiming calibration.
        margin = max(-20.0, min(20.0, float(weight)))
        return 1.0 / (1.0 + math.exp(-margin))

    def detect(self, bgr: np.ndarray) -> tuple[PersonDetection, ...]:
        _validate_bgr(bgr)
        height, width = bgr.shape[:2]
        target_width = min(self._max_width, max(self._min_width, width))
        resize_scale = float(target_width) / float(width)
        if resize_scale != 1.0:
            detector_bgr = cv2.resize(
                bgr,
                (target_width,
                 max(1, int(round(height * resize_scale)))),
                interpolation=(
                    cv2.INTER_LINEAR if resize_scale > 1.0 else cv2.INTER_AREA
                ),
            )
        else:
            detector_bgr = bgr

        rectangles, weights = self._hog.detectMultiScale(
            detector_bgr,
            hitThreshold=self._hit_threshold,
            winStride=(self._win_stride, self._win_stride),
            padding=(8, 8),
            scale=self._pyramid_scale,
            useMeanshiftGrouping=False,
        )
        inverse = 1.0 / resize_scale
        detections: list[PersonDetection] = []
        flat_weights = np.asarray(weights, dtype=np.float64).reshape(-1)
        for index, rectangle in enumerate(rectangles):
            x, y, box_width, box_height = (int(value) for value in rectangle)
            bbox = BoundingBox(
                int(round(x * inverse)),
                int(round(y * inverse)),
                int(round(box_width * inverse)),
                int(round(box_height * inverse)),
            ).clipped(width, height)
            if bbox is None:
                continue
            weight = float(flat_weights[index]) if index < len(flat_weights) else 0.0
            detections.append(PersonDetection(bbox, self._confidence(weight)))
        return _non_maximum_suppression(detections, self._nms_iou_threshold)


@dataclass(frozen=True)
class ShortTermTrackingConfig:
    """Strict limits for HOG-seeded optical-flow box continuation.

    Tracking can never acquire a target by itself.  A HOG-selected person box
    starts the tracker, and at most ``max_frames`` consecutive HOG misses may
    be bridged before a new HOG acquisition is required.
    """

    max_frames: int = 3
    confidence_decay: float = 0.90
    max_features: int = 80
    min_features: int = 8
    feature_quality: float = 0.01
    feature_min_distance_px: float = 4.0
    forward_backward_error_px: float = 1.0
    min_inlier_fraction: float = 0.55
    ransac_reprojection_px: float = 2.5
    max_center_step_fraction: float = 0.15
    min_step_scale: float = 0.80
    max_step_scale: float = 1.25
    max_rotation_degrees: float = 15.0
    min_seed_area_ratio: float = 0.50
    max_seed_area_ratio: float = 2.00
    min_visible_fraction: float = 0.75

    def __post_init__(self) -> None:
        for name, value in {
            "max_frames": self.max_frames,
            "max_features": self.max_features,
            "min_features": self.min_features,
        }.items():
            if int(value) != value or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        if self.min_features > self.max_features:
            raise ValueError("min_features must not exceed max_features")
        unit_intervals = {
            "confidence_decay": self.confidence_decay,
            "feature_quality": self.feature_quality,
            "min_inlier_fraction": self.min_inlier_fraction,
            "max_center_step_fraction": self.max_center_step_fraction,
            "min_visible_fraction": self.min_visible_fraction,
        }
        for name, value in unit_intervals.items():
            if not math.isfinite(value) or not 0.0 < value <= 1.0:
                raise ValueError(f"{name} must be in (0, 1]")
        finite_positive = {
            "feature_min_distance_px": self.feature_min_distance_px,
            "forward_backward_error_px": self.forward_backward_error_px,
            "ransac_reprojection_px": self.ransac_reprojection_px,
            "min_step_scale": self.min_step_scale,
            "max_step_scale": self.max_step_scale,
            "max_rotation_degrees": self.max_rotation_degrees,
            "min_seed_area_ratio": self.min_seed_area_ratio,
            "max_seed_area_ratio": self.max_seed_area_ratio,
        }
        for name, value in finite_positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.max_step_scale < self.min_step_scale:
            raise ValueError("max_step_scale must not be below min_step_scale")
        if self.max_seed_area_ratio < self.min_seed_area_ratio:
            raise ValueError(
                "max_seed_area_ratio must not be below min_seed_area_ratio"
            )


class ShortTermOpticalFlowBoxTracker:
    """Continue a confirmed person box briefly using verified optical flow.

    Each update uses forward/backward Lucas-Kanade consistency plus a RANSAC
    partial-affine fit.  Failure clears all state immediately; success is still
    bounded by ``max_frames`` so a tracker cannot silently become a detector.
    """

    def __init__(self, config: ShortTermTrackingConfig | None = None) -> None:
        self._config = config or ShortTermTrackingConfig()
        self.reset()

    def reset(self) -> None:
        self._previous_gray: np.ndarray | None = None
        self._previous_bbox: BoundingBox | None = None
        self._seed_area = 0
        self._seed_confidence = 0.0
        self._tracked_frames = 0

    def start(
        self,
        bgr: np.ndarray,
        bbox: BoundingBox,
        confidence: float,
    ) -> None:
        _validate_bgr(bgr)
        clipped = bbox.clipped(bgr.shape[1], bgr.shape[0])
        if clipped is None:
            self.reset()
            return
        self._previous_gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        self._previous_bbox = clipped
        self._seed_area = clipped.area
        self._seed_confidence = max(0.0, min(1.0, float(confidence)))
        self._tracked_frames = 0

    def _features(self) -> np.ndarray | None:
        if self._previous_gray is None or self._previous_bbox is None:
            return None
        bbox = self._previous_bbox
        inset_x = max(1, int(round(bbox.width * 0.12)))
        inset_y = max(1, int(round(bbox.height * 0.08)))
        x0 = bbox.x + inset_x
        y0 = bbox.y + inset_y
        x1 = bbox.x + bbox.width - inset_x
        y1 = bbox.y + bbox.height - inset_y
        if x1 <= x0 or y1 <= y0:
            return None
        mask = np.zeros(self._previous_gray.shape, dtype=np.uint8)
        mask[y0:y1, x0:x1] = 255
        return cv2.goodFeaturesToTrack(
            self._previous_gray,
            maxCorners=self._config.max_features,
            qualityLevel=self._config.feature_quality,
            minDistance=self._config.feature_min_distance_px,
            mask=mask,
            blockSize=5,
        )

    @staticmethod
    def _transformed_bbox(
        bbox: BoundingBox,
        transform: np.ndarray,
        image_width: int,
        image_height: int,
    ) -> tuple[BoundingBox | None, float]:
        corners = np.float32(
            [[
                [bbox.x, bbox.y],
                [bbox.x + bbox.width, bbox.y],
                [bbox.x + bbox.width, bbox.y + bbox.height],
                [bbox.x, bbox.y + bbox.height],
            ]]
        )
        transformed = cv2.transform(corners, transform)[0]
        if not np.all(np.isfinite(transformed)):
            return None, 0.0
        minimum = np.min(transformed, axis=0)
        maximum = np.max(transformed, axis=0)
        raw = BoundingBox(
            int(round(float(minimum[0]))),
            int(round(float(minimum[1]))),
            int(round(float(maximum[0] - minimum[0]))),
            int(round(float(maximum[1] - minimum[1]))),
        )
        clipped = raw.clipped(image_width, image_height)
        if clipped is None or raw.area <= 0:
            return None, 0.0
        return clipped, float(clipped.area) / float(raw.area)

    def update(self, bgr: np.ndarray) -> PersonDetection | None:
        _validate_bgr(bgr)
        if (
            self._previous_gray is None
            or self._previous_bbox is None
            or self._tracked_frames >= self._config.max_frames
        ):
            self.reset()
            return None
        previous_points = self._features()
        if previous_points is None or len(previous_points) < self._config.min_features:
            self.reset()
            return None

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        next_points, forward_status, _ = cv2.calcOpticalFlowPyrLK(
            self._previous_gray,
            gray,
            previous_points,
            None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                30,
                0.01,
            ),
        )
        if next_points is None or forward_status is None:
            self.reset()
            return None
        previous_flat = previous_points.reshape(-1, 2)
        next_flat = next_points.reshape(-1, 2)
        forward_usable = (
            (forward_status.reshape(-1) > 0)
            & np.all(np.isfinite(next_flat), axis=1)
        )
        forward_source = previous_flat[forward_usable]
        forward_destination = next_flat[forward_usable]
        if len(forward_source) < self._config.min_features:
            self.reset()
            return None
        backward_points, backward_status, _ = cv2.calcOpticalFlowPyrLK(
            gray,
            self._previous_gray,
            forward_destination.reshape(-1, 1, 2),
            None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                30,
                0.01,
            ),
        )
        if backward_points is None or backward_status is None:
            self.reset()
            return None

        backward_flat = backward_points.reshape(-1, 2)
        forward_backward_error = np.linalg.norm(
            forward_source - backward_flat,
            axis=1,
        )
        usable = (
            (backward_status.reshape(-1) > 0)
            & np.all(np.isfinite(backward_flat), axis=1)
            & np.isfinite(forward_backward_error)
            & (
                forward_backward_error
                <= self._config.forward_backward_error_px
            )
        )
        source_points = forward_source[usable]
        destination_points = forward_destination[usable]
        if len(source_points) < self._config.min_features:
            self.reset()
            return None

        transform, inlier_mask = cv2.estimateAffinePartial2D(
            source_points,
            destination_points,
            method=cv2.RANSAC,
            ransacReprojThreshold=self._config.ransac_reprojection_px,
            maxIters=200,
            confidence=0.99,
            refineIters=10,
        )
        if transform is None or inlier_mask is None:
            self.reset()
            return None
        inlier_fraction = float(np.mean(inlier_mask.reshape(-1) > 0))
        if (
            int(np.sum(inlier_mask)) < self._config.min_features
            or inlier_fraction < self._config.min_inlier_fraction
        ):
            self.reset()
            return None

        step_scale = math.hypot(float(transform[0, 0]), float(transform[1, 0]))
        rotation_degrees = abs(
            math.degrees(math.atan2(float(transform[1, 0]), float(transform[0, 0])))
        )
        if (
            not self._config.min_step_scale
            <= step_scale
            <= self._config.max_step_scale
            or rotation_degrees > self._config.max_rotation_degrees
        ):
            self.reset()
            return None

        previous_bbox = self._previous_bbox
        bbox, visible_fraction = self._transformed_bbox(
            previous_bbox,
            transform,
            bgr.shape[1],
            bgr.shape[0],
        )
        if bbox is None:
            self.reset()
            return None
        diagonal = math.hypot(bgr.shape[1], bgr.shape[0])
        center_step_fraction = math.hypot(
            bbox.center[0] - previous_bbox.center[0],
            bbox.center[1] - previous_bbox.center[1],
        ) / diagonal
        seed_area_ratio = float(bbox.area) / float(max(1, self._seed_area))
        if (
            visible_fraction < self._config.min_visible_fraction
            or center_step_fraction > self._config.max_center_step_fraction
            or not self._config.min_seed_area_ratio
            <= seed_area_ratio
            <= self._config.max_seed_area_ratio
            or bbox.width < 24
            or bbox.height < 48
        ):
            self.reset()
            return None

        self._tracked_frames += 1
        self._previous_gray = gray
        self._previous_bbox = bbox
        confidence = self._seed_confidence * (
            self._config.confidence_decay ** self._tracked_frames
        )
        return PersonDetection(bbox, confidence)


@dataclass(frozen=True)
class RgbdDistanceConfig:
    """Tunable estimator limits; all dimensions are metres or image fractions."""

    depth_scale_m: float = 0.001
    min_depth_m: float = 0.30
    max_depth_m: float = 8.0
    min_valid_fraction: float = 0.20
    min_valid_samples: int = 64
    depth_horizontal_inset: float = 0.25
    depth_vertical_inset: float = 0.20
    trim_percentile: float = 10.0
    max_depth_mad_m: float = 0.75
    max_frame_age_s: float = 0.60
    future_timestamp_tolerance_s: float = 0.05
    association_min_iou: float = 0.10
    association_max_center_fraction: float = 0.20
    association_memory_frames: int = 3
    detection_confirmation_frames: int = 2
    enable_short_term_tracking: bool = True
    tracking_max_frames: int = 3
    enable_center_fallback: bool = False
    center_fallback_width_fraction: float = 0.35
    center_fallback_height_fraction: float = 0.75
    center_fallback_confidence: float = 0.20
    max_distance_jump_m: float = 1.25
    jump_confirmation_frames: int = 2
    jump_consistency_m: float = 0.30
    smoothing_window: int = 3

    def __post_init__(self) -> None:
        finite_positive = {
            "depth_scale_m": self.depth_scale_m,
            "min_depth_m": self.min_depth_m,
            "max_depth_m": self.max_depth_m,
            "max_depth_mad_m": self.max_depth_mad_m,
            "max_frame_age_s": self.max_frame_age_s,
            "max_distance_jump_m": self.max_distance_jump_m,
            "jump_consistency_m": self.jump_consistency_m,
        }
        for name, value in finite_positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.max_depth_m <= self.min_depth_m:
            raise ValueError("max_depth_m must exceed min_depth_m")
        unit_intervals = {
            "min_valid_fraction": self.min_valid_fraction,
            "association_min_iou": self.association_min_iou,
            "association_max_center_fraction": self.association_max_center_fraction,
            "center_fallback_width_fraction": self.center_fallback_width_fraction,
            "center_fallback_height_fraction": self.center_fallback_height_fraction,
            "center_fallback_confidence": self.center_fallback_confidence,
        }
        for name, value in unit_intervals.items():
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        for name, value in {
            "depth_horizontal_inset": self.depth_horizontal_inset,
            "depth_vertical_inset": self.depth_vertical_inset,
        }.items():
            if not math.isfinite(value) or not 0.0 <= value < 0.5:
                raise ValueError(f"{name} must be in [0, 0.5)")
        if not 0.0 <= self.trim_percentile < 50.0:
            raise ValueError("trim_percentile must be in [0, 50)")
        if self.future_timestamp_tolerance_s < 0.0:
            raise ValueError("future_timestamp_tolerance_s must be non-negative")
        for name, value in {
            "min_valid_samples": self.min_valid_samples,
            "association_memory_frames": self.association_memory_frames,
            "detection_confirmation_frames": self.detection_confirmation_frames,
            "tracking_max_frames": self.tracking_max_frames,
            "jump_confirmation_frames": self.jump_confirmation_frames,
            "smoothing_window": self.smoothing_window,
        }.items():
            if int(value) != value or value < 1:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class DistanceEstimate:
    """One explicit valid or rejected RGB-D target observation."""

    valid: bool
    status: str
    source: str
    confidence: float
    distance_m: float | None
    raw_distance_m: float | None
    bbox: BoundingBox | None
    depth_roi: BoundingBox | None
    valid_fraction: float
    valid_samples: int
    depth_mad_m: float | None
    frame_timestamp_s: float
    frame_age_s: float


def _validate_bgr(bgr: np.ndarray) -> None:
    if not isinstance(bgr, np.ndarray) or bgr.dtype != np.uint8:
        raise ValueError("bgr must be a uint8 numpy array")
    if bgr.ndim != 3 or bgr.shape[2] != 3 or min(bgr.shape[:2]) < 1:
        raise ValueError("bgr must have non-empty HxWx3 shape")


def _validate_depth(depth: np.ndarray, shape: tuple[int, int]) -> None:
    if not isinstance(depth, np.ndarray) or depth.dtype != np.uint16:
        raise ValueError("aligned_depth must be a uint16 numpy array")
    if depth.ndim != 2 or depth.shape != shape:
        raise ValueError("aligned_depth must match the BGR height and width")


class RgbdPersonDistanceEstimator:
    """Stateful, single-target RGB-D distance estimator.

    A previous HOG box is used for data association and may seed a strictly
    bounded optical-flow continuation across short HOG misses.  Tracking never
    acquires a target.  If association is lost, the optional center fallback is
    labeled with low confidence rather than being presented as a detected
    person.  Sudden depth changes require a second consistent frame before
    becoming the new accepted target distance.
    """

    def __init__(
        self,
        detector: PersonDetector | None = None,
        config: RgbdDistanceConfig | None = None,
    ) -> None:
        self._config = config or RgbdDistanceConfig()
        self._detector = detector or OpenCvHogPersonDetector()
        self._tracker = ShortTermOpticalFlowBoxTracker(
            ShortTermTrackingConfig(max_frames=self._config.tracking_max_frames)
        )
        self._previous_bbox: BoundingBox | None = None
        self._pending_detection_bbox: BoundingBox | None = None
        self._pending_detection_count = 0
        self._missed_detections = 0
        self._last_frame_timestamp_s: float | None = None
        self._last_accepted_raw_m: float | None = None
        self._distance_history: deque[float] = deque(
            maxlen=self._config.smoothing_window
        )
        self._pending_jump_m: float | None = None
        self._pending_jump_count = 0

    def reset(self) -> None:
        self._previous_bbox = None
        self._pending_detection_bbox = None
        self._pending_detection_count = 0
        self._tracker.reset()
        self._missed_detections = 0
        self._last_frame_timestamp_s = None
        self._last_accepted_raw_m = None
        self._distance_history.clear()
        self._pending_jump_m = None
        self._pending_jump_count = 0

    def _result(
        self,
        *,
        valid: bool,
        status: str,
        source: str,
        confidence: float,
        frame_timestamp_s: float,
        frame_age_s: float,
        distance_m: float | None = None,
        raw_distance_m: float | None = None,
        bbox: BoundingBox | None = None,
        depth_roi: BoundingBox | None = None,
        valid_fraction: float = 0.0,
        valid_samples: int = 0,
        depth_mad_m: float | None = None,
    ) -> DistanceEstimate:
        return DistanceEstimate(
            valid=valid,
            status=status,
            source=source,
            confidence=max(0.0, min(1.0, float(confidence))),
            distance_m=distance_m,
            raw_distance_m=raw_distance_m,
            bbox=bbox,
            depth_roi=depth_roi,
            valid_fraction=valid_fraction,
            valid_samples=valid_samples,
            depth_mad_m=depth_mad_m,
            frame_timestamp_s=frame_timestamp_s,
            frame_age_s=frame_age_s,
        )

    def _select_target(
        self,
        bgr: np.ndarray,
        detections: Sequence[PersonDetection],
        image_width: int,
        image_height: int,
    ) -> tuple[BoundingBox | None, float, str]:
        usable: list[PersonDetection] = []
        for detection in detections:
            confidence = float(detection.confidence)
            bbox = detection.bbox.clipped(image_width, image_height)
            if bbox is None or not math.isfinite(confidence):
                continue
            usable.append(
                PersonDetection(bbox, max(0.0, min(1.0, confidence)))
            )

        selected: PersonDetection | None = None
        source = "none"

        def associated_with(
            reference: BoundingBox,
        ) -> list[tuple[float, PersonDetection]]:
            diagonal = math.hypot(image_width, image_height)
            associated: list[tuple[float, PersonDetection]] = []
            for candidate in usable:
                overlap = bbox_iou(reference, candidate.bbox)
                old_center = reference.center
                new_center = candidate.bbox.center
                center_fraction = math.hypot(
                    new_center[0] - old_center[0],
                    new_center[1] - old_center[1],
                ) / diagonal
                if (
                    overlap >= self._config.association_min_iou
                    or center_fraction <= self._config.association_max_center_fraction
                ):
                    rank = 2.0 * overlap - center_fraction + 0.25 * candidate.confidence
                    associated.append((rank, candidate))
            return associated

        if usable and self._previous_bbox is None:
            candidate = max(usable, key=lambda item: item.confidence)
            if self._config.detection_confirmation_frames == 1:
                selected = candidate
            elif self._pending_detection_bbox is None:
                self._pending_detection_bbox = candidate.bbox
                self._pending_detection_count = 1
            else:
                pending_matches = associated_with(self._pending_detection_bbox)
                if pending_matches:
                    candidate = max(pending_matches, key=lambda item: item[0])[1]
                    self._pending_detection_bbox = candidate.bbox
                    self._pending_detection_count += 1
                    if (
                        self._pending_detection_count
                        >= self._config.detection_confirmation_frames
                    ):
                        selected = candidate
                else:
                    self._pending_detection_bbox = candidate.bbox
                    self._pending_detection_count = 1
            if selected is not None:
                source = "hog"
        elif not usable and self._previous_bbox is None:
            self._pending_detection_bbox = None
            self._pending_detection_count = 0
        elif usable and self._previous_bbox is not None:
            associated = associated_with(self._previous_bbox)
            if associated:
                selected = max(associated, key=lambda item: item[0])[1]
                source = "hog-associated"

        if selected is not None:
            self._previous_bbox = selected.bbox
            self._pending_detection_bbox = None
            self._pending_detection_count = 0
            self._missed_detections = 0
            if self._config.enable_short_term_tracking:
                self._tracker.start(bgr, selected.bbox, selected.confidence)
            return selected.bbox, selected.confidence, source

        if self._config.enable_short_term_tracking:
            tracked = self._tracker.update(bgr)
            if tracked is not None:
                self._previous_bbox = tracked.bbox
                self._missed_detections += 1
                return tracked.bbox, tracked.confidence, "short-track"

        self._missed_detections += 1
        self._tracker.reset()
        if self._missed_detections >= self._config.association_memory_frames:
            self._previous_bbox = None
        if not self._config.enable_center_fallback:
            return None, 0.0, "none"

        width = max(1, int(round(image_width * self._config.center_fallback_width_fraction)))
        height = max(1, int(round(image_height * self._config.center_fallback_height_fraction)))
        bbox = BoundingBox(
            (image_width - width) // 2,
            (image_height - height) // 2,
            width,
            height,
        )
        return bbox, self._config.center_fallback_confidence, "center-fallback"

    def _depth_roi(self, bbox: BoundingBox) -> BoundingBox:
        inset_x = int(round(bbox.width * self._config.depth_horizontal_inset))
        inset_y = int(round(bbox.height * self._config.depth_vertical_inset))
        return BoundingBox(
            bbox.x + inset_x,
            bbox.y + inset_y,
            max(1, bbox.width - 2 * inset_x),
            max(1, bbox.height - 2 * inset_y),
        )

    def estimate(
        self,
        bgr: np.ndarray,
        aligned_depth: np.ndarray,
        *,
        frame_timestamp_s: float | None = None,
        now_s: float | None = None,
    ) -> DistanceEstimate:
        """Estimate target distance from one aligned BGR/depth frame pair.

        ``frame_timestamp_s`` and ``now_s`` must use the same clock.  Omitting
        both uses the local monotonic clock, which is suitable for an in-process
        ROS callback after converting the source stamp to receive age.
        """

        _validate_bgr(bgr)
        _validate_depth(aligned_depth, bgr.shape[:2])
        now = time.monotonic() if now_s is None else float(now_s)
        timestamp = now if frame_timestamp_s is None else float(frame_timestamp_s)
        if not math.isfinite(now) or not math.isfinite(timestamp):
            raise ValueError("frame timestamps must be finite")
        age = now - timestamp
        if age < -self._config.future_timestamp_tolerance_s:
            return self._result(
                valid=False,
                status="future_frame",
                source="none",
                confidence=0.0,
                frame_timestamp_s=timestamp,
                frame_age_s=age,
            )
        if age > self._config.max_frame_age_s:
            return self._result(
                valid=False,
                status="stale_frame",
                source="none",
                confidence=0.0,
                frame_timestamp_s=timestamp,
                frame_age_s=age,
            )
        if (
            self._last_frame_timestamp_s is not None
            and timestamp <= self._last_frame_timestamp_s
        ):
            return self._result(
                valid=False,
                status="out_of_order_frame",
                source="none",
                confidence=0.0,
                frame_timestamp_s=timestamp,
                frame_age_s=age,
            )
        self._last_frame_timestamp_s = timestamp

        height, width = bgr.shape[:2]
        bbox, detector_confidence, source = self._select_target(
            bgr, self._detector.detect(bgr), width, height
        )
        if bbox is None:
            return self._result(
                valid=False,
                status="target_not_detected",
                source=source,
                confidence=0.0,
                frame_timestamp_s=timestamp,
                frame_age_s=age,
            )

        depth_roi = self._depth_roi(bbox).clipped(width, height)
        if depth_roi is None:
            return self._result(
                valid=False,
                status="empty_depth_roi",
                source=source,
                confidence=0.0,
                bbox=bbox,
                frame_timestamp_s=timestamp,
                frame_age_s=age,
            )
        raw_depth = aligned_depth[
            depth_roi.y:depth_roi.y + depth_roi.height,
            depth_roi.x:depth_roi.x + depth_roi.width,
        ]
        distance_values = raw_depth.astype(np.float32) * self._config.depth_scale_m
        valid_mask = (
            (raw_depth > 0)
            & np.isfinite(distance_values)
            & (distance_values >= self._config.min_depth_m)
            & (distance_values <= self._config.max_depth_m)
        )
        valid_values = distance_values[valid_mask]
        valid_samples = int(valid_values.size)
        valid_fraction = float(valid_samples) / float(raw_depth.size)
        depth_quality = min(1.0, valid_fraction / max(self._config.min_valid_fraction, 1e-9))
        confidence = detector_confidence * (0.5 + 0.5 * depth_quality)
        if (
            valid_samples < self._config.min_valid_samples
            or valid_fraction < self._config.min_valid_fraction
        ):
            return self._result(
                valid=False,
                status="insufficient_depth",
                source=source,
                confidence=confidence,
                bbox=bbox,
                depth_roi=depth_roi,
                valid_fraction=valid_fraction,
                valid_samples=valid_samples,
                frame_timestamp_s=timestamp,
                frame_age_s=age,
            )

        lower, upper = np.percentile(
            valid_values,
            [self._config.trim_percentile, 100.0 - self._config.trim_percentile],
        )
        trimmed = valid_values[(valid_values >= lower) & (valid_values <= upper)]
        raw_distance = float(np.median(trimmed))
        depth_mad = float(np.median(np.abs(trimmed - raw_distance)))
        if depth_mad > self._config.max_depth_mad_m:
            return self._result(
                valid=False,
                status="dispersed_depth",
                source=source,
                confidence=confidence * 0.5,
                raw_distance_m=raw_distance,
                bbox=bbox,
                depth_roi=depth_roi,
                valid_fraction=valid_fraction,
                valid_samples=valid_samples,
                depth_mad_m=depth_mad,
                frame_timestamp_s=timestamp,
                frame_age_s=age,
            )

        jump_confirmed = False
        if (
            self._last_accepted_raw_m is not None
            and abs(raw_distance - self._last_accepted_raw_m)
            > self._config.max_distance_jump_m
        ):
            if (
                self._pending_jump_m is not None
                and abs(raw_distance - self._pending_jump_m)
                <= self._config.jump_consistency_m
            ):
                self._pending_jump_count += 1
                self._pending_jump_m = (
                    self._pending_jump_m * (self._pending_jump_count - 1) + raw_distance
                ) / self._pending_jump_count
            else:
                self._pending_jump_m = raw_distance
                self._pending_jump_count = 1
            if self._pending_jump_count < self._config.jump_confirmation_frames:
                return self._result(
                    valid=False,
                    status="distance_jump",
                    source=source,
                    confidence=confidence * 0.5,
                    raw_distance_m=raw_distance,
                    bbox=bbox,
                    depth_roi=depth_roi,
                    valid_fraction=valid_fraction,
                    valid_samples=valid_samples,
                    depth_mad_m=depth_mad,
                    frame_timestamp_s=timestamp,
                    frame_age_s=age,
                )
            raw_distance = float(self._pending_jump_m)
            self._distance_history.clear()
            jump_confirmed = True

        self._pending_jump_m = None
        self._pending_jump_count = 0
        self._last_accepted_raw_m = raw_distance
        self._distance_history.append(raw_distance)
        smoothed_distance = float(np.median(tuple(self._distance_history)))
        return self._result(
            valid=True,
            status="jump_confirmed" if jump_confirmed else "ok",
            source=source,
            confidence=confidence,
            distance_m=smoothed_distance,
            raw_distance_m=raw_distance,
            bbox=bbox,
            depth_roi=depth_roi,
            valid_fraction=valid_fraction,
            valid_samples=valid_samples,
            depth_mad_m=depth_mad,
            frame_timestamp_s=timestamp,
            frame_age_s=age,
        )
