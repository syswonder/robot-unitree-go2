"""ROS-independent latest-frame pairing for aligned RGB-D streams."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any


def _finite_float(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


@dataclass(frozen=True)
class TimedRgbdFrame:
    """One image together with source-clock and receive-clock timestamps."""

    image: Any
    source_timestamp_s: float
    received_monotonic: float
    sequence: int


@dataclass(frozen=True)
class RgbdFramePair:
    """A latest RGB frame and the latest sufficiently close depth frame."""

    rgb: TimedRgbdFrame
    depth: TimedRgbdFrame
    source_delta_s: float

    @property
    def received_monotonic(self) -> float:
        """Conservative pair age anchor: when the older member arrived."""

        return min(self.rgb.received_monotonic, self.depth.received_monotonic)

    @property
    def completed_monotonic(self) -> float:
        """When both members of the pair had arrived."""

        return max(self.rgb.received_monotonic, self.depth.received_monotonic)


@dataclass(frozen=True)
class RgbdPairDecision:
    """Pairing result; ``invalidate_measurement`` distinguishes wait states."""

    pair: RgbdFramePair | None
    status: str
    detail: str
    invalidate_measurement: bool


class LatestRgbdPairer:
    """Manually match each latest RGB frame with one aligned depth frame.

    A given RGB or depth frame is emitted at most once.  This lets either ROS
    callback complete a pair without running person detection twice or pairing
    a new frame with the already-consumed member of the preceding pair.
    """

    def __init__(self, *, max_source_delta_s: float, max_frame_age_s: float) -> None:
        source_delta = _finite_float(max_source_delta_s, "max_source_delta_s")
        max_age = _finite_float(max_frame_age_s, "max_frame_age_s")
        if source_delta < 0.0:
            raise ValueError("max_source_delta_s must be non-negative")
        if max_age <= 0.0:
            raise ValueError("max_frame_age_s must be positive")
        self._max_source_delta_s = source_delta
        self._max_frame_age_s = max_age
        self._lock = threading.Lock()
        self._rgb: TimedRgbdFrame | None = None
        self._depth: TimedRgbdFrame | None = None
        self._rgb_sequence = 0
        self._depth_sequence = 0
        self._emitted_rgb_sequence = 0
        self._emitted_depth_sequence = 0

    def offer_rgb(
        self,
        image: Any,
        *,
        source_timestamp_s: float,
        received_monotonic: float | None = None,
    ) -> RgbdPairDecision:
        received = (
            time.monotonic()
            if received_monotonic is None
            else _finite_float(received_monotonic, "received_monotonic")
        )
        source = _finite_float(source_timestamp_s, "source_timestamp_s")
        with self._lock:
            self._rgb_sequence += 1
            self._rgb = TimedRgbdFrame(
                image=image,
                source_timestamp_s=source,
                received_monotonic=received,
                sequence=self._rgb_sequence,
            )
            return self._match_locked(now_monotonic=received)

    def offer_depth(
        self,
        image: Any,
        *,
        source_timestamp_s: float,
        received_monotonic: float | None = None,
    ) -> RgbdPairDecision:
        received = (
            time.monotonic()
            if received_monotonic is None
            else _finite_float(received_monotonic, "received_monotonic")
        )
        source = _finite_float(source_timestamp_s, "source_timestamp_s")
        with self._lock:
            self._depth_sequence += 1
            self._depth = TimedRgbdFrame(
                image=image,
                source_timestamp_s=source,
                received_monotonic=received,
                sequence=self._depth_sequence,
            )
            return self._match_locked(now_monotonic=received)

    def _match_locked(self, *, now_monotonic: float) -> RgbdPairDecision:
        rgb = self._rgb
        depth = self._depth
        if rgb is None:
            return RgbdPairDecision(
                None,
                "missing_rgb",
                "no RGB frame is available for the latest depth frame",
                True,
            )
        if rgb.sequence <= self._emitted_rgb_sequence:
            return RgbdPairDecision(
                None,
                "awaiting_new_rgb",
                "the latest RGB frame was already processed",
                False,
            )
        if depth is None:
            return RgbdPairDecision(
                None,
                "missing_depth",
                "no aligned depth frame is available",
                True,
            )
        if depth.sequence <= self._emitted_depth_sequence:
            return RgbdPairDecision(
                None,
                "awaiting_new_depth",
                "the latest aligned depth frame was already processed",
                False,
            )

        rgb_age = now_monotonic - rgb.received_monotonic
        depth_age = now_monotonic - depth.received_monotonic
        if rgb_age < 0.0 or depth_age < 0.0:
            return RgbdPairDecision(
                None,
                "future_receive_time",
                "RGB-D receive timestamps are ahead of the pairing clock",
                True,
            )
        if rgb_age > self._max_frame_age_s or depth_age > self._max_frame_age_s:
            return RgbdPairDecision(
                None,
                "stale_rgbd_pair",
                (
                    f"RGB/depth receive ages are {rgb_age:.3f}s/"
                    f"{depth_age:.3f}s"
                ),
                True,
            )

        source_delta = abs(rgb.source_timestamp_s - depth.source_timestamp_s)
        if source_delta > self._max_source_delta_s and not math.isclose(
            source_delta,
            self._max_source_delta_s,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            if rgb.source_timestamp_s > depth.source_timestamp_s:
                status = "awaiting_matching_depth"
                waiting_for = "depth"
            else:
                status = "awaiting_matching_rgb"
                waiting_for = "RGB"
            return RgbdPairDecision(
                None,
                status,
                (
                    f"RGB/depth source stamp delta {source_delta:.3f}s exceeds "
                    f"{self._max_source_delta_s:.3f}s; waiting for matching "
                    f"{waiting_for} frame"
                ),
                False,
            )

        self._emitted_rgb_sequence = rgb.sequence
        self._emitted_depth_sequence = depth.sequence
        return RgbdPairDecision(
            RgbdFramePair(rgb=rgb, depth=depth, source_delta_s=source_delta),
            "paired",
            f"RGB/depth source stamp delta {source_delta:.3f}s",
            False,
        )

    def clear(self) -> None:
        with self._lock:
            self._rgb = None
            self._depth = None
            self._rgb_sequence = 0
            self._depth_sequence = 0
            self._emitted_rgb_sequence = 0
            self._emitted_depth_sequence = 0


__all__ = [
    "LatestRgbdPairer",
    "RgbdFramePair",
    "RgbdPairDecision",
    "TimedRgbdFrame",
]
