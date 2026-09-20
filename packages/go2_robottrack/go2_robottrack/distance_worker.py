"""Latest-only asynchronous RGB-D distance estimation.

Person detection and optical-flow tracking are intentionally kept out of the
ROS executor callback.  While one pair is being evaluated, newly submitted
pairs replace the pending pair so the estimator always catches up to the most
recent camera input instead of building latency.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Protocol, TypeVar

from .rgbd_sync import RgbdFramePair


class DistanceEstimator(Protocol):
    def reset(self) -> None:
        """Clear state retained across RGB-D frame pairs."""

    def estimate(
        self,
        bgr: Any,
        aligned_depth: Any,
        *,
        frame_timestamp_s: float | None = None,
        now_s: float | None = None,
    ) -> Any:
        ...


ResultCallback = Callable[[RgbdFramePair, Any, int], None]
ErrorCallback = Callable[[RgbdFramePair, Exception, int], None]
CommitResult = TypeVar("CommitResult")


def fresh_rgbd_measurement_timestamp(
    pair: RgbdFramePair,
    *,
    now_monotonic: float,
    max_age_s: float,
) -> float | None:
    """Return the pair receive time only while a completed result is fresh."""

    now = float(now_monotonic)
    maximum_age = float(max_age_s)
    received = float(pair.received_monotonic)
    if (
        not math.isfinite(now)
        or not math.isfinite(maximum_age)
        or not math.isfinite(received)
    ):
        raise ValueError("RGB-D result timing values must be finite")
    if maximum_age <= 0.0:
        raise ValueError("max_age_s must be positive")
    age = now - received
    if age < 0.0 or (
        age > maximum_age
        and not math.isclose(age, maximum_age, rel_tol=0.0, abs_tol=1e-9)
    ):
        return None
    return received


class RgbdDistanceWorker:
    """Evaluate one RGB-D pair at a time and retain only the newest pending pair."""

    def __init__(
        self,
        estimator: DistanceEstimator,
        *,
        on_result: ResultCallback | None = None,
        on_error: ErrorCallback | None = None,
    ) -> None:
        self._estimator = estimator
        self._on_result = on_result
        self._on_error = on_error
        self._condition = threading.Condition()
        self._latest: tuple[int, RgbdFramePair] | None = None
        self._reset_before_estimate = False
        self._epoch = 0
        self._stop = False
        self._started = False
        self._thread = threading.Thread(
            target=self._run,
            name="go2-robottrack-rgbd-distance",
            daemon=True,
        )

    def start(self) -> None:
        with self._condition:
            if self._started:
                return
            if self._stop:
                return
            self._started = True
        self._thread.start()

    def submit(self, pair: RgbdFramePair) -> bool:
        """Replace pending work without revoking a running measurement.

        Epochs represent explicit invalidation, not camera frame sequence.
        Incrementing on every submit starves results when camera frames arrive
        faster than estimation finishes. The result consumer still checks the
        original frame age before committing; a newer frame must not refresh it.
        """

        with self._condition:
            if self._stop:
                return False
            self._latest = (self._epoch, pair)
            self._condition.notify()
            return True

    def invalidate(self) -> bool:
        """Invalidate results and reset estimator state before the next pair.

        The reset is intentionally deferred to the worker thread.  A stateful
        detector/tracker must never be reset concurrently with ``estimate``.
        """

        with self._condition:
            if self._stop:
                return False
            self._epoch += 1
            self._latest = None
            self._reset_before_estimate = True
            self._condition.notify_all()
            return True

    def is_current(self, epoch: int) -> bool:
        """Return whether ``epoch`` may still update shared measurement state."""

        with self._condition:
            return not self._stop and int(epoch) == self._epoch

    def commit_if_current(
        self,
        epoch: int,
        commit: Callable[[], CommitResult],
    ) -> tuple[bool, CommitResult | None]:
        """Atomically validate an epoch and perform its short state update."""

        with self._condition:
            if self._stop or int(epoch) != self._epoch:
                return False, None
            return True, commit()

    def request_stop(self) -> None:
        """Wake an idle worker and discard pending work without joining it."""

        with self._condition:
            self._stop = True
            self._epoch += 1
            self._latest = None
            self._reset_before_estimate = False
            self._condition.notify_all()

    def stop(self, timeout_s: float = 2.0) -> bool:
        """Request shutdown and wait no longer than ``timeout_s``."""

        self.request_stop()
        if not self._started:
            return True
        self._thread.join(max(0.0, float(timeout_s)))
        return not self._thread.is_alive()

    @property
    def running(self) -> bool:
        return self._started and self._thread.is_alive()

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._latest is None and not self._stop:
                    self._condition.wait()
                if self._stop:
                    return
                work = self._latest
                self._latest = None
                reset_before_estimate = self._reset_before_estimate
                self._reset_before_estimate = False

            assert work is not None
            epoch, pair = work
            if reset_before_estimate:
                try:
                    self._estimator.reset()
                except Exception as error:
                    with self._condition:
                        if self._stop:
                            return
                        # Do not evaluate any later pair until a reset has
                        # succeeded.  A pair offered while reset was running
                        # remains the latest pending work for the next attempt.
                        self._reset_before_estimate = True
                    if self._on_error is not None:
                        try:
                            self._on_error(pair, error, epoch)
                        except Exception:
                            pass
                    continue

            with self._condition:
                if self._stop:
                    return
                if epoch != self._epoch:
                    continue
            try:
                result = self._estimator.estimate(
                    pair.rgb.image,
                    pair.depth.image,
                    frame_timestamp_s=pair.rgb.received_monotonic,
                    now_s=time.monotonic(),
                )
            except Exception as error:  # keep serving later RGB-D pairs
                with self._condition:
                    if self._stop:
                        return
                    if epoch != self._epoch:
                        continue
                if self._on_error is not None:
                    try:
                        self._on_error(pair, error, epoch)
                    except Exception:
                        # Diagnostics/state callbacks must not kill the worker.
                        pass
                continue

            with self._condition:
                if self._stop:
                    return
                if epoch != self._epoch:
                    continue
            if self._on_result is not None:
                try:
                    self._on_result(pair, result, epoch)
                except Exception as error:
                    if self._on_error is not None:
                        try:
                            self._on_error(pair, error, epoch)
                        except Exception:
                            pass


__all__ = [
    "DistanceEstimator",
    "RgbdDistanceWorker",
    "fresh_rgbd_measurement_timestamp",
]
