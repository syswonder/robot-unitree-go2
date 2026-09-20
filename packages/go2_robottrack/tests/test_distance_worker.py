from __future__ import annotations

import threading
import time
import unittest

from go2_robottrack.distance_worker import (
    RgbdDistanceWorker,
    fresh_rgbd_measurement_timestamp,
)
from go2_robottrack.rgbd_sync import RgbdFramePair, TimedRgbdFrame


def _pair(sequence: int) -> RgbdFramePair:
    received = time.monotonic()
    rgb = TimedRgbdFrame(
        image=f"rgb-{sequence}",
        source_timestamp_s=float(sequence),
        received_monotonic=received,
        sequence=sequence,
    )
    depth = TimedRgbdFrame(
        image=f"depth-{sequence}",
        source_timestamp_s=float(sequence),
        received_monotonic=received,
        sequence=sequence,
    )
    return RgbdFramePair(rgb=rgb, depth=depth, source_delta_s=0.0)


def _pair_with_receive_times(
    sequence: int,
    *,
    rgb_received: float,
    depth_received: float,
) -> RgbdFramePair:
    rgb = TimedRgbdFrame(
        image=f"rgb-{sequence}",
        source_timestamp_s=float(sequence),
        received_monotonic=rgb_received,
        sequence=sequence,
    )
    depth = TimedRgbdFrame(
        image=f"depth-{sequence}",
        source_timestamp_s=float(sequence),
        received_monotonic=depth_received,
        sequence=sequence,
    )
    return RgbdFramePair(rgb=rgb, depth=depth, source_delta_s=0.0)


class BlockingEstimator:
    def __init__(self) -> None:
        self.entered = threading.Event()
        self.release = threading.Event()
        self.sequences: list[int] = []

    def reset(self) -> None:
        pass

    def estimate(self, rgb, depth, *, frame_timestamp_s=None, now_s=None):
        del depth, frame_timestamp_s, now_s
        sequence = int(str(rgb).split("-")[-1])
        self.sequences.append(sequence)
        if len(self.sequences) == 1:
            self.entered.set()
            self.release.wait(1.0)
        return f"result-{sequence}"


class BlockingStatefulEstimator:
    def __init__(self) -> None:
        self.entered = threading.Event()
        self.release = threading.Event()
        self.second_complete = threading.Event()
        self.reset_complete = threading.Event()
        self.operations: list[str] = []
        self._estimating = False

    def reset(self) -> None:
        if self._estimating:
            raise AssertionError("reset must not run concurrently with estimate")
        self.operations.append("reset")
        self.reset_complete.set()

    def estimate(self, rgb, depth, *, frame_timestamp_s=None, now_s=None):
        del depth, frame_timestamp_s, now_s
        sequence = int(str(rgb).split("-")[-1])
        self._estimating = True
        self.operations.append(f"estimate-start-{sequence}")
        try:
            if sequence == 1:
                self.entered.set()
                self.release.wait(1.0)
            return f"result-{sequence}"
        finally:
            self.operations.append(f"estimate-end-{sequence}")
            self._estimating = False
            if sequence == 2:
                self.second_complete.set()


class DistanceWorkerTests(unittest.TestCase):
    def test_busy_worker_drops_intermediate_pair_and_takes_latest(self) -> None:
        estimator = BlockingEstimator()
        results: list[tuple[int, str]] = []
        worker = RgbdDistanceWorker(
            estimator,
            on_result=lambda pair, result, epoch: results.append(
                (pair.rgb.sequence, result)
            ),
        )
        worker.start()
        try:
            self.assertTrue(worker.submit(_pair(1)))
            self.assertTrue(estimator.entered.wait(1.0))
            self.assertTrue(worker.submit(_pair(2)))
            self.assertTrue(worker.submit(_pair(3)))
            estimator.release.set()
            deadline = time.monotonic() + 1.0
            while (
                (len(estimator.sequences) < 2 or not results)
                and time.monotonic() < deadline
            ):
                time.sleep(0.005)
            self.assertEqual(estimator.sequences[:2], [1, 3])
            self.assertEqual(results, [(1, "result-1"), (3, "result-3")])
            self.assertNotIn(2, estimator.sequences)
        finally:
            estimator.release.set()
            self.assertTrue(worker.stop())

    def test_continuous_new_pairs_do_not_starve_inflight_measurements(self) -> None:
        """New camera frames must not revoke a still-fresh running estimate."""
        entered = [threading.Event() for _ in range(3)]
        release = [threading.Event() for _ in range(3)]
        committed = [threading.Event() for _ in range(3)]
        measurements: list[int] = []

        class Estimator:
            calls = 0

            def reset(self) -> None:
                pass

            def estimate(self, rgb, depth, **kwargs):
                index = self.calls
                self.calls += 1
                if index < 3:
                    entered[index].set()
                    release[index].wait(2.0)
                return rgb

        def on_result(pair, result, epoch) -> None:
            accepted, _ = worker.commit_if_current(
                epoch, lambda: measurements.append(pair.rgb.sequence)
            )
            if accepted and pair.rgb.sequence <= 3:
                committed[pair.rgb.sequence - 1].set()

        worker = RgbdDistanceWorker(Estimator(), on_result=on_result)
        worker.start()
        try:
            worker.submit(_pair(1))
            for index in range(3):
                self.assertTrue(entered[index].wait(1.0))
                worker.submit(_pair(index + 2))
                release[index].set()
                self.assertTrue(
                    committed[index].wait(1.0),
                    "a newer pending pair discarded the running measurement",
                )
            self.assertEqual(measurements[:3], [1, 2, 3])
        finally:
            for event in release:
                event.set()
            self.assertTrue(worker.stop())

    def test_result_callback_receives_pair_and_result(self) -> None:
        callback = threading.Event()
        observed: list[tuple[int, str]] = []

        class Estimator:
            def reset(self) -> None:
                pass

            def estimate(self, rgb, depth, *, frame_timestamp_s=None, now_s=None):
                self.args = (rgb, depth, frame_timestamp_s, now_s)
                return "valid"

        estimator = Estimator()

        def on_result(pair, result, epoch) -> None:
            self.assertTrue(worker.is_current(epoch))
            observed.append((pair.rgb.sequence, result))
            callback.set()

        worker = RgbdDistanceWorker(estimator, on_result=on_result)
        worker.start()
        try:
            pair = _pair(7)
            self.assertTrue(worker.submit(pair))
            self.assertTrue(callback.wait(1.0))
            self.assertEqual(observed, [(7, "valid")])
            self.assertEqual(estimator.args[0:2], ("rgb-7", "depth-7"))
            self.assertEqual(estimator.args[2], pair.rgb.received_monotonic)
            self.assertIsInstance(estimator.args[3], float)
        finally:
            self.assertTrue(worker.stop())

    def test_estimator_exception_calls_error_and_worker_continues(self) -> None:
        errors: list[tuple[int, str]] = []
        results: list[int] = []
        complete = threading.Event()

        class Estimator:
            def reset(self) -> None:
                pass

            def estimate(self, rgb, depth, *, frame_timestamp_s=None, now_s=None):
                del depth, frame_timestamp_s, now_s
                if rgb == "rgb-1":
                    raise RuntimeError("detector failed")
                return "ok"

        def on_result(pair, result, epoch) -> None:
            self.assertTrue(worker.is_current(epoch))
            self.assertEqual(result, "ok")
            results.append(pair.rgb.sequence)
            complete.set()

        worker = RgbdDistanceWorker(
            Estimator(),
            on_result=on_result,
            on_error=lambda pair, error, epoch: errors.append(
                (pair.rgb.sequence, str(error))
            ),
        )
        worker.start()
        try:
            self.assertTrue(worker.submit(_pair(1)))
            deadline = time.monotonic() + 1.0
            while not errors and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertEqual(errors, [(1, "detector failed")])
            self.assertTrue(worker.submit(_pair(2)))
            self.assertTrue(complete.wait(1.0))
            self.assertEqual(results, [2])
        finally:
            self.assertTrue(worker.stop())

    def test_stop_wakes_idle_worker_and_rejects_new_pairs(self) -> None:
        class Estimator:
            def reset(self) -> None:
                pass

            def estimate(self, rgb, depth, *, frame_timestamp_s=None, now_s=None):
                raise AssertionError("idle worker must not evaluate")

        worker = RgbdDistanceWorker(Estimator())
        worker.start()
        self.assertTrue(worker.running)
        started = time.monotonic()
        self.assertTrue(worker.stop(timeout_s=0.5))
        self.assertLess(time.monotonic() - started, 0.5)
        self.assertFalse(worker.running)
        self.assertFalse(worker.submit(_pair(1)))

    def test_stop_join_is_bounded_while_estimator_is_blocked(self) -> None:
        estimator = BlockingEstimator()
        callbacks: list[str] = []
        worker = RgbdDistanceWorker(
            estimator,
            on_result=lambda pair, result, epoch: callbacks.append(result),
        )
        worker.start()
        self.assertTrue(worker.submit(_pair(1)))
        self.assertTrue(estimator.entered.wait(1.0))
        started = time.monotonic()
        self.assertFalse(worker.stop(timeout_s=0.02))
        self.assertLess(time.monotonic() - started, 0.2)
        estimator.release.set()
        self.assertTrue(worker.stop(timeout_s=1.0))
        self.assertEqual(callbacks, [])

    def test_invalidate_suppresses_inflight_result_before_next_pair(self) -> None:
        estimator = BlockingEstimator()
        measurements: list[int] = []
        next_complete = threading.Event()

        def on_result(pair, result, epoch) -> None:
            committed, _value = worker.commit_if_current(
                epoch,
                lambda: measurements.append(pair.rgb.sequence),
            )
            if committed and pair.rgb.sequence == 2:
                next_complete.set()

        worker = RgbdDistanceWorker(estimator, on_result=on_result)
        worker.start()
        try:
            self.assertTrue(worker.submit(_pair(1)))
            self.assertTrue(estimator.entered.wait(1.0))
            self.assertTrue(worker.invalidate())
            estimator.release.set()
            self.assertTrue(worker.submit(_pair(2)))
            self.assertTrue(next_complete.wait(1.0))
            self.assertEqual(estimator.sequences, [1, 2])
            self.assertEqual(measurements, [2])
        finally:
            estimator.release.set()
            self.assertTrue(worker.stop())

    def test_invalidate_resets_state_after_blocked_estimate_before_recovery(self) -> None:
        estimator = BlockingStatefulEstimator()
        results: list[int] = []
        result_complete = threading.Event()

        def on_result(pair, result, epoch) -> None:
            committed, _value = worker.commit_if_current(
                epoch,
                lambda: results.append(pair.rgb.sequence),
            )
            if committed:
                self.assertEqual(result, "result-2")
                result_complete.set()

        worker = RgbdDistanceWorker(estimator, on_result=on_result)
        worker.start()
        try:
            self.assertTrue(worker.submit(_pair(1)))
            self.assertTrue(estimator.entered.wait(1.0))
            self.assertTrue(worker.invalidate())
            self.assertTrue(worker.submit(_pair(2)))

            # invalidate() must not reset the stateful estimator from the
            # caller thread while estimate(1) is still blocked.
            self.assertFalse(estimator.reset_complete.wait(0.05))
            estimator.release.set()

            self.assertTrue(estimator.reset_complete.wait(1.0))
            self.assertTrue(estimator.second_complete.wait(1.0))
            self.assertTrue(result_complete.wait(1.0))
            self.assertEqual(
                estimator.operations,
                [
                    "estimate-start-1",
                    "estimate-end-1",
                    "reset",
                    "estimate-start-2",
                    "estimate-end-2",
                ],
            )
            self.assertEqual(results, [2])
        finally:
            estimator.release.set()
            self.assertTrue(worker.stop())

    def test_delayed_result_keeps_pair_time_and_expires_instead_of_refreshing(self) -> None:
        pair = _pair(9)
        received = pair.received_monotonic
        self.assertEqual(
            fresh_rgbd_measurement_timestamp(
                pair,
                now_monotonic=received + 0.6,
                max_age_s=0.6,
            ),
            received,
        )
        self.assertIsNone(
            fresh_rgbd_measurement_timestamp(
                pair,
                now_monotonic=received + 0.600001,
                max_age_s=0.6,
            )
        )

    def test_delayed_result_ages_from_older_rgbd_member(self) -> None:
        pair = _pair_with_receive_times(
            10,
            rgb_received=50.0,
            depth_received=50.4,
        )
        self.assertEqual(pair.received_monotonic, 50.0)
        self.assertIsNone(
            fresh_rgbd_measurement_timestamp(
                pair,
                now_monotonic=50.600001,
                max_age_s=0.6,
            )
        )

    def test_estimator_timestamp_uses_rgb_receive_not_pair_completion(self) -> None:
        complete = threading.Event()

        class Estimator:
            def __init__(self) -> None:
                self.frame_timestamp_s: float | None = None

            def reset(self) -> None:
                pass

            def estimate(self, rgb, depth, *, frame_timestamp_s=None, now_s=None):
                del rgb, depth, now_s
                self.frame_timestamp_s = frame_timestamp_s
                return "ok"

        estimator = Estimator()
        pair = _pair_with_receive_times(
            11,
            rgb_received=60.0,
            depth_received=60.4,
        )
        worker = RgbdDistanceWorker(
            estimator,
            on_result=lambda pair, result, epoch: complete.set(),
        )
        worker.start()
        try:
            self.assertTrue(worker.submit(pair))
            self.assertTrue(complete.wait(1.0))
            self.assertEqual(estimator.frame_timestamp_s, 60.0)
            self.assertEqual(pair.completed_monotonic, 60.4)
            self.assertEqual(pair.received_monotonic, 60.0)
            self.assertIsNone(
                fresh_rgbd_measurement_timestamp(
                    pair,
                    now_monotonic=60.600001,
                    max_age_s=0.6,
                )
            )
        finally:
            self.assertTrue(worker.stop())

    def test_new_rgb_timestamps_increase_when_reusing_one_depth_frame(self) -> None:
        first_complete = threading.Event()
        second_complete = threading.Event()
        observed_results: list[int] = []

        class Estimator:
            def __init__(self) -> None:
                self.frame_timestamps: list[float] = []

            def reset(self) -> None:
                pass

            def estimate(self, rgb, depth, *, frame_timestamp_s=None, now_s=None):
                del depth, now_s
                self.frame_timestamps.append(float(frame_timestamp_s))
                return str(rgb)

        estimator = Estimator()

        def on_result(pair, result, epoch) -> None:
            del result, epoch
            observed_results.append(pair.rgb.sequence)
            if len(observed_results) == 1:
                first_complete.set()
            elif len(observed_results) == 2:
                second_complete.set()

        worker = RgbdDistanceWorker(estimator, on_result=on_result)
        worker.start()
        try:
            first = _pair_with_receive_times(
                12,
                rgb_received=70.2,
                depth_received=70.0,
            )
            second = _pair_with_receive_times(
                13,
                rgb_received=70.4,
                depth_received=70.0,
            )
            self.assertTrue(worker.submit(first))
            self.assertTrue(first_complete.wait(1.0))
            self.assertTrue(worker.submit(second))
            self.assertTrue(second_complete.wait(1.0))
            self.assertEqual(observed_results, [12, 13])
            self.assertEqual(estimator.frame_timestamps, [70.2, 70.4])
            self.assertEqual(first.received_monotonic, 70.0)
            self.assertEqual(second.received_monotonic, 70.0)
        finally:
            self.assertTrue(worker.stop())


if __name__ == "__main__":
    unittest.main()
