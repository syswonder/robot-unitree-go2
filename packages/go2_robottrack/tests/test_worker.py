from __future__ import annotations

import threading
import time
import unittest

from go2_robottrack.core import (
    InferencePlan,
    LatestFrameMailbox,
    PlanStore,
    VelocityCommand,
)
from go2_robottrack.worker import InferenceWorker


class BlockingEvaluator:
    def __init__(self) -> None:
        self.entered = threading.Event()
        self.release = threading.Event()
        self.sequences: list[int] = []

    def evaluate(self, frame):
        self.sequences.append(frame.sequence)
        if len(self.sequences) == 1:
            self.entered.set()
            self.release.wait(1.0)
        return InferencePlan(VelocityCommand(frame.sequence * 0.01, 0.0), None)


class WorkerTests(unittest.TestCase):
    def test_busy_worker_skips_intermediate_frames_and_takes_latest(self) -> None:
        mailbox = LatestFrameMailbox()
        plans = PlanStore()
        evaluator = BlockingEvaluator()
        worker = InferenceWorker(mailbox, evaluator, plans)
        worker.start()
        try:
            mailbox.put(b"one", source_timestamp=1.0)
            self.assertTrue(evaluator.entered.wait(1.0))
            mailbox.put(b"two", source_timestamp=2.0)
            mailbox.put(b"three", source_timestamp=3.0)
            evaluator.release.set()
            deadline = time.monotonic() + 1.0
            while len(evaluator.sequences) < 2 and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertEqual(evaluator.sequences[:2], [1, 3])
            self.assertNotIn(2, evaluator.sequences)
        finally:
            evaluator.release.set()
            self.assertTrue(worker.stop())


if __name__ == "__main__":
    unittest.main()
