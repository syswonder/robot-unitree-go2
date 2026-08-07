"""Latest-frame inference worker without ROS dependencies."""

from __future__ import annotations

import threading
from typing import Callable, Protocol

from .core import EncodedFrame, InferencePlan, LatestFrameMailbox, PlanStore


class Evaluator(Protocol):
    def evaluate(self, frame: EncodedFrame) -> InferencePlan:
        ...


class InferenceWorker:
    """Run one blocking inference at a time and always pick the newest next frame."""

    def __init__(
        self,
        mailbox: LatestFrameMailbox,
        evaluator: Evaluator,
        plans: PlanStore,
        *,
        on_plan: Callable[[EncodedFrame, InferencePlan], None] | None = None,
        on_error: Callable[[EncodedFrame, Exception], None] | None = None,
    ) -> None:
        self._mailbox = mailbox
        self._evaluator = evaluator
        self._plans = plans
        self._on_plan = on_plan
        self._on_error = on_error
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="go2-robottrack-inference",
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> bool:
        self._stop_event.set()
        close = getattr(self._evaluator, "close", None)
        if callable(close):
            close()
        thread = self._thread
        if thread is None:
            return True
        thread.join(max(0.0, float(timeout_s)))
        return not thread.is_alive()

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _run(self) -> None:
        last_sequence = 0
        while not self._stop_event.is_set():
            frame = self._mailbox.wait_after(
                last_sequence,
                timeout_s=0.1,
                stop_event=self._stop_event,
            )
            if frame is None:
                continue
            last_sequence = frame.sequence
            try:
                plan = self._evaluator.evaluate(frame)
                self._plans.update(plan)
                if self._on_plan is not None:
                    self._on_plan(frame, plan)
            except Exception as error:  # keep serving later camera frames
                if self._on_error is not None:
                    self._on_error(frame, error)
