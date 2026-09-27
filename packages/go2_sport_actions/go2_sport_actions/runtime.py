# SPDX-License-Identifier: Apache-2.0
"""One-at-a-time Skill jobs through the already-owned Go2 chassis daemon.

An exact SDK request witness is dispatch evidence, not proof that the remote
accepted it or that a physical trick finished. Navigation and this Skill use separate runtime
sessions: the daemon socket accepts exactly one controller at a time.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import threading
import time
import uuid
from typing import Callable

from .ipc import ARM, DISARM, MOVE, PING, SPORT_ACTION, STOP, DaemonClient


@dataclass(frozen=True)
class Step:
    operation: int
    duration_s: float = 0.0
    action_id: int = 0
    vx: float = 0.0


# IDs match go2_chassis::SportAction. These durations are supervision windows; no
# duration is presented as measured completion of a vendor animation.
PLANS: dict[str, tuple[Step, ...]] = {
    "hello": (Step(SPORT_ACTION, 15.0, 1),),
    "stretch": (Step(SPORT_ACTION, 20.0, 2),),
    # The remote's complete Dance1 lasts about 20 s, while Client/firmware
    # dispatch can precede visible movement. This is a bounded supervision
    # window, not a claim that the robot dances for the entire duration.
    "dance_1": (Step(SPORT_ACTION, 45.0, 3),),
    "dance_2": (Step(SPORT_ACTION, 55.0, 4),),
    "new_year_greeting": (Step(SPORT_ACTION, 30.0, 17),),
    "crouch": (Step(SPORT_ACTION, 8.0, 14), Step(SPORT_ACTION, 8.0, 15)),
    "handstand": (Step(SPORT_ACTION, 8.0, 5), Step(SPORT_ACTION, 4.0, 6)),
    "handstand_walk": (
        Step(SPORT_ACTION, 8.0, 5), Step(MOVE, 5.0, vx=0.06),
        Step(STOP), Step(SPORT_ACTION, 4.0, 6),
    ),
}


@dataclass
class Run:
    name: str
    request_id: str
    run_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    state: str = "QUEUED"
    dispatched_steps: int = 0
    sdk_requests_accepted: int = 0
    detail: str = ""
    stop_acknowledged: bool = False
    cancel_requested: threading.Event = field(default_factory=threading.Event)
    thread: threading.Thread | None = None

    def snapshot(self) -> dict:
        return {
            "known": True,
            "run_id": self.run_id,
            "name": self.name,
            "request_id": self.request_id,
            "state": self.state,
            "dispatched_steps": self.dispatched_steps,
            "sdk_requests_accepted": self.sdk_requests_accepted,
            "detail": self.detail,
            "stop_acknowledged": self.stop_acknowledged,
            "physical_completion_verified": False,
        }


class Cancelled(Exception):
    pass


class ActionManager:
    def __init__(self, socket_path: str = "", *, client_factory: Callable = DaemonClient):
        self.socket_path = socket_path
        self._client_factory = client_factory
        self._lock = threading.RLock()
        self._runs: dict[str, Run] = {}
        self._active_run_id: str | None = None

    @property
    def configured(self) -> bool:
        return bool(self.socket_path)

    def execute(self, name: str, request_id: str = "") -> dict:
        if not isinstance(name, str) or name not in PLANS:
            return {"accepted": False, "run_id": "", "reason": "action_not_in_physical_plan",
                    "request_id": request_id, "motion_sent": False}
        if not self.configured:
            return {"accepted": False, "run_id": "", "reason": "physical_backend_not_configured",
                    "request_id": request_id, "motion_sent": False}
        with self._lock:
            if request_id:
                for prior in self._runs.values():
                    if prior.request_id == request_id:
                        return {"accepted": True, "run_id": prior.run_id,
                                "reason": "duplicate_request_id_existing_run",
                                "request_id": request_id, "motion_sent": False}
            if self._active_run_id is not None:
                return {"accepted": False, "run_id": "", "reason": "action_in_progress",
                        "request_id": request_id, "motion_sent": False}
            run = Run(name=name, request_id=request_id)
            self._runs[run.run_id] = run
            self._active_run_id = run.run_id
            worker = threading.Thread(target=self._worker, args=(run,), daemon=True,
                                      name=f"go2-sport-{run.run_id[:8]}")
            run.thread = worker
            worker.start()
            return {"accepted": True, "run_id": run.run_id,
                    "reason": "queued_check_status_for_dispatch",
                    "request_id": request_id, "motion_sent": False}

    def status(self, run_id: str) -> dict:
        with self._lock:
            run = self._runs.get(run_id)
            return (run.snapshot() if run else
                    {"known": False, "run_id": run_id, "state": "UNKNOWN",
                     "reason": "unknown_run_id", "motion_sent": False})

    def cancel(self, run_id: str) -> dict:
        with self._lock:
            run = self._runs.get(run_id)
            if run is None:
                return {"ok": False, "run_id": run_id, "reason": "unknown_run_id",
                        "stop_acknowledged": False}
            if self._active_run_id != run_id:
                return {"ok": False, "run_id": run_id, "reason": "run_already_finished",
                        "stop_acknowledged": run.stop_acknowledged}
            run.cancel_requested.set()
            return {"ok": True, "run_id": run_id, "reason": "cancel_requested",
                    "stop_acknowledged": run.stop_acknowledged}

    def cancel_active(self) -> dict:
        with self._lock:
            if self._active_run_id is None:
                return {"ok": False, "run_id": "", "reason": "no_active_action",
                        "stop_acknowledged": False}
            return self.cancel(self._active_run_id)

    def shutdown(self) -> None:
        with self._lock:
            active = self._runs.get(self._active_run_id or "")
        if active is not None:
            active.cancel_requested.set()
            if active.thread is not None:
                active.thread.join(timeout=3.0)

    @staticmethod
    def _hold(daemon: DaemonClient, run: Run, step: Step) -> None:
        end = time.monotonic() + step.duration_s
        while time.monotonic() < end:
            if run.cancel_requested.is_set():
                raise Cancelled()
            time.sleep(min(0.08, max(0.0, end - time.monotonic())))
            if run.cancel_requested.is_set():
                raise Cancelled()
            if step.operation == MOVE:
                daemon.call(MOVE, vx=step.vx)
            else:
                daemon.call(PING)

    def _worker(self, run: Run) -> None:
        handstand_entered = False
        crouched = False
        stop_ok = False
        error = ""
        cancelled = False
        with self._lock:
            run.state = "CONNECTING"
        try:
            with self._client_factory(self.socket_path) as daemon:
                try:
                    daemon.call(ARM)
                    with self._lock:
                        run.state = "RUNNING"
                    for step in PLANS[run.name]:
                        if run.cancel_requested.is_set():
                            raise Cancelled()
                        daemon.call(step.operation, action_id=step.action_id, vx=step.vx)
                        with self._lock:
                            run.dispatched_steps += 1
                            if step.operation == SPORT_ACTION:
                                run.sdk_requests_accepted += 1
                        if step.action_id == 5:
                            handstand_entered = True
                        elif step.action_id == 6:
                            handstand_entered = False
                        elif step.action_id == 14:
                            crouched = True
                        elif step.action_id == 15:
                            crouched = False
                        self._hold(daemon, run, step)
                except Cancelled:
                    cancelled = True
                except Exception as exc:
                    error = str(exc)
                finally:
                    # Never report a stop unless the daemon acknowledged it.
                    try:
                        daemon.call(STOP)
                        stop_ok = True
                    except Exception as exc:
                        error = error or f"stop_unconfirmed: {exc}"
                    if handstand_entered:
                        try:
                            daemon.call(SPORT_ACTION, action_id=6)
                        except Exception as exc:
                            error = error or f"handstand_exit_unconfirmed: {exc}"
                    if crouched:
                        try:
                            daemon.call(SPORT_ACTION, action_id=15)
                        except Exception as exc:
                            error = error or f"standup_unconfirmed: {exc}"
                    try:
                        daemon.call(DISARM)
                    except Exception as exc:
                        error = error or f"disarm_unconfirmed: {exc}"
        except Exception as exc:
            error = error or f"daemon_unavailable: {exc}"
        with self._lock:
            run.stop_acknowledged = stop_ok
            run.detail = error or "Exact requests dispatched; remote acceptance and physical completion not measured"
            run.state = ("FAILED" if error else "CANCELLED" if cancelled
                         else "SDK_ACCEPTED_COMPLETION_UNVERIFIED")
            if self._active_run_id == run.run_id:
                self._active_run_id = None
