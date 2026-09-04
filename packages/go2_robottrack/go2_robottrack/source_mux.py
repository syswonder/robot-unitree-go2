"""ROS-independent mutually exclusive command-source mux."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Iterable


SOURCE_NAVIGATION = "navigation"
SOURCE_ROBOTTRACK = "robottrack"
VALID_SOURCES = (SOURCE_NAVIGATION, SOURCE_ROBOTTRACK)


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class TwistCommand:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

    @classmethod
    def finite(cls, values: Iterable[float]) -> "TwistCommand":
        items = tuple(values)
        if len(items) != 6:
            raise ValueError("twist must contain exactly six components")
        checked = tuple(_finite(value, f"twist[{index}]") for index, value in enumerate(items))
        return cls(*checked)


ZERO_TWIST = TwistCommand()


@dataclass(frozen=True)
class MuxSelection:
    command: TwistCommand
    selected_source: str
    reason: str
    age_s: float | None
    version: int


class CommandSourceMux:
    """Keep one latest command per source and emit exactly the selected source."""

    def __init__(self, selected_source: str, *, max_age_s: float) -> None:
        selected = str(selected_source).strip().lower()
        if selected not in VALID_SOURCES:
            raise ValueError(f"selected_source must be one of {VALID_SOURCES}")
        max_age = _finite(max_age_s, "max_age_s")
        if max_age <= 0.0:
            raise ValueError("max_age_s must be positive")
        self._lock = threading.Lock()
        self._selected_source = selected
        self._max_age_s = max_age
        self._latest: dict[str, tuple[TwistCommand, float, int]] = {}
        self._version = 0

    @property
    def selected_source(self) -> str:
        with self._lock:
            return self._selected_source

    def update(
        self,
        source: str,
        command: TwistCommand,
        *,
        received_monotonic: float | None = None,
    ) -> bool:
        name = str(source).strip().lower()
        if name not in VALID_SOURCES:
            raise ValueError(f"source must be one of {VALID_SOURCES}")
        received = (
            time.monotonic()
            if received_monotonic is None
            else _finite(received_monotonic, "received_monotonic")
        )
        checked = TwistCommand.finite(
            (
                command.linear_x,
                command.linear_y,
                command.linear_z,
                command.angular_x,
                command.angular_y,
                command.angular_z,
            )
        )
        with self._lock:
            previous = self._latest.get(name)
            if previous is not None and received <= previous[1]:
                return False
            self._version += 1
            self._latest[name] = (checked, received, self._version)
            return True

    def select_source(self, source: str) -> None:
        name = str(source).strip().lower()
        if name not in VALID_SOURCES:
            raise ValueError(f"source must be one of {VALID_SOURCES}")
        with self._lock:
            if name == self._selected_source:
                return
            self._selected_source = name
            # Switching is an epoch boundary. A command received before the
            # switch must not become active simply because it is still young.
            self._latest.clear()
            self._version += 1

    def output(self, *, now: float | None = None) -> MuxSelection:
        current = time.monotonic() if now is None else _finite(now, "now")
        with self._lock:
            selected = self._selected_source
            entry = self._latest.get(selected)
            version = self._version
        if entry is None:
            return MuxSelection(ZERO_TWIST, selected, "selected_source_missing", None, version)
        command, received, command_version = entry
        age = max(0.0, current - received)
        if age > self._max_age_s:
            return MuxSelection(ZERO_TWIST, selected, "selected_source_stale", age, command_version)
        return MuxSelection(command, selected, "selected_source_fresh", age, command_version)
