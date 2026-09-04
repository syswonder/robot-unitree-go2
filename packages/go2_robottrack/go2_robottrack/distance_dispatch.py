"""ROS-independent selection of the optional distance-adjusted command."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .core import DispatchState, VelocityCommand, ZERO_COMMAND
from .distance_control import DistanceControlResult


@dataclass(frozen=True)
class DistanceDispatchSelection:
    command: VelocityCommand
    reason: str


def select_distance_dispatch(
    state: DispatchState,
    *,
    distance_enabled: bool,
    apply_distance: Callable[[VelocityCommand, float], DistanceControlResult],
    now_monotonic: float,
) -> DistanceDispatchSelection:
    """Keep missing/stale plans at strict zero and adjust only active plans."""

    if state.reason != "active_plan":
        return DistanceDispatchSelection(ZERO_COMMAND, state.reason)
    if not distance_enabled:
        return DistanceDispatchSelection(state.command, state.reason)
    result = apply_distance(state.command, now_monotonic)
    return DistanceDispatchSelection(result.command, result.reason)


__all__ = ["DistanceDispatchSelection", "select_distance_dispatch"]
