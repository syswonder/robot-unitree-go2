"""Robonix lifecycle provider for the Go2 RobotTrack ROS runtime."""

from __future__ import annotations

import threading
from pathlib import Path
import sys
from typing import Any, Callable

from robonix_api import Deferred, Err, Ok, Primitive

def _add_codegen_paths() -> None:
    package_root = Path(__file__).resolve().parent.parent
    for path in (
        package_root / "rbnx-build" / "codegen" / "proto_gen",
        package_root / "rbnx-build" / "codegen" / "robonix_mcp_types",
    ):
        if path.is_dir():
            sys.path.insert(0, str(path))


_add_codegen_paths()

from go2_robottrack_control_mcp import (  # noqa: E402
    FollowDistance_Request,
    FollowDistance_Response,
)

from .core import RuntimeConfig
from .follow_distance_runtime import configure_distance_runtime, distance_runtime


provider = Primitive(id="go2_robottrack", namespace="robonix/primitive/follow")
DISTANCE_CONTRACT = "robonix/primitive/follow/distance"

_lock = threading.Lock()
_config: dict[str, Any] = {}
_deployment_metadata: dict[str, str] = {}
_active = False
_runtime_thread: threading.Thread | None = None
_runtime_stop: threading.Event | None = None
_runtime_exited: threading.Event | None = None
_runtime_errors: list[str] = []


def _default_runner(
    config: dict[str, Any],
    stop_event: threading.Event,
    ready_event: threading.Event,
    errors: list[str],
) -> None:
    from .ros_node import run_ros_runtime

    run_ros_runtime(config, stop_event, ready_event, errors)


_ros_runner: Callable[
    [dict[str, Any], threading.Event, threading.Event, list[str]], None
] = _default_runner


def _ros_runner_entry(
    config: dict[str, Any],
    stop_event: threading.Event,
    ready_event: threading.Event,
    errors: list[str],
    exited_event: threading.Event,
) -> None:
    """Run the replaceable ROS entrypoint and publish an atomic exit state."""

    global _active
    try:
        # Keep the four-argument runner interface stable for the real ROS
        # implementation and for lifecycle tests that replace it.
        _ros_runner(config, stop_event, ready_event, errors)
    finally:
        # Set this before taking the provider lock.  An activation that is
        # concurrently checking Thread.is_alive() can therefore observe that
        # the runner returned even while this finalizer waits for the lock.
        exited_event.set()
        with _lock:
            if (
                _runtime_thread is threading.current_thread()
                and _runtime_exited is exited_event
            ):
                _active = False
                distance_runtime.set_active(
                    False, "RobotTrack ROS runtime exited"
                )


def _normalize_metadata(config: dict[str, Any]) -> dict[str, str]:
    result: dict[str, str] = {}
    for name in ("camera_info_topic", "asset_manifest", "upstream_root"):
        if config.get(name) is None:
            continue
        value = str(config[name]).strip()
        if not value:
            raise ValueError(f"{name} must not be empty when provided")
        result[name] = value
    return result


@provider.on_init
def initialize(config: dict[str, Any]):
    global _config, _deployment_metadata
    if not isinstance(config, dict):
        return Err("RobotTrack config must be a JSON object")
    with _lock:
        if _active or _runtime_thread is not None:
            return Err("cannot reinitialize while RobotTrack is active")
    try:
        runtime = RuntimeConfig.from_mapping(config)
        metadata = _normalize_metadata(config)
    except (TypeError, ValueError) as error:
        return Err(str(error))
    parameters = runtime.as_ros_parameters()
    try:
        configure_distance_runtime(runtime)
    except (TypeError, ValueError) as error:
        return Err(str(error))
    _config = parameters
    _deployment_metadata = metadata
    return Ok()


@provider.on_activate
def activate():
    global _active, _runtime_thread, _runtime_stop, _runtime_exited
    global _runtime_errors
    if not _config:
        return Err("RobotTrack provider was not initialized")
    with _lock:
        # A non-None handle is an ownership reservation even before Thread.start
        # and after a timed-out stop. Never create a second ROS publisher set
        # until the prior runtime has been observed fully exited and cleared.
        if _active or _runtime_thread is not None:
            return Deferred("RobotTrack provider is already active")
        stop_event = threading.Event()
        ready_event = threading.Event()
        exited_event = threading.Event()
        errors: list[str] = []
        thread = threading.Thread(
            target=_ros_runner_entry,
            args=(
                dict(_config),
                stop_event,
                ready_event,
                errors,
                exited_event,
            ),
            name="go2-robottrack-ros",
            daemon=True,
        )
        _runtime_stop = stop_event
        _runtime_thread = thread
        _runtime_exited = exited_event
        _runtime_errors = errors
    try:
        thread.start()
    except RuntimeError as error:
        with _lock:
            if _runtime_thread is thread:
                _runtime_thread = None
                _runtime_stop = None
                _runtime_exited = None
                _runtime_errors = []
        return Err(f"RobotTrack ROS runtime thread could not start: {error}")
    if not ready_event.wait(5.0):
        stop_event.set()
        thread.join(1.0)
        if not thread.is_alive():
            with _lock:
                if _runtime_thread is thread:
                    _runtime_thread = None
                    _runtime_stop = None
                    _runtime_exited = None
                    _runtime_errors = []
        return Err("RobotTrack ROS runtime did not start within 5 seconds")

    failure_detail = ""
    with _lock:
        owns_runtime = (
            _runtime_thread is thread and _runtime_exited is exited_event
        )
        # Read liveness first, then the explicit exit signal.  If the runner
        # returns between these reads, its finalizer sets exited_event before
        # waiting for this lock and activation fails instead of committing a
        # stale active=True state.
        alive = thread.is_alive()
        exited = exited_event.is_set()
        if not owns_runtime:
            failure_detail = "lifecycle changed during startup"
        elif errors:
            failure_detail = errors[-1]
        elif exited or not alive:
            failure_detail = "ROS runtime exited during startup"
        else:
            _active = True
            # Commit both lifecycle views while the exit finalizer is excluded
            # by the same provider lock.  Whichever transition wins the lock
            # is immediately followed by the other if the runner then exits.
            distance_runtime.set_active(True)
            return Ok()

        if owns_runtime:
            _active = False
            distance_runtime.set_active(
                False, "RobotTrack ROS runtime failed during startup"
            )

    stop_event.set()
    thread.join(1.0)
    if not thread.is_alive():
        with _lock:
            if _runtime_thread is thread:
                _runtime_thread = None
                _runtime_stop = None
                _runtime_exited = None
                _runtime_errors = []
    return Err(f"RobotTrack ROS runtime failed: {failure_detail}")


def _stop_runtime() -> tuple[bool, str]:
    global _active, _runtime_thread, _runtime_stop, _runtime_exited
    global _runtime_errors
    with _lock:
        thread = _runtime_thread
        stop_event = _runtime_stop
        _active = False
    distance_runtime.set_active(False)
    if stop_event is not None:
        stop_event.set()
    if thread is not None:
        thread.join(4.0)
        if thread.is_alive():
            return False, "RobotTrack ROS runtime did not stop within 4 seconds"
    with _lock:
        if _runtime_thread is thread:
            _runtime_thread = None
            _runtime_stop = None
            _runtime_exited = None
            _runtime_errors = []
    return True, ""


@provider.on_deactivate
def deactivate():
    ok, detail = _stop_runtime()
    return Ok() if ok else Err(detail)


@provider.on_shutdown
def shutdown():
    ok, detail = _stop_runtime()
    return Ok() if ok else Err(detail)


@provider.mcp(DISTANCE_CONTRACT)
def follow_distance(
    request: FollowDistance_Request,
) -> FollowDistance_Response:
    """Set, adjust, or read the optional metric follow-distance target."""

    result = distance_runtime.handle(request.operation, request.meters)
    snapshot = result.snapshot
    measured = snapshot.measured_distance_m
    return FollowDistance_Response(
        accepted=result.accepted,
        enabled=snapshot.enabled,
        active=snapshot.active,
        has_measurement=measured is not None,
        target_distance_m=snapshot.target_distance_m,
        measured_distance_m=0.0 if measured is None else measured,
        status=snapshot.status,
        detail=result.detail,
    )


if __name__ == "__main__":
    provider.run()
