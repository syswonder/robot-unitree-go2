"""Robonix lifecycle provider for the Go2 RobotTrack ROS runtime."""

from __future__ import annotations

import threading
from typing import Any, Callable

from robonix_api import Deferred, Err, Ok, Primitive

from .core import RuntimeConfig


provider = Primitive(id="go2_robottrack", namespace="robonix/primitive/follow")

_lock = threading.Lock()
_config: dict[str, Any] = {}
_deployment_metadata: dict[str, str] = {}
_active = False
_runtime_thread: threading.Thread | None = None
_runtime_stop: threading.Event | None = None
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
    _config = runtime.as_ros_parameters()
    _deployment_metadata = metadata
    return Ok()


@provider.on_activate
def activate():
    global _active, _runtime_thread, _runtime_stop, _runtime_errors
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
        errors: list[str] = []
        thread = threading.Thread(
            target=_ros_runner,
            args=(dict(_config), stop_event, ready_event, errors),
            name="go2-robottrack-ros",
            daemon=True,
        )
        _runtime_stop = stop_event
        _runtime_thread = thread
        _runtime_errors = errors
    try:
        thread.start()
    except RuntimeError as error:
        with _lock:
            if _runtime_thread is thread:
                _runtime_thread = None
                _runtime_stop = None
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
        return Err("RobotTrack ROS runtime did not start within 5 seconds")
    if errors or not thread.is_alive():
        detail = errors[-1] if errors else "ROS runtime exited during startup"
        stop_event.set()
        thread.join(1.0)
        if not thread.is_alive():
            with _lock:
                if _runtime_thread is thread:
                    _runtime_thread = None
                    _runtime_stop = None
        return Err(f"RobotTrack ROS runtime failed: {detail}")
    with _lock:
        _active = True
    return Ok()


def _stop_runtime() -> tuple[bool, str]:
    global _active, _runtime_thread, _runtime_stop, _runtime_errors
    with _lock:
        thread = _runtime_thread
        stop_event = _runtime_stop
        _active = False
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


if __name__ == "__main__":
    provider.run()
