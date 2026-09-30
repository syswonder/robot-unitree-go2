# SPDX-License-Identifier: Apache-2.0
"""Robonix Skill endpoints backed by the existing single-owner chassis daemon."""
import os
from pathlib import Path
import signal
import stat
import subprocess
import threading
import time

from robonix_api import Err, Ok, Skill, Service
from go2_sport_actions.skills import RUNTIME_ID, RUNTIME_NAMESPACE
from go2_sport_actions import catalog as core
from go2_sport_actions.daemon_owner import ManagedDaemon
from go2_sport_actions.runtime import ActionManager, PLANS
from go2_sport_actions.voice import action_from_utterance, is_cancel_utterance
from go2_sport_actions_mcp import (
    ListActions_Response, PreviewAction_Response, ExecuteAction_Response,
    GetActionStatus_Response, CancelAction_Response,
    ListActions_Request, PreviewAction_Request, ExecuteAction_Request,
    GetActionStatus_Request, CancelAction_Request,
    ExecuteUtterance_Request, ExecuteUtterance_Response,
)

IS_RUNTIME = os.environ.get("GO2_SPORT_PROVIDER_ROLE") == "runtime"
NAMESPACE = RUNTIME_NAMESPACE if IS_RUNTIME else "robonix/skill/go2_sport_actions"
skill = (Service(id=RUNTIME_ID, namespace=NAMESPACE, md_path="") if IS_RUNTIME else
         Skill(id="go2_sport_actions", namespace=NAMESPACE))


def endpoint(leaf):
    # Legacy list/preview/voice remain available for old deployments only.
    if IS_RUNTIME and leaf not in {"execute", "status", "cancel"}:
        return lambda fn: fn
    if IS_RUNTIME:
        # Old Pilot discovers all MCP endpoints regardless of user_invocable.
        # Use internal typed gRPC so only the seven action Skills reach its tools.
        def internal(fn):
            import go2_sport_actions_pb2 as messages
            response_name = {"execute": "ExecuteAction_Response",
                             "status": "GetActionStatus_Response",
                             "cancel": "CancelAction_Response"}[leaf]
            def handler(request, _context):
                return getattr(messages, response_name)(**fn(request).to_dict())
            skill.grpc(f"{NAMESPACE}/{leaf}")(handler)
            return fn
        return internal
    return skill.mcp(f"{NAMESPACE}/{leaf}")
_manager = ActionManager()
_managed_daemon: ManagedDaemon | None = None
_daemon_process = None
_daemon_lock = threading.RLock()


def _stop_managed_daemon() -> bool:
    global _daemon_process
    process = _daemon_process
    if process is None or process.poll() is not None:
        _daemon_process = None
        return True
    for sig, timeout in ((signal.SIGINT, 3.0), (signal.SIGTERM, 2.0)):
        try:
            os.killpg(os.getpgid(process.pid), sig)
            process.wait(timeout=timeout)
            _daemon_process = None
            return True
        except ProcessLookupError:
            _daemon_process = None
            return True
        except subprocess.TimeoutExpired:
            continue
    return False


def _ensure_managed_daemon() -> str:
    global _daemon_process
    managed = _managed_daemon
    if managed is None:
        return ""
    with _daemon_lock:
        if _daemon_process is not None and _daemon_process.poll() is None:
            return ""
        if not managed.ready_to_activate():
            return "existing Go2 operator and staged-motion runtime acknowledgement is missing"
        if not managed.binary.is_file() or not os.access(managed.binary, os.X_OK):
            return f"Go2 sport daemon is not built: {managed.binary}"
        previous_inode = None
        try:
            previous_inode = managed.socket_path.stat().st_ino
        except FileNotFoundError:
            pass
        try:
            _daemon_process = skill.spawn(
                managed.argv(), env=managed.environment(), log="sdk-daemon.log",
                cwd=Path(__file__).resolve().parents[1])
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                if _daemon_process.poll() is not None:
                    _stop_managed_daemon()
                    return "Go2 sport daemon exited during startup; see sdk-daemon.log"
                try:
                    current = managed.socket_path.stat()
                except FileNotFoundError:
                    current = None
                if (current is not None and stat.S_ISSOCK(current.st_mode)
                        and current.st_ino != previous_inode):
                    return ""
                time.sleep(0.05)
        except OSError as exc:
            _stop_managed_daemon()
            return f"Go2 sport daemon startup failed: {exc}"
        _stop_managed_daemon()
        return "Go2 sport daemon did not create its IPC socket"


def _execute_action(name: str | None, request_id: str) -> dict:
    if not isinstance(name, str) or (name not in PLANS and name != "dance"):
        return _manager.execute(name, request_id)
    error = _ensure_managed_daemon()
    if error:
        return {"accepted": False, "run_id": "", "reason": error,
                "request_id": request_id, "motion_sent": False}
    return _manager.execute(name, request_id)


@endpoint("list")
def list_actions(_request: ListActions_Request) -> ListActions_Response:
    value = core.catalog()
    value["daemon_socket_configured"] = _manager.configured
    for action in value["actions"]:
        action["physical_plan_prepared"] = action["name"] in PLANS
        action["backend_configured"] = _manager.configured
    return ListActions_Response(catalog_json=core.dumps(value))


@endpoint("preview")
def preview(request: PreviewAction_Request) -> PreviewAction_Response:
    value = core.preview(request.name)
    value["physical_plan_prepared"] = request.name in PLANS
    value["daemon_socket_configured"] = _manager.configured
    return PreviewAction_Response(result_json=core.dumps(value))


@endpoint("execute")
def execute(request: ExecuteAction_Request) -> ExecuteAction_Response:
    """Run one canonical Go2 action: dance_1, dance_2, new_year_greeting,
    crouch, handstand, handstand_walk, stretch, or hello.
    SDK acknowledgement is not proof of physical completion;
    use status and the live camera/operator view.
    """
    value = _execute_action(request.name, request.request_id)
    return ExecuteAction_Response(accepted=value["accepted"], run_id=value["run_id"],
                                  message=core.dumps(value))


@endpoint("execute_utterance")
def execute_utterance(request: ExecuteUtterance_Request) -> ExecuteUtterance_Response:
    """For explicit Chinese Go2 stunt requests such as ‘跳舞’, ‘拜年’,
    ‘倒立’, ‘倒立向前走’, ‘伸展一下’, or ‘打个招呼’, map the speech to a named
    action and dispatch through the same chassis owner. ‘停止动作’ or ‘停下’
    requests cancellation; it does not claim the chassis already stopped.
    Never treat ‘不要跳舞’ or unrelated navigation speech as an action.
    """
    if is_cancel_utterance(request.text):
        value = _manager.cancel_active()
        return ExecuteUtterance_Response(
            accepted=value["ok"], run_id=value["run_id"], action_name="cancel",
            message=core.dumps(value))
    name = action_from_utterance(request.text)
    value = (_execute_action(name, request.request_id) if name else
             {"accepted": False, "run_id": "", "reason": "utterance_not_an_action",
              "request_id": request.request_id, "motion_sent": False})
    return ExecuteUtterance_Response(accepted=value["accepted"], run_id=value["run_id"],
                                     action_name=name or "", message=core.dumps(value))


@endpoint("status")
def status(request: GetActionStatus_Request) -> GetActionStatus_Response:
    return GetActionStatus_Response(result_json=core.dumps(_manager.status(request.run_id)))


@endpoint("cancel")
def cancel(request: CancelAction_Request) -> CancelAction_Response:
    value = _manager.cancel(request.run_id) if request.run_id else _manager.cancel_active()
    return CancelAction_Response(ok=value["ok"], message=core.dumps(value))


@skill.on_init
def init(config):
    global _manager, _managed_daemon
    if config and not isinstance(config, dict):
        return Err("configuration must be an object")
    config = config or {}
    path = str(config.get("daemon_socket") or
               os.environ.get("GO2_SPORT_DAEMON_SOCKET", "")).strip()
    if path and not os.path.isabs(path):
        return Err("daemon_socket must be an absolute Unix socket path")
    backend = str(config.get("backend") or ("external" if path else "catalog"))
    if backend not in {"catalog", "external", "managed"}:
        return Err("backend must be catalog, external or managed")
    if backend == "external" and not path:
        return Err("external backend requires daemon_socket")
    try:
        replacement = ActionManager(path if backend != "catalog" else "",
            dance_variants=config.get("dance_variants", ("dance_1", "dance_2")))
        managed = (ManagedDaemon.from_config(config, path)
                   if backend == "managed" else None)
    except (ValueError, TypeError) as exc:
        return Err(str(exc))
    _manager.shutdown()
    _stop_managed_daemon()
    _managed_daemon = managed
    _manager = replacement
    return Ok()


@skill.on_activate
def activate():
    return Ok()  # Read-only list/preview remain available before live action.


@skill.on_deactivate
def deactivate():
    _manager.shutdown()
    return Ok() if _stop_managed_daemon() else Err("Go2 sport daemon did not exit")


@skill.on_shutdown
def shutdown():
    _manager.shutdown()
    return Ok() if _stop_managed_daemon() else Err("Go2 sport daemon did not exit")


if __name__ == "__main__":
    skill.run()
