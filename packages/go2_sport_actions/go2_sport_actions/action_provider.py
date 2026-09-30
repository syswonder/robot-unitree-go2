# SPDX-License-Identifier: Apache-2.0
"""Seven independently registered Skills; one shared physical action owner.

Each configured process declares only its own execute/status/cancel contracts.
The action is deployment data, never a model-supplied SDK name or dance number.
"""
import asyncio
import json
from pathlib import Path
import sys
import uuid

from robonix_api import ATLAS, Err, Ok, Skill
from go2_sport_actions.skills import BY_ACTION, RUNTIME_ID, RUNTIME_NAMESPACE
from go2_sport_actions_mcp import (
    PerformAction_Request, PerformAction_Response,
    GetActionStatus_Request, GetActionStatus_Response,
    CancelAction_Request, CancelAction_Response,
)


class ActionSkill:
    def __init__(self, action: str):
        self.definition = BY_ACTION[action]
        self.action = action
        self.runtime_id = RUNTIME_ID
        self.initialized = False
        root = Path(__file__).resolve().parents[1]
        self.skill = Skill(id=self.definition["provider_id"],
                           namespace=self.definition["namespace"], pkg_root=root,
                           md_path=str(root / "docs" / "skills" / f"{action}.md"))

        # Robonix attaches schema metadata to functions, not bound methods.
        async def execute(request: PerformAction_Request) -> PerformAction_Response:
            return await self.execute(request)

        async def status(request: GetActionStatus_Request) -> GetActionStatus_Response:
            return await self.status(request)

        async def cancel(request: CancelAction_Request) -> CancelAction_Response:
            return await self.cancel(request)

        self.skill.mcp(f'{self.definition["namespace"]}/execute',
                       description=self.definition["description"])(execute)
        self.skill.mcp(f'{self.definition["namespace"]}/status',
                       description=f'查询{self.definition["label"]}任务状态；受理不代表物理完成。')(status)
        self.skill.mcp(f'{self.definition["namespace"]}/cancel',
                       description='取消宇树 Go2 当前动作；run_id 为空时取消当前共享运行，操作者仍需确认停止。')(cancel)
        self.skill.on_init(self.init)
        self.skill.on_activate(lambda: Ok() if self.initialized else Err("not initialized"))
        # This wrapper never owns a daemon; tearing down an unrelated wrapper
        # must not stop another action. The runtime owns stop/cancel/watchdog.
        self.skill.on_deactivate(lambda: Ok())
        self.skill.on_shutdown(lambda: Ok())

    def init(self, config):
        if not isinstance(config, dict) or config.get("action") != self.action:
            return Err(f"action configuration must match this Skill: {self.action}")
        runtime_id = config.get("runtime_provider_id", RUNTIME_ID)
        if not isinstance(runtime_id, str) or not runtime_id.strip():
            return Err("runtime_provider_id must be a non-empty provider id")
        self.runtime_id = runtime_id.strip()
        self.initialized = True
        return Ok()

    async def _call(self, leaf: str, arguments: dict) -> dict:
        if not self.initialized:
            raise RuntimeError("Skill is not initialized")
        import grpc
        import go2_sport_actions_pb2 as messages
        import robonix_contracts_pb2_grpc as contracts
        contract = f"{RUNTIME_NAMESPACE}/{leaf}"
        # Resolve the configured service specifically, never an arbitrary Go2.
        channel = await asyncio.to_thread(ATLAS.connect_capability,
            consumer_id=self.skill.id, contract_id=contract,
            provider_id=self.runtime_id, transport="grpc")
        try:
            service, method = {
                "execute": ("Execute", "ExecuteAction"),
                "status": ("Status", "GetActionStatus"),
                "cancel": ("Cancel", "CancelAction"),
            }[leaf]
            # This is an IPC request timeout, never a robot StopMove timer.
            # The background runtime continues to own the accepted action.
            def invoke():
                with grpc.insecure_channel(channel.endpoint,
                        options=(("grpc.enable_http_proxy", 0),)) as wire:
                    stub = getattr(contracts, f"RobonixServiceUnitreeGo2SportRuntime{service}Stub")(wire)
                    request = getattr(messages, f"{method}_Request")(**arguments)
                    result = getattr(stub, method)(request, timeout=15)
                    return {field.name: getattr(result, field.name)
                            for field in result.DESCRIPTOR.fields}
            return await asyncio.to_thread(invoke)
        finally:
            await asyncio.to_thread(channel.close)

    async def execute(self, request: PerformAction_Request) -> PerformAction_Response:
        request_id = request.request_id or str(uuid.uuid4())
        try:
            value = await self._call("execute", {"name": self.action, "request_id": request_id})
            return PerformAction_Response(**value)
        except Exception as exc:
            # Transport failure can occur AFTER dispatch. Never retry implicitly
            # or tell the operator the robot did not move in this case.
            return PerformAction_Response(accepted=False, run_id="", message=json.dumps({
                "reason": "dispatch_unconfirmed", "detail": str(exc),
                "request_id": request_id, "physical_completion_verified": False,
            }, ensure_ascii=False))

    async def status(self, request: GetActionStatus_Request) -> GetActionStatus_Response:
        """查询动作受理及执行状态；不把 SDK 回执表述为物理完成。"""
        value = await self._call("status", {"run_id": request.run_id})
        return GetActionStatus_Response(**value)

    async def cancel(self, request: CancelAction_Request) -> CancelAction_Response:
        """请求停止共享执行器中的动作，随后查询状态并确认实际姿态。"""
        value = await self._call("cancel", {"run_id": request.run_id})
        return CancelAction_Response(**value)


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in BY_ACTION:
        raise SystemExit("Select one configured action from skills.json")
    ActionSkill(sys.argv[1]).skill.run()


if __name__ == "__main__":
    main()
