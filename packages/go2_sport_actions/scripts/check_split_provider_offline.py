# SPDX-License-Identifier: Apache-2.0
"""Actual generated handlers with fake transport; no Atlas or hardware calls."""
import asyncio
import json
from unittest.mock import AsyncMock
from go2_sport_actions.action_provider import ActionSkill
from go2_sport_actions.skills import DEFINITIONS
from go2_sport_actions_mcp import PerformAction_Request, GetActionStatus_Request, CancelAction_Request


async def main():
    for definition in DEFINITIONS:
        provider = ActionSkill(definition["action"])
        provider.init(definition["init"])
        assert provider.initialized
        assert len(provider.skill._mcp_handlers) == 3
        provider._call = AsyncMock(return_value={"accepted": True, "run_id": "fake", "message": "{}"})
        result = await provider.execute(PerformAction_Request(request_id="offline"))
        assert result.accepted
        provider._call.assert_awaited_once_with("execute", {"name": definition["action"], "request_id": "offline"})
        provider._call = AsyncMock(return_value={"result_json": '{"state":"UNKNOWN"}'})
        assert "UNKNOWN" in (await provider.status(GetActionStatus_Request(run_id="fake"))).result_json
        provider._call = AsyncMock(return_value={"ok": False, "message": "not active"})
        assert not (await provider.cancel(CancelAction_Request(run_id="fake"))).ok
        provider._call = AsyncMock(side_effect=RuntimeError("transport lost"))
        uncertain = await provider.execute(PerformAction_Request(request_id="uncertain"))
        assert json.loads(uncertain.message)["reason"] == "dispatch_unconfirmed"
        provider._call.assert_awaited_once()  # no implicit retry
        uninitialized = ActionSkill(definition["action"])
        uninitialized.init({"action": "wrong"})
        assert not uninitialized.initialized
        print(definition["provider_id"], "generated schema + initialization + delegation: PASS")


if __name__ == "__main__":
    asyncio.run(main())
