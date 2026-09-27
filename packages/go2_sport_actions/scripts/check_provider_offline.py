# SPDX-License-Identifier: Apache-2.0
"""Import generated Robonix bindings and exercise non-motion Skill endpoints."""

import json
import os
from unittest.mock import patch

from go2_sport_actions.provider import (
    activate, execute_utterance, init, list_actions, preview,
)
from go2_sport_actions_mcp import (
    ExecuteUtterance_Request, ListActions_Request, PreviewAction_Request,
)


def main():
    init({})
    activate()
    listing = json.loads(list_actions(ListActions_Request()).catalog_json)
    assert listing["daemon_socket_configured"] is False
    assert all(item["name"] != "bow" for item in listing["actions"])
    bow = json.loads(preview(PreviewAction_Request(name="bow")).result_json)
    assert not bow["known"] and not bow["physical_plan_prepared"]
    result = execute_utterance(ExecuteUtterance_Request(text="请跳舞", request_id="offline"))
    assert result.action_name == "dance_1" and not result.accepted
    assert json.loads(result.message)["reason"] == "physical_backend_not_configured"
    negated = execute_utterance(ExecuteUtterance_Request(text="不要跳舞", request_id="offline-2"))
    assert not negated.accepted and not negated.action_name
    cancelled = execute_utterance(ExecuteUtterance_Request(text="停止动作", request_id="offline-3"))
    assert not cancelled.accepted and cancelled.action_name == "cancel"
    # A managed package stays read-only without the existing runtime operator
    # acknowledgement; this check never starts SDK2 or sends an action.
    with patch.dict(os.environ, {"GO2_OPERATOR_PRESENT": "",
                                 "GO2_STAGED_NAV2_RUNTIME_ACK": ""}):
        init({"backend": "managed", "daemon_socket": "/nonexistent/sport.sock",
              "network_interface": "not-a-robot", "daemon_binary": "/nonexistent/daemon"})
        denied = execute_utterance(ExecuteUtterance_Request(text="请跳舞", request_id="offline-4"))
        assert not denied.accepted and "acknowledgement" in denied.message
    print("provider offline smoke: PASS; no daemon connection or motion")


if __name__ == "__main__":
    main()
