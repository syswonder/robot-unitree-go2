# SPDX-License-Identifier: Apache-2.0
"""Loopback OpenAI-compatible Pilot router for explicit Go2 sport utterances.

The router never calls the robot. It emits one typed Robonix Skill leaf for a
new user turn and no further motion leaf when Executor feeds that result back.
"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import time
from typing import Any

from .voice import action_from_utterance, is_cancel_utterance
from .skills import BY_ACTION


MODEL = "go2-sport-router"
# Pilot qualifies MCP tools as provider_id + penultimate/last contract ID
# segments, e.g. go2_sport_actions.go2_sport_actions_execute_utterance.
def capability_for(action: str, leaf: str = "execute") -> str:
    identity = BY_ACTION[action]["provider_id"]
    return f"{identity}.{identity}_{leaf}"
FEEDBACK_PREFIX = "Executor feedback for the current RTDL leaf (not a new user request): "
MAX_BODY = 2 * 1024 * 1024


def _text(message: dict[str, Any]) -> str:
    content = message.get("content", "")
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        return "\n".join(part.get("text", "") for part in content
                         if isinstance(part, dict) and isinstance(part.get("text"), str))
    return ""


def _feedback_acceptance(messages: list[dict[str, Any]]) -> bool | None:
    """Read the Skill's typed acceptance, not Executor's transport success."""
    decoder = json.JSONDecoder()
    for message in reversed(messages):
        raw = _text(message)
        if not raw.startswith(FEEDBACK_PREFIX):
            continue
        raw = raw[len(FEEDBACK_PREFIX):]
        for offset, char in enumerate(raw):
            if char not in "{[":
                continue
            try:
                root, _ = decoder.raw_decode(raw[offset:])
            except json.JSONDecodeError:
                continue
            pending = [root]
            while pending:
                node = pending.pop()
                if isinstance(node, dict):
                    contract = node.get("contract_id") or node.get("contractId")
                    if isinstance(contract, str) and contract.endswith(("/execute_utterance", "/execute")):
                        output = node.get("output")
                        if isinstance(output, str):
                            try:
                                output = json.loads(output)
                            except json.JSONDecodeError:
                                output = None
                        if isinstance(output, dict) and isinstance(output.get("accepted"), bool):
                            return output["accepted"]
                    pending.extend(node.values())
                elif isinstance(node, list):
                    pending.extend(node)
    return None


def decide(messages: list[dict[str, Any]]) -> dict[str, Any]:
    command = ""
    feedback = False
    for message in messages:
        if message.get("role") != "user":
            continue
        value = _text(message).strip()
        if value.startswith(FEEDBACK_PREFIX):
            feedback = True
        else:
            command = value
            feedback = False

    tree: dict[str, Any] = {
        "op": "sequence", "op_id": 0,
        "description": "no new sport action", "children": [],
    }
    if feedback:
        accepted = _feedback_acceptance(messages)
        if accepted is False:
            content = "动作 Skill 未接受请求，机器狗未因本次指令开始动作；请检查运行状态。"
        elif accepted is True:
            content = "动作 Skill 已受理请求；SDK 回执不代表实机动作完成，请观察机器狗。"
        else:
            content = "未能确认动作 Skill 是否受理；请检查状态，勿当作实机动作成功。"
    elif is_cancel_utterance(command):
        content = "正在通过 Robonix Skill 请求停止当前动作，请确认机器狗已经停止。"
    elif action_from_utterance(command) is not None:
        content = "已识别动作指令，正交给 Go2 动作 Skill；请观察实际执行情况。"
    else:
        content = "未识别到单一明确的 Go2 动作指令，未下发动作。"

    should_call = not feedback and (is_cancel_utterance(command) or
                                   action_from_utterance(command) is not None)
    if should_call:
        tree = {
            "op": "sequence", "op_id": 0,
            "description": "dispatch one Unitree Go2 skill",
            "children": [{
                "op": "do", "op_id": 0,
                "description": "Go2 named sport action through Skill",
                "cap": (capability_for("dance", "cancel") if is_cancel_utterance(command)
                        else capability_for(action_from_utterance(command))),
                "args": ({"run_id": ""} if is_cancel_utterance(command) else {"request_id": ""}),
            }],
        }
    return {"content": content, "rtdl_description": tree["description"],
            "rtdl": tree, "task_update": None}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, _format: str, *_args: Any) -> None:
        # Utterances and model prompts must not enter the web-server log.
        pass

    def _json(self, status: int, body: dict[str, Any]) -> None:
        payload = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:  # noqa: N802
        if self.path.rstrip("/").endswith("/models"):
            self._json(200, {"object": "list", "data": [{"id": MODEL, "object": "model"}]})
        else:
            self.send_error(404)

    def do_POST(self) -> None:  # noqa: N802
        if not self.path.rstrip("/").endswith("/chat/completions"):
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_error(400)
            return
        if not 0 < length <= MAX_BODY:
            self.send_error(413)
            return
        try:
            request = json.loads(self.rfile.read(length))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self.send_error(400)
            return
        messages = request.get("messages")
        if not isinstance(messages, list) or not all(isinstance(m, dict) for m in messages):
            self.send_error(400)
            return
        envelope = decide(messages)
        content = json.dumps(envelope, ensure_ascii=False)
        model = str(request.get("model") or MODEL)
        base = {"id": "go2-sport-router", "created": int(time.time()), "model": model}
        if not request.get("stream", False):
            self._json(200, dict(base, object="chat.completion", choices=[{
                "index": 0, "message": {"role": "assistant", "content": content},
                "finish_reason": "stop",
            }]))
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        for delta, finish in (({"role": "assistant", "content": content}, None), ({}, "stop")):
            chunk = dict(base, object="chat.completion.chunk", choices=[{
                "index": 0, "delta": delta, "finish_reason": finish,
            }])
            self.wfile.write(f"data: {json.dumps(chunk, ensure_ascii=False)}\n\n".encode())
            self.wfile.flush()
        self.wfile.write(b"data: [DONE]\n\n")
        self.wfile.flush()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18081)
    args = parser.parse_args()
    if args.host != "127.0.0.1" or not 1 <= args.port <= 65535:
        parser.error("sport router must bind to IPv4 loopback on a valid port")
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
