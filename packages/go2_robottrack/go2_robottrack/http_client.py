"""Standard-library client for the official MiniCPM-RobotTrack ``/eval_dual`` API."""

from __future__ import annotations

import json
from http.client import HTTPConnection, HTTPSConnection
import threading
import time
from typing import Any, Callable, Mapping
from urllib.parse import urlsplit
import uuid

from .core import EncodedFrame, InferencePlan, RuntimeConfig, VelocityCommand, parse_inference_response


Transport = Callable[[str, bytes, Mapping[str, str], float], bytes]


def build_multipart_body(
    metadata: Mapping[str, Any],
    jpeg: bytes,
    *,
    boundary: str | None = None,
) -> tuple[bytes, str]:
    """Build the official ``json`` form field plus ``image`` JPEG part."""

    payload = bytes(jpeg)
    if not payload:
        raise ValueError("jpeg must not be empty")
    marker = boundary or f"robottrack-{uuid.uuid4().hex}"
    if not marker or any(character in marker for character in "\r\n\""):
        raise ValueError("multipart boundary contains invalid characters")
    json_bytes = json.dumps(
        dict(metadata), separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")
    prefix = (
        f"--{marker}\r\n"
        'Content-Disposition: form-data; name="json"\r\n'
        "Content-Type: application/json; charset=utf-8\r\n\r\n"
    ).encode("ascii")
    image_header = (
        f"\r\n--{marker}\r\n"
        'Content-Disposition: form-data; name="image"; filename="rgb_image.jpg"\r\n'
        "Content-Type: image/jpeg\r\n\r\n"
    ).encode("ascii")
    suffix = f"\r\n--{marker}--\r\n".encode("ascii")
    return prefix + json_bytes + image_header + payload + suffix, marker


class HttpTransport:
    """One-request transport whose active socket can be closed on deactivate."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._active: HTTPConnection | HTTPSConnection | None = None

    def __call__(
        self,
        url: str,
        body: bytes,
        headers: Mapping[str, str],
        timeout_s: float,
    ) -> bytes:
        parsed = urlsplit(url)
        connection_type = HTTPSConnection if parsed.scheme == "https" else HTTPConnection
        connection = connection_type(
            parsed.hostname,
            parsed.port,
            timeout=timeout_s,
        )
        target = parsed.path or "/"
        if parsed.query:
            target += f"?{parsed.query}"
        with self._lock:
            self._active = connection
        try:
            request_headers = dict(headers)
            request_headers["Content-Length"] = str(len(body))
            connection.request("POST", target, body=body, headers=request_headers)
            response = connection.getresponse()
            payload = response.read()
            if not 200 <= int(response.status) < 300:
                raise RuntimeError(
                    f"RobotTrack server returned HTTP {int(response.status)}"
                )
            return payload
        finally:
            with self._lock:
                if self._active is connection:
                    self._active = None
            connection.close()

    def close(self) -> None:
        with self._lock:
            connection = self._active
        if connection is not None:
            connection.close()


class RobotTrackHttpClient:
    """Stateful `/eval_dual` client matching the official request metadata."""

    def __init__(self, config: RuntimeConfig, *, transport: Transport | None = None) -> None:
        self._config = config
        self._transport = transport or HttpTransport()
        self._lock = threading.Lock()
        self._index = 0
        self._reset = True
        self._executed = VelocityCommand()

    def set_executed_command(self, command: VelocityCommand) -> None:
        with self._lock:
            self._executed = command.clipped(self._config.max_vx, self._config.max_wz)

    def _metadata(self) -> dict[str, Any]:
        with self._lock:
            index = self._index
            reset = self._reset
            executed = self._executed
        return {
            "reset": reset,
            "idx": index,
            "instruction": self._config.instruction,
            "client_control_mode": "server_velocity",
            "client_exec_velocity": [executed.vx, 0.0, executed.wz],
            "client_send_timestamp": time.time(),
        }

    def evaluate(self, frame: EncodedFrame) -> InferencePlan:
        metadata = self._metadata()
        body, boundary = build_multipart_body(metadata, frame.jpeg)
        response_bytes = self._transport(
            self._config.server_url,
            body,
            {
                "Content-Type": f"multipart/form-data; boundary={boundary}",
                "Accept": "application/json",
            },
            self._config.http_timeout_s,
        )
        try:
            decoded = json.loads(response_bytes.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ValueError(f"RobotTrack server returned invalid JSON: {error}") from error
        plan = parse_inference_response(
            decoded,
            max_vx=self._config.max_vx,
            max_wz=self._config.max_wz,
        )
        with self._lock:
            self._index += 1
            self._reset = False
        return plan

    @property
    def request_index(self) -> int:
        with self._lock:
            return self._index

    def close(self) -> None:
        close = getattr(self._transport, "close", None)
        if callable(close):
            close()
