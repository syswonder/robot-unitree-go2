"""Best-effort, latest-frame camera preview uploads for RobotTrack.

This module is deliberately independent from the inference client.  A slow or
unavailable browser-preview endpoint therefore cannot delay ``/eval_dual`` or
change the image sent to the official model.
"""

from __future__ import annotations

from http.client import HTTPConnection, HTTPSConnection
import threading
import time
from typing import Callable, Mapping
from urllib.parse import urlsplit, urlunsplit


PreviewTransport = Callable[[str, bytes, Mapping[str, str], float], None]
ErrorCallback = Callable[[Exception], None]


def camera_frame_url(server_url: str) -> str:
    """Derive the sibling ``/api/camera-frame`` URL from ``/eval_dual``."""

    parsed = urlsplit(str(server_url).strip())
    if parsed.scheme not in {"http", "https"} or not parsed.hostname:
        raise ValueError("server_url must be an HTTP(S) URL")
    if parsed.username or parsed.password:
        raise ValueError("server_url must not contain credentials")
    suffix = "/eval_dual"
    if not parsed.path.endswith(suffix):
        raise ValueError("server_url must select the official /eval_dual endpoint")
    preview_path = parsed.path[: -len(suffix)] + "/api/camera-frame"
    return urlunsplit((parsed.scheme, parsed.netloc, preview_path, "", ""))


class CameraFrameHttpTransport:
    """Raw-JPEG POST transport whose active socket can be closed on shutdown."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._active: HTTPConnection | HTTPSConnection | None = None

    def __call__(
        self,
        url: str,
        jpeg: bytes,
        headers: Mapping[str, str],
        timeout_s: float,
    ) -> None:
        parsed = urlsplit(url)
        connection_type = HTTPSConnection if parsed.scheme == "https" else HTTPConnection
        connection = connection_type(parsed.hostname, parsed.port, timeout=timeout_s)
        target = parsed.path or "/"
        if parsed.query:
            target += f"?{parsed.query}"
        with self._lock:
            self._active = connection
        try:
            request_headers = dict(headers)
            request_headers["Content-Length"] = str(len(jpeg))
            connection.request("POST", target, body=jpeg, headers=request_headers)
            response = connection.getresponse()
            response.read()
            if not 200 <= int(response.status) < 300:
                raise RuntimeError(
                    f"RobotTrack camera preview returned HTTP {int(response.status)}"
                )
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


class CameraFrameUploadWorker:
    """Asynchronously upload only the newest full-frame JPEG at a bounded rate."""

    def __init__(
        self,
        server_url: str,
        *,
        max_hz: float = 5.0,
        timeout_s: float = 1.0,
        transport: PreviewTransport | None = None,
        on_error: ErrorCallback | None = None,
    ) -> None:
        if not 0.0 < float(max_hz) <= 30.0:
            raise ValueError("max_hz must be in (0, 30]")
        if not 0.0 < float(timeout_s) <= 10.0:
            raise ValueError("timeout_s must be in (0, 10]")
        self.endpoint = camera_frame_url(server_url)
        self._interval_s = 1.0 / float(max_hz)
        self._timeout_s = float(timeout_s)
        self._transport = transport or CameraFrameHttpTransport()
        self._on_error = on_error
        self._condition = threading.Condition()
        self._latest: tuple[int, bytes] | None = None
        self._stop = False
        self._started = False
        self._thread = threading.Thread(
            target=self._run,
            name="robottrack-camera-preview",
            daemon=True,
        )

    def start(self) -> None:
        with self._condition:
            if self._started:
                return
            self._started = True
        self._thread.start()

    def submit(self, sequence: int, jpeg: bytes) -> None:
        payload = bytes(jpeg)
        if not payload or not payload.startswith(b"\xff\xd8"):
            raise ValueError("camera preview must be a non-empty JPEG")
        with self._condition:
            if self._stop:
                return
            self._latest = (int(sequence), payload)
            self._condition.notify()

    def _run(self) -> None:
        next_send = 0.0
        while True:
            with self._condition:
                while self._latest is None and not self._stop:
                    self._condition.wait()
                if self._stop:
                    return
                sequence, payload = self._latest
                self._latest = None

            delay = next_send - time.monotonic()
            while delay > 0.0:
                with self._condition:
                    self._condition.wait(timeout=delay)
                    if self._stop:
                        return
                    # Prefer any newer frame that arrived while rate limiting.
                    if self._latest is not None:
                        sequence, payload = self._latest
                        self._latest = None
                delay = next_send - time.monotonic()

            try:
                self._transport(
                    self.endpoint,
                    payload,
                    {
                        "Content-Type": "image/jpeg",
                        "Accept": "application/json",
                        "X-Frame-Seq": str(sequence),
                    },
                    self._timeout_s,
                )
            except Exception as error:
                if self._on_error is not None:
                    try:
                        self._on_error(error)
                    except Exception:
                        # Preview diagnostics must not terminate the best-effort
                        # uploader if a logging callback itself fails.
                        pass
            next_send = time.monotonic() + self._interval_s

    def close(self, timeout_s: float = 2.0) -> None:
        with self._condition:
            self._stop = True
            self._condition.notify_all()
        close = getattr(self._transport, "close", None)
        if callable(close):
            close()
        if self._started:
            self._thread.join(timeout_s)
