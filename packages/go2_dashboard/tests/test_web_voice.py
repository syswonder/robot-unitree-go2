from __future__ import annotations

import io
import struct
import threading
import unittest
import wave

try:
    import fastapi  # noqa: F401 - availability gate for the first build pass
    import httpx
except ImportError:  # First offline pass intentionally has no PyPI imports.
    httpx = None  # type: ignore[assignment]


def _wav(seconds: float = 0.4) -> bytes:
    frames = int(16_000 * seconds)
    with io.BytesIO() as output:
        with wave.open(output, "wb") as target:
            target.setnchannels(1)
            target.setsampwidth(2)
            target.setframerate(16_000)
            target.writeframes(struct.pack("<h", 900) * frames)
        return output.getvalue()


@unittest.skipUnless(httpx is not None, "FastAPI HTTP test dependencies not installed")
class VoiceHttpTests(unittest.IsolatedAsyncioTestCase):
    def _client(self, *, enabled: bool, client_host: str = "127.0.0.1"):
        from go2_dashboard.state import DashboardState
        from go2_dashboard.voice_gateway import BrowserVoiceGateway, VoiceConfig
        from go2_dashboard.web import create_app

        state = DashboardState()
        ran = threading.Event()

        def runner(config, session_id, pcm, duration, update, cancel):
            del config, session_id, pcm, duration, cancel
            update(status="completed", message="offline test completed")
            ran.set()

        gateway = BrowserVoiceGateway(
            state, VoiceConfig(enabled=enabled), runner=runner
        )
        app = create_app(state=state, voice_gateway=gateway)
        transport = httpx.ASGITransport(
            app=app,
            client=(client_host, 41000),
        )
        client = httpx.AsyncClient(
            transport=transport,
            base_url="http://127.0.0.1:8092",
        )
        self.addAsyncCleanup(client.aclose)
        return client, gateway, ran

    @staticmethod
    def _headers(nonce: str) -> dict[str, str]:
        return {
            "Origin": "http://127.0.0.1:8092",
            "Sec-Fetch-Site": "same-origin",
            "Content-Type": "audio/wav",
            "X-Go2-Voice-Nonce": nonce,
        }

    async def test_disabled_post_is_hidden_and_ui_is_not_globally_read_only_when_enabled(self) -> None:
        disabled, _, _ = self._client(enabled=False)
        self.assertEqual(
            (await disabled.post("/api/voice", content=_wav())).status_code, 404
        )
        self.assertTrue((await disabled.get("/api/status")).json()["read_only"])

        enabled, _, _ = self._client(enabled=True)
        status = (await enabled.get("/api/status")).json()
        self.assertFalse(status["read_only"])
        self.assertTrue(status["telemetry_read_only"])

    async def test_post_rejects_non_loopback_forwarding_and_bad_nonce(self) -> None:
        remote, _, _ = self._client(enabled=True, client_host="192.168.1.50")
        nonce = (await remote.get("/api/voice")).json()["csrf_nonce"]
        response = await remote.post(
            "/api/voice", content=_wav(), headers=self._headers(nonce)
        )
        self.assertEqual(response.status_code, 403)

        local, _, _ = self._client(enabled=True)
        response = await local.post(
            "/api/voice", content=_wav(), headers=self._headers("wrong")
        )
        self.assertEqual(response.status_code, 403)
        nonce = (await local.get("/api/voice")).json()["csrf_nonce"]
        headers = self._headers(nonce)
        headers["Forwarded"] = "for=127.0.0.1"
        self.assertEqual(
            (await local.post("/api/voice", content=_wav(), headers=headers)).status_code,
            403,
        )

    async def test_post_requires_length_mime_and_enforces_declared_size(self) -> None:
        client, _, _ = self._client(enabled=True)
        nonce = (await client.get("/api/voice")).json()["csrf_nonce"]
        headers = self._headers(nonce)
        headers["Content-Length"] = "not-an-integer"
        self.assertEqual(
            (await client.post("/api/voice", content=_wav(), headers=headers)).status_code,
            411,
        )

        nonce = (await client.get("/api/voice")).json()["csrf_nonce"]
        headers = self._headers(nonce)
        headers["Content-Length"] = "300001"
        self.assertEqual(
            (await client.post("/api/voice", content=b"small", headers=headers)).status_code,
            413,
        )

        nonce = (await client.get("/api/voice")).json()["csrf_nonce"]
        headers = self._headers(nonce)
        headers["Content-Type"] = "audio/webm"
        self.assertEqual(
            (await client.post("/api/voice", content=b"not-wav", headers=headers)).status_code,
            415,
        )

    async def test_valid_post_is_accepted_once_and_consumes_nonce(self) -> None:
        client, gateway, ran = self._client(enabled=True)
        nonce = (await client.get("/api/voice")).json()["csrf_nonce"]
        response = await client.post(
            "/api/voice", content=_wav(), headers=self._headers(nonce)
        )
        self.assertEqual(response.status_code, 202, response.text)
        self.assertTrue(ran.wait(timeout=1.0))
        self.assertFalse(gateway.verify_nonce(nonce))


if __name__ == "__main__":
    unittest.main()
