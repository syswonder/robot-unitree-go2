from __future__ import annotations

import json
import unittest

from go2_robottrack.core import EncodedFrame, RuntimeConfig, VelocityCommand
from go2_robottrack.http_client import RobotTrackHttpClient, build_multipart_body


class HttpClientTests(unittest.TestCase):
    def test_multipart_contract_and_request_state_without_network(self) -> None:
        calls: list[tuple[str, bytes, dict[str, str], float]] = []

        def transport(url, body, headers, timeout):
            calls.append((url, body, dict(headers), timeout))
            return json.dumps(
                {
                    "waypoints": [[0.0, 0.0, 0.0] for _ in range(8)],
                    "base_velocity": [0.12, 0.0, -0.2],
                    "control_dt": 0.1,
                }
            ).encode("utf-8")

        client = RobotTrackHttpClient(RuntimeConfig.from_mapping({}), transport=transport)
        client.set_executed_command(VelocityCommand(0.02, 0.03))
        frame = EncodedFrame(1, b"\xff\xd8fake-jpeg\xff\xd9", 1.0, 2.0)
        first = client.evaluate(frame)
        second = client.evaluate(frame)

        self.assertEqual(first.command, VelocityCommand(0.12, -0.2))
        self.assertEqual(second.command, first.command)
        self.assertEqual(client.request_index, 2)
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[0][0], "http://127.0.0.1:5801/eval_dual")
        self.assertEqual(calls[0][3], 30.0)
        content_type = calls[0][2]["Content-Type"]
        boundary = content_type.split("boundary=", 1)[1]
        body = calls[0][1]
        self.assertIn(b'name="json"', body)
        self.assertIn(b'name="image"; filename="rgb_image.jpg"', body)
        self.assertIn(frame.jpeg, body)
        self.assertTrue(body.endswith(f"--{boundary}--\r\n".encode("ascii")))
        self.assertIn(b'"reset":true', body)
        self.assertIn(b'"idx":0', body)
        self.assertIn(b'"client_control_mode":"server_velocity"', body)
        self.assertIn(b'"client_exec_velocity":[0.02,0.0,0.03]', body)
        self.assertIn(b'"reset":false', calls[1][1])
        self.assertIn(b'"idx":1', calls[1][1])

    def test_invalid_response_does_not_advance_request_index(self) -> None:
        client = RobotTrackHttpClient(
            RuntimeConfig.from_mapping({}),
            transport=lambda *_args: b"not json",
        )
        with self.assertRaisesRegex(ValueError, "invalid JSON"):
            client.evaluate(EncodedFrame(1, b"jpeg", 1.0, 2.0))
        self.assertEqual(client.request_index, 0)

    def test_multipart_builder_rejects_empty_image(self) -> None:
        with self.assertRaisesRegex(ValueError, "must not be empty"):
            build_multipart_body({}, b"")

    def test_close_is_forwarded_to_cancellable_transport(self) -> None:
        class Transport:
            def __init__(self):
                self.closed = False

            def __call__(self, *_args):
                raise AssertionError("request is not expected")

            def close(self):
                self.closed = True

        transport = Transport()
        client = RobotTrackHttpClient(
            RuntimeConfig.from_mapping({}),
            transport=transport,
        )
        client.close()
        self.assertTrue(transport.closed)


if __name__ == "__main__":
    unittest.main()
