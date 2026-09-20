from __future__ import annotations

import threading
import unittest

from go2_robottrack.camera_preview import CameraFrameUploadWorker, camera_frame_url


class CameraPreviewTests(unittest.TestCase):
    def test_camera_endpoint_is_derived_from_inference_server(self) -> None:
        self.assertEqual(
            camera_frame_url("http://127.0.0.1:5801/eval_dual"),
            "http://127.0.0.1:5801/api/camera-frame",
        )
        self.assertEqual(
            camera_frame_url("https://example.test/robot/eval_dual?ignored=yes"),
            "https://example.test/robot/api/camera-frame",
        )

    def test_worker_posts_raw_jpeg_with_sequence_header(self) -> None:
        calls = []
        posted = threading.Event()

        def transport(url, jpeg, headers, timeout_s):
            calls.append((url, jpeg, dict(headers), timeout_s))
            posted.set()

        worker = CameraFrameUploadWorker(
            "http://127.0.0.1:5801/eval_dual",
            max_hz=30.0,
            timeout_s=0.25,
            transport=transport,
        )
        worker.start()
        worker.submit(17, b"\xff\xd8full-frame\xff\xd9")
        self.assertTrue(posted.wait(1.0))
        worker.close()

        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][0], "http://127.0.0.1:5801/api/camera-frame")
        self.assertEqual(calls[0][1], b"\xff\xd8full-frame\xff\xd9")
        self.assertEqual(calls[0][2]["Content-Type"], "image/jpeg")
        self.assertEqual(calls[0][2]["X-Frame-Seq"], "17")
        self.assertEqual(calls[0][3], 0.25)

    def test_worker_rejects_non_jpeg_without_calling_transport(self) -> None:
        calls = []
        worker = CameraFrameUploadWorker(
            "http://127.0.0.1:5801/eval_dual",
            transport=lambda *args: calls.append(args),
        )
        with self.assertRaisesRegex(ValueError, "non-empty JPEG"):
            worker.submit(1, b"not-a-jpeg")
        worker.close()
        self.assertEqual(calls, [])


if __name__ == "__main__":
    unittest.main()
