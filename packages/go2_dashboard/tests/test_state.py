from __future__ import annotations

import math
import unittest

from go2_dashboard.state import DashboardState


class _Clock:
    def __init__(self, value: float) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value


class StateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.monotonic = _Clock(10.0)
        self.wall = _Clock(1_700_000_000.0)
        self.state = DashboardState(
            monotonic_fn=self.monotonic, wall_fn=self.wall
        )

    def test_topic_transitions_from_missing_to_fresh_to_stale(self) -> None:
        self.assertEqual(self.state.snapshot()["topics"]["odom"]["state"], "missing")
        self.state.observe("odom", {"position": {"x": 0.0}}, frame_id="odom")
        current = self.state.snapshot()["topics"]["odom"]
        self.assertEqual(current["state"], "fresh")
        self.assertEqual(current["sequence"], 1)
        self.monotonic.value = 12.0
        self.assertEqual(self.state.snapshot()["topics"]["odom"]["state"], "stale")

    def test_parser_error_is_visible_and_next_frame_clears_it(self) -> None:
        self.state.note_error("camera", "unsupported encoding")
        topic = self.state.snapshot()["topics"]["camera"]
        self.assertEqual(topic["state"], "error")
        self.assertIn("unsupported", topic["error"])
        self.state.set_camera(b"jpeg", {"width": 1, "height": 1}, frame_id="camera")
        self.assertEqual(self.state.snapshot()["topics"]["camera"]["state"], "fresh")

    def test_binary_previews_are_not_embedded_in_json_snapshot(self) -> None:
        self.state.set_camera(b"jpeg-data", {"width": 1}, frame_id="camera")
        self.state.set_map(b"png-data", {"width": 2, "height": 2}, frame_id="map")
        snapshot = self.state.snapshot()
        self.assertNotIn(b"jpeg-data", snapshot.values())
        self.assertEqual(self.state.camera_image(), (b"jpeg-data", 1))
        self.assertEqual(self.state.map_image(), (b"png-data", 1))

    def test_camera_quality_is_exposed_without_binary_data(self) -> None:
        self.state.set_camera_quality(
            {
                "rate_hz": 1.88,
                "quality_error_ratio": 0.25,
                "healthy": False,
                "message": "x" * 500,
                "api_code_semantics": "opaque vendor return code; not interpreted",
            }
        )
        quality = self.state.snapshot()["camera_quality"]
        self.assertEqual(quality["rate_hz"], 1.88)
        self.assertFalse(quality["healthy"])
        self.assertEqual(len(quality["message"]), 240)
        self.assertIn("not interpreted", quality["api_code_semantics"])

    def test_semantic_status_update_is_metadata_only_and_bounded(self) -> None:
        task = self.state.update_semantic_task(
            {
                "task_id": "task-1",
                "target_name": "自动售货机",
                "status": "resolved",
                "message": "matched",
                "pose": {"frame_id": "map", "x": 1, "y": 2, "yaw": 0.3},
            }
        )
        self.assertTrue(task["read_only_effect"])
        self.assertEqual(task["status"], "resolved")
        self.assertEqual(task["revision"], 1)
        self.assertEqual(task["pose"]["x"], 1.0)
        with self.assertRaisesRegex(ValueError, "unsupported"):
            self.state.update_semantic_task({"status": "execute"})
        with self.assertRaisesRegex(ValueError, "non-finite"):
            self.state.update_semantic_task(
                {
                    "status": "resolved",
                    "pose": {"x": math.inf, "y": 0, "yaw": 0},
                }
            )

    def test_bridge_error_is_bounded(self) -> None:
        self.state.set_bridge(running=False, connected=False, error="x" * 1000)
        self.assertEqual(len(self.state.snapshot()["bridge"]["error"]), 400)

    def test_voice_state_is_disabled_by_default_and_session_scoped(self) -> None:
        voice = self.state.snapshot()["voice"]
        self.assertFalse(voice["enabled"])
        self.assertEqual(voice["status"], "disabled")
        self.assertFalse(voice["direct_robot_control"])
        self.state.configure_voice(True, {"max_duration_s": 8.0})
        current = self.state.update_voice(
            session_id="a" * 32,
            status="recognized",
            message="ok",
            transcript="x" * 500,
            active=True,
        )
        self.assertEqual(len(current["transcript"]), 300)
        with self.assertRaisesRegex(ValueError, "stale"):
            self.state.update_voice(
                session_id="b" * 32,
                status="pilot",
                message="wrong session",
                active=True,
            )
        self.state.update_voice(
            session_id="a" * 32,
            status="completed",
            message="done",
            active=False,
        )
        replacement = self.state.update_voice(
            session_id="b" * 32,
            status="accepted",
            message="next",
            active=True,
        )
        self.assertEqual(replacement["session_id"], "b" * 32)


if __name__ == "__main__":
    unittest.main()
