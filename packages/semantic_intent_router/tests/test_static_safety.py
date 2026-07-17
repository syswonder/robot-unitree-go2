from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class StaticSafetyTests(unittest.TestCase):
    def test_router_has_no_ros_or_unitree_command_surface(self) -> None:
        source = (ROOT / "semantic_intent_router" / "server.py").read_text(
            encoding="utf-8"
        )
        forbidden = (
            "import rclpy",
            "from rclpy",
            "unitree_sdk",
            "SportClient",
            "create_publisher",
            "/api/sport/request",
            "/lowcmd",
            "/cmd_vel",
        )
        for token in forbidden:
            self.assertNotIn(token, source)

    def test_start_wrapper_does_not_enable_motion(self) -> None:
        source = (ROOT / "scripts" / "start.sh").read_text(encoding="utf-8")
        self.assertNotIn("GO2_ALLOW_MOTION", source)
        self.assertNotIn("sudo", source)
        self.assertNotIn("ros2 ", source)


if __name__ == "__main__":
    unittest.main()
