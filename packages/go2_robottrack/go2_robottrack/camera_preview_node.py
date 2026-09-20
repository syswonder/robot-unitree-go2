"""ROS-only subscriber that feeds the RobotTrack raw-camera monitor stream."""

from __future__ import annotations

import argparse
import time
from typing import Sequence

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .camera_preview import CameraFrameUploadWorker


class CameraPreviewBridge(Node):
    """Rate-limit, encode, and asynchronously upload uncropped RGB frames."""

    def __init__(
        self,
        *,
        rgb_topic: str,
        server_url: str,
        max_hz: float,
        jpeg_quality: int,
        timeout_s: float,
    ) -> None:
        super().__init__("go2_robottrack_camera_preview")
        self._bridge = CvBridge()
        self._jpeg_quality = int(jpeg_quality)
        self._min_frame_interval_s = 1.0 / float(max_hz)
        self._next_frame_at = 0.0
        self._sequence = 0
        self._last_error_log = 0.0
        self._uploader = CameraFrameUploadWorker(
            server_url,
            max_hz=max_hz,
            timeout_s=timeout_s,
            on_error=self._on_upload_error,
        )
        self._uploader.start()
        self._subscription = self.create_subscription(
            Image,
            rgb_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "RobotTrack camera preview bridge active: "
            f"rgb={rgb_topic}, endpoint={self._uploader.endpoint}, max_hz={max_hz:g}"
        )

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        if now < self._next_frame_at:
            return
        self._next_frame_at = now + self._min_frame_interval_s
        try:
            # Keep the original full field of view.  The official inference
            # process continues to receive its independent 384x384 center crop.
            bgr = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(
                ".jpg",
                bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
            )
            if not ok:
                raise ValueError("OpenCV JPEG encoder returned false")
            self._sequence += 1
            self._uploader.submit(self._sequence, encoded.tobytes())
        except Exception as error:
            self._log_error("D435i camera preview frame rejected", error)

    def _on_upload_error(self, error: Exception) -> None:
        self._log_error("RobotTrack camera preview upload failed", error)

    def _log_error(self, prefix: str, error: Exception) -> None:
        now = time.monotonic()
        if now - self._last_error_log >= 1.0:
            self.get_logger().error(f"{prefix}: {type(error).__name__}: {error}")
            self._last_error_log = now

    def close(self) -> None:
        self._uploader.close()


def _arguments(argv: Sequence[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rgb-topic", default="/go2/d435i/color/image_raw")
    parser.add_argument(
        "--server-url", default="http://127.0.0.1:5801/eval_dual"
    )
    parser.add_argument("--max-hz", type=float, default=5.0)
    parser.add_argument("--jpeg-quality", type=int, default=70)
    parser.add_argument("--http-timeout-s", type=float, default=1.0)
    args, ros_args = parser.parse_known_args(argv)
    if not args.rgb_topic.startswith("/"):
        parser.error("--rgb-topic must be an absolute ROS topic")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be in [1, 100]")
    return args, ros_args


def main(argv: Sequence[str] | None = None) -> None:
    args, ros_args = _arguments(argv)
    rclpy.init(args=ros_args)
    node = CameraPreviewBridge(
        rgb_topic=args.rgb_topic,
        server_url=args.server_url,
        max_hz=args.max_hz,
        jpeg_quality=args.jpeg_quality,
        timeout_s=args.http_timeout_s,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
