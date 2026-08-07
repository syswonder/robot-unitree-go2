"""ROS 2 image, inference, dispatch, and command-source mux node."""

from __future__ import annotations

import threading
import time
from typing import Any, Mapping, MutableSequence, Sequence

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .camera_preview import CameraFrameUploadWorker
from .core import (
    LatestFrameMailbox,
    PlanStore,
    RuntimeConfig,
    VelocityCommand,
    ZERO_COMMAND,
)
from .http_client import RobotTrackHttpClient
from .image_preprocess import prepare_center_crop_height
from .source_mux import CommandSourceMux, TwistCommand
from .worker import InferenceWorker


PARAMETER_DEFAULTS = RuntimeConfig().as_ros_parameters()
CAMERA_PREVIEW_MAX_HZ = 5.0
CAMERA_PREVIEW_TIMEOUT_S = 1.0


def _velocity_twist(command: VelocityCommand) -> Twist:
    message = Twist()
    message.linear.x = float(command.vx)
    message.angular.z = float(command.wz)
    return message


def _full_twist(command: TwistCommand) -> Twist:
    message = Twist()
    message.linear.x = float(command.linear_x)
    message.linear.y = float(command.linear_y)
    message.linear.z = float(command.linear_z)
    message.angular.x = float(command.angular_x)
    message.angular.y = float(command.angular_y)
    message.angular.z = float(command.angular_z)
    return message


def _from_twist(message: Twist) -> TwistCommand:
    return TwistCommand.finite(
        (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
    )


class RobotTrackNode(Node):
    """Forward the freshest D435i frame and dispatch official RobotTrack commands."""

    def __init__(
        self,
        *,
        config_overrides: Mapping[str, Any] | None = None,
        context: Context | None = None,
    ) -> None:
        overrides = [
            Parameter(name, value=value)
            for name, value in dict(config_overrides or {}).items()
            if name in PARAMETER_DEFAULTS
        ]
        super().__init__(
            "go2_robottrack",
            context=context,
            parameter_overrides=overrides,
        )
        parameter_values: dict[str, Any] = {}
        for name, default in PARAMETER_DEFAULTS.items():
            parameter_values[name] = self.declare_parameter(name, default).value
        self._config = RuntimeConfig.from_mapping(parameter_values)

        self._bridge = CvBridge()
        self._mailbox = LatestFrameMailbox()
        self._plans = PlanStore(
            max_plan_age_s=self._config.max_plan_age_s,
            max_vx=self._config.max_vx,
            max_wz=self._config.max_wz,
        )
        self._http = RobotTrackHttpClient(self._config)
        self._mux = CommandSourceMux(
            self._config.selected_source,
            max_age_s=self._config.source_max_age_s,
        )
        self._last_frame_error_log = 0.0
        self._last_camera_preview_error_log = 0.0
        self._last_inference_error_log = 0.0
        self._last_dispatch_log = 0.0
        self._closed = False
        self._raw_publisher = None
        self._selected_publisher = None
        self._nav_subscription = None
        self._robottrack_subscription = None
        self._camera_preview_sequence = 0
        self._next_camera_preview_at = 0.0

        self._camera_preview = CameraFrameUploadWorker(
            self._config.server_url,
            max_hz=CAMERA_PREVIEW_MAX_HZ,
            timeout_s=CAMERA_PREVIEW_TIMEOUT_S,
            on_error=self._on_camera_preview_error,
        )
        self._camera_preview.start()

        self._image_subscription = self.create_subscription(
            Image,
            self._config.rgb_topic,
            self._on_image,
            qos_profile_sensor_data,
        )

        # Dry-run intentionally creates no velocity publisher. Live has no
        # armed/disarmed state: activation immediately starts the two publishers
        # and the mutually exclusive source mux.
        if self._config.mode == "live":
            self._raw_publisher = self.create_publisher(
                Twist, self._config.command_topic, 1
            )
            self._selected_publisher = self.create_publisher(
                Twist, self._config.selected_output_topic, 1
            )
            self._nav_subscription = self.create_subscription(
                Twist,
                self._config.nav_raw_topic,
                self._on_navigation_command,
                1,
            )
            self._robottrack_subscription = self.create_subscription(
                Twist,
                self._config.robottrack_raw_topic,
                self._on_robottrack_command,
                1,
            )

        self._worker = InferenceWorker(
            self._mailbox,
            self._http,
            self._plans,
            on_plan=self._on_plan,
            on_error=self._on_inference_error,
        )
        self._worker.start()
        self._dispatch_timer = self.create_timer(
            1.0 / self._config.dispatch_hz,
            self._dispatch,
        )
        self.get_logger().info(
            "RobotTrack active: "
            f"mode={self._config.mode}, rgb={self._config.rgb_topic}, "
            f"server={self._config.server_url}, source={self._config.selected_source}"
        )

    @property
    def runtime_config(self) -> RuntimeConfig:
        return self._config

    def _on_image(self, message: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self._log_frame_error(error)
            return

        # Queue the official inference crop first.  The raw-camera JPEG uses a
        # separate encoder result and asynchronous HTTP worker, so preview
        # failures cannot change or block the model request payload.
        try:
            model_bgr, _crop_geometry = prepare_center_crop_height(
                bgr,
                crop_size=self._config.model_crop_size,
            )
            ok, encoded = cv2.imencode(
                ".jpg",
                model_bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), self._config.jpeg_quality],
            )
            if not ok:
                raise ValueError("OpenCV JPEG encoder returned false")
            source_stamp = (
                float(message.header.stamp.sec)
                + float(message.header.stamp.nanosec) * 1e-9
            )
            self._mailbox.put(
                encoded.tobytes(),
                source_timestamp=source_stamp,
            )
        except Exception as error:
            self._log_frame_error(error)

        self._offer_camera_preview(bgr)

    def _log_frame_error(self, error: Exception) -> None:
        now = time.monotonic()
        if now - self._last_frame_error_log >= 1.0:
            self.get_logger().error(
                f"D435i RGB frame rejected: {type(error).__name__}: {error}"
            )
            self._last_frame_error_log = now

    def _offer_camera_preview(self, full_bgr: Any) -> None:
        now = time.monotonic()
        if now < self._next_camera_preview_at:
            return
        self._next_camera_preview_at = now + 1.0 / CAMERA_PREVIEW_MAX_HZ
        try:
            ok, encoded = cv2.imencode(
                ".jpg",
                full_bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), self._config.jpeg_quality],
            )
            if not ok:
                raise ValueError("OpenCV JPEG encoder returned false")
            self._camera_preview_sequence += 1
            self._camera_preview.submit(
                self._camera_preview_sequence,
                encoded.tobytes(),
            )
        except Exception as error:
            self._on_camera_preview_error(error)

    def _on_camera_preview_error(self, error: Exception) -> None:
        now = time.monotonic()
        if now - self._last_camera_preview_error_log >= 1.0:
            self.get_logger().error(
                "RobotTrack camera preview failed: "
                f"{type(error).__name__}: {error}"
            )
            self._last_camera_preview_error_log = now

    def _on_navigation_command(self, message: Twist) -> None:
        try:
            self._mux.update("navigation", _from_twist(message))
        except ValueError as error:
            self.get_logger().error(f"navigation raw Twist rejected: {error}")

    def _on_robottrack_command(self, message: Twist) -> None:
        try:
            self._mux.update("robottrack", _from_twist(message))
        except ValueError as error:
            self.get_logger().error(f"RobotTrack raw Twist rejected: {error}")

    def _on_plan(self, frame: Any, plan: Any) -> None:
        del frame
        self.get_logger().debug(
            f"RobotTrack plan: vx={plan.command.vx:.3f}, "
            f"wz={plan.command.wz:.3f}, source={plan.velocity_source}"
        )

    def _on_inference_error(self, frame: Any, error: Exception) -> None:
        del frame
        now = time.monotonic()
        if now - self._last_inference_error_log >= 1.0:
            self.get_logger().error(
                f"RobotTrack inference request failed: {type(error).__name__}: {error}"
            )
            self._last_inference_error_log = now

    def _dispatch(self) -> None:
        now = time.monotonic()
        state = self._plans.dispatch(now=now)
        if self._config.mode == "dry-run":
            self._http.set_executed_command(ZERO_COMMAND)
            if now - self._last_dispatch_log >= 1.0:
                self.get_logger().info(
                    "dry-run prediction: "
                    f"vx={state.command.vx:.3f}, wz={state.command.wz:.3f}, "
                    f"state={state.reason}"
                )
                self._last_dispatch_log = now
            return

        assert self._raw_publisher is not None
        assert self._selected_publisher is not None
        raw_message = _velocity_twist(state.command)
        self._raw_publisher.publish(raw_message)
        self._http.set_executed_command(state.command)
        # Feed the locally generated command into the same mux epoch immediately;
        # the self-subscription still verifies the configured raw topic, while this
        # avoids one timer-period delay before a fresh plan or zero reaches output.
        self._mux.update(
            "robottrack",
            TwistCommand(linear_x=state.command.vx, angular_z=state.command.wz),
            received_monotonic=now,
        )

        selection = self._mux.output(now=now)
        self._selected_publisher.publish(_full_twist(selection.command))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._plans.clear()
        if self._raw_publisher is not None:
            self._raw_publisher.publish(Twist())
        if self._selected_publisher is not None:
            self._selected_publisher.publish(Twist())
        self._http.set_executed_command(ZERO_COMMAND)
        # Interrupt an in-flight request before joining the worker so provider
        # deactivation is not delayed until the full HTTP timeout.
        self._camera_preview.close()
        self._http.close()
        self._worker.stop(timeout_s=2.0)

    def destroy_node(self) -> bool:
        self.close()
        return super().destroy_node()


def run_ros_runtime(
    config: Mapping[str, Any],
    stop_event: threading.Event,
    ready_event: threading.Event,
    errors: MutableSequence[str],
) -> None:
    """Run one provider-owned ROS context until lifecycle deactivation."""

    context = Context()
    node: RobotTrackNode | None = None
    executor: SingleThreadedExecutor | None = None
    try:
        rclpy.init(args=[], context=context)
        node = RobotTrackNode(config_overrides=config, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        ready_event.set()
        while context.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)
    except Exception as error:
        errors.append(f"{type(error).__name__}: {error}")
        ready_event.set()
    finally:
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception:
                pass
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
