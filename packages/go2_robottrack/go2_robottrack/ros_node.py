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
from .distance_control import DistanceMeasurement
from .distance_dispatch import select_distance_dispatch
from .distance_worker import (
    RgbdDistanceWorker,
    fresh_rgbd_measurement_timestamp,
)
from .follow_distance_runtime import configure_distance_runtime, distance_runtime
from .http_client import RobotTrackHttpClient
from .image_preprocess import prepare_center_crop_height
from .rgbd_distance import RgbdDistanceConfig, RgbdPersonDistanceEstimator
from .rgbd_sync import LatestRgbdPairer, RgbdPairDecision, RgbdFramePair
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


def _image_source_timestamp(message: Image) -> float:
    return (
        float(message.header.stamp.sec)
        + float(message.header.stamp.nanosec) * 1e-9
    )


class RobotTrackNode(Node):
    """Forward the freshest D435i frame and dispatch official RobotTrack commands."""

    def __init__(
        self,
        *,
        config_overrides: Mapping[str, Any] | None = None,
        context: Context | None = None,
        manage_distance_runtime: bool = True,
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
        self._manage_distance_runtime = bool(manage_distance_runtime)
        if self._manage_distance_runtime:
            configure_distance_runtime(self._config)
            distance_runtime.set_active(
                self._config.distance_enabled,
                "standalone RobotTrack ROS runtime active",
            )

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
        self._last_distance_error_log = 0.0
        self._last_dispatch_log = 0.0
        self._distance_state_lock = threading.Lock()
        self._closed = False
        self._raw_publisher = None
        self._selected_publisher = None
        self._nav_subscription = None
        self._robottrack_subscription = None
        self._depth_subscription = None
        self._distance_pairer: LatestRgbdPairer | None = None
        self._distance_estimator: RgbdPersonDistanceEstimator | None = None
        self._distance_worker: RgbdDistanceWorker | None = None
        self._distance_measurement_received_monotonic: float | None = None
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
        if self._config.distance_enabled:
            self._distance_pairer = LatestRgbdPairer(
                max_source_delta_s=self._config.distance_pair_max_delta_s,
                max_frame_age_s=self._config.distance_measurement_max_age_s,
            )
            self._distance_estimator = RgbdPersonDistanceEstimator(
                config=RgbdDistanceConfig(
                    max_frame_age_s=self._config.distance_measurement_max_age_s,
                    enable_center_fallback=self._config.distance_center_fallback,
                )
            )
            self._distance_worker = RgbdDistanceWorker(
                self._distance_estimator,
                on_result=self._on_distance_estimate,
                on_error=self._on_distance_estimation_error,
            )
            self._distance_worker.start()
            self._depth_subscription = self.create_subscription(
                Image,
                self._config.depth_topic,
                self._on_depth,
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
            + (
                f", fixed_distance={self._config.target_distance_m:.2f}m, "
                f"depth={self._config.depth_topic}"
                if self._config.distance_enabled
                else ""
            )
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

        source_stamp = _image_source_timestamp(message)

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
            self._mailbox.put(
                encoded.tobytes(),
                source_timestamp=source_stamp,
            )
        except Exception as error:
            self._log_frame_error(error)

        self._offer_camera_preview(bgr)
        if self._distance_pairer is not None:
            try:
                decision = self._distance_pairer.offer_rgb(
                    bgr,
                    source_timestamp_s=source_stamp,
                )
                self._handle_rgbd_decision(decision)
            except Exception as error:
                self._invalidate_distance_measurement(
                    "rgbd_processing_error",
                    f"RGB frame could not enter distance estimation: {error}",
                )
                self._log_distance_error(error)

    def _on_depth(self, message: Image) -> None:
        pairer = self._distance_pairer
        if pairer is None:
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(
                message,
                desired_encoding="passthrough",
            )
            decision = pairer.offer_depth(
                depth,
                source_timestamp_s=_image_source_timestamp(message),
            )
            self._handle_rgbd_decision(decision)
        except Exception as error:
            self._invalidate_distance_measurement(
                "depth_frame_error",
                f"D435i aligned depth frame rejected: {error}",
            )
            self._log_distance_error(error)

    def _handle_rgbd_decision(self, decision: RgbdPairDecision) -> None:
        if decision.pair is not None:
            worker = self._distance_worker
            if worker is not None:
                worker.submit(decision.pair)
        elif decision.invalidate_measurement:
            self._invalidate_distance_measurement(decision.status, decision.detail)

    def _on_distance_estimate(
        self,
        pair: RgbdFramePair,
        estimate: Any,
        epoch: int,
    ) -> None:
        detail = (
            f"{estimate.status}; source={estimate.source}; "
            f"pair_delta={pair.source_delta_s:.3f}s"
        )
        with self._distance_state_lock:
            if self._closed:
                return
            worker = self._distance_worker
            if worker is None:
                return

            def commit() -> None:
                now = time.monotonic()
                received = fresh_rgbd_measurement_timestamp(
                    pair,
                    now_monotonic=now,
                    max_age_s=self._config.distance_measurement_max_age_s,
                )
                if received is None:
                    age = now - pair.received_monotonic
                    self._clear_distance_measurement_locked(
                        "stale_measurement",
                        (
                            f"RGB-D result age {age:.3f}s exceeds "
                            f"{self._config.distance_measurement_max_age_s:.3f}s"
                        ),
                        source=estimate.source,
                    )
                    return
                if not estimate.valid or estimate.distance_m is None:
                    self._clear_distance_measurement_locked(
                        estimate.status,
                        detail,
                        source=estimate.source,
                    )
                    return
                distance_runtime.update_measurement(
                    DistanceMeasurement(
                        distance_m=estimate.distance_m,
                        confidence=estimate.confidence,
                        received_monotonic=pair.received_monotonic,
                    ),
                    status=estimate.status,
                    source=estimate.source,
                    detail=detail,
                )
                self._distance_measurement_received_monotonic = (
                    pair.received_monotonic
                )

            worker.commit_if_current(epoch, commit)

    def _on_distance_estimation_error(
        self,
        pair: RgbdFramePair,
        error: Exception,
        epoch: int,
    ) -> None:
        del pair
        with self._distance_state_lock:
            if self._closed:
                return
            worker = self._distance_worker
            if worker is None:
                return
            committed, _result = worker.commit_if_current(
                epoch,
                lambda: self._clear_distance_measurement_locked(
                    "distance_estimation_error",
                    f"RGB-D distance estimation failed: {error}",
                ),
            )
        if committed:
            self._log_distance_error(error)

    def _invalidate_distance_measurement(
        self,
        status: str,
        detail: str,
        *,
        source: str = "",
    ) -> None:
        worker = self._distance_worker
        if worker is not None:
            worker.invalidate()
        self._clear_distance_measurement(status, detail, source=source)

    def _clear_distance_measurement_locked(
        self,
        status: str,
        detail: str,
        *,
        source: str = "",
    ) -> None:
        distance_runtime.update_measurement(
            None,
            status=status,
            source=source,
            detail=detail,
        )
        self._distance_measurement_received_monotonic = None

    def _clear_distance_measurement(
        self,
        status: str,
        detail: str,
        *,
        source: str = "",
    ) -> None:
        with self._distance_state_lock:
            if self._closed:
                return
            self._clear_distance_measurement_locked(
                status,
                detail,
                source=source,
            )

    def _expire_distance_measurement(self, now: float) -> None:
        with self._distance_state_lock:
            received = self._distance_measurement_received_monotonic
            if (
                received is None
                or now - received
                <= self._config.distance_measurement_max_age_s
            ):
                return
            distance_runtime.update_measurement(
                None,
                status="stale_measurement",
                source="",
                detail="the latest RGB-D target distance measurement expired",
            )
            self._distance_measurement_received_monotonic = None

    def _log_distance_error(self, error: Exception) -> None:
        now = time.monotonic()
        if now - self._last_distance_error_log >= 1.0:
            self.get_logger().error(
                "RobotTrack distance input failed: "
                f"{type(error).__name__}: {error}"
            )
            self._last_distance_error_log = now

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
        self._expire_distance_measurement(now)
        state = self._plans.dispatch(now=now)
        selection = select_distance_dispatch(
            state,
            distance_enabled=self._config.distance_enabled,
            apply_distance=lambda command, current: distance_runtime.apply(
                command,
                now_monotonic=current,
            ),
            now_monotonic=now,
        )
        command = selection.command
        if (
            self._config.distance_enabled
            and selection.reason == "stale_measurement"
            and distance_runtime.snapshot().measured_distance_m is not None
        ):
            self._expire_distance_measurement(now)
        if self._config.mode == "dry-run":
            self._http.set_executed_command(ZERO_COMMAND)
            if now - self._last_dispatch_log >= 1.0:
                self.get_logger().info(
                    "dry-run prediction: "
                    f"vx={command.vx:.3f}, wz={command.wz:.3f}, "
                    f"state={selection.reason}"
                )
                self._last_dispatch_log = now
            return

        assert self._raw_publisher is not None
        assert self._selected_publisher is not None
        raw_message = _velocity_twist(command)
        self._raw_publisher.publish(raw_message)
        self._http.set_executed_command(command)
        # Feed the locally generated command into the same mux epoch immediately;
        # the self-subscription still verifies the configured raw topic, while this
        # avoids one timer-period delay before a fresh plan or zero reaches output.
        self._mux.update(
            "robottrack",
            TwistCommand(linear_x=command.vx, angular_z=command.wz),
            received_monotonic=now,
        )

        selection = self._mux.output(now=now)
        self._selected_publisher.publish(_full_twist(selection.command))

    def close(self) -> None:
        with self._distance_state_lock:
            if self._closed:
                return
            self._closed = True
        distance_worker = self._distance_worker
        if distance_worker is not None:
            # Wake it immediately, but do not delay the zero-command stop path
            # on a detector invocation that is already in progress.
            distance_worker.request_stop()
        self._plans.clear()
        if self._distance_pairer is not None:
            self._distance_pairer.clear()
        if self._config.distance_enabled:
            distance_runtime.update_measurement(
                None,
                status="inactive",
                source="",
                detail="RobotTrack RGB-D runtime closed",
            )
            distance_runtime.set_active(False, "RobotTrack RGB-D runtime closed")
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
        if distance_worker is not None:
            distance_worker.stop(timeout_s=2.0)

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
        node = RobotTrackNode(
            config_overrides=config,
            context=context,
            manage_distance_runtime=False,
        )
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
