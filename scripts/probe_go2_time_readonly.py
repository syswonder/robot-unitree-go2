#!/usr/bin/env python3
"""Bounded, no-adjust timestamp probe for the physical Go2 data plane.

The process creates ROS subscriptions only.  It neither publishes ROS data nor
opens a Unitree client.  Source headers are compared with clocks sampled at the
callback boundary and are written unchanged to private evidence files.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import socket
import sys
import time
from typing import Any, Callable


ROOT = Path(__file__).resolve().parents[1]
TIME_SYNC_DIR = ROOT / "deploy" / "time-sync"
sys.path.insert(0, str(TIME_SYNC_DIR))

from go2_time_core import (  # noqa: E402
    Observation,
    StreamTracker,
    pairwise_median_offsets,
    read_clock_pair,
)


DEFAULT_TOPICS = {
    "sport_primary": "/sportmodestate",
    "sport_fallback": "/lf/sportmodestate",
    "mid360_cloud": "/utlidar/cloud",
    "mid360_imu": "/utlidar/imu",
}


def _bounded_int(value: str, minimum: int, maximum: int, label: str) -> int:
    try:
        parsed = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"{label} must be an integer") from error
    if not minimum <= parsed <= maximum:
        raise argparse.ArgumentTypeError(
            f"{label} must be in {minimum}..{maximum}"
        )
    return parsed


def _topic(value: str) -> str:
    if not value.startswith("/") or value == "/" or "//" in value:
        raise argparse.ArgumentTypeError("topic names must be absolute and non-empty")
    allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_/")
    if any(character not in allowed for character in value):
        raise argparse.ArgumentTypeError("topic contains an unsupported character")
    return value


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="READ-ONLY Go2 source timestamp probe; never adjusts a clock"
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--duration-seconds",
        type=lambda value: _bounded_int(value, 1, 86_400, "duration-seconds"),
        default=60,
    )
    parser.add_argument(
        "--max-samples",
        type=lambda value: _bounded_int(value, 1, 10_000_000, "max-samples"),
        default=200_000,
    )
    parser.add_argument(
        "--retained-offsets-per-stream",
        type=lambda value: _bounded_int(
            value, 100, 500_000, "retained-offsets-per-stream"
        ),
        default=200_000,
    )
    parser.add_argument("--primary-topic", type=_topic, default=DEFAULT_TOPICS["sport_primary"])
    parser.add_argument("--fallback-topic", type=_topic, default=DEFAULT_TOPICS["sport_fallback"])
    parser.add_argument("--cloud-topic", type=_topic, default=DEFAULT_TOPICS["mid360_cloud"])
    parser.add_argument("--imu-topic", type=_topic, default=DEFAULT_TOPICS["mid360_imu"])
    return parser


def _prepare_output(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    resolved.mkdir(mode=0o700, parents=True, exist_ok=True)
    if not resolved.is_dir():
        raise RuntimeError(f"output path is not a directory: {resolved}")
    for name in ("samples.jsonl", "summary.json", "metadata.json"):
        candidate = resolved / name
        if candidate.exists():
            raise RuntimeError(f"refusing to overwrite existing evidence: {candidate}")
    return resolved


def _write_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    os.chmod(temporary, 0o600)
    temporary.replace(path)


def _stamp_from_sport(message: Any) -> tuple[int, int]:
    return int(message.stamp.sec), int(message.stamp.nanosec)


def _stamp_from_header(message: Any) -> tuple[int, int]:
    return int(message.header.stamp.sec), int(message.header.stamp.nanosec)


def run_probe(arguments: argparse.Namespace) -> int:
    # ROS imports are deliberately delayed.  Pure timestamp tests therefore do
    # not initialize DDS and can run on a machine with no ROS installation.
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import Imu, PointCloud2
        from unitree_go.msg import SportModeState
    except ImportError as error:
        print(f"missing ROS 2 or Unitree message dependency: {error}", file=sys.stderr)
        return 127

    output_dir = _prepare_output(arguments.output_dir)
    sample_path = output_dir / "samples.jsonl"
    os.umask(0o077)
    topics = {
        "sport_primary": arguments.primary_topic,
        "sport_fallback": arguments.fallback_topic,
        "mid360_cloud": arguments.cloud_topic,
        "mid360_imu": arguments.imu_topic,
    }
    trackers = {
        name: StreamTracker(
            name,
            topic,
            retained_offset_limit=arguments.retained_offsets_per_stream,
        )
        for name, topic in topics.items()
    }

    started_realtime_ns, started_monotonic_ns, started_clock_span_ns = read_clock_pair()
    metadata = {
        "schema_version": 1,
        "mode": "read-only-no-adjust",
        "clock_adjustment_requested": False,
        "ros_publishers_created": False,
        "unitree_clients_created": False,
        "hostname": socket.gethostname(),
        "pid": os.getpid(),
        "started_realtime_ns": started_realtime_ns,
        "started_monotonic_ns": started_monotonic_ns,
        "started_clock_read_span_ns": started_clock_span_ns,
        "duration_seconds": arguments.duration_seconds,
        "max_samples": arguments.max_samples,
        "topics": topics,
        "interpretation": (
            "source_minus_receipt is measurement evidence only; it is never "
            "used to replace a ROS header timestamp"
        ),
    }
    _write_json(output_dir / "metadata.json", metadata)

    total_samples = 0
    sample_stream = sample_path.open("x", encoding="utf-8", buffering=1)
    os.chmod(sample_path, 0o600)

    class ReadOnlyTimeProbe(Node):
        def __init__(self) -> None:
            super().__init__("go2_time_probe_readonly")
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )

            def callback_for(
                stream_name: str, stamp_getter: Callable[[Any], tuple[int, int]]
            ) -> Callable[[Any], None]:
                def callback(message: Any) -> None:
                    nonlocal total_samples
                    receipt_realtime, receipt_monotonic, clock_span = read_clock_pair()
                    seconds, nanoseconds = stamp_getter(message)
                    observation: Observation = trackers[stream_name].observe(
                        seconds,
                        nanoseconds,
                        receipt_realtime,
                        receipt_monotonic,
                        clock_span,
                    )
                    sample_stream.write(
                        json.dumps(
                            observation.as_dict(),
                            ensure_ascii=False,
                            sort_keys=True,
                            separators=(",", ":"),
                        )
                        + "\n"
                    )
                    total_samples += 1

                return callback

            # Subscriptions are retained explicitly.  This node intentionally
            # has no publisher, service, action client, or Unitree SDK object.
            self.subscriptions = [
                self.create_subscription(
                    SportModeState,
                    topics["sport_primary"],
                    callback_for("sport_primary", _stamp_from_sport),
                    qos,
                ),
                self.create_subscription(
                    SportModeState,
                    topics["sport_fallback"],
                    callback_for("sport_fallback", _stamp_from_sport),
                    qos,
                ),
                self.create_subscription(
                    PointCloud2,
                    topics["mid360_cloud"],
                    callback_for("mid360_cloud", _stamp_from_header),
                    qos,
                ),
                self.create_subscription(
                    Imu,
                    topics["mid360_imu"],
                    callback_for("mid360_imu", _stamp_from_header),
                    qos,
                ),
            ]

    rclpy.init(args=None)
    node = ReadOnlyTimeProbe()
    deadline_ns = time.monotonic_ns() + arguments.duration_seconds * 1_000_000_000
    exit_reason = "duration_elapsed"
    try:
        while rclpy.ok() and time.monotonic_ns() < deadline_ns:
            if total_samples >= arguments.max_samples:
                exit_reason = "max_samples_reached"
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        exit_reason = "interrupted"
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sample_stream.close()

    finished_realtime_ns, finished_monotonic_ns, finished_clock_span_ns = read_clock_pair()
    summaries = [tracker.summary() for tracker in trackers.values()]
    unsafe_counts = sum(
        tracker.zero + tracker.malformed + tracker.duplicates + tracker.regressions
        for tracker in trackers.values()
    )
    summary = {
        "schema_version": 1,
        "mode": "read-only-no-adjust",
        "exit_reason": exit_reason,
        "total_samples": total_samples,
        "started_realtime_ns": started_realtime_ns,
        "finished_realtime_ns": finished_realtime_ns,
        "elapsed_monotonic_ns": finished_monotonic_ns - started_monotonic_ns,
        "finished_clock_read_span_ns": finished_clock_span_ns,
        "streams": summaries,
        "pairwise_median_offset_comparisons": pairwise_median_offsets(
            list(trackers.values())
        ),
        "timestamp_anomaly_count": unsafe_counts,
        "safe_for_clock_discipline": False,
        "note": (
            "This probe never authorizes clock discipline. Publisher locator, "
            "cold-boot, long-duration drift, and operator review remain mandatory."
        ),
    }
    _write_json(output_dir / "summary.json", summary)
    print(f"READ-ONLY time evidence saved to {output_dir}")
    return 0


def main(argv: list[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    return run_probe(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
