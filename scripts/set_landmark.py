#!/usr/bin/env python3
"""Create/update a local, ignored, physically verified landmark pose."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import math
from pathlib import Path
import tempfile

import yaml


def uint64(value: str) -> int:
    try:
        parsed = int(value, 10)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a base-10 uint64") from exc
    if parsed < 0 or parsed > (1 << 64) - 1:
        raise argparse.ArgumentTypeError("must be within uint64 range")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Save a collision-free robot base approach pose; this does not move the robot."
    )
    parser.add_argument("--name", default="自动售货机")
    parser.add_argument("--map-id", required=True)
    parser.add_argument(
        "--generation",
        required=True,
        type=uint64,
        help="generation from the live robonix/service/map/lifecycle sample",
    )
    parser.add_argument("--x", required=True, type=float)
    parser.add_argument("--y", required=True, type=float)
    parser.add_argument("--yaw", required=True, type=float, help="radians in map frame")
    parser.add_argument("--measured-by", required=True)
    parser.add_argument(
        "--confirm-free-space",
        required=True,
        choices=["YES_POSE_AND_FOOTPRINT_ARE_CLEAR"],
        help="explicit physical clearance acknowledgement",
    )
    parser.add_argument(
        "--output",
        default="config/semantic_landmarks.local.yaml",
        help="local file is intentionally gitignored",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.map_id.strip():
        raise SystemExit("map-id must not be empty")
    for label, value in (("x", args.x), ("y", args.y), ("yaw", args.yaw)):
        if not math.isfinite(value):
            raise SystemExit(f"{label} must be finite")
    root = Path(__file__).resolve().parent.parent
    template = root / "config" / "semantic_landmarks.yaml"
    output = Path(args.output)
    if not output.is_absolute():
        output = root / output
    source = output if output.is_file() else template
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise SystemExit(f"landmark document must be a mapping: {source}")
    data["schema_version"] = 2
    data["map_id"] = args.map_id.strip()
    data["map_generation"] = args.generation
    matches = [row for row in data.get("landmarks", []) if row.get("name") == args.name]
    if len(matches) != 1:
        raise SystemExit(f"expected exactly one landmark named {args.name!r}, found {len(matches)}")
    row = matches[0]
    row["pose"] = {
        "x": round(args.x, 6),
        "y": round(args.y, 6),
        "yaw": round(math.atan2(math.sin(args.yaw), math.cos(args.yaw)), 6),
    }
    row["verified"] = True
    row["metadata"] = {
        "measured_by": args.measured_by,
        "measured_at": datetime.now(timezone.utc).isoformat(),
        "source": "physical_map_clearance_check",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=output.parent, delete=False) as handle:
        yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)
        temporary = Path(handle.name)
    temporary.chmod(0o600)
    temporary.replace(output)
    print(f"saved verified landmark to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
