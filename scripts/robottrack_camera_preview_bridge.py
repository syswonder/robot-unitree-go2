#!/usr/bin/env python3
"""Publish D435i full-frame JPEG previews to a local RobotTrack server.

This process only subscribes to one ROS Image topic.  It creates no ROS
publishers and is independent of RobotTrack inference and motion control.
"""

from __future__ import annotations

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "packages" / "go2_robottrack"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from go2_robottrack.camera_preview_node import main


if __name__ == "__main__":
    main()
