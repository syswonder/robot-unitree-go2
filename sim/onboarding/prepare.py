#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Reproduce the pinned Go2 model bundle; no SDK, ROS, or controller imports."""

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import urllib.request

HERE = Path(__file__).resolve().parent
DEFAULT_OUTPUT = HERE.parents[1] / ".runtime/mujoco-onboarding/unitree_go2"
PREFIX = "unitree_robots/go2/"
SCENE = '''<mujoco model="go2 onboarding flat scene">
  <include file="go2.xml"/>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="onboarding_floor" type="plane" size="10 10 0.1"/>
  </worldbody>
</mujoco>
'''


def relative_path(value):
    """Accept only canonical, portable asset-relative paths."""
    if not isinstance(value, str) or not value or "\\" in value:
        raise ValueError(f"Invalid asset path: {value!r}")
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts or str(path) != value or value == ".":
        raise ValueError(f"Invalid asset path: {value!r}")
    return path


def destination(source):
    relative_path(source)
    if source == "LICENSE":
        return "vendor/LICENSE"
    if not source.startswith(PREFIX):
        raise ValueError(f"Unexpected upstream file: {source}")
    return "vendor/" + source[len(PREFIX):]


def load_lock():
    """Validate the lock before any filesystem writes or network requests."""
    lock = json.loads((HERE / "upstream.lock.json").read_text())
    if lock["repository"] != "unitreerobotics/unitree_mujoco":
        raise ValueError("Unexpected repository")
    if not re.fullmatch(r"[0-9a-f]{40}", lock["commit"]):
        raise ValueError("Upstream revision must be a full commit")
    paths = []
    for entry in lock["files"]:
        paths.append(destination(entry["path"]))
        if not re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]):
            raise ValueError("Invalid SHA256")
    if len(set(paths)) != len(paths):
        raise ValueError("Duplicate asset destinations")
    return lock


def checked_path(root, relative):
    """Do not follow existing symlinks outside the explicitly chosen bundle."""
    path = root / relative_path(relative)
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError(f"Asset escapes bundle: {relative}")
    return path


def write_unchanged(path, data):
    """Reuse identical outputs and refuse to overwrite an edited asset."""
    if path.exists():
        if path.read_bytes() != data:
            raise ValueError(f"Existing file differs; choose a new output directory: {path}")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(data)


def prepare(output, source_root=None):
    """Fetch or copy exactly the locked files; SHA256-check before writing."""
    lock = load_lock()
    index = []
    for entry in lock["files"]:
        rel = destination(entry["path"])
        target = checked_path(output, rel)
        if target.exists():
            data = target.read_bytes()
        elif source_root is not None:
            data = checked_path(source_root, entry["path"]).read_bytes()
        else:
            url = f"https://raw.githubusercontent.com/{lock['repository']}/{lock['commit']}/{entry['path']}"
            with urllib.request.urlopen(url, timeout=60) as response:
                data = response.read()
        if hashlib.sha256(data).hexdigest() != entry["sha256"]:
            raise ValueError(f"SHA256 mismatch: {entry['path']}")
        write_unchanged(target, data)
        if entry["path"] != "LICENSE":
            index.append(rel)
    # The scene lives beside go2.xml so its original meshdir remains correct.
    write_unchanged(checked_path(output, "vendor/scene.xml"), SCENE.encode())
    index.append("vendor/scene.xml")
    write_unchanged(checked_path(output, "index.json"),
                    (json.dumps(sorted(index), indent=2) + "\n").encode())
    write_unchanged(checked_path(output, "upstream.lock.json"),
                    (HERE / "upstream.lock.json").read_bytes())
    return {"files_verified": len(lock["files"]), "asset_index_entries": len(index),
            "output": str(output), "status": "assets_prepared_not_runtime_validated"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--source-root", type=Path,
                        help="Existing pinned unitree_mujoco tree; prevents network access")
    args = parser.parse_args()
    print(json.dumps(prepare(args.output, args.source_root), indent=2))


if __name__ == "__main__":
    main()
