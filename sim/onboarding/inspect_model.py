#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Inspect the pinned model contract without importing MuJoCo/ROS/Unitree SDK."""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from prepare import DEFAULT_OUTPUT, SCENE, checked_path, destination, load_lock

SDK_JOINTS = tuple(f"{leg}_{joint}_joint" for leg in ("FR", "FL", "RR", "RL")
                   for joint in ("hip", "thigh", "calf"))


def inspect_bundle(root):
    """Check immutable inputs, complete assets and the named joint contract."""
    lock = load_lock()
    for entry in lock["files"]:
        path = checked_path(root, destination(entry["path"]))
        if hashlib.sha256(path.read_bytes()).hexdigest() != entry["sha256"]:
            raise ValueError(f"SHA256 mismatch: {path}")
    if checked_path(root, "upstream.lock.json").read_bytes() != (Path(__file__).parent / "upstream.lock.json").read_bytes():
        raise ValueError("Bundle provenance does not match lock")
    if checked_path(root, "vendor/scene.xml").read_text() != SCENE:
        raise ValueError("Generated scene differs")
    index = json.loads(checked_path(root, "index.json").read_text())
    expected = {destination(entry["path"]) for entry in lock["files"]
                if entry["path"] != "LICENSE"} | {"vendor/scene.xml"}
    if len(index) != len(set(index)) or set(index) != expected:
        raise ValueError("Incomplete or duplicated asset index")
    for path in index:
        checked_path(root, path).read_bytes()
    model = ET.parse(root / "vendor/go2.xml").getroot()
    for mesh in model.findall("asset/mesh"):
        path = "vendor/assets/" + mesh.attrib["file"]
        if path not in expected:
            raise ValueError(f"Mesh omitted from index: {path}")
    joints = [item.attrib["name"] for item in model.findall("worldbody//joint")]
    motors = list(model.find("actuator"))
    if len(joints) != 12 or set(joints) != set(SDK_JOINTS):
        raise ValueError("Unexpected articulated joint names")
    if len(motors) != 12 or tuple(m.attrib["joint"] for m in motors) != SDK_JOINTS:
        raise ValueError("Unexpected actuator contract")
    if len(model.findall("worldbody//freejoint")) != 1:
        raise ValueError("Expected exactly one floating base")
    mapping = [{"sdk_index": i, "joint": name, "actuator": motors[i].attrib["name"],
                "xml_joint_order": joints.index(name)} for i, name in enumerate(SDK_JOINTS)]
    return {"status": "static_pass", "upstream_commit": lock["commit"],
            "files_verified": len(lock["files"]), "asset_index_entries": len(index),
            "root_body": "base_link", "actuator_semantics": "motor torque, not joint position",
            "quaternion_order": "wxyz in MuJoCo, xyzw in ROS",
            "model_mass_sum_kg": sum(float(n.attrib["mass"]) for n in model.findall("worldbody//inertial")),
            "joint_mapping": mapping, "runtime_validated": False,
            "walking_controller_included": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    print(json.dumps(inspect_bundle(args.assets), indent=2))


if __name__ == "__main__":
    main()
