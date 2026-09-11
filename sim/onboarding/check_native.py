#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Optional CPU-only model compilation/passive-step check, NOT walking control."""

import argparse
import json
import math
from pathlib import Path

from inspect_model import SDK_JOINTS, inspect_bundle
from prepare import DEFAULT_OUTPUT


def check_native(root):
    """Resolve addresses by name; never assume SDK order equals qpos order."""
    inspect_bundle(root)
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(root / "vendor/scene.xml"))
    if (model.nq, model.nv, model.nu) != (19, 18, 12):
        raise ValueError(f"Unexpected model dimensions: {model.nq, model.nv, model.nu}")
    data = mujoco.MjData(model)
    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home < 0:
        raise ValueError("Missing home keyframe")
    mujoco.mj_resetDataKeyframe(model, data, home)
    # Upstream home.ctrl contains angle-shaped values, but motors take torque.
    # Zero passive torque for this short load smoke test; no standing claim.
    data.ctrl[:] = 0
    mujoco.mj_forward(model, data)
    mapping = []
    for name in SDK_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name.removesuffix("_joint"))
        if jid < 0 or aid < 0 or int(model.actuator_trnid[aid, 0]) != jid:
            raise ValueError(f"Missing or mismatched transmission: {name}")
        if int(model.jnt_type[jid]) != int(mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError(f"Expected hinge joint: {name}")
        mapping.append({"joint": name, "actuator_id": aid,
                        "qpos_address": int(model.jnt_qposadr[jid]),
                        "qvel_address": int(model.jnt_dofadr[jid]),
                        "ctrlrange": model.actuator_ctrlrange[aid].tolist()})
    for _ in range(20):
        mujoco.mj_step(model, data)
        if not all(math.isfinite(float(v)) for a in (data.qpos, data.qvel, data.qacc) for v in a):
            raise ValueError("Non-finite passive simulation state")
    if any(w.number for w in data.warning):
        raise ValueError("MuJoCo raised a simulation warning")
    return {"status": "native_load_and_passive_steps_pass", "mujoco_version": mujoco.__version__,
            "simulation_seconds": float(data.time), "joint_mapping": mapping,
            "walking_validated": False, "ros_bridge_validated": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    try:
        print(json.dumps(check_native(args.assets), indent=2))
    except ModuleNotFoundError as exc:
        if exc.name != "mujoco":
            raise
        parser.exit(2, "MuJoCo is not installed in this Python environment. See README.md; no installation was attempted.\n")


if __name__ == "__main__":
    main()
