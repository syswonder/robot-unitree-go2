#!/usr/bin/env python3
"""Render the persistent Go2 profile with MiniCPM-RobotTrack following.

The existing persistent navigation profile remains the baseline.  This
renderer only inserts the RobotTrack primitive and a pre-smoother source mux:
Nav2 and RobotTrack publish to separate raw topics, the mux is the sole
publisher on ``/cmd_vel_nav``, and the existing Nav2 smoother, guard, staged
chassis topic, watchdogs, localization, and map remain intact.  This profile
also leaves the operator-selected Classic gait untouched instead of issuing a
redundant ``ClassicWalk(true)`` request during arm.
"""

from __future__ import annotations

import argparse
import copy
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import yaml

from render_workstation_nomotion_d435i_preview_manifest import (
    D435I_CAMERA_INFO_TOPIC,
    D435I_RGB_TOPIC,
    ManifestError,
    _named,
    write_manifest,
)
from render_workstation_persistent_nav2_manifest import (
    CHASSIS_COMMAND_TOPIC,
    MAPS_DIR,
    MANIFEST_NAME as PERSISTENT_MANIFEST_NAME,
    NAV2_BT_THROUGH_POSES_XML_FILE,
    NAV2_BT_XML_FILE,
    NAV2_PARAMS_FILE,
    PROFILE,
    render as render_persistent,
    validate_rendered as validate_persistent,
)


MANIFEST_NAME = "robonix-go2-workstation-robottrack-follow"
ROBOTTRACK_PROVIDER_ID = "go2_robottrack"
ROBOTTRACK_PACKAGE_PATH = "${ROBONIX_DEPLOY_DIR}/packages/go2_robottrack"
ROBOTTRACK_ASSET_MANIFEST = (
    "${ROBONIX_DEPLOY_DIR}/config/robottrack_assets.yaml"
)
# ROBONIX_DEPLOY_DIR is packages/robot-unitree-go2; the pinned upstream source
# is intentionally kept at the outer Robonix-Go2 workspace level.
ROBOTTRACK_UPSTREAM_ROOT = (
    "${ROBONIX_DEPLOY_DIR}/../../upstream/MiniCPM-Robot"
)
ROBOTTRACK_SERVER_URL = "http://127.0.0.1:5801/eval_dual"
ROBOTTRACK_INSTRUCTION = "Follow the person ahead"
NAV_RAW_COMMAND_TOPIC = "/go2/robottrack/nav_cmd_vel_raw"
ROBOTTRACK_RAW_COMMAND_TOPIC = "/go2/robottrack/cmd_vel_raw"
SMOOTHER_INPUT_TOPIC = "/cmd_vel_nav"
DEFAULT_SOURCE = "robottrack"
# The first physical runs showed reliable yaw following but weak visible
# translation at 0.15-0.30 m/s.  This explicitly selected RobotTrack profile
# may use 0.50 m/s while ordinary Nav2 remains controller-limited to 0.30 m/s.
# Linear acceleration and angular tuning are intentionally unchanged.
ROBOTTRACK_LIVE_MAX_VX = 0.50
ROBOTTRACK_LIVE_MAX_WZ = 0.30


def _config(manifest: dict[str, Any], section: str, name: str) -> dict[str, Any]:
    config = _named(manifest.get(section), name, section).get("config")
    if not isinstance(config, dict):
        raise ManifestError(f"{section}.{name} config is missing")
    return config


def _robottrack_primitive(
    *,
    server_url: str = ROBOTTRACK_SERVER_URL,
    instruction: str = ROBOTTRACK_INSTRUCTION,
) -> dict[str, Any]:
    if not isinstance(server_url, str) or not server_url.strip():
        raise ManifestError("RobotTrack server_url must be a non-empty string")
    parsed_server = urlparse(server_url.strip())
    if (
        parsed_server.scheme not in {"http", "https"}
        or not parsed_server.hostname
        or parsed_server.username is not None
        or parsed_server.password is not None
        or not parsed_server.path.endswith("/eval_dual")
    ):
        raise ManifestError(
            "RobotTrack server_url must be a credential-free HTTP(S) "
            "official /eval_dual endpoint"
        )
    if not isinstance(instruction, str) or not instruction.strip():
        raise ManifestError("RobotTrack instruction must be a non-empty string")
    return {
        "name": ROBOTTRACK_PROVIDER_ID,
        "path": ROBOTTRACK_PACKAGE_PATH,
        "config": {
            # This is an explicitly selected follow profile. The package's
            # generic/default config remains dry-run; only this renderer asks
            # it to publish bounded commands to its private raw topic.
            "mode": "live",
            "rgb_topic": D435I_RGB_TOPIC,
            "camera_info_topic": D435I_CAMERA_INFO_TOPIC,
            "command_topic": ROBOTTRACK_RAW_COMMAND_TOPIC,
            "server_url": server_url.strip(),
            "instruction": instruction.strip(),
            "model_input_mode": "center_crop_height",
            "model_crop_size": 384,
            "waypoint_strategy": "first",
            "control_dt": 0.1,
            "dispatch_hz": 50.0,
            "max_plan_age_s": 1.5,
            "max_vx": ROBOTTRACK_LIVE_MAX_VX,
            "max_wz": ROBOTTRACK_LIVE_MAX_WZ,
            "asset_manifest": ROBOTTRACK_ASSET_MANIFEST,
            "upstream_root": ROBOTTRACK_UPSTREAM_ROOT,
            "source_mux": {
                "nav_input_topic": NAV_RAW_COMMAND_TOPIC,
                "robottrack_input_topic": ROBOTTRACK_RAW_COMMAND_TOPIC,
                "output_topic": SMOOTHER_INPUT_TOPIC,
                "selected_source": DEFAULT_SOURCE,
            },
        },
    }


def render(
    base: dict[str, Any],
    *,
    state_marker: int,
    passive_state_markers: list[int] | tuple[int, ...],
    server_url: str = ROBOTTRACK_SERVER_URL,
    instruction: str = ROBOTTRACK_INSTRUCTION,
) -> dict[str, Any]:
    rendered = render_persistent(
        base,
        state_marker=state_marker,
        passive_state_markers=passive_state_markers,
    )
    rendered["name"] = MANIFEST_NAME

    primitives = rendered.get("primitive")
    if not isinstance(primitives, list):
        raise ManifestError("manifest primitive must be a list")
    rendered["primitive"] = [
        entry
        for entry in primitives
        if not isinstance(entry, dict)
        or entry.get("name") != ROBOTTRACK_PROVIDER_ID
    ]
    rendered["primitive"].append(
        _robottrack_primitive(
            server_url=server_url,
            instruction=instruction,
        )
    )

    nav = _config(rendered, "service", "nav2")
    nav["controller_velocity_output_topic"] = NAV_RAW_COMMAND_TOPIC

    # The on-site operator selects and verifies Classic mode before this
    # explicitly selected RobotTrack profile is armed.  Do not re-request the
    # same Unitree mode during arm: some already-Classic firmware revisions
    # reject that redundant API call even though the read-only state remains
    # Classic.  This override is intentionally local to the RobotTrack
    # manifest; the persistent voice/Nav2 renderer keeps its existing default.
    chassis = _config(rendered, "primitive", "go2_chassis")
    chassis["preserve_classic_walk"] = False
    chassis["max_linear_x_mps"] = ROBOTTRACK_LIVE_MAX_VX

    environment = rendered.get("env")
    if not isinstance(environment, dict):
        raise ManifestError("manifest env must be a mapping")
    environment["ROBOTTRACK_UPSTREAM_ROOT"] = ROBOTTRACK_UPSTREAM_ROOT

    validate_rendered(rendered)
    return rendered


def validate_rendered(manifest: dict[str, Any]) -> None:
    # The base validator proves the previously validated map/localization,
    # Nav2 guard, chassis, D435i preview, voice, and semantic stack remain.
    persistent_view = copy.deepcopy(manifest)
    persistent_view["name"] = PERSISTENT_MANIFEST_NAME
    persistent_view["primitive"] = [
        entry
        for entry in persistent_view.get("primitive", [])
        if not isinstance(entry, dict)
        or entry.get("name") != ROBOTTRACK_PROVIDER_ID
    ]
    _config(persistent_view, "service", "nav2").pop(
        "controller_velocity_output_topic", None
    )
    # Reconstruct the exact persistent baseline only in this validation copy.
    # The persistent validator remains strict about its own ClassicWalk and
    # velocity policies; the final RobotTrack manifest is checked separately.
    persistent_chassis = _config(
        persistent_view, "primitive", "go2_chassis"
    )
    persistent_chassis["preserve_classic_walk"] = True
    persistent_chassis["max_linear_x_mps"] = 0.30
    persistent_environment = persistent_view.get("env")
    if isinstance(persistent_environment, dict):
        persistent_environment.pop("ROBOTTRACK_UPSTREAM_ROOT", None)
    validate_persistent(persistent_view)
    chassis = _config(manifest, "primitive", "go2_chassis")
    robottrack = _config(manifest, "primitive", ROBOTTRACK_PROVIDER_ID)
    nav = _config(manifest, "service", "nav2")
    mapping = _config(manifest, "service", "mapping")
    source_mux = robottrack.get("source_mux")
    environment = manifest.get("env")

    required = {
        "name": manifest.get("name") == MANIFEST_NAME,
        "robottrack_live_private_output": robottrack.get("mode") == "live"
        and robottrack.get("rgb_topic") == D435I_RGB_TOPIC
        and robottrack.get("command_topic")
        == ROBOTTRACK_RAW_COMMAND_TOPIC,
        "runtime_endpoint_and_instruction": isinstance(
            robottrack.get("server_url"), str
        )
        and bool(robottrack["server_url"].strip())
        and isinstance(robottrack.get("instruction"), str)
        and bool(robottrack["instruction"].strip()),
        "official_runtime_shape": robottrack.get("waypoint_strategy") == "first"
        and robottrack.get("model_input_mode") == "center_crop_height"
        and robottrack.get("model_crop_size") == 384
        and robottrack.get("control_dt") == 0.1
        and robottrack.get("dispatch_hz") == 50.0
        and robottrack.get("max_plan_age_s") == 1.5
        and robottrack.get("max_vx") == ROBOTTRACK_LIVE_MAX_VX
        and robottrack.get("max_wz") == ROBOTTRACK_LIVE_MAX_WZ,
        "source_mux": isinstance(source_mux, dict)
        and source_mux
        == {
            "nav_input_topic": NAV_RAW_COMMAND_TOPIC,
            "robottrack_input_topic": ROBOTTRACK_RAW_COMMAND_TOPIC,
            "output_topic": SMOOTHER_INPUT_TOPIC,
            "selected_source": DEFAULT_SOURCE,
        },
        "nav_raw_opt_in": nav.get("controller_velocity_output_topic")
        == NAV_RAW_COMMAND_TOPIC,
        "existing_nav_output": nav.get("velocity_output_topic")
        == CHASSIS_COMMAND_TOPIC
        and nav.get("external_velocity_guard") is False
        and nav.get("params_file") == NAV2_PARAMS_FILE
        and nav.get("bt_xml_file") == NAV2_BT_XML_FILE
        and nav.get("bt_through_poses_xml_file")
        == NAV2_BT_THROUGH_POSES_XML_FILE,
        "existing_chassis_route": chassis.get("twist_in_topic")
        == CHASSIS_COMMAND_TOPIC
        and chassis.get("motion_profile") == PROFILE
        and chassis.get("preserve_classic_walk") is False
        and chassis.get("max_linear_x_mps") == ROBOTTRACK_LIVE_MAX_VX,
        "generation_map_binding": mapping.get("map_mode") == "localization"
        and mapping.get("map_id") == "${GO2_MAP_ID}"
        and mapping.get("reset_map") is False
        and _config(manifest, "service", "go2_dashboard").get(
            "initial_pose_maps_dir"
        )
        == MAPS_DIR,
        "asset_paths": robottrack.get("asset_manifest")
        == ROBOTTRACK_ASSET_MANIFEST
        and robottrack.get("upstream_root") == ROBOTTRACK_UPSTREAM_ROOT
        and isinstance(environment, dict)
        and environment.get("ROBOTTRACK_UPSTREAM_ROOT")
        == ROBOTTRACK_UPSTREAM_ROOT,
        "no_direct_raw_to_chassis": len(
            {
                NAV_RAW_COMMAND_TOPIC,
                ROBOTTRACK_RAW_COMMAND_TOPIC,
                SMOOTHER_INPUT_TOPIC,
                CHASSIS_COMMAND_TOPIC,
            }
        )
        == 4,
    }
    failed = sorted(name for name, passed in required.items() if not passed)
    if failed:
        raise ManifestError(
            "invalid RobotTrack follow manifest: " + ",".join(failed)
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--state-marker", required=True, type=int)
    parser.add_argument("--passive-state-markers", required=True)
    parser.add_argument("--server-url", default=ROBOTTRACK_SERVER_URL)
    parser.add_argument("--instruction", default=ROBOTTRACK_INSTRUCTION)
    args = parser.parse_args()
    if not args.base.is_file() or not args.output.is_absolute():
        parser.error("base must be a file and output must be absolute")
    try:
        markers = [token.strip() for token in args.passive_state_markers.split(",")]
        if any(not token or not token.isdecimal() for token in markers):
            raise ManifestError(
                "passive-state-markers must be comma-separated decimal integers"
            )
        base = yaml.safe_load(args.base.read_text(encoding="utf-8"))
        manifest = render(
            base,
            state_marker=args.state_marker,
            passive_state_markers=[int(token, 10) for token in markers],
            server_url=args.server_url,
            instruction=args.instruction,
        )
        write_manifest(args.output, manifest)
    except (OSError, yaml.YAMLError, ManifestError) as error:
        parser.error(str(error))
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
