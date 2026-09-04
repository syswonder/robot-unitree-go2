# Go2 MiniCPM-RobotTrack bridge

This package connects the Go2-mounted D435i RGB stream to the official
MiniCPM-RobotTrack HTTP inference server and converts its response into the
existing ROS velocity path. It deliberately does not call Unitree SDK2.

The existing RGB-only follow path remains the default. A separate optional
fixed-distance profile combines the same RobotTrack yaw command with aligned
D435i depth and a metric longitudinal setpoint.

## Runtime data path

```text
/go2/d435i/color/image_raw
  -> latest-frame JPEG mailbox
  -> POST http://127.0.0.1:5801/eval_dual
  -> finite base_velocity or finite 8x3 waypoints
  -> /go2/robottrack/cmd_vel_raw
  -> mutually exclusive source mux
  -> /cmd_vel_nav
```

The worker performs one HTTP request at a time. Frames arriving during an
inference replace the one-slot mailbox, so the next request always takes the
newest available frame rather than draining an old queue.

Before JPEG encoding, the D435i frame follows the official Go2 preprocessing:
resize proportionally to 384 pixels high, expand by width first for portrait
input when necessary, then center-crop an exact 384 x 384 square. This prevents
the server's square fallback from stretching a 4:3 person image horizontally.

The request matches the official multipart protocol:

- form field `json` contains `reset`, `idx`, `instruction`,
  `client_control_mode`, `client_exec_velocity`, and
  `client_send_timestamp`;
- form file `image` is the latest JPEG;
- the endpoint is `/eval_dual`.

The response parser accepts the official full response (`waypoints` plus
`base_velocity`) and the official trimmed velocity response. Every supplied
value must be finite. Full waypoint output must be exactly 8 rows of
`[x, y, yaw]`. If the server omits velocity but returns waypoints, the bridge
uses waypoint index 1 over `control_dt=0.1`, matching the official direct/first
controller. Final commands are limited to `|vx| <= 0.15 m/s` and
`|wz| <= 0.30 rad/s`.

Commands are dispatched at 50 Hz. A plan older than 1.5 s becomes zero. The
source mux retains only the latest message from navigation and RobotTrack,
forwards only its configured source, rejects out-of-order updates, and emits
zero when that selected source is old.

With `distance_enabled=true`, the node additionally subscribes to
`/go2/d435i/aligned_depth_to_color/image_raw`. OpenCV HOG supplies a person box;
images narrower than 384 pixels are enlarged to 384 for distant-person
coverage, images wider than 512 are reduced to 512, and boxes are mapped back
to the original aligned RGB-D coordinates. New acquisition requires two
consistent detections, and HOG-seeded optical
flow may bridge at most three missed frames without ever acquiring by itself.
Short-term association keeps the same single target, and the inner box depth
is reduced with validity filtering, percentile trimming, median and MAD.
The proportional distance loop replaces only `vx`; RobotTrack still supplies
`wz`. Missing/stale RGB-D, no accepted person box, invalid depth, low
confidence, or a stale model plan produces zero longitudinal speed.

HOG, optical flow, and depth reduction run in a latest-only background worker,
not in the ROS executor callback. A newer pair or an invalid RGB-D decision
advances the worker generation, so an older in-flight result cannot restore a
cleared measurement; completed results retain the original pair receive time.

This is a forward-only distance controller because the established Go2
chassis path intentionally rejects reverse commands. When the person is too
close, it stops forward translation and retains RobotTrack yaw. Increasing the
setpoint therefore waits for the person to open the gap; decreasing it makes
the Go2 move forward to close the gap.

## Configuration

The standalone ROS defaults are in `config/go2_robottrack.yaml`:

| setting | default |
| --- | --- |
| `mode` | `dry-run` |
| `rgb_topic` | `/go2/d435i/color/image_raw` |
| `depth_topic` | `/go2/d435i/aligned_depth_to_color/image_raw` |
| `server_url` | `http://127.0.0.1:5801/eval_dual` |
| `instruction` | `Follow the person ahead` |
| `model_input_mode` | `center_crop_height` |
| `model_crop_size` | `384` |
| `command_topic` | `/go2/robottrack/cmd_vel_raw` |
| `waypoint_strategy` | `first` |
| `control_dt` | `0.1` |
| `dispatch_hz` | `50.0` |
| `max_plan_age_s` | `1.5` |
| `max_vx` / `max_wz` | `0.15` / `0.30` |
| `nav_raw_topic` | `/go2/robottrack/nav_cmd_vel_raw` |
| `robottrack_raw_topic` | `/go2/robottrack/cmd_vel_raw` |
| `selected_output_topic` | `/cmd_vel_nav` |
| `selected_source` | `robottrack` |
| `distance_enabled` | `false` |
| `target_distance_m` | `5.0` |
| `min_target_distance_m` / `max_target_distance_m` | `0.5` / `7.0` |
| `distance_deadband_m` / `distance_kp` | `0.25` / `0.25` |
| `distance_max_forward_mps` / `distance_max_reverse_mps` | `0.15` / `0.0` |
| `distance_measurement_max_age_s` | `0.60` |
| `distance_min_confidence` | `0.50` |
| `distance_pair_max_delta_s` | `0.20` |
| `distance_center_fallback` | `false` |

These are the package's conservative standalone/dry-run defaults. The
dedicated workstation fixed-distance renderer explicitly uses the already
validated RobotTrack live forward limit of `0.50 m/s`; reverse remains `0.0`.

The Robonix `on_init` form accepts the same top-level settings and this nested
source-mux form used by the deployment renderer:

```yaml
mode: live
rgb_topic: /go2/d435i/color/image_raw
server_url: http://127.0.0.1:5801/eval_dual
source_mux:
  nav_input_topic: /go2/robottrack/nav_cmd_vel_raw
  robottrack_input_topic: /go2/robottrack/cmd_vel_raw
  output_topic: /cmd_vel_nav
  selected_source: robottrack
```

`camera_info_topic`, `asset_manifest`, and `upstream_root` may accompany the
provider configuration as deployment metadata. The RGB-only inference bridge
does not consume them.

The model/inference server is a separate process from the OpenBMB source tree.
This package neither stores Hugging Face credentials nor embeds model weights.
Runtime diagnostics go to the provider/ROS stdout stream; the existing Robonix
startup chain is responsible for retaining that stream in its normal package
logs. This package does not create a second ad-hoc JSONL or approval artifact.

`classic_walk` is the deployment's recommended movement profile. This package
does not inspect state error fields or invoke any gait-switching API; the
already established external Go2 runtime owns that setup.

## Runtime distance control

The provider publishes `robonix/primitive/follow/distance` with one synchronous
request shape:

```text
operation: set | adjust | get
meters: absolute metres for set, signed delta metres for adjust
```

The response reports whether the update was accepted plus enabled/active,
target distance, latest measured distance, status and detail. The target is
held in the provider process and may be changed while following. The local
semantic router maps `离我远一点` to `adjust +1.0`, `靠近一点` to
`adjust -1.0`, and phrases such as `保持5米` to `set 5.0`. Executor feedback
ends the voice turn so a relative adjustment is never replayed.

Use the existing RGB-only profile exactly as before:

```bash
bash scripts/start_workstation_robottrack_follow.sh
```

Use the opt-in fixed-distance profile with an initial 5 m target:

```bash
bash scripts/start_workstation_robottrack_distance_follow.sh
```

An alternate initial target may be selected for that invocation with
`ROBOTTRACK_TARGET_DISTANCE_M`; ordinary navigation and the ordinary follow
launcher do not inherit the distance mode.

The fixed-distance launcher selects a memory-lean Robonix manifest that omits
only the unused Scene system. Mapping/localization, Nav2 velocity smoothing,
the external velocity guard, the single chassis owner, D435i, speech,
Pilot/Executor, and the distance capability remain present. The ordinary
RGB-only follow launcher keeps the full manifest.

## Physical validation

The dedicated profile completed a supervised `300.164594791 s` physical run
on 2026-08-09. Runtime requests successfully changed the target through
`5 -> 6 -> 5 -> 1 -> 6 -> 5 m`; the operator explicitly accepted both the
close `1 m` and distant `6 m` behavior, and the final step returned only the
current runtime target to `5 m`. The post-clamp SDK stream contained `11,657`
Move requests over `299.821889 s`, with `vx <= 0.50 m/s`,
`|wz| <= 0.30 rad/s`, and no command interval above `0.1 s`. The measured
chassis session distance was `16.9967 m`, and the final state was `DISARMED`.

See
`docs/reports/2026-08-09/robottrack-fixed-distance-voice300-physical-result.md`
for the evidence boundary, including the unavailable final-window rosbag.

## Build and lifecycle start

```bash
bash build.sh
bash start.sh
```

`start.sh` runs `Primitive(id="go2_robottrack",
namespace="robonix/primitive/follow")`. Robonix initialization supplies the
configuration, activation starts the ROS thread, and deactivate/shutdown stop
it. `live` starts publishing immediately; there is no additional armed state.

For a standalone ROS launch after building:

```bash
source .build/ros/install/setup.bash
ros2 launch go2_robottrack go2_robottrack.launch.py mode:=dry-run
```

Changing `mode:=live` creates velocity publishers. Generic/package defaults
remain dry-run so offline checks and ordinary package startup cannot publish a
velocity accidentally.

## Offline tests

```bash
bash tests/run_offline_tests.sh
```

The suite uses only in-process synthetic data and fake transports/providers.
It does not import ROS, open a socket, contact the inference server, or connect
to a robot.
