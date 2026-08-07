# Go2 MiniCPM-RobotTrack bridge

This package connects the Go2-mounted D435i RGB stream to the official
MiniCPM-RobotTrack HTTP inference server and converts its response into the
existing ROS velocity path. It deliberately does not call Unitree SDK2.

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

## Configuration

The standalone ROS defaults are in `config/go2_robottrack.yaml`:

| setting | default |
| --- | --- |
| `mode` | `dry-run` |
| `rgb_topic` | `/go2/d435i/color/image_raw` |
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
