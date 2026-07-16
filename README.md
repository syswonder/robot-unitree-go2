# Robonix deployment for Unitree Go2

This repository adapts a Unitree Go2 to Robonix and Nav2 using only the
high-level Unitree Sport service. It standardizes chassis odometry, IMU,
onboard lidar and video; supplies the Go2 model/TF tree; configures RTAB-Map
and Nav2; maps Chinese semantic landmark names to verified map poses; and
provides a read-only browser dashboard.

All non-DDS control/capability/audio/dashboard listeners are loopback-only.
The Scene debug UI and Mapping administration UI are disabled; remote viewing
uses an authenticated SSH tunnel.

The first supported task is:

> “走到前面自动售货机那里”

The production path is:

```text
Chinese speech -> Robonix ASR/Liaison/Pilot
-> semantic_navigation.navigate_landmark("自动售货机")
-> verified pose in the active saved map
-> Robonix navigation service -> Nav2 NavigateToPose
-> guarded /cmd_vel -> Go2 chassis adapter -> Unitree SportClient
```

Motion is **off by default** and boot never arms the chassis. No posture or
low-level motor API is implemented. Read [docs/SAFETY.md](docs/SAFETY.md)
before connecting hardware.

## Clone and inspect safely

```bash
git clone --recursive https://github.com/syswonder/robot-unitree-go2.git
cd robot-unitree-go2
git submodule update --init --recursive
./scripts/validate_offline.sh
```

The validation command is offline: it does not initialize ROS/DDS, start a
Unitree SDK client or contact hardware. See
[known upstream and commissioning limits](docs/KNOWN_LIMITATIONS.md) before
attempting a build or boot.

## What is ready without hardware

- Deployment/Catalog manifests and Soma robot model.
- Pinned official `unitree_ros2` and `unitree_sdk2` git submodules; Unitree
  motion examples are never built or run by this repository.
- Official Go2 model assets with a `base_link` navigation root.
- Fail-closed chassis and sensor adapters with process isolation.
- Conservative RTAB-Map/Nav2 configuration and non-spinning recovery tree.
- Deterministic Chinese landmark resolver and exact-phrase tests.
- Read-only UI for image, lidar, map, pose, odometry and task state.
- Static and offline unit tests; no test publishes a velocity command.

## Hardware-dependent gates

The operator must identify the dedicated wired NIC, verify the real Go2 topic
names/QoS/frames, measure sensor extrinsics, create/load the laboratory map,
and save a verified approach pose in
`config/semantic_landmarks.local.yaml`, including the exact `(map_id,
generation)` from the mapping lifecycle while it is in localization mode.
Until those checks pass, semantic navigation rejects the target and motion
remains disarmed.

See [docs/HARDWARE_CHECKLIST.md](docs/HARDWARE_CHECKLIST.md) for the exact
operator steps and [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) for build/start
commands.

## Licenses

Integration code is Apache-2.0. The aggregate deployment also redistributes
BSD-3-Clause Unitree code/model assets, MIT dependencies and the private
CycloneDDS runtime under EPL-2.0 OR EDL-1.0. See [NOTICE](NOTICE),
[THIRD_PARTY.md](THIRD_PARTY.md), and the authoritative license files retained
in the pinned submodules.
