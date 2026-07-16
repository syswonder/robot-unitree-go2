# Deployment

## Prerequisites

- Ubuntu 22.04 x86_64 and ROS 2 Humble.
- Docker Engine for the upstream x86 Scene, Mapping and Navigation package
  targets. Their `jetson-native` manifests are ARM64/JetPack-only and must not
  be selected on this workstation.
- CycloneDDS RMW, Nav2, RTAB-Map, pointcloud-to-laserscan, colcon and CMake.
- Robonix `rbnx`, Python build dependencies, Unitree ROS 2 messages and
  Unitree SDK2.
- A dedicated Go2 Ethernet NIC configured according to the approved runbook.

No install command is run automatically. System package installation and
network changes require a separately approved operator step.

## Prepare local configuration

```bash
git clone --recursive https://github.com/syswonder/robot-unitree-go2.git
cd robot-unitree-go2
git submodule update --init --recursive
cp .env.example .env
```

Edit `.env` locally. At minimum set the real `GO2_NETWORK_INTERFACE`, map ID
and Pilot endpoint. Keep `GO2_ALLOW_MOTION=false` through all read-only tests.

Inspect the PointCloud2 field table and record whether a valid per-point time
field exists. The checked-in mapping and scan-projection profiles keep deskew
disabled until that evidence and a bag-replay test exist; a message header
timestamp is not enough.

The public landmark template is deliberately invalid for movement. After the
map is loaded in localization mode, read its latched identity without
publishing anything:

```bash
timeout 5s ros2 topic echo --once \
  /robonix/map/lifecycle map/msg/MapLifecycle
```

Require `mode: localization`, and copy both `map_id` and `generation` from that
same sample. After the approach pose is physically checked, create the ignored
local file:

```bash
python3 scripts/set_landmark.py \
  --map-id lab_go2 \
  --generation <MAP_LIFECYCLE_GENERATION> \
  --x <MAP_X_METRES> --y <MAP_Y_METRES> --yaw <MAP_YAW_RADIANS> \
  --measured-by <OPERATOR_NAME> \
  --confirm-free-space YES_POSE_AND_FOOTPRINT_ARE_CLEAR
```

Then change the local `.env` entry to
`SEMANTIC_LANDMARKS_FILE=config/semantic_landmarks.local.yaml`. Before that,
the deployment loads the public unverified template and safely rejects the
navigation request. A later map load/reset or mode/generation change also
invalidates the saved pose and cancels unfinished semantic navigation.

## Offline validation and build

```bash
./scripts/validate_offline.sh
./build.sh
```

The validation command parses every YAML/XML file and runs only offline unit
tests. It does not initialize ROS, DDS or Unitree SDK.

`build.sh` compiles only the `unitree_api` and `unitree_go` ROS message
packages from the pinned official submodule. It forces Unitree SDK example
builds off; none of the upstream motion examples are built or run by this
deployment.

The deployment deliberately uses each upstream service's default x86 Docker
manifest. A future Jetson installation needs a separately reviewed deployment
profile; changing the target name alone is not a portable conversion.

After the remote packages are built, a static compatibility gate verifies the
audited Scene lidar/odom/QoS contracts, the Robonix provider bind-host control,
container forwarding of the bind/advertise and CycloneDDS variables, and the
Navigation pending-cancel latch. It writes the actual Robonix, Mapping and
Navigation commits to `rbnx-build/upstream-lock.txt`. Build and start fail
closed until the corresponding upstream changes are present; branch names are
not treated as evidence by themselves.

## Read-only boot

```bash
GO2_ALLOW_MOTION=false ./start.sh
```

All non-DDS endpoints bind only to `127.0.0.1`: Atlas `50051`, Executor
`50061`, Pilot `50071`, Liaison/client entry `50081`, Soma `50091`, reverse
audio `60002`, capability providers on their allocated ports, and the Go2
read-only dashboard on `8092` by default. Scene's debug UI and Mapping's
unauthenticated administration UI are disabled. Mapping visualization is also
disabled; the upstream compatibility gate requires its launcher to avoid X11
probing, mounts and `xhost` entirely in that mode. `DISPLAY` is additionally
removed as defense in depth.

For access from another trusted computer, use an authenticated SSH tunnel;
never expose these ports directly:

```bash
ssh -N \
  -L 50081:127.0.0.1:50081 \
  -L 60002:127.0.0.1:60002 \
  -L 8092:127.0.0.1:8092 \
  <user>@<ubuntu-host>
```

Enter the SSH password interactively and do not save it in this repository.

`start.sh` never changes network state. It fails closed unless the selected
interface is a real, non-Wi-Fi physical NIC that is UP with exactly
`192.168.123.99/24`, no other IPv4 address, IPv6 disabled, and no gateway or
DNS entry in either the kernel routing state or NetworkManager. It then
ignores any inherited `CYCLONEDDS_URI` and unconditionally binds this
deployment's CycloneDDS graph to that one NIC. The Go2 link and ROS domain must
be treated as a trusted, isolated control network; do not bridge or route it to
Wi-Fi, a VPN, or an untrusted LAN.

While stationary, stop the lidar replay/input and confirm the local and global
obstacle layers become non-current within the configured 0.5 s expected update
period. Restore input, place then remove a harmless test object, and verify the
projected `+Inf` rays clear it from both costmaps. These are required checks,
not offline-test claims.

Saved maps, scene data, generated bindings, logs and model/build caches are
kept under the ignored `rbnx-build/` or `logs/` directories in this checkout;
the deployment does not write runtime state into another project directory.
Scene receives one audited host mount at `rbnx-build/data/scene` and keeps its
object-memory database, scene-graph cache and annotations below that mount;
none of those paths is left in the disposable container filesystem.

## Stop

Press Ctrl-C in the foreground deployment or run `./stop.sh` from a second
terminal. Shutdown is scoped to this Robonix deployment; scripts never use a
broad process-name kill.
