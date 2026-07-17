# Jetson/NX read-only deployment

This profile is the first deployable hardware gate for the Go2 EDU 100 TOPS NX
module.  It is an ARM64, Ubuntu 22.04, ROS 2 Humble **CPU-only** container that
is compatible with a JetPack 5/L4T R35 host.  It does not use the NVIDIA
runtime or GPU.

It is deliberately not the final autonomous-navigation deployment.  It only
contains:

- the official `unitree_api` and `unitree_go` ROS interface packages;
- `go2_chassis_adapter` with immutable `allow_motion: false` parameters;
- the lidar/IMU relay and optional read-only VideoClient camera bridge;
- the pinned Go2 description and `robot_state_publisher`.

There is no chassis SDK daemon, Unitree motion example, Nav2, mapping, scene,
speech, dashboard, semantic navigation, or Robonix control service.  The
passive chassis process does not create an SDK IPC client, `/cmd_vel`
subscription, arm service, or control timer when motion is disabled.  No
component in this profile publishes velocity commands.

## Fail-closed host assumptions

Both the host launcher and container validate, without changing networking:

- architecture is `aarch64`;
- interface is exactly `eth0` and has carrier;
- its only global IPv4 address is exactly `192.168.123.18/24`.

Any mismatch exits before DDS or the VideoClient starts.  Fixing a mismatch is
a separate, explicitly approved network operation; none of these scripts uses
`nmcli`, Netplan, DHCP, `ip ... add`, or `ip link set`.

## Native build on the NX

Docker Engine is the only orchestrator.  Docker Compose is not used or needed.
The NX Docker daemon may be unable to pull through an SSH loopback proxy, and
`docker save/load` may not retain `RepoDigests`.  Therefore this profile uses
two independent records:

1. the reviewed official upstream `ros:humble-ros-base-jammy` digest;
2. the exact ARM64 image ID, embedded in a local tag and rechecked on the NX.

On the networked staging machine, pull the ARM64 image after approval, record
its digest and ID, create the content-named alias, then save that alias.  The
commands below illustrate the data flow; use the digest already captured by
your deployment audit rather than inventing one:

```bash
docker pull --platform linux/arm64 ros:humble-ros-base-jammy
docker image inspect --format '{{json .RepoDigests}} {{.Id}} {{.Architecture}}' \
  ros:humble-ros-base-jammy
image_id='sha256:<64-hex-image-id>'
local_alias="robonix-local/ros-humble-ros-base-jammy:sha256-${image_id#sha256:}"
docker tag "$image_id" "$local_alias"
docker save --output ros-humble-arm64.tar "$local_alias"
```

Transfer the tar and its separately recorded checksum, verify that checksum,
then `docker load` it on the NX.  Set all three gates before building:

```bash
export JETSON_READONLY_ROS_IMAGE='robonix-local/ros-humble-ros-base-jammy:sha256-<64-hex-image-id>'
export JETSON_READONLY_ROS_IMAGE_ID='sha256:<64-hex-image-id>'
export JETSON_READONLY_ROS_UPSTREAM_DIGEST='sha256:<reviewed-upstream-digest>'
```

`build.sh` rejects a tag that does not contain the complete expected image ID,
an absent local image, a non-Linux/non-ARM64 image, or an ID mismatch.  It uses
`--pull=false`, so `FROM` resolves the loaded alias without registry access.
The final image records both the upstream digest and loaded image ID as OCI
base-image labels.

If package downloads require a proxy, export `HTTP_PROXY`, `HTTPS_PROXY`, and
`NO_PROXY` (or their lowercase forms) before the build.  They are forwarded
only as Docker's predefined proxy arguments and are not declared in the
Dockerfile, so their values do not enter image history or cache metadata.  The
build uses host networking so an explicit NX-side loopback proxy (for example
an approved SSH reverse tunnel at `http://127.0.0.1:7897`) is reachable inside
build steps.  To avoid leaking credentials or accepting an accidental campus
proxy, the script accepts only unauthenticated loopback HTTP(S) proxy URLs and
validates their ports.  The laptop-only `127.0.0.1` proxy is not automatically
the NX loopback; establish the approved tunnel on the NX first or unset the
proxy variables.

The build replaces the pinned base image's plain-HTTP Ubuntu Ports endpoint
with the HTTPS Tsinghua University Ubuntu Ports mirror registered by Ubuntu as
an official mirror, and upgrades the ROS repository URL to HTTPS.  APT's
signed metadata and package verification remain enabled.  Builder and runtime
package downloads use the same locked BuildKit cache, so the two stages do not
compete for a small proxy and successfully downloaded archives survive a
failed build attempt.  Those cache mounts are builder-local and are never
copied into the runtime image.  Network operations still have bounded retries
and timeouts.

Then run from the repository root on the JetPack 5 host:

```bash
./deploy/jetson-readonly/validate-static.sh
./deploy/jetson-readonly/build.sh
```

`build.sh` refuses non-ARM64 hosts.  The Dockerfile-specific ignore file gives
the builder only the five ROS packages and the minimum vendor SDK files needed
to link the camera reader.  Vendor examples are absent.  The final stage gets
only the camera executable and its private DDS libraries; the SDK archive and
headers stay in the discarded builder stage.  A symbol/string gate rejects a
camera binary that accidentally pulls SportClient, MotionSwitcher, LowCmd, or
sport-request code.

To transfer a built image through an explicitly chosen file, use:

```bash
./deploy/jetson-readonly/export.sh /chosen/path/robonix-go2-readonly.tar
```

The exporter refuses to overwrite a file, checks ARM64 architecture, uses a
private umask, and emits a SHA-256 sidecar.  On the target, loading is the
standard explicit operation:

```bash
docker load --input /chosen/path/robonix-go2-readonly.tar
```

## Run and stop

The safe default does not invoke the Unitree video RPC:

```bash
./deploy/jetson-readonly/run.sh
```

After separate approval for camera reads, use:

```bash
./deploy/jetson-readonly/run.sh --camera
```

Stop with:

```bash
./deploy/jetson-readonly/stop.sh
```

The launcher uses host networking for CycloneDDS but sets all Linux
capabilities to none, enables `no-new-privileges`, forces the ordinary `runc`
CPU runtime, uses a read-only root filesystem, and provides only a bounded
`noexec,nosuid,nodev` `/tmp`.  It adds no devices, bind mounts, Docker socket,
host IPC/PID namespace, restart policy, or persistent log driver.  It runs as
UID/GID 10001.  The container is `--rm` and writes only ephemeral PID files,
ROS logs, the generated robot-description parameter file, and camera IPC under
its tmpfs.

The image health check verifies process liveness, the immutable motion gate,
and the exact interface/address.  It intentionally does not report that robot
state is semantically safe: source-clock skew, `SportModeState.error_code`, TF
coverage, sensor extrinsics and calibration remain separate acceptance gates.
The current read-only hardware sample reports `SportModeState.error_code=100`.
That does not prevent this passive inventory runtime from running, but it is a
hard fail-closed blocker for adding Nav2 or any motion-capable process until
Unitree's meaning is identified and the state is measured healthy.

## Expected read-only outputs

- `/odom`, `odom -> base_link`, `/imu/data`, chassis diagnostics, if a fresh
  valid `SportModeState` is received;
- `/scanner/cloud` and `/scanner/imu`, only for fresh source stamps;
- `/tf_static` from the pinned description;
- with `--camera`, `/camera/color/image_raw` and uncalibrated camera info.

An earlier physical audit found a large clock offset.  After correcting the NX
clock, remeasure source stamps through this read-only profile before accepting
the clock gate.  The guards still reject stale samples; do not restamp data or
relax the bounds to make this profile appear healthy.
