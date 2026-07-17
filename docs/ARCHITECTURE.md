# Architecture

## Command path

```text
Robonix Liaison/Pilot
  -> semantic_navigation.navigate_landmark(name)
  -> map-id + verified-pose gate
  -> Atlas resolves robonix/service/navigation/navigate
  -> service-navigation-rbnx
  -> Nav2 NavigateToPose
  -> planner + controller + live costmaps
  -> bounded velocity_smoother (0.25 s timeout)
  -> /cmd_vel_guard_input -> navigation velocity guard -> /cmd_vel
  -> ROS-only go2_chassis_adapter
  -> local authenticated Unix SOCK_SEQPACKET
  -> SDK-only go2_sport_daemon
  -> Unitree SportClient::Move / StopMove
```

No process links ROS 2 Humble's CycloneDDS and Unitree SDK2's bundled
CycloneDDS into the same address space. The SDK daemon has its own watchdog,
validates the local peer UID and packet schema, and knows no ROS topic. The
ROS adapter has no Unitree SDK dependency.

Outside DDS, Robonix system services, capability providers, reverse audio and
the read-only dashboard all bind to `127.0.0.1`. The deployment owns those
values after loading local configuration, and a static compatibility gate
requires the upstream bind-host implementation and container forwarding.
Scene's debug UI and Mapping's unauthenticated administration UI remain off.

The dashboard's optional browser voice entry is a narrow exception to a
purely observational UI, not a hardware command surface. It is disabled by
default and accepts only strict 16 kHz mono PCM WAV from its own loopback
origin. The handoff path is browser → dashboard memory → the pinned local
`audio_client_bridge` → `robonix/system/liaison/voice`. Liaison remains the
only ASR/Pilot entry; the dashboard has no navigation client, ROS publisher,
or Unitree dependency. The telemetry capabilities remain read-only and the
independent chassis motion gates are unchanged.

## State and localization

```text
/sportmodestate
  -> position/velocity/quaternion validation
  -> IMU-oriented /odom + odom->base_link
  -> /imu/data -> read-only sensor relay -> /scanner/imu (health/UI)

/utlidar/cloud + /odom
  -> RTAB-Map mapping/localization
  -> /map + map->odom
  -> Nav2 local/global costmaps
```

The chassis owns `odom -> base_link`; localization owns `map -> odom`. There
must never be two publishers for either transform.

Lidar deskew is disabled until a read-only audit proves usable per-point
timestamps on the target firmware. Both RTAB-Map and the Nav2 scan projector
remain functional without deskew; they must not guess timestamps from the
PointCloud2 header.

Soma serves the pinned Go2 URDF to the local `robot_description` primitive.
That primitive rejects any byte mismatch, then the standard
`robot_state_publisher` publishes the remaining fixed joints on `/tf_static`
under CycloneDDS.

## Sensor path

The read-only sensor package relays the onboard PointCloud2 to
`/scanner/cloud`, republishes the validated chassis IMU under the sensor
provider, and isolates Unitree VideoClient from ROS through a second local IPC
bridge. The dashboard subscribes only; it has no publisher or motion control.

## Semantic target

The public file `config/semantic_landmarks.yaml` is an unverified template.
The ignored local file contains the real saved approach pose. Matching is
Unicode-normalized longest-alias matching. It refuses unknown, ambiguous,
unverified, non-finite, wrong-frame, wrong-map and wrong-generation targets
before the navigation service is called. The skill resolves the mapping
service's `robonix/service/map/lifecycle` contract through Atlas, subscribes to
its transient-local `map/msg/MapLifecycle` topic, and requires the live
`(map_id, generation)` in `localization` mode before every goal. A lifecycle
change marks unfinished semantic runs failed and requests cancellation through
the navigation service; the skill does not publish a ROS or chassis command.
Its optional dashboard notifier sends display-only task phases over a validated
literal-loopback HTTP endpoint on a best-effort worker; it disables proxies and
redirects, and dashboard availability is not part of navigation acceptance.
Only one semantic navigation may be outstanding. The latch is released solely
by an explicit provider `SUCCEEDED`, `CANCELED`, or `FAILED` status—not by a
local semantic failure or an accepted asynchronous cancel request. Cancellation
therefore retries while polling status for a bounded window; a timeout remains
fail-closed and visibly high-risk until a later provider terminal confirmation.
