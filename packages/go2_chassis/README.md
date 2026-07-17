# Go2 chassis adapter

A split-ready Unitree Go2 chassis primitive for ROS 2 Humble and Robonix. The
ROS process consumes `unitree_go/msg/SportModeState`, publishes standard
odometry, IMU, TF and diagnostics, and forwards guarded velocity decisions over
a same-UID Unix `SOCK_SEQPACKET` connection. A separate SDK-only daemon owns the
Unitree `SportClient` so ROS and Unitree's bundled DDS libraries are never
loaded into one process.

The package is motion-disabled by default and has not been physically
validated merely because it builds. Do not enable its motion gates until the
read-only sensor/TF checks, remote takeover, clear test area, conservative
staged test, watchdog, cancel, and stop checks are approved at the robot.

## Components

- `ros2_ws/src/go2_chassis_adapter`: ROS-only adapter; never links SDK2.
- `sdk_daemon`: SDK2-only `Move`/`StopMove` process; no ROS dependency.
- `include/go2_chassis`: wire protocol and independently testable guards.
- `tests`: offline fake-client tests that cannot contact a robot.
- `go2_chassis/runtime_config.py`: strict `Driver(CMD_INIT)` config validation.
- `scripts`: package build/start plus motion-disabled diagnostic wrappers.

Set `UNITREE_ROS2_SETUP` to the setup file containing `unitree_go` messages and
`UNITREE_SDK2_DIR` to the official SDK2 checkout before building. The package
build uses `RBNX_PACKAGE_ROOT`, runs `rbnx codegen -p "$PKG" --ros2`, builds and
sources the generated ROS interface overlay, then builds the ROS-only adapter
and isolated SDK daemon. It does not install missing dependencies and forces
all upstream examples off.

The Robonix start command only launches the Python provider. Per-instance
topics, frames, limits, and gates arrive later through `Driver(CMD_INIT)`; the
provider owns child processes with `Primitive.spawn()`. With the public config,
only the ROS adapter starts, its motion gate is false, and its only allowed
SportModeState mode is the impossible sentinel `255`. The SDK daemon starts
only after all motion gates and an explicit read-only-audited
`GO2_ALLOWED_MODES` process value pass validation. Manual wrappers reject
motion-enabled use.

The adapter accepts `/sportmodestate` as its primary state source and uses
`/lf/sportmodestate` only after the primary stream becomes stale. It publishes
standard odometry, IMU, TF, status, and diagnostics locally. Runtime logs are
written below `rbnx-build/data`; the Unix socket uses a short per-package path
below `/tmp`. See `CAPABILITY.md` for every gate required before any future
physical motion test.

Every accepted state message must carry a well-formed, non-zero source
timestamp within `0.20 s` behind to `0.05 s` ahead of the adapter's ROS clock.
Rejected zero, stale, or future-dated samples neither refresh the motion
watchdog nor publish canonical odometry, TF, or IMU. Canonical outputs retain
the validated Unitree source timestamp; they are never restamped with receipt
time. A detected clock offset must therefore be fixed by synchronizing the
host/robot clocks, not hidden by restamping or relaxing the gate. When motion
is enabled, source timestamps must also strictly advance; duplicate or
regressing timestamps do not refresh the state watchdog. Initial linear motion
is forward-only: the adapter clamps reverse requests to zero and the SDK daemon
independently rejects them.
