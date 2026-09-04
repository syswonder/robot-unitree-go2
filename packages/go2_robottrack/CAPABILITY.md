---
description: MiniCPM-RobotTrack D435i RGB following and mutually exclusive velocity-source routing.
---

# Go2 RobotTrack follow capability

Provider ID: `go2_robottrack`

The Robonix primitive namespace is `robonix/primitive/follow`. Initialization
accepts the RobotTrack HTTP, D435i RGB, velocity, and source-mux configuration.
Activation owns one ROS 2 node and one latest-frame inference worker;
deactivation and shutdown stop that runtime.

The optional capability `robonix/primitive/follow/distance` accepts `set`,
`adjust`, and `get` operations. It owns a bounded `0.5..7.0 m` target and
reports the latest aligned RGB-D person distance. The ordinary profile leaves
this controller disabled. The dedicated profile uses confirmed HOG
single-person association, at most three frames of HOG-seeded optical-flow
continuation, and D435i aligned depth to replace only longitudinal speed while
preserving RobotTrack yaw. Optical flow never acquires a target by itself.

The provider does not load or modify the MiniCPM model. It sends the freshest
encoded D435i RGB frame and instruction to the separately running official
`/eval_dual` inference server, accepts finite velocity-only output or a finite
8-by-3 waypoint plan, and applies the official `first` strategy with a 0.1 s
control interval.

`dry-run` is the package default and creates no velocity publisher. `live`
immediately publishes bounded RobotTrack commands to
`/go2/robottrack/cmd_vel_raw`. A mutually exclusive source mux selects either
that stream or `/go2/robottrack/nav_cmd_vel_raw` and is the package's only
publisher to `/cmd_vel_nav`.

The fixed-distance implementation is forward-only: farther than the setpoint
drives bounded forward motion, the deadband stops translation, and too close
holds `vx=0`. It does not weaken the existing chassis rule that rejects reverse
velocity.

The package never calls Unitree SDK APIs and does not select a gait. The
deployment may use `classic_walk`; gait setup remains outside this capability.
