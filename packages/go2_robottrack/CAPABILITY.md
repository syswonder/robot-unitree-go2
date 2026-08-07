---
description: MiniCPM-RobotTrack D435i RGB following and mutually exclusive velocity-source routing.
---

# Go2 RobotTrack follow capability

Provider ID: `go2_robottrack`

The Robonix primitive namespace is `robonix/primitive/follow`. Initialization
accepts the RobotTrack HTTP, D435i RGB, velocity, and source-mux configuration.
Activation owns one ROS 2 node and one latest-frame inference worker;
deactivation and shutdown stop that runtime.

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

The package never calls Unitree SDK APIs and does not select a gait. The
deployment may use `classic_walk`; gait setup remains outside this capability.
