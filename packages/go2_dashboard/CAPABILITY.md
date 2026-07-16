---
description: Read-only Go2 web telemetry and health dashboard provider.
---

# Go2 telemetry dashboard

This service owns a child web-dashboard process and exposes one read-only
status capability. It observes camera, lidar, map, TF, odometry, and Nav2
action-status data. It has no robot command capability.

`robonix/service/telemetry/dashboard/status` returns the web URL and a bounded
JSON health snapshot. Calling it never starts a task or changes ROS state.
