---
description: Read-only Go2 web telemetry and health dashboard provider.
---

# Go2 telemetry dashboard

This service owns a child web-dashboard process and exposes one read-only
status capability. It observes camera, lidar, map, TF, odometry, and Nav2
action-status data. It has no robot command capability.

An optional browser push-to-talk route is disabled unless
`GO2_DASHBOARD_BROWSER_VOICE_ENABLED=1`. It accepts only bounded same-origin
loopback PCM WAV, feeds the pinned local `audio_client_bridge`, and invokes the
official `robonix/system/liaison/voice` server-streaming contract. It doesn't
call ASR, navigation, ROS command APIs, or Unitree directly, and it doesn't add
an Atlas command capability to this provider.

`robonix/service/telemetry/dashboard/status` returns the web URL and a bounded
JSON health snapshot. Calling it never starts a task or changes ROS state.
