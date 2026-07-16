---
description: Fail-closed Unitree Go2 chassis topics for guarded Nav2 control.
---

# Unitree Go2 chassis capability

This package provides the standard Robonix chassis contracts needed by Nav2:

- `robonix/primitive/chassis/twist_in` maps to `/cmd_vel`.
- `robonix/primitive/chassis/odom` maps to `/odom`.
- `robonix/primitive/chassis/driver` manages registration only.

It intentionally does not expose posture, stand, sit, low-level control, or a
single-shot movement RPC. Nav2 is expected to be the only velocity controller.

## Safety state machine

The committed configuration starts as `DISABLED`. A motion-capable deployment
requires all of the following independent gates:

1. Driver config contains `allow_motion=true`, `operator_present=true`, the
   exact safety acknowledgement, and an explicit network interface.
2. `GO2_ALLOWED_MODES` explicitly lists modes measured during read-only audit;
   the public/default list is the impossible sentinel `[255]`.
3. The SDK daemon receives its internal `--allow-motion` gate only after the
   provider validates the preceding inputs.
4. The ROS adapter receives `allow_motion:=true`.
5. A fresh, numerically valid `SportModeState` has an allowed mode, no reported
   error, a non-zero source timestamp, and a source timestamp that strictly
   advances on every accepted state while motion is enabled.
6. A fresh zero `/cmd_vel` is held for at least 0.5 seconds.
7. An operator explicitly calls the SetBool arm service with `data=true`.

Per-instance topics, limits, and motion acknowledgements are delivered through
`Driver(CMD_INIT)`. `GO2_ALLOWED_MODES` is a separate, explicit process
environment input populated only from the read-only robot audit. Process
startup itself has no per-instance config and therefore starts only the Python
provider. Neither configuration nor daemon startup performs the arm action.
The provider must itself start with CycloneDDS already bound to the approved
Go2 NIC through `CYCLONEDDS_URI`; this cannot be deferred to `CMD_INIT`
because the provider's ROS context is created before per-instance config
arrives.

State or command staleness, a zero/repeated/regressing motion-state timestamp,
invalid numbers, disallowed mode, odometry jumps, IPC failure, SDK failure,
controller disconnect, shutdown, and daemon watchdog expiry all fail closed. A
repeated or regressing timestamp cannot refresh the state watchdog. A fault is
latched in the ROS process until an explicit disarm followed by a new arm
sequence. Read-only mode continues to accept zero or repeated source timestamps
for firmware compatibility while the robot is stationary.

The initial limits are 0 to 0.25 m/s forward (reverse is rejected), no lateral
motion, 0.40 rad/s yaw,
0.30 m/s² linear acceleration, and 0.80 rad/s² angular acceleration. These are
defense-in-depth limits in both processes, not a claim that physical motion has
been validated.
