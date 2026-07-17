# Real full-stack readiness gate

`scripts/check_stack_readiness.sh` is the final read-only check between
`rbnx boot` and any later, separately approved physical-motion gate. Robonix
boot is intentionally best-effort for non-core packages, so seeing its
“components up” line is not proof that the Go2 navigation deployment is ready.

Run it in another terminal after the stack has settled:

```bash
cd /path/to/robot-unitree-go2
bash scripts/check_stack_readiness.sh
```

The script never publishes a ROS message, sends an action goal, invokes a
Robonix task capability, or calls a Unitree command API. Every ROS CLI read is
wrapped by `timeout`. It exits zero only when every check is `PASS`; `FAIL` and
`UNKNOWN` both exit non-zero. A timestamped, mode-0600 JSON evidence report is
written under `logs/readiness/`.

The gate verifies:

- the current deployment's `rbnx-boot/state.json`, live boot process and the
  complete direct-component set (including best-effort services);
- ACTIVE Atlas providers, exact required capabilities, and no namespace
  mismatch or provider error state;
- Atlas, Executor, Pilot, Liaison, Soma, semantic router and dashboard listen
  only on literal loopback addresses;
- semantic router `/v1/models`, dashboard `/api/status` plus `/healthz`, ROS
  connection, and fresh UI telemetry;
- current-session Speech log evidence that streaming ASR initialized as `OK`
  and is not the mock backend;
- fresh, exactly typed camera, point-cloud, IMU, odometry and map messages with
  audited frame IDs and non-stale source timestamps;
- the exact latched `(map_id, localization, generation)` tuple against the
  physically verified vending-machine landmark file;
- fresh `map -> odom -> base_link` TF, the composed map pose, and the static
  lidar/camera/IMU links;
- all Nav2 lifecycle nodes configured by this deployment are `active [3]` and
  exactly one correctly typed server exists for `navigate_to_pose`,
  `compute_path_to_pose`, and `follow_path`.

Current Robonix boot state does not contain a `failures` array. The gate does
not treat that absence as success: it requires every manifest-owned direct
component to be present and alive, then independently requires the Soma-owned
primitives/skill and every service to be ACTIVE in Atlas. Missing or
uninspectable evidence remains `UNKNOWN` and blocks readiness.

This gate is necessary but never grants motion authority. Physical movement
still requires all safety gates in `docs/SAFETY.md` and an explicit staged test
approval with a present operator and working takeover/stop path.
