# RobotTrack fixed-distance and voice-adjustment physical result

Date: 2026-08-09 (Asia/Shanghai)

## Result

The optional D435i RGB-D fixed-distance profile passed supervised physical
acceptance on the same Unitree Go2 EDU used for the existing RobotTrack and
generation-1 navigation trials. The operator confirmed that longitudinal
following, yaw response, the close `1 m` setting, and runtime switching across
the supported distance range all behaved as intended.

The established RGB-only RobotTrack launcher and generation-1 navigation
remain separate. The accepted profile starts at `5.0 m` on every new launch
and may be adjusted at runtime without restarting the stack.

## Physical sequence

The final continuous window ran from `2026-08-09 21:21:02.627771662 CST` to
`21:26:02.792959212 CST`, for `300.164594791 s`.

Runtime distance changes completed inside that window:

| Time (CST) | Command | Result |
| --- | --- | --- |
| `21:22:27.477` | `离我远一点` | `5 -> 6 m` |
| `21:23:37.725` | `靠近一点` | `6 -> 5 m` |
| `21:24:03.275` | set `1 m` | `5 -> 1 m` |
| `21:24:56.593` | set `6 m` | `1 -> 6 m` |
| `21:25:16` | direct MCP set `5 m` | `6 -> 5 m` |

The operator explicitly confirmed that the `1 m` and `6 m` settings were
intentional acceptance checks, not recognition errors. The runtime was
returned to `5.0 m` before the window ended; every new launch also retains that
normal initial target, and future runs can again switch freely within
`0.5..7.0 m`.

## Motion and stop evidence

The post-clamp SDK daemon log contains `11,657` Unitree `Move` requests over
`299.821889 s` (`38.876 Hz`). Consecutive request intervals were
`25.20..33.02 ms`, with no interval above `0.1 s`.

- `vx`: `0..0.50 m/s`, non-zero for `73.946 s`, time integral `22.181 m`;
- `vy`: always `0`;
- `wz`: `-0.30..+0.30 rad/s`, non-zero for `299.826 s`;
- chassis session distance: `16.9967 m` over `300.18 s`.

The test entered exact `ARMED` before the measured window. There was no
chassis fault, RobotTrack fault, process OOM, or command-stream gap during the
window. The end-of-window StopMove acknowledgement was delayed, but the
existing independent stop path completed within `0.596 s`; the measured final
state was `DISARMED` with `daemon_armed=false`. No watchdog limit was relaxed.

The memory-lean distance profile omitted only the unused Scene system after a
full profile reproduced host OOM pressure. Mapping, localization, Nav2 command
smoothing, velocity guard, chassis ownership, D435i, speech, Pilot/Executor,
and the semantic distance route remained present. Across 62 samples the host
had `816.5..1389.1 MiB` available memory and used `5223..5454 MiB` of the
`8188 MiB` GPU memory; no OOM occurred.

## Root cause fixed before acceptance

An earlier trial produced short forward pulses because a new RGB frame could
reuse the previously consumed depth frame. The temporary timestamp mismatch
then cleared the measurement and reset the person tracker every camera cycle.
The pairer now consumes each RGB and depth sequence once and waits for the
matching member without invalidating the current measurement. Recorded-event
replay changed the faulty pattern from `208` mismatch invalidations to exact
same-stamp RGB-D pairs, and package regressions cover the 5 Hz arrival order.

## Evidence boundary

The final five-minute rosbag process did not start because a preflight bag had
already created the same minute-level output directory. That existing
`3.980 s` bag is not presented as the formal run. The motion values above come
from the timestamped event, Robonix provider, chassis adapter, SDK daemon,
speech, Pilot and Executor logs retained locally. The test recorder now uses a
second-and-millisecond timestamp so a rapid retry cannot reuse the directory.

Consequently this report does not claim five-minute odometry-topic statistics
or a formal bag hash. It does claim the measured 300-second chassis command
stream, runtime distance calls, operator-observed follow behavior and final
disarmed state.

## Reuse

Start the official inference service, then use:

```bash
bash scripts/start_workstation_robottrack_distance_follow.sh
```

The launcher defaults to `5.0 m`. Runtime commands include `离我远一点`,
`靠近一点`, `保持5米`, and explicit values such as `设置跟随距离为1米`.
Ordinary RobotTrack follow continues to use
`scripts/start_workstation_robottrack_follow.sh`; neither launcher replaces
the saved generation-1 navigation path.
