# MiniCPM-RobotTrack 0.50/0.30 ClassicWalk full 45-second result

Date: 2026-08-06 (Asia/Shanghai)

## Result

The formal Go2 visual-follow run passed with the RobotTrack forward limit set
to `0.50 m/s`, the RobotTrack yaw limit restored to `0.30 rad/s`, and the
existing ClassicWalk state retained.  The recorded formal window was
`22:31:21.017066446` through `22:32:06.064883275`, or `45.047816829 s`.

This result supersedes the earlier statement that only turn following had
passed.  It does not erase the earlier bags or operator observations: those
remain valid records of the lower-speed and incomplete attempts.

## Configuration and runtime

- generation 1 map: `go2_second_location_refined_20260724_01`;
- allowed Unitree mode: `0`;
- requested and observed state marker during the formal window: `2010`;
- RobotTrack command limits: `vx <= 0.50 m/s`, `vy = 0`,
  `abs(wz) <= 0.30 rad/s`;
- Robonix run directory: `rbnx-build/run/stg.18MLaw`;
- post-reboot identity evidence:
  `logs/go2-readonly/robottrack-speed50-yaw30-classic2010-20260806T142320Z-identity`;
- post-reboot time epoch:
  `logs/go2-readonly/robottrack-speed50-yaw30-classic2010-20260806T142518Z-epoch`;
- approval used by that boot:
  `rbnx-build/run/robottrack-speed50-yaw30-classic2010-20260806T142518Z-approval.json`.

The approval and writer identity are boot-specific evidence and must not be
reused after the robot or relevant writers restart.

## Rosbag coverage

The bag is:

`docs/reports/2026-08-06/robottrack-speed50-yaw30-classic2010-full45-20260806T223105CST-rosbag/`

It runs from `22:31:17.648874579` through `22:32:09.823588614`, covering the
formal window with `3.368 s` of pre-roll and `3.759 s` of post-roll.  It
contains 23,632 messages and all 11 requested topics.  The map lifecycle
message is in pre-roll; the other ten streams cover the formal window.

SHA-256:

- `metadata.yaml`:
  `ec544c87b512c69b5ece80429b8ec46a08302111687fb0acd086817da36b9b3d`;
- SQLite bag:
  `cb5d25b10a4aeb934c03cc081043e5332b4718974c2875f9ef5b6998f0d9c4bf`.

## Formal-window measurements

| Stream | Samples | Approx. rate | Non-zero commands | Range |
| --- | ---: | ---: | ---: | --- |
| `/go2/robottrack/cmd_vel_raw` | 2,252 | 50.000 Hz | 2,244 (99.645%) | `vx=[0,0.5]`, `wz=[-0.3,+0.3]` |
| `/cmd_vel_nav` | 2,252 | 50.000 Hz | 2,244 (99.645%) | `vx=[0,0.5]`, `wz=[-0.3,+0.3]` |
| `/go2/staged_nav2/cmd_vel` | 901 | 20.000 Hz | 897 (99.556%) | `vx=[0,0.5]`, `wz=[-0.3,+0.3]` |
| `/odom` | 910 | 20.202 Hz | n/a | net `11.3616 m`, path `12.4467 m` |
| `/scanner/scan` | 344 | 7.628 Hz | n/a | live throughout |
| D435i RGB | 225 | 4.996 Hz | n/a | 225/225 content hashes unique |
| corrected sport state | 12,073 | 267.985 Hz | n/a | mode `0`, marker `2010` only |

The raw/selected stream had positive `vx` for 2,183 of 2,252 samples
(`96.94%`) and about `43.656 s` under zero-order hold.  The staged stream had
positive `vx` for 895 of 901 samples (`99.33%`) and about `44.727 s` under
zero-order hold.  All three command streams stayed within the requested
`0.50/0.30` limits.

Odom moved from approximately `(-1.15879, -0.03004)` to
`(10.04690, 1.84579)`: 2D net displacement `11.36161 m` and accumulated 2D
path `12.44669 m`.  Heading changed only about `-2.10 deg`, consistent with a
predominantly forward follow run rather than an in-place-turn-only result.

## Stop and UI evidence

All 225 chassis status samples in the formal window were `ARMED`; 224 reported
`motion_in_progress`.  The first post-window `DISARMED` status arrived
`0.564961 s` after the formal end, with stop reason
`explicit_disarm_or_cancel`.  The disarm helper returned status 1 because the
daemon did not acknowledge its StopMove receipt, but the independent status
stream then explicitly proved `daemon_armed=false` and remained `DISARMED`
through the end of the bag.

The D435i RGB stream contained 225 distinct frames during the formal window.
After the run, the local monitor continued updating: `step` and
`camera_frame_seq` advanced, raw/model-input/overlay flags were all true, and
warm inference remained approximately `63--103 ms`.

At the time this report was written, the full stack, D435i bridge and model
service were intentionally left running for operator inspection, but the
chassis was `DISARMED`.  Treat that as a dated runtime snapshot, not a future
startup assumption.
