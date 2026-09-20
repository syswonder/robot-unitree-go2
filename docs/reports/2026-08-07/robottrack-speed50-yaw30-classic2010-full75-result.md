# MiniCPM-RobotTrack 0.50/0.30 ClassicWalk full 75-second retest

Date: 2026-08-07 (Asia/Shanghai)

## Result

The requested 75-second Go2 visual-follow retest completed successfully with
the already validated RobotTrack configuration: forward limit `0.50 m/s`, yaw
limit `0.30 rad/s`, generation 1 map, Unitree mode `0`, and ClassicWalk marker
`2010`.

The formal window was `10:34:39.374584885` through
`10:35:54.404337106`, or `75.029752221 s`.  The model ran continuously for
the entire formal window.  The chassis was explicitly disarmed afterwards.

## Startup evidence

This was a new Go2/writer boot, so the 2026-08-06 approval was not reused.

- dedicated Go2 Wi-Fi interface on `192.168.123.0/24`, with no default route;
- Go2 and Orin were reachable on that isolated subnet;
- post-boot writer identity:
  `logs/go2-readonly/robottrack-speed50-yaw30-classic2010-75s-20260807T022529Z-identity`;
- post-boot time epoch:
  `logs/go2-readonly/robottrack-speed50-yaw30-classic2010-75s-20260807T022734Z-epoch`;
- boot-specific approval:
  `rbnx-build/run/robottrack-speed50-yaw30-classic2010-75s-20260807T022734Z-approval.json`;
- Robonix runtime: `rbnx-build/run/stg.QQKsaP`.

The AIC8800 USB adapter was present but its old system build targeted kernel
`6.8.0-134`.  The already retained, content-verified `6.8.0-136` modules under
`rbnx-build/aic8800-k6.8.0-136.GnKFgF/` were loaded for this boot only.  No
driver was installed and no default route was changed.

Before arm, the observed robot state was `mode=0`, marker `2010`, zero velocity
and zero yaw speed.  MID-360 cloud was approximately `15.4 Hz`, raw odom about
`149 Hz`, D435i RGB/depth/camera-info about `5 Hz`, and the corrected scan
about `7.7 Hz`.

## Rosbag coverage

The formal bag is:

`docs/reports/2026-08-07/robottrack-speed50-yaw30-classic2010-full75-20260807T103423CST-rosbag/`

It runs from `10:34:34.967805530` through `10:35:58.261829984`, or
`83.294024454 s`: `4.407 s` of pre-roll and `3.857 s` of post-roll around the
complete formal window.  It contains 37,874 messages across all 11 requested
topics.

SHA-256:

- `metadata.yaml`:
  `b1263b159edaecdf363a511a9b8bb6ea3268ce8dccd2d86485b118d1ba0f82ee`;
- SQLite bag:
  `88cb657092338c230c8778c60c1e1679e38378c53affd9ec3fcb2712db1fc18c`.

## Formal-window measurements

| Stream | Samples | Approx. rate | Non-zero commands | Range |
| --- | ---: | ---: | ---: | --- |
| `/go2/robottrack/cmd_vel_raw` | 3,751 | 50.000 Hz | 3,744 (99.813%) | `vx=[0,0.5]`, `wz=[-0.3,+0.3]` |
| `/cmd_vel_nav` | 3,751 | 50.000 Hz | 3,744 (99.813%) | `vx=[0,0.5]`, `wz=[-0.3,+0.3]` |
| `/go2/staged_nav2/cmd_vel` | 1,501 | 20.000 Hz | 1,498 (99.800%) | `vx=[0,0.5]`, `wz=[-0.3,+0.3]` |
| `/odom` | 1,497 | 19.940 Hz | n/a | net `2.7471 m`, path `12.3602 m` |
| `/scanner/scan` | 572 | 7.625 Hz | n/a | live throughout |
| D435i RGB | 375 | 4.996 Hz | n/a | 375/375 content hashes unique |
| corrected sport state | 20,520 | 273.490 Hz | n/a | mode `0`, marker `2010`, gait `0` only |

Raw and selected commands had positive forward velocity in 2,673 of 3,751
samples (`71.261%`, approximately `53.455 s` under zero-order hold); staged
commands had positive forward velocity in 1,153 of 1,501 samples (`76.816%`,
approximately `57.618 s`).  All three streams remained strictly within the
deployed `0.50/0.30` limits.

Odom moved from approximately `(-0.1831, -0.1007)` to
`(2.5472, 0.2025)`.  The 2D net displacement was `2.7471 m`, while accumulated
2D path was `12.3602 m`; heading changed by approximately `-179.4 deg`.  This
is consistent with a follow trial containing substantial turning rather than
a straight-line distance test.

All 375 formal-window chassis status samples were `ARMED`; 374 reported
`motion_in_progress` and one was the initial `armed_zero`.  Diagnostics had no
level-2 error.  The first post-window `DISARMED` sample arrived `0.612235 s`
after the formal end with `daemon_armed=false` and stop reason
`explicit_disarm_or_cancel`.  The bag ended in `DISARMED`.

After motion ended, the opaque marker changed from `2010` to `100`.  This
occurred outside the formal motion window and is the already accepted firmware
state marker transition, not an application error.

The disarm helper returned status 1 because the daemon did not acknowledge its
StopMove receipt, matching the earlier tested behavior.  The independent
status stream is the measured proof that the chassis disarmed.

## Monitor

The local monitor at `http://127.0.0.1:5801/web` was opened and placed in the
foreground.  Raw camera and model/overlay stream hashes changed across samples;
`step` and `camera_frame_seq` advanced, all three frame flags were true, and
warm inference was approximately `66--72 ms` during the browser check.

After inspection, the operator manually placed the Go2 in its lying posture and
requested that the test end.  The Robonix follow stack then completed its normal
teardown, and the model service and read-only D435i bridge were stopped.  A
process and listening-port check found none of those test services remaining.
No posture API was called by the integration.  The dedicated Wi-Fi connection
was left unchanged for the current boot, while all logs and the formal bag were
retained.
