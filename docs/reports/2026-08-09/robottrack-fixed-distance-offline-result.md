# RobotTrack fixed-distance follow offline result

Date: 2026-08-09 (Asia/Shanghai)

Historical status: this file records the pre-hardware checkpoint. The later
supervised physical result is
`robottrack-fixed-distance-voice300-physical-result.md`; keep this report for
the offline design and replay evidence rather than treating its remaining-work
section as current state.

## Result

An opt-in forward fixed-distance layer is implemented in the independent
RobotTrack PR worktree. This is an offline result only: no new D435i depth
measurement or physical Go2 motion has been claimed.

The existing generation-1 navigation and ordinary RobotTrack follow launchers
remain unchanged in function. Ordinary follow explicitly renders
`distance_enabled=false`; the new
`scripts/start_workstation_robottrack_distance_follow.sh` renders only the
additional fixed-distance profile with an initial `5.0 m` target.

## Data path

```text
D435i RGB -> RobotTrack model ---------------------------> model wz
       + aligned depth -> confirmed person box -> depth -> measured Z
                                                            |
Robonix follow/distance setpoint ---------------------------+
                                                            v
                                  proportional forward vx + model wz
                                                            |
                         existing source mux/smoother/guard/chassis path
```

- Target range: `0.5..7.0 m`; default `5.0 m`.
- Dedicated live deadband: `0.10 m`; proportional gain: `0.50`. These values
  supersede the first physical-run values (`0.25 m` / `0.25`) after that run
  showed an imperceptible `0.028 m/s` request at a measured `5.36 m`.
- Dedicated live maximum: `0.50 m/s` forward, `0.0 m/s` reverse.
- RobotTrack continues to supply yaw; the distance layer replaces only `vx`.
- Missing, stale, future, low-confidence, invalid, or cleared measurement sets
  forward speed to zero. Missing/stale model plans remain strict full zero.
- RGB-D work is latest-only and asynchronous. Pair submission or invalidation
  advances an epoch, so an old in-flight result cannot restore a cleared
  measurement. Result age is based on the original pair receive time.

The established chassis path rejects negative `vx`, so this implementation does
not add reverse motion. If the target is too close, the Go2 stops translating
forward. Increasing the setpoint waits for the person to open the gap;
decreasing it drives forward toward the new setpoint.

## Robonix and voice control

The new MCP contract is `robonix/primitive/follow/distance`, exposed to Pilot
as `go2_robottrack.follow_distance`. It supports `set`, `adjust`, and `get`.

- `离我远一点` -> `adjust +1.0 m`
- `靠近一点` -> `adjust -1.0 m`
- `保持5米` / `跟随距离设为5米` -> `set 5.0 m`

Relative adjustments are issued once. Executor feedback terminates the turn,
so history replay cannot add or subtract the same metre again. An active
semantic-navigation run retains its existing status/cancel flow.

## Saved-bag person-box replay

Source:

`packages/robot-unitree-go2/docs/reports/2026-08-07/robottrack-speed50-yaw30-classic2010-full75-20260807T103423CST-rosbag`

The bag supplies `416` RGB frames at about `5.001 Hz`, resolution `320x240`,
but no aligned depth. A uniform synthetic `3 m` depth image was used only to
isolate person-box coverage and CPU time. It does not validate distance
accuracy.

| Detector path | Controller-acceptable frames | Mean time |
| --- | ---: | ---: |
| Original HOG (`scale=1.05`, stride `8`) | `145/416` (`34.86%`) | about `2.7 ms` |
| New HOG/flow without small-image enlargement | `236/416` (`56.73%`) | about `10.8 ms` |
| Current 384-width normalization, no flow | `265/416` (`63.70%`) | `16.61 ms` |
| Current 384-width normalization plus flow | `276/416` (`66.35%`) | `16.47..17.3 ms` |

The distant-person segment at frames `384..411` improved from `6/28` to
`28/28`. New targets require two consistent HOG frames. Confirmed boxes may be
continued with forward/backward Lucas-Kanade flow plus a RANSAC transform for
at most three missed frames (`0.600 s` in this bag). The tracker cannot acquire
a target by itself. For tracked boxes with adjacent HOG references, IoU was
minimum `0.371`, median `0.651`.

The total-frame percentage is not a person-visible recall: roughly 75--78% of
frames appeared to contain a complete or mostly visible person in a coarse
contact-sheet review. The RGB-only replay also found one continuous eight-frame
empty-scene false box. HOG confidence, box geometry, and image motion overlap
real-person distributions, so no guessed RGB threshold was added. The physical
check must record real depth, estimated physical box height, and foreground vs
surrounding depth before deciding whether another filter is justified.

## Offline verification

- `go2_robottrack`: `101/101` tests passed.
- `semantic_intent_router`: `43/43` tests passed.
- RobotTrack manifest/launcher: `21/21` tests passed.
- `rbnx codegen --mcp --ros2`, package build, and `rbnx validate` passed.
- The real generated MCP request/response types imported and handled an
  `adjust +1.0 m` request (`5.0 -> 6.0 m`).
- Python compile checks, shell syntax checks, and `git diff --check` passed.
- The repository offline validator now runs the full RobotTrack suite when
  OpenCV/numpy are present and the non-vision RobotTrack core when they are not;
  it installs no dependency. The complete repository run passed `815` tests.

## Physical work still required

After the Go2, router, Wi-Fi adapter, and D435i are available, first inspect the
aligned RGB/depth stream without motion and record target/empty-scene values at
approximately `3 m`, `5 m`, and `6 m`. In particular, verify the eight-frame
RGB false-box class against real depth and confirm that `5 m` has sufficient
valid depth samples. Then run the dedicated profile under the already working
remote takeover, stop, watchdog, and single-controller path.

Do not treat this report as a physical fixed-distance acceptance.
