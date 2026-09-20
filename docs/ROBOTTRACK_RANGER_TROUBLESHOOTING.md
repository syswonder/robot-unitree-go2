# Ranger reference: fixed distance stops, RGB-only moves without a target

Source review and offline reproduction: 2026-09-20. This is a porting reference,
not a Ranger hardware acceptance result. Preserve the recipient's existing
stop, timeout, collision protection and single-controller ownership.

## Version clarification

PR #6 includes the RGB follow Primitive, RGB-D fixed-distance controller,
distance set/adjust/get, voice routing and the Go2 mux integration. The original
`robottrack-ranger-reference-2026-09-20.zip` was exported from `7c11eeb`, the
same PR head before this review. Switching from that ZIP to that commit does
not upgrade the distance implementation.

## Reproduced worker starvation and repair

In `go2_robottrack/distance_worker.py`, `submit()` previously incremented the
invalidation epoch for every newly paired camera frame. If a new pair arrived
before the running estimate completed, its result was discarded. Continuous
input faster than estimation could therefore leave no accepted measurements;
the fixed-distance dispatcher held forward velocity at zero.

The repair replaces only the pending pair, while allowing the running estimate
to complete in the same validity epoch. Explicit `invalidate()` and shutdown
still revoke in-flight work. `ros_node.py` still checks freshness using the
original RGB-D receive time, and invalid/no-person/low-confidence results still
clear or reject distance. No timeout is enlarged, no timestamps are refreshed,
and no guard is bypassed. A deterministic regression test submits a newer pair
during each of three consecutive estimates and verifies all three can commit.

This is distinct from the earlier `rgbd_sync.py` once-per-frame pairing fix.
Port both the pairer and the worker semantics, not just a speed setting. The
new regression fails against the old worker and passes after this repair.
It reproduces a possible cause matching the reported symptom; only Ranger
logs can establish whether it is the actual cause on that robot.

## Why disabling fixed distance can cause movement without a person

The ordinary path forwards the model's velocity (subject to its normal limits,
freshness and downstream controls). It does not independently establish that a
person is present. A nonzero model command is not proof of a person detection,
and this symptom alone does not identify DINOv3 as defective.

Fixed-distance mode replaces forward speed using person distance; invalid or
unavailable distance holds **forward speed** at zero. It retains model yaw when
the model command is valid, so this is not a guaranteed whole-chassis stop.
The HOG/flow person-distance estimator can also produce false positives.
Neither mode is a collision-avoidance implementation: distance to a tracked
person does not describe walls, glass doors or obstacles along the path.
Do not use distance-off as a remedy for zero forward speed on the floor.

## Orbbec Gemini 336L / Ranger read-only checks

1. Verify RGB and depth topic names, encoding, units, timestamps, frame IDs,
   registration/alignment and matching camera calibration. A D435i topic or
   depth-unit assumption must not be copied without checking the actual driver.
2. Log pair status/delta, estimator duration, measurement age/status/confidence,
   reported distance and configured target distance. Compare distance with a
   measured person position. Check `no_measurement`, `stale_measurement`,
   `low_confidence`, `within_deadband` and `too_close_forward_hold`.
3. At the same time, compare `linear.x` and `angular.z` at RobotTrack raw,
   selected mux output, smoother output, guard output and Ranger driver input.
   Raw zero points upstream; nonzero before a stage and zero after it points to
   that stage's configuration/state. Inspect its reason instead of removing it.
4. Check actual processing time against the existing freshness limit. This
   repair prevents starvation but does not make genuinely expired images valid.

The recipient must configure their own camera/driver/velocity limits. Do not
execute the Go2 workstation launcher, use Go2 gait IDs, or bypass Ranger's
motion ownership and obstacle handling. First verify commands with motion
disabled; subsequent hardware testing remains supervised by the local operator.
