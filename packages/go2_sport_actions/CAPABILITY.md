# Go2 named sport-action Skill

Use this Skill for explicit Go2 actions through the existing single-owner
chassis daemon. Supported names: hello, stretch, new_year_greeting, crouch,
dance_1, dance_2, handstand, handstand_walk. There is no executable bow,
sway, front_jump or backflip. This is a Go2-specific Skill, not a generic
navigation or low-level motor controller.

Chinese commands, in the accepted demonstration order:
“打个招呼”, “伸展一下”, “拜年”, “卧倒再起身”, “跳第一支舞”,
“跳第二支舞”, “做个倒立”, “倒立向前走”.
Each utterance must request one action. “停止动作” and “停下” cancel the current
run; negative utterances, questions and multi-action sentences must not start it.

Call list/preview to inspect without motion. execute(name, request_id) or
execute_utterance(text, request_id) returns acceptance and run_id. Poll status;
do not issue the next action just because the previous request was accepted.
Nonempty request_id deduplicates within this process only. Concurrent actions
are rejected. cancel(run_id) is a request; check stop_acknowledged afterward.

The handstand-walk plan enters HandStand(true), supervises 8 seconds, sends
Move at vx=0.06 m/s for 5 seconds, sends StopMove, then HandStand(false)
and supervises 4 seconds. Other duration windows are documented in README.
These durations are command supervision, not measured animation completion.

The operator observed all eight actions individually on 2026-09-27 and
confirmed the complete Client voice sequence on 2026-09-28 on one Go2 EDU.
Other devices and firmware need their own acceptance. Runtime reports
SDK_ACCEPTED_COMPLETION_UNVERIFIED, never physical success inferred solely
from dispatch. Exact witnessed SDK timeout dispatch is not remote completion.

The managed backend starts the existing Go2 chassis SDK daemon on demand.
Default boot/list/preview do not arm motion. It uses the existing operator
acknowledgement, fresh state, command timeout, one controller and stop paths.
Run this deployment separately from motion-enabled navigation/following.
StopMove may not instantly interrupt vendor animation; the operator retains
the remote. D435i, maps, model weights and a particular scene are not needed.
