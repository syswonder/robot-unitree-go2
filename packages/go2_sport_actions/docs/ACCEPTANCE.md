# Physical acceptance — Go2 EDU

Individual actions were observed on 2026-09-27 (Asia/Shanghai). On 2026-09-28
the operator confirmed that the complete Client voice demonstration met the
expected behavior, all connections were disconnected, and the Go2 was powered
off. This record describes that device and firmware only.

The deployed Robonix runtime banner was v0.1.0 (178fd2a+). This record does not
claim that every later dev version or other firmware was physically tested.
The host uses SDK2 high-level DDS calls; D435i was removed and was not required.
No MuJoCo policy or low-level joint controller was deployed to the real robot.

## Eight demonstrated actions

| Action | First accepted request in the full voice session | Run ID | Observed result |
| --- | --- | --- | --- |
| hello | 2026-09-27 21:53:06 | d7769d8e-ac2f-4a5a-a2b9-eaa1a5851c82 | Visible greeting |
| stretch | 21:53:50 | 3e126a5a-797a-43f6-88c6-5744e3d47a4d | Visible stretch |
| new_year_greeting | 21:54:17 | 18355ab3-6e32-456b-8458-33a407c9856d | Matched remote New Year action |
| crouch | 21:55:18 | 95b89c09-b071-44cd-bd60-db9edfd5992f | Lay down, stood up, stationary |
| dance_1 | 21:56:47 | 773a6c8f-6928-4a89-8243-df78471c71c9 | Complete dance |
| dance_2 | 21:59:55 | 52919a5e-d2c3-4400-9ff1-0fa516e6f250 | Natural finish, normal standing |
| handstand | 22:02:32 | a296f018-71e1-46d2-9cc9-28107c533f6e | Entered and exited handstand |
| handstand_walk | 22:03:37 | 775a68ca-4a62-478f-8c05-49416c75008c | Forward in handstand, then normal standing |

The session continued until 22:18 with repeated operator requests: 20 accepted
requests across the eight actions and two requests refused as action_in_progress.
The refusal happened when the next utterance arrived before the previous
supervision window ended; those two requests did not dispatch another action.

The separately observed 5-second handstand walk at command vx=0.06 m/s was
visibly farther than the 2-second version and recovered normally. Actual
distance was not measured. Dance1/Dance2 also succeeded from normal mode 100.

## Evidence and limits

The local executor log records Client utterances, canonical actions and run IDs.
The daemon log records exact SDK request witnesses, HandStand true/false,
one-way Move emissions and response-bearing StopMove. Raw local logs remain
outside version control; this table is the relevant non-secret extract.

The operator's visual confirmation supplies physical outcome evidence.
Neither a request witness, accepted=true nor SDK_ACCEPTED_COMPLETION_UNVERIFIED
proves that an animation physically completed. Firmware does not provide
a complete per-action visual-success signal here. StopMove acknowledgements
do not establish instantaneous interruption of a vendor animation.

Only the eight listed actions are published. Removed experimental actions
are not part of this acceptance. The release cleanup removes their routes
and protocol IDs without altering the eight accepted motion plans.
