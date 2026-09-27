# Contributing

This standalone package is published from
[robot-unitree-go2/packages/go2_sport_actions](https://github.com/syswonder/robot-unitree-go2/tree/main/packages/go2_sport_actions).
Submit source changes there so Skill and chassis protocol changes remain
reviewable together. Publish the directory to this repository with git subtree
split after its source PR passes checks and is merged. Do not copy generated
bindings, SDK binaries, model weights, logs or credentials.

Run `PYTHONPATH=. python3 -m unittest discover -s tests -q`,
`rbnx validate .`, and the parent robot's offline validation for daemon changes.
Default execution must remain motion-disabled. Preserve runtime stop/cancel,
command timeout, fresh state and single-controller behavior. Report physical
evidence separately from offline tests.

Use the responsible human's Git author and committer identity. Disclose
material AI assistance with `Assisted-by: AGENT_NAME:MODEL_VERSION`; do not
fabricate sign-offs, human reviews or physical testing trailers.
