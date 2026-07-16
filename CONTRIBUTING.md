# Contributing

Keep changes small, reviewable and fail-closed. Before opening a pull request:

1. Run `./scripts/validate_offline.sh`.
2. Confirm no test or helper publishes `/cmd_vel`, `/lowcmd` or
   `/api/sport/request`, invokes a posture API, or starts a Unitree motion
   example.
3. Add an offline regression test for every safety, lifecycle or contract
   change.
4. Update `NOTICE` and `THIRD_PARTY.md` when redistributing another component.
5. Do not commit `.env`, logs, maps, bags, generated builds, credentials,
   calibration tied to a private device, or device handover documents.

Hardware evidence must start with the read-only checklist. A pull request
must state explicitly whether it was validated only offline, with recorded
data, or on supervised physical hardware; never imply hardware validation from
unit tests.
