# Host-native time-sync templates (inactive)

This directory contains the clock accounting library and an optional
host-native deployment design.  Nothing here is an installer, and every
configuration/unit file is intentionally suffixed `.template`.  There is no
`[Install]` section and no command in this directory enables a service.

The preferred first deployment is `deploy/jetson-time-sync`, which separates
a zero-capability DDS feeder from a networkless `SYS_TIME` chronyd container
and leaves the NX host package/config filesystem unchanged.  Use these host templates only if a later field audit
requires a persistent service and the operator separately approves every
package, file installation, service, and clock-change action.

`go2_clock_ref.py` is subscription-only and defaults to a bounded
`observe-only` run.  Its explicit `feed-chrony` path still cannot start without
an exact enable token, a root-controlled same-session writer approval, a live
GID match against schema-v2 original evidence, and an existing chronyd-owned
SOCK refclock socket.  It never calls
a clock-setting API; chronyd is the only intended clock servo.
