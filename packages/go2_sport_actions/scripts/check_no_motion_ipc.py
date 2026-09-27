# SPDX-License-Identifier: Apache-2.0
"""Check wire compatibility against a daemon started without --allow-motion."""

import sys

from go2_sport_actions.ipc import ARM, DaemonClient, DaemonError


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: check_no_motion_ipc.py SOCKET_PATH")
    with DaemonClient(sys.argv[1]) as daemon:
        try:
            daemon.call(ARM)
        except DaemonError as exc:
            if "code=-4" not in str(exc):
                raise
        else:
            raise AssertionError("no-motion daemon unexpectedly accepted ARM")
    print("no-motion daemon IPC: PASS; ARM rejected, no SDK initialized")


if __name__ == "__main__":
    main()
