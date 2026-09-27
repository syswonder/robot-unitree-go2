# SPDX-License-Identifier: Apache-2.0
"""Launch the existing audited Go2 daemon for a dedicated Skill session.

The Skill and navigation never share a live session.  The daemon itself keeps
the 300 ms watchdog, one IPC peer, fresh SportModeState and StopMove cleanup.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import re


MOTION_ACK = "GO2_PHYSICAL_MOTION_APPROVED"
STAGED_ACK = "I_APPROVE_GO2_STAGED_NAV2_MOTION"
PROFILE = "workstation-staged-nav2-corrected-v1"


@dataclass(frozen=True)
class ManagedDaemon:
    socket_path: Path
    binary: Path
    network_interface: str

    @classmethod
    def from_config(cls, config: dict, socket_path: str) -> "ManagedDaemon":
        package_root = Path(__file__).resolve().parents[1]
        default_binary = (package_root.parent / "go2_chassis" / "rbnx-build"
                          / "sdk" / "install" / "bin" / "go2_sport_daemon")
        binary = Path(str(config.get("daemon_binary") or default_binary))
        interface = str(config.get("network_interface") or "").strip()
        if not socket_path or not Path(socket_path).is_absolute():
            raise ValueError("managed backend requires an absolute daemon_socket")
        if not binary.is_absolute():
            raise ValueError("daemon_binary must be absolute")
        if not re.fullmatch(r"[A-Za-z0-9_.:-]+", interface):
            raise ValueError("network_interface is missing or invalid")
        return cls(Path(socket_path), binary, interface)

    def argv(self) -> list[str]:
        # Reuse the already-audited long-session daemon envelope. The Skill's
        # handstand-walk plan requests 0.06 m/s, below this envelope's 0.30.
        # There is no Nav2 controller in the dedicated action session.
        return [
            str(self.binary), "--socket", str(self.socket_path),
            "--watchdog-ms", "300", "--max-vx", "0.30",
            "--max-vy", "0", "--max-wz", "0.40",
            "--max-motion-ms", "0", "--allow-motion",
            "--interface", self.network_interface,
            "--motion-ack", MOTION_ACK,
            "--motion-profile", PROFILE,
        ]

    def environment(self) -> dict[str, str]:
        # SDK2 ships its own CycloneDDS. Do not mix its libddscxx with the ROS
        # overlay inherited by Robonix Client/Speech.
        return {"LD_LIBRARY_PATH": str(self.binary.resolve().parent.parent / "lib")}

    def ready_to_activate(self, environ: dict[str, str] | None = None) -> bool:
        values = os.environ if environ is None else environ
        return (values.get("GO2_OPERATOR_PRESENT") in {"1", "true", "TRUE"}
                and values.get("GO2_STAGED_NAV2_RUNTIME_ACK") == STAGED_ACK)
