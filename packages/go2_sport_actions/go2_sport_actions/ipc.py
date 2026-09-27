# SPDX-License-Identifier: Apache-2.0
"""Client for the existing Go2 chassis daemon's local, single-owner socket.

The packet layout is go2_chassis/protocol.hpp version 1.  This module does
not initialize DDS or create another Unitree SportClient.
"""

from __future__ import annotations

from dataclasses import dataclass
import socket
import struct
import time


MAGIC = 0x47324348
VERSION = 1
COMMAND = struct.Struct("<IHHQQQBBHfffI")
REPLY = struct.Struct("<IHHQiBBHI")
ARM, DISARM, MOVE, STOP, PING, RESTORE_CLASSIC, SPORT_ACTION = range(1, 8)
OK = 0


class DaemonError(RuntimeError):
    pass


def _checksum(packet: bytes) -> int:
    value = 2166136261
    for byte in packet:
        value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
    return value


def make_command(
    operation: int,
    sequence: int,
    *,
    action_id: int = 0,
    vx: float = 0.0,
    vy: float = 0.0,
    wz: float = 0.0,
    now_ns: int | None = None,
) -> bytes:
    if operation not in (ARM, DISARM, MOVE, STOP, PING, RESTORE_CLASSIC, SPORT_ACTION):
        raise ValueError("unknown daemon operation")
    if operation == SPORT_ACTION:
        if action_id not in (1, 2, 3, 4, 5, 6, 14, 15, 17):
            raise ValueError("unknown sport action ID")
    elif action_id != 0:
        raise ValueError("action ID only belongs to sport actions")
    if operation != MOVE and (vx != 0.0 or vy != 0.0 or wz != 0.0):
        raise ValueError("non-move command cannot carry velocity")
    stamp = time.monotonic_ns() if now_ns is None else now_ns
    packet = COMMAND.pack(
        MAGIC, VERSION, COMMAND.size, sequence, stamp, stamp + 500_000_000,
        operation, action_id, 0, vx, vy, wz, 0,
    )
    return packet[:-4] + struct.pack("<I", _checksum(packet))


@dataclass(frozen=True)
class Reply:
    sequence: int
    code: int
    armed: bool
    faulted: bool


def parse_reply(packet: bytes, expected_sequence: int) -> Reply:
    if len(packet) != REPLY.size:
        raise DaemonError("daemon reply has incorrect length")
    magic, version, size, sequence, code, armed, faulted, reserved, checksum = REPLY.unpack(packet)
    if (
        magic != MAGIC or version != VERSION or size != REPLY.size
        or sequence != expected_sequence or reserved != 0
        or checksum != _checksum(packet[:-4] + b"\0\0\0\0")
    ):
        raise DaemonError("daemon reply failed protocol validation")
    return Reply(sequence, code, bool(armed), bool(faulted))


class DaemonClient:
    def __init__(self, socket_path: str, timeout_s: float = 0.24):
        self.socket_path = socket_path
        self.timeout_s = timeout_s
        self._socket: socket.socket | None = None
        self._sequence = 0

    def __enter__(self) -> "DaemonClient":
        connection = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        connection.settimeout(self.timeout_s)
        try:
            connection.connect(self.socket_path)
        except Exception:
            connection.close()
            raise
        self._socket = connection
        return self

    def __exit__(self, *_exc: object) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def call(self, operation: int, *, action_id: int = 0,
             vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> Reply:
        if self._socket is None:
            raise DaemonError("daemon socket is not connected")
        self._sequence += 1
        packet = make_command(operation, self._sequence, action_id=action_id,
                              vx=vx, vy=vy, wz=wz)
        sent = self._socket.send(packet)
        if sent != len(packet):
            raise DaemonError("daemon command was not sent in full")
        reply = parse_reply(self._socket.recv(REPLY.size), self._sequence)
        if reply.code != OK:
            raise DaemonError(f"daemon rejected operation={operation} code={reply.code}")
        return reply
