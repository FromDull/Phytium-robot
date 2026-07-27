#!/usr/bin/env python3
"""Export read-only chassis telemetry from the RPMsg broker as JSON."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import socket
import struct
import tempfile
import time


FRAME_MAGIC = 0xA5
MAX_PAYLOAD = 120
CMD_CHASSIS_STATUS = 63
CHASSIS_TELEMETRY_VERSION = 1
CHASSIS_TELEMETRY_SIZE = 44
DEFAULT_SOCKET = "/run/rpmsg-broker/rpmsg.sock"
DEFAULT_OUTPUT = "/home/user/robot_data/chassis_state.json"


def checksum(data: bytes) -> int:
    return (-sum(data)) & 0xFF


def encode_status_request(sequence: int) -> bytes:
    frame = bytes((FRAME_MAGIC, CMD_CHASSIS_STATUS, sequence & 0xFF, 0))
    return frame + bytes((checksum(frame),))


def decode_status_reply(frame: bytes, expected_sequence: int) -> bytes:
    if len(frame) < 5 or frame[0] != FRAME_MAGIC:
        raise ValueError("invalid RPMsg frame header")
    if len(frame) != frame[3] + 5:
        raise ValueError("invalid RPMsg frame length")
    if checksum(frame[:-1]) != frame[-1]:
        raise ValueError("invalid RPMsg frame checksum")
    if frame[1] != CMD_CHASSIS_STATUS:
        raise ValueError(f"unexpected reply command {frame[1]}")
    if frame[2] != (expected_sequence & 0xFF):
        raise ValueError("broker returned a mismatched sequence")
    return frame[4:-1]


def decode_telemetry(payload: bytes) -> dict:
    if len(payload) < CHASSIS_TELEMETRY_SIZE:
        raise ValueError("short chassis telemetry")
    values = struct.unpack(">BBBBIiiiiiiiii", payload[:CHASSIS_TELEMETRY_SIZE])
    if values[0] != CHASSIS_TELEMETRY_VERSION:
        raise ValueError("unsupported chassis telemetry version")
    scaled = [value / 1_000_000.0 for value in values[5:]]
    return {
        "status": values[1],
        "balance_state": values[2],
        "fault": values[3],
        "command_age_ms": values[4],
        "target_linear_m_s": scaled[0],
        "target_angular_rad_s": scaled[1],
        "applied_linear_m_s": scaled[2],
        "applied_angular_rad_s": scaled[3],
        "measured_linear_m_s": scaled[4],
        "measured_angular_rad_s": scaled[5],
        "wheel_position_m": scaled[6],
        "yaw_position_rad": scaled[7],
        "wheel_track_m": scaled[8],
    }


class BrokerStatusClient:
    def __init__(self, socket_path: str, timeout_s: float) -> None:
        self.socket_path = socket_path
        self.timeout_s = timeout_s
        self.sequence = 0
        self.connection: socket.socket | None = None

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
            self.connection = None

    def request(self) -> dict:
        self.sequence = (self.sequence + 1) & 0xFF
        try:
            if self.connection is None:
                connection = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
                connection.settimeout(self.timeout_s)
                connection.connect(self.socket_path)
                self.connection = connection
            self.connection.sendall(encode_status_request(self.sequence))
            frame = self.connection.recv(MAX_PAYLOAD + 5)
            return decode_telemetry(decode_status_reply(frame, self.sequence))
        except (OSError, ValueError):
            self.close()
            raise


def state_from_telemetry(telemetry: dict, sequence: int, sampled_at: float) -> dict:
    return {
        "sequence": sequence,
        "updated_at": sampled_at,
        "state_valid": int(telemetry["status"]) == 0,
        **telemetry,
    }


def atomic_write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as output:
            json.dump(payload, output, separators=(",", ":"))
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", default=DEFAULT_SOCKET)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=0.25)
    args = parser.parse_args()
    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")

    output = Path(args.output)
    client = BrokerStatusClient(args.socket, args.timeout)
    period = 1.0 / args.rate
    sequence = 0
    try:
        while True:
            started = time.monotonic()
            sequence += 1
            sampled_at = time.time()
            try:
                payload = state_from_telemetry(client.request(), sequence, sampled_at)
            except Exception as error:
                payload = {
                    "sequence": sequence,
                    "updated_at": sampled_at,
                    "state_valid": False,
                    "error": str(error)[:160],
                }
            atomic_write(output, payload)
            time.sleep(max(0.01, period - (time.monotonic() - started)))
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
