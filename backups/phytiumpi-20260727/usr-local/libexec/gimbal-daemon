#!/usr/bin/env python3
"""Single-writer RPMsg gimbal daemon with a small JSON Unix socket API."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import select
import signal
import socket
import struct
import sys
import time
from typing import Any


MAGIC = 0xA5
MAX_PAYLOAD = 120
TELEMETRY_VERSION = 1
TELEMETRY_SIZE = 68

CMD_ENABLE = 70
CMD_DISABLE = 71
CMD_SET_TARGET = 72
CMD_STATUS = 73
CMD_SET_LIMITS = 75
CMD_ESTOP = 77

STATE_DISABLED = 0
STATE_ACTIVE = 3
STATE_FAULT = 6
STATUS_OK = 0
STATUS_NO_FEEDBACK = 3

DEFAULT_SOCKET = "/run/gimbal-daemon/gimbal.sock"
DEFAULT_BROKER_SOCKET = "/run/rpmsg-broker/rpmsg.sock"
DEFAULT_CONFIG = "/home/user/openamp_rpmsg_comm/gimbal.conf"


class GimbalError(RuntimeError):
    pass


@dataclass(frozen=True)
class GimbalConfig:
    yaw_min_deg: float
    yaw_max_deg: float
    pitch_min_deg: float
    pitch_max_deg: float
    home_speed_rpm: int
    home_torque_percent: int
    return_speed_rpm: int
    return_torque_percent: int
    move_speed_rpm: int
    move_torque_percent: int

    @classmethod
    def load(cls, path: str) -> "GimbalConfig":
        values: dict[str, str] = {}
        for raw_line in Path(path).read_text(encoding="utf-8").splitlines():
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            if "=" not in line:
                raise GimbalError(f"invalid config line: {raw_line}")
            key, value = (item.strip() for item in line.split("=", 1))
            if key in values:
                raise GimbalError(f"duplicate config key: {key}")
            values[key] = value

        expected = set(cls.__dataclass_fields__)
        if set(values) != expected:
            missing = sorted(expected - set(values))
            unknown = sorted(set(values) - expected)
            raise GimbalError(f"invalid config keys: missing={missing}, unknown={unknown}")
        try:
            config = cls(
                yaw_min_deg=float(values["yaw_min_deg"]),
                yaw_max_deg=float(values["yaw_max_deg"]),
                pitch_min_deg=float(values["pitch_min_deg"]),
                pitch_max_deg=float(values["pitch_max_deg"]),
                home_speed_rpm=int(values["home_speed_rpm"]),
                home_torque_percent=int(values["home_torque_percent"]),
                return_speed_rpm=int(values["return_speed_rpm"]),
                return_torque_percent=int(values["return_torque_percent"]),
                move_speed_rpm=int(values["move_speed_rpm"]),
                move_torque_percent=int(values["move_torque_percent"]),
            )
        except ValueError as error:
            raise GimbalError(f"invalid config value: {error}") from error
        config.validate()
        return config

    def validate(self) -> None:
        if not (self.yaw_min_deg < 0 < self.yaw_max_deg):
            raise GimbalError("yaw limits must contain zero")
        if not (self.pitch_min_deg < 0 < self.pitch_max_deg):
            raise GimbalError("pitch limits must contain zero")
        if max(map(abs, (self.yaw_min_deg, self.yaw_max_deg,
                         self.pitch_min_deg, self.pitch_max_deg))) > 360:
            raise GimbalError("configured angle exceeds 360 degrees")
        if not (1 <= self.home_speed_rpm <= 60 and
                1 <= self.return_speed_rpm <= 60 and
                1 <= self.move_speed_rpm <= 1000):
            raise GimbalError("configured speed is out of range")
        if not (5 <= self.home_torque_percent <= 80 and
                5 <= self.return_torque_percent <= 80 and
                1 <= self.move_torque_percent <= 80):
            raise GimbalError("configured torque is out of range")


@dataclass(frozen=True)
class Telemetry:
    command_status: int
    state: int
    fault: int
    limits_valid_mask: int
    feedback_valid_mask: int
    torque_percent: int
    yaw_deg: float
    pitch_deg: float
    yaw_speed_rpm: int
    pitch_speed_rpm: int
    yaw_current_a: float
    pitch_current_a: float
    yaw_target_deg: float
    pitch_target_deg: float
    yaw_min_deg: float
    yaw_max_deg: float
    pitch_min_deg: float
    pitch_max_deg: float
    command_speed_rpm: int
    yaw_feedback_age_ms: int
    pitch_feedback_age_ms: int
    timeout_remaining_ms: int
    startup_pitch_deg: float

    @classmethod
    def parse(cls, payload: bytes) -> "Telemetry":
        if len(payload) < TELEMETRY_SIZE or payload[0] != TELEMETRY_VERSION:
            raise GimbalError("invalid gimbal telemetry")
        i32 = lambda offset: struct.unpack_from(">i", payload, offset)[0]
        i16 = lambda offset: struct.unpack_from(">h", payload, offset)[0]
        u16 = lambda offset: struct.unpack_from(">H", payload, offset)[0]
        u32 = lambda offset: struct.unpack_from(">I", payload, offset)[0]
        return cls(
            command_status=payload[1], state=payload[2], fault=payload[3],
            limits_valid_mask=payload[4], feedback_valid_mask=payload[5],
            torque_percent=payload[6], yaw_deg=i32(8) / 100.0,
            pitch_deg=i32(12) / 100.0, yaw_speed_rpm=i16(16),
            pitch_speed_rpm=i16(18), yaw_current_a=i16(20) / 100.0,
            pitch_current_a=i16(22) / 100.0,
            yaw_target_deg=i32(24) / 100.0,
            pitch_target_deg=i32(28) / 100.0,
            yaw_min_deg=i32(32) / 100.0, yaw_max_deg=i32(36) / 100.0,
            pitch_min_deg=i32(40) / 100.0,
            pitch_max_deg=i32(44) / 100.0,
            command_speed_rpm=u16(48), yaw_feedback_age_ms=u32(52),
            pitch_feedback_age_ms=u32(56), timeout_remaining_ms=u32(60),
            startup_pitch_deg=i32(64) / 100.0,
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def checksum(data: bytes) -> int:
    return (-sum(data)) & 0xFF


def encode_frame(command: int, sequence: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise GimbalError("payload too large")
    frame = bytes((MAGIC, command, sequence, len(payload))) + payload
    return frame + bytes((checksum(frame),))


def decode_frame(data: bytes) -> tuple[int, int, bytes]:
    if len(data) < 5 or data[0] != MAGIC:
        raise GimbalError("invalid RPMsg frame")
    length = data[3]
    if length > MAX_PAYLOAD or len(data) != length + 5:
        raise GimbalError("invalid RPMsg frame length")
    if checksum(data[:-1]) != data[-1]:
        raise GimbalError("invalid RPMsg checksum")
    return data[1], data[2], data[4:-1]


class RpmsgGimbal:
    def __init__(self, broker_socket: str, config: GimbalConfig):
        self.broker_socket = broker_socket
        self.config = config
        self.transport: socket.socket | None = None
        self.sequence = 0

    def open(self) -> None:
        self.transport = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.transport.connect(self.broker_socket)

    def close(self) -> None:
        if self.transport is not None:
            self.transport.close()
            self.transport = None

    def transact(self, command: int, payload: bytes = b"", timeout: float = 5.0) -> Telemetry:
        if self.transport is None:
            raise GimbalError("RPMsg broker is not connected")
        self.sequence = (self.sequence + 1) & 0xFF
        frame = encode_frame(command, self.sequence, payload)
        if self.transport.send(frame) != len(frame):
            raise GimbalError("short broker write")
        readable, _, _ = select.select([self.transport], [], [], timeout)
        if not readable:
            raise GimbalError("RPMsg reply timeout")
        reply_command, reply_sequence, reply_payload = decode_frame(
            self.transport.recv(128)
        )
        if reply_command != command or reply_sequence != self.sequence:
            raise GimbalError("unexpected RPMsg reply")
        return Telemetry.parse(reply_payload)

    def status(self) -> Telemetry:
        return self.transact(CMD_STATUS)

    def apply_limits(self) -> Telemetry:
        values = (
            self.config.yaw_min_deg, self.config.yaw_max_deg,
            self.config.pitch_min_deg, self.config.pitch_max_deg,
        )
        payload = b"".join(struct.pack(">i", round(value * 100)) for value in values)
        return self.transact(CMD_SET_LIMITS, payload)

    def enable(self) -> Telemetry:
        limits = self.apply_limits()
        if limits.command_status != STATUS_OK:
            return limits
        payload = struct.pack(
            ">BHBH", self.config.home_torque_percent,
            self.config.home_speed_rpm, self.config.return_torque_percent,
            self.config.return_speed_rpm,
        )
        telemetry = self.transact(CMD_ENABLE, payload)
        for _ in range(20):
            if telemetry.command_status != STATUS_NO_FEEDBACK:
                return telemetry
            time.sleep(0.1)
            telemetry = self.transact(CMD_ENABLE, payload)
        return telemetry

    def disable(self) -> Telemetry:
        return self.transact(CMD_DISABLE)

    def estop(self) -> Telemetry:
        return self.transact(CMD_ESTOP)

    def set_target(self, yaw_deg: float, pitch_deg: float,
                   speed_rpm: int, torque_percent: int,
                   timeout_ms: int = 0) -> Telemetry:
        payload = struct.pack(
            ">iiHBH", round(yaw_deg * 100), round(pitch_deg * 100),
            speed_rpm, torque_percent, timeout_ms,
        )
        return self.transact(CMD_SET_TARGET, payload)


class GimbalService:
    def __init__(self, gimbal: RpmsgGimbal, limit_margin_deg: float = 3.0):
        self.gimbal = gimbal
        self.limit_margin_deg = limit_margin_deg

    def _business_limits(self) -> tuple[float, float, float, float]:
        config = self.gimbal.config
        margin = self.limit_margin_deg
        return (
            config.yaw_min_deg + margin,
            config.yaw_max_deg - margin,
            config.pitch_min_deg + margin,
            config.pitch_max_deg - margin,
        )

    def _return_position_is_safe(self, status: Telemetry) -> bool:
        config = self.gimbal.config
        return (
            config.pitch_min_deg <= status.startup_pitch_deg <=
            config.pitch_max_deg
        )

    def controlled_shutdown(self) -> Telemetry:
        status = self.gimbal.status()
        if status.state in {STATE_DISABLED, STATE_FAULT}:
            return status
        if self._return_position_is_safe(status):
            return self.gimbal.disable()
        return self.gimbal.estop()

    def _require_motion_ready(self) -> Telemetry:
        status = self.gimbal.status()
        if status.command_status != STATUS_OK:
            raise GimbalError(f"status rejected: {status.command_status}")
        if status.fault != 0:
            raise GimbalError(
                f"gimbal fault: state={status.state} fault=0x{status.fault:02x}"
            )
        if status.state != STATE_ACTIVE:
            raise GimbalError(f"gimbal is not active: state={status.state}")
        if status.limits_valid_mask != 0x0F:
            raise GimbalError("gimbal limits are not ready")
        if status.feedback_valid_mask != 0x03:
            raise GimbalError("gimbal feedback is not valid")
        if max(status.yaw_feedback_age_ms, status.pitch_feedback_age_ms) >= 500:
            raise GimbalError("gimbal feedback is stale")
        return status

    def _wait_for_target(self, yaw_deg: float, pitch_deg: float,
                         timeout: float = 15.0) -> Telemetry:
        deadline = time.monotonic() + timeout
        settled_samples = 0
        last_status: Telemetry | None = None
        while time.monotonic() < deadline:
            time.sleep(0.1)
            last_status = self._require_motion_ready()
            position_ready = (
                abs(last_status.yaw_deg - yaw_deg) <= 1.0 and
                abs(last_status.pitch_deg - pitch_deg) <= 1.0
            )
            speed_ready = (
                abs(last_status.yaw_speed_rpm) <= 5 and
                abs(last_status.pitch_speed_rpm) <= 5
            )
            if position_ready and speed_ready:
                settled_samples += 1
                if settled_samples >= 3:
                    return last_status
            else:
                settled_samples = 0
        detail = "no telemetry" if last_status is None else (
            f"last position yaw={last_status.yaw_deg:.2f}, "
            f"pitch={last_status.pitch_deg:.2f}"
        )
        raise GimbalError(f"gimbal target was not reached within {timeout:.1f}s: {detail}")

    def handle(self, request: dict[str, Any]) -> dict[str, Any]:
        command = str(request.get("command", "")).lower()
        if command == "status":
            telemetry = self.gimbal.status()
        elif command == "estop":
            telemetry = self.gimbal.estop()
        elif command == "enable":
            if request.get("confirm") is not True:
                raise GimbalError("enable requires confirm=true")
            telemetry = self.gimbal.enable()
            if (telemetry.command_status == STATUS_OK and
                    not self._return_position_is_safe(telemetry)):
                unsafe_pitch = telemetry.startup_pitch_deg
                self.gimbal.estop()
                raise GimbalError(
                    f"startup return pitch {unsafe_pitch:.2f} deg is outside "
                    "configured pitch limits; "
                    "gimbal was emergency-stopped"
                )
        elif command == "disable":
            if request.get("confirm") is not True:
                raise GimbalError("disable requires confirm=true")
            status = self.gimbal.status()
            if not self._return_position_is_safe(status):
                raise GimbalError(
                    f"startup return pitch {status.startup_pitch_deg:.2f} deg is "
                    "outside configured pitch limits; "
                    "use estop instead of disable"
                )
            telemetry = self.gimbal.disable()
        elif command in {"set", "center"}:
            self._require_motion_ready()
            yaw = 0.0 if command == "center" else float(request["yaw_deg"])
            pitch = 0.0 if command == "center" else float(request["pitch_deg"])
            yaw_min, yaw_max, pitch_min, pitch_max = self._business_limits()
            if not (yaw_min <= yaw <= yaw_max and pitch_min <= pitch <= pitch_max):
                raise GimbalError(
                    f"target exceeds safe workspace: yaw=[{yaw_min:.2f}, {yaw_max:.2f}], "
                    f"pitch=[{pitch_min:.2f}, {pitch_max:.2f}]"
                )
            telemetry = self.gimbal.set_target(
                yaw, pitch, speed_rpm=self.gimbal.config.move_speed_rpm,
                torque_percent=self.gimbal.config.move_torque_percent,
            )
            if telemetry.command_status == STATUS_OK:
                telemetry = self._wait_for_target(yaw, pitch)
        else:
            raise GimbalError("unsupported command")
        return {"ok": telemetry.command_status == STATUS_OK,
                "telemetry": telemetry.to_dict()}


def send_json(connection: socket.socket, payload: dict[str, Any]) -> None:
    connection.sendall((json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8"))


def receive_json(connection: socket.socket) -> dict[str, Any]:
    raw = bytearray()
    while b"\n" not in raw:
        chunk = connection.recv(1024)
        if not chunk:
            break
        raw.extend(chunk)
        if len(raw) > 4096:
            raise GimbalError("request is too large")
    if not raw:
        raise GimbalError("empty request")
    request = json.loads(bytes(raw).split(b"\n", 1)[0].decode("utf-8"))
    if not isinstance(request, dict):
        raise GimbalError("request must be a JSON object")
    return request


def serve(args: argparse.Namespace) -> int:
    config = GimbalConfig.load(args.config)
    gimbal = RpmsgGimbal(args.broker_socket, config)
    gimbal.open()
    service = GimbalService(gimbal, args.limit_margin_deg)
    socket_path = Path(args.socket)
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    socket_path.unlink(missing_ok=True)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(socket_path))
    os.chmod(socket_path, 0o660)
    server.listen(8)
    server.settimeout(1.0)
    stopping = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    limits = service._business_limits()
    print(
        f"gimbal daemon ready: socket={socket_path} "
        f"yaw=[{limits[0]:.2f}, {limits[1]:.2f}] "
        f"pitch=[{limits[2]:.2f}, {limits[3]:.2f}]",
        flush=True,
    )
    try:
        while not stopping:
            try:
                connection, _ = server.accept()
            except socket.timeout:
                continue
            with connection:
                try:
                    response = service.handle(receive_json(connection))
                except Exception as error:
                    response = {"ok": False, "error": str(error)}
                try:
                    send_json(connection, response)
                except (BrokenPipeError, ConnectionResetError):
                    # A command may legitimately outlive its requester.  A stale
                    # local client must never terminate the safety daemon.
                    continue
    finally:
        try:
            service.controlled_shutdown()
        except Exception as error:
            print(f"controlled shutdown failed: {error}", file=sys.stderr, flush=True)
        server.close()
        socket_path.unlink(missing_ok=True)
        gimbal.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker-socket", default=DEFAULT_BROKER_SOCKET)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--socket", default=DEFAULT_SOCKET)
    parser.add_argument("--limit-margin-deg", type=float, default=3.0)
    return parser


if __name__ == "__main__":
    raise SystemExit(serve(build_parser().parse_args()))
