import socket
import struct
from dataclasses import dataclass


FRAME_MAGIC = 0xA5
MAX_PAYLOAD = 120
CMD_BALANCE_ENABLE = 50
CMD_BALANCE_DISABLE = 51
CMD_CHASSIS_SET_VELOCITY = 62
CMD_CHASSIS_STATUS = 63
CMD_CHASSIS_SET_TRACK_WIDTH = 64
CHASSIS_TELEMETRY_VERSION = 1
CHASSIS_TELEMETRY_SIZE = 44


def checksum(data: bytes) -> int:
    return (-sum(data)) & 0xFF


def encode_frame(command: int, sequence: int, payload: bytes = b"") -> bytes:
    if not 0 <= command <= 0xFF or not 0 <= sequence <= 0xFF:
        raise ValueError("command and sequence must fit in one byte")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("RPMsg payload is too large")
    frame = bytes((FRAME_MAGIC, command, sequence, len(payload))) + payload
    return frame + bytes((checksum(frame),))


def decode_frame(frame: bytes) -> tuple[int, int, bytes]:
    if len(frame) < 5 or frame[0] != FRAME_MAGIC:
        raise ValueError("invalid RPMsg frame header")
    if len(frame) != frame[3] + 5:
        raise ValueError("invalid RPMsg frame length")
    if checksum(frame[:-1]) != frame[-1]:
        raise ValueError("invalid RPMsg frame checksum")
    return frame[1], frame[2], frame[4:-1]


@dataclass(frozen=True)
class ChassisTelemetry:
    status: int
    balance_state: int
    fault: int
    command_age_ms: int
    target_linear_m_s: float
    target_angular_rad_s: float
    applied_linear_m_s: float
    applied_angular_rad_s: float
    measured_linear_m_s: float
    measured_angular_rad_s: float
    wheel_position_m: float
    yaw_position_rad: float
    wheel_track_m: float


def decode_chassis_telemetry(payload: bytes) -> ChassisTelemetry:
    if len(payload) < CHASSIS_TELEMETRY_SIZE:
        raise ValueError("short chassis telemetry")
    values = struct.unpack(">BBBBIiiiiiiiii", payload[:CHASSIS_TELEMETRY_SIZE])
    if values[0] != CHASSIS_TELEMETRY_VERSION:
        raise ValueError("unsupported chassis telemetry version")
    scaled = [value / 1_000_000.0 for value in values[5:]]
    return ChassisTelemetry(
        status=values[1],
        balance_state=values[2],
        fault=values[3],
        command_age_ms=values[4],
        target_linear_m_s=scaled[0],
        target_angular_rad_s=scaled[1],
        applied_linear_m_s=scaled[2],
        applied_angular_rad_s=scaled[3],
        measured_linear_m_s=scaled[4],
        measured_angular_rad_s=scaled[5],
        wheel_position_m=scaled[6],
        yaw_position_rad=scaled[7],
        wheel_track_m=scaled[8],
    )


class BrokerClient:
    def __init__(self, socket_path: str, timeout_s: float = 0.15):
        self.socket_path = socket_path
        self.timeout_s = timeout_s
        self.sequence = 0
        self.socket: socket.socket | None = None

    def close(self) -> None:
        if self.socket is not None:
            self.socket.close()
            self.socket = None

    def _connect(self) -> None:
        if self.socket is not None:
            return
        client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        client.settimeout(self.timeout_s)
        client.connect(self.socket_path)
        self.socket = client

    def request(self, command: int, payload: bytes = b"") -> tuple[int, bytes]:
        self.sequence = (self.sequence + 1) & 0xFF
        expected_sequence = self.sequence
        try:
            self._connect()
            assert self.socket is not None
            self.socket.sendall(encode_frame(command, expected_sequence, payload))
            reply = self.socket.recv(MAX_PAYLOAD + 5)
            reply_command, reply_sequence, reply_payload = decode_frame(reply)
            if reply_sequence != expected_sequence:
                raise ValueError("broker returned a mismatched sequence")
            return reply_command, reply_payload
        except (OSError, ValueError):
            self.close()
            raise


def velocity_payload(linear_m_s: float, angular_rad_s: float,
                     timeout_ms: int) -> bytes:
    return struct.pack(
        ">iiH",
        round(linear_m_s * 1_000_000.0),
        round(angular_rad_s * 1_000_000.0),
        timeout_ms,
    )


def track_width_payload(wheel_track_m: float) -> bytes:
    return struct.pack(">i", round(wheel_track_m * 1_000_000.0))
