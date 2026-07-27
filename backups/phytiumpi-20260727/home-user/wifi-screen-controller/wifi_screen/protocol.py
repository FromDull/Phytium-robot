from dataclasses import dataclass
from enum import Enum, auto


TERMINATOR = b"\xff\xff\xff"
CUSTOM_HEADER = 0x55


class EventKind(Enum):
    SELECT = auto()
    RESCAN = auto()
    CONNECT = auto()
    DISCONNECT = auto()
    NEXT_PAGE = auto()
    PREVIOUS_PAGE = auto()
    SYNC = auto()
    SYSTEM_SYNC = auto()
    SYSTEM_REFRESH = auto()
    POWEROFF = auto()
    HOME_SYNC = auto()
    BLUETOOTH_SYNC = auto()
    BLUETOOTH_SCAN = auto()
    BLUETOOTH_SELECT = auto()
    BLUETOOTH_PAIR = auto()
    BLUETOOTH_CONNECT = auto()
    BLUETOOTH_DISCONNECT = auto()
    BLUETOOTH_PREVIOUS_PAGE = auto()
    BLUETOOTH_NEXT_PAGE = auto()
    FACE_SYNC = auto()
    SCREEN_READY = auto()
    UNKNOWN = auto()


@dataclass(frozen=True)
class ScreenEvent:
    kind: EventKind
    data: bytes = b""


class TjcFrameParser:
    def __init__(self, max_buffer: int = 4096):
        self.buffer = bytearray()
        self.max_buffer = max_buffer

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer.extend(data)
        frames: list[bytes] = []
        while True:
            end = self.buffer.find(TERMINATOR)
            if end < 0:
                break
            frames.append(bytes(self.buffer[:end]))
            del self.buffer[: end + len(TERMINATOR)]
        if len(self.buffer) > self.max_buffer:
            del self.buffer[: -self.max_buffer]
        return frames


def parse_screen_frame(payload: bytes) -> ScreenEvent:
    if payload == b"\x88":
        return ScreenEvent(EventKind.SCREEN_READY)
    if len(payload) < 2 or payload[0] != CUSTOM_HEADER:
        return ScreenEvent(EventKind.UNKNOWN, payload)

    command = payload[1]
    if command == 0x01 and len(payload) == 3:
        return ScreenEvent(EventKind.SELECT, payload[2:3])
    if command == 0x02 and len(payload) == 2:
        return ScreenEvent(EventKind.RESCAN)
    if command == 0x03 and len(payload) >= 3:
        length = payload[2]
        password = payload[3:]
        if len(password) == length:
            return ScreenEvent(EventKind.CONNECT, password)
    if command == 0x04 and len(payload) == 2:
        return ScreenEvent(EventKind.DISCONNECT)
    if command == 0x05 and len(payload) == 2:
        return ScreenEvent(EventKind.NEXT_PAGE)
    if command == 0x06 and len(payload) == 2:
        return ScreenEvent(EventKind.PREVIOUS_PAGE)
    if command == 0x07 and len(payload) == 2:
        return ScreenEvent(EventKind.SYNC)
    if command == 0x10 and len(payload) == 2:
        return ScreenEvent(EventKind.SYSTEM_SYNC)
    if command == 0x11 and len(payload) == 2:
        return ScreenEvent(EventKind.SYSTEM_REFRESH)
    if command == 0x12 and len(payload) == 2:
        return ScreenEvent(EventKind.POWEROFF)
    if command == 0x20 and len(payload) == 2:
        return ScreenEvent(EventKind.HOME_SYNC)
    if command == 0x30 and len(payload) == 2:
        return ScreenEvent(EventKind.BLUETOOTH_SYNC)
    if command == 0x31 and len(payload) == 2:
        return ScreenEvent(EventKind.BLUETOOTH_SCAN)
    if command == 0x32 and len(payload) == 3:
        return ScreenEvent(EventKind.BLUETOOTH_SELECT, payload[2:3])
    if command == 0x33 and len(payload) == 2:
        return ScreenEvent(EventKind.BLUETOOTH_PAIR)
    if command == 0x34 and len(payload) == 2:
        return ScreenEvent(EventKind.BLUETOOTH_CONNECT)
    if command == 0x35 and len(payload) == 2:
        return ScreenEvent(EventKind.BLUETOOTH_DISCONNECT)
    if command == 0x36 and len(payload) == 2:
        return ScreenEvent(EventKind.BLUETOOTH_PREVIOUS_PAGE)
    if command == 0x37 and len(payload) == 2:
        return ScreenEvent(EventKind.BLUETOOTH_NEXT_PAGE)
    if command == 0x40 and len(payload) == 3 and payload[2] <= 9:
        return ScreenEvent(EventKind.FACE_SYNC, payload[2:3])
    return ScreenEvent(EventKind.UNKNOWN, payload)
