import os
import select
import termios
import time


class SerialPort:
    def __init__(self, device: str, baudrate: int):
        self.device = device
        self.baudrate = baudrate
        self.fd: int | None = None

    @property
    def is_open(self) -> bool:
        return self.fd is not None

    def open(self) -> None:
        if self.fd is not None:
            return
        speed = getattr(termios, f"B{self.baudrate}", None)
        if speed is None:
            raise ValueError(f"unsupported baud rate: {self.baudrate}")

        fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        self.fd = fd

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def read(self, size: int = 1024) -> bytes:
        if self.fd is None:
            return b""
        try:
            return os.read(self.fd, size)
        except BlockingIOError:
            return b""

    def write(self, data: bytes, timeout: float = 1.0) -> None:
        if self.fd is None:
            raise OSError("serial port is closed")
        view = memoryview(data)
        deadline = time.monotonic() + timeout
        while view:
            try:
                written = os.write(self.fd, view)
                view = view[written:]
            except BlockingIOError:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("serial write timed out")
                select.select([], [self.fd], [], remaining)

    def wait_readable(self, timeout: float) -> bool:
        if self.fd is None:
            time.sleep(timeout)
            return False
        readable, _, _ = select.select([self.fd], [], [], timeout)
        return bool(readable)
