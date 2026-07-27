import shutil
import socket
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SystemMetrics:
    hostname: str
    uptime: str
    cpu_percent: int
    memory_percent: int
    disk_percent: int
    temperature_c: float | None
    ip_address: str
    clock: str


def _clamp_percent(value: float) -> int:
    return max(0, min(100, round(value)))


def _format_uptime(seconds: float) -> str:
    total_minutes = max(0, int(seconds)) // 60
    days, remainder = divmod(total_minutes, 24 * 60)
    hours, minutes = divmod(remainder, 60)
    if days:
        return f"{days}天 {hours:02d}:{minutes:02d}"
    return f"{hours:02d}:{minutes:02d}"


class SystemMetricsCollector:
    def __init__(
        self,
        wifi_interface: str,
        proc_root: str | Path = "/proc",
        sys_root: str | Path = "/sys",
    ):
        self.wifi_interface = wifi_interface
        self.proc_root = Path(proc_root)
        self.sys_root = Path(sys_root)
        self.previous_cpu = self._read_cpu_times()

    def collect(self) -> SystemMetrics:
        return SystemMetrics(
            hostname=socket.gethostname(),
            uptime=_format_uptime(self._read_uptime()),
            cpu_percent=self._cpu_percent(),
            memory_percent=self._memory_percent(),
            disk_percent=self._disk_percent(),
            temperature_c=self._temperature(),
            ip_address=self._ip_address(),
            clock=time.strftime("%H:%M:%S"),
        )

    def _read_cpu_times(self) -> tuple[int, int]:
        line = (self.proc_root / "stat").read_text(encoding="ascii").splitlines()[0]
        values = [int(value) for value in line.split()[1:]]
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        return sum(values), idle

    def _cpu_percent(self) -> int:
        current = self._read_cpu_times()
        previous = self.previous_cpu
        self.previous_cpu = current
        total_delta = current[0] - previous[0]
        idle_delta = current[1] - previous[1]
        if total_delta <= 0:
            return 0
        return _clamp_percent(100 * (total_delta - idle_delta) / total_delta)

    def _read_uptime(self) -> float:
        return float((self.proc_root / "uptime").read_text(encoding="ascii").split()[0])

    def _memory_percent(self) -> int:
        values: dict[str, int] = {}
        for line in (self.proc_root / "meminfo").read_text(encoding="ascii").splitlines():
            name, raw_value = line.split(":", 1)
            values[name] = int(raw_value.split()[0])
        total = values.get("MemTotal", 0)
        available = values.get("MemAvailable", values.get("MemFree", 0))
        if total <= 0:
            return 0
        return _clamp_percent(100 * (total - available) / total)

    @staticmethod
    def _disk_percent() -> int:
        usage = shutil.disk_usage("/")
        if usage.total <= 0:
            return 0
        return _clamp_percent(100 * usage.used / usage.total)

    def _temperature(self) -> float | None:
        thermal_root = self.sys_root / "class" / "thermal"
        for path in sorted(thermal_root.glob("thermal_zone*/temp")):
            try:
                value = float(path.read_text(encoding="ascii").strip())
            except (OSError, ValueError):
                continue
            if value > 1000:
                value /= 1000
            if -20 <= value <= 150:
                return value
        return None

    def _ip_address(self) -> str:
        try:
            result = subprocess.run(
                ["ip", "-4", "-o", "addr", "show", "dev", self.wifi_interface],
                check=False,
                capture_output=True,
                text=True,
                timeout=2,
            )
        except (OSError, subprocess.TimeoutExpired):
            return ""
        for item in result.stdout.split():
            if "/" in item and item[0].isdigit():
                return item.split("/", 1)[0]
        return ""
