import os
import subprocess
from dataclasses import dataclass
from typing import Callable, Sequence


@dataclass(frozen=True)
class WifiNetwork:
    ssid: str
    signal: int
    security: str
    connected: bool = False

    @property
    def is_open(self) -> bool:
        return not self.security or self.security == "--"


@dataclass(frozen=True)
class WifiStatus:
    connected: bool
    connection: str = ""
    ip_address: str = ""


@dataclass(frozen=True)
class CommandResult:
    success: bool
    message: str


class NmcliError(RuntimeError):
    pass


Runner = Callable[[Sequence[str], float], subprocess.CompletedProcess[str]]


def split_nmcli_escaped(line: str, separator: str = ":") -> list[str]:
    fields: list[str] = []
    current: list[str] = []
    escaped = False
    for char in line:
        if escaped:
            current.append(char)
            escaped = False
        elif char == "\\":
            escaped = True
        elif char == separator:
            fields.append("".join(current))
            current = []
        else:
            current.append(char)
    if escaped:
        current.append("\\")
    fields.append("".join(current))
    return fields


class WifiManager:
    def __init__(self, interface: str, runner: Runner | None = None):
        self.interface = interface
        self.runner = runner or self._default_runner

    @staticmethod
    def _default_runner(args: Sequence[str], timeout: float) -> subprocess.CompletedProcess[str]:
        env = os.environ.copy()
        env["LC_ALL"] = "C.UTF-8"
        return subprocess.run(
            list(args),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
            env=env,
            check=False,
        )

    def _run(self, args: Sequence[str], timeout: float) -> str:
        result = self.runner(args, timeout)
        if result.returncode != 0:
            detail = (result.stderr or result.stdout or "nmcli failed").strip()
            raise NmcliError(detail)
        return result.stdout

    def scan(self) -> list[WifiNetwork]:
        output = self._run(
            [
                "nmcli",
                "-t",
                "--escape",
                "yes",
                "-f",
                "IN-USE,SSID,SIGNAL,SECURITY",
                "device",
                "wifi",
                "list",
                "ifname",
                self.interface,
                "--rescan",
                "yes",
            ],
            timeout=25,
        )
        by_ssid: dict[str, WifiNetwork] = {}
        for line in output.splitlines():
            fields = split_nmcli_escaped(line)
            if len(fields) != 4:
                continue
            in_use, ssid, signal_text, security = fields
            if not ssid:
                continue
            try:
                signal = max(0, min(100, int(signal_text)))
            except ValueError:
                signal = 0
            candidate = WifiNetwork(ssid, signal, security, in_use == "*")
            previous = by_ssid.get(ssid)
            if previous is None:
                by_ssid[ssid] = candidate
            else:
                strongest = candidate if candidate.signal > previous.signal else previous
                by_ssid[ssid] = WifiNetwork(
                    ssid=ssid,
                    signal=strongest.signal,
                    security=strongest.security,
                    connected=previous.connected or candidate.connected,
                )
        return sorted(
            by_ssid.values(),
            key=lambda item: (not item.connected, -item.signal, item.ssid.casefold()),
        )

    def status(self) -> WifiStatus:
        output = self._run(
            [
                "nmcli",
                "-g",
                "GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS",
                "device",
                "show",
                self.interface,
            ],
            timeout=5,
        )
        lines = output.splitlines()
        state = lines[0] if lines else ""
        connection = lines[1] if len(lines) > 1 and lines[1] != "--" else ""
        ip_address = lines[2].split("/", 1)[0] if len(lines) > 2 else ""
        return WifiStatus(state.startswith("100"), connection, ip_address)

    def connect(self, ssid: str, password: str) -> CommandResult:
        args = ["nmcli", "--wait", "30", "device", "wifi", "connect", ssid]
        if password:
            args.extend(["password", password])
        args.extend(["ifname", self.interface])
        try:
            message = self._run(args, timeout=35).strip()
            return CommandResult(True, message or "connected")
        except (NmcliError, subprocess.TimeoutExpired) as exc:
            return CommandResult(False, str(exc))

    def disconnect(self) -> CommandResult:
        try:
            message = self._run(
                ["nmcli", "--wait", "10", "device", "disconnect", self.interface],
                timeout=15,
            ).strip()
            return CommandResult(True, message or "disconnected")
        except (NmcliError, subprocess.TimeoutExpired) as exc:
            return CommandResult(False, str(exc))
