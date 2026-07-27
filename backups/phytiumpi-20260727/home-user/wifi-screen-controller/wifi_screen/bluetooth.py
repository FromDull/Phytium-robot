import os
import re
import subprocess
from dataclasses import dataclass
from typing import Callable, Sequence

from .network import CommandResult


@dataclass(frozen=True)
class BluetoothDevice:
    address: str
    name: str
    paired: bool = False
    connected: bool = False
    rssi: int | None = None

    @property
    def has_friendly_name(self) -> bool:
        normalized = self.address.replace(":", "-").casefold()
        return self.name.casefold() not in {self.address.casefold(), normalized}


class BluetoothError(RuntimeError):
    pass


Runner = Callable[[Sequence[str], float], subprocess.CompletedProcess[str]]
MAC_PATTERN = re.compile(r"^[0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5}$")


class BluetoothManager:
    def __init__(self, runner: Runner | None = None, scan_seconds: int = 8):
        self.runner = runner or self._default_runner
        self.scan_seconds = scan_seconds

    @staticmethod
    def _default_runner(
        args: Sequence[str], timeout: float
    ) -> subprocess.CompletedProcess[str]:
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
        try:
            result = self.runner(args, timeout)
        except subprocess.TimeoutExpired as exc:
            raise BluetoothError("蓝牙控制器响应超时") from exc
        output = "\n".join(part for part in (result.stdout, result.stderr) if part).strip()
        if result.returncode != 0 or "Failed" in output:
            raise BluetoothError(output or "bluetoothctl failed")
        return output

    def scan(self) -> list[BluetoothDevice]:
        self._run(["bluetoothctl", "power", "on"], 5)
        self._run(
            ["bluetoothctl", "--timeout", str(self.scan_seconds), "scan", "on"],
            self.scan_seconds + 5,
        )
        return self.devices()

    def devices(self) -> list[BluetoothDevice]:
        output = self._run(["bluetoothctl", "devices"], 5)
        devices: list[BluetoothDevice] = []
        seen: set[str] = set()
        for line in output.splitlines():
            fields = line.strip().split(maxsplit=2)
            if len(fields) < 2 or fields[0] != "Device" or not MAC_PATTERN.match(fields[1]):
                continue
            address = fields[1].upper()
            if address in seen:
                continue
            seen.add(address)
            fallback_name = fields[2] if len(fields) > 2 else address
            devices.append(self._device_info(address, fallback_name))
        return sorted(
            devices,
            key=lambda item: (
                not item.connected,
                not item.paired,
                not item.has_friendly_name,
                -(item.rssi if item.rssi is not None else -999),
                item.name.casefold(),
            ),
        )

    def connected_addresses(self) -> set[str]:
        output = self._run(["bluetoothctl", "devices", "Connected"], 5)
        addresses: set[str] = set()
        for line in output.splitlines():
            fields = line.strip().split(maxsplit=2)
            if len(fields) >= 2 and fields[0] == "Device" and MAC_PATTERN.match(fields[1]):
                addresses.add(fields[1].upper())
        return addresses

    def _device_info(self, address: str, fallback_name: str) -> BluetoothDevice:
        try:
            output = self._run(["bluetoothctl", "info", address], 5)
        except BluetoothError:
            return BluetoothDevice(address, fallback_name)
        properties: dict[str, str] = {}
        for line in output.splitlines():
            key, separator, value = line.strip().partition(":")
            if separator:
                properties[key] = value.strip()
        rssi_text = properties.get("RSSI", "")
        try:
            rssi = int(rssi_text)
        except ValueError:
            rssi = None
        return BluetoothDevice(
            address=address,
            name=properties.get("Name") or properties.get("Alias") or fallback_name,
            paired=properties.get("Paired", "no").lower() == "yes",
            connected=properties.get("Connected", "no").lower() == "yes",
            rssi=rssi,
        )

    def pair(self, address: str) -> CommandResult:
        try:
            message = self._run(["bluetoothctl", "pair", address], 35)
            self._run(["bluetoothctl", "trust", address], 8)
            return CommandResult(True, message or "paired")
        except (BluetoothError, subprocess.TimeoutExpired) as exc:
            return CommandResult(False, str(exc))

    def connect(self, address: str) -> CommandResult:
        try:
            message = self._run(["bluetoothctl", "connect", address], 20)
            return CommandResult(True, message or "connected")
        except (BluetoothError, subprocess.TimeoutExpired) as exc:
            return CommandResult(False, str(exc))

    def disconnect(self, address: str) -> CommandResult:
        try:
            message = self._run(["bluetoothctl", "disconnect", address], 15)
            return CommandResult(True, message or "disconnected")
        except (BluetoothError, subprocess.TimeoutExpired) as exc:
            return CommandResult(False, str(exc))
