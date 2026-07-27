import math
from collections.abc import Callable

from .bluetooth import BluetoothDevice
from .config import AppConfig
from .network import WifiNetwork, WifiStatus
from .protocol import TERMINATOR
from .system_status import SystemMetrics


def truncate_encoded(text: str, max_bytes: int, encoding: str) -> str:
    output: list[str] = []
    used = 0
    for char in text:
        encoded = char.encode(encoding, errors="replace")
        if used + len(encoded) > max_bytes:
            break
        output.append(char)
        used += len(encoded)
    return "".join(output)


class TjcScreen:
    def __init__(self, config: AppConfig, writer: Callable[[bytes], None]):
        self.config = config
        self.writer = writer
        self.cache: dict[str, str] = {}

    def clear_cache(self) -> None:
        self.cache.clear()

    def raw_command(self, command: str) -> None:
        encoded = command.encode(self.config.screen_encoding, errors="replace")
        self.writer(encoded + TERMINATOR)

    def set_face_state(self, state: int, reload_page: bool = False) -> None:
        if not 0 <= state <= 9:
            raise ValueError("face state must be between 0 and 9")
        self.raw_command(f"{self.config.face_state_variable}={state}")
        if reload_page:
            self.raw_command(f"page {self.config.face_page_name}")

    def _qualified(self, component: str, page_name: str | None = None) -> str:
        return f"{page_name or self.config.page_name}.{component}"

    @staticmethod
    def _escape_text(text: str) -> str:
        return (
            text.replace("\\", "\\\\")
            .replace('"', '\\"')
            .replace("\r", " ")
            .replace("\n", " ")
        )

    def set_text(
        self,
        component: str,
        text: str,
        force: bool = False,
        page_name: str | None = None,
    ) -> None:
        page = page_name or self.config.page_name
        key = f"{page}.{component}.txt"
        if not force and self.cache.get(key) == text:
            return
        escaped = self._escape_text(text)
        self.raw_command(f'{self._qualified(component, page)}.txt="{escaped}"')
        self.cache[key] = text

    def set_background_color(
        self,
        component: str,
        color: int,
        force: bool = False,
        page_name: str | None = None,
    ) -> None:
        """Set a component background colour (TJC RGB565 value)."""
        page = page_name or self.config.page_name
        key = f"{page}.{component}.bco"
        encoded = str(int(color))
        if not force and self.cache.get(key) == encoded:
            return
        self.raw_command(f"{self._qualified(component, page)}.bco={encoded}")
        self.cache[key] = encoded

    def set_value(
        self,
        component: str,
        value: int,
        force: bool = False,
        page_name: str | None = None,
    ) -> None:
        page = page_name or self.config.page_name
        value = int(value)
        key = f"{page}.{component}.val"
        encoded_value = str(value)
        if not force and self.cache.get(key) == encoded_value:
            return
        self.raw_command(f"{self._qualified(component, page)}.val={value}")
        self.cache[key] = encoded_value

    def set_touch(
        self, component: str, enabled: bool, page_name: str | None = None
    ) -> None:
        page = page_name or self.config.page_name
        key = f"{page}.{component}.touch"
        value = "1" if enabled else "0"
        if self.cache.get(key) == value:
            return
        self.raw_command(f"tsw {self._qualified(component, page)},{value}")
        self.cache[key] = value

    def set_status(self, text: str) -> None:
        self.set_text(self.config.components.status, text)

    def show_powered_off(self) -> None:
        """Show the latched powered-off state before Linux powers down."""
        page = self.config.home_page_name
        self.set_background_color(
            self.config.components.home_title, 63488, force=True, page_name=page
        )
        self.set_text(
            self.config.components.home_title, "已经关机", force=True, page_name=page
        )
        self.set_touch(self.config.components.power, False, page_name=page)

    def set_connection(self, status: WifiStatus) -> None:
        if status.connected:
            text = f"已连接: {status.connection}"
            if status.ip_address:
                text += f"  IP: {status.ip_address}"
        else:
            text = "Wi-Fi 未连接"
        self.set_text(self.config.components.current, text)

    def set_selected(self, network: WifiNetwork | None) -> None:
        text = f"已选择: {network.ssid}" if network else "请选择 Wi-Fi"
        self.set_text(self.config.components.selected, text)

    def clear_password(self) -> None:
        # The user edits this value on the screen, so the local send cache is not authoritative.
        self.set_text(self.config.components.password, "", force=True)

    def render_networks(
        self,
        networks: list[WifiNetwork],
        page_index: int,
        selected_ssid: str,
    ) -> None:
        size = self.config.list_page_size
        start = page_index * size
        visible = networks[start : start + size]
        for slot in range(size):
            button = self.config.components.wifi_button_pattern.format(slot)
            if slot < len(visible):
                network = visible[slot]
                selected = network.ssid == selected_ssid
                prefix = "> " if selected else ""
                active = "* " if network.connected else ""
                security = "OPEN" if network.is_open else network.security
                label = f"{prefix}{active}{network.ssid}  {network.signal}%  {security}"
                label = truncate_encoded(
                    label,
                    self.config.button_text_max_bytes,
                    self.config.screen_encoding,
                )
                self.set_text(button, label)
                self.set_touch(button, True)
            else:
                self.set_text(button, "")
                self.set_touch(button, False)

        page_count = max(1, math.ceil(len(networks) / size))
        self.set_text(self.config.components.page, f"{page_index + 1}/{page_count}")
        self.set_touch(self.config.components.previous, page_index > 0)
        self.set_touch(self.config.components.next, page_index + 1 < page_count)

    def render_system(self, metrics: SystemMetrics, force: bool = False) -> None:
        page = self.config.system_page_name
        self.set_text("t_hostname", f"主机: {metrics.hostname}", force, page)
        self.set_text("t_uptime", f"运行: {metrics.uptime}", force, page)
        self.set_value("j_cpu", metrics.cpu_percent, force, page)
        self.set_text("t_cpu_value", f"{metrics.cpu_percent}%", force, page)
        self.set_value("j_mem", metrics.memory_percent, force, page)
        self.set_text("t_mem_value", f"{metrics.memory_percent}%", force, page)
        self.set_value("j_disk", metrics.disk_percent, force, page)
        self.set_text("t_disk_value", f"{metrics.disk_percent}%", force, page)
        temperature = "--" if metrics.temperature_c is None else f"{metrics.temperature_c:.1f}C"
        self.set_text("t_temperature", f"温度: {temperature}", force, page)
        self.set_text("t_sys_ip", f"IP: {metrics.ip_address or '--'}", force, page)
        self.set_text("t_service", "服务: Wi-Fi控制器运行中", force, page)
        self.set_text("t_sys_clock", metrics.clock, force, page)

    def render_home(
        self,
        metrics: SystemMetrics,
        wifi_status: WifiStatus,
        force: bool = False,
    ) -> None:
        page = self.config.home_page_name
        connection = wifi_status.connection or "--"
        ip_address = wifi_status.ip_address or metrics.ip_address or "--"
        temperature = "--" if metrics.temperature_c is None else f"{metrics.temperature_c:.1f}"
        text = (
            f"Wi-Fi:{connection}  IP:{ip_address}  "
            f"{temperature}C  {metrics.clock}"
        )
        self.set_text("t_home_network", text, force, page)

    def set_bluetooth_status(self, text: str, force: bool = False) -> None:
        self.set_text(
            "t_bt_status", text, force, self.config.bluetooth_page_name
        )

    def render_bluetooth(
        self,
        devices: list[BluetoothDevice],
        selected_address: str,
        page_index: int,
        force: bool = False,
    ) -> None:
        page = self.config.bluetooth_page_name
        selected = next(
            (item for item in devices if item.address == selected_address), None
        )
        connected = next((item for item in devices if item.connected), None)
        if connected:
            current_text = f"当前: {connected.name}"
        elif selected:
            current_text = f"已选择: {selected.name}"
        else:
            current_text = "当前: 未连接"
        self.set_text("t_bt_current", current_text, force, page)

        size = self.config.bluetooth_list_size
        page_count = max(1, math.ceil(len(devices) / size))
        page_index = max(0, min(page_index, page_count - 1))
        start = page_index * size
        visible = devices[start : start + size]
        for slot in range(size):
            button = f"b_bt{slot}"
            if slot < len(visible):
                device = visible[slot]
                prefix = "> " if device.address == selected_address else ""
                state = "* " if device.connected else "P " if device.paired else ""
                rssi = "" if device.rssi is None else f"  {device.rssi}dBm"
                label = truncate_encoded(
                    f"{prefix}{state}{device.name}{rssi}",
                    self.config.button_text_max_bytes,
                    self.config.screen_encoding,
                )
                self.set_text(button, label, force, page)
                self.set_touch(button, True, page)
            else:
                self.set_text(button, "", force, page)
                self.set_touch(button, False, page)

        self.set_text("t_bt_page", f"{page_index + 1}/{page_count}", force, page)
        self.set_touch("b_bt_prev", page_index > 0, page)
        self.set_touch("b_bt_next", page_index + 1 < page_count, page)
