import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class ComponentConfig:
    current: str = "t_current"
    status: str = "t_status"
    selected: str = "t_selected"
    password: str = "t_password"
    page: str = "t_page"
    wifi_button_pattern: str = "b_wifi{}"
    previous: str = "b_prev"
    next: str = "b_next"
    home_title: str = "t_home_title"
    power: str = "b_home_off"


@dataclass(frozen=True)
class AppConfig:
    serial_device: str = "/dev/ttyAMA2"
    baudrate: int = 115200
    screen_encoding: str = "gbk"
    wifi_interface: str = "wlan0"
    scan_interval_seconds: float = 15.0
    status_interval_seconds: float = 2.0
    system_refresh_interval_seconds: float = 1.0
    list_page_size: int = 6
    button_text_max_bytes: int = 64
    page_name: str = "wifi"
    system_page_name: str = "system"
    home_page_name: str = "home"
    bluetooth_page_name: str = "bluetooth"
    face_page_name: str = "page0"
    face_state_variable: str = "sys2"
    expression_socket_path: str = "/run/wifi-screen/face.sock"
    expression_socket_mode: int = 0o666
    bluetooth_list_size: int = 5
    bluetooth_status_interval_seconds: float = 2.0
    components: ComponentConfig = field(default_factory=ComponentConfig)


def load_config(path: str | Path) -> AppConfig:
    raw: dict[str, Any] = json.loads(Path(path).read_text(encoding="utf-8"))
    component_raw = raw.pop("components", {})
    config = AppConfig(components=ComponentConfig(**component_raw), **raw)
    if config.list_page_size < 1:
        raise ValueError("list_page_size must be at least 1")
    if config.scan_interval_seconds < 2:
        raise ValueError("scan_interval_seconds must be at least 2 seconds")
    if config.system_refresh_interval_seconds < 0.5:
        raise ValueError("system_refresh_interval_seconds must be at least 0.5 seconds")
    if config.bluetooth_list_size < 1:
        raise ValueError("bluetooth_list_size must be at least 1")
    if config.bluetooth_status_interval_seconds < 1:
        raise ValueError("bluetooth_status_interval_seconds must be at least 1 second")
    if not config.expression_socket_path:
        raise ValueError("expression_socket_path must not be empty")
    if not 0 <= config.expression_socket_mode <= 0o777:
        raise ValueError("expression_socket_mode must be between 0 and 511")
    return config
