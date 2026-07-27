import logging
import queue
import subprocess
import time

from .bluetooth import BluetoothDevice, BluetoothManager
from .config import AppConfig
from .expression import ExpressionManager
from .expression_api import ExpressionSocketServer, execute_expression_command
from .network import CommandResult, WifiManager, WifiNetwork, WifiStatus
from .protocol import EventKind, TjcFrameParser, parse_screen_frame
from .screen import TjcScreen
from .serial_port import SerialPort
from .system_status import SystemMetricsCollector
from .worker import TaskWorker, WifiWorker, WorkerResult


LOG = logging.getLogger(__name__)

FACE_DEFAULT = 0
FACE_SCANNING = 1
FACE_WIFI_SUCCESS = 2
FACE_BLUETOOTH_SUCCESS = 3
FACE_FAILURE = 4
FACE_SYSTEM_ALERT = 5

FACE_SCAN_SECONDS = 3.0
FACE_SUCCESS_SECONDS = 6.0
FACE_FAILURE_SECONDS = 8.0

PRIORITY_SCAN = 20
PRIORITY_RESULT = 40
PRIORITY_SYSTEM_ALERT = 100


class WifiScreenController:
    def __init__(self, config: AppConfig):
        self.config = config
        self.serial = SerialPort(config.serial_device, config.baudrate)
        self.parser = TjcFrameParser()
        self.worker = WifiWorker(WifiManager(config.wifi_interface))
        self.bluetooth_worker = TaskWorker(BluetoothManager(), "bluetooth-worker")
        self.screen = TjcScreen(config, self._write_serial)
        self.system_metrics = SystemMetricsCollector(config.wifi_interface)
        self.expression_manager = ExpressionManager(
            self._apply_expression, time.monotonic, FACE_DEFAULT
        )
        self.expression_api = ExpressionSocketServer(
            config.expression_socket_path, config.expression_socket_mode
        )
        self.running = False
        self.networks: list[WifiNetwork] = []
        self.wifi_status = WifiStatus(False)
        self.bluetooth_devices: list[BluetoothDevice] = []
        self.selected_bluetooth_address = ""
        self.bluetooth_page_index = 0
        self.connected_bluetooth_addresses: set[str] = set()
        self.bluetooth_connection_state_initialized = False
        self.system_alert = False
        self.selected_ssid = ""
        self.page_index = 0
        self.pending: set[str] = set()
        self.bluetooth_pending: set[str] = set()
        self.next_serial_open = 0.0
        self.next_scan = 0.0
        self.next_status = 0.0
        self.next_system_refresh = 0.0
        self.next_home_refresh = 0.0
        self.next_bluetooth_status = 0.0
        self.active_page = config.home_page_name

    def stop(self) -> None:
        self.running = False

    def run(self) -> None:
        LOG.info(
            "starting with serial=%s baud=%d wifi=%s",
            self.config.serial_device,
            self.config.baudrate,
            self.config.wifi_interface,
        )
        self.running = True
        self.worker.start()
        self.bluetooth_worker.start()
        try:
            self.expression_api.start()
            LOG.info("expression API listening on %s", self.config.expression_socket_path)
            while self.running:
                now = time.monotonic()
                self._ensure_serial(now)
                self._read_serial()
                self._drain_expression_commands()
                self._drain_worker_results()
                self._drain_bluetooth_results()
                self._schedule_periodic(now)
                self.serial.wait_readable(0.1)
        finally:
            self.expression_api.stop()
            self.worker.stop()
            self.bluetooth_worker.stop()
            self.serial.close()
            LOG.info("stopped")

    def _ensure_serial(self, now: float) -> None:
        if self.serial.is_open or now < self.next_serial_open:
            return
        try:
            self.serial.open()
            LOG.info("opened serial port %s", self.config.serial_device)
            self.screen.clear_cache()
            self._write_serial(b"\x00\xff\xff\xff")
            self.screen.raw_command("bkcmd=0")
            self._apply_expression(
                self.expression_manager.current_expression, force_page=False
            )
            self._render_all()
            self._submit("scan")
            self._submit("status")
        except OSError as exc:
            LOG.warning("cannot open serial port: %s", exc)
            self.serial.close()
            self.next_serial_open = now + 2.0

    def _write_serial(self, data: bytes) -> None:
        try:
            self.serial.write(data)
        except (OSError, TimeoutError) as exc:
            LOG.warning("serial write failed: %s", exc)
            self.serial.close()
            self.next_serial_open = time.monotonic() + 2.0

    def _read_serial(self) -> None:
        if not self.serial.is_open:
            return
        try:
            data = self.serial.read()
        except OSError as exc:
            LOG.warning("serial read failed: %s", exc)
            self.serial.close()
            return
        for frame in self.parser.feed(data):
            event = parse_screen_frame(frame)
            LOG.debug("screen event=%s data_len=%d", event.kind.name, len(event.data))
            self._handle_event(event.kind, event.data)

    def _handle_event(self, kind: EventKind, data: bytes) -> None:
        if kind == EventKind.SCREEN_READY:
            self.active_page = self.config.home_page_name
            self.screen.clear_cache()
            self._render_home(force=True)
            self._submit("status")
        elif kind == EventKind.SYNC:
            self.active_page = self.config.page_name
            self.screen.clear_cache()
            self._render_all()
            self._request_expression(
                FACE_SCANNING,
                source="wifi.scan",
                duration_seconds=FACE_SCAN_SECONDS,
                priority=PRIORITY_SCAN,
            )
            self._submit("scan")
            self._submit("status")
        elif kind == EventKind.SYSTEM_SYNC:
            self.active_page = self.config.system_page_name
            self._render_system(force=True)
            self.next_system_refresh = (
                time.monotonic() + self.config.system_refresh_interval_seconds
            )
        elif kind == EventKind.SYSTEM_REFRESH:
            self._render_system(force=True)
            self.next_system_refresh = (
                time.monotonic() + self.config.system_refresh_interval_seconds
            )
        elif kind == EventKind.POWEROFF:
            LOG.warning("poweroff requested from serial screen")
            # The screen is about to lose its Linux-side updater.  Latch a
            # visible red/offline state first so the user gets feedback.
            self.screen.show_powered_off()
            time.sleep(0.25)
            subprocess.run(["/usr/bin/systemctl", "poweroff"], check=False)
        elif kind == EventKind.HOME_SYNC:
            self.active_page = self.config.home_page_name
            self._render_home(force=True)
            self._submit("status")
            self.next_home_refresh = (
                time.monotonic() + self.config.system_refresh_interval_seconds
            )
        elif kind == EventKind.BLUETOOTH_SYNC:
            self.active_page = self.config.bluetooth_page_name
            self.bluetooth_page_index = 0
            self.screen.render_bluetooth(
                self.bluetooth_devices,
                self.selected_bluetooth_address,
                self.bluetooth_page_index,
                force=True,
            )
            self.screen.set_bluetooth_status("状态: 正在扫描...", force=True)
            self._submit_bluetooth("scan")
        elif kind == EventKind.BLUETOOTH_SCAN:
            self.bluetooth_page_index = 0
            self.screen.set_bluetooth_status("状态: 正在扫描...", force=True)
            self._submit_bluetooth("scan")
        elif kind == EventKind.BLUETOOTH_SELECT:
            index = (
                self.bluetooth_page_index * self.config.bluetooth_list_size + data[0]
            )
            if index < len(self.bluetooth_devices):
                self.selected_bluetooth_address = self.bluetooth_devices[index].address
                self.screen.render_bluetooth(
                    self.bluetooth_devices,
                    self.selected_bluetooth_address,
                    self.bluetooth_page_index,
                    force=True,
                )
                self.screen.set_bluetooth_status("状态: 已选择设备", force=True)
        elif kind == EventKind.BLUETOOTH_PAIR:
            self._start_bluetooth_action("pair", "状态: 正在配对...")
        elif kind == EventKind.BLUETOOTH_CONNECT:
            self._start_bluetooth_action("connect", "状态: 正在连接...")
        elif kind == EventKind.BLUETOOTH_DISCONNECT:
            self._start_bluetooth_action("disconnect", "状态: 正在断开...")
        elif kind == EventKind.BLUETOOTH_PREVIOUS_PAGE:
            self._change_bluetooth_page(-1)
        elif kind == EventKind.BLUETOOTH_NEXT_PAGE:
            self._change_bluetooth_page(1)
        elif kind == EventKind.FACE_SYNC:
            self.active_page = self.config.face_page_name
            if self.expression_manager.has_active_requests():
                if data[0] != self.expression_manager.current_expression:
                    self._apply_expression(
                        self.expression_manager.current_expression,
                        force_page=False,
                    )
            else:
                self.expression_manager.set_default(data[0], apply=False)
        elif kind == EventKind.RESCAN:
            self._request_expression(
                FACE_SCANNING,
                source="wifi.scan",
                duration_seconds=FACE_SCAN_SECONDS,
                priority=PRIORITY_SCAN,
            )
            self.screen.set_status("正在扫描 Wi-Fi...")
            self._submit("scan")
        elif kind == EventKind.SELECT:
            absolute_index = self.page_index * self.config.list_page_size + data[0]
            if absolute_index < len(self.networks):
                self.selected_ssid = self.networks[absolute_index].ssid
                self.screen.set_selected(self.networks[absolute_index])
                self.screen.clear_password()
                self.screen.render_networks(
                    self.networks, self.page_index, self.selected_ssid
                )
        elif kind == EventKind.CONNECT:
            self._connect(data)
        elif kind == EventKind.DISCONNECT:
            self.screen.set_status("正在断开 Wi-Fi...")
            self._submit("disconnect")
        elif kind == EventKind.NEXT_PAGE:
            self._change_page(1)
        elif kind == EventKind.PREVIOUS_PAGE:
            self._change_page(-1)

    def _connect(self, password_bytes: bytes) -> None:
        if not self.selected_ssid:
            self.screen.set_status("请先选择一个 Wi-Fi")
            return
        password = password_bytes.decode(self.config.screen_encoding, errors="replace")
        self.screen.set_status(f"正在连接 {self.selected_ssid}...")
        self._submit("connect", self.selected_ssid, password)

    def _change_page(self, delta: int) -> None:
        max_page = max(0, (len(self.networks) - 1) // self.config.list_page_size)
        self.page_index = max(0, min(max_page, self.page_index + delta))
        self.screen.render_networks(self.networks, self.page_index, self.selected_ssid)

    def _change_bluetooth_page(self, delta: int) -> None:
        max_page = max(
            0,
            (len(self.bluetooth_devices) - 1) // self.config.bluetooth_list_size,
        )
        self.bluetooth_page_index = max(
            0, min(max_page, self.bluetooth_page_index + delta)
        )
        self.screen.render_bluetooth(
            self.bluetooth_devices,
            self.selected_bluetooth_address,
            self.bluetooth_page_index,
        )

    def _submit(self, operation: str, *args: object) -> None:
        if operation in self.pending:
            return
        self.pending.add(operation)
        self.worker.submit(operation, *args)

    def _submit_bluetooth(self, operation: str, *args: object) -> None:
        if operation in self.bluetooth_pending:
            return
        self.bluetooth_pending.add(operation)
        self.bluetooth_worker.submit(operation, *args)

    def _start_bluetooth_action(self, operation: str, status: str) -> None:
        device = next(
            (
                item
                for item in self.bluetooth_devices
                if item.address == self.selected_bluetooth_address
            ),
            None,
        )
        if operation == "disconnect" and device is None:
            device = next((item for item in self.bluetooth_devices if item.connected), None)
        if device is None:
            self.screen.set_bluetooth_status("状态: 请先选择设备", force=True)
            return
        self.screen.set_bluetooth_status(status, force=True)
        self._submit_bluetooth(operation, device.address)

    def _request_expression(
        self,
        state: int,
        *,
        source: str,
        duration_seconds: float,
        priority: int,
        force_page: bool = False,
    ) -> None:
        self.expression_manager.show(
            state,
            source=source,
            duration_ms=round(duration_seconds * 1000),
            priority=priority,
            force_page=force_page,
        )

    def _apply_expression(self, state: int, force_page: bool) -> None:
        reload_page = force_page or self.active_page == self.config.face_page_name
        self.screen.set_face_state(state, reload_page=reload_page)
        if reload_page:
            self.active_page = self.config.face_page_name
        LOG.info(
            "expression changed state=%d force_page=%s", state, force_page
        )

    def _drain_expression_commands(self) -> None:
        while True:
            try:
                command = self.expression_api.commands.get_nowait()
            except queue.Empty:
                return
            response = execute_expression_command(
                self.expression_manager, command.payload
            )
            command.response.put(response)

    def _update_system_alert(self, metrics: object) -> None:
        temperature = getattr(metrics, "temperature_c", None)
        alert = (
            getattr(metrics, "cpu_percent", 0) >= 95
            or getattr(metrics, "memory_percent", 0) >= 90
            or getattr(metrics, "disk_percent", 0) >= 95
            or (temperature is not None and temperature >= 80.0)
        )
        self.system_alert = alert
        if alert and not self.expression_manager.has_request("system.alert"):
            self.expression_manager.show(
                FACE_SYSTEM_ALERT,
                source="system.alert",
                duration_ms=0,
                priority=PRIORITY_SYSTEM_ALERT,
                force_page=False,
            )
        elif not alert and self.expression_manager.has_request("system.alert"):
            self.expression_manager.clear("system.alert")

    def _schedule_periodic(self, now: float) -> None:
        self.expression_manager.tick()
        if now >= self.next_bluetooth_status:
            self._submit_bluetooth("connected_addresses")
            self.next_bluetooth_status = (
                now + self.config.bluetooth_status_interval_seconds
            )
        if now >= self.next_status:
            self._submit("status")
            self.next_status = now + self.config.status_interval_seconds
        if now >= self.next_scan:
            self._submit("scan")
            self.next_scan = now + self.config.scan_interval_seconds
        if (
            self.active_page == self.config.system_page_name
            and now >= self.next_system_refresh
        ):
            self._render_system()
            self.next_system_refresh = (
                now + self.config.system_refresh_interval_seconds
            )
        if (
            self.active_page == self.config.home_page_name
            and now >= self.next_home_refresh
        ):
            self._render_home()
            self.next_home_refresh = (
                now + self.config.system_refresh_interval_seconds
            )

    def _drain_worker_results(self) -> None:
        while True:
            try:
                result = self.worker.results.get_nowait()
            except queue.Empty:
                return
            self.pending.discard(result.operation)
            self._handle_worker_result(result)

    def _handle_worker_result(self, result: WorkerResult) -> None:
        if result.error:
            LOG.warning("%s failed: %s", result.operation, result.error)
            self._request_expression(
                FACE_FAILURE,
                source="wifi.error",
                duration_seconds=FACE_FAILURE_SECONDS,
                priority=PRIORITY_RESULT,
            )
            self.screen.set_status(f"操作失败: {result.error[:40]}")
            return

        if result.operation == "scan":
            self.networks = result.value
            max_page = max(0, (len(self.networks) - 1) // self.config.list_page_size)
            self.page_index = min(self.page_index, max_page)
            self.screen.render_networks(self.networks, self.page_index, self.selected_ssid)
            self.screen.set_status(f"发现 {len(self.networks)} 个 Wi-Fi")
        elif result.operation == "status":
            status: WifiStatus = result.value
            self.wifi_status = status
            self.screen.set_connection(status)
        elif result.operation in ("connect", "disconnect"):
            command_result: CommandResult = result.value
            if command_result.success:
                text = "Wi-Fi 连接成功" if result.operation == "connect" else "Wi-Fi 已断开"
                self.screen.set_status(text)
                self.screen.clear_password()
                if result.operation == "connect":
                    self._request_expression(
                        FACE_WIFI_SUCCESS,
                        source="wifi.connection",
                        duration_seconds=FACE_SUCCESS_SECONDS,
                        priority=PRIORITY_RESULT,
                    )
            else:
                LOG.warning("%s failed: %s", result.operation, command_result.message)
                self._request_expression(
                    FACE_FAILURE,
                    source="wifi.error",
                    duration_seconds=FACE_FAILURE_SECONDS,
                    priority=PRIORITY_RESULT,
                )
                self.screen.set_status(f"连接失败: {command_result.message[:36]}")
            self._submit("status")
            self._submit("scan")

    def _drain_bluetooth_results(self) -> None:
        while True:
            try:
                result = self.bluetooth_worker.results.get_nowait()
            except queue.Empty:
                return
            self.bluetooth_pending.discard(result.operation)
            self._handle_bluetooth_result(result)

    def _handle_bluetooth_result(self, result: WorkerResult) -> None:
        if result.error:
            LOG.warning("bluetooth %s failed: %s", result.operation, result.error)
            if result.operation == "connected_addresses":
                return
            self._request_expression(
                FACE_FAILURE,
                source="bluetooth.error",
                duration_seconds=FACE_FAILURE_SECONDS,
                priority=PRIORITY_RESULT,
            )
            self.screen.set_bluetooth_status(
                f"状态: 操作失败 {result.error[:28]}", force=True
            )
            return
        if result.operation == "connected_addresses":
            connected = set(result.value)
            if self.bluetooth_connection_state_initialized:
                newly_connected = connected - self.connected_bluetooth_addresses
                if newly_connected:
                    LOG.info(
                        "bluetooth device connected externally: %s",
                        ", ".join(sorted(newly_connected)),
                    )
                    self._request_expression(
                        FACE_BLUETOOTH_SUCCESS,
                        source="bluetooth.connection",
                        duration_seconds=FACE_SUCCESS_SECONDS,
                        priority=PRIORITY_RESULT,
                    )
                if connected != self.connected_bluetooth_addresses:
                    self._submit_bluetooth("devices")
            self.connected_bluetooth_addresses = connected
            self.bluetooth_connection_state_initialized = True
            return
        if result.operation in ("scan", "devices"):
            self.bluetooth_devices = result.value
            max_page = max(
                0,
                (len(self.bluetooth_devices) - 1)
                // self.config.bluetooth_list_size,
            )
            self.bluetooth_page_index = min(self.bluetooth_page_index, max_page)
            if self.selected_bluetooth_address not in {
                item.address for item in self.bluetooth_devices
            }:
                self.selected_bluetooth_address = ""
            self.screen.render_bluetooth(
                self.bluetooth_devices,
                self.selected_bluetooth_address,
                self.bluetooth_page_index,
                force=True,
            )
            if result.operation == "scan":
                self.screen.set_bluetooth_status(
                    f"状态: 发现 {len(self.bluetooth_devices)} 个设备", force=True
                )
            return

        command_result: CommandResult = result.value
        if command_result.success:
            labels = {"pair": "配对成功", "connect": "连接成功", "disconnect": "已断开"}
            self.screen.set_bluetooth_status(
                f"状态: {labels[result.operation]}", force=True
            )
            if result.operation in ("pair", "connect"):
                self._request_expression(
                    FACE_BLUETOOTH_SUCCESS,
                    source="bluetooth.connection",
                    duration_seconds=FACE_SUCCESS_SECONDS,
                    priority=PRIORITY_RESULT,
                )
            if result.operation == "connect" and self.selected_bluetooth_address:
                self.connected_bluetooth_addresses.add(
                    self.selected_bluetooth_address
                )
                self.bluetooth_connection_state_initialized = True
            elif result.operation == "disconnect":
                self.connected_bluetooth_addresses.discard(
                    self.selected_bluetooth_address
                )
            self._submit_bluetooth("devices")
        else:
            self._request_expression(
                FACE_FAILURE,
                source="bluetooth.error",
                duration_seconds=FACE_FAILURE_SECONDS,
                priority=PRIORITY_RESULT,
            )
            self.screen.set_bluetooth_status(
                f"状态: 操作失败 {command_result.message[:28]}", force=True
            )

    def _render_all(self) -> None:
        self.screen.set_status("Wi-Fi 控制器已启动")
        selected = next(
            (item for item in self.networks if item.ssid == self.selected_ssid), None
        )
        self.screen.set_selected(selected)
        self.screen.render_networks(self.networks, self.page_index, self.selected_ssid)

    def _render_system(self, force: bool = False) -> None:
        try:
            metrics = self.system_metrics.collect()
        except (OSError, ValueError) as exc:
            LOG.warning("cannot collect system metrics: %s", exc)
            return
        self._update_system_alert(metrics)
        self.screen.render_system(metrics, force=force)

    def _render_home(self, force: bool = False) -> None:
        try:
            metrics = self.system_metrics.collect()
        except (OSError, ValueError) as exc:
            LOG.warning("cannot collect home metrics: %s", exc)
            return
        self._update_system_alert(metrics)
        self.screen.render_home(metrics, self.wifi_status, force=force)
