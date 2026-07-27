#!/usr/bin/env python3
"""Read-only RPMsg broker observer with an HTTP/SSE dashboard."""

from __future__ import annotations

import argparse
from collections import Counter, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import signal
import socket
import statistics
import threading
import time
from pathlib import Path
from typing import Any


DEFAULT_MONITOR_SOCKET = "/run/rpmsg-broker/monitor.sock"
DEFAULT_PORT = 8092

COMMAND_NAMES = {
    1: "HEARTBEAT", 10: "CAN_ENABLE", 11: "CAN_ZERO", 12: "CAN_PVT",
    13: "CAN_STOP", 14: "CAN_MODE", 15: "CAN_INIT", 16: "MOTOR_TEST",
    17: "CAN_ORIGIN", 18: "TORQUE_TEST", 19: "MOTOR_FAULT",
    20: "SPEED_DIAG", 30: "SERVO_SET", 31: "SERVO_CENTER",
    32: "SERVO_POLARITY", 33: "SERVO_MOVE", 34: "SERVO_STATUS",
    35: "SERVO_STOP", 36: "LEG_ENABLE", 37: "LEG_MOVE",
    38: "SERVO_TEST_ONE",
    40: "IMU_INIT", 41: "IMU_READ", 42: "IMU_TELEMETRY",
    43: "IMU_CALIBRATE", 50: "BALANCE_ENABLE", 51: "BALANCE_DISABLE",
    52: "BALANCE_STATUS", 53: "BALANCE_TRIM", 54: "BALANCE_GAINS",
    55: "BALANCE_CONFIG", 56: "BALANCE_RESET", 57: "BALANCE_SPEED_LIMIT",
    58: "BALANCE_TELEMETRY", 59: "BALANCE_FILTER",
    60: "BALANCE_POSTURE", 61: "BALANCE_TORQUE_LIMIT",
    62: "CHASSIS_VELOCITY", 63: "CHASSIS_STATUS", 64: "CHASSIS_TRACK",
    65: "BALANCE_POSITION_HOLD",
    70: "GIMBAL_ENABLE", 71: "GIMBAL_DISABLE", 72: "GIMBAL_TARGET",
    73: "GIMBAL_STATUS", 74: "GIMBAL_CAL_LIMIT", 75: "GIMBAL_LIMITS",
    76: "GIMBAL_RESET", 77: "GIMBAL_ESTOP",
}


class MonitorState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.events: deque[dict[str, Any]] = deque(maxlen=600)
        self.rate_events: deque[tuple[float, str]] = deque(maxlen=2000)
        self.latencies: deque[float] = deque(maxlen=300)
        self.command_counts: Counter[str] = Counter()
        self.totals = {"tx_frames": 0, "rx_frames": 0, "errors": 0,
                       "tx_bytes": 0, "rx_bytes": 0}
        self.serial = 0
        self.connected = False
        self.connected_at = 0.0
        self.last_event_at = 0.0
        self.last_error = "waiting for broker monitor socket"
        self.reconnects = 0

    def set_connection(self, connected: bool, error: str = "") -> None:
        with self.condition:
            if connected and not self.connected:
                self.connected_at = time.monotonic()
            if not connected and self.connected:
                self.reconnects += 1
            self.connected = connected
            if error:
                self.last_error = error
            elif connected:
                self.last_error = ""
            self.condition.notify_all()

    def add_event(self, event: dict[str, Any]) -> None:
        now = time.monotonic()
        direction = str(event.get("direction", "unknown"))
        command_type = int(event.get("type", 0))
        with self.condition:
            self.serial += 1
            event = dict(event)
            event["serial"] = self.serial
            event["command"] = COMMAND_NAMES.get(command_type,
                                                   f"TYPE_{command_type}")
            self.events.append(event)
            self.rate_events.append((now, direction))
            self.command_counts[event["command"]] += 1
            latency = event.get("latency_ms", -1)
            if direction == "rx" and isinstance(latency, (int, float)) and latency >= 0:
                self.latencies.append(float(latency))
            totals = event.get("totals")
            if isinstance(totals, dict):
                for key in self.totals:
                    if key in totals:
                        self.totals[key] = int(totals[key])
            self.last_event_at = now
            self.condition.notify_all()

    def _snapshot_locked(self) -> dict[str, Any]:
        now = time.monotonic()
        while self.rate_events and now - self.rate_events[0][0] > 5.0:
            self.rate_events.popleft()
        one_second = [direction for timestamp, direction in self.rate_events
                      if now - timestamp <= 1.0]
        latencies = sorted(self.latencies)
        p95_index = max(0, int(len(latencies) * 0.95) - 1)
        top_commands = self.command_counts.most_common(12)
        return {
            "connected": self.connected,
            "link_state": "online" if self.connected else "offline",
            "connected_seconds": round(now - self.connected_at, 1)
            if self.connected else 0,
            "idle_seconds": round(now - self.last_event_at, 2)
            if self.last_event_at else None,
            "last_error": self.last_error,
            "reconnects": self.reconnects,
            "rates": {
                "tx_fps": one_second.count("tx"),
                "rx_fps": one_second.count("rx"),
                "error_fps": one_second.count("error") + one_second.count("drop"),
            },
            "latency": {
                "latest_ms": self.latencies[-1] if self.latencies else None,
                "average_ms": round(statistics.fmean(latencies), 2)
                if latencies else None,
                "p95_ms": round(latencies[p95_index], 2) if latencies else None,
                "samples": list(self.latencies)[-80:],
            },
            "totals": dict(self.totals),
            "commands": [{"name": name, "count": count}
                         for name, count in top_commands],
            "events": list(self.events)[-100:],
            "serial": self.serial,
        }

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return self._snapshot_locked()

    def wait_after(self, serial: int, timeout: float = 1.0) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        with self.condition:
            if self.serial <= serial:
                self.condition.wait(timeout)
            events = [event for event in self.events
                      if int(event["serial"]) > serial]
            return events, self._snapshot_locked()


class BrokerObserver(threading.Thread):
    def __init__(self, socket_path: str, state: MonitorState,
                 stopping: threading.Event) -> None:
        super().__init__(name="rpmsg-broker-observer", daemon=True)
        self.socket_path = socket_path
        self.state = state
        self.stopping = stopping

    def run(self) -> None:
        while not self.stopping.is_set():
            transport: socket.socket | None = None
            try:
                transport = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
                transport.settimeout(1.0)
                transport.connect(self.socket_path)
                self.state.set_connection(True)
                while not self.stopping.is_set():
                    try:
                        packet = transport.recv(4096)
                    except socket.timeout:
                        continue
                    if not packet:
                        raise ConnectionError("broker monitor socket closed")
                    event = json.loads(packet.decode("utf-8"))
                    if isinstance(event, dict):
                        self.state.add_event(event)
            except (OSError, ValueError, json.JSONDecodeError) as error:
                self.state.set_connection(False, str(error))
                self.stopping.wait(1.0)
            finally:
                if transport is not None:
                    transport.close()


def make_handler(state: MonitorState, html: bytes):
    class Handler(BaseHTTPRequestHandler):
        server_version = "rpmsg-monitor/1.0"

        def log_message(self, fmt: str, *args: Any) -> None:
            return

        def _send_json(self, payload: dict[str, Any]) -> None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            path = self.path.split("?", 1)[0]
            if path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(html)))
                self.end_headers()
                self.wfile.write(html)
                return
            if path == "/api/snapshot":
                self._send_json(state.snapshot())
                return
            if path == "/api/events":
                self._serve_events()
                return
            if path == "/api/health":
                snapshot = state.snapshot()
                self._send_json({"ok": snapshot["connected"],
                                 "state": snapshot["link_state"]})
                return
            self.send_error(404)

        def _serve_events(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            serial = 0
            try:
                while True:
                    events, snapshot = state.wait_after(serial, 1.0)
                    for event in events:
                        serial = max(serial, int(event["serial"]))
                        data = json.dumps(event, separators=(",", ":"))
                        self.wfile.write(f"event: packet\ndata: {data}\n\n".encode())
                    data = json.dumps(snapshot, separators=(",", ":"))
                    self.wfile.write(f"event: snapshot\ndata: {data}\n\n".encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--monitor-socket", default=DEFAULT_MONITOR_SOCKET)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--html", default=str(Path(__file__).with_name("rpmsg_monitor.html")))
    args = parser.parse_args()

    html = Path(args.html).read_bytes()
    state = MonitorState()
    stopping = threading.Event()
    observer = BrokerObserver(args.monitor_socket, state, stopping)
    server = ThreadingHTTPServer((args.bind, args.port), make_handler(state, html))
    server.daemon_threads = True

    def stop(_signum: int, _frame: Any) -> None:
        stopping.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    observer.start()
    print(f"rpmsg-monitor: http://{args.bind}:{args.port} source={args.monitor_socket}",
          flush=True)
    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        stopping.set()
        observer.join(timeout=2.0)
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
