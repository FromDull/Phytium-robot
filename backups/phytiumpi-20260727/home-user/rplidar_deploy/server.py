#!/usr/bin/env python3
import base64
import hashlib
import hmac
import json
import math
import os
import pty
import select
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

ICP_CELL_SIZE = 0.08
ICP_MAX_DISTANCE = 0.20
ICP_MIN_MATCHES = 40
ICP_TRIM_RATIO = 0.75
ICP_MAX_RMS = 0.08
MOTION_DEADBAND = 0.002
STATIC_TRANSLATION_THRESHOLD = 0.012
STATIC_ROTATION_THRESHOLD = 0.018
RAW_SCAN_STATIC_RMS = 0.012
MIN_MOTION_TRANSLATION = 0.008
MIN_MOTION_ROTATION = 0.012
MAX_FRAME_TRANSLATION = 0.035
MAX_FRAME_ROTATION = 0.08
MOTION_CONFIRM_FRAMES = 2


def format_uptime(seconds):
    seconds = max(0, int(seconds))
    days, seconds = divmod(seconds, 86400)
    hours, seconds = divmod(seconds, 3600)
    minutes, _ = divmod(seconds, 60)
    if days:
        return f"{days}天 {hours:02d}:{minutes:02d}"
    return f"{hours:02d}:{minutes:02d}"


class SystemMonitor:
    """Low-overhead host metrics available inside the host-network container."""

    def __init__(self):
        self.previous_cpu = None
        self.last_update = 0.0
        self.last_snapshot = {}

    @staticmethod
    def _read_cpu_times():
        fields = Path("/proc/stat").read_text(encoding="ascii").splitlines()[0].split()[1:]
        values = [int(value) for value in fields]
        total = sum(values)
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        return total, idle

    @staticmethod
    def _read_memory():
        values = {}
        for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
            key, value = line.split(":", 1)
            values[key] = int(value.strip().split()[0]) * 1024
        total = values.get("MemTotal", 0)
        available = values.get("MemAvailable", values.get("MemFree", 0))
        used = max(0, total - available)
        return total, used

    @staticmethod
    def _read_temperature():
        for path in sorted(Path("/sys/class/thermal").glob("thermal_zone*/temp")):
            try:
                value = float(path.read_text(encoding="ascii").strip())
                return value / 1000.0 if value > 1000 else value
            except (OSError, ValueError):
                continue
        return None

    @staticmethod
    def _read_ip():
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.connect(("192.0.2.1", 80))
                return sock.getsockname()[0]
        except OSError:
            return "--"

    def collect(self):
        now = time.monotonic()
        if now - self.last_update < 1.0 and self.last_snapshot:
            return dict(self.last_snapshot)

        try:
            total, idle = self._read_cpu_times()
            if self.previous_cpu is None:
                cpu_percent = 0.0
            else:
                delta_total = total - self.previous_cpu[0]
                delta_idle = idle - self.previous_cpu[1]
                cpu_percent = 0.0 if delta_total <= 0 else 100.0 * (1.0 - delta_idle / delta_total)
            self.previous_cpu = (total, idle)
            memory_total, memory_used = self._read_memory()
            with open("/proc/uptime", encoding="ascii") as uptime_file:
                uptime_seconds = float(uptime_file.read().split()[0])
            self.last_snapshot = {
                "cpu_percent": round(max(0.0, min(100.0, cpu_percent)), 1),
                "load": [round(value, 2) for value in os.getloadavg()],
                "memory_total_mb": round(memory_total / 1024 / 1024),
                "memory_used_mb": round(memory_used / 1024 / 1024),
                "memory_percent": round(100.0 * memory_used / memory_total, 1) if memory_total else 0.0,
                "temperature_c": self._read_temperature(),
                "uptime": format_uptime(uptime_seconds),
                "ip_address": self._read_ip(),
                "hostname": socket.gethostname(),
            }
        except (OSError, ValueError, IndexError):
            self.last_snapshot = {"error": "system metrics unavailable"}
        self.last_update = now
        return dict(self.last_snapshot)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return ax + c * bx - s * by, ay + s * bx + c * by, normalize_angle(ayaw + byaw)


def inverse_pose(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return -c * x - s * y, s * x - c * y, -yaw


def rigid_transform(source, target):
    sx = sum(point[0] for point in source) / len(source)
    sy = sum(point[1] for point in source) / len(source)
    tx = sum(point[0] for point in target) / len(target)
    ty = sum(point[1] for point in target) / len(target)
    cross = 0.0
    dot = 0.0
    for (px, py), (qx, qy) in zip(source, target):
        px -= sx
        py -= sy
        qx -= tx
        qy -= ty
        cross += px * qy - py * qx
        dot += px * qx + py * qy
    yaw = math.atan2(cross, dot)
    c, s = math.cos(yaw), math.sin(yaw)
    return tx - c * sx + s * sy, ty - s * sx - c * sy, yaw


def build_grid(points, cell_size):
    grid = {}
    for point in points:
        key = (math.floor(point[0] / cell_size), math.floor(point[1] / cell_size))
        grid.setdefault(key, []).append(point)
    return grid


def nearest_from_grid(grid, point, cell_size, radius):
    cell_x = math.floor(point[0] / cell_size)
    cell_y = math.floor(point[1] / cell_size)
    best = None
    best_distance_sq = float("inf")
    for offset_x in range(-radius, radius + 1):
        for offset_y in range(-radius, radius + 1):
            for candidate in grid.get((cell_x + offset_x, cell_y + offset_y), ()):
                distance_sq = (candidate[0] - point[0]) ** 2 + (
                    candidate[1] - point[1]
                ) ** 2
                if distance_sq < best_distance_sq:
                    best = candidate
                    best_distance_sq = distance_sq
    return best, best_distance_sq


def icp_2d(source, target, initial=(0.0, 0.0, 0.0)):
    """Lightweight robust point-to-point ICP for consecutive 2D scans."""
    metrics = {"inliers": 0, "rms": None, "zero_rms": None}
    if len(source) < ICP_MIN_MATCHES or len(target) < ICP_MIN_MATCHES:
        return initial, False, metrics

    transform = initial
    max_distance_sq = ICP_MAX_DISTANCE * ICP_MAX_DISTANCE
    grid = build_grid(target, ICP_CELL_SIZE)
    search_radius = math.ceil(ICP_MAX_DISTANCE / ICP_CELL_SIZE)
    zero_matches = []
    for point in source:
        _, distance_sq = nearest_from_grid(grid, point, ICP_CELL_SIZE, search_radius)
        if distance_sq <= max_distance_sq:
            zero_matches.append(distance_sq)
    if len(zero_matches) >= ICP_MIN_MATCHES:
        zero_matches.sort()
        zero_matches = zero_matches[: max(ICP_MIN_MATCHES, int(len(zero_matches) * ICP_TRIM_RATIO))]
        zero_rms = math.sqrt(sum(zero_matches) / len(zero_matches))
    else:
        zero_rms = None
    metrics["zero_rms"] = zero_rms

    for _ in range(6):
        c, s = math.cos(transform[2]), math.sin(transform[2])
        transformed = [
            (c * x - s * y + transform[0], s * x + c * y + transform[1])
            for x, y in source
        ]
        matches = []
        for point in transformed:
            nearest, distance_sq = nearest_from_grid(
                grid, point, ICP_CELL_SIZE, search_radius
            )
            if nearest is not None and distance_sq <= max_distance_sq:
                matches.append((distance_sq, point, nearest))
        if len(matches) < ICP_MIN_MATCHES:
            return transform, False, metrics

        matches.sort(key=lambda item: item[0])
        matches = matches[: max(ICP_MIN_MATCHES, int(len(matches) * ICP_TRIM_RATIO))]
        matches_source = [item[1] for item in matches]
        matches_target = [item[2] for item in matches]
        rms = math.sqrt(sum(item[0] for item in matches) / len(matches))
        metrics = {"inliers": len(matches), "rms": rms, "zero_rms": zero_rms}
        if rms > ICP_MAX_RMS:
            return transform, False, metrics

        correction = rigid_transform(matches_source, matches_target)
        transform = compose(correction, transform)
        if sum(abs(value) for value in correction) < 1e-4:
            break
    return transform, True, metrics


class LatestData(Node):
    def __init__(self):
        super().__init__("lidar_web_viewer")
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.lock = threading.Lock()
        self.scan = None
        self.odom = None
        self.previous_points = None
        self.previous_beams = None
        self.pose = (0.0, 0.0, 0.0)
        self.last_stamp = None
        self.odom_quality = {"inliers": 0, "rms": None, "converged": False}
        self.motion_candidate = (0.0, 0.0, 0.0)
        self.motion_confirmations = 0
        self.system_monitor = SystemMonitor()
        self.scan_times = deque(maxlen=80)
        self.last_scan_monotonic = None
        self.path = deque([(0.0, 0.0)], maxlen=400)
        self.events = deque(maxlen=12)
        self.radar_seen = False
        self._add_event_locked("网页仪表盘已启动")
        self.odom_publisher = self.create_publisher(Odometry, "/scan_odom", 10)
        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.on_scan, scan_qos
        )

    def _add_event_locked(self, message):
        self.events.appendleft(
            {"time": time.strftime("%H:%M:%S"), "message": message}
        )

    def reset_odometry(self):
        with self.lock:
            self.pose = (0.0, 0.0, 0.0)
            self.path.clear()
            self.path.append((0.0, 0.0))
            self.previous_points = None
            self.previous_beams = None
            self.motion_candidate = (0.0, 0.0, 0.0)
            self.motion_confirmations = 0
            self.odom_quality = {"inliers": 0, "rms": None, "converged": False}
            self._add_event_locked("里程计原点已重置")

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            age = None if self.last_scan_monotonic is None else now - self.last_scan_monotonic
            scan_hz = 0.0
            if len(self.scan_times) > 1:
                elapsed = self.scan_times[-1] - self.scan_times[0]
                if elapsed > 0:
                    scan_hz = (len(self.scan_times) - 1) / elapsed
            radar = {
                "online": age is not None and age < 1.5,
                "last_scan_age_ms": None if age is None else round(age * 1000),
                "scan_hz": round(scan_hz, 1),
                "point_count": 0 if self.scan is None else len(self.scan["points"]),
                "frame": None if self.scan is None else self.scan["frame"],
            }
            return {
                "system": self.system_monitor.collect(),
                "radar": radar,
                "scan": self.scan,
                "odom": self.odom,
                "path": list(self.path),
                "events": list(self.events),
            }

    def on_scan(self, msg):
        self.get_logger().info("received scan", throttle_duration_sec=5.0)
        received_at = time.monotonic()
        points = []
        scan_points = []
        beams = []
        angle = msg.angle_min
        stride = max(1, len(msg.ranges) // 360)
        for index, distance in enumerate(msg.ranges):
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                points.append({"a": angle, "r": distance})
                if index % stride == 0:
                    beams.append((index, distance))
                    scan_points.append(
                        (math.cos(angle) * distance, math.sin(angle) * distance)
                    )
            angle += msg.angle_increment

        delta = (0.0, 0.0, 0.0)
        converged = False
        raw_scan_rms = None
        if self.previous_beams is not None:
            previous = dict(self.previous_beams)
            differences = [
                distance - previous[index]
                for index, distance in beams
                if index in previous
            ]
            if differences:
                raw_scan_rms = math.sqrt(
                    sum(value * value for value in differences) / len(differences)
                )
        if self.previous_points is not None:
            delta, converged, quality = icp_2d(self.previous_points, scan_points)
            if raw_scan_rms is not None and raw_scan_rms < RAW_SCAN_STATIC_RMS:
                converged = False
            if converged and (
                math.hypot(delta[0], delta[1]) > MAX_FRAME_TRANSLATION
                or abs(delta[2]) > MAX_FRAME_ROTATION
            ):
                converged = False
            if converged and (
                math.hypot(delta[0], delta[1]) < MIN_MOTION_TRANSLATION
                and abs(delta[2]) < MIN_MOTION_ROTATION
            ):
                converged = False
            motion_size = math.hypot(delta[0], delta[1]) + abs(delta[2])
            candidate_size = math.hypot(
                self.motion_candidate[0], self.motion_candidate[1]
            ) + abs(self.motion_candidate[2])
            if converged and motion_size > 0.0 and candidate_size > 0.0:
                same_direction = (
                    delta[0] * self.motion_candidate[0]
                    + delta[1] * self.motion_candidate[1]
                    + delta[2] * self.motion_candidate[2]
                ) > 0.0
                self.motion_confirmations = (
                    self.motion_confirmations + 1 if same_direction else 1
                )
            else:
                self.motion_confirmations = 0
            self.motion_candidate = delta
            if self.motion_confirmations < MOTION_CONFIRM_FRAMES:
                converged = False
            if converged:
                if math.hypot(delta[0], delta[1]) < MOTION_DEADBAND and abs(delta[2]) < MOTION_DEADBAND:
                    delta = (0.0, 0.0, 0.0)
                self.pose = compose(self.pose, inverse_pose(delta))
            else:
                delta = (0.0, 0.0, 0.0)
            self.odom_quality = {
                "inliers": quality["inliers"],
                "rms": quality["rms"],
                "zero_rms": quality["zero_rms"],
                "raw_scan_rms": raw_scan_rms,
                "converged": converged,
            }
        self.previous_points = scan_points
        self.previous_beams = beams

        stamp = msg.header.stamp
        dt = 0.0
        if self.last_stamp is not None:
            dt = (stamp.sec - self.last_stamp.sec) + (
                stamp.nanosec - self.last_stamp.nanosec
            ) / 1e9
        self.last_stamp = stamp

        with self.lock:
            self.last_scan_monotonic = received_at
            self.scan_times.append(received_at)
            if not self.radar_seen:
                self.radar_seen = True
                self._add_event_locked("雷达数据流已连接")
            self.scan = {
                "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
                "frame": msg.header.frame_id,
                "points": points,
            }
            self.odom = {
                "x": self.pose[0],
                "y": self.pose[1],
                "yaw": self.pose[2],
                "frame": "odom",
                **self.odom_quality,
            }
            if converged and delta != (0.0, 0.0, 0.0):
                self.path.append((self.pose[0], self.pose[1]))

        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "odom"
        odom.child_frame_id = msg.header.frame_id or "laser"
        odom.pose.pose.position.x = self.pose[0]
        odom.pose.pose.position.y = self.pose[1]
        odom.pose.pose.orientation.z = math.sin(self.pose[2] / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.pose[2] / 2.0)
        if dt > 0.0:
            velocity = inverse_pose(delta)
            odom.twist.twist.linear.x = velocity[0] / dt
            odom.twist.twist.linear.y = velocity[1] / dt
            odom.twist.twist.angular.z = velocity[2] / dt
        self.odom_publisher.publish(odom)


DATA = None
INDEX = Path(__file__).with_name("index.html").read_bytes()


class TerminalServer:
    """Minimal WebSocket-to-PTY bridge for the local operator dashboard."""

    def __init__(self, port, token, audit_log):
        self.port = port
        self.token = token
        self.audit_log = Path(audit_log)
        self._token_used = False
        self._token_lock = threading.Lock()

    def start(self):
        if not self.token:
            print("Browser terminal disabled: TERMINAL_TOKEN is not set", flush=True)
            return
        thread = threading.Thread(target=self._serve, daemon=True)
        thread.start()
        print(f"Browser terminal listening on ws://0.0.0.0:{self.port}", flush=True)

    def _audit(self, event, peer, **extra):
        record = {"time": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "event": event, "peer": peer, **extra}
        try:
            self.audit_log.parent.mkdir(parents=True, exist_ok=True)
            with self.audit_log.open("a", encoding="utf-8") as log:
                log.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError:
            pass

    def _serve(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("0.0.0.0", self.port))
            listener.listen(8)
            while True:
                client, address = listener.accept()
                threading.Thread(
                    target=self._handle_client, args=(client, address[0]), daemon=True
                ).start()

    @staticmethod
    def _read_http_request(client):
        data = b""
        while b"\r\n\r\n" not in data and len(data) < 16384:
            chunk = client.recv(4096)
            if not chunk:
                return None, {}
            data += chunk
        try:
            lines = data.decode("iso-8859-1").split("\r\n")
            headers = {}
            for line in lines[1:]:
                if ":" in line:
                    key, value = line.split(":", 1)
                    headers[key.strip().lower()] = value.strip()
            return lines[0], headers
        except UnicodeDecodeError:
            return None, {}

    @staticmethod
    def _send_http_error(client, status):
        body = b"terminal authorization failed\n"
        client.sendall(
            f"HTTP/1.1 {status}\r\nContent-Type: text/plain\r\nContent-Length: {len(body)}\r\nConnection: close\r\n\r\n".encode()
            + body
        )

    @staticmethod
    def _recv_exact(client, length):
        data = b""
        while len(data) < length:
            chunk = client.recv(length - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    @classmethod
    def _read_frame(cls, client):
        header = cls._recv_exact(client, 2)
        if header is None:
            return None, b""
        opcode = header[0] & 0x0F
        masked = header[1] & 0x80
        length = header[1] & 0x7F
        if length == 126:
            encoded = cls._recv_exact(client, 2)
            if encoded is None:
                return None, b""
            length = int.from_bytes(encoded, "big")
        elif length == 127:
            encoded = cls._recv_exact(client, 8)
            if encoded is None:
                return None, b""
            length = int.from_bytes(encoded, "big")
        if length > 1024 * 1024:
            return None, b""
        mask = cls._recv_exact(client, 4) if masked else None
        payload = cls._recv_exact(client, length)
        if payload is None:
            return None, b""
        if mask:
            payload = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
        return opcode, payload

    @staticmethod
    def _send_frame(client, opcode, payload=b""):
        if isinstance(payload, str):
            payload = payload.encode("utf-8")
        length = len(payload)
        if length < 126:
            header = bytes([0x80 | opcode, length])
        elif length < 65536:
            header = bytes([0x80 | opcode, 126]) + length.to_bytes(2, "big")
        else:
            header = bytes([0x80 | opcode, 127]) + length.to_bytes(8, "big")
        client.sendall(header + payload)

    def _authorize(self, request, headers):
        if not request or not request.startswith("GET "):
            return False
        if headers.get("upgrade", "").lower() != "websocket" or not headers.get("sec-websocket-key"):
            return False
        try:
            target = request.split()[1]
            candidate = parse_qs(urlparse(target).query).get("token", [""])[0]
        except (IndexError, ValueError):
            return False
        with self._token_lock:
            if self._token_used or not hmac.compare_digest(candidate, self.token):
                return False
            self._token_used = True
        return True

    def _handle_client(self, client, peer):
        with client:
            request, headers = self._read_http_request(client)
            if not self._authorize(request, headers):
                self._audit("rejected", peer)
                self._send_http_error(client, "401 Unauthorized")
                return
            accept = base64.b64encode(
                hashlib.sha1((headers["sec-websocket-key"] + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()
            ).decode()
            client.sendall(
                ("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                 f"Sec-WebSocket-Accept: {accept}\r\n\r\n").encode()
            )
            self._run_shell(client, peer)

    def _run_shell(self, client, peer):
        started = time.monotonic()
        pid, terminal_fd = pty.fork()
        if pid == 0:
            env = os.environ.copy()
            env.update({"TERM": "xterm-256color", "COLORTERM": "truecolor", "LANG": "C.UTF-8"})
            os.execvpe("/bin/bash", ["bash", "--login"], env)
        self._audit("session_started", peer, pid=pid)
        client.setblocking(False)
        try:
            while True:
                readable, _, _ = select.select([client, terminal_fd], [], [], 0.5)
                if terminal_fd in readable:
                    try:
                        output = os.read(terminal_fd, 8192)
                    except OSError:
                        break
                    if not output:
                        break
                    self._send_frame(client, 1, output.decode("utf-8", errors="replace"))
                if client in readable:
                    opcode, payload = self._read_frame(client)
                    if opcode is None or opcode == 8:
                        break
                    if opcode == 9:
                        self._send_frame(client, 10, payload)
                    elif opcode == 1:
                        os.write(terminal_fd, payload)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            try:
                os.close(terminal_fd)
            except OSError:
                pass
            try:
                os.kill(pid, 1)
            except ProcessLookupError:
                pass
            try:
                _, status = os.waitpid(pid, 0)
            except ChildProcessError:
                status = 0
            self._audit("session_ended", peer, pid=pid, seconds=round(time.monotonic() - started, 1), status=status)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(INDEX)
            return
        if self.path == "/api/state":
            payload = DATA.snapshot()
            body = json.dumps(payload, separators=(",", ":")).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/api/stream":
            self._serve_event_stream()
            return
        if self.path == "/api/info":
            body = json.dumps(
                {"transport": "sse", "interval_ms": 250, "version": 2}
            ).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/api/health":
            body = json.dumps({"ok": True, "radar": DATA.snapshot()["radar"]}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def _serve_event_stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache, no-transform")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        try:
            while True:
                payload = json.dumps(DATA.snapshot(), separators=(",", ":"))
                self.wfile.write(f"event: state\ndata: {payload}\n\n".encode())
                self.wfile.flush()
                time.sleep(0.25)
        except (BrokenPipeError, ConnectionResetError):
            return

    def do_POST(self):
        if self.path == "/api/reset-odom":
            DATA.reset_odometry()
            body = b'{"ok":true}'
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def log_message(self, fmt, *args):
        return


def main():
    global DATA
    rclpy.init()
    DATA = LatestData()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(DATA)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    port = int(os.environ.get("WEB_VIEWER_PORT", "8080"))
    terminal = TerminalServer(
        int(os.environ.get("TERMINAL_PORT", "8765")),
        os.environ.get("TERMINAL_TOKEN", ""),
        os.environ.get("TERMINAL_AUDIT_LOG", "/tmp/lidar-terminal-audit.jsonl"),
    )
    terminal.start()
    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print(f"LiDAR web viewer listening on http://0.0.0.0:{port}", flush=True)
    try:
        server.serve_forever()
    finally:
        server.shutdown()
        executor.shutdown()
        DATA.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
