#!/usr/bin/env python3
import base64
import hashlib
import hmac
import io
import json
import math
import os
import pty
import select
import socket
import struct
import threading
import time
import zlib
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rclpy
from PIL import Image as PILImage
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from semantic_map import SemanticMapStore

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
SERVICE_CONTROL_SOCKET = os.environ.get(
    "ROBOT_SERVICE_CONTROL_SOCKET", "/run/robot-service-control/control.sock"
)


def service_control_request(payload):
    encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"
    if len(encoded) > 8192:
        raise ValueError("service control request is too large")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(6.0)
        client.connect(SERVICE_CONTROL_SOCKET)
        client.sendall(encoded)
        response = bytearray()
        while len(response) <= 65536:
            chunk = client.recv(4096)
            if not chunk:
                break
            response.extend(chunk)
            if b"\n" in chunk:
                break
    if not response or len(response) > 65536:
        raise OSError("invalid response from service control helper")
    result = json.loads(bytes(response).split(b"\n", 1)[0].decode("utf-8"))
    if not isinstance(result, dict):
        raise ValueError("invalid service control response")
    return result


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


def read_acoustic_state(path, maximum_age=2.0):
    """Read a JSON state file produced by the I2C reader without blocking ROS."""
    try:
        with open(path, encoding="utf-8") as state_file:
            state = json.load(state_file)
        timestamp = float(state.get("timestamp", 0.0))
        state["age_ms"] = round(max(0.0, time.time() - timestamp) * 1000)
        state["online"] = timestamp > 0.0 and state["age_ms"] < maximum_age * 1000
        return state
    except (OSError, ValueError, TypeError):
        return {"online": False}


class HardwareMonitor:
    """Cached adapters for the host gimbal, serial screen, and RPMsg IMU."""

    GIMBAL_STATES = {
        0: "disabled",
        1: "starting",
        2: "homing",
        3: "active",
        4: "returning",
        5: "stopping",
        6: "fault",
    }
    GIMBAL_INSET_DEG = 5.0
    RPMSG_MAGIC = 0xA5
    RPMSG_IMU_TELEMETRY = 42

    def __init__(self):
        self.gimbal_socket = os.environ.get(
            "GIMBAL_SOCKET", "/run/gimbal-daemon/gimbal.sock"
        )
        self.screen_socket = os.environ.get(
            "SCREEN_SOCKET", "/run/wifi-screen/face.sock"
        )
        self.rpmsg_socket = os.environ.get(
            "RPMSG_BROKER_SOCKET", "/run/rpmsg-broker/rpmsg.sock"
        )
        self.lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.last_update = 0.0
        self.last_snapshot = {
            "gimbal": {"online": False},
            "screen": {"online": False},
            "imu": {"online": False},
        }

    @staticmethod
    def _json_stream_request(path, payload, timeout=0.5):
        encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(timeout)
            connection.connect(path)
            connection.sendall(encoded)
            received = bytearray()
            while b"\n" not in received and len(received) <= 65536:
                chunk = connection.recv(4096)
                if not chunk:
                    break
                received.extend(chunk)
        if len(received) > 65536:
            raise ValueError("hardware response is too large")
        response = json.loads(bytes(received).split(b"\n", 1)[0].decode("utf-8"))
        if not isinstance(response, dict):
            raise ValueError("hardware response is not an object")
        return response

    def _gimbal_status(self):
        try:
            response = self._json_stream_request(
                self.gimbal_socket, {"command": "status"}, 0.6
            )
            telemetry = response.get("telemetry") or {}
            state = int(telemetry.get("state", -1))
            limits_valid = int(telemetry.get("limits_valid_mask", 0)) == 0x0F
            limits = None
            if limits_valid:
                limits = {
                    "yaw_min_deg": round(
                        float(telemetry["yaw_min_deg"]) + self.GIMBAL_INSET_DEG, 2
                    ),
                    "yaw_max_deg": round(
                        float(telemetry["yaw_max_deg"]) - self.GIMBAL_INSET_DEG, 2
                    ),
                    "pitch_min_deg": round(
                        float(telemetry["pitch_min_deg"]) + self.GIMBAL_INSET_DEG, 2
                    ),
                    "pitch_max_deg": round(
                        float(telemetry["pitch_max_deg"]) - self.GIMBAL_INSET_DEG, 2
                    ),
                    "inset_deg": self.GIMBAL_INSET_DEG,
                }
                limits_valid = (
                    limits["yaw_min_deg"] < limits["yaw_max_deg"]
                    and limits["pitch_min_deg"] < limits["pitch_max_deg"]
                )
                if not limits_valid:
                    limits = None
            ready = (
                response.get("ok") is True
                and state == 3
                and int(telemetry.get("fault", -1)) == 0
                and limits_valid
                and int(telemetry.get("feedback_valid_mask", 0)) == 0x03
                and max(
                    int(telemetry.get("yaw_feedback_age_ms", 999999)),
                    int(telemetry.get("pitch_feedback_age_ms", 999999)),
                ) < 500
            )
            return {
                "online": True,
                "ready": ready,
                "state_name": self.GIMBAL_STATES.get(state, f"state-{state}"),
                "telemetry": telemetry,
                "web_limits": limits,
            }
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            return {"online": False, "error": str(error)}

    def _screen_status(self):
        try:
            response = self._json_stream_request(
                self.screen_socket, {"version": 1, "action": "status"}, 0.6
            )
            status = response.get("status") or {}
            return {
                "online": response.get("ok") is True,
                "expression": status.get("current"),
                "active_source": status.get("active_source"),
            }
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            return {"online": False, "error": str(error)}

    @staticmethod
    def _decode_imu(payload):
        if len(payload) < 52 or payload[0] != 1:
            raise ValueError("invalid IMU telemetry payload")
        i32 = lambda offset: struct.unpack_from(">i", payload, offset)[0]
        u32 = lambda offset: struct.unpack_from(">I", payload, offset)[0]
        radians_to_degrees = 180.0 / math.pi
        return {
            "status": int(payload[1]),
            "valid": bool(payload[2]),
            "calibrated": bool(payload[3]),
            "roll_deg": round(i32(4) / 1_000_000.0 * radians_to_degrees, 3),
            "roll_rate_rad_s": round(i32(8) / 1_000_000.0, 4),
            "pitch_deg": round(i32(12) / 1_000_000.0 * radians_to_degrees, 3),
            "pitch_rate_rad_s": round(i32(16) / 1_000_000.0, 4),
            "accel_m_s2": [round(i32(offset) / 1_000_000.0, 4) for offset in (20, 24, 28)],
            "gyro_rad_s": [round(i32(offset) / 1_000_000.0, 4) for offset in (32, 36, 40)],
            "samples": u32(44),
            "age_ms": u32(48),
        }

    def _imu_status(self):
        header = bytes((self.RPMSG_MAGIC, self.RPMSG_IMU_TELEMETRY, 1, 0))
        request = header + bytes(((-sum(header)) & 0xFF,))
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as connection:
                connection.settimeout(0.6)
                connection.connect(self.rpmsg_socket)
                connection.sendall(request)
                response = connection.recv(128)
            if (
                len(response) < 5
                or response[0] != self.RPMSG_MAGIC
                or response[1] != self.RPMSG_IMU_TELEMETRY
                or len(response) != response[3] + 5
                or sum(response) & 0xFF
            ):
                raise ValueError("invalid RPMsg IMU response")
            telemetry = self._decode_imu(response[4:-1])
            telemetry["online"] = (
                telemetry["status"] == 0
                and telemetry["valid"]
                and telemetry["age_ms"] < 500
            )
            return telemetry
        except (OSError, ValueError, struct.error) as error:
            return {"online": False, "error": str(error)}

    def collect(self, force=False):
        with self.lock:
            now = time.monotonic()
            if not force and now - self.last_update < 1.0:
                return dict(self.last_snapshot)
            if self.command_lock.locked():
                gimbal = dict(self.last_snapshot.get("gimbal") or {"online": True})
                gimbal["command_pending"] = True
            else:
                gimbal = self._gimbal_status()
            self.last_snapshot = {
                "gimbal": gimbal,
                "screen": self._screen_status(),
                "imu": self._imu_status(),
            }
            self.last_update = now
            return dict(self.last_snapshot)

    @staticmethod
    def risk_summary(hardware):
        """Aggregate Linux-observable hazards without replacing from-core safety."""
        risks = []
        gimbal = hardware.get("gimbal") or {}
        imu = hardware.get("imu") or {}
        screen = hardware.get("screen") or {}
        if not gimbal.get("online"):
            risks.append({"code": "GIMBAL_OFFLINE", "level": "fault", "action": "禁止云台运动"})
        elif not gimbal.get("ready"):
            telemetry = gimbal.get("telemetry") or {}
            if int(telemetry.get("fault", 0)):
                risks.append({"code": "GIMBAL_FAULT", "level": "fault", "action": "立即急停并检查云台"})
            else:
                risks.append({"code": "GIMBAL_NOT_READY", "level": "warning", "action": "完成限位、反馈和启用检查"})
        if not imu.get("online"):
            risks.append({"code": "IMU_OFFLINE", "level": "fault", "action": "禁止进入实时运动控制"})
        elif int(imu.get("age_ms", 0)) >= 300:
            risks.append({"code": "IMU_STALE", "level": "warning", "action": "检查 RPMsg 和从核遥测"})
        if not screen.get("online"):
            risks.append({"code": "SCREEN_OFFLINE", "level": "warning", "action": "检查串口屏服务和 UART"})
        highest = "ok"
        if any(item["level"] == "fault" for item in risks):
            highest = "fault"
        elif risks:
            highest = "warning"
        return {
            "level": highest,
            "safe_to_command": highest == "ok" and bool(gimbal.get("ready")),
            "risks": risks,
        }

    def command_gimbal(self, request):
        command = str(request.get("command", "")).lower()
        if command not in {"set", "center", "enable", "disable", "estop"}:
            raise ValueError("unsupported gimbal command")
        if command != "estop" and request.get("confirm") is not True:
            raise ValueError("gimbal command requires confirm=true")
        payload = {"command": command}
        if command in {"enable", "disable"}:
            payload["confirm"] = True
        with self.command_lock:
            if command == "set":
                yaw = float(request.get("yaw_deg"))
                pitch = float(request.get("pitch_deg"))
                if not math.isfinite(yaw) or not math.isfinite(pitch):
                    raise ValueError("gimbal target must be finite")
                status = self._gimbal_status()
                if not status.get("online"):
                    raise ValueError("gimbal status unavailable")
                limits = status.get("web_limits")
                if not status.get("ready") or not limits:
                    raise ValueError("gimbal is not ready or limits are unavailable")
                if not (
                    limits["yaw_min_deg"] <= yaw <= limits["yaw_max_deg"]
                    and limits["pitch_min_deg"] <= pitch <= limits["pitch_max_deg"]
                ):
                    raise ValueError(
                        "target exceeds web safety limits "
                        f"yaw=[{limits['yaw_min_deg']}, {limits['yaw_max_deg']}] "
                        f"pitch=[{limits['pitch_min_deg']}, {limits['pitch_max_deg']}]"
                    )
                payload.update(yaw_deg=yaw, pitch_deg=pitch)
            response = self._json_stream_request(self.gimbal_socket, payload, 17.0)
            self.last_update = 0.0
        return response


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
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.lock = threading.Lock()
        self.scan = None
        self.odom = None
        self.previous_points = None
        self.previous_beams = None
        self.pose = (0.0, 0.0, 0.0)
        self.last_stamp = None
        self.velocity = (0.0, 0.0, 0.0)
        self.odom_quality = {"inliers": 0, "rms": None, "converged": False}
        self.motion_candidate = (0.0, 0.0, 0.0)
        self.motion_confirmations = 0
        self.system_monitor = SystemMonitor()
        self.hardware_monitor = HardwareMonitor()
        self.camera_topics = {
            "rgb": os.environ.get("CAMERA_RGB_TOPIC", "/camera/color/image_raw"),
            "depth": os.environ.get("CAMERA_DEPTH_TOPIC", "/camera/depth/image_raw"),
            "rgb_info": os.environ.get(
                "CAMERA_RGB_INFO_TOPIC", "/camera/color/camera_info"
            ),
            "depth_info": os.environ.get(
                "CAMERA_DEPTH_INFO_TOPIC", "/camera/depth/camera_info"
            ),
            "aligned_depth": os.environ.get(
                "CAMERA_ALIGNED_DEPTH_TOPIC",
                "/camera/aligned_depth_to_color/image_raw",
            ),
            "aligned_depth_info": os.environ.get(
                "CAMERA_ALIGNED_DEPTH_INFO_TOPIC",
                "/camera/aligned_depth_to_color/camera_info",
            ),
        }
        self.camera_frames = {"rgb": None, "depth": None, "aligned_depth": None}
        self.camera_sequences = {"rgb": 0, "depth": 0, "aligned_depth": 0}
        self.camera_times = {
            "rgb": deque(maxlen=40),
            "depth": deque(maxlen=40),
            "aligned_depth": deque(maxlen=40),
        }
        self.camera_seen = {"rgb": False, "depth": False, "aligned_depth": False}
        self.aligned_camera_info = None
        self.targets_3d = None
        self.semantic_map = SemanticMapStore(
            os.environ.get(
                "SEMANTIC_MAP_PATH", "/var/lib/semantic-map/semantic-map.json"
            ),
            association_radius_m=float(
                os.environ.get("SEMANTIC_ASSOCIATION_RADIUS_M", "0.8")
            ),
            movement_threshold_m=float(
                os.environ.get("SEMANTIC_MOVEMENT_THRESHOLD_M", "0.35")
            ),
        )
        self.camera_condition = threading.Condition(self.lock)
        self.rgb_jpeg_lock = threading.Lock()
        self.rgb_jpeg_cache = {}
        self.rgb_jpeg_cache_sequence = -1
        self.rgb_jpeg_quality = max(
            40, min(95, int(os.environ.get("CAMERA_JPEG_QUALITY", "80")))
        )
        self.scan_times = deque(maxlen=80)
        self.last_scan_monotonic = None
        self.path = deque([(0.0, 0.0)], maxlen=400)
        self.map_frame = None
        self.map_sequence = 0
        self.radar_pose = None
        self.map_frame_id = os.environ.get("MAP_FRAME", "map")
        self.radar_frame_id = os.environ.get("RADAR_FRAME", "laser")
        self.world_frame_ids = {
            "base_link": os.environ.get("BASE_FRAME", "base_link"),
            "laser": self.radar_frame_id,
            "gimbal_yaw": os.environ.get("GIMBAL_YAW_FRAME", "gimbal_yaw_link"),
            "gimbal_pitch": os.environ.get("GIMBAL_PITCH_FRAME", "gimbal_pitch_link"),
            "camera": os.environ.get(
                "CAMERA_WORLD_FRAME", "camera_color_optical_frame"
            ),
        }
        self.world_poses = {}
        self.events = deque(maxlen=12)
        self.radar_seen = False
        self._add_event_locked("网页仪表盘已启动")
        self.odom_publisher = self.create_publisher(Odometry, "/scan_odom", 10)
        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.on_scan, scan_qos
        )
        self.rgb_subscription = self.create_subscription(
            Image, self.camera_topics["rgb"], self.on_rgb_image, camera_qos
        )
        self.depth_subscription = self.create_subscription(
            Image, self.camera_topics["depth"], self.on_depth_image, camera_qos
        )
        self.aligned_depth_subscription = self.create_subscription(
            Image,
            self.camera_topics["aligned_depth"],
            self.on_aligned_depth_image,
            camera_qos,
        )
        self.aligned_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_topics["aligned_depth_info"],
            self.on_aligned_camera_info,
            camera_qos,
        )
        self.targets_subscription = self.create_subscription(
            String, "/vision/targets_3d", self.on_targets_3d, 10
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.on_map, map_qos
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_timer = self.create_timer(0.1, self.update_world_poses)

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
            self.velocity = (0.0, 0.0, 0.0)
            self._add_event_locked("里程计原点已重置")

    def add_event(self, message):
        with self.lock:
            self._add_event_locked(message)

    def snapshot(self, compact=False):
        hardware = self.hardware_monitor.collect()
        hardware["risk"] = self.hardware_monitor.risk_summary(hardware)
        semantic_map = self.semantic_map.snapshot()
        if compact:
            semantic_map["changes"] = semantic_map["changes"][:12]
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
            rgb = self.camera_frames["rgb"]
            depth = self.camera_frames["depth"]
            aligned_depth = self.camera_frames["aligned_depth"]
            rgb_age = None if rgb is None else now - rgb["received_at"]
            depth_age = None if depth is None else now - depth["received_at"]
            aligned_depth_age = (
                None
                if aligned_depth is None
                else now - aligned_depth["received_at"]
            )
            rgb_online = rgb_age is not None and rgb_age < 2.0
            depth_online = depth_age is not None and depth_age < 2.0
            aligned_depth_online = (
                aligned_depth_age is not None and aligned_depth_age < 2.0
            )
            camera = {
                "online": rgb_online or depth_online or aligned_depth_online,
                "rgb_online": rgb_online,
                "depth_online": depth_online,
                "aligned_depth_online": aligned_depth_online,
                "rgb_hz": self._camera_hz_locked("rgb"),
                "depth_hz": self._camera_hz_locked("depth"),
                "aligned_depth_hz": self._camera_hz_locked("aligned_depth"),
                "rgb_resolution": None if rgb is None else f'{rgb["width"]}x{rgb["height"]}',
                "depth_resolution": None if depth is None else f'{depth["width"]}x{depth["height"]}',
                "aligned_depth_resolution": None if aligned_depth is None else f'{aligned_depth["width"]}x{aligned_depth["height"]}',
                "rgb_encoding": None if rgb is None else rgb["encoding"],
                "depth_encoding": None if depth is None else depth["encoding"],
                "aligned_depth_encoding": None if aligned_depth is None else aligned_depth["encoding"],
                "rgb_frame": None if rgb is None else rgb["frame_id"],
                "depth_frame": None if depth is None else depth["frame_id"],
                "aligned_depth_frame": None if aligned_depth is None else aligned_depth["frame_id"],
                "rgb_age_ms": None if rgb_age is None else round(rgb_age * 1000),
                "depth_age_ms": None if depth_age is None else round(depth_age * 1000),
                "aligned_depth_age_ms": None if aligned_depth_age is None else round(aligned_depth_age * 1000),
                "aligned": aligned_depth_online and bool(self.aligned_camera_info),
                "aligned_camera_info": self.aligned_camera_info,
                "topics": dict(self.camera_topics),
                "integration_state": "streaming" if rgb_online or depth_online or aligned_depth_online else "waiting_for_camera",
            }
            targets_3d = self.targets_3d
            if targets_3d is not None:
                targets_3d = dict(targets_3d)
                targets_age = now - targets_3d.pop("received_at")
                targets_3d["age_ms"] = round(targets_age * 1000)
                targets_3d["online"] = targets_age < 2.0
            current_map = self.map_frame
            map_age = None if current_map is None else now - current_map["received_at"]
            current_pose = self.radar_pose
            pose_age = None if current_pose is None else now - current_pose["received_at"]
            radar_pose = None
            if current_pose is not None:
                radar_pose = {
                    "x": current_pose["x"],
                    "y": current_pose["y"],
                    "yaw": current_pose["yaw"],
                    "frame": current_pose["frame"],
                    "child_frame": current_pose["child_frame"],
                    "age_ms": round(pose_age * 1000),
                    "online": pose_age < 1.0,
                }
            world_poses = {}
            for name, pose in self.world_poses.items():
                world_age = now - pose["received_at"]
                world_poses[name] = {
                    "frame": pose["frame"],
                    "child_frame": pose["child_frame"],
                    "position": dict(pose["position"]),
                    "quaternion": dict(pose["quaternion"]),
                    "age_ms": round(world_age * 1000),
                    "online": world_age < 1.0,
                }
            map_state = {
                "online": map_age is not None and map_age < 5.0,
                "age_ms": None if map_age is None else round(map_age * 1000),
                "sequence": 0 if current_map is None else current_map["sequence"],
                "frame": None if current_map is None else current_map["frame_id"],
                "resolution": None if current_map is None else current_map["resolution"],
                "width": None if current_map is None else current_map["width"],
                "height": None if current_map is None else current_map["height"],
                "radar_pose": radar_pose,
                "world_poses": world_poses,
            }
            scan = self.scan
            if compact and scan is not None:
                points = scan["points"]
                stride = max(1, math.ceil(len(points) / 180))
                scan = {
                    "stamp": scan["stamp"],
                    "frame": scan["frame"],
                    "points": [
                        [round(point["a"], 3), round(point["r"], 3)]
                        for point in points[::stride]
                    ],
                }
            acoustic = {
                "sound": read_acoustic_state("/run/acoustic-eye/angle.json"),
                "environment": read_acoustic_state("/run/acoustic-eye/aht20.json", 10.0),
            }
            return {
                "system": self.system_monitor.collect(),
                "radar": radar,
                "camera": camera,
                "scan": scan,
                "map": map_state,
                "targets_3d": targets_3d,
                "semantic_map": semantic_map,
                "acoustic": acoustic,
                "hardware": hardware,
                "events": list(self.events),
            }

    def _camera_hz_locked(self, kind):
        samples = self.camera_times[kind]
        if len(samples) < 2:
            return 0.0
        elapsed = samples[-1] - samples[0]
        return round((len(samples) - 1) / elapsed, 1) if elapsed > 0 else 0.0

    def _store_camera_image(self, kind, msg):
        channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
        bytes_per_pixel = channels or (2 if msg.encoding == "16UC1" else 4 if msg.encoding == "32FC1" else None)
        if bytes_per_pixel is None:
            self.get_logger().warning(
                f"unsupported {kind} encoding: {msg.encoding}", throttle_duration_sec=10.0
            )
            return
        row_bytes = msg.width * bytes_per_pixel
        source = memoryview(msg.data)
        if msg.step == row_bytes:
            payload = bytes(source[: row_bytes * msg.height])
        else:
            payload = b"".join(
                bytes(source[row * msg.step : row * msg.step + row_bytes])
                for row in range(msg.height)
            )
        received_at = time.monotonic()
        with self.camera_condition:
            self.camera_sequences[kind] += 1
            self.camera_frames[kind] = {
                "data": payload,
                "width": msg.width,
                "height": msg.height,
                "encoding": msg.encoding,
                "is_bigendian": bool(msg.is_bigendian),
                "frame_id": msg.header.frame_id,
                "received_at": received_at,
                "sequence": self.camera_sequences[kind],
            }
            self.camera_times[kind].append(received_at)
            if not self.camera_seen[kind]:
                self.camera_seen[kind] = True
                labels = {"rgb": "RGB", "depth": "深度", "aligned_depth": "对齐深度"}
                self._add_event_locked(f'{labels.get(kind, kind)}图像流已连接')
            self.camera_condition.notify_all()

    def on_rgb_image(self, msg):
        self._store_camera_image("rgb", msg)

    def on_depth_image(self, msg):
        self._store_camera_image("depth", msg)

    def on_aligned_depth_image(self, msg):
        self._store_camera_image("aligned_depth", msg)

    def on_aligned_camera_info(self, msg):
        info = {
            "width": msg.width,
            "height": msg.height,
            "frame": msg.header.frame_id,
            "fx": msg.k[0],
            "fy": msg.k[4],
            "cx": msg.k[2],
            "cy": msg.k[5],
        }
        with self.lock:
            self.aligned_camera_info = info

    def on_targets_3d(self, msg):
        try:
            targets = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        targets["received_at"] = time.monotonic()
        self.semantic_map.ingest(targets.get("targets") or [])
        with self.lock:
            self.targets_3d = targets

    def camera_frame(self, kind):
        with self.lock:
            frame = self.camera_frames.get(kind)
            return None if frame is None else dict(frame)

    def wait_for_camera_frame(self, kind, after, timeout=1.0):
        deadline = time.monotonic() + timeout
        with self.camera_condition:
            while True:
                frame = self.camera_frames.get(kind)
                if frame is not None and frame["sequence"] > after:
                    return dict(frame)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.camera_condition.wait(remaining)

    def rgb_jpeg_frame(self, requested_width=None, requested_quality=None):
        frame = self.camera_frame("rgb")
        if frame is None:
            return None
        width = frame["width"]
        if requested_width is not None:
            width = max(120, min(frame["width"], int(requested_width)))
        height = max(1, round(frame["height"] * width / frame["width"]))
        quality = self.rgb_jpeg_quality
        if requested_quality is not None:
            quality = max(30, min(85, int(requested_quality)))
        cache_key = (width, quality)
        with self.rgb_jpeg_lock:
            if self.rgb_jpeg_cache_sequence != frame["sequence"]:
                self.rgb_jpeg_cache.clear()
                self.rgb_jpeg_cache_sequence = frame["sequence"]
            if cache_key in self.rgb_jpeg_cache:
                return dict(self.rgb_jpeg_cache[cache_key])
            size = (frame["width"], frame["height"])
            encoding = frame["encoding"]
            if encoding == "rgb8":
                image = PILImage.frombytes("RGB", size, frame["data"])
            elif encoding == "bgr8":
                image = PILImage.frombytes("RGB", size, frame["data"], "raw", "BGR")
            elif encoding == "rgba8":
                image = PILImage.frombytes("RGBA", size, frame["data"]).convert("RGB")
            elif encoding == "bgra8":
                image = PILImage.frombytes("RGBA", size, frame["data"], "raw", "BGRA").convert(
                    "RGB"
                )
            else:
                return None
            if width != frame["width"]:
                image = image.resize((width, height), PILImage.BILINEAR)
            output = io.BytesIO()
            image.save(
                output,
                "JPEG",
                quality=quality,
                optimize=False,
                subsampling=2,
            )
            encoded = {
                "data": output.getvalue(),
                "width": width,
                "height": height,
                "sequence": frame["sequence"],
            }
            self.rgb_jpeg_cache[cache_key] = encoded
            return dict(encoded)

    def map_data(self):
        with self.lock:
            return None if self.map_frame is None else dict(self.map_frame)

    def on_map(self, msg):
        received_at = time.monotonic()
        payload = bytes(value & 0xFF for value in msg.data)
        orientation = msg.info.origin.orientation
        origin_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        with self.lock:
            self.map_sequence += 1
            self.map_frame = {
                "data": payload,
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "frame_id": msg.header.frame_id,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                "origin_yaw": origin_yaw,
                "received_at": received_at,
                "sequence": self.map_sequence,
            }
            if self.map_sequence == 1:
                self._add_event_locked("SLAM 地图数据流已连接")

    @staticmethod
    def _world_pose(transform, received_at):
        translation = transform.transform.translation
        orientation = transform.transform.rotation
        return {
            "frame": transform.header.frame_id,
            "child_frame": transform.child_frame_id,
            "position": {
                "x": translation.x,
                "y": translation.y,
                "z": translation.z,
            },
            "quaternion": {
                "x": orientation.x,
                "y": orientation.y,
                "z": orientation.z,
                "w": orientation.w,
            },
            "received_at": received_at,
        }

    @staticmethod
    def _rotate_world_vector(vector, quaternion):
        x, y, z = vector
        qx = quaternion["x"]
        qy = quaternion["y"]
        qz = quaternion["z"]
        qw = quaternion["w"]
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        return (
            x + qw * tx + qy * tz - qz * ty,
            y + qw * ty + qz * tx - qx * tz,
            z + qw * tz + qx * ty - qy * tx,
        )

    @staticmethod
    def _multiply_quaternions(left, right):
        lx, ly, lz, lw = (left[key] for key in ("x", "y", "z", "w"))
        rx, ry, rz, rw = (right[key] for key in ("x", "y", "z", "w"))
        return {
            "x": lw * rx + lx * rw + ly * rz - lz * ry,
            "y": lw * ry - lx * rz + ly * rw + lz * rx,
            "z": lw * rz + lx * ry - ly * rx + lz * rw,
            "w": lw * rw - lx * rx - ly * ry - lz * rz,
        }

    @classmethod
    def _compose_world_pose(cls, base_pose, relative_transform, received_at):
        translation = relative_transform.transform.translation
        orientation = relative_transform.transform.rotation
        rotated = cls._rotate_world_vector(
            (translation.x, translation.y, translation.z),
            base_pose["quaternion"],
        )
        base_position = base_pose["position"]
        relative_quaternion = {
            "x": orientation.x,
            "y": orientation.y,
            "z": orientation.z,
            "w": orientation.w,
        }
        return {
            "frame": base_pose["frame"],
            "child_frame": relative_transform.child_frame_id,
            "position": {
                "x": base_position["x"] + rotated[0],
                "y": base_position["y"] + rotated[1],
                "z": base_position["z"] + rotated[2],
            },
            "quaternion": cls._multiply_quaternions(
                base_pose["quaternion"], relative_quaternion
            ),
            "received_at": received_at,
        }

    def update_world_poses(self):
        received_at = time.monotonic()
        updates = {}
        for name, child_frame in self.world_frame_ids.items():
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame_id, child_frame, Time()
                )
            except TransformException:
                continue
            updates[name] = self._world_pose(transform, received_at)
        base_pose = updates.get("base_link")
        if base_pose is not None:
            for name in ("gimbal_yaw", "gimbal_pitch", "camera"):
                if name in updates:
                    continue
                try:
                    relative = self.tf_buffer.lookup_transform(
                        self.world_frame_ids["base_link"],
                        self.world_frame_ids[name],
                        Time(),
                    )
                except TransformException:
                    continue
                updates[name] = self._compose_world_pose(
                    base_pose, relative, received_at
                )
        if not updates:
            return

        radar = updates.get("laser")
        with self.lock:
            first_radar_pose = self.radar_pose is None and radar is not None
            self.world_poses.update(updates)
            if radar is not None:
                position = radar["position"]
                orientation = radar["quaternion"]
                yaw = math.atan2(
                    2.0
                    * (
                        orientation["w"] * orientation["z"]
                        + orientation["x"] * orientation["y"]
                    ),
                    1.0
                    - 2.0
                    * (
                        orientation["y"] * orientation["y"]
                        + orientation["z"] * orientation["z"]
                    ),
                )
                self.radar_pose = {
                    "x": position["x"],
                    "y": position["y"],
                    "yaw": yaw,
                    "frame": radar["frame"],
                    "child_frame": radar["child_frame"],
                    "received_at": received_at,
                }
            if first_radar_pose:
                self._add_event_locked("雷达与机器人世界坐标 TF 已连接")

    def on_scan(self, msg):
        self.get_logger().info("received scan", throttle_duration_sec=5.0)
        received_at = time.monotonic()
        points = []
        angle = msg.angle_min
        for distance in msg.ranges:
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                points.append({"a": angle, "r": distance})
            angle += msg.angle_increment

        with self.lock:
            self.last_scan_monotonic = received_at
            self.scan_times.append(received_at)
            if not self.radar_seen:
                self.radar_seen = True
                self._add_event_locked("雷达数据流已连接")
            self.scan = {
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                "frame": msg.header.frame_id,
                "points": points,
            }


DATA = None
INDEX = Path(__file__).with_name("index.html").read_bytes()
THREE_JS = Path(__file__).with_name("three.min.js").read_bytes()
INDEX_DEFLATE = zlib.compress(INDEX, 9)
THREE_JS_DEFLATE = zlib.compress(THREE_JS, 9)


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
    def _send_json_response(self, payload, status=200):
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode(
            "utf-8"
        )
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        if self.path in ("/", "/index.html"):
            self._send_compressible(
                INDEX,
                "text/html; charset=utf-8",
                (("Cache-Control", "no-store"),),
                INDEX_DEFLATE,
            )
            return
        if self.path == "/three.min.js":
            self._send_compressible(
                THREE_JS,
                "text/javascript; charset=utf-8",
                (("Cache-Control", "public, max-age=604800, immutable"),),
                THREE_JS_DEFLATE,
            )
            return
        if self.path == "/api/state":
            payload = DATA.snapshot()
            body = json.dumps(payload, separators=(",", ":")).encode()
            self._send_compressible(body, "application/json", (("Cache-Control", "no-store"),))
            return
        if self.path == "/api/stream":
            self._serve_event_stream()
            return
        if self.path == "/api/semantic-map":
            self._send_json_response(DATA.semantic_map.snapshot())
            return
        if self.path == "/api/gimbal/status":
            self._send_json_response(DATA.hardware_monitor.collect(force=True)["gimbal"])
            return
        if self.path == "/api/services":
            try:
                response = service_control_request({"action": "status"})
                self._send_json_response(response, 200 if response.get("ok") else 503)
            except (OSError, ValueError, json.JSONDecodeError) as error:
                self._send_json_response({"ok": False, "error": str(error)}, 503)
            return
        if parsed.path == "/api/camera/rgb/frame":
            self._serve_camera_frame("rgb", parsed.query)
            return
        if parsed.path == "/api/camera/rgb.mjpg":
            self._serve_rgb_mjpeg()
            return
        if parsed.path == "/api/camera/rgb.jpg":
            self._serve_rgb_jpeg(parsed.query)
            return
        if parsed.path == "/api/camera/depth/frame":
            self._serve_camera_frame("depth", parsed.query)
            return
        if parsed.path == "/api/camera/aligned_depth/frame":
            self._serve_camera_frame("aligned_depth", parsed.query)
            return
        if parsed.path == "/api/map/frame":
            self._serve_map_frame(parsed.query)
            return
        if self.path == "/api/info":
            body = json.dumps(
                {"transport": "sse+gzip", "interval_ms": 100, "version": 3}
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

    def _serve_camera_frame(self, kind, query):
        frame = DATA.camera_frame(kind)
        if frame is None:
            self.send_error(404, "camera frame unavailable")
            return
        try:
            after = int(parse_qs(query).get("after", ["-1"])[0])
        except ValueError:
            after = -1
        if frame["sequence"] <= after:
            self.send_response(204)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            return
        header = json.dumps(
            {
                "width": frame["width"],
                "height": frame["height"],
                "encoding": frame["encoding"],
                "is_bigendian": frame["is_bigendian"],
                "frame_id": frame["frame_id"],
                "sequence": frame["sequence"],
            },
            separators=(",", ":"),
        ).encode()
        body = len(header).to_bytes(4, "big") + header + frame["data"]
        self._send_compressible(
            body,
            "application/octet-stream",
            (("Cache-Control", "no-store"), ("X-Camera-Encoding", frame["encoding"])),
        )

    def _serve_rgb_mjpeg(self):
        boundary = b"frame"
        self.send_response(200)
        self.send_header(
            "Content-Type", "multipart/x-mixed-replace; boundary=frame"
        )
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        last_sequence = -1
        try:
            while True:
                frame = DATA.rgb_jpeg_frame()
                if frame is None or frame["sequence"] == last_sequence:
                    time.sleep(0.01)
                    continue
                jpeg = frame["data"]
                part_header = (
                    b"--"
                    + boundary
                    + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode("ascii")
                    + b"\r\nX-Frame-Sequence: "
                    + str(frame["sequence"]).encode("ascii")
                    + b"\r\n\r\n"
                )
                self.wfile.write(part_header)
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                last_sequence = frame["sequence"]
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def _serve_rgb_jpeg(self, query):
        parameters = parse_qs(query)
        try:
            after = int(parameters.get("after", ["-1"])[0])
        except ValueError:
            after = -1
        try:
            width = int(parameters.get("width", ["320"])[0])
            quality = int(parameters.get("quality", [str(DATA.rgb_jpeg_quality)])[0])
        except ValueError:
            width, quality = 320, DATA.rgb_jpeg_quality
        if DATA.wait_for_camera_frame("rgb", after) is None:
            self.send_response(204)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            return
        frame = DATA.rgb_jpeg_frame(width, quality)
        if frame is None:
            self.send_error(404, "RGB frame unavailable")
            return
        jpeg = frame["data"]
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("X-Frame-Sequence", str(frame["sequence"]))
        self.send_header("Content-Length", str(len(jpeg)))
        self.end_headers()
        self.wfile.write(jpeg)

    def _serve_map_frame(self, query):
        frame = DATA.map_data()
        if frame is None:
            self.send_error(404, "map frame unavailable")
            return
        try:
            after = int(parse_qs(query).get("after", ["-1"])[0])
        except ValueError:
            after = -1
        if frame["sequence"] <= after:
            self.send_response(204)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            return
        header = json.dumps(
            {
                "width": frame["width"],
                "height": frame["height"],
                "resolution": frame["resolution"],
                "frame_id": frame["frame_id"],
                "origin_x": frame["origin_x"],
                "origin_y": frame["origin_y"],
                "origin_yaw": frame["origin_yaw"],
                "sequence": frame["sequence"],
            },
            separators=(",", ":"),
        ).encode()
        body = len(header).to_bytes(4, "big") + header + frame["data"]
        self._send_compressible(
            body, "application/octet-stream", (("Cache-Control", "no-store"),)
        )

    def _send_compressible(self, body, content_type, headers=(), deflated_body=None):
        accepts = self.headers.get("Accept-Encoding", "").lower()
        use_deflate = "deflate" in accepts and len(body) >= 1024
        wire_body = (deflated_body or zlib.compress(body, 1)) if use_deflate else body
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        for name, value in headers:
            self.send_header(name, value)
        if use_deflate:
            self.send_header("Content-Encoding", "deflate")
            self.send_header("Vary", "Accept-Encoding")
        self.send_header("Content-Length", str(len(wire_body)))
        self.end_headers()
        self.wfile.write(wire_body)

    def _serve_event_stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache, no-transform")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        try:
            while True:
                payload = json.dumps(DATA.snapshot(compact=True), separators=(",", ":"))
                event = f"event: state\ndata: {payload}\n\n".encode()
                self.wfile.write(event)
                self.wfile.flush()
                time.sleep(0.20)
        except (BrokenPipeError, ConnectionResetError):
            return

    def do_POST(self):
        if self.path in ("/api/semantic-map/remove", "/api/semantic-map/clear"):
            try:
                content_length = int(self.headers.get("Content-Length", "0"))
                if content_length <= 0 or content_length > 4096:
                    raise ValueError("invalid request length")
                request = json.loads(self.rfile.read(content_length).decode("utf-8"))
                if not isinstance(request, dict) or request.get("confirm") is not True:
                    raise ValueError("explicit confirmation is required")
                if self.path.endswith("/remove"):
                    object_id = str(request.get("object_id", ""))
                    removed = DATA.semantic_map.remove(object_id)
                    if removed:
                        DATA.add_event(f"语义地标已删除：{object_id}")
                    self._send_json_response(
                        {"ok": removed, "object_id": object_id}, 200 if removed else 404
                    )
                else:
                    count = DATA.semantic_map.clear()
                    DATA.add_event(f"语义地图已清空：{count} 个地标")
                    self._send_json_response({"ok": True, "removed": count})
            except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
                self._send_json_response({"ok": False, "error": str(error)}, 400)
            return
        if self.path == "/api/reset-odom":
            DATA.reset_odometry()
            body = b'{"ok":true}'
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/api/gimbal/command":
            try:
                content_length = int(self.headers.get("Content-Length", "0"))
                if content_length <= 0 or content_length > 4096:
                    raise ValueError("invalid request length")
                request = json.loads(self.rfile.read(content_length).decode("utf-8"))
                if not isinstance(request, dict):
                    raise ValueError("request must be a JSON object")
                response = DATA.hardware_monitor.command_gimbal(request)
                command = str(request.get("command", ""))
                DATA.add_event(f"网页云台命令：{command}")
                self._send_json_response(response, 200 if response.get("ok") else 409)
            except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
                self._send_json_response({"ok": False, "error": str(error)}, 400)
            return
        if self.path == "/api/services/restart":
            try:
                content_length = int(self.headers.get("Content-Length", "0"))
                if content_length <= 0 or content_length > 4096:
                    raise ValueError("invalid request length")
                request = json.loads(self.rfile.read(content_length).decode("utf-8"))
                if not isinstance(request, dict):
                    raise ValueError("request must be a JSON object")
                response = service_control_request(
                    {
                        "action": "restart",
                        "service": str(request.get("service", "")),
                        "confirm": request.get("confirm") is True,
                    }
                )
                service = str(request.get("service", ""))
                if response.get("ok"):
                    DATA.add_event(f"网页请求重启服务：{service}")
                self._send_json_response(response, 202 if response.get("ok") else 409)
            except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
                self._send_json_response({"ok": False, "error": str(error)}, 400)
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
