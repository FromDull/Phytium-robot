"""Fail-closed motion safety decisions from read-only robot state."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Any
import urllib.request


@dataclass(frozen=True)
class MotionRequest:
    linear_m_s: float = 0.0
    angular_rad_s: float = 0.0
    mode: str = "direct"
    require_gimbal: bool = False
    require_target_tf: bool = False


@dataclass(frozen=True)
class SafetyReason:
    code: str
    detail: str


@dataclass(frozen=True)
class SafetyDecision:
    allowed: bool
    reasons: tuple[SafetyReason, ...]
    warnings: tuple[SafetyReason, ...]
    front_clearance_m: float | None
    evaluated_at: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "allowed": self.allowed,
            "reasons": [reason.__dict__ for reason in self.reasons],
            "warnings": [warning.__dict__ for warning in self.warnings],
            "front_clearance_m": self.front_clearance_m,
            "evaluated_at": self.evaluated_at,
        }


class MotionSafetySupervisor:
    def __init__(
        self,
        minimum_front_clearance_m: float = 0.45,
        rotation_clearance_m: float = 0.35,
        max_pitch_deg: float = 5.0,
        max_roll_deg: float = 5.0,
    ) -> None:
        self.minimum_front_clearance_m = minimum_front_clearance_m
        self.rotation_clearance_m = rotation_clearance_m
        self.max_pitch_deg = max_pitch_deg
        self.max_roll_deg = max_roll_deg

    def evaluate(
        self,
        request: MotionRequest,
        state: dict[str, Any],
        chassis: dict[str, Any] | None = None,
    ) -> SafetyDecision:
        reasons: list[SafetyReason] = []
        warnings: list[SafetyReason] = []

        system = state.get("system", {})
        cpu = _number(system.get("cpu_percent"))
        load = system.get("load") or []
        if cpu is None:
            reasons.append(SafetyReason("SYSTEM_STATE_MISSING", "CPU state is unavailable"))
        elif cpu >= 95.0:
            reasons.append(SafetyReason("SYSTEM_OVERLOAD", f"CPU usage is {cpu:.1f}%"))
        elif cpu >= 85.0:
            warnings.append(SafetyReason("SYSTEM_LOAD_HIGH", f"CPU usage is {cpu:.1f}%"))
        if load and _number(load[0]) is not None and float(load[0]) >= 9.0:
            warnings.append(SafetyReason("LOAD_AVERAGE_HIGH", f"one-minute load is {float(load[0]):.2f}"))

        radar = state.get("radar", {})
        if not radar.get("online"):
            reasons.append(SafetyReason("RADAR_OFFLINE", "radar is not online"))
        elif _age_exceeds(radar.get("last_scan_age_ms"), 200):
            reasons.append(SafetyReason("RADAR_STALE", "radar scan is older than 200 ms"))

        camera = state.get("camera", {})
        if not all(camera.get(key) for key in ("online", "depth_online", "aligned_depth_online", "aligned")):
            reasons.append(SafetyReason("DEPTH_NOT_READY", "aligned depth camera is not ready"))
        elif max(_age(camera.get("depth_age_ms")), _age(camera.get("aligned_depth_age_ms"))) > 250:
            reasons.append(SafetyReason("DEPTH_STALE", "depth data is older than 250 ms"))

        hardware = state.get("hardware", {})
        imu = hardware.get("imu", {})
        if not imu.get("online") or not imu.get("valid"):
            reasons.append(SafetyReason("IMU_INVALID", "IMU data is unavailable or invalid"))
        else:
            pitch = abs(_number(imu.get("pitch_deg")) or 0.0)
            roll = abs(_number(imu.get("roll_deg")) or 0.0)
            if pitch > self.max_pitch_deg or roll > self.max_roll_deg:
                reasons.append(SafetyReason("POSTURE_UNSAFE", f"pitch={pitch:.2f} roll={roll:.2f} deg"))
            if not imu.get("calibrated"):
                reasons.append(SafetyReason("IMU_NOT_CALIBRATED", "IMU calibration is not confirmed"))

        map_state = state.get("map", {})
        base_pose = (map_state.get("world_poses") or {}).get("base_link", {})
        if not map_state.get("online") or not base_pose.get("online"):
            reasons.append(SafetyReason("LOCALIZATION_UNAVAILABLE", "map to base_link pose is unavailable"))
        elif _age_exceeds(base_pose.get("age_ms"), 300):
            reasons.append(SafetyReason("LOCALIZATION_STALE", "base_link pose is older than 300 ms"))

        if request.mode == "nav" and not state.get("nav", {}).get("available", False):
            reasons.append(SafetyReason("NAV_UNAVAILABLE", "Nav2 action is unavailable"))
        if request.require_target_tf:
            targets = state.get("targets_3d", {})
            if not targets.get("tf_valid"):
                reasons.append(SafetyReason("TARGET_TF_INVALID", str(targets.get("tf_reason", "target TF unavailable"))))
        if request.require_gimbal:
            gimbal = hardware.get("gimbal", {})
            if not gimbal.get("online") or not gimbal.get("ready"):
                reasons.append(SafetyReason("GIMBAL_NOT_READY", "gimbal status is not motion-ready"))

        if chassis is None:
            reasons.append(SafetyReason("CHASSIS_STATE_MISSING", "broker-backed chassis state is not available"))
        else:
            if int(chassis.get("fault", -1)) != 0:
                reasons.append(SafetyReason("CHASSIS_FAULT", f"fault={chassis.get('fault')}"))
            if int(chassis.get("balance_state", -1)) != 3:
                reasons.append(SafetyReason("BALANCE_NOT_ACTIVE", f"balance_state={chassis.get('balance_state')}"))
            if _age_exceeds(chassis.get("age_ms"), 200):
                reasons.append(SafetyReason("CHASSIS_STATE_STALE", "chassis state is older than 200 ms"))

        clearance = self._clearance_for_request(request, state.get("scan", {}).get("points", []))
        if abs(request.linear_m_s) > 1e-6:
            if clearance is None:
                reasons.append(SafetyReason("CLEARANCE_UNKNOWN", "no valid radar points in travel corridor"))
            elif clearance < self.minimum_front_clearance_m:
                reasons.append(SafetyReason("OBSTACLE_TOO_CLOSE", f"travel clearance is {clearance:.3f} m"))
        elif abs(request.angular_rad_s) > 1e-6:
            if clearance is None:
                reasons.append(SafetyReason("ROTATION_CLEARANCE_UNKNOWN", "no valid radar points for rotation"))
            elif clearance < self.rotation_clearance_m:
                reasons.append(SafetyReason("ROTATION_BLOCKED", f"radial clearance is {clearance:.3f} m"))

        return SafetyDecision(not reasons, tuple(reasons), tuple(warnings), clearance, time.time())

    def _clearance_for_request(self, request: MotionRequest, points: list[dict[str, Any]]) -> float | None:
        values: list[float] = []
        rotating_only = abs(request.linear_m_s) <= 1e-6 and abs(request.angular_rad_s) > 1e-6
        direction = 0.0 if request.linear_m_s >= 0 else math.pi
        for point in points:
            angle = _number(point.get("a"))
            distance = _number(point.get("r"))
            if angle is None or distance is None or not math.isfinite(distance) or distance <= 0.05:
                continue
            if rotating_only or abs(_normalize_radians(angle - direction)) <= math.radians(25):
                values.append(distance)
        return min(values) if values else None


def load_json_url(url: str, timeout: float = 3.0) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        value = json.load(response)
    if not isinstance(value, dict):
        raise ValueError("state endpoint did not return a JSON object")
    return value


def load_optional_json(path: str | Path | None) -> dict[str, Any] | None:
    if not path:
        return None
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("chassis state must be a JSON object")
    return value


def _number(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _age(value: Any) -> float:
    number = _number(value)
    return math.inf if number is None else number


def _age_exceeds(value: Any, maximum: float) -> bool:
    return _age(value) > maximum


def _normalize_radians(value: float) -> float:
    return (value + math.pi) % (2 * math.pi) - math.pi
