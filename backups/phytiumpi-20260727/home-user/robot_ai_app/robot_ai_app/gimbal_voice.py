"""Deterministic, safety-bounded local voice commands for the gimbal."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
import subprocess
import time


FEEDBACK_MAX_AGE_MS = 500
VOICE_LIMIT_INSET_DEG = 5.0
DOA_DEADBAND_DEG = 4.0
GIMBAL_QUERY_TIMEOUT_SECONDS = 5
GIMBAL_MOTION_TIMEOUT_SECONDS = 20


@dataclass(frozen=True)
class GimbalVoiceResult:
    recognized: bool
    ok: bool = False
    reply: str = ""
    action: str | None = None


def _run_gimbalctl(executable: str, *arguments: object) -> dict:
    command = str(arguments[0]) if arguments else ""
    timeout_seconds = (
        GIMBAL_MOTION_TIMEOUT_SECONDS
        if command in {"set", "center"}
        else GIMBAL_QUERY_TIMEOUT_SECONDS
    )
    completed = subprocess.run(
        [executable, *(str(item) for item in arguments)],
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout_seconds,
    )
    output = completed.stdout.strip()
    if not output:
        detail = completed.stderr.strip() or f"exit code {completed.returncode}"
        raise RuntimeError(detail)
    response = json.loads(output)
    if not isinstance(response, dict):
        raise RuntimeError("invalid gimbal response")
    return response


def _chinese_number(text: str) -> float | None:
    digits = {"零": 0, "〇": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
              "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}
    if not text:
        return None
    if "十" not in text:
        if len(text) == 1 and text in digits:
            return float(digits[text])
        return None
    tens, ones = text.split("十", 1)
    tens_value = 1 if not tens else digits.get(tens)
    ones_value = 0 if not ones else digits.get(ones)
    if tens_value is None or ones_value is None:
        return None
    return float(tens_value * 10 + ones_value)


def _step_degrees(text: str, default: float) -> float:
    match = re.search(r"(\d+(?:\.\d+)?)\s*(?:度|°)", text)
    value = float(match.group(1)) if match else None
    if value is None:
        match = re.search(r"([零〇一二两三四五六七八九十]+)\s*(?:度|°)", text)
        value = _chinese_number(match.group(1)) if match else None
    if value is not None:
        return max(0.5, min(value, 30.0))
    if any(word in text for word in ("大幅", "大一点", "多一点", "多一些")):
        return min(default * 2.0, 30.0)
    if any(word in text for word in ("一点点", "稍微", "小幅")):
        return max(1.0, min(default * 0.5, 5.0))
    return max(0.5, min(default, 30.0))


def _normalize_signed_angle(angle_deg: float) -> float:
    value = (float(angle_deg) + 180.0) % 360.0 - 180.0
    return 180.0 if value == -180.0 else value


def _active_telemetry(status_response: dict, require_limits: bool = True) -> tuple[dict | None, str | None]:
    telemetry = status_response.get("telemetry", {})
    if not status_response.get("ok") or telemetry.get("state") != 3:
        return None, "云台未启用，请先说启用云台"
    if telemetry.get("fault") != 0 or telemetry.get("feedback_valid_mask") != 3:
        return None, "云台反馈异常，已取消动作"
    feedback_age = max(
        int(telemetry.get("yaw_feedback_age_ms", FEEDBACK_MAX_AGE_MS)),
        int(telemetry.get("pitch_feedback_age_ms", FEEDBACK_MAX_AGE_MS)),
    )
    if feedback_age >= FEEDBACK_MAX_AGE_MS:
        return None, "云台反馈超时，已取消动作"
    if require_limits and telemetry.get("limits_valid_mask") != 15:
        return None, "云台机械限位无效，已取消动作"
    return telemetry, None


def _safe_limits(telemetry: dict, inset_deg: float = VOICE_LIMIT_INSET_DEG) -> tuple[float, float, float, float]:
    inset = max(0.0, float(inset_deg))
    yaw_min = float(telemetry["yaw_min_deg"]) + inset
    yaw_max = float(telemetry["yaw_max_deg"]) - inset
    pitch_min = float(telemetry["pitch_min_deg"]) + inset
    pitch_max = float(telemetry["pitch_max_deg"]) - inset
    if yaw_min >= yaw_max or pitch_min >= pitch_max:
        raise ValueError("机械限位没有有效安全区间")
    return yaw_min, yaw_max, pitch_min, pitch_max


def _motion_feedback_reached(
    executable: str,
    target_yaw: float,
    target_pitch: float,
    position_tolerance_deg: float = 1.5,
) -> bool:
    """Confirm a command that settled just after the daemon wait window."""
    try:
        response = _run_gimbalctl(executable, "status")
        telemetry, error = _active_telemetry(response)
        if error or telemetry is None:
            return False
        position_ready = (
            abs(float(telemetry["yaw_deg"]) - target_yaw) <= position_tolerance_deg
            and abs(float(telemetry["pitch_deg"]) - target_pitch) <= position_tolerance_deg
        )
        speed_ready = (
            abs(float(telemetry.get("yaw_speed_rpm", 999))) <= 5
            and abs(float(telemetry.get("pitch_speed_rpm", 999))) <= 5
        )
        return position_ready and speed_ready
    except Exception:
        return False


def _contains_any(text: str, phrases: tuple[str, ...]) -> bool:
    return any(phrase in text for phrase in phrases)


def execute_gimbal_voice_command(
    text: str,
    executable: str = "/usr/local/bin/gimbalctl",
    default_step_deg: float = 5.0,
) -> GimbalVoiceResult:
    normalized = "".join(text.lower().split())

    if _contains_any(normalized, ("云台急停", "紧急停止云台", "相机急停", "镜头急停")):
        try:
            response = _run_gimbalctl(executable, "estop")
            return GimbalVoiceResult(True, bool(response.get("ok")), "云台已急停", "estop")
        except Exception as error:
            return GimbalVoiceResult(True, False, f"云台急停失败：{error}", "estop")

    if _contains_any(normalized, ("启用云台", "打开云台", "启动云台", "开启云台")):
        try:
            response = _run_gimbalctl(executable, "enable", "--confirm")
            return GimbalVoiceResult(
                True,
                bool(response.get("ok")),
                "云台正在启用，请稍候" if response.get("ok") else f"云台启用失败：{response.get('error', '状态异常')}",
                "enable",
            )
        except Exception as error:
            return GimbalVoiceResult(True, False, f"云台启用失败：{error}", "enable")

    if _contains_any(normalized, ("关闭云台", "停用云台", "收起云台", "关闭镜头")):
        try:
            response = _run_gimbalctl(executable, "disable", "--confirm")
            return GimbalVoiceResult(
                True,
                bool(response.get("ok")),
                "云台正在安全回收并关闭" if response.get("ok") else f"云台关闭失败：{response.get('error', '状态异常')}",
                "disable",
            )
        except Exception as error:
            return GimbalVoiceResult(True, False, f"云台关闭失败：{error}", "disable")

    center = _contains_any(
        normalized,
        ("云台回中", "镜头回中", "相机回中", "回到中间", "回正镜头", "镜头回正", "看向正前方"),
    )
    left = _contains_any(
        normalized,
        ("向左看", "往左看", "镜头向左", "云台向左", "相机向左", "镜头左转", "云台左转", "相机左转", "左转镜头"),
    )
    right = _contains_any(
        normalized,
        ("向右看", "往右看", "镜头向右", "云台向右", "相机向右", "镜头右转", "云台右转", "相机右转", "右转镜头"),
    )
    up = _contains_any(
        normalized,
        ("抬头", "向上看", "往上看", "镜头向上", "云台向上", "相机向上", "抬高镜头"),
    )
    down = _contains_any(
        normalized,
        ("低头", "向下看", "往下看", "镜头向下", "云台向下", "相机向下", "降低镜头"),
    )
    if not (center or left or right or up or down):
        return GimbalVoiceResult(False)
    if left and right or up and down:
            return GimbalVoiceResult(True, False, "云台方向指令互相冲突，请重新说", "move")

    try:
        status_response = _run_gimbalctl(executable, "status")
        telemetry, error = _active_telemetry(status_response)
        if error:
            return GimbalVoiceResult(True, False, error, "center" if center else "move")
        assert telemetry is not None

        if center:
            response = _run_gimbalctl(executable, "center")
            reply = "镜头已回到正前方"
        else:
            step = _step_degrees(normalized, default_step_deg)
            yaw = float(telemetry["yaw_deg"])
            pitch = float(telemetry["pitch_deg"])
            yaw_min, yaw_max, pitch_min, pitch_max = _safe_limits(telemetry)
            target_yaw = yaw + step * (1 if left else -1 if right else 0)
            target_pitch = pitch + step * (1 if up else -1 if down else 0)
            target_yaw = max(yaw_min, min(yaw_max, target_yaw))
            target_pitch = max(pitch_min, min(pitch_max, target_pitch))
            if abs(target_yaw - yaw) < 0.25 and abs(target_pitch - pitch) < 0.25:
                return GimbalVoiceResult(True, True, "镜头已到当前方向的安全限位", "move")
            response = _run_gimbalctl(executable, "set", f"{target_yaw:.2f}", f"{target_pitch:.2f}")
            direction_parts = []
            if left or right:
                direction_parts.append("向左" if left else "向右")
            if up or down:
                direction_parts.append(("并" if direction_parts else "") + ("抬高" if up else "降低"))
            directions = "".join(direction_parts)
            reply = f"镜头已{directions}调整到水平{target_yaw:.0f}度、俯仰{target_pitch:.0f}度"

        if response.get("ok"):
            return GimbalVoiceResult(True, True, reply, "center" if center else "move")
        return GimbalVoiceResult(
            True,
            False,
            f"云台没有执行：{response.get('error', '状态异常')}",
            "center" if center else "move",
        )
    except Exception as error:
        return GimbalVoiceResult(True, False, f"云台控制失败：{error}", "center" if center else "move")


def execute_look_at_me(
    doa_angle_deg: float | None,
    executable: str = "/usr/local/bin/gimbalctl",
    limit_inset_deg: float = 15.0,
    activation_wait_seconds: float = 0.0,
) -> GimbalVoiceResult:
    """Turn toward a calibrated speaker bearing with jitter and slew limits."""
    if doa_angle_deg is None or not math.isfinite(doa_angle_deg):
        return GimbalVoiceResult(True, False, "声源方向不可靠，未转动云台")
    try:
        deadline = time.monotonic() + max(0.0, activation_wait_seconds)
        while True:
            status_response = _run_gimbalctl(executable, "status")
            telemetry, error = _active_telemetry(status_response)
            if error is None or time.monotonic() >= deadline:
                break
            raw_telemetry = status_response.get("telemetry", {})
            if not status_response.get("ok") or raw_telemetry.get("fault", 0) != 0:
                break
            time.sleep(0.2)
        if error:
            return GimbalVoiceResult(True, False, error)
        assert telemetry is not None
        yaw = float(telemetry["yaw_deg"])
        pitch = float(telemetry["pitch_deg"])
        safe_min, safe_max, _, _ = _safe_limits(telemetry, limit_inset_deg)
        requested = _normalize_signed_angle(doa_angle_deg)
        desired = max(safe_min, min(safe_max, requested))
        delta = desired - yaw
        if abs(delta) < DOA_DEADBAND_DEG:
            return GimbalVoiceResult(True, True, "人物已在声源方向附近")
        target = desired
        try:
            response = _run_gimbalctl(executable, "set", f"{target:.2f}", f"{pitch:.2f}")
            motion_ok = bool(response.get("ok"))
        except subprocess.TimeoutExpired:
            response = {"ok": False, "error": "motion confirmation timed out"}
            motion_ok = False
        if not motion_ok:
            motion_ok = _motion_feedback_reached(executable, target, pitch)
        if motion_ok:
            if abs(desired - requested) >= 0.5:
                return GimbalVoiceResult(True, True, f"已朝声源转动，受安全限位保护为{target:.0f}度")
            return GimbalVoiceResult(True, True, f"已朝人物声源转动到{target:.0f}度")
        return GimbalVoiceResult(
            True,
            False,
            f"云台没有执行声源转向：{response.get('error', '状态异常')}",
        )
    except Exception as error:
        return GimbalVoiceResult(True, False, f"云台声源转向失败：{error}")


def refine_person_alignment(box, width, height, executable="/usr/local/bin/gimbalctl") -> GimbalVoiceResult:
    try:
        if float(width) <= 0 or float(height) <= 0:
            raise ValueError("invalid image size")
        status_response = _run_gimbalctl(executable, "status")
        status, error = _active_telemetry(status_response)
        if error:
            return GimbalVoiceResult(True, False, error)
        assert status is not None
        x1, y1, x2, y2 = (float(value) for value in box)
        yaw, pitch = float(status["yaw_deg"]), float(status["pitch_deg"])
        yaw_min, yaw_max, pitch_min, pitch_max = _safe_limits(status)
        horizontal_error = ((x1 + x2) * 0.5 - width * 0.5) / width
        head_y = y1 + 0.2 * (y2 - y1)
        vertical_error = (height * 0.5 - head_y) / height
        yaw_step = max(-3.0, min(3.0, -horizontal_error * 60.0))
        pitch_step = max(-6.0, min(6.0, vertical_error * 50.0))
        if abs(horizontal_error) < 0.025:
            yaw_step = 0.0
        if abs(vertical_error) < 0.035:
            pitch_step = 0.0
        if yaw_step == 0.0 and pitch_step == 0.0:
            return GimbalVoiceResult(True, True, "人物已位于画面中心")
        target_yaw = max(yaw_min, min(yaw_max, yaw + yaw_step))
        target_pitch = max(pitch_min, min(pitch_max, pitch + pitch_step))
        response = _run_gimbalctl(executable, "set", f"{target_yaw:.2f}", f"{target_pitch:.2f}")
        return GimbalVoiceResult(True, bool(response.get("ok")), "已根据人物位置微调云台" if response.get("ok") else "人物微调未执行")
    except Exception as error:
        return GimbalVoiceResult(True, False, f"人物微调失败：{error}")


def return_yaw_to_forward(executable="/usr/local/bin/gimbalctl") -> GimbalVoiceResult:
    try:
        status_response = _run_gimbalctl(executable, "status")
        status, error = _active_telemetry(status_response)
        if error:
            return GimbalVoiceResult(True, False, error)
        assert status is not None
        response = _run_gimbalctl(executable, "set", "0.00", f"{float(status['pitch_deg']):.2f}")
        return GimbalVoiceResult(True, bool(response.get("ok")), "未识别到人物，云台已回正" if response.get("ok") else "云台回正未执行")
    except Exception as error:
        return GimbalVoiceResult(True, False, f"云台回正失败：{error}")
