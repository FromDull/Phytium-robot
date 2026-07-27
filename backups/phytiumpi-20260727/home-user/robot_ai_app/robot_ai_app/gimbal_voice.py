"""Deterministic local voice commands for the gimbal daemon."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
import subprocess


@dataclass(frozen=True)
class GimbalVoiceResult:
    recognized: bool
    ok: bool = False
    reply: str = ""


def _run_gimbalctl(executable: str, *arguments: object) -> dict:
    completed = subprocess.run(
        [executable, *(str(item) for item in arguments)],
        check=False,
        capture_output=True,
        text=True,
        timeout=15,
    )
    output = completed.stdout.strip()
    if not output:
        detail = completed.stderr.strip() or f"exit code {completed.returncode}"
        raise RuntimeError(detail)
    response = json.loads(output)
    if not isinstance(response, dict):
        raise RuntimeError("invalid gimbal response")
    return response


def _step_degrees(text: str, default: float) -> float:
    match = re.search(r"(\d+(?:\.\d+)?)\s*(?:度|°)", text)
    if match:
        return min(float(match.group(1)), 10.0)
    if any(word in text for word in ("大一点", "多一点")):
        return min(default * 2.0, 10.0)
    if any(word in text for word in ("一点", "一点点", "稍微")):
        return default
    return default


def execute_gimbal_voice_command(
    text: str,
    executable: str = "/usr/local/bin/gimbalctl",
    default_step_deg: float = 2.0,
) -> GimbalVoiceResult:
    normalized = "".join(text.lower().split())

    if any(phrase in normalized for phrase in ("云台急停", "紧急停止云台", "相机急停")):
        try:
            response = _run_gimbalctl(executable, "estop")
            return GimbalVoiceResult(True, bool(response.get("ok")), "云台已急停")
        except Exception as error:
            return GimbalVoiceResult(True, False, f"云台急停失败：{error}")

    command = ""
    if any(phrase in normalized for phrase in ("云台回中", "镜头回中", "相机回中", "回到中间", "回正镜头")):
        command = "center"
    elif any(phrase in normalized for phrase in ("向左看", "往左看", "镜头向左", "云台向左", "相机向左")):
        command = "left"
    elif any(phrase in normalized for phrase in ("向右看", "往右看", "镜头向右", "云台向右", "相机向右")):
        command = "right"
    elif any(phrase in normalized for phrase in ("抬头", "向上看", "往上看", "镜头向上", "云台向上")):
        command = "up"
    elif any(phrase in normalized for phrase in ("低头", "向下看", "往下看", "镜头向下", "云台向下")):
        command = "down"
    else:
        return GimbalVoiceResult(False)

    try:
        status_response = _run_gimbalctl(executable, "status")
        telemetry = status_response.get("telemetry", {})
        if not status_response.get("ok") or telemetry.get("state") != 3:
            return GimbalVoiceResult(True, False, "云台未启用，请先手动启用")
        if telemetry.get("fault") != 0 or telemetry.get("feedback_valid_mask") != 3:
            return GimbalVoiceResult(True, False, "云台状态异常，已取消动作")

        if command == "center":
            response = _run_gimbalctl(executable, "center")
            reply = "镜头已回到中间"
        else:
            step = _step_degrees(normalized, default_step_deg)
            yaw = float(telemetry["yaw_deg"])
            pitch = float(telemetry["pitch_deg"])
            if command == "left":
                yaw += step
                reply = "镜头已向左转"
            elif command == "right":
                yaw -= step
                reply = "镜头已向右转"
            elif command == "up":
                pitch += step
                reply = "镜头已抬高"
            else:
                pitch -= step
                reply = "镜头已降低"
            response = _run_gimbalctl(executable, "set", f"{yaw:.2f}", f"{pitch:.2f}")

        if response.get("ok"):
            return GimbalVoiceResult(True, True, reply)
        return GimbalVoiceResult(True, False, f"云台没有执行：{response.get('error', '状态异常')}")
    except Exception as error:
        return GimbalVoiceResult(True, False, f"云台控制失败：{error}")


def execute_look_at_me(
    doa_angle_deg: float | None,
    executable: str = "/usr/local/bin/gimbalctl",
    limit_inset_deg: float = 15.0,
) -> GimbalVoiceResult:
    """Turn to a speaker bearing, inset from live mechanical limits."""
    if doa_angle_deg is None or not math.isfinite(doa_angle_deg):
        return GimbalVoiceResult(True, False, "声源方向不可靠，未转动云台")
    try:
        status_response = _run_gimbalctl(executable, "status")
        telemetry = status_response.get("telemetry", {})
        if (not status_response.get("ok") or telemetry.get("state") != 3 or
                telemetry.get("fault") != 0 or
                telemetry.get("limits_valid_mask") != 15 or
                telemetry.get("feedback_valid_mask") != 3 or
                max(telemetry.get("yaw_feedback_age_ms", 999999),
                    telemetry.get("pitch_feedback_age_ms", 999999)) >= 500):
            return GimbalVoiceResult(True, False, "云台未处于安全可动作状态")

        yaw = float(telemetry["yaw_deg"])
        pitch = float(telemetry["pitch_deg"])
        yaw_min = float(telemetry["yaw_min_deg"])
        yaw_max = float(telemetry["yaw_max_deg"])
        safe_min = yaw_min + max(0.0, limit_inset_deg)
        safe_max = yaw_max - max(0.0, limit_inset_deg)
        if safe_min >= safe_max:
            return GimbalVoiceResult(True, False, "云台机械限位无有效安全区间")
        requested = float(doa_angle_deg)
        target = max(safe_min, min(safe_max, requested))
        if abs(target - yaw) < 1.0:
            return GimbalVoiceResult(True, True, "人物已在声源方向附近")
        response = _run_gimbalctl(executable, "set", f"{target:.2f}", f"{pitch:.2f}")
        if response.get("ok"):
            if target != requested:
                return GimbalVoiceResult(
                    True, True,
                    f"已朝声源方向转动，目标受机械限位保护为{target:.0f}度",
                )
            return GimbalVoiceResult(True, True, "已朝人物声源方向转动")
        return GimbalVoiceResult(True, False, "云台没有执行声源转向")
    except Exception as error:
        return GimbalVoiceResult(True, False, f"云台声源转向失败：{error}")


def refine_person_alignment(box, width, height, executable="/usr/local/bin/gimbalctl") -> GimbalVoiceResult:
    try:
        status = _run_gimbalctl(executable, "status").get("telemetry", {})
        if status.get("state") != 3 or status.get("fault") != 0 or status.get("feedback_valid_mask") != 3:
            return GimbalVoiceResult(True, False, "云台状态异常，停止对人微调")
        x1, y1, x2, y2 = (float(v) for v in box)
        yaw, pitch = float(status["yaw_deg"]), float(status["pitch_deg"])
        # right-of-image requires negative yaw; use upper 20% of the person box as head reference.
        yaw_step = max(-2.0, min(2.0, -((x1 + x2) * 0.5 - width * 0.5) / width * 60.0))
        head_y = y1 + 0.2 * (y2 - y1)
        # Use the full calibrated Pitch workspace for a large vertical error;
        # the final target is still clamped below to the business safety range.
        pitch_step = (height * 0.5 - head_y) / height * 50.0
        target_yaw = max(-72.0, min(72.0, yaw + yaw_step))
        target_pitch = max(-47.0, min(97.0, pitch + pitch_step))
        response = _run_gimbalctl(executable, "set", f"{target_yaw:.2f}", f"{target_pitch:.2f}")
        return GimbalVoiceResult(True, bool(response.get("ok")), "已根据人物位置微调云台" if response.get("ok") else "人物微调未执行")
    except Exception as error:
        return GimbalVoiceResult(True, False, f"人物微调失败：{error}")


def return_yaw_to_forward(executable="/usr/local/bin/gimbalctl") -> GimbalVoiceResult:
    try:
        status = _run_gimbalctl(executable, "status").get("telemetry", {})
        response = _run_gimbalctl(executable, "set", "0.00", f"{float(status['pitch_deg']):.2f}")
        return GimbalVoiceResult(True, bool(response.get("ok")), "未识别到人物，云台已回正" if response.get("ok") else "云台回正未执行")
    except Exception as error:
        return GimbalVoiceResult(True, False, f"云台回正失败：{error}")
