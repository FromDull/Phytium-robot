"""Chinese user intent parser used before calling the model."""

from __future__ import annotations

from dataclasses import dataclass
import re


DIRECT_ACTIONS = {"move_base", "stop", "look"}


@dataclass(frozen=True)
class ParsedIntent:
    action: str | None
    confidence: float
    kind: str
    reason: str
    params: dict[str, float] | None = None

    @property
    def is_direct(self) -> bool:
        return self.action in DIRECT_ACTIONS and self.confidence >= 0.8


def parse_user_intent(text: str) -> ParsedIntent:
    normalized = normalize_text(text)
    duration = parse_duration(normalized)
    scale = parse_strength_scale(normalized)

    if not normalized:
        return ParsedIntent(None, 0.0, "empty", "empty input")
    if is_complex_instruction(normalized):
        return ParsedIntent(None, 0.0, "complex", "包含条件、观察或后续动作，交给模型综合判断")
    if is_high_level_motion(normalized):
        return ParsedIntent(None, 0.0, "high_level_motion", "包含距离、角度或导航语义，交给模型生成高层任务")
    if contains_any(normalized, ["停", "停止", "停下", "别动", "不要动", "刹车", "stop"]):
        return ParsedIntent("stop", 1.0, "direct_stop", "用户明确要求停止")
    if contains_any(normalized, ["左转", "向左转", "往左转", "左拐", "left"]):
        return ParsedIntent("move_base", 1.0, "direct_motion", "用户明确要求左转", motion_params(0.0, 0.3 * scale, duration))
    if contains_any(normalized, ["右转", "向右转", "往右转", "右拐", "right"]):
        return ParsedIntent("move_base", 1.0, "direct_motion", "用户明确要求右转", motion_params(0.0, -0.3 * scale, duration))
    if contains_any(normalized, ["后退", "倒退", "往后", "向后", "退后", "back"]):
        return ParsedIntent("move_base", 1.0, "direct_motion", "用户明确要求后退", motion_params(-0.03 * scale, 0.0, duration))
    if contains_any(normalized, ["前进", "向前", "往前", "走近", "靠近", "forward"]):
        return ParsedIntent("move_base", 1.0, "direct_motion", "用户明确要求前进", motion_params(0.03 * scale, 0.0, duration))
    if contains_any(normalized, ["看", "看看", "观察", "拍照", "识别", "前面有什么"]):
        return ParsedIntent("look", 0.9, "direct_perception", "用户明确要求观察")
    return ParsedIntent(None, 0.0, "open_ended", "开放式或复杂指令")


def normalize_text(text: str) -> str:
    return "".join(text.lower().strip().split())


def contains_any(text: str, keywords: list[str]) -> bool:
    return any(keyword in text for keyword in keywords)


def is_complex_instruction(text: str) -> bool:
    return contains_any(text, ["然后", "如果", "要是", "判断", "找", "空的地方", "空旷", "避开", "绕开"])


def is_high_level_motion(text: str) -> bool:
    return bool(re.search(r"\d+(?:\.\d+)?(米|m|厘米|cm|度|°)", text)) or contains_any(
        text, ["避障", "导航", "走到", "移动到", "去到", "到达"]
    )


def parse_strength_scale(text: str) -> float:
    if contains_any(text, ["大一点", "多一点", "快一点", "快点", "久一点", "远一点"]):
        return 1.6
    if contains_any(text, ["小一点", "慢一点", "慢点", "轻轻", "一点点"]):
        return 0.6
    return 1.0


def parse_duration(text: str) -> float:
    match = re.search(r"(\d+(?:\.\d+)?)秒", text)
    if match:
        return float(match.group(1))
    if contains_any(text, ["久一点", "多一点"]):
        return 0.6
    if contains_any(text, ["一点点", "短一点"]):
        return 0.15
    return 0.25


def motion_params(vx: float, wz: float, duration: float) -> dict[str, float]:
    return {"vx": vx, "wz": wz, "duration": duration}
