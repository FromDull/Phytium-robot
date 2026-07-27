"""Conversational Qwen policy for ROS-native robot control."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

from .agent_types import ALLOWED_ACTIONS
from .intent_parser import ParsedIntent
from .qwen_policy import QwenVisionPolicy, extract_json_object, image_file_to_data_url
from .robot_capabilities import DEFAULT_CAPABILITIES
from .robot_manual import build_robot_manual
from .task_schema import RobotTask, task_from_action


CHAT_ACTIONS = sorted(ALLOWED_ACTIONS | {"none"})


@dataclass(frozen=True)
class ChatDecision:
    reply: str
    action: str
    reason: str = ""
    safety: str = "safe"
    params: dict[str, float] | None = None
    task: RobotTask | None = None

    def __post_init__(self):
        if self.action not in CHAT_ACTIONS:
            raise ValueError(f"Unsupported chat action: {self.action}")
        if self.task is None:
            object.__setattr__(self, "task", task_from_action(self.action, self.params, self.reason))


class QwenChatPolicy(QwenVisionPolicy):
    def decide_from_user(
        self,
        user_text: str,
        state: dict[str, Any],
        image_path: str | None = None,
        history: list[dict[str, str]] | None = None,
        intent: ParsedIntent | None = None,
    ) -> ChatDecision:
        payload = self._build_chat_payload(user_text, state, image_path, history or [], intent)
        try:
            response = self._post_json(payload)
            content = response["choices"][0]["message"]["content"]
            decision = parse_chat_decision(content)
        except Exception as exc:
            return ChatDecision(reply=f"决策失败，已停止。原因：{exc}", action="stop", reason=str(exc), safety="unsafe")

        if decision.action == "finish":
            return ChatDecision(reply=decision.reply, action="none", reason="finish is blocked in chat mode")
        if intent is not None and intent.is_direct:
            decision = constrain_decision_to_intent(decision, intent)
        return decision

    def _build_chat_payload(
        self,
        user_text: str,
        state: dict[str, Any],
        image_path: str | None,
        history: list[dict[str, str]],
        intent: ParsedIntent | None,
    ) -> dict[str, Any]:
        compact_state = {
            "status": state.get("status", {}),
            "command": state.get("command", {}),
            "camera": state.get("camera", {}),
            "nav": state.get("nav", {}),
            "limits": state.get("limits", {}),
        }
        messages: list[dict[str, Any]] = [
            {
                "role": "system",
                "content": "你是一个 ROS 机器人对话控制助手。你只能选择一个白名单任务，动作必须保守、安全、小步执行。",
            }
        ]
        messages.extend(history[-6:])
        content: list[dict[str, Any]] = [{"type": "text", "text": build_chat_prompt(user_text, compact_state, intent)}]
        if image_path:
            content.append({"type": "image_url", "image_url": {"url": image_file_to_data_url(image_path)}})
        messages.append({"role": "user", "content": content})
        return {"model": self.model, "messages": messages, "temperature": 0.2, "max_tokens": 220}


def build_chat_prompt(user_text: str, state: dict[str, Any], intent: ParsedIntent | None = None) -> str:
    intent_block = "无高置信本地意图。"
    if intent is not None:
        intent_block = (
            f"本地意图解析：action={intent.action}, confidence={intent.confidence}, "
            f"kind={intent.kind}, reason={intent.reason}。"
        )
    return (
        "用户正在通过自然语言控制 ROS 机器人。请输出一个高层任务。\n\n"
        f"动作白名单：{CHAT_ACTIONS}\n\n"
        f"{build_robot_manual(DEFAULT_CAPABILITIES)}\n\n"
        "安全规则：\n"
        "1. 模糊指令选择 look 或 none，并询问用户。\n"
        "2. 前方不清楚、可能有障碍、姿态异常时选择 stop 或 look。\n"
        "3. 每轮只选一个任务，不输出连续动作计划。\n"
        "4. 用户要求导航到坐标时，可以输出 navigate_to_pose，但不要带 yaw，除非用户明确要求最终朝向。\n"
        "5. 用户要求导航到物体时，若没有目标识别或语义地图，只能 inspect_target。\n"
        "6. 高置信本地方向意图必须尊重；不安全时只能 stop 或 look，不能改成相反方向。\n\n"
        "7. reply用于语音播报，默认最多25个汉字，只说一句简短中文；用户明确要求详细解释时除外。\n\n"
        "8. 如果用户是在闲聊或询问知识，正常简短回答，并将action设为none；不要因为没有机器人动作而拒绝回答。\n\n"
        "只输出 JSON。示例：\n"
        '{"reply":"我先观察门的位置；当前目标识别未启用，不能可靠导航到门。","action":"inspect_target","task":{"type":"inspect_target","params":{"target":"door"}},"safety":"safe","reason":"目标导航能力未启用"}\n\n'
        f"用户指令：{user_text}\n"
        f"{intent_block}\n"
        f"机器人状态：{json.dumps(state, ensure_ascii=False)}"
    )


def parse_chat_decision(text: str) -> ChatDecision:
    parsed = json.loads(extract_json_object(text))
    raw_task = parsed.get("task")
    task_action = raw_task.get("type", "none") if isinstance(raw_task, dict) else "none"
    return ChatDecision(
        reply=str(parsed.get("reply", "")),
        action=str(parsed.get("action") or task_action),
        reason=str(parsed.get("reason", "")),
        safety=str(parsed.get("safety", "safe")),
        params=parse_params(parsed.get("params")),
        task=parse_task(parsed),
    )


def constrain_decision_to_intent(decision: ChatDecision, intent: ParsedIntent) -> ChatDecision:
    if decision.task is not None and is_task_compatible_with_intent(decision.task, intent):
        return decision
    if decision.action in {"stop", "look", "none", intent.action}:
        return decision
    return ChatDecision(
        reply=f"我理解你的指令是：{intent.reason}。我会按这个方向小步执行。",
        action=intent.action or "none",
        reason=f"模型动作 {decision.action} 与明确用户意图 {intent.action} 冲突，已按用户意图修正",
        safety="safe",
        params=intent.params,
    )


def is_task_compatible_with_intent(task: RobotTask, intent: ParsedIntent) -> bool:
    if intent.action != "move_base" or not intent.params:
        return False
    if task.type == "move_distance":
        distance = float(task.params.get("distance_m", 0.0))
        intent_vx = float(intent.params.get("vx", 0.0))
        return distance == 0.0 or intent_vx == 0.0 or (distance > 0) == (intent_vx > 0)
    if task.type == "turn_angle":
        angle = float(task.params.get("angle_deg", 0.0))
        intent_wz = float(intent.params.get("wz", 0.0))
        return angle == 0.0 or intent_wz == 0.0 or (angle > 0) == (intent_wz > 0)
    return False


def parse_params(raw: object) -> dict[str, float] | None:
    if not isinstance(raw, dict):
        return None
    params: dict[str, float] = {}
    for key in ("vx", "wz", "duration"):
        if key in raw:
            params[key] = float(raw[key])
    return params


def parse_task(parsed: dict[str, object]) -> RobotTask | None:
    raw_task = parsed.get("task")
    if isinstance(raw_task, dict):
        task_type = str(raw_task.get("type", parsed.get("action", "none")))
        params = raw_task.get("params")
        return RobotTask(task_type, params if isinstance(params, dict) else {}, str(parsed.get("reason", "")))
    return None
