"""Qwen-VL policies. They choose tasks only; ROS code executes them."""

from __future__ import annotations

import base64
import json
import mimetypes
import os
from pathlib import Path
import re
import ssl
import time
from typing import Any
from urllib import request
from urllib.error import HTTPError, URLError

from .agent_types import ALLOWED_ACTIONS, AgentDecision, Observation


DEFAULT_QWEN_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
DEFAULT_QWEN_MODEL = "qwen-vl-max-latest"


class QwenPolicyError(RuntimeError):
    pass


class QwenVisionPolicy:
    def __init__(
        self,
        api_key: str | None = None,
        model: str | None = None,
        base_url: str | None = None,
        timeout: float = 20.0,
    ):
        self.api_key = api_key or os.getenv("DASHSCOPE_API_KEY")
        self.model = model or os.getenv("QWEN_MODEL", DEFAULT_QWEN_MODEL)
        self.base_url = (base_url or os.getenv("QWEN_BASE_URL", DEFAULT_QWEN_BASE_URL)).rstrip("/")
        self.chat_completions_url = resolve_chat_completions_url(self.base_url)
        self.timeout = timeout
        if not self.api_key:
            raise QwenPolicyError("DASHSCOPE_API_KEY is not set")

    def decide(self, observation: Observation) -> AgentDecision:
        if not observation.last_image_path:
            return AgentDecision("look", "need a fresh camera frame before visual decision")

        try:
            payload = self._build_payload(observation)
            response = self._post_json(payload)
            content = response["choices"][0]["message"]["content"]
            decision = parse_decision_text(content)
        except Exception as exc:
            return AgentDecision("stop", f"qwen policy failed: {exc}")

        if decision.action not in ALLOWED_ACTIONS or decision.action == "finish":
            return AgentDecision("stop", f"blocked unsupported or terminal action: {decision.action}")
        return decision

    def _build_payload(self, observation: Observation) -> dict[str, Any]:
        return {
            "model": self.model,
            "messages": [
                {"role": "system", "content": "你是机器人安全决策器。只能从给定动作白名单中选择一个动作。"},
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": build_robot_prompt(observation.state)},
                        {"type": "image_url", "image_url": {"url": image_file_to_data_url(observation.last_image_path)}},
                    ],
                },
            ],
            "temperature": 0.1,
            "max_tokens": 256,
        }

    def _post_json(self, payload: dict[str, Any]) -> dict[str, Any]:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        req = request.Request(
            self.chat_completions_url,
            data=body,
            method="POST",
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
        )
        for attempt in range(2):
            try:
                with request.urlopen(req, timeout=self.timeout) as resp:
                    raw = resp.read().decode("utf-8")
                break
            except HTTPError as exc:
                detail = exc.read().decode("utf-8", errors="replace")
                raise QwenPolicyError(f"Qwen HTTP {exc.code}: {detail}") from exc
            except (URLError, TimeoutError, ssl.SSLError, OSError) as exc:
                if attempt == 0:
                    time.sleep(0.4)
                    continue
                raise QwenPolicyError(f"Cannot reach Qwen API: {exc}") from exc
        return json.loads(raw)


def build_robot_prompt(state: dict[str, Any]) -> str:
    compact_state = {
        "status": state.get("status", {}),
        "command": state.get("command", {}),
        "camera": state.get("camera", {}),
        "nav": state.get("nav", {}),
        "limits": state.get("limits", {}),
    }
    allowed = ["look", "get_status", "move_forward_short", "move_backward_short", "turn_left_small", "turn_right_small", "stop"]
    return (
        "请根据机器人摄像头图像和状态选择下一步动作。\n"
        "安全规则：前方不清晰、有碰撞风险、机器人姿态异常时选择 stop 或 look；优先小步探索。\n\n"
        f"动作白名单：{allowed}\n\n"
        "只输出 JSON，不要输出 Markdown。格式：\n"
        '{"action":"move_forward_short","reason":"前方看起来空旷，可以小步前进"}\n\n'
        f"机器人状态：{json.dumps(compact_state, ensure_ascii=False)}"
    )


def resolve_chat_completions_url(base_url: str) -> str:
    stripped = base_url.rstrip("/")
    if stripped.endswith("/chat/completions"):
        return stripped
    return f"{stripped}/chat/completions"


def image_file_to_data_url(path: str | Path) -> str:
    image_path = Path(path)
    data = image_path.read_bytes()
    mime = mimetypes.guess_type(str(image_path))[0] or "image/jpeg"
    return f"data:{mime};base64,{base64.b64encode(data).decode('ascii')}"


def parse_decision_text(text: str) -> AgentDecision:
    parsed = json.loads(extract_json_object(text))
    return AgentDecision(action=str(parsed.get("action", "stop")), reason=str(parsed.get("reason", "")))


def extract_json_object(text: str) -> str:
    stripped = text.strip()
    if stripped.startswith("{") and stripped.endswith("}"):
        return stripped
    fenced = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.DOTALL)
    if fenced:
        return fenced.group(1)
    start = text.find("{")
    end = text.rfind("}")
    if start >= 0 and end > start:
        return text[start : end + 1]
    raise ValueError(f"No JSON object found in model response: {text!r}")
