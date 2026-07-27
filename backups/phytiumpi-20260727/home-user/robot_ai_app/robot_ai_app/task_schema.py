"""High-level task schema for AI robot control."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


TASK_TYPES = {
    "none",
    "stop",
    "look",
    "move_base",
    "move_distance",
    "turn_left",
    "turn_right",
    "turn_angle",
    "inspect_front",
    "inspect_target",
    "navigate_to_object",
    "navigate_to_pose",
}


@dataclass(frozen=True)
class RobotTask:
    type: str
    params: dict[str, Any] = field(default_factory=dict)
    reason: str = ""

    def __post_init__(self):
        if self.type not in TASK_TYPES:
            raise ValueError(f"Unsupported task type: {self.type}")


def task_from_action(action: str, params: dict[str, Any] | None = None, reason: str = "") -> RobotTask:
    if action in {"none", "stop", "look", "move_base"}:
        return RobotTask(action, params or {}, reason)
    if action == "move_forward_short":
        return RobotTask("move_base", {"vx": 0.03, "wz": 0.0, "duration": 0.25}, reason)
    if action == "move_backward_short":
        return RobotTask("move_base", {"vx": -0.03, "wz": 0.0, "duration": 0.25}, reason)
    if action == "turn_left_small":
        return RobotTask("move_base", {"vx": 0.0, "wz": 0.3, "duration": 0.25}, reason)
    if action == "turn_right_small":
        return RobotTask("move_base", {"vx": 0.0, "wz": -0.3, "duration": 0.25}, reason)
    if action == "turn_left":
        return RobotTask("turn_angle", {"angle_deg": 90.0}, reason)
    if action == "turn_right":
        return RobotTask("turn_angle", {"angle_deg": -90.0}, reason)
    if action in {"move_distance", "turn_angle", "inspect_front", "inspect_target", "navigate_to_object", "navigate_to_pose"}:
        return RobotTask(action, params or {}, reason)
    raise ValueError(f"Cannot convert action to task: {action}")
