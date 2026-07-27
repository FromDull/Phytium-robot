"""ROS-independent types shared by AI policies and the robot agent."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


ALLOWED_ACTIONS = {
    "look",
    "get_status",
    "move_forward_short",
    "move_backward_short",
    "turn_left_small",
    "turn_right_small",
    "move_base",
    "move_distance",
    "turn_left",
    "turn_right",
    "turn_angle",
    "inspect_front",
    "inspect_target",
    "navigate_to_object",
    "navigate_to_pose",
    "stop",
    "finish",
}


@dataclass(frozen=True)
class Observation:
    step: int
    state: dict
    last_image_path: str | None = None


@dataclass(frozen=True)
class AgentDecision:
    action: str
    reason: str = ""

    def __post_init__(self):
        if self.action not in ALLOWED_ACTIONS:
            raise ValueError(f"Unsupported agent action: {self.action}")


class AgentPolicy(Protocol):
    def decide(self, observation: Observation) -> AgentDecision:
        """Return the next semantic action for the robot."""
