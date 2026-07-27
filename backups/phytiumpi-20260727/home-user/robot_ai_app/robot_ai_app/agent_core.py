"""Agent integration layer for ROS-native robot tools."""

from __future__ import annotations

from .agent_types import AgentDecision, AgentPolicy, Observation
from .robot_tools import RobotTools, ToolResult


class ScriptedPolicy:
    def __init__(self, actions: list[str] | None = None):
        self.actions = actions or ["get_status", "look", "move_forward_short", "turn_left_small", "stop", "look", "finish"]

    def decide(self, observation: Observation) -> AgentDecision:
        index = min(observation.step, len(self.actions) - 1)
        return AgentDecision(action=self.actions[index], reason="scripted validation sequence")


class ReactiveSafetyPolicy:
    def decide(self, observation: Observation) -> AgentDecision:
        status = observation.state.get("status", {})
        pitch_deg = abs(float(status.get("pitch_deg", 0.0)))
        command_active = bool(status.get("command_active", False))

        if pitch_deg > 12.0:
            return AgentDecision("stop", f"unsafe posture: pitch_deg={pitch_deg:.2f}")
        if command_active:
            return AgentDecision("get_status", "wait for current command to settle")

        cycle = observation.step % 5
        if cycle == 0:
            return AgentDecision("look", "refresh visual observation")
        if cycle in (1, 2):
            return AgentDecision("move_forward_short", "slow exploration")
        if cycle == 3:
            return AgentDecision("turn_left_small", "scan nearby view")
        return AgentDecision("stop", "brief settle period")


class AgentRunner:
    def __init__(self, tools: RobotTools, policy: AgentPolicy, capture_dir: str = "captures"):
        self.tools = tools
        self.policy = policy
        self.capture_dir = capture_dir
        self.last_image_path: str | None = None

    def run_step(self, step: int) -> ToolResult:
        state_result = self.tools.get_status()
        observation = Observation(step=step, state=state_result.data, last_image_path=self.last_image_path)
        decision = self.policy.decide(observation)

        if decision.action == "finish":
            return ToolResult(ok=True, action="finish", data={"reason": decision.reason})

        result = self._execute(decision.action)
        result.data["decision_reason"] = decision.reason
        return result

    def _execute(self, action: str) -> ToolResult:
        if action == "get_status":
            return self.tools.get_status()
        if action == "look":
            result = self.tools.look(self.capture_dir)
            self.last_image_path = result.data["path"]
            return result
        if action == "move_forward_short":
            return self.tools.move_forward_short()
        if action == "move_backward_short":
            return self.tools.move_backward_short()
        if action == "turn_left_small":
            return self.tools.turn_left_small()
        if action == "turn_right_small":
            return self.tools.turn_right_small()
        if action == "stop":
            return self.tools.stop()
        raise ValueError(f"AgentRunner only executes immediate actions, got: {action}")
