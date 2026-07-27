"""Capability declaration shared by prompts, executors, and launch config."""

from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class RobotCapabilities:
    camera: bool = True
    localization: bool = False
    navigation: bool = False
    target_detection: bool = True
    semantic_map: bool = False
    basic_turn: bool = False
    basic_motion: bool = False

    def to_dict(self) -> dict[str, bool]:
        return asdict(self)


DEFAULT_CAPABILITIES = RobotCapabilities()
