"""Semantic map extension point for object-aware navigation."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SemanticPlace:
    name: str
    x: float
    y: float
    yaw: float = 0.0
    confidence: float = 1.0


class SemanticMap:
    available = False

    def lookup(self, target: str) -> SemanticPlace | None:
        del target
        return None
