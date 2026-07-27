from collections.abc import Callable
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ExpressionDefinition:
    state: int
    name: str
    description: str


EXPRESSIONS = (
    ExpressionDefinition(0, "smile", "default friendly idle expression"),
    ExpressionDefinition(1, "peek", "scanning or observing"),
    ExpressionDefinition(2, "happy", "successful operation"),
    ExpressionDefinition(3, "love", "positive social interaction"),
    ExpressionDefinition(4, "dizzy", "operation failure or confusion"),
    ExpressionDefinition(5, "angry", "system warning"),
    ExpressionDefinition(6, "sleepy", "idle or sleep state"),
    ExpressionDefinition(7, "surprised", "unexpected event"),
    ExpressionDefinition(8, "uwu", "playful interaction"),
    ExpressionDefinition(9, "hello", "greeting or startup"),
)

EXPRESSION_BY_STATE = {item.state: item for item in EXPRESSIONS}
EXPRESSION_BY_NAME = {item.name: item for item in EXPRESSIONS}
EXPRESSION_ALIASES = {
    "eye_peek": "peek",
    "emotion_smile": "smile",
    "emotion_happy": "happy",
    "emotion_love": "love",
    "emotion_dizzy": "dizzy",
    "emotion_angry": "angry",
    "action_sleepy": "sleepy",
    "emotion_surprised": "surprised",
    "emotion_uwu": "uwu",
}


def resolve_expression(value: object) -> ExpressionDefinition:
    if isinstance(value, bool):
        raise ValueError("expression must be a name or an integer from 0 to 9")
    if isinstance(value, int):
        definition = EXPRESSION_BY_STATE.get(value)
    elif isinstance(value, str):
        normalized = value.strip().casefold()
        if normalized.isdecimal():
            definition = EXPRESSION_BY_STATE.get(int(normalized))
        else:
            normalized = EXPRESSION_ALIASES.get(normalized, normalized)
            definition = EXPRESSION_BY_NAME.get(normalized)
    else:
        definition = None
    if definition is None:
        raise ValueError("unknown expression; use list to get valid names and IDs")
    return definition


@dataclass(frozen=True)
class ExpressionRequest:
    source: str
    expression: int
    priority: int
    force_page: bool
    created_at: float
    expires_at: float | None
    sequence: int


ExpressionCallback = Callable[[int, bool], None]


class ExpressionManager:
    def __init__(
        self,
        callback: ExpressionCallback,
        clock: Callable[[], float],
        default_expression: object = "smile",
    ):
        self.callback = callback
        self.clock = clock
        self.default_expression = resolve_expression(default_expression).state
        self.requests: dict[str, ExpressionRequest] = {}
        self.sequence = 0
        self.current_expression = self.default_expression
        self.current_request_source: str | None = None
        self.current_request_sequence = 0

    @staticmethod
    def _validate_source(source: object) -> str:
        if not isinstance(source, str) or not source.strip():
            raise ValueError("source must be a non-empty string")
        normalized = source.strip()
        if len(normalized.encode("utf-8")) > 64:
            raise ValueError("source must not exceed 64 UTF-8 bytes")
        return normalized

    @staticmethod
    def _validate_priority(priority: object) -> int:
        if isinstance(priority, bool) or not isinstance(priority, int):
            raise ValueError("priority must be an integer from 0 to 100")
        if not 0 <= priority <= 100:
            raise ValueError("priority must be between 0 and 100")
        return priority

    @staticmethod
    def _validate_duration(duration_ms: object) -> int:
        if isinstance(duration_ms, bool) or not isinstance(duration_ms, int):
            raise ValueError("duration_ms must be a non-negative integer")
        if duration_ms < 0:
            raise ValueError("duration_ms must be a non-negative integer")
        return duration_ms

    def show(
        self,
        expression: object,
        *,
        source: object,
        duration_ms: object = 0,
        priority: object = 60,
        force_page: bool = True,
    ) -> dict[str, Any]:
        definition = resolve_expression(expression)
        source_text = self._validate_source(source)
        duration = self._validate_duration(duration_ms)
        priority_value = self._validate_priority(priority)
        if not isinstance(force_page, bool):
            raise ValueError("force_page must be true or false")
        now = self.clock()
        self.sequence += 1
        request = ExpressionRequest(
            source=source_text,
            expression=definition.state,
            priority=priority_value,
            force_page=force_page,
            created_at=now,
            expires_at=None if duration == 0 else now + duration / 1000.0,
            sequence=self.sequence,
        )
        self.requests[source_text] = request
        self._refresh(now)
        return self._request_dict(request, now)

    def clear(self, source: object) -> bool:
        source_text = self._validate_source(source)
        removed = self.requests.pop(source_text, None) is not None
        if removed:
            self._refresh(self.clock())
        return removed

    def clear_all(self) -> int:
        count = len(self.requests)
        self.requests.clear()
        if count:
            self._refresh(self.clock())
        return count

    def set_default(self, expression: object, *, apply: bool = True) -> None:
        self.default_expression = resolve_expression(expression).state
        if not self.requests:
            self.current_expression = self.default_expression
            self.current_request_source = None
            self.current_request_sequence = 0
            if apply:
                self.callback(self.default_expression, False)

    def has_active_requests(self) -> bool:
        return bool(self.requests)

    def has_request(self, source: str) -> bool:
        return source in self.requests

    def tick(self) -> None:
        now = self.clock()
        expired = [
            source
            for source, request in self.requests.items()
            if request.expires_at is not None and request.expires_at <= now
        ]
        for source in expired:
            del self.requests[source]
        if expired:
            self._refresh(now)

    def status(self) -> dict[str, Any]:
        now = self.clock()
        active = self._winner()
        requests = sorted(
            self.requests.values(),
            key=lambda item: (-item.priority, -item.sequence),
        )
        return {
            "current": self._expression_dict(self.current_expression),
            "default": self._expression_dict(self.default_expression),
            "active_source": active.source if active else None,
            "requests": [self._request_dict(item, now) for item in requests],
        }

    def _winner(self) -> ExpressionRequest | None:
        if not self.requests:
            return None
        return max(
            self.requests.values(),
            key=lambda item: (item.priority, item.sequence),
        )

    def _refresh(self, now: float) -> None:
        winner = self._winner()
        expression = winner.expression if winner else self.default_expression
        source = winner.source if winner else None
        sequence = winner.sequence if winner else 0
        if (
            expression == self.current_expression
            and source == self.current_request_source
            and sequence == self.current_request_sequence
        ):
            return
        self.current_expression = expression
        self.current_request_source = source
        self.current_request_sequence = sequence
        self.callback(expression, winner.force_page if winner else False)

    @staticmethod
    def _expression_dict(state: int) -> dict[str, object]:
        definition = EXPRESSION_BY_STATE[state]
        return {"id": definition.state, "name": definition.name}

    def _request_dict(
        self, request: ExpressionRequest, now: float
    ) -> dict[str, object]:
        remaining = None
        if request.expires_at is not None:
            remaining = max(0, round((request.expires_at - now) * 1000))
        return {
            "source": request.source,
            "expression": self._expression_dict(request.expression),
            "priority": request.priority,
            "force_page": request.force_page,
            "remaining_ms": remaining,
        }
