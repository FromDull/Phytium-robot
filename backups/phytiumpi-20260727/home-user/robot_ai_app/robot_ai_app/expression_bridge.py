"""Best-effort voice state bridge for the serial-screen expression service."""

from __future__ import annotations

import json
from pathlib import Path
import socket
import sys
import time
from typing import Callable


DEFAULT_SOCKET = "/run/wifi-screen/face.sock"
SOURCE = "voice"
PRIORITIES = {
    "idle": 20,
    "listening": 40,
    "wake": 50,
    "thinking": 50,
    "speaking": 50,
    "happy": 60,
    "gimbal": 60,
    "look": 60,
    "confused": 60,
    "error": 90,
}


class ExpressionBridge:
    def __init__(
        self,
        mapping: dict[str, str],
        socket_path: str = DEFAULT_SOCKET,
        timeout_s: float = 0.2,
        error_interval_s: float = 30.0,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self.mapping = dict(mapping)
        self.socket_path = socket_path
        self.timeout_s = timeout_s
        self.error_interval_s = error_interval_s
        self.logger = logger or (lambda message: print(message, file=sys.stderr))
        self.current_state: str | None = None
        self.keep_on_next_idle = False
        self.last_error_at = -float("inf")

    @classmethod
    def from_file(
        cls,
        path: str | Path,
        socket_path: str = DEFAULT_SOCKET,
        **kwargs,
    ) -> "ExpressionBridge":
        value = json.loads(Path(path).read_text(encoding="utf-8"))
        mapping = value.get("mapping") if isinstance(value, dict) else None
        if not isinstance(mapping, dict):
            raise ValueError("face map must contain a mapping object")
        normalized = {
            str(state): str(expression)
            for state, expression in mapping.items()
            if str(state) and str(expression)
        }
        return cls(normalized, socket_path=socket_path, **kwargs)

    def show(self, state: str, duration_ms: int = 0) -> bool:
        expression = self.mapping.get(state)
        if not expression:
            return False
        if self.current_state == state and duration_ms == 0:
            return True
        payload = {
            "version": 1,
            "action": "show",
            "expression": expression,
            "duration_ms": max(0, int(duration_ms)),
            "priority": PRIORITIES.get(state, 50),
            "source": SOURCE,
            "force_page": True,
        }
        if not self._request(payload):
            return False
        self.current_state = state if duration_ms == 0 else None
        return True

    def keep_current(self) -> None:
        """Keep the current result visible when this turn performs final cleanup."""
        self.keep_on_next_idle = self.current_state is not None

    def idle(self, force: bool = False) -> bool:
        if self.keep_on_next_idle and not force:
            self.keep_on_next_idle = False
            return True
        self.keep_on_next_idle = False
        if self.current_state is None and not force:
            return True
        if not self._request({"version": 1, "action": "clear", "source": SOURCE}):
            return False
        self.current_state = None
        return True

    def set_default(self, state: str = "idle") -> bool:
        expression = self.mapping.get(state)
        if not expression:
            return False
        return self._request({
            "version": 1,
            "action": "set_default",
            "expression": expression,
        })

    def _request(self, payload: dict) -> bool:
        if not hasattr(socket, "AF_UNIX"):
            return False
        connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        raw = bytearray()
        try:
            connection.settimeout(self.timeout_s)
            connection.connect(self.socket_path)
            connection.sendall(
                json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"
            )
            while b"\n" not in raw and len(raw) <= 65536:
                chunk = connection.recv(4096)
                if not chunk:
                    break
                raw.extend(chunk)
            if not raw or len(raw) > 65536:
                raise RuntimeError("expression service returned an invalid response")
            response = json.loads(bytes(raw).split(b"\n", 1)[0].decode("utf-8"))
            if not isinstance(response, dict) or not response.get("ok"):
                detail = response.get("error", "request failed") if isinstance(response, dict) else "invalid response"
                raise RuntimeError(str(detail))
            return True
        except (OSError, RuntimeError, UnicodeDecodeError, json.JSONDecodeError) as error:
            now = time.monotonic()
            if now - self.last_error_at >= self.error_interval_s:
                self.logger(f"Face unavailable: {error}")
                self.last_error_at = now
            return False
        finally:
            connection.close()
