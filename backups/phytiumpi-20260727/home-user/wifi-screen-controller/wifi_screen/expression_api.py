import json
import os
import queue
import socket
import stat
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .expression import EXPRESSIONS, ExpressionManager


API_VERSION = 1
MAX_REQUEST_BYTES = 16 * 1024


@dataclass
class QueuedExpressionCommand:
    payload: dict[str, Any]
    response: queue.Queue[dict[str, Any]] = field(
        default_factory=lambda: queue.Queue(maxsize=1)
    )


def execute_expression_command(
    manager: ExpressionManager, payload: dict[str, Any]
) -> dict[str, Any]:
    try:
        if payload.get("version", API_VERSION) != API_VERSION:
            raise ValueError(f"unsupported version; expected {API_VERSION}")
        action = payload.get("action")
        if action == "show":
            request = manager.show(
                payload.get("expression"),
                source=payload.get("source", "external"),
                duration_ms=payload.get("duration_ms", 0),
                priority=payload.get("priority", 60),
                force_page=payload.get("force_page", True),
            )
            return {"ok": True, "request": request, "status": manager.status()}
        if action == "clear":
            removed = manager.clear(payload.get("source"))
            return {"ok": True, "removed": removed, "status": manager.status()}
        if action == "clear_all":
            removed = manager.clear_all()
            return {"ok": True, "removed": removed, "status": manager.status()}
        if action == "set_default":
            manager.set_default(payload.get("expression"))
            return {"ok": True, "status": manager.status()}
        if action == "status":
            return {"ok": True, "status": manager.status()}
        if action == "list":
            return {
                "ok": True,
                "expressions": [
                    {
                        "id": item.state,
                        "name": item.name,
                        "description": item.description,
                    }
                    for item in EXPRESSIONS
                ],
            }
        raise ValueError(
            "action must be show, clear, clear_all, set_default, status, or list"
        )
    except (TypeError, ValueError) as exc:
        return {"ok": False, "error": str(exc)}


class ExpressionSocketServer:
    def __init__(self, path: str, mode: int = 0o666):
        self.path = Path(path)
        self.mode = mode
        self.commands: queue.Queue[QueuedExpressionCommand] = queue.Queue()
        self.listener: socket.socket | None = None
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists() or self.path.is_socket():
            file_mode = self.path.lstat().st_mode
            if not stat.S_ISSOCK(file_mode):
                raise OSError(f"refusing to remove non-socket path: {self.path}")
            self.path.unlink()
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            listener.bind(str(self.path))
            os.chmod(self.path, self.mode)
            listener.listen(8)
            listener.settimeout(0.2)
        except Exception:
            listener.close()
            raise
        self.listener = listener
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self._serve, name="expression-api", daemon=True
        )
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.listener is not None:
            self.listener.close()
            self.listener = None
        if self.thread is not None:
            self.thread.join(timeout=2)
            self.thread = None
        try:
            if self.path.is_socket():
                self.path.unlink()
        except FileNotFoundError:
            pass

    def _serve(self) -> None:
        while not self.stop_event.is_set():
            try:
                assert self.listener is not None
                connection, _ = self.listener.accept()
            except (OSError, socket.timeout):
                continue
            with connection:
                connection.settimeout(3)
                response = self._handle_connection(connection)
                try:
                    connection.sendall(
                        json.dumps(response, ensure_ascii=False).encode("utf-8") + b"\n"
                    )
                except OSError:
                    pass

    def _handle_connection(self, connection: socket.socket) -> dict[str, Any]:
        data = bytearray()
        try:
            while len(data) <= MAX_REQUEST_BYTES:
                chunk = connection.recv(4096)
                if not chunk:
                    break
                data.extend(chunk)
                if b"\n" in chunk:
                    break
            if len(data) > MAX_REQUEST_BYTES:
                return {"ok": False, "error": "request exceeds 16384 bytes"}
            payload = json.loads(bytes(data).split(b"\n", 1)[0].decode("utf-8"))
            if not isinstance(payload, dict):
                return {"ok": False, "error": "request must be a JSON object"}
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            return {"ok": False, "error": f"invalid request: {exc}"}
        command = QueuedExpressionCommand(payload)
        self.commands.put(command)
        try:
            return command.response.get(timeout=3)
        except queue.Empty:
            return {"ok": False, "error": "controller response timeout"}
