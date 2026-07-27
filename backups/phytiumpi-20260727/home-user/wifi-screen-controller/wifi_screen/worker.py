import queue
import threading
from dataclasses import dataclass
from typing import Any

@dataclass(frozen=True)
class WorkerResult:
    operation: str
    value: Any = None
    error: str = ""


class TaskWorker:
    def __init__(self, manager: Any, name: str = "task-worker"):
        self.manager = manager
        self.tasks: queue.Queue[tuple[str, tuple[Any, ...]] | None] = queue.Queue()
        self.results: queue.Queue[WorkerResult] = queue.Queue()
        self.thread = threading.Thread(target=self._run, name=name, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.tasks.put(None)
        self.thread.join(timeout=2)

    def submit(self, operation: str, *args: Any) -> None:
        self.tasks.put((operation, args))

    def _run(self) -> None:
        while True:
            task = self.tasks.get()
            if task is None:
                return
            operation, args = task
            try:
                method = getattr(self.manager, operation)
                self.results.put(WorkerResult(operation, method(*args)))
            except Exception as exc:
                self.results.put(WorkerResult(operation, error=str(exc)))


WifiWorker = TaskWorker
