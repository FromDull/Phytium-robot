#!/usr/bin/env python3
"""Execute only fresh, explicitly confirmed new-map requests from the web UI."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from pathlib import Path


NEW_SITE_SCRIPT = "/home/user/rplidar_deploy/slam/prepare-new-site.sh"


def atomic_write(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as output:
        json.dump(payload, output, ensure_ascii=False, separators=(",", ":"))
        output.flush()
        os.fsync(output.fileno())
    os.replace(temporary, path)


def process_request(processing_path, status_path, runner=subprocess.run, clock=time.time):
    try:
        request = json.loads(processing_path.read_text(encoding="utf-8"))
        if not isinstance(request, dict):
            raise ValueError("request is not an object")
        age = clock() - float(request.get("timestamp", 0.0))
        if age < -2.0 or age > 15.0:
            raise ValueError("request is stale")
        if (
            request.get("action") != "new_site"
            or request.get("confirm") is not True
            or request.get("preview_only") is not True
        ):
            raise ValueError("request confirmation is invalid")
        sequence = request.get("sequence")
        atomic_write(
            status_path,
            {
                "state": "running",
                "sequence": sequence,
                "timestamp": clock(),
                "preview_only": True,
                "message": "正在备份旧场地并启动新地图",
            },
        )
        completed = runner(
            [NEW_SITE_SCRIPT],
            capture_output=True,
            text=True,
            timeout=180,
            check=False,
        )
        if completed.returncode:
            raise RuntimeError(
                (completed.stderr or completed.stdout or "new-map script failed").strip()
            )
        atomic_write(
            status_path,
            {
                "state": "completed",
                "sequence": sequence,
                "timestamp": clock(),
                "preview_only": True,
                "message": "新地图已启动",
            },
        )
        return True
    except (OSError, ValueError, TypeError, RuntimeError, subprocess.SubprocessError) as error:
        atomic_write(
            status_path,
            {
                "state": "failed",
                "timestamp": clock(),
                "preview_only": True,
                "message": str(error)[:300],
            },
        )
        return False
    finally:
        try:
            processing_path.unlink()
        except FileNotFoundError:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--request",
        default="/home/user/robot_data/web-requests/new_map_request.json",
    )
    parser.add_argument(
        "--status",
        default="/home/user/robot_data/web-requests/new_map_status.json",
    )
    args = parser.parse_args()
    request_path = Path(args.request)
    status_path = Path(args.status)
    processing_path = request_path.with_suffix(request_path.suffix + ".processing")
    request_path.parent.mkdir(parents=True, exist_ok=True)
    while True:
        if not request_path.exists():
            time.sleep(0.25)
            continue
        try:
            os.replace(request_path, processing_path)
        except FileNotFoundError:
            continue
        process_request(processing_path, status_path)


if __name__ == "__main__":
    main()
