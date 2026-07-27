#!/usr/bin/env python3
"""Export gimbal telemetry for the containerized ROS 2 TF publisher."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import socket
import tempfile
import time


DEFAULT_SOCKET = "/run/gimbal-daemon/gimbal.sock"
DEFAULT_OUTPUT = "/home/user/robot_data/gimbal_tf_state.json"


def request_status(socket_path: str, timeout_s: float) -> dict:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout_s)
        connection.connect(socket_path)
        connection.sendall(b'{"command":"status"}\n')
        raw = bytearray()
        while b"\n" not in raw:
            chunk = connection.recv(4096)
            if not chunk:
                break
            raw.extend(chunk)
            if len(raw) > 65536:
                raise RuntimeError("gimbal status response is too large")
    if not raw:
        raise RuntimeError("empty gimbal status response")
    response = json.loads(bytes(raw).split(b"\n", 1)[0].decode("utf-8"))
    if not response.get("ok"):
        raise RuntimeError(str(response.get("error", "gimbal status failed")))
    return response


def state_from_response(
    response: dict, sequence: int, maximum_feedback_age_ms: int, sampled_at: float
) -> dict:
    telemetry = response.get("telemetry")
    if not isinstance(telemetry, dict):
        raise RuntimeError("gimbal response has no telemetry")
    yaw_age = int(telemetry.get("yaw_feedback_age_ms", 0xFFFFFFFF))
    pitch_age = int(telemetry.get("pitch_feedback_age_ms", 0xFFFFFFFF))
    fault = int(telemetry.get("fault", 0))
    feedback_valid = int(telemetry.get("feedback_valid_mask", 0)) == 0x03
    pose_valid = (
        feedback_valid
        and fault == 0
        and max(yaw_age, pitch_age) <= maximum_feedback_age_ms
    )
    return {
        "sequence": sequence,
        "updated_at": sampled_at,
        "pose_valid": pose_valid,
        "active": int(telemetry.get("state", 0)) == 3,
        "fault": fault,
        "feedback_valid_mask": int(telemetry.get("feedback_valid_mask", 0)),
        "yaw_feedback_age_ms": yaw_age,
        "pitch_feedback_age_ms": pitch_age,
        "yaw_deg": float(telemetry.get("yaw_deg", 0.0)),
        "pitch_deg": float(telemetry.get("pitch_deg", 0.0)),
    }


def atomic_write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as output:
            json.dump(payload, output, separators=(",", ":"))
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", default=DEFAULT_SOCKET)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=0.25)
    parser.add_argument("--maximum-feedback-age-ms", type=int, default=500)
    args = parser.parse_args()
    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    if args.maximum_feedback_age_ms <= 0:
        parser.error("--maximum-feedback-age-ms must be positive")

    output = Path(args.output)
    period = 1.0 / args.rate
    sequence = 0
    while True:
        started = time.monotonic()
        sequence += 1
        sampled_at = time.time()
        try:
            response = request_status(args.socket, args.timeout)
            payload = state_from_response(
                response, sequence, args.maximum_feedback_age_ms, sampled_at
            )
        except Exception as error:
            payload = {
                "sequence": sequence,
                "updated_at": sampled_at,
                "pose_valid": False,
                "active": False,
                "error": str(error)[:160],
            }
        atomic_write(output, payload)
        time.sleep(max(0.01, period - (time.monotonic() - started)))


if __name__ == "__main__":
    raise SystemExit(main())
