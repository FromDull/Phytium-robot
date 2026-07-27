#!/usr/bin/env python3
"""Bridge recognized navigation phrases to a file; never publish motion commands."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import time
from pathlib import Path


NAVIGATION_PATTERNS = (
    re.compile(r"(?:请)?(?:导航|规划路线|规划路径|带我|带我们)(?:到|去|前往)\s*(.+)"),
    re.compile(r"(?:请)?前往\s*(.+)"),
    re.compile(r"(?:我的)?目的地(?:是|设为|设置为)\s*(.+)"),
    re.compile(r"(?:我要|我们要)去\s*(.+)"),
)


def extract_destination(text):
    normalized = re.sub(r"\s+", " ", str(text or "")).strip()
    for pattern in NAVIGATION_PATTERNS:
        match = pattern.search(normalized)
        if not match:
            continue
        destination = re.sub(
            r"(?:怎么走|的路线|路线|路径|位置|附近)?[。！!，,？?\s]*$",
            "",
            match.group(1).strip(),
        )
        if destination:
            return destination[:120]
    return None


def atomic_write(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as output:
        json.dump(payload, output, ensure_ascii=False, separators=(",", ":"))
        output.flush()
        os.fsync(output.fileno())
    os.replace(temporary, path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default="/home/user/robot_data/voice_navigation_command.json",
    )
    args = parser.parse_args()
    output_path = Path(args.output)
    command = [
        "journalctl",
        "--user",
        "--unit=robot-ai-voice.service",
        "--follow",
        "--lines=0",
        "--output=cat",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    if process.stdout is None:
        raise RuntimeError("journal stream unavailable")
    for line in process.stdout:
        line = line.strip()
        if not line.startswith("你>"):
            continue
        utterance = line.split(">", 1)[1].strip()
        destination = extract_destination(utterance)
        if destination is None:
            continue
        payload = {
            "sequence": time.time_ns(),
            "timestamp": time.time(),
            "utterance": utterance[:240],
            "destination": destination,
            "preview_only": True,
        }
        atomic_write(output_path, payload)
        print(f"voice preview destination: {destination}", flush=True)
    return process.wait()


if __name__ == "__main__":
    raise SystemExit(main())
