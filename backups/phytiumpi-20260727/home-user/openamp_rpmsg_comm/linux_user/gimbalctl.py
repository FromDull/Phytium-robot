#!/usr/bin/env python3
"""Command-line client for gimbal_daemon.py."""

from __future__ import annotations

import argparse
import json
import socket


DEFAULT_SOCKET = "/run/gimbal-daemon/gimbal.sock"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", default=DEFAULT_SOCKET)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("status")
    commands.add_parser("estop")
    commands.add_parser("center")
    for name in ("enable", "disable"):
        sub = commands.add_parser(name)
        sub.add_argument("--confirm", action="store_true", required=True)
    move = commands.add_parser("set")
    move.add_argument("yaw_deg", type=float)
    move.add_argument("pitch_deg", type=float)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    request = {"command": args.command}
    if args.command in {"enable", "disable"}:
        request["confirm"] = args.confirm
    elif args.command == "set":
        request.update(yaw_deg=args.yaw_deg, pitch_deg=args.pitch_deg)

    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.connect(args.socket)
        connection.sendall((json.dumps(request) + "\n").encode("utf-8"))
        response = json.loads(connection.recv(65536).decode("utf-8"))
    print(json.dumps(response, ensure_ascii=False, indent=2))
    return 0 if response.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
