#!/usr/bin/env python3
import argparse
import json
import socket
import sys
import time
from typing import Any


DEFAULT_SOCKET = "/run/wifi-screen/face.sock"


def send_request(path: str, payload: dict[str, Any]) -> dict[str, Any]:
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        client.settimeout(5)
        client.connect(path)
        client.sendall(json.dumps(payload).encode("utf-8") + b"\n")
        data = bytearray()
        while b"\n" not in data:
            chunk = client.recv(4096)
            if not chunk:
                break
            data.extend(chunk)
    finally:
        client.close()
    if not data:
        raise RuntimeError("expression service returned no response")
    return json.loads(bytes(data).split(b"\n", 1)[0].decode("utf-8"))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control the robot face expression")
    parser.add_argument("--socket", default=DEFAULT_SOCKET)
    subparsers = parser.add_subparsers(dest="command", required=True)

    show = subparsers.add_parser("show", help="show an expression")
    show.add_argument("expression")
    show.add_argument("--duration", type=float, default=0, help="seconds; 0 persists")
    show.add_argument("--priority", type=int, default=60)
    show.add_argument("--source", default="facectl")
    show.add_argument("--no-page", action="store_true")

    clear = subparsers.add_parser("clear", help="clear one source request")
    clear.add_argument("source", nargs="?", default="facectl")
    subparsers.add_parser("clear-all", help="clear every active request")

    default = subparsers.add_parser("set-default", help="set the idle expression")
    default.add_argument("expression")
    subparsers.add_parser("status", help="show current expression state")
    subparsers.add_parser("list", help="list valid expressions")

    demo = subparsers.add_parser("demo", help="play all expressions")
    demo.add_argument("--seconds", type=float, default=2)
    demo.add_argument("--priority", type=int, default=60)
    demo.add_argument("--source", default="facectl.demo")
    return parser


def request_for_args(args: argparse.Namespace) -> dict[str, Any]:
    if args.command == "show":
        if args.duration < 0:
            raise ValueError("duration must be non-negative")
        return {
            "version": 1,
            "action": "show",
            "expression": args.expression,
            "duration_ms": round(args.duration * 1000),
            "priority": args.priority,
            "source": args.source,
            "force_page": not args.no_page,
        }
    if args.command == "clear":
        return {"version": 1, "action": "clear", "source": args.source}
    if args.command == "clear-all":
        return {"version": 1, "action": "clear_all"}
    if args.command == "set-default":
        return {
            "version": 1,
            "action": "set_default",
            "expression": args.expression,
        }
    return {"version": 1, "action": args.command}


def run_demo(args: argparse.Namespace) -> dict[str, Any]:
    if args.seconds <= 0:
        raise ValueError("seconds must be greater than zero")
    listed = send_request(args.socket, {"version": 1, "action": "list"})
    if not listed.get("ok"):
        return listed
    last: dict[str, Any] = listed
    for expression in listed["expressions"]:
        last = send_request(
            args.socket,
            {
                "version": 1,
                "action": "show",
                "expression": expression["name"],
                "duration_ms": round(args.seconds * 1000),
                "priority": args.priority,
                "source": args.source,
                "force_page": True,
            },
        )
        if not last.get("ok"):
            return last
        print(f'{expression["id"]}: {expression["name"]}')
        time.sleep(args.seconds)
    return send_request(
        args.socket,
        {"version": 1, "action": "clear", "source": args.source},
    )


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        response = run_demo(args) if args.command == "demo" else send_request(
            args.socket, request_for_args(args)
        )
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f"facectl: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(response, ensure_ascii=False, indent=2))
    return 0 if response.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
