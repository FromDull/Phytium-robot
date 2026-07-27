#!/usr/bin/env python3
"""Password-protected WebSocket bridge to a normal-user Bash PTY."""

import argparse
import base64
import hashlib
import hmac
import json
import os
import pty
import select
import socket
import threading
import time
from pathlib import Path
from urllib.parse import urlparse

MAX_FRAME_SIZE = 1024 * 1024


class PasswordVerifier:
    def __init__(self, password_file):
        try:
            record = json.loads(Path(password_file).read_text(encoding="utf-8"))
            self.iterations = int(record["iterations"])
            self.salt = bytes.fromhex(record["salt"])
            self.digest = bytes.fromhex(record["digest"])
        except (OSError, ValueError, KeyError, TypeError) as error:
            raise SystemExit(f"cannot load terminal password configuration: {error}") from error
        if not 100_000 <= self.iterations <= 10_000_000 or len(self.salt) < 16 or len(self.digest) != 32:
            raise SystemExit("terminal password configuration is invalid")

    def verify(self, password):
        if not isinstance(password, str) or not 12 <= len(password) <= 256:
            return False
        candidate = hashlib.pbkdf2_hmac(
            "sha256", password.encode("utf-8"), self.salt, self.iterations
        )
        return hmac.compare_digest(candidate, self.digest)


class TerminalServer:
    def __init__(self, port, verifier, audit_log):
        self.port = port
        self.verifier = verifier
        self.audit_log = Path(audit_log)
        self.session_lock = threading.Lock()
        self.session_active = False

    def audit(self, event, peer, **extra):
        record = {"time": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "event": event, "peer": peer, **extra}
        try:
            self.audit_log.parent.mkdir(parents=True, exist_ok=True)
            with self.audit_log.open("a", encoding="utf-8") as output:
                output.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError:
            pass

    @staticmethod
    def receive_exact(client, length):
        data = b""
        while len(data) < length:
            chunk = client.recv(length - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    @classmethod
    def receive_frame(cls, client):
        header = cls.receive_exact(client, 2)
        if header is None:
            return None, b""
        opcode, length = header[0] & 15, header[1] & 127
        masked = header[1] & 128
        if length == 126:
            encoded = cls.receive_exact(client, 2)
            if encoded is None:
                return None, b""
            length = int.from_bytes(encoded, "big")
        elif length == 127:
            encoded = cls.receive_exact(client, 8)
            if encoded is None:
                return None, b""
            length = int.from_bytes(encoded, "big")
        if length > MAX_FRAME_SIZE:
            return None, b""
        mask = cls.receive_exact(client, 4) if masked else None
        payload = cls.receive_exact(client, length)
        if payload is None:
            return None, b""
        if mask:
            payload = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
        return opcode, payload

    @staticmethod
    def send_frame(client, opcode, payload=b""):
        if isinstance(payload, str):
            payload = payload.encode("utf-8")
        length = len(payload)
        if length < 126:
            header = bytes([128 | opcode, length])
        elif length < 65536:
            header = bytes([128 | opcode, 126]) + length.to_bytes(2, "big")
        else:
            header = bytes([128 | opcode, 127]) + length.to_bytes(8, "big")
        client.sendall(header + payload)

    @classmethod
    def send_control(cls, client, **payload):
        cls.send_frame(client, 1, json.dumps(payload, separators=(",", ":")))

    @staticmethod
    def http_error(client, status):
        body = b"terminal connection rejected\n"
        client.sendall(
            f"HTTP/1.1 {status}\r\nContent-Type: text/plain\r\nContent-Length: {len(body)}\r\nConnection: close\r\n\r\n".encode() + body
        )

    def handshake(self, client):
        request = b""
        while b"\r\n\r\n" not in request and len(request) < 16384:
            chunk = client.recv(4096)
            if not chunk:
                return False
            request += chunk
        try:
            lines = request.decode("iso-8859-1").split("\r\n")
            method = lines[0].split()[0]
            headers = dict(line.split(":", 1) for line in lines[1:] if ":" in line)
            headers = {key.lower().strip(): value.strip() for key, value in headers.items()}
            origin_host = urlparse(headers.get("origin", "")).hostname
            host = headers.get("host", "").split(":")[0]
        except (IndexError, ValueError, UnicodeDecodeError):
            return False
        if method != "GET" or headers.get("upgrade", "").lower() != "websocket" or not headers.get("sec-websocket-key"):
            return False
        if origin_host and host and origin_host != host:
            return False
        accept = base64.b64encode(hashlib.sha1((headers["sec-websocket-key"] + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
        client.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n" f"Sec-WebSocket-Accept: {accept}\r\n\r\n").encode())
        return True

    def authenticate(self, client, peer):
        client.settimeout(20)
        try:
            opcode, payload = self.receive_frame(client)
        except OSError:
            return False
        finally:
            client.settimeout(None)
        if opcode != 1:
            self.send_control(client, type="auth", ok=False, error="请先输入密码")
            return False
        try:
            message = json.loads(payload.decode("utf-8"))
            password = message["password"] if message.get("type") == "auth" else None
        except (UnicodeDecodeError, json.JSONDecodeError, AttributeError, KeyError):
            password = None
        if not self.verifier.verify(password):
            self.audit("authentication_failed", peer)
            self.send_control(client, type="auth", ok=False, error="密码错误")
            return False
        with self.session_lock:
            if self.session_active:
                self.send_control(client, type="auth", ok=False, error="已有终端会话，请先断开")
                return False
            self.session_active = True
        self.send_control(client, type="auth", ok=True)
        return True

    def release_session(self):
        with self.session_lock:
            self.session_active = False

    def run_shell(self, client, peer):
        started = time.monotonic()
        pid, terminal_fd = pty.fork()
        if pid == 0:
            env = os.environ.copy()
            env.update({"TERM": "xterm-256color", "COLORTERM": "truecolor", "LANG": "C.UTF-8"})
            os.execvpe("/bin/bash", ["bash", "--login"], env)
        self.audit("session_started", peer, pid=pid)
        try:
            while True:
                readable, _, _ = select.select([client, terminal_fd], [], [], 0.5)
                if terminal_fd in readable:
                    try:
                        output = os.read(terminal_fd, 8192)
                    except OSError:
                        break
                    if not output:
                        break
                    self.send_frame(client, 1, output.decode("utf-8", errors="replace"))
                if client in readable:
                    opcode, payload = self.receive_frame(client)
                    if opcode is None or opcode == 8:
                        break
                    if opcode == 9:
                        self.send_frame(client, 10, payload)
                    elif opcode == 1:
                        os.write(terminal_fd, payload)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            try:
                os.close(terminal_fd)
            except OSError:
                pass
            try:
                os.kill(pid, 1)
            except ProcessLookupError:
                pass
            try:
                _, status = os.waitpid(pid, 0)
            except ChildProcessError:
                status = 0
            self.audit("session_ended", peer, pid=pid, seconds=round(time.monotonic() - started, 1), status=status)
            self.release_session()

    def handle(self, client, peer):
        with client:
            if not self.handshake(client):
                self.audit("rejected", peer)
                self.http_error(client, "403 Forbidden")
                return
            if self.authenticate(client, peer):
                self.run_shell(client, peer)

    def serve(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("0.0.0.0", self.port))
            listener.listen(8)
            print(f"Browser terminal listening on ws://0.0.0.0:{self.port}", flush=True)
            while True:
                client, address = listener.accept()
                threading.Thread(target=self.handle, args=(client, address[0]), daemon=True).start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--password-file", required=True)
    parser.add_argument("--audit-log", required=True)
    args = parser.parse_args()
    TerminalServer(args.port, PasswordVerifier(args.password_file), args.audit_log).serve()


if __name__ == "__main__":
    main()
