#!/usr/bin/env python3
"""Create or replace the password hash used by the browser terminal."""

import argparse
import getpass
import hashlib
import hmac
import json
import os
from pathlib import Path

ITERATIONS = 310_000


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--password-file",
        default=os.path.expanduser("~/.config/lidar-web-terminal/password.json"),
    )
    args = parser.parse_args()
    first = getpass.getpass("设置网页终端密码（至少 12 位）：")
    second = getpass.getpass("再次输入网页终端密码：")
    if not 12 <= len(first) <= 256:
        raise SystemExit("密码长度必须为 12 到 256 个字符")
    if not hmac.compare_digest(first, second):
        raise SystemExit("两次密码不一致，未做任何修改")
    salt = os.urandom(16)
    record = {
        "version": 1,
        "algorithm": "pbkdf2_sha256",
        "iterations": ITERATIONS,
        "salt": salt.hex(),
        "digest": hashlib.pbkdf2_hmac("sha256", first.encode("utf-8"), salt, ITERATIONS).hex(),
    }
    output = Path(args.password_file)
    output.parent.mkdir(parents=True, exist_ok=True)
    os.chmod(output.parent, 0o700)
    temporary = output.with_suffix(".tmp")
    with open(temporary, "w", encoding="utf-8") as file:
        os.chmod(temporary, 0o600)
        json.dump(record, file, separators=(",", ":"))
        file.write("\n")
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, output)
    print(f"密码已保存为哈希：{output}")
    print("请重新运行 lidar-stack.sh start 以启用新密码。")


if __name__ == "__main__":
    main()
