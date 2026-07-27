#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="/home/user/rplidar_deploy"
PORT="${TERMINAL_PORT:-8765}"
STATE_DIR="${HOME}/.local/state/lidar-web-viewer"
CONFIG_DIR="${HOME}/.config/lidar-web-terminal"
PASSWORD_FILE="${TERMINAL_PASSWORD_FILE:-${CONFIG_DIR}/password.json}"
AUDIT_LOG="${TERMINAL_AUDIT_LOG:-${STATE_DIR}/terminal-audit.jsonl}"

install -d -m 700 "$STATE_DIR"
if [[ ! -r "$PASSWORD_FILE" ]]; then
  echo "Browser terminal password is not configured." >&2
  echo "Run: python3 $BASE_DIR/set-web-terminal-password.py" >&2
  exit 2
fi

echo "Browser terminal password authentication enabled"
echo "Audit log: $AUDIT_LOG"
exec python3 "$BASE_DIR/web-terminal.py" --port "$PORT" --password-file "$PASSWORD_FILE" --audit-log "$AUDIT_LOG"
