#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="/home/user/rplidar_deploy"
RADAR_LOG="/tmp/rplidar.log"
WEB_LOG="/tmp/lidar-web-viewer.log"
TERMINAL_LOG="/tmp/lidar-web-terminal.log"
TERMINAL_PID_FILE="/tmp/lidar-web-terminal.pid"
TERMINAL_PASSWORD_FILE="/home/user/.config/lidar-web-terminal/password.json"

is_running() {
  docker ps --format '{{.Names}}' | grep -qx "$1"
}

stop_terminal() {
  if [[ -f "$TERMINAL_PID_FILE" ]]; then
    pid="$(cat "$TERMINAL_PID_FILE" 2>/dev/null || true)"
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
    rm -f "$TERMINAL_PID_FILE"
  fi
}

start_stack() {
  if [[ ! -c /dev/rplidar ]]; then
    echo "RPLIDAR device not found: /dev/rplidar" >&2
    echo "Reconnect the radar USB/CH340 and try again." >&2
    exit 2
  fi

  docker rm -f lidar_web_viewer rplidar >/dev/null 2>&1 || true
  stop_terminal

  cd "$BASE_DIR"
  nohup ./run-rplidar.sh >"$RADAR_LOG" 2>&1 &

  for _ in {1..30}; do
    if grep -q "health status : OK" "$RADAR_LOG" 2>/dev/null && is_running rplidar; then
      break
    fi
    sleep 1
  done

  if ! is_running rplidar || ! grep -q "health status : OK" "$RADAR_LOG" 2>/dev/null; then
    echo "RPLIDAR driver failed to start" >&2
    tail -40 "$RADAR_LOG" >&2 || true
    exit 3
  fi

  cd "$BASE_DIR/web-viewer"
  nohup ./run-web-viewer.sh >"$WEB_LOG" 2>&1 &

  for _ in {1..20}; do
    if curl --fail --silent --output /dev/null http://127.0.0.1:8080/api/state; then
      break
    fi
    sleep 1
  done

  if ! curl --fail --silent --output /dev/null http://127.0.0.1:8080/api/state; then
    echo "Web viewer failed to start" >&2
    tail -40 "$WEB_LOG" >&2 || true
    exit 4
  fi

  if [[ -r "$TERMINAL_PASSWORD_FILE" ]]; then
    cd "$BASE_DIR"
    nohup ./run-web-terminal.sh >"$TERMINAL_LOG" 2>&1 &
    terminal_pid=$!
    printf '%s\n' "$terminal_pid" >"$TERMINAL_PID_FILE"
    for _ in {1..10}; do
      if ss -ltn | grep -q ':8765 '; then
        break
      fi
      sleep 1
    done
    if ! kill -0 "$terminal_pid" 2>/dev/null || ! ss -ltn | grep -q ':8765 '; then
      echo "Browser terminal failed to start" >&2
      tail -40 "$TERMINAL_LOG" >&2 || true
      exit 5
    fi
  else
    echo "Browser terminal is not configured. Run: ./set-web-terminal-password.py"
  fi

  echo "LiDAR stack started"
  echo "Browser: http://192.168.43.122:8080"
  echo "Terminal login: Dashboard > Terminal > enter the dashboard password"
  echo "Terminal audit: /home/user/.local/state/lidar-web-viewer/terminal-audit.jsonl"
  echo "Radar log: $RADAR_LOG"
  echo "Web log: $WEB_LOG"
  echo "Terminal log: $TERMINAL_LOG"
}

stop_stack() {
  docker rm -f lidar_web_viewer rplidar >/dev/null 2>&1 || true
  stop_terminal
  echo "LiDAR stack stopped"
}

status_stack() {
  echo "--- containers ---"
  docker ps -a --format '{{.Names}} {{.Status}}' | grep -E '^(rplidar|lidar_web_viewer) ' || echo "not running"
  echo "--- device ---"
  ls -l /dev/rplidar 2>/dev/null || echo "/dev/rplidar not found"
  echo "--- web ---"
  if payload=$(curl --fail --silent http://127.0.0.1:8080/api/state); then
    echo "${payload:0:300}"
  else
    echo "web unavailable"
  fi
  echo "--- terminal ---"
  if [[ -f "$TERMINAL_PID_FILE" ]] && kill -0 "$(cat "$TERMINAL_PID_FILE")" 2>/dev/null; then
    echo "running on ws://127.0.0.1:8765"
  else
    echo "not running"
  fi
}

case "${1:-status}" in
  start) start_stack ;;
  stop) stop_stack ;;
  status) status_stack ;;
  *) echo "Usage: $0 {start|stop|status}" >&2; exit 64 ;;
esac
