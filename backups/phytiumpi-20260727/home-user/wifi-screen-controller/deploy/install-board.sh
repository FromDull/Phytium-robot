#!/bin/sh
set -eu

PROJECT_DIR="/home/user/wifi-screen-controller"
SERVICE_FILE="$PROJECT_DIR/deploy/wifi-screen.service"

if [ "$(id -u)" -ne 0 ]; then
    echo "Run with sudo: sudo ./deploy/install-board.sh" >&2
    exit 1
fi

if [ ! -f "$PROJECT_DIR/main.py" ] || [ ! -f "$SERVICE_FILE" ]; then
    echo "Incomplete project directory: $PROJECT_DIR" >&2
    exit 1
fi

chmod +x "$PROJECT_DIR/facectl.py" "$PROJECT_DIR/deploy/verify-board.sh"
ln -sf "$PROJECT_DIR/facectl.py" /usr/local/bin/facectl
cp "$SERVICE_FILE" /etc/systemd/system/wifi-screen.service

systemctl daemon-reload
systemctl enable wifi-screen.service
systemctl restart wifi-screen.service

"$PROJECT_DIR/deploy/verify-board.sh"
