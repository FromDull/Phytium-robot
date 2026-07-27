#!/bin/sh
set -eu

PROJECT_DIR="/home/user/wifi-screen-controller"
REQUIRED_FILES="
main.py
facectl.py
config.json
wifi_screen/__init__.py
wifi_screen/bluetooth.py
wifi_screen/config.py
wifi_screen/controller.py
wifi_screen/expression.py
wifi_screen/expression_api.py
wifi_screen/network.py
wifi_screen/protocol.py
wifi_screen/screen.py
wifi_screen/serial_port.py
wifi_screen/system_status.py
wifi_screen/worker.py
"

for relative_path in $REQUIRED_FILES; do
    if [ ! -f "$PROJECT_DIR/$relative_path" ]; then
        echo "MISSING: $PROJECT_DIR/$relative_path" >&2
        exit 1
    fi
done

cd "$PROJECT_DIR"
python3 -m py_compile main.py facectl.py wifi_screen/*.py

test -c /dev/ttyAMA2
systemctl is-enabled wifi-screen.service
systemctl is-active wifi-screen.service
test -S /run/wifi-screen/face.sock

if ! journalctl -u wifi-screen.service -n 100 --no-pager | grep -q "serial OK: /dev/ttyAMA2 at 115200 baud"; then
    echo "The service log does not confirm that UART2 opened successfully." >&2
    journalctl -u wifi-screen.service -n 30 --no-pager >&2
    exit 1
fi

echo "Serial-screen controller verification passed."
