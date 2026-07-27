#!/bin/sh

set -eu

FIRMWARE_SOURCE="/home/user/openamp_core0.elf"
FIRMWARE_TARGET="/lib/firmware/openamp_core0.elf"
REMOTEPROC="/sys/class/remoteproc/remoteproc0"
CAN_DRIVER="/sys/bus/platform/drivers/phytium_can_platform"
RPMSG_BINDER="/home/user/openamp_rpmsg_comm/bind_rpmsg.sh"

release_linux_can() {
    for interface in can0 can1; do
        ip link set "$interface" down 2>/dev/null || true
    done

    for device in 2800a000.can 2800b000.can; do
        if [ -L "/sys/bus/platform/devices/$device/driver" ] &&
           [ -e "$CAN_DRIVER/unbind" ]; then
            echo "$device" > "$CAN_DRIVER/unbind"
        fi
    done
}

wait_for_remoteproc_state() {
    expected="$1"
    attempts=0

    while [ "$attempts" -lt 50 ]; do
        if [ "$(cat "$REMOTEPROC/state" 2>/dev/null || true)" = "$expected" ]; then
            return 0
        fi
        sleep 0.1
        attempts=$((attempts + 1))
    done

    echo "remoteproc0 did not reach state: $expected" >&2
    return 1
}

[ -r "$FIRMWARE_SOURCE" ] || {
    echo "firmware not found: $FIRMWARE_SOURCE" >&2
    exit 1
}

[ -e "$REMOTEPROC/state" ] || {
    echo "remoteproc0 is not available" >&2
    exit 1
}

[ -x "$RPMSG_BINDER" ] || {
    echo "RPMsg binding helper is missing or not executable: $RPMSG_BINDER" >&2
    exit 1
}

release_linux_can

if [ "$(cat "$REMOTEPROC/state")" = "running" ]; then
    echo stop > "$REMOTEPROC/state"
    wait_for_remoteproc_state offline
fi

install -m 0644 "$FIRMWARE_SOURCE" "$FIRMWARE_TARGET"
sync

echo start > "$REMOTEPROC/state"
wait_for_remoteproc_state running

"$RPMSG_BINDER"

[ -e /dev/rpmsg0 ] || {
    echo "/dev/rpmsg0 was not created" >&2
    exit 1
}

echo "OpenAMP remote core is running and /dev/rpmsg0 is ready"
