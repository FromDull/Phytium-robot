#!/bin/sh

set -e

DEV="virtio0.rpmsg-openamp-demo-channel.-1.0"
BASE="/sys/bus/rpmsg/devices/$DEV"

i=0
while [ ! -d "$BASE" ] && [ "$i" -lt 50 ]; do
    sleep 0.1
    i=$((i + 1))
done

if [ ! -d "$BASE" ]; then
    echo "rpmsg device not found: $BASE"
    echo "current rpmsg devices:"
    ls /sys/bus/rpmsg/devices 2>/dev/null || true
    dmesg | tail -n 40
    exit 1
fi

echo rpmsg_chrdev | sudo tee "$BASE/driver_override"
sudo modprobe rpmsg_char

if [ -e "$BASE/driver/unbind" ]; then
    echo "$DEV" | sudo tee "$BASE/driver/unbind" >/dev/null 2>&1 || true
fi

echo "$DEV" | sudo tee /sys/bus/rpmsg/drivers/rpmsg_chrdev/bind >/dev/null 2>&1 || true

sleep 0.2
ls /dev/rpmsg* 2>/dev/null || true

