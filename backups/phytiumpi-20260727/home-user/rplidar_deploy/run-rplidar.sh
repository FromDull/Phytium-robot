#!/usr/bin/env bash
set -euo pipefail

device="${RPLIDAR_DEVICE:-/dev/rplidar}"
domain_id="${ROS_DOMAIN_ID:-10}"

if [[ ! -c "$device" ]]; then
  echo "RPLIDAR serial device not found: $device" >&2
  echo "Connect the CH340 adapter and check: ls -l /dev/rplidar /dev/ttyUSB*" >&2
  exit 1
fi

docker rm -f rplidar >/dev/null 2>&1 || true

exec docker run --rm \
  --name rplidar \
  --network host \
  --device "$device:$device" \
  --env ROS_DOMAIN_ID="$domain_id" \
  --env RMW_FASTRTPS_USE_SHM=0 \
  --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  rplidar-ros2:humble \
  bash -lc "source /opt/ros/humble/setup.bash && source /opt/rplidar_ws/install/setup.bash && exec ros2 run rplidar_ros rplidar_node --ros-args -p channel_type:=serial -p serial_port:=$device -p serial_baudrate:=115200 -p frame_id:=laser -p angle_compensate:=true"
