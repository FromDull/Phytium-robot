#!/usr/bin/env bash
set -euo pipefail

domain_id="${ROS_DOMAIN_ID:-10}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker rm -f lidar_odom >/dev/null 2>&1 || true

exec docker run --rm \
  --name lidar_odom \
  --network host \
  --ipc host \
  --env ROS_DOMAIN_ID="$domain_id" \
  --env RMW_FASTRTPS_USE_SHM=0 \
  --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --env ROS_LOG_DIR=/tmp/ros-log \
  --volume "$script_dir:/opt/lidar_odometry:ro" \
  --volume /run/rpmsg-broker:/run/rpmsg-broker:ro \
  rplidar-ros2:humble \
  bash -lc 'source /opt/ros/humble/setup.bash && cd /opt/lidar_odometry && exec python3 lidar_odometry_node.py --ros-args --params-file lidar-odometry-config.yaml'
