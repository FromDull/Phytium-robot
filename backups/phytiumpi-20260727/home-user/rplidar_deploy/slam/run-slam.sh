#!/usr/bin/env bash
set -euo pipefail

domain_id="${ROS_DOMAIN_ID:-10}"
map_dir=/home/user/rplidar_deploy/slam/maps
mkdir -p "$map_dir"
docker rm -f slam_toolbox >/dev/null 2>&1 || true

exec docker run --rm \
  --name slam_toolbox \
  --network host \
  --ipc host \
  --env ROS_DOMAIN_ID="$domain_id" \
  --env ROS_LOG_DIR=/tmp/ros-log \
  --volume /home/user/rplidar_deploy/slam/slam_toolbox.yaml:/opt/slam/config/slam_toolbox.yaml:ro \
  --volume "$map_dir":/opt/slam/maps:rw \
  slam-toolbox:humble \
  bash -lc 'source /opt/ros/humble/setup.bash && exec ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/opt/slam/config/slam_toolbox.yaml'
