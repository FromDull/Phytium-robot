#!/usr/bin/env bash
set -euo pipefail

domain_id="${ROS_DOMAIN_ID:-10}"
port="${WEB_VIEWER_PORT:-8080}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
semantic_data_dir="${SEMANTIC_MAP_DATA_DIR:-$(dirname "$script_dir")/data/semantic-map}"
request_data_dir="${WEB_REQUEST_DATA_DIR:-/run/user/1000/robot-web-requests}"
mkdir -p "$semantic_data_dir" "$request_data_dir"
docker rm -f lidar_web_viewer >/dev/null 2>&1 || true

exec docker run --rm \
  --name lidar_web_viewer \
  --network host \
  --ipc host \
  --volume /run/acoustic-eye:/run/acoustic-eye:ro \
  --volume /run/gimbal-daemon:/run/gimbal-daemon \
  --volume /run/rpmsg-broker:/run/rpmsg-broker \
  --volume /run/wifi-screen:/run/wifi-screen \
  --volume /run/robot-service-control:/run/robot-service-control:ro \
  --volume /home/user/robot_data:/var/lib/robot-data:ro \
  --volume "$request_data_dir:/var/lib/robot-requests" \
  --volume "$semantic_data_dir:/var/lib/semantic-map" \
  --env ROS_DOMAIN_ID="$domain_id" \
  --env WEB_VIEWER_PORT="$port" \
  --env CAMERA_MJPEG_FPS="${CAMERA_MJPEG_FPS:-6}" \
  --env SEMANTIC_MAP_PATH=/var/lib/semantic-map/semantic-map.json \
  --env VOICE_NAVIGATION_PATH=/var/lib/robot-data/voice_navigation_command.json \
  --env NEW_MAP_REQUEST_PATH=/var/lib/robot-requests/new_map_request.json \
  --env NEW_MAP_STATUS_PATH=/var/lib/robot-requests/new_map_status.json \
  --env DEPTH_VOXEL_PERIOD_S="${DEPTH_VOXEL_PERIOD_S:-2.0}" \
  --env DEPTH_VOXEL_PIXEL_STRIDE="${DEPTH_VOXEL_PIXEL_STRIDE:-8}" \
  --env DEPTH_VOXEL_SIZE_M="${DEPTH_VOXEL_SIZE_M:-0.10}" \
  --env DEPTH_VOXEL_MAX_DEPTH_M="${DEPTH_VOXEL_MAX_DEPTH_M:-4.0}" \
  --env DEPTH_VOXEL_MAXIMUM="${DEPTH_VOXEL_MAXIMUM:-10000}" \
  --env DEPTH_VOXEL_MAXIMUM_OUTPUT="${DEPTH_VOXEL_MAXIMUM_OUTPUT:-4000}" \
  lidar-web-viewer:humble
