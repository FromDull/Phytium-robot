#!/usr/bin/env bash
set -euo pipefail

domain_id="${ROS_DOMAIN_ID:-10}"
port="${WEB_VIEWER_PORT:-8080}"
docker rm -f lidar_web_viewer >/dev/null 2>&1 || true

exec docker run --rm \
  --name lidar_web_viewer \
  --network host \
  --env ROS_DOMAIN_ID="$domain_id" \
  --env RMW_FASTRTPS_USE_SHM=0 \
  --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --env WEB_VIEWER_PORT="$port" \
  lidar-web-viewer:humble
