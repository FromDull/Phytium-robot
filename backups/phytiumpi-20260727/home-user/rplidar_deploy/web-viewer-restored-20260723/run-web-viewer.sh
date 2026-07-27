#!/usr/bin/env bash
set -euo pipefail

domain_id="${ROS_DOMAIN_ID:-10}"
port="${WEB_VIEWER_PORT:-8080}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
semantic_data_dir="${SEMANTIC_MAP_DATA_DIR:-$(dirname "$script_dir")/data/semantic-map}"
mkdir -p "$semantic_data_dir"
docker rm -f lidar_web_viewer >/dev/null 2>&1 || true

exec docker run --rm \
  --name lidar_web_viewer \
  --network host \
  --volume /run/acoustic-eye:/run/acoustic-eye:ro \
  --volume /run/gimbal-daemon:/run/gimbal-daemon \
  --volume /run/rpmsg-broker:/run/rpmsg-broker \
  --volume /run/wifi-screen:/run/wifi-screen \
  --volume /run/robot-service-control:/run/robot-service-control:ro \
  --volume "$semantic_data_dir:/var/lib/semantic-map" \
  --env ROS_DOMAIN_ID="$domain_id" \
  --env RMW_FASTRTPS_USE_SHM=0 \
  --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --env WEB_VIEWER_PORT="$port" \
  --env SEMANTIC_MAP_PATH=/var/lib/semantic-map/semantic-map.json \
  lidar-web-viewer:restored-20260723
