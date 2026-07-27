#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker rm -f nav2_preview >/dev/null 2>&1 || true

exec docker run --rm \
  --name nav2_preview \
  --network host \
  --ipc host \
  --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}" \
  --volume "$script_dir/nav2-preview.yaml:/opt/nav2-preview/config/nav2-preview.yaml:ro" \
  nav2-preview:humble
