#!/usr/bin/env bash
set -euo pipefail

name="${1:-competition-$(date +%Y%m%d-%H%M%S)}"
case "$name" in
  *[!A-Za-z0-9._-]*|'')
    echo "map name may contain only letters, numbers, dot, underscore and dash" >&2
    exit 2
    ;;
esac

host_dir=/home/user/rplidar_deploy/slam/maps
container_base="/opt/slam/maps/$name"
mkdir -p "$host_dir"

if ! docker inspect -f '{{.State.Running}}' slam_toolbox 2>/dev/null | grep -qx true; then
  echo "slam_toolbox container is not running" >&2
  exit 1
fi

docker exec slam_toolbox bash -lc \
  "source /opt/ros/humble/setup.bash && ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{filename: '$container_base'}\""
docker exec slam_toolbox bash -lc \
  "source /opt/ros/humble/setup.bash && ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: '$container_base'}}\""

sleep 1
files=("$host_dir/$name.posegraph" "$host_dir/$name.data" "$host_dir/$name.yaml" "$host_dir/$name.pgm")
for file in "${files[@]}"; do
  if [[ ! -s "$file" ]]; then
    echo "missing or empty output: $file" >&2
    exit 1
  fi
done
sha256sum "${files[@]}"
