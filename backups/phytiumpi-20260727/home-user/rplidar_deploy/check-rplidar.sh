#!/usr/bin/env bash
set -euo pipefail

echo "Docker: $(docker --version)"
echo "Architecture: $(uname -m)"
docker image inspect rplidar-ros2:humble --format 'Image: {{.RepoTags}} / {{.Architecture}}'
docker run --rm rplidar-ros2:humble bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/rplidar_ws/install/setup.bash && ros2 pkg prefix rplidar_ros && ros2 pkg executables rplidar_ros'

if [[ -c /dev/rplidar ]]; then
  echo "Serial device: $(readlink -f /dev/rplidar)"
else
  echo "Serial device: not connected (expected /dev/rplidar)"
fi
