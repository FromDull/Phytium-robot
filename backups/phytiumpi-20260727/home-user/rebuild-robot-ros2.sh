#!/usr/bin/env bash
set -euo pipefail

cd /root/ros2_ws
stamp="$(date +%Y%m%d-%H%M%S)"
backup_dir="/root/ros2_ws/backups/pre-full-build-${stamp}"
mkdir -p "$backup_dir"

for directory in build install log; do
    if [[ -e "$directory" ]]; then
        mv "$directory" "$backup_dir/$directory"
    fi
done

set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --executor sequential --symlink-install --cmake-clean-cache --event-handlers console_direct+

set +u
source /root/ros2_ws/install/setup.bash
set -u
ros2 pkg prefix astra_camera_ros2
ros2 pkg prefix gimbal_camera_tf_ros2
ros2 pkg prefix target_localization_ros2
