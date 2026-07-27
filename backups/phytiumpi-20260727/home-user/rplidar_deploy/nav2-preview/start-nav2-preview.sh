#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
set -eo pipefail

ros2 run nav2_planner planner_server \
  --ros-args --params-file /opt/nav2-preview/config/nav2-preview.yaml &
planner_pid=$!

ros2 run nav2_lifecycle_manager lifecycle_manager \
  --ros-args --params-file /opt/nav2-preview/config/nav2-preview.yaml \
  -r __node:=lifecycle_manager_navigation &
lifecycle_pid=$!

python3 /opt/nav2-preview/planner_bridge.py &
bridge_pid=$!

cleanup() {
  kill -TERM "$bridge_pid" "$lifecycle_pid" "$planner_pid" 2>/dev/null || true
  wait "$bridge_pid" "$lifecycle_pid" "$planner_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait -n "$planner_pid" "$lifecycle_pid" "$bridge_pid"
