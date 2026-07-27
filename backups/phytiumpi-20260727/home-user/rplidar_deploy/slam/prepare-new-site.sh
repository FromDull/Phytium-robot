#!/usr/bin/env bash
set -euo pipefail

stamp="$(date +%Y%m%d-%H%M%S)"
archive_dir="/home/user/rplidar_deploy/backups/site-change-$stamp"
semantic_file="/home/user/rplidar_deploy/data/semantic-map/semantic-map.json"
mkdir -p "$archive_dir"

echo "[1/5] 保存当前地图和位姿图"
/home/user/rplidar_deploy/slam/save-slam-map.sh "before-site-change-$stamp"

echo "[2/5] 备份当前语义地图"
if [[ -f "$semantic_file" ]]; then
  cp "$semantic_file" "$archive_dir/semantic-map.json"
fi

echo "[3/5] 清除网页路径和旧场地语义地标"
curl -fsS -X POST http://127.0.0.1:8080/api/navigation/clear >/dev/null
curl -fsS -X POST -H 'Content-Type: application/json' \
  -d '{"confirm":true,"new_site":true}' http://127.0.0.1:8080/api/semantic-map/clear >/dev/null

echo "[4/5] 重启 SLAM 容器，从空白地图开始新场地建图"
docker stop --time 10 slam_toolbox >/dev/null
for _ in $(seq 1 40); do
  if docker inspect -f '{{.State.Running}}' slam_toolbox 2>/dev/null | grep -qx true; then
    break
  fi
  sleep 1
done
if ! docker inspect -f '{{.State.Running}}' slam_toolbox 2>/dev/null | grep -qx true; then
  echo "SLAM container did not recover" >&2
  exit 1
fi

echo "[5/5] 等待新地图"
for _ in $(seq 1 30); do
  if curl -fsS http://127.0.0.1:8080/api/state | grep -q '"online":true'; then
    echo "新场地模式已就绪；旧数据备份：$archive_dir"
    exit 0
  fi
  sleep 1
done
echo "SLAM 已重启，但网页尚未收到新地图，请检查服务状态" >&2
exit 1
