#!/usr/bin/env python3
"""Minimal host-side systemd control socket for the robot dashboard."""

import json
import os
import socket
import socketserver
import struct
import subprocess
import time


SOCKET_PATH = os.environ.get(
    "ROBOT_SERVICE_CONTROL_SOCKET", "/run/robot-service-control/control.sock"
)

SERVICES = {
    "astra-ros2.service": ("vision", "RGB-D 相机", "Astra ROS 2 彩色与深度节点"),
    "yolo-detector.service": ("vision", "YOLO 识别", "YOLO26 ONNX 推理服务"),
    "target-localizer.service": ("vision", "目标三维定位", "检测框、对齐深度与 TF 融合"),
    "rplidar.service": ("navigation", "激光雷达", "RPLIDAR ROS 2 驱动"),
    "lidar-odometry.service": ("navigation", "雷达里程计", "扫描匹配里程计"),
    "slam-toolbox.service": ("navigation", "SLAM Toolbox", "在线建图节点"),
    "rpmsg-broker.service": ("hardware", "RPMsg Broker", "从核通信独占代理"),
    "gimbal-daemon.service": ("hardware", "云台控制", "双轴云台控制与保护状态机"),
    "gimbal-tf-state.service": ("hardware", "云台 TF 状态", "云台角度到 ROS TF 的状态桥"),
    "gimbal-camera-tf.service": ("hardware", "相机动态 TF", "云台安装相机动态坐标变换"),
    "acoustic-eye-angle.service": ("hardware", "声源方向", "AcousticEye I²C 方位读取"),
    "acoustic-eye-aht20.service": ("hardware", "温湿度", "AHT20 I²C 环境读取"),
    "wifi-screen.service": ("hardware", "串口屏", "Wi-Fi 与串口表情屏控制"),
    "lidar-web-viewer.service": ("system", "网页控制台", "本诊断与控制页面"),
}


def systemd_status():
    properties = ["Id", "ActiveState", "SubState", "MainPID", "NRestarts"]
    command = ["systemctl", "show", *SERVICES]
    for item in properties:
        command.extend(("-p", item))
    completed = subprocess.run(command, capture_output=True, text=True, timeout=5, check=False)
    blocks = [block for block in completed.stdout.strip().split("\n\n") if block.strip()]
    states = {}
    for block in blocks:
        values = dict(line.split("=", 1) for line in block.splitlines() if "=" in line)
        unit = values.get("Id")
        if unit:
            states[unit] = values
    result = []
    for unit, (category, label, description) in SERVICES.items():
        values = states.get(unit, {})
        result.append(
            {
                "unit": unit,
                "category": category,
                "label": label,
                "description": description,
                "active_state": values.get("ActiveState", "unknown"),
                "sub_state": values.get("SubState", "unknown"),
                "main_pid": int(values.get("MainPID", "0") or 0),
                "restarts": int(values.get("NRestarts", "0") or 0),
            }
        )
    return result


def handle_request(request):
    action = str(request.get("action", ""))
    if action == "status":
        return {"ok": True, "services": systemd_status(), "updated_at": time.time()}
    if action != "restart":
        raise ValueError("unsupported action")
    unit = str(request.get("service", ""))
    if unit not in SERVICES:
        raise ValueError("service is not in the dashboard allowlist")
    if request.get("confirm") is not True:
        raise ValueError("restart confirmation is required")
    completed = subprocess.run(
        ["systemctl", "--no-block", "restart", unit],
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )
    if completed.returncode:
        raise RuntimeError((completed.stderr or completed.stdout or "restart failed").strip())
    print(f"dashboard requested restart: {unit}", flush=True)
    return {"ok": True, "service": unit, "message": "restart queued"}


class RequestHandler(socketserver.StreamRequestHandler):
    def handle(self):
        peer = self.request.getsockopt(
            socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i")
        )
        _pid, uid, _gid = struct.unpack("3i", peer)
        if uid != 0:
            self.wfile.write(b'{"ok":false,"error":"unauthorized peer"}\n')
            return
        raw = self.rfile.readline(8193)
        if not raw or len(raw) > 8192:
            self.wfile.write(b'{"ok":false,"error":"invalid request length"}\n')
            return
        try:
            request = json.loads(raw.decode("utf-8"))
            if not isinstance(request, dict):
                raise ValueError("request must be a JSON object")
            response = handle_request(request)
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            response = {"ok": False, "error": str(error)}
        self.wfile.write(json.dumps(response, separators=(",", ":")).encode() + b"\n")


class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True


def main():
    try:
        os.unlink(SOCKET_PATH)
    except FileNotFoundError:
        pass
    server = Server(SOCKET_PATH, RequestHandler)
    os.chmod(SOCKET_PATH, 0o660)
    print(f"robot service control listening on {SOCKET_PATH}", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        try:
            os.unlink(SOCKET_PATH)
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    main()
