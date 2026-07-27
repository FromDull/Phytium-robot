# RPLIDAR A2M6 on Phytium Pi

## Unified commands from Windows

From `C:\Users\34403\Desktop\phytium`:

```powershell
.\rplidar-deploy\lidar-stack.ps1 start
.\rplidar-deploy\lidar-stack.ps1 status
.\rplidar-deploy\lidar-stack.ps1 stop
```

`start` starts the radar driver, waits for the health check, starts the browser viewer, and opens the browser automatically. The remote helper is `/home/user/rplidar_deploy/lidar-stack.sh`.

## Dashboard v2

Open `http://192.168.43.122:8080` after starting the stack. The dashboard provides:

- Overview: CPU, memory, temperature, uptime, network IP, radar rate, and a trend chart.
- Radar: live 2D scan, adjustable range, local odometry path with grid, heading, speed, and reset-origin controls.
- Diagnostics: stream status, scan latency, ICP quality, and recent events.

The browser receives live updates through server-sent events at `/api/stream`; `/api/state` remains available for integration and `/api/reset-odom` resets the local odometry origin.
The odometry is scan-matching based and therefore relative: use the reset button
before each test, and treat long paths as an estimate that can drift without
wheel encoders, IMU, or map-based localization.

## Browser terminal

First set a dedicated dashboard password from Windows:

```powershell
.\rplidar-deploy\lidar-stack.ps1 set-password
```

The password is saved on the board only as a PBKDF2-SHA256 hash. In the
dashboard, open **终端**, enter that password, and click **连接终端**. It opens
one interactive Bash session as the regular `user` account; it does not grant
root access. Session metadata is recorded at:

```bash
~/.local/state/lidar-web-viewer/terminal-audit.jsonl
```

The terminal WebSocket uses port `8765`, so keep the dashboard and the board on
the same trusted LAN. One active browser-terminal session is allowed at a time.
Use HTTPS before exposing this feature outside a trusted LAN.

This deployment runs ROS 2 Humble and `rplidar_ros` in an ARM64 Docker image.

## Run

Connect the powered CH340 adapter, then check the stable device link:

```bash
ls -l /dev/rplidar
```

Start the driver:

```bash
cd /home/user/rplidar_deploy
./run-rplidar.sh
```

The node publishes `sensor_msgs/msg/LaserScan` on `/scan`, uses frame `laser`,
and defaults to ROS domain 10.

## Inspect from another shell

```bash
docker exec rplidar bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 topic hz /scan'
```

The systemd unit is supplied but should only be enabled after a successful
hardware test.
