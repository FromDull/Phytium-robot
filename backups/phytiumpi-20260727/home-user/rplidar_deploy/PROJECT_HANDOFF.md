# 飞腾派 2D 雷达与网页仪表盘交接文档

更新日期：2026-07-17

## 1. 项目目标与当前状态

本项目在飞腾派上运行 SLAMTEC RPLIDAR A2M6，并提供一个局域网网页用于查看：

- 2D 雷达实时点云；
- 基于相邻雷达帧 ICP 扫描匹配的相对里程计；
- CPU、内存、温度、网络、雷达频率及诊断信息；
- 通过网页访问飞腾派普通用户 `user` 的交互式 Bash 终端。

飞腾派当前处于断电状态。最后一次在线验证时，雷达驱动、网页仪表盘和原生终端代理都已正常启动，雷达扫描频率约为 25 Hz。

注意：工作区还有一批**尚未部署**的可视化优化：里程计速度、轨迹网格、起点与朝向箭头。这些修改已保存在本机源码中，需在飞腾派重新上电后重新构建网页镜像才会生效。

## 2. 访问方式

飞腾派 SSH 配置名为 `phytiumpi`，最后使用的地址为 `192.168.43.122`。网页地址：

```text
http://192.168.43.122:8080
```

若重启后无法访问，先确认飞腾派供电、网络连接和 IP 地址；不要假设该 IP 永久不变。

## 3. 架构

| 模块 | 运行位置 | 作用 |
| --- | --- | --- |
| `rplidar` | Docker 容器 | ROS 2 Humble 的 `rplidar_ros` 驱动，发布 `/scan` |
| `lidar_web_viewer` | Docker 容器 | 订阅 `/scan`，计算相对里程计，提供仪表盘 HTTP/SSE 服务 |
| `web-terminal.py` | 飞腾派主机的 `user` 用户 | 将浏览器 WebSocket 转为主机 Bash PTY |
| 浏览器 | Windows 或同一局域网客户端 | 显示仪表盘、雷达和终端 |

端口说明：

| 端口 | 协议 | 用途 |
| --- | --- | --- |
| 8080 | HTTP / SSE | 仪表盘、`/api/state`、`/api/health` |
| 8765 | WebSocket | 浏览器终端，登录后建立交互式会话 |

终端代理运行在主机而非容器中，因此终端内看到的是飞腾派的 `user` 用户环境，可以使用已授权的 Docker 命令；它不自动授予 root 权限。

## 4. 关键目录与文件

本机工作区：`C:\Users\34403\Desktop\phytium\rplidar-deploy`

飞腾派部署目录：`/home/user/rplidar_deploy`

| 文件 | 说明 |
| --- | --- |
| `lidar-stack.ps1` | Windows 端统一入口，通过 SSH 调用飞腾派脚本并在启动后打开网页 |
| `lidar-stack.sh` | 飞腾派端统一启动、停止、状态检查脚本 |
| `run-rplidar.sh` | 启动雷达 ROS 2 Docker 驱动 |
| `web-viewer/server.py` | ROS 订阅、ICP 里程计、HTTP/SSE API |
| `web-viewer/index.html` | 仪表盘前端、雷达与轨迹绘制、网页终端界面 |
| `web-viewer/run-web-viewer.sh` | 启动网页仪表盘容器 |
| `web-terminal.py` | 主机侧 WebSocket 到 Bash PTY 的终端代理 |
| `run-web-terminal.sh` | 读取密码哈希配置并启动终端代理 |
| `set-web-terminal-password.py` | 交互式设置或更新仪表盘终端密码 |
| `PROJECT_HANDOFF.md` | 本交接文档 |

## 5. 常用操作

在 Windows PowerShell 中、位于 `C:\Users\34403\Desktop\phytium` 时执行：

```powershell
.\rplidar-deploy\lidar-stack.ps1 start
.\rplidar-deploy\lidar-stack.ps1 status
.\rplidar-deploy\lidar-stack.ps1 stop
```

`start` 的流程：

1. 检查 `/dev/rplidar` 是否存在；
2. 启动 `rplidar` 容器并等待驱动健康状态；
3. 启动网页仪表盘容器，确认 `http://127.0.0.1:8080/api/state` 可用；
4. 启动主机侧终端代理；
5. 若已设置仪表盘终端密码，启动终端代理并输出网页地址和日志位置。

## 5.1 使用 MobaXterm / XTerm 查看和输入命令

飞腾派当前配置为图形启动目标，且已安装 `/usr/bin/xterm`。Windows 上可使用已提供的 MobaXterm，它自带 X11 Server：

1. 打开 `MobaXterm_Personal_23.2.exe`，新建 SSH 会话，主机填写飞腾派当前 IP、用户填写 `user`，并启用 **X11-forwarding**；
2. 在 MobaXterm 终端中执行：

   ```bash
   xterm -geometry 120x38 -title "Phytium Pi 开发终端"
   ```

   这会在 Windows 桌面显示由飞腾派运行的 XTerm 窗口；在其中输入的命令直接在飞腾派执行；
3. 若只需文本终端，直接使用 MobaXterm 当前 SSH 标签页即可，推荐用于复制日志和观察构建过程。

X11 转发仅适合可信局域网；不要将其直接暴露到公网。

首次使用或修改密码时，在 Windows PowerShell 执行：

```powershell
.\rplidar-deploy\lidar-stack.ps1 set-password
```

密码只在飞腾派上保存 PBKDF2-SHA256 哈希，配置文件权限为仅 `user` 可读。不要使用 SSH/root 的系统登录密码作为网页终端密码。

## 6. 上电后的验证顺序

1. 接通雷达与飞腾派电源，等待系统联网。
2. 在 Windows 运行：

   ```powershell
   .\rplidar-deploy\lidar-stack.ps1 start
   ```

3. 打开网页，确认“雷达在线”、扫描频率非零且有点云。
4. 在网页“终端”输入已设置的仪表盘终端密码。
5. 在终端逐条运行：

   ```bash
   whoami
   hostname
   pwd
   docker ps
   curl -s http://127.0.0.1:8080/api/health
   ```

   预期用户为 `user`，主机名为 `phytiumpi`，`docker ps` 显示 `rplidar` 与 `lidar_web_viewer`。

6. 验证雷达话题：

   ```bash
   docker exec -it rplidar bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic hz /scan'
   ```

   输出应持续显示频率。此命令当前存在网页端 `Ctrl+C` 中断不可靠的问题，见“已知问题”。

## 7. 里程计说明

当前里程计由 `web-viewer/server.py` 内的轻量 2D ICP 实现：

- 输入：`/scan` 的相邻雷达帧；
- 输出：网页状态中的 `x`、`y`、`yaw`，并发布 ROS 话题 `/scan_odom`；
- 坐标：以网页“重置原点”时的位置为 `(0, 0, 0)`；
- 质量指标：内点数、ICP RMS、原始扫描 RMS、是否已确认匹配；
- 限制：它是无轮编码器、无 IMU 的相对估计，会随距离和环境重复度积累漂移，静止时也可能受噪声影响。

建议每次测试前重置原点。若要获得稳定的长距离定位，应后续融合轮编码器、IMU，或引入 SLAM/地图定位。

## 8. 终端的安全设计与已知问题

设计：

- 终端代理以 `user` 运行；
- 密码使用 PBKDF2-SHA256（310,000 次迭代）和随机盐哈希保存；
- 密码配置位置：`~/.config/lidar-web-terminal/password.json`，目录权限为 `700`、文件权限为 `600`；
- 密码通过 WebSocket 建连后的第一条认证消息提交，不放入 URL；
- 一次仅允许一个已认证终端会话；
- 会话开始、结束、来源 IP 与持续时间写入：

  ```text
  /home/user/.local/state/lidar-web-viewer/terminal-audit.jsonl
  ```

- 审计日志不记录输入内容，避免把命令中的敏感信息写入日志。

已知问题：网页终端的 `Ctrl+C` 按钮目前没有可靠中断持续运行命令。后续上电后应优先做隔离测试：运行 `sleep 30`，再点击网页 `Ctrl+C`，分别检查浏览器事件、WebSocket 收包、PTY 信号和 Bash 前台进程组。不要在修复前把网页终端作为唯一的故障恢复入口。

网页终端曾显示 Bash 的标题控制符（如 `ESC]0`）；前端已在本机补充过滤，但同样需要在下一次网页镜像构建后才会部署。

## 9. 待部署的本机改动

以下改动目前已修改本机源码，但在飞腾派上一次断电前尚未重新构建部署：

- `web-viewer/server.py`：在网页状态中增加 `vx`、`vy`、`wz`；
- `web-viewer/index.html`：总览增加速度；轨迹视图增加网格、比例、起点和朝向箭头；
- `web-viewer/index.html`：过滤更多 Bash ANSI/OSC 控制符；
- `README.md`：补充相对里程计的漂移说明。

设备恢复后，可从 Windows 执行以下部署步骤：

```powershell
scp .\rplidar-deploy\web-viewer\server.py .\rplidar-deploy\web-viewer\index.html phytiumpi:/home/user/rplidar_deploy/web-viewer/
ssh phytiumpi "cd /home/user/rplidar_deploy && docker build -t lidar-web-viewer:humble web-viewer && ./lidar-stack.sh start"
```

部署会重启雷达驱动和网页服务。完成后刷新浏览器，并在首次使用时运行 `set-password` 设置仪表盘终端密码。

## 10. 故障排查

| 现象 | 检查方式 |
| --- | --- |
| SSH 连不上 | 检查飞腾派供电、Windows 与飞腾派是否在同一网络、IP 是否改变 |
| 找不到雷达 | 飞腾派执行 `ls -l /dev/rplidar /dev/ttyUSB*`，检查 CH340 与雷达供电 |
| 网页打不开 | 执行 `./lidar-stack.sh status`，检查 8080 监听和 `/tmp/lidar-web-viewer.log` |
| 雷达无点云 | 查看 `/tmp/rplidar.log`、`docker ps`，再检查 `/scan` 频率 |
| 终端连不上 | 先执行 `lidar-stack.ps1 set-password`，再执行 `start`；确认 8765 正在监听且没有其他终端会话 |
| 终端命令粘在一起 | 一次只输入一条命令，按 Enter 并等待提示符返回后再输入下一条 |

日志位置：

```text
/tmp/rplidar.log
/tmp/lidar-web-viewer.log
/tmp/lidar-web-terminal.log
~/.local/state/lidar-web-viewer/terminal-audit.jsonl
```

## 11. 交接注意事项

- 不要把 SSH 密码、私钥或仪表盘终端密码提交到仓库或转发给不可信人员。
- 不要用 `git reset --hard`、`git checkout --` 等命令覆盖现有工作区；本项目可能包含尚未部署的本地修改。
- 雷达数据和相对里程计只能在硬件通电后验证；离线时可修改前端、脚本和文档，但不要声称已在板端测试。
