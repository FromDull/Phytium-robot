# ROS 2 RGB-D 相机接入规划

更新日期：2026-07-17

## 目标

在现有飞腾派机器人控制台中显示 RGB 图像、深度伪彩色图、图像参数和 ROS 2 话题状态，同时保留当前雷达 `/scan` 与里程计 `/scan_odom` 链路。

## 数据链路

```text
RGB-D 相机驱动（ROS 2）
  ├─ RGB Image ───────┐
  ├─ Depth Image ─────┼─> 网页后端 rclpy 订阅
  └─ CameraInfo ──────┘      ├─ 最新帧缓存
                              ├─ RGB JPEG 编码
                              ├─ Depth 伪彩色转换
                              └─ MJPEG/图片接口 ─> 浏览器

雷达 /scan ─> 现有 ICP 里程计与 SSE 状态接口（保持不变）
```

浏览器不直接连接 ROS。相机状态、帧率和分辨率继续通过 `/api/stream` 的 SSE 推送；大尺寸图像使用独立的 MJPEG/图片接口，不把 Base64 图像塞进 SSE JSON。

## 默认话题与配置

话题名通过网页容器环境变量配置，默认值为：

| 环境变量 | 默认 ROS 2 话题 |
| --- | --- |
| `CAMERA_RGB_TOPIC` | `/camera/color/image_raw` |
| `CAMERA_DEPTH_TOPIC` | `/camera/depth/image_raw` |
| `CAMERA_RGB_INFO_TOPIC` | `/camera/color/camera_info` |
| `CAMERA_DEPTH_INFO_TOPIC` | `/camera/depth/camera_info` |

接入真实相机后，以 `ros2 topic list` 和相机驱动文档为准。若驱动提供对齐后的深度图，优先把 `CAMERA_DEPTH_TOPIC` 指向 `aligned_depth_to_color` 对应话题。

## ROS 2 订阅策略

- 图像类型：`sensor_msgs/msg/Image`；若相机提供压缩 RGB，可订阅 `sensor_msgs/msg/CompressedImage`。
- 相机参数：`sensor_msgs/msg/CameraInfo`。
- 图像 QoS：Sensor Data 风格，`BEST_EFFORT`、`KEEP_LAST`、深度 2。
- 缓冲策略：只保留最新帧；浏览器慢时丢弃旧帧，不能反向阻塞相机驱动。
- RGB 与深度同步：第一版独立显示；需要点云或像素对应时，再加入时间同步和外参/TF 校验。
- ROS Domain：继续使用 `ROS_DOMAIN_ID=10`，容器保持 host network 与 UDPv4 配置。

## 图像处理

RGB：

- 推荐输出 640×480、10–15 FPS；
- JPEG 质量建议 70–80；
- 常见编码需要支持 `rgb8`、`bgr8`、`rgba8`、`bgra8`。

深度：

- 支持 `16UC1`（通常单位为毫米）和 `32FC1`（通常单位为米）；
- 无效值、0、NaN 显示为黑色；
- 浏览器提供最近/最远距离和色带选择；
- 伪彩色转换应在飞腾派端限帧处理，默认不高于 10 FPS，避免占满 CPU。

若当前 ARM64 镜像没有 OpenCV/Pillow，优先增加一个小型图像转换依赖层，而不是手写 JPEG 编码。最终依赖要固定在 Dockerfile 中。

## 网页接口

计划接口：

| 接口 | 用途 |
| --- | --- |
| `/api/stream` | 相机在线状态、帧率、分辨率、编码、对齐状态和话题名 |
| `/api/camera/rgb.mjpg` | RGB 连续 JPEG 流 |
| `/api/camera/depth.mjpg` | 深度伪彩色连续 JPEG 流 |
| `/api/camera/config` | 更新深度显示范围和色带（后续） |

当前网页已加入 RGB-D 相机页面和离线状态模型，但尚未创建真实图像订阅与 MJPEG 接口。

## 实施顺序

1. 接入相机并记录 `ros2 topic list -t`、`ros2 topic info -v`、编码、分辨率和帧率。
2. 确认 ARM64 相机驱动容器和 ROS Domain 10 通信正常。
3. 实现 RGB 最新帧订阅与 MJPEG 输出，先验证延迟和 CPU。
4. 实现深度解码、距离裁剪和伪彩色输出。
5. 接入 CameraInfo、TF 和 RGB/深度对齐状态。
6. 做雷达与相机同时运行的压力测试。
7. 若需要三维点云，再增加 `PointCloud2` 或在后端根据深度与内参生成点云；不要在第一版同时引入。

## 验收标准

- RGB 和深度页面均能识别在线/离线；
- 页面显示的话题、分辨率、编码和帧率与 ROS 2 一致；
- 画面断开后 2 秒内显示离线，恢复后自动继续；
- RGB 端到端延迟目标低于 300 ms；
- 深度距离显示与实测基本一致；
- 相机和雷达同时运行时 `/scan` 频率不明显下降；
- 网页断开不会影响 ROS 相机驱动；
- 长时间运行不会持续增长内存。
