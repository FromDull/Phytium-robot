# 云台相机动态 TF

坐标链：

```text
base_link -> gimbal_yaw_link -> gimbal_pitch_link
          -> camera_link -> camera_color_optical_frame
```

宿主机的 `gimbal-tf-state-bridge` 直接读取 `gimbal-daemon` Unix socket，并把带
源时间戳和反馈有效性的状态原子写入共享目录。ROS 2 节点仅在双轴反馈有效、无故障且
状态新鲜时发布动态 TF，同时持续发布 `/gimbal/tf_status`。消费者必须检查该状态，
不能继续使用 TF2 缓存中的最后一帧失效姿态。

外参模板在 `config/gimbal_tf_calibration.json`。轴方向和零偏必须用实机动作校准，
不要通过反复启用云台猜测符号。

```bash
colcon build --packages-select gimbal_camera_tf_ros2 --symlink-install
ros2 topic echo /gimbal/tf_status
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```
