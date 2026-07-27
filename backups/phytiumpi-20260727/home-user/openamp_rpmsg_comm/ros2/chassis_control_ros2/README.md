# OpenAMP ROS 2 底盘节点

该节点通过现有 `rpmsg-broker` 独占分发架构访问从核，不直接打开 `/dev/rpmsg0`。它订阅 `/cmd_vel`，将速度目标和看门狗发送给从核，并发布轮速里程计 `/odom` 与 `/diagnostics`。

## 安全特性

- 从核只在平衡状态为 `active` 时接受速度命令；
- ROS 命令超过 `cmd_vel_timeout_s` 后节点发送零速度；
- RPMsg 命令超过从核看门狗后，从核平滑降到零速度；
- 线速度和角速度在 ROS 与从核两侧都有限幅；
- 未配置真实轮距时，`angular.z` 强制为零；
- 节点不会启动时自动启用平衡。

## 构建

```bash
mkdir -p ~/ros2_ws/src
ln -s ~/openamp_rpmsg_comm/ros2/chassis_control_ros2 ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select chassis_control_ros2
source install/setup.bash
```

先测量左右轮与地面接触中心之间的距离，并修改 `config/chassis.yaml` 的 `wheel_track_m`。没有测量值时保持 `0.0`，只测试直线。

## 启动与低风险验证

确保 `rpmsg-broker.service` 正常运行，然后：

```bash
ros2 launch chassis_control_ros2 chassis.launch.py
ros2 service call /chassis/enable std_srvs/srv/Trigger '{}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.02}, angular: {z: 0.0}}"
ros2 topic echo /odom
ros2 topic echo /diagnostics
```

第一次测试必须使用保护绳、固定腿高和开阔地面。确认直线方向正确后再逐步增加到 `0.05 m/s`。只有轮距正确并验证左右轮方向后，才从 `angular.z=0.05 rad/s` 开始转向测试。

停止：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
ros2 service call /chassis/disable std_srvs/srv/Trigger '{}'
```

默认 `publish_tf=false`，避免与现有激光里程计或定位节点同时发布 `odom -> base_link`。单独使用本节点里程计时才改为 `true`。

## 从核调试命令

```bash
rprun chassis-track 0.25
rprun balance-enable
rprun chassis-velocity 0.02 0.0 300
rprun chassis-status
rprun chassis-velocity 0.0 0.0 100
rprun balance-disable
```

`0.25` 只是格式示例，不是机器人实测轮距。
