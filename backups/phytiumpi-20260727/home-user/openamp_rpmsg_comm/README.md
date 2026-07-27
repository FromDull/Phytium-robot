# OpenAMP 核间通信

## 功能目标

本项目对应“OpenAMP 核间通信”。基础版完成一个简单可靠的命令帧格式，用于 Linux 主核和从核之间传输结构化数据。

命令帧包含：

- 帧头
- 类型
- 序号
- 长度
- 负载
- 校验

异常处理设计：

- 校验错误：丢弃帧
- 长度错误：丢弃帧
- 超时：Linux 侧重发或进入安全状态
- 从核异常：Linux 侧发送 `CMD_CAN_SAFE_STOP`

## 直接验证

```bash
make test
./build/test_protocol
```

完整部署和板上验证步骤见：

```text
DEPLOY_VERIFY.md
```

平衡位置外环的控制结构、协议、遥测、调参和当前部署边界见
[POSITION_HOLD_OUTER_LOOP_HANDOFF.md](POSITION_HOLD_OUTER_LOOP_HANDOFF.md)。

## 板上验证步骤

1. 飞腾派启动 Linux。
2. 启动从核：

```bash
echo start | sudo tee /sys/class/remoteproc/remoteproc0/state
```

3. 检查通道：

```bash
ls /sys/bus/rpmsg/devices/
ls /dev/rpmsg*
```

4. 绑定字符设备驱动：

```bash
cd /sys/bus/rpmsg/devices/virtio0.rpmsg-openamp-demo-channel.-1.0
echo rpmsg_chrdev | sudo tee driver_override
sudo modprobe rpmsg_char
```

5. Linux 端可参考 `linux_user/rpmsg_client.c`，从核裸机端可参考 `remote_firmware/slave_app.c`。

## OpenAMP 开机自动初始化

`integration/systemd/openamp-initproc.sh` 对应板端 `.bashrc` 中的 `initproc`：
先关闭 Linux 的 CAN0/CAN1 并解绑两个 CAN 控制器，再复制从核 ELF、重启
`remoteproc0`、绑定 `rpmsg_chrdev`，最后确认 `/dev/rpmsg0` 已创建。

安装并启用服务：

```bash
sudo install -m 0755 integration/systemd/openamp-initproc.sh \
    /usr/local/sbin/openamp-initproc
sudo install -m 0644 integration/systemd/openamp-initproc.service \
    /etc/systemd/system/openamp-initproc.service
sudo systemctl daemon-reload
sudo systemctl enable --now openamp-initproc.service
```

上传新固件时，将新文件放到固定路径：

```text
/home/user/openamp_core0.elf
```

然后用 systemd 重新复制、启动和绑定，推荐使用：

```bash
sudo systemctl restart openamp-initproc.service
sudo systemctl status openamp-initproc.service --no-pager
rprun heartbeat
```

原有 shell 别名也可使用，注意准确拼写是 `reloadrproc`，不是
`reloadproc`：

```bash
source ~/.bashrc
reloadrproc
```

`systemctl restart` 可用于脚本、SSH 和未加载 `.bashrc` 的终端，因而更适合作为
固件更新后的标准重载方法。

## Linux 统一 RPMsg Broker

`rpmsg-broker` 是 Linux 侧唯一直接打开 `/dev/rpmsg0` 的进程。`rprun`、
`balance_logger`、`gimbal_test` 和 `gimbal-daemon` 都通过
`/run/rpmsg-broker/rpmsg.sock` 收发，因而可以并存，不会竞争回复。

安装和完整验收步骤见 [RPMSG_BROKER_GUIDE.md](RPMSG_BROKER_GUIDE.md)。

## CAN 电机控制接入

电机 CAN 帧组装代码位于：

```text
remote_firmware/motor_can.c
remote_firmware/motor_can.h
```

飞腾从核 CAN 适配层在：

```text
remote_firmware/phytium_can_port.c
remote_firmware/phytium_can_port.h
```

Linux 客户端支持以下命令：

```bash
sudo ./rpmsg_client /dev/rpmsg0 heartbeat
sudo ./rpmsg_client /dev/rpmsg0 enable 1
sudo ./rpmsg_client /dev/rpmsg0 zero 1
sudo ./rpmsg_client /dev/rpmsg0 test
sudo ./rpmsg_client /dev/rpmsg0 torque-test 1 0.05 200
sudo ./rpmsg_client /dev/rpmsg0 pvt 1 1000 100 20
sudo ./rpmsg_client /dev/rpmsg0 stop 1
```

## 平衡控制运行时配置

从核固件支持在不重新编译 ELF 的情况下修改俯仰零点、四个 LQR 增益、
角速度滤波、姿态优先角、轮速保护和单轮力矩限制。配置保存在从核 RAM 中，
重启从核后恢复固件默认值。所有修改命令
只能在 `disabled` 或 `fault` 状态执行，先停止平衡控制：

```bash
rprun balance-disable
rprun balance-config
```

设置运行时单轮硬超速保护阈值，单位为 `m/s`，允许范围为 `0.2–2.0`。
默认值仍为 `1.0 m/s`：

```bash
rprun balance-disable
rprun balance-speed-limit 1.2
rprun balance-config
rprun balance-enable
```

轮径 `0.03225 m` 时，`1.2 m/s` 约为 `355 rpm`。该命令不会修改静止
使能条件的低速门槛，也不能关闭超速保护。超过默认值只用于有保护绳的诊断，
不能用来掩盖发散或振荡。

`balance-trim` 设置机器人处于真实机械直立位置时 IMU 应扣除的俯仰角，
单位为度，允许范围为 `-5` 到 `+5` 度。例如机械直立时状态显示
`pitch=+1.0 deg`：

```bash
rprun balance-trim 1.0
rprun balance-config
```

设置四个直接总力矩 LQR 增益：

```bash
rprun balance-gains -3.759674 -0.486785 -0.062457 -0.247058
rprun balance-config
```

增益顺序固定为 `K_theta K_theta_rate K_position K_velocity`。固件只接受
与当前模型符号一致的有限范围参数，但范围检查不能证明参数一定稳定；新参数
必须架设保护绳并从小倾角开始验证。

设置俯仰角速度一阶低通截止频率，允许 `5–40 Hz`，默认 `20 Hz`：

```bash
rprun balance-disable
rprun balance-filter 20
rprun balance-config
```

设置姿态优先角，允许 `1–10` 度，默认 `3` 度：

```bash
rprun balance-posture-angle 3
```

设置单轮力矩限制，允许 `0.05–0.30 N*m`，默认 `0.22 N*m`：

```bash
rprun balance-torque-limit 0.22
```

`0.22 N*m` 以上只用于确认电机、驱动器、电池和机械结构均允许更高输出后的
短时保护绳测试。提高该限制会增加电机电流、跌倒冲击和机械损坏风险。

恢复编译默认配置：

```bash
rprun balance-reset-config
```

当俯仰角绝对值达到 `3` 度，并且位置/速度力矩与姿态恢复力矩方向相反时，
控制器会临时屏蔽相反的行走力矩，优先使用电机力矩恢复姿态。回到阈值以内后
自动恢复完整四状态 LQR。

## 底盘速度控制与 ROS 2

固定腿高并完成平衡验证后，从核可以接收带看门狗的线速度和角速度目标。
线速度经过加速度限制后进入 LQR 的速度参考，同时积分为连续位置参考；角速度
使用左右轮差动力矩闭环。姿态超过姿态优先角时，从核暂停转向力矩，把执行器
能力优先留给俯仰恢复。

先在平衡关闭时配置实测轮距，再启用平衡和发送低速命令：

```bash
rprun balance-disable
rprun chassis-track 0.25
rprun balance-enable
rprun chassis-velocity 0.02 0.0 300
rprun chassis-status
rprun chassis-velocity 0.0 0.0 100
rprun balance-disable
```

`0.25 m` 只是命令格式示例，必须替换为左右轮与地面接触中心的实测距离。
未设置轮距时从核拒绝非零角速度。命令超时后从核独立把速度目标平滑降到零，
因此 ROS、Broker 或主核应用异常退出不会留下永久运动命令。

ROS 2 包位于 `ros2/chassis_control_ros2`，订阅 `/cmd_vel`，发布 `/odom` 和
`/diagnostics`。构建、参数和首次低风险验证见该目录的 `README.md`。可变腿高
不会在本轮随底盘命令一起开放，实施门槛和增益调度方案见
`VARIABLE_LEG_HEIGHT_CONTROL_PLAN.md`。

## 高频平衡日志

`balance_logger` 使用一个常驻进程和轻量 RPMsg 遥测回复，不再为每个样本
启动一次 `sudo rpmsg_client`。默认采样率为 20 Hz，最高允许 50 Hz。推荐由
日志器负责启用平衡控制；按 `Ctrl+C`、收到 `SIGTERM` 或达到指定时长时，
它会自动发送 `balance-disable`：

```bash
make logger
sudo ./build/balance_logger --rate 20 --enable --stop-on-fault
```

短时间 50 Hz 测试并在 30 秒后自动停机：

```bash
sudo ./build/balance_logger --rate 50 --duration 30 --enable --stop-on-fault
```

只记录已经运行的控制器，不负责启停：

```bash
sudo ./build/balance_logger --rate 20
```

日志默认写入 `logs/balance/`，每小时创建新 CSV，并在切换后后台压缩上一小时
文件。即使使用 `sudo`，程序也会在打开 RPMsg 设备后恢复为原用户身份，因此
日志不会变成 root 所有。CSV 包含 `read_ok`、RPMsg `latency_us`、从核
`loop_count`、真实 `state/fault`、LQR 状态、力矩、电流和原始轮速。

启用 `rpmsg-broker` 后，日志器运行期间可以同时执行 `rprun` 和
`gimbalctl`。所有请求由 Broker 串行写入从核，并按 sequence 返回给原客户端。

`pvt 1 1000 100 20` 含义：

```text
1 号电机
目标位置 10.00 度
速度 100 rpm
力矩百分比 20%
```

从核收到 `CMD_CAN_PVT` 后会打包为 `0x25` PVT CAN 帧，并通过 `phytium_can_send()` 发送。`test` 命令会初始化两台电机，并让两台电机各转动一圈。

`torque-test` 仅用于排查 ID 1/2 轮电机的力矩模式。力矩范围为 -0.22–0.22 N·m，脉冲时间为 20–2000 ms；命令结束后会自动发送零力矩并进入 idle。长时间测试前必须先用机械方式约束车轮，不能用手接触旋转中的车轮。

读取电机驱动器错误寄存器 `0x000C`：

```bash
sudo ./build/rpmsg_client /dev/rpmsg0 motor-fault 1
sudo ./build/rpmsg_client /dev/rpmsg0 motor-fault 2
sudo ./build/rpmsg_client /dev/rpmsg0 motor-fault 3
sudo ./build/rpmsg_client /dev/rpmsg0 motor-fault 4
```

`motor-fault` 支持 ID 1～4；读取 ID 3/4 时云台必须处于 `disabled` 或 `fault`。`torque-test` 仍只允许 ID 1/2。

轮速三方对照诊断只能在平衡控制停用且车轮架空或机械约束时运行。它会短时施加不超过 `0.10 N*m` 的力矩，同时比较周期反馈 `0x2A`、实时速度寄存器 `0x0006` 和编码器位置差分得到的速度，结束后自动清零力矩并进入 idle：

```bash
rprun balance-disable
sudo ./build/rpmsg_client /dev/rpmsg0 motor-speed-diag 1 0.05 1000
sudo ./build/rpmsg_client /dev/rpmsg0 motor-speed-diag 2 -0.05 1000
```

实机三方对照确认周期反馈速度、`0x0006` 寄存器速度和位置差分速度误差小于约 1.2%，因此平衡控制使用 `1.0` 速度系数。

## 云台控制和安全标定

`linux_user/gimbal_test.c` 是 Linux 侧云台控制和标定工具，固定使用 CAN ID 3 作为 yaw、ID 4 作为 pitch。编译命令为：

```bash
make gimbal
```

### 永久零点

首次安装时先可靠托住云台，将 yaw、pitch 手动放到机械中位。确认摄像头和支架都有足够活动空间后执行：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 setzero CONFIRM
```

该命令向两台驱动器写入永久原点寄存器 `0x00A6`。驱动器会保存当前位置并重启；只有重新安装电机、驱动板或机械结构变化时才需要再次执行。通用客户端的 `zero <id>` 使用临时原点 `0x00A7`，不能替代永久零点。

### 软件安全范围

永久零点完成后必须标定四个软件边界。边界应留在真正碰撞位置以内，不能把机械止挡或线缆刚好拉紧的位置作为软件边界。

保持从核状态为 `disabled`，手动移动对应轴到安全位置后分别确认：

`limit` 第一次发现驱动器处于 idle 且没有新反馈时，会让 yaw、pitch
进入零力矩标定模式并自动等待实时角度；这个过程只唤醒反馈，不发送位置目标。
全部四个边界完成、执行 `disable`/`estop` 或 30 秒没有继续标定时，两轴自动
回到 idle。

```bash
sudo ./build/gimbal_test /dev/rpmsg0 limit yaw min CONFIRM
sudo ./build/gimbal_test /dev/rpmsg0 limit yaw max CONFIRM
sudo ./build/gimbal_test /dev/rpmsg0 limit pitch min CONFIRM
sudo ./build/gimbal_test /dev/rpmsg0 limit pitch max CONFIRM
sudo ./build/gimbal_test /dev/rpmsg0 status
```

只有显示 `limits: valid=0x0f` 才允许启动。最小值必须为负角度、最大值必须为正角度，永久零点必须位于每一对边界之间。

软件边界在从核重启后会清除。仓库根目录的 `gimbal.conf` 保存四个实机边界和常用运动参数；`enable` 会自动读取该文件并在启动前恢复软件边界，不再需要每次手动执行 `limits`。配置项如下：

```ini
yaw_min_deg=-37.94
yaw_max_deg=213.97
pitch_min_deg=-50.13
pitch_max_deg=97.50
home_speed_rpm=5
home_torque_percent=50
return_speed_rpm=5
return_torque_percent=50
move_speed_rpm=20
move_torque_percent=50
```

四个角度必须替换成当前机械结构的实测安全边界。需要把配置放在其他路径时设置 `GIMBAL_CONFIG=/path/to/gimbal.conf`。配置文件不存在时保留旧的手动限位工作方式；配置存在但内容不完整或越界时，运动命令会拒绝执行。`estop`、`disable`、`status` 和标定命令不依赖配置文件，配置写错时仍可停机和重新标定。

也可以直接手动恢复：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 limits -60 60 -25 35 CONFIRM
```

上面的数字只是格式示例，必须替换成实机标定结果。清除全部边界使用 `reset-limits CONFIRM`。

### 启动归零和位置控制

边界有效后启动。两台电机进入位置模式，并以 `5 rpm` 缓慢归零；状态依次经过 `starting`、`homing`、`active`：

如果两轴已经 idle、周期反馈停止，`enable` 会先以零力矩模式自动唤醒反馈；只有收到两轴新鲜角度后才进入位置模式。客户端会等待并自动重试，无需重复手动执行 `enable`。

```bash
sudo ./build/gimbal_test /dev/rpmsg0 enable 15
sudo ./build/gimbal_test /dev/rpmsg0 status
```

`enable` 默认使用 `gimbal.conf` 中的归位速度和力矩；可选参数是 `5..80` 的临时归位力矩百分比，只覆盖本次启动。普通位置命令允许 `1..80%`。默认力矩为 50%；继续增大时必须逐级测试，不要使用 80% 试撞机械限位。`init` 是 `enable` 的兼容别名。只有状态为 `active` 才接受目标。yaw 和 pitch 通过一条原子 RPMsg 命令提交，从核先同时检查两轴边界，再连续发出两条 CAN 帧：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 set 5 0
sudo ./build/gimbal_test /dev/rpmsg0 set 0 5 20 10
sudo ./build/gimbal_test /dev/rpmsg0 center
sudo ./build/gimbal_test /dev/rpmsg0 sweep 5 2
```

持续循环测试使用 `test [angle_deg]`，按 `Ctrl+C` 会请求受控关闭。测试不会自动启动、标零或恢复边界。

归零和 active 状态会每 50 ms 重发同一个受限位置目标，以维持驱动器闭环和 `0x2A` 反馈。归零超过 12 秒、任一反馈超过 500 ms 未更新、实际位置越过边界 1 度、目标未在按角差和速度计算的期限内到位或 CAN 发送失败时，从核会立即让两台电机进入 idle。故障位为：`0x01` yaw 反馈、`0x02` pitch 反馈、`0x04` CAN、`0x08` 边界配置、`0x10` 实际越界、`0x20` 归零超时、`0x40` 流式命令超时、`0x80` 运动到位超时/疑似卡滞。

### 受控关闭和紧急停止

每次 `enable` 收到两轴新鲜反馈后，从核都会记录当时的 pitch。正常关闭保持当前 yaw，先按 `gimbal.conf` 的返回速度和力矩把 pitch 缓慢送回该启动角度；到位并稳定后，每 100 ms 降低 2% 力矩，最后进入 idle。状态依次为 `active -> returning -> stopping -> disabled`，应等待 `disabled` 再断电：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 disable
sudo ./build/gimbal_test /dev/rpmsg0 status
```

`stop` 是 `disable` 的兼容别名。返回阶段反馈丢失、越界、超时或 CAN 失败会立即进入 fault 并 idle，不会盲目继续运动。即将碰撞、机构卡死或其他紧急情况使用立即 idle，不返回启动角度，也不经过力矩缓降：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 estop
```

`status` 会显示目标和实际角度、速度、电流、反馈年龄、软件边界、运行状态和故障。正常缓降只能减少突然失力，无法保证所有重心和摩擦条件下摄像头都不下坠，机械结构仍应配置限位和必要的阻尼或防坠措施。

## 云台相机动态 TF

`ros2/gimbal_camera_tf_ros2` 发布以下坐标链：

```text
base_link -> gimbal_yaw_link -> gimbal_pitch_link
          -> camera_link -> camera_color_optical_frame
```

宿主机的 `gimbal-tf-state-bridge` 直接访问 `gimbal-daemon` Unix socket，不再以
高频子进程调用 `gimbalctl`。ROS 节点只在双轴反馈有效、无故障且状态新鲜时发布
动态 TF，并通过 `/gimbal/tf_status` 显式通知消费者当前姿态是否可信。

```bash
sudo make install-gimbal-tf-state
cd ros2
colcon build --packages-select gimbal_camera_tf_ros2 --symlink-install
```

目标定位等消费者必须同时检查 `/gimbal/tf_status` 和变换时间戳，不能在反馈失效后
继续使用 TF2 缓存中的最后一帧姿态。外参、轴方向和零偏见该包的配置与 README。

## 从核工程必须配置

在飞腾 Pi OS 从核工程配置文件中至少启用：

```text
CONFIG_USE_FCAN=y
```

配置文件通常在：

```text
phytium-pi-os/output/build/phytium-standalone-openamp-v1.0/example/system/amp/openamp_for_linux/configs/phytiumpi_aarch64_firefly_openamp_core0.config
```

如果编译提示 `fio_mux.h`、`fcan.h` 等头文件找不到，需要参考：

```text
example/peripherals/can/can/configs/
```

把 CAN 示例中的 IOMUX/CAN 相关配置同步到从核配置文件。

还需要在 `slaver_00_example.c` 初始化阶段调用：

```c
#include "phytium_can_port.h"

phytium_can_init();
```

## 是否需要从核 ELF 源码

需要。板上通信验证至少要说明两部分代码：

- Linux 端：打开 `/dev/rpmsgX`，发送命令帧。
- 从核端：编译进 `openamp_core0.elf`，在 rpmsg callback 中解析命令并回复。

当前 `remote_firmware/slave_app.c` 给出了从核业务代码框架。真正编译 ELF 时，把其中的 `slave_handle_frame()` 接入飞腾 Standalone OpenAMP 示例工程即可。
