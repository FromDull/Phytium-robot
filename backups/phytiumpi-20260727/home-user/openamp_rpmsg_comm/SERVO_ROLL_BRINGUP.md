# 四舵机与 IMU Roll 基础验证

本阶段只验证四路 MG996R 接线、命令方向、同步缓动，以及 BMI088 的
roll/roll_rate 融合。不要同时启用轮式平衡，也不要把 roll 直接闭环到腿部。

## 1. 接线与供电

四路 PWM 映射如下：

| 关节命令 | PWM | 飞腾派 40Pin |
| --- | --- | --- |
| servo0 | FPWM1 channel 0 / PWM2_OUT | Pin 32 |
| servo1 | FPWM2 channel 1 / PWM5_OUT | Pin 33 |
| servo2 | FPWM3 channel 0 / PWM6_OUT | Pin 7 |
| servo3 | FPWM3 channel 1 / PWM7_OUT | A43 对应引脚 |

- 四个 MG996R 使用独立的 5–6 V 大电流电源，不能由飞腾派 5 V 引脚供电。
- 舵机电源地和飞腾派地必须共地。
- 首次验证应拆下舵盘或断开连杆。装上连杆后，机器人必须悬空并有刚性支撑。
- `servo-stop` 会停止轨迹并关闭四路 PWM，MG996R 会立即失去保持力。执行前必须托住
  机身；卡死或命令无响应时仍应立即切断舵机电源。

## 2. 编译与部署

从核 ELF 只能在装有飞腾裸机 SDK 和交叉编译器的编译机上生成。以下命令在编译机执行，
不在飞腾派上执行：

```bash
cd ~/phytium-work/openamp_rpmsg_comm
git pull --ff-only
make firmware-elf
scp ../phytium-pi-os/output/build/phytium-standalone-openamp-v1.0/example/system/amp/openamp_for_linux/phytiumpi_aarch64_firefly_openamp_core0.elf \
    user@192.168.137.167:/home/user/openamp_core0.elf
```

若保留 `makeelf` 别名，应让它执行仓库目标，不能直接在旧 SDK 快照上运行 `make`：

```bash
alias makeelf='make firmware-elf'
```

以下命令在飞腾派上执行。飞腾派可以使用本机 GCC 编译 Linux 客户端，但不能编译从核
裸机 ELF：

```bash
cd /home/user/openamp_rpmsg_comm
git pull --ff-only
make client
reloadrproc
```

确认新客户端版本和从核通信：

```bash
rprun heartbeat
```

客户端版本应为 `0.20.1-servo-safe-default` 或更高。

历史命令名 `servocenter` 现在不再表示四路 90 度；它被重新定义为腿部安全参考原始角
`[45,135,45,135]`。固件启动缓存和禁用状态下的 PWM 预装值也使用同一参考值。

## 3. 单独验证 IMU Roll

先停用平衡和云台，把机器人固定并保持完全静止：

```bash
rprun balance-disable
rpgimbal estop
rprun imu-calibrate 100
rprun imu-attitude
```

连续观察：

```bash
watch -n 0.1 'rprun imu-attitude'
```

验证项目：

1. 水平静止时 `roll` 和 `pitch` 应稳定，`roll_rate`、`pitch_rate` 接近 0。
2. 绕 IMU X 轴缓慢左右倾斜时，roll 应连续变化，回到水平后回到原值附近。
3. 静止倾斜时 roll 不能持续漂移；快速倾斜时 roll_rate 应立即响应。
4. 当前定义中，加速度 Y 为正时 roll 为正。若实机左右方向与机器人坐标约定相反，
   修改 `PHYTIUM_BMI088_ROLL_DIRECTION`，不要交换其他 IMU 轴。

`imu-calibrate` 仅允许在平衡和云台均停用时运行。标定过程中必须静止；命令默认采集
100 个样本，约需 1 秒。

## 4. 逐路验证舵机

确保平衡处于 `disabled`：

```bash
rprun balance-status
rprun servo-status
```

以下动作以 90 度为装配中位，只移动 5 度。首次应在舵盘或连杆断开的状态执行：

```bash
rprun servo-move 2000 95 90 90 90
rprun servo-move 2000 90 90 90 90

rprun servo-move 2000 90 95 90 90
rprun servo-move 2000 90 90 90 90

rprun servo-move 2000 90 90 95 90
rprun servo-move 2000 90 90 90 90

rprun servo-move 2000 90 90 90 95
rprun servo-move 2000 90 90 90 90
```

每次记录：实际运动关节、正角方向、是否存在抖动/卡滞、90 度时舵盘机械位置。
`servo-status` 显示的是命令角度和 PWM 脉宽，不是实际位置反馈。

## 5. 原地同步变腿高

实机已确认的舵机与关节关系如下。表中的有效角度统一定义为从折叠端向外增大，
不等于底层舵机原始角度。

| 底层通道 | 物理关节 | 有效角到原始角 |
| --- | --- | --- |
| servo0 / 1 号舵机 | 右前 | `raw = effective` |
| servo1 / 2 号舵机 | 右后 | `raw = 180 - effective` |
| servo2 / 3 号舵机 | 左后 | `raw = effective` |
| servo3 / 4 号舵机 | 左前 | `raw = 180 - effective` |

`leg-joints` 的参数顺序固定为右前、右后、左前、左后。已验证的稳定参考姿态为四个
有效角均为 45 度，客户端会自动转换为底层原始角 `[45,135,45,135]`：

```bash
rprun balance-disable
rprun servo-status
rprun leg-enable CONFIRM
rprun servo-status
```

`leg-enable` 不声称知道实际关节角。执行前必须支撑机器人；它只会输出固定安全参考命令
`[45,135,45,135]`，随后保持 `starting` 约 3 秒。MG996R 没有位置反馈，因此状态中的角度
始终是命令坐标而非实测角度。启动稳定期结束前，`leg-joints` 必须返回 `busy`；上电后或
`servo-stop` 后未启用时则必须返回 `not-armed`。

装上连杆后优先使用 `leg-joints`。`servo-move` 是底层原始舵机命令，只用于连杆断开时
逐路诊断，不应用它直接做并联腿同步动作。

在稳定参考姿态附近验证时，每次只改变 2 度，且左右两侧保持完全对称：

```bash
rprun leg-joints 3000 43 43 43 43
rprun leg-joints 3000 45 45 45 45
```

当前实机已知有效范围为 `0..45 deg`，客户端和从核会同时拒绝任何 `leg-*` 超界值。
不得再测试 47 度。四路各自更精确的安装零偏和机械下限尚未标定，因此第一阶段只允许
从人工确认的 45 度姿态小幅减小到 43 度，再返回 45 度。

结束测试后执行 `rprun servo-stop` 会真正关闭四路 PWM，同时控制器回到 `unarmed`；下次
运动必须重新支撑机器人并执行 `leg-enable CONFIRM`。

持续观察命令状态：

```bash
watch -n 0.1 'rprun servo-status'
```

判断通过的条件：

- 四个关节同时开始、同时结束，过程中没有连杆顶死。
- 机身高度变化方向一致，左右高度肉眼无明显差异。
- 支架上的机身 roll 没有持续增大。
- 舵机和电源线不过热，电源没有明显掉压或复位。

当前同步动作是四关节命令插值，还不是并联腿的笛卡尔高度控制。通用五连杆正逆运动学
模块已在 `src/leg_kinematics.*` 中准备好，但在杆长、解支和安全角范围实测完成前不会连接
到 PWM。详细测量项目见 `LEG_KINEMATICS_GUIDE.md`。在此之前不要落地动态变腿高，也不要
同时执行 `balance-enable`。
