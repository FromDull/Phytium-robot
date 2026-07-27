# OpenAMP 从核 LQR 基础控制实施报告

日期：2026-07-15

## 目标与范围

第一版控制对象为轮腿机器人左右轮电机，腿高在启用前由机械结构或已有执行机构固定，LQR 运行期间不改变腿高。状态量为机身俯仰角、俯仰角速度、平均轮位移和平均轮速度；输出为左右轮电机力矩。

本次没有加入转向、行驶速度指令、腿高调节、位置跟踪积分器或 Linux 侧实时控制环。

## 已实现内容

- 新增独立 `lqr_controller`，实现启用瞬间轮位置归零、状态反馈、两轮总力矩到单轮力矩分配、力矩限幅、倾倒保护和非法浮点数保护。
- 固定腿高模型已加入轮电机对车身的等大反向作用力矩，并按真实 `100 Hz` 周期离散化后使用 `dlqr`。
- 当前直接总力矩增益 `K_tau = [-3.144999938, -0.294300374, -0.037757324, -0.159143067]`，输出单位为两轮总力矩 `N·m`。
- 新增 CAN 力矩指令，写寄存器 `0x0020`，单位按协议定义为 `0.01 N·m`。
- 新增 CAN 反馈解析，解析 `0x580 + motor_id`、命令字 `0x2A` 的位置、转速和电流反馈。
- 新增 CAN RX FIFO 非阻塞轮询、每个电机的最新反馈缓存和通用定时器时间戳。
- BMI088 数据转换为 `m/s²` 和 `rad/s`，启用时采集 100 个静止样本校准陀螺零偏，并用互补滤波生成 pitch。
- 新增 100 Hz 从核控制状态机：`disabled -> arming -> active`，故障进入 `fault` 并发送零力矩和 idle 命令。
- OpenAMP 主循环改为 `platform_poll_nonblocking()`，RPMsg 通信不再阻塞控制周期。
- 新增 RPMsg 命令 `balance-enable`、`balance-disable` 和 `balance-status`。
- ACK 扩展到 120 字节，返回状态、故障位、控制频率、姿态、轮状态、左右力矩和循环次数。
- 主核 `rpmsg_client` 已支持构建和解码上述命令。

## 默认参数

| 参数 | 当前值 |
| --- | --- |
| 左轮电机 ID | 1 |
| 右轮电机 ID | 2 |
| 左轮方向 | +1 |
| 右轮方向 | -1 |
| 整车总质量 | 6.9500 kg |
| 车身刚体质量 | 6.7600 kg |
| 旋转轮组总质量 | 0.1900 kg |
| 轮半径 | 0.03225 m |
| 质心高度 | 0.07300 m |
| 车身俯仰惯量 | 0.025000 kg·m² |
| 轮组转动惯量 | 0.000098806 kg·m²（两实心圆盘估算） |
| 控制频率 | 100 Hz |
| 当前模型单轮力矩限制 | 0.22 N·m |
| 启用时相对固定直立目标的最大俯仰角 | 5° |
| 启用时最大绝对俯仰角速度 | 0.15 rad/s |
| 启用前单轮最大允许线速度 | 0.10 m/s |
| 运行超速保护阈值 | 1.0 m/s |
| 倾倒阈值 | 相对固定车身直立目标 15° |
| 电机反馈超时 | 100 ms |
| IMU 超时 | 30 ms |
| 等待反馈超时 | 2 s |

控制律直接输出两轮总力矩：

```text
tau_total = -(K1*theta + K2*theta_dot + K3*position + K4*velocity)
tau_single = clamp(tau_total / 2, -0.22, 0.22)
```

左右轮再按物理安装方向映射为 CAN 力矩命令。当前 `0.22 N·m` 仅是首轮实机保护限值，不代表电机最终安全额定力矩。

标准平衡车的 `theta=0` 是固定车身坐标系的直立姿态，由加速度计的重力方向和固定的 IMU 安装偏角共同确定。当前 `BALANCE_IMU_MOUNT_PITCH_RAD=0` 假设 IMU 坐标轴与车身坐标轴对齐。`balance-enable` 只校准陀螺仪零偏并将当前轮位置设为 `p=0`，不再修改俯仰零点。绳子只作为松弛的防摔保护，不定义平衡角。

对 `balance-enable` / `balance-disable` / `balance-status` 的 ACK，原 servo PWM 调试字段临时复用为左右电机反馈电流和原始 rpm，主核客户端输出 `motor feedback`。

新增 `torque-test <motor_id> <torque_nm> [duration_ms]` 执行器独立诊断。该命令只在平衡环非 active/arming 时执行，力矩限制为 ±0.22 N·m，时间限制为 20–2000 ms，结束后自动发送零力矩并进入 idle。ACK 返回测试期间的峰值反馈电流和带符号峰值 rpm。

新增 `motor-fault <motor_id>` 读取驱动器 `0x000C` 32 位错误寄存器，主机客户端会解码过流、欠压、过压、过温、编码器和运行状态等错误位。

启用时要求车身俯仰角相对固定直立目标不超过 5°、俯仰角速度不超过 0.15 rad/s，且任一轮线速度绝对值不超过 0.10 m/s，否则拒绝进入 active。active 状态下平均轮速绝对值超过 1.0 m/s 时立即进入故障并下发零力矩和 idle。

控制频率当前设为 100 Hz，因为 BMI088 现有加速度计和陀螺仪配置均为 100 Hz。提升到 500 Hz 前必须先同步修改 IMU ODR、滤波带宽和超时阈值。

## 故障位

| 位 | 含义 |
| --- | --- |
| `0x01` | IMU 无效或超时 |
| `0x02` | 左轮反馈无效或超时 |
| `0x04` | 右轮反馈无效或超时 |
| `0x08` | CAN 初始化、总线或发送故障 |
| `0x10` | 超过倾倒角 |
| `0x20` | 控制周期严重超时 |
| `0x40` | 启用后等待电机反馈超时 |
| `0x80` | 启用姿态/速度不合格、运行轮速超限，或参数/控制器配置错误 |

## 构建结果

从核固件已生成：

```text
output/target/lib/firmware/openamp_core0.elf
```

文件为静态链接 AArch64 ELF，入口地址为 `0xb0100000`。

主核测试程序：

```text
/home/cnvhk/phytium-work/openamp_rpmsg_comm/build/rpmsg_client
```

当前开发机上的该二进制为 x86_64，仅用于协议和编译检查。把源码放到飞腾派后执行：

```bash
cd ~/openamp_rpmsg_comm
make client
```

即可生成飞腾派本机可运行的 AArch64 Linux 客户端。

## 实机测试顺序

首次测试必须架空轮子、扶正机身并保持至少 1 秒静止：

```bash
sudo ./build/rpmsg_client /dev/rpmsg0 balance-status
sudo ./build/rpmsg_client /dev/rpmsg0 balance-enable
watch -n 0.1 'sudo ./build/rpmsg_client /dev/rpmsg0 balance-status'
sudo ./build/rpmsg_client /dev/rpmsg0 balance-disable
```

`balance-enable` 会先做 1 秒陀螺零偏校准，再切换 ID 1/2 到力矩模式并等待新反馈。架空时向前轻微倾斜，两个轮子在地面接触方向上都应向前追赶；方向不对时立即执行 `balance-disable`，先修正方向配置，不修改 LQR 增益。

## 实机前必须确认

- ID 1/2 是否确实为左右轮，且右轮镜像方向是否为 `-1`。
- BMI088 绕轮轴是否为 gyro Y，pitch 正方向是否正确。
- 电机反馈位置是否为 `0.01°`、速度是否为 rpm、电流是否为 `0.01 A`。
- 力矩寄存器 `0x0020` 的有符号值和 `0.01 N·m` 比例是否与实际电机固件一致。
- 32.25 mm 是否为负载后的有效轮半径。
- 当前 K 已按实测质量和惯量模型重新计算，但模型仍需通过架空及保护架测试验证符号、执行器比例和未建模动态。
- 两个轮电机的力矩爬升率需调到能在一个 10 ms 控制周期内完成 `+0.22 -> -0.22 N·m` 反转；建议初值为 `50 N·m/s`。
- `Iw` 目前是按总转动轮组质量 `0.19 kg` 的两实心圆盘估算，有 CAD 或实测惯量后必须重新运行 MATLAB 脚本。
- 当前没有已知 GPIO 映射可接物理急停，软件急停入口为 `balance-disable`，其余自动保护由从核状态机执行。

## 源码位置

实际持久化构建源位于：

```text
package/phytium-standalone/openamp_app/
```

Buildroot 补丁和外设配置位于：

```text
package/phytium-standalone/0001-openamp-enable-application-peripherals.patch
package/phytium-standalone/0002-openamp-connect-application-rpmsg-handler.patch
```

可移植业务副本位于：

```text
/home/cnvhk/phytium-work/openamp_rpmsg_comm/remote_firmware/
```

修正后的离散 LQR 建模、限幅、量化和一拍延迟仿真脚本位于：

```text
/home/cnvhk/phytium-work/openamp_rpmsg_comm/wheel_leg_lqr_corrected.m
```
