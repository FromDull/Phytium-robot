建议按以下顺序验证，首次测试全程托住摄像头，并保证机器人平衡控制关闭。

**1. 部署**

开发电脑：

```bash
scpelf
```

飞腾派：

```bash
reloadrproc

cd ~/openamp_rpmsg_comm
git pull origin dev
make gimbal
```

确认工具存在：

```bash
ls -lh build/gimbal_test
sudo ./build/gimbal_test /dev/rpmsg0 status
```

预期：

- `state=disabled`
- `feedback valid=0x03`
- yaw/pitch 角度、电流、转速能够读取
- 从核刚重启时 `limits valid=0x00` 是正常的；执行 `enable` 时客户端会从 `gimbal.conf` 自动恢复

仓库根目录的 `gimbal.conf` 是常用云台配置。先确认其中四个角度与当前实机标定结果一致；归位、关闭返回和普通动作的速度/力矩也在这里设置。若命令不是从仓库目录执行，可设置：

```bash
export GIMBAL_CONFIG=$HOME/openamp_rpmsg_comm/gimbal.conf
```

**2. 永久零点**

只有尚未设置永久零点，或机械结构发生变化时才执行。

人工把 yaw、pitch 放到真正的机械中位，摄像头朝向期望的正前方：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 setzero CONFIRM
sleep 3
sudo ./build/gimbal_test /dev/rpmsg0 status
```

这会写电机永久零点并重启驱动器，同时清空软件限位。不要每次启动都执行。

**3. 标定安全限位**

保持云台 `disabled`。手动移动对应轴，边界必须比真实碰撞位置提前留出余量，建议至少预留 5～10°。

首次执行 `limit` 时，如果驱动器 idle 状态没有周期反馈，工具会自动让两轴进入零力矩标定模式并等待实时角度。该过程不会发送位置目标；完成四个边界、执行 `disable`/`estop` 或 30 秒没有继续标定后会自动回到 idle。

依次移动并记录：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 limit yaw min CONFIRM
sudo ./build/gimbal_test /dev/rpmsg0 limit yaw max CONFIRM
sudo ./build/gimbal_test /dev/rpmsg0 limit pitch min CONFIRM
sudo ./build/gimbal_test /dev/rpmsg0 limit pitch max CONFIRM

sudo ./build/gimbal_test /dev/rpmsg0 status
```

必须看到：

```text
limits: valid=0x0f
```

同时确认：

- yaw 最小值 < 0，最大值 > 0
- pitch 最小值 < 0，最大值 > 0
- 数值确实对应安全范围

记下四个角度并写入仓库根目录的 `gimbal.conf`。以后从核重启后执行 `enable` 会自动恢复限位，也可直接手动恢复，例如：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 limits <yaw_min> <yaw_max> <pitch_min> <pitch_max> CONFIRM
```

不要直接照抄 README 中的示例角度。

**4. 验证未标定保护**

这项可在首次标定前做，也可先清除再恢复：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 reset-limits CONFIRM
sudo ./build/gimbal_test /dev/rpmsg0 enable 10
```

预期拒绝启动：

```text
status=limits-not-ready
fault=0x08
```

然后用记录的四个角度恢复限位。

**5. 验证启动归零**

托住相机，准备随时执行 `estop`：

如果 idle 状态没有周期反馈，`enable` 会先在零力矩模式唤醒两轴反馈，工具自动等待后再启动归零；不要连续重复执行 `enable`。

```bash
sudo ./build/gimbal_test /dev/rpmsg0 enable
```

`enable` 默认使用 `gimbal.conf` 的归位速度和 50% 力矩。后面的可选参数是仅本次生效的归位力矩百分比，允许范围为 `5..80`；普通位置命令允许 `1..80%`。负载较重且反馈持续正常但轴无法移动时，应逐级增加。调高前必须托住相机并确认软件限位正确，禁止直接使用 80% 试撞
机械限位。

另一个终端观察：

```bash
for i in $(seq 1 60); do
    sudo ./build/gimbal_test /dev/rpmsg0 status
    sleep 0.2
done
```

预期状态变化：

```text
starting -> homing -> active
```

归零应使用约 `5 rpm` 缓慢运动，并在12秒内进入 `active`。若方向错误、接近碰撞或异常发力，立即：

归零和 active 状态会每 50 ms 重发同一个位置目标，以持续获得驱动器反馈；任一轴超过 500 ms 没有新反馈才会进入反馈故障并自动 idle。

```bash
sudo ./build/gimbal_test /dev/rpmsg0 estop
```

**6. 小角度动作测试**

先使用低速、小角度和10%力矩：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 set 2 0 5 10
sudo ./build/gimbal_test /dev/rpmsg0 set -2 0 5 10
sudo ./build/gimbal_test /dev/rpmsg0 set 0 2 5 10
sudo ./build/gimbal_test /dev/rpmsg0 set 0 -2 5 10
sudo ./build/gimbal_test /dev/rpmsg0 center
```

确认：

- yaw/pitch 方向正确
- 两轴不会互相串动
- `target` 与 `feedback` 最终接近
- 状态始终为 `active`
- 没有异常电流、线缆拉扯或机械干涉

随后测试组合动作：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 set 3 3 5 10
sudo ./build/gimbal_test /dev/rpmsg0 set -3 -3 5 10
sudo ./build/gimbal_test /dev/rpmsg0 center
```

**7. 验证目标越界拒绝**

假设实测 yaw 最大限位是 `50°`，发送一个超过边界的目标：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 set 51 0 5 10
```

预期：

- 返回 `status=invalid`
- 云台不应执行这个目标
- 当前状态不应失控
- 原目标保持不变

不要通过推动运行中的云台越过真实边界来测试 `0x10`，这会增加损坏风险。

**8. 验证受控关闭**

```bash
sudo ./build/gimbal_test /dev/rpmsg0 disable
for i in $(seq 1 60); do
    sudo ./build/gimbal_test /dev/rpmsg0 status
    sleep 0.2
done
```

预期：

```text
active -> returning -> stopping -> disabled
```

从核会记录 `enable` 时的 pitch。关闭时保持当前 yaw，先按 `gimbal.conf` 的返回速度/力矩缓慢回到该 pitch，稳定后再逐步降低力矩。`status` 的 `startup return pitch` 可核对记录值。摄像头仍需托住，因为完全失能后是否下坠取决于重心和机械阻尼。

**9. 验证紧急停止**

再次启用并进入 `active`，执行一个很小的动作，然后立即：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 set 2 0 5 10
sudo ./build/gimbal_test /dev/rpmsg0 estop
sudo ./build/gimbal_test /dev/rpmsg0 status
```

预期立即进入 `disabled`，没有0.5秒缓降。

故障位含义：

```text
0x01 yaw反馈丢失
0x02 pitch反馈丢失
0x04 CAN发送失败
0x08 限位未配置或无效
0x10 实际位置越界
0x20 归零超时
0x40 流式命令超时
0x80 到位超时、疑似卡滞
```

完成这些验证后，再运行：

```bash
sudo ./build/gimbal_test /dev/rpmsg0 sweep 3 2
```

先从 `3°` 开始，不要直接使用大范围连续测试。
