# 从核 CAN 位时序问题总结

## 现象

飞腾派 Linux 主核通过 OpenAMP/rpmsg 给从核发送命令后，从核能够正常返回 ACK。

调试输出显示：

```text
init_ret=0
send_ret=0
can_id=0
baudrate=1000000
last can frame: id=0x601 dlc=8 data=25 00 00 01 F4 00 32 0A
```

这说明：

- rpmsg 通信正常；
- 从核已经收到命令；
- 从核生成的电机 CAN 帧内容正确；
- `FCanSend()` 返回成功。

但是一开始电机没有动作，并且 CAN 寄存器回读出现：

```text
tx_err=128 或 256
tx_fifo 持续增加
```

这说明 CAN 帧已经进入发送 FIFO，但总线上没有完成正常发送确认。

## 原因分析

CAN 总线不是只要波特率写成 `1000000` 就一定能正常通信。

CAN 的 1Mbps 还包含更细的位时序参数，例如：

- BRP：波特率预分频；
- Prop Seg：传播时间段；
- Phase Seg1；
- Phase Seg2；
- SJW：同步跳转宽度；
- Sample Point：采样点。

Linux SocketCAN 能让电机运动时，实际参数是：

```text
bitrate 1000000
sample-point 0.750
tq 50
prop-seg 7
phase-seg1 7
phase-seg2 5
sjw 2
brp 10
clock 200000000
```

而从核原先使用：

```c
arb_segment_config.auto_calc = TRUE;
arb_segment_config.baudrate = 1000000;
```

这会让 Phytium Standalone SDK 自动计算 CAN 位时序。自动计算出来的参数虽然也可能是 1Mbps，但不一定和 Linux SocketCAN 成功通信时的采样点、SJW、时间段完全一致。

对于 CAN 电机这种实时总线设备，位时序不完全匹配时，可能出现：

- 电机无法正确采样；
- 总线没有 ACK；
- CAN 控制器发送错误计数增加；
- TX FIFO 中数据堆积；
- `FCanSend()` 返回成功，但物理总线没有成功完成发送。

所以 `FCanSend()` 成功只能说明“写入控制器发送 FIFO 成功”，不能等价于“电机已经收到 CAN 帧”。

## 修改方法

将从核 CAN 初始化由自动计算改为手动配置，并与 Linux SocketCAN 验证成功的参数保持一致：

```c
arb_segment_config.baudrate = 1000000;
arb_segment_config.auto_calc = FALSE;
arb_segment_config.segment = FCAN_ARB_SEGMENT;
arb_segment_config.sample_point = 750;
arb_segment_config.prop_seg = 7;
arb_segment_config.phase_seg1 = 7;
arb_segment_config.phase_seg2 = 5;
arb_segment_config.sjw = 2;
arb_segment_config.brp = 10;
```

同时保持经典 CAN 模式：

```c
FCanFdEnable(&g_can, FALSE);
FCanSetMode(&g_can, FCAN_PROBE_NORMAL_MODE);
FCanBaudrateSet(&g_can, &arb_segment_config);
FCanEnable(&g_can, TRUE);
```

## 为什么修改后可以运动

修改后，从核 CAN 控制器的位时序与 Linux 成功测试时一致：

```text
1Mbps + 75% sample point + brp 10 + sjw 2
```

这样从核发出的 CAN 信号在电机端能够被正确采样，电机可以正常 ACK，CAN 控制器错误计数不再持续增加，发送 FIFO 不再异常堆积。

此外，测试时还确认了电机必须上电。如果电机未上电，总线上没有节点 ACK，也会出现类似：

```text
tx_err 上升
tx_fifo 增加
```

因此最终问题由两部分组成：

1. 从核 CAN 位时序需要与 Linux 已验证成功的参数一致；
2. 电机必须上电，否则 CAN 总线没有 ACK。

## 最终结论

本问题不是 rpmsg 协议问题，也不是电机控制帧内容错误。

真正的问题是：

```text
从核 CAN 控制器虽然写入发送 FIFO 成功，但原先自动计算的 CAN 位时序不够匹配，导致总线发送无法被电机正常确认。
```

解决后：

```text
Linux 主核通过 rpmsg 下发命令；
从核解析命令；
从核使用 FCAN 按固定 1Mbps 位时序发送 CAN 帧；
CAN 电机上电后能够正常响应并运动。
```

该结果说明飞腾派异构多核链路已经打通：

```text
Linux 主核 -> rpmsg -> OpenAMP 从核 -> FCAN -> CAN 电机
```

