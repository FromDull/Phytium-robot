# 从核裸机固件代码说明

`slave_app.c` 是从核裸机端业务代码框架，用于集成到飞腾 Standalone OpenAMP 示例工程中，最终编译成：

```text
openamp_core0.elf
```

建议集成位置：

```text
phytium-pi-os/output/build/phytium-standalone-openamp-v1.0/example/system/amp/openamp_for_linux/src/slaver_00_example.c
```

当前实现内容：

- 解析 Linux 主核发送的命令帧。
- 支持心跳、CAN 电机、4 路 PWM 舵机和 BMI088 命令。
- 支持 `CMD_MOTOR_TEST` 初始化两台电机并各转动一圈。
- 支持 ID 1/2 轮电机的力矩控制和 CAN 反馈解析。
- 支持 BMI088 零偏校准、物理量换算和 pitch 互补滤波。
- 支持 100 Hz LQR、力矩限幅、反馈超时、倾倒和周期超时保护。
- 支持 `CMD_BALANCE_ENABLE`、`CMD_BALANCE_DISABLE` 和 `CMD_BALANCE_STATUS`。
- 支持 ID 3/4 云台状态机、原子双轴目标、反馈超时和软件边界保护。
- 支持云台安全边界标定、低速启动归零、受控力矩缓降关闭和紧急 idle。
- 校验失败或未知命令时丢弃消息。
- 返回 CAN、舵机、BMI088 和 LQR 的综合诊断状态。

说明：

由于飞腾 Standalone SDK 的 OpenAMP 初始化、resource table、IPI 和 rpmsg endpoint 创建代码由官方示例提供，本目录实现业务命令和硬件控制逻辑。集成时需要把本目录全部 `.c/.h` 文件复制到 `openamp_for_linux/src`，包括新增的 `gimbal_controller.c/.h`，并且：

1. RPMsg endpoint 创建后调用 `slave_app_init()`。
2. 主循环使用 `platform_poll_nonblocking()`，并持续调用 `slave_app_poll()`。
3. endpoint 销毁前调用 `slave_app_shutdown()`。

可直接参考：

```text
integration/phytium-pi-os/0001-openamp-enable-application-peripherals.patch
integration/phytium-pi-os/0002-openamp-connect-application-rpmsg-handler.patch
```
