# PWM 舵机无波形问题总结

## 1. 问题现象

在飞腾派 OpenAMP 从核固件中加入 PWM 舵机控制后，Linux 侧通过 `rpmsg_client` 下发舵机控制命令：

```bash
rprun servocenter CONFIRM
rprun servo 0 90 90 90 CONFIRM
rprun servo 180 90 90 90 CONFIRM
```

从 Linux 端回传结果看，OpenAMP 通信和从核 PWM API 调用均正常：

```text
servo debug: init_ret=0 last_ret=0 angles=0,90,90,90
servo debug: init_ret=0 last_ret=0 angles=180,90,90,90
```

但是 MG996R 舵机没有动作。使用逻辑分析仪测量 40Pin Pin32，也就是 `AG57 / PWM2_OUT`，发现引脚始终为高电平，没有 20 ms 周期 PWM 波形。

## 2. 排查过程

首先确认软件链路已经打通：

- `rpmsg_client` 能正常发送舵机命令。
- 从核能正常解析 `CMD_SERVO_SET4` 和 `CMD_SERVO_CENTER`。
- `phytium_servo_init()` 返回 `0`。
- `FPwmVariableSet()` 返回成功。
- `FPwmPulseSet()` 返回成功。
- 舵机角度状态能正常回传。

因此问题不在 rpmsg 通信，也不在命令协议，而是在 PWM 控制器输出配置或引脚复用。

随后检查飞腾 standalone SDK 的 `fpwm.h`，发现 `FPwmPolarity` 枚举定义如下：

```c
typedef enum
{
    FPWM_POLARITY_OUTPUT_HIGH = 0b000,
    FPWM_POLARITY_OUTPUT_LOW = 0b001,
    FPWM_POLARITY_OUTPUT_FLIP = 0b010,
    FPWM_POLARITY_INVERSED = 0b011,
    FPWM_POLARITY_NORMAL = 0b100,
    FPWM_POLARITY_CCR_LOW = 0b101,
    FPWM_POLARITY_CCR_HIGH = 0b110,
    FPWM_POLARITY_INIT = 0b111,
    FPWM_POLARITY_NUM
} FPwmPolarity;
```

原代码中使用的是：

```c
pwm_cfg.pwm_mode = 0;
pwm_cfg.pwm_polarity = 0;
```

这并不是正常 PWM 输出。`pwm_polarity = 0` 实际对应：

```c
FPWM_POLARITY_OUTPUT_HIGH
```

也就是固定输出高电平。这与逻辑分析仪看到的“一直高电平”完全一致。

## 3. 修改内容

将 PWM 模式和极性改为官方枚举值：

```c
pwm_cfg.pwm_mode = FPWM_OUTPUT_COMPARE;
pwm_cfg.pwm_polarity = FPWM_POLARITY_NORMAL;
```

同时保留前面已经完成的基础修正：

```c
FIOMuxInit();
FIOPadSetPwmMux(FPWM1_ID, 0);
```

当前舵机测试映射为：

```text
servo0 -> FPWM1 channel 0 -> PWM2_OUT -> AG57 -> 40Pin Pin32
```

PWM 参数为：

```text
参考时钟：50 MHz
分频值：49
计数频率：约 1 MHz
周期：20000
脉宽：500 ~ 2500
```

对应舵机控制波形：

```text
周期约 20 ms
0 度：约 0.5 ms 高电平
90 度：约 1.5 ms 高电平
180 度：约 2.5 ms 高电平
```

## 4. 结果

重新编译并部署从核固件后，再次使用逻辑分析仪测量 Pin32，PWM 波形恢复正常，舵机可以响应控制命令。

测试命令：

```bash
rprun servocenter CONFIRM
rprun servo 0 90 90 90 CONFIRM
rprun servo 180 90 90 90 CONFIRM
rprun servo 45 135 45 135 CONFIRM
```

Linux 端回传：

```text
servo debug: init_ret=0 last_ret=0 angles=0,90,90,90
servo debug: init_ret=0 last_ret=0 angles=180,90,90,90
servo debug: init_ret=0 last_ret=0 angles=90,90,90,90
```

逻辑分析仪可以观察到对应占空比变化的 PWM 波形。

## 5. 结论

本次问题的根因是误把 `pwm_polarity` 设置为裸数值 `0`。在飞腾 FPWM 驱动中，`0` 表示 `FPWM_POLARITY_OUTPUT_HIGH`，会导致 PWM 引脚固定输出高电平，而不是正常 PWM 波形。

正确做法是使用 SDK 中定义的枚举常量：

```c
pwm_cfg.pwm_mode = FPWM_OUTPUT_COMPARE;
pwm_cfg.pwm_polarity = FPWM_POLARITY_NORMAL;
```

后续在使用飞腾 standalone SDK 配置外设时，应尽量使用官方枚举值，避免直接写裸数值。对于 PWM 这类波形外设，API 返回成功只能说明寄存器配置被接受，最终是否输出正确波形还需要用逻辑分析仪或示波器确认。
