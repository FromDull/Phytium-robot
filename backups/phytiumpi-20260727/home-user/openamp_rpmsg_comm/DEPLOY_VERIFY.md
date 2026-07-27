# OpenAMP 核间通信部署验证步骤

本项目验证分为三层：

1. PC/Ubuntu 上验证命令帧协议是否正确。
2. 飞腾派板上验证 `remoteproc0`、`openamp_core0.elf` 和 rpmsg 通道是否可用。
3. 将本项目从核业务代码接入 `openamp_core0.elf` 后，验证 Linux 主核和从核之间的自定义命令帧。

## 1. PC 或 Ubuntu 协议验证

进入项目目录：

```bash
cd /path/to/code/openamp_rpmsg_comm
```

编译并运行测试：

```bash
make clean
make test
./build/test_protocol
```

预期输出：

```text
rpmsg protocol tests passed
```

说明：

- `src/rpmsg_protocol.c`：命令帧编码、解码和校验。
- `tests/test_protocol.c`：验证正常帧和错误校验帧。

## 2. 飞腾派基础通道验证

确认从核固件存在：

```bash
ls -lh /lib/firmware/openamp_core0.elf
```

查看 `remoteproc0`：

```bash
cat /sys/class/remoteproc/remoteproc0/firmware
cat /sys/class/remoteproc/remoteproc0/state
```

如果状态是 `offline`，启动从核：

```bash
echo start | sudo tee /sys/class/remoteproc/remoteproc0/state
```

查看状态和日志：

```bash
cat /sys/class/remoteproc/remoteproc0/state
dmesg | tail -n 80
```

预期能看到：

```text
remote processor homo_core0 is now up
creating channel rpmsg-openamp-demo-channel
```

查看 rpmsg 设备：

```bash
ls /sys/bus/rpmsg/devices/
ls /dev/rpmsg*
ls /dev/rpmsg_ctrl*
```

如果没有 `/dev/rpmsgX`，绑定字符设备驱动：

```bash
cd /sys/bus/rpmsg/devices/virtio0.rpmsg-openamp-demo-channel.-1.0
echo rpmsg_chrdev | sudo tee driver_override
sudo modprobe rpmsg_char
ls /dev/rpmsg*
```

说明：

- 这一步验证的是飞腾派 OpenAMP 基础链路。
- 如果使用官方 demo 固件，通常可运行 `rpmsg-demo-single` 看到 Hello World 回包。
- 官方 demo 不一定识别本项目自定义二进制命令帧。

## 3. 编译 Linux 端测试程序

把本项目拷贝到飞腾派，例如：

```bash
scp -r openamp_rpmsg_comm user@<board-ip>:/home/user/
```

在飞腾派上编译 Linux 端客户端：

```bash
cd /home/user/openamp_rpmsg_comm
gcc linux_user/rpmsg_client.c src/rpmsg_protocol.c -I./src -o rpmsg_client
```

运行：

```bash
sudo ./rpmsg_client /dev/rpmsg0
```

预期输出：

```text
heartbeat frame sent to /dev/rpmsg0
```

注意：

- 当前 `rpmsg_client.c` 只发送自定义 heartbeat 命令帧。
- 如果从核仍是官方 Hello World demo，可能只证明 Linux 用户态成功写入 rpmsg 设备。
- 要验证从核解析并回包，需要执行下一步。

## 4. 接入从核裸机业务代码

从核业务代码在：

```text
remote_firmware/slave_app.c
```

建议集成到飞腾 Pi OS 编译目录：

```text
phytium-pi-os/output/build/phytium-standalone-openamp-v1.0/example/system/amp/openamp_for_linux/src/slaver_00_example.c
```

集成方式：

1. 复制 `src/rpmsg_protocol.h` 和 `src/rpmsg_protocol.c` 到从核工程。
2. 复制 `remote_firmware` 下的业务源码和头文件，包括 `slave_app`、平衡控制、云台控制及硬件适配文件。
3. 在原 OpenAMP rpmsg 回调中调用：

```c
uint8_t reply[32];
size_t reply_len = slave_handle_frame(data, len, reply, sizeof(reply));
if (reply_len > 0) {
    rpmsg_send(ept, reply, reply_len);
}
```

4. 在装有飞腾 SDK 的编译机上增量编译从核固件；不要在飞腾派本机执行：

```bash
cd ~/phytium-work/openamp_rpmsg_comm
make firmware-elf
```

5. 拷贝新固件到飞腾派：

```bash
scp ../phytium-pi-os/output/build/phytium-standalone-openamp-v1.0/example/system/amp/openamp_for_linux/phytiumpi_aarch64_firefly_openamp_core0.elf \
    user@<board-ip>:/home/user/openamp_core0.elf
```

6. 在飞腾派上替换固件：

```bash
echo stop | sudo tee /sys/class/remoteproc/remoteproc0/state
sudo cp /home/user/openamp_core0.elf /lib/firmware/openamp_core0.elf
sync
echo start | sudo tee /sys/class/remoteproc/remoteproc0/state
```

## 5. 完整验收现象

完整验证通过时，应能说明：

- `remoteproc0` 可以启动从核固件。
- `dmesg` 中出现 rpmsg 通道创建日志。
- Linux 端 `rpmsg_client` 可以发送自定义命令帧。
- 从核端在 `slave_handle_frame()` 中解析命令。
- 错误帧或安全停止命令会进入安全状态。

## 6. 常见问题

### `Permission denied`

写 `/sys/class/remoteproc/.../state` 时需要 root 权限：

```bash
echo start | sudo tee /sys/class/remoteproc/remoteproc0/state
```

### 找不到 `/dev/rpmsg0`

先确认通道存在：

```bash
ls /sys/bus/rpmsg/devices/
```

再绑定：

```bash
echo rpmsg_chrdev | sudo tee driver_override
sudo modprobe rpmsg_char
```

### `rpmsg_client` 发送后没有回包

当前 Linux 客户端只发送命令。若从核仍是官方 demo，它不一定识别本项目帧格式。需要把 `remote_firmware/slave_app.c` 集成进从核固件后再验证自定义协议。

