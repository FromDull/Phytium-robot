# RPMsg 统一 Broker 使用说明

## 1. 架构

Linux 侧只允许 `rpmsg-broker` 直接打开 `/dev/rpmsg0`：

```text
/dev/rpmsg0
    |
rpmsg-broker
    |
    +-- rpmsg_client / rprun
    +-- balance_logger / rplog
    +-- gimbal-daemon
    +-- gimbal_test
    |
    +-- monitor.sock（只读广播）-- rpmsg-monitor -- 浏览器 :8092
```

Broker 使用 `/run/rpmsg-broker/rpmsg.sock` 接收完整 RPMsg 协议帧，给每个
从核事务重新分配 sequence，串行执行设备读写，再把原客户端 sequence 写回
响应。多个 Linux 程序可以同时使用协议，不会互相取走回复。

`/run/rpmsg-broker/monitor.sock` 是独立的只读 `SOCK_SEQPACKET` 旁路。它广播
主核 TX、从核 RX、超时、校验失败、序号不匹配、帧长度、原始十六进制数据和累计计数。
监视客户端不能通过该 socket 向从核发送命令；慢客户端使用非阻塞发送，绝不会阻塞控制事务。

## 2. 安装

保持平衡和云台失能，然后执行：

```bash
cd /home/user/openamp_rpmsg_comm
make broker client logger gimbal
sudo make install-rpmsg-broker
sudo make install-rpmsg-monitor
sudo make install-gimbal-daemon
sudo systemctl daemon-reload
sudo systemctl enable --now rpmsg-broker.service
sudo systemctl enable --now rpmsg-monitor.service
sudo systemctl enable --now gimbal-daemon.service
```

检查唯一设备持有者和服务状态：

```bash
sudo systemctl status rpmsg-broker.service --no-pager -l
sudo systemctl status gimbal-daemon.service --no-pager -l
sudo systemctl status rpmsg-monitor.service --no-pager -l
sudo lsof /dev/rpmsg0
```

`lsof` 中应只有 `rpmsg-broker` 持有 `/dev/rpmsg0`。

浏览器访问 `http://<飞腾派IP>:8092/`。机器人主仪表盘也可用 iframe 将该地址
作为“主从通信”页签；页面通过 SSE 接收数据，不轮询 `/dev/rpmsg0`。

## 3. 日常命令

原命令格式不变：

```bash
rprun heartbeat
rprun balance-status
rplog --rate 50
gimbalctl status
sudo ./build/gimbal_test /dev/rpmsg0 status
```

虽然 C 客户端参数仍写 `/dev/rpmsg0`，它们实际连接 Broker，不再自行打开字符
设备。普通命令不再需要 `sudo`；日志目录权限等操作仍按原工具要求执行。

## 4. 底层直连诊断

只有 Broker 已停止且所有上层服务已停止时，才允许显式直连：

```bash
sudo systemctl stop gimbal-daemon.service
sudo systemctl stop rpmsg-broker.service
sudo ./build/rpmsg_client direct:/dev/rpmsg0 heartbeat
```

完成后恢复：

```bash
sudo systemctl start rpmsg-broker.service
sudo systemctl start gimbal-daemon.service
```

不要在 Broker 运行时使用 `direct:/dev/rpmsg0`。

## 5. 固件重载

`rpmsg-broker.service` 是 `openamp-initproc.service` 的 `PartOf`，通过 systemd
重启从核时会一起重启。更新固件后的标准操作是：

```bash
sudo systemctl restart openamp-initproc.service
sudo systemctl status rpmsg-broker.service --no-pager
rprun heartbeat
```

## 6. 故障排查

```bash
sudo journalctl -u rpmsg-broker.service -n 100 --no-pager
sudo journalctl -u gimbal-daemon.service -n 100 --no-pager
ls -l /run/rpmsg-broker/rpmsg.sock
ls -l /run/rpmsg-broker/monitor.sock
curl http://127.0.0.1:8092/api/health
sudo lsof /dev/rpmsg0
```

如果 Broker 未运行，客户端会报告连接 `/run/rpmsg-broker/rpmsg.sock` 失败，
不会静默回退到直接访问 `/dev/rpmsg0`。
