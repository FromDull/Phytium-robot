# OpenAMP 主从实时通信页面

## 数据路径

```text
Linux clients -> rpmsg.sock -> rpmsg-broker -> /dev/rpmsg0 -> remote core
                                |
                                +-> monitor.sock (read-only)
                                      |
                                      +-> rpmsg-monitor -> SSE -> browser
```

监视路径只复制已发生的事务。它不能构造控制命令，也不直接打开 `/dev/rpmsg0`。
浏览器关闭、网络断开或监控程序处理过慢都不会改变平衡、云台或舵机控制时序。

## 安装

```bash
make broker
sudo make install-rpmsg-broker
sudo make install-rpmsg-monitor
sudo systemctl daemon-reload
sudo systemctl enable --now rpmsg-broker.service rpmsg-monitor.service
```

访问：

```text
http://<飞腾派IP>:8092/
```

健康检查：

```bash
curl http://127.0.0.1:8092/api/health
systemctl status rpmsg-monitor.service --no-pager -l
journalctl -u rpmsg-monitor.service -n 100 --no-pager
```

页面展示实时链路动画、TX/RX 帧率、平均和 P95 往返延迟、累计字节、异常数量、
命令类型分布、事务时间线以及原始帧十六进制检查器。命令名称由
`linux_user/rpmsg_monitor.py` 中的只读映射表解释，未知类型仍会按 `TYPE_n` 显示。

## 主仪表盘接入

在主仪表盘增加一个 iframe 页签，首次打开时设置：

```javascript
frame.src = `${location.protocol}//${location.hostname}:8092/`;
```

iframe 只加载可视化页面。控制站点与监控站点端口不同不会影响 SSE，且不需要把
Broker socket 暴露给容器或浏览器。
