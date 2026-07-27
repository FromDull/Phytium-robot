# Gimbal daemon

`rpmsg-broker` is the single Linux owner of `/dev/rpmsg0`. `gimbal-daemon`
connects to the broker and provides low-frequency voice and operator commands.
It does not expose calibration, permanent-zero, limit-reset, sweep, or test
commands.

The first deployment is intentionally conservative:

- absolute yaw and pitch targets only;
- business workspace limited to +/-10 degrees on each axis;
- configured movement speed (currently 20 rpm);
- configured movement torque (currently 80 percent);
- every target requires active state, no fault, complete limits, valid
  feedback, and feedback younger than 500 ms;
- enable and disable require explicit confirmation;
- emergency stop is always available.

This service is suitable for low-frequency voice pointing. It is not yet the
2-4 Hz visual-tracking interface because target commands currently use a zero
streaming timeout.

## Install

Keep the gimbal disabled, then run:

```bash
cd /home/user/openamp_rpmsg_comm
sudo make install-rpmsg-broker
sudo make install-gimbal-daemon
sudo systemctl daemon-reload
sudo systemctl enable --now rpmsg-broker.service
sudo systemctl enable --now gimbal-daemon.service
```

Check the service and read status before enabling motion:

```bash
sudo systemctl status gimbal-daemon.service --no-pager -l
gimbalctl status
```

Stop `gimbal-daemon` before calibration or test commands that alter the gimbal
state. Keep `rpmsg-broker` running; `gimbal_test` also uses the broker.

## Apply configuration changes

The daemon reads `/home/user/openamp_rpmsg_comm/gimbal.conf` only when it
starts. It does not hot-reload this file. After changing only `gimbal.conf`,
use the following safe sequence:

```bash
gimbalctl disable --confirm
sleep 5
gimbalctl status

sudo systemctl restart gimbal-daemon.service
sudo systemctl status gimbal-daemon.service --no-pager -l
gimbalctl status
```

Support the camera, verify that the reported limits and configuration are
correct, and then enable motion again:

```bash
gimbalctl enable --confirm
```

If the service does not start, inspect the configuration or validation error:

```bash
sudo journalctl -u gimbal-daemon.service -n 50 --no-pager
```

An invalid, incomplete, duplicated, or unknown configuration field makes the
daemon fail at startup. Fix `gimbal.conf` and restart the service again.

Changing only `gimbal.conf` does **not** require `make`, `makeelf`,
`reloadrproc`, or `systemctl daemon-reload`. These configuration values belong
to the Linux gimbal daemon and do not change the motor driver's PID settings.
Use `systemctl daemon-reload` only after changing the systemd unit file, and
use `reloadrproc` only after installing a new remote-core firmware image.

Both `move_speed_rpm` and `move_torque_percent` apply to `set` and `center`
after the daemon restarts.

The pitch measured when `enable` starts is the normal shutdown return
position. It may be a folded or resting camera position far from zero, but it
must remain inside the calibrated `pitch_min_deg..pitch_max_deg` range. Normal
`set` and `center` targets still keep the configured 3 degree workspace margin.
If the startup pitch is outside the calibrated range, enable is rejected and
the gimbal is emergency-stopped.

## Manual acceptance

Support the camera and keep another terminal ready to run:

```bash
gimbalctl estop
```

Enable and inspect status:

```bash
gimbalctl enable --confirm
sleep 5
gimbalctl status
```

Continue only when state is 3 (active), fault is zero, limits mask is 15,
feedback mask is 3, and both feedback ages are below 500 ms.

Run one command at a time and inspect status after each command:

```bash
gimbalctl set 2 0
sleep 2
gimbalctl status

gimbalctl center
sleep 2
gimbalctl status

gimbalctl set 0 2
sleep 2
gimbalctl status
```

Return to center and shut down normally:

```bash
gimbalctl center
sleep 2
gimbalctl disable --confirm
sleep 5
gimbalctl status
```

## Socket API

The socket is `/run/gimbal-daemon/gimbal.sock`. Each connection sends one JSON
object followed by a newline and receives one JSON object followed by a newline.

Examples:

```json
{"command":"status"}
{"command":"enable","confirm":true}
{"command":"set","yaw_deg":2.0,"pitch_deg":-2.0}
{"command":"center"}
{"command":"disable","confirm":true}
{"command":"estop"}
```

Yaw positive is counter-clockwise. Pitch positive looks upward.
