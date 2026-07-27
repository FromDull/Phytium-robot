# Balance Position-Hold Outer Loop Handoff

## 1. Scope and current status

This change adds a cascaded position/velocity outer loop around the existing
pitch LQR inner loop. Its purpose is to keep the fixed-leg robot near its
arming position after the inner loop can already balance the body.

Implementation status:

- Source branch: `dev`
- Control implementation: `cdfe1d4 feat: add cascaded balance position hold`
- Telemetry implementation: `6911217 feat: log balance position hold telemetry`
- Host C tests, 26 Python tests, Linux clients, and the real AArch64 remote-core
  ELF build pass.
- Latest locally built ELF SHA-256:
  `3a57f93cac3fb3b045605d322e0770d7c9e8111ab3083ff3485398e850c39444`
- The feature has **not** been loaded on the board. The board root filesystem
  has confirmed ext4 metadata corruption and must be repaired first.

Do not run `reloadrproc` until the board filesystem is repaired and a newly
uploaded ELF has passed SHA-256 verification.

## 2. Controller architecture

The active control path is:

```text
position error + velocity error
              |
              v
 limited desired pitch (outer loop)
              |
              v
 existing K1/K2 pitch LQR (inner loop)
              |
              v
        left/right torque
```

The state convention is:

```text
position_error = measured_wheel_position - position_target
velocity_error = measured_wheel_velocity - applied_velocity_target

pitch_target = clamp(
    -(Kp * position_error + Kd * velocity_error),
    -pitch_limit,
    +pitch_limit)
```

The leading minus sign is intentional. Existing hardware logs show negative
pitch correlating with negative travel and positive pitch correlating with
positive travel. If the robot has moved in the positive direction, the outer
loop must therefore request a negative pitch to return it.

When position hold is enabled, `balance_control_poll()` calls:

```c
lqr_set_targets(&g_lqr, pitch_target, wheel_position, wheel_velocity);
```

Setting the LQR position and velocity targets equal to their current measured
values zeros the direct K3/K4 travel terms. This prevents the old direct
position feedback and the new pitch-target outer loop from fighting each
other. K3/K4 remain stored and immediately become active again when position
hold is disabled.

The commanded chassis velocity still works. Its ramp-limited velocity is
integrated into an independent `g_position_target_m`; the outer loop follows
that moving reference instead of always pulling the robot back to zero.

## 3. Safety and fallback behavior

- Position hold defaults to disabled after boot, remote-core reload, or
  `balance-reset-config`.
- Runtime settings are RAM-only; they do not survive `reloadrproc` or power
  loss.
- Position-hold configuration changes are accepted only while balance is
  disabled. The remote returns `busy` while arming or active.
- The desired pitch is independently limited to `0.1..3.0 deg`.
- Disabling position hold restores the original four-state LQR path without
  changing the configured K1..K4 gains.
- `balance-disable` remains the immediate motor stop path.

Fallback command, issued while balance is disabled:

```bash
sudo ./build/rpmsg_client /dev/rpmsg0 balance-position-hold off
```

## 4. Runtime command and units

Protocol command ID:

```text
CMD_BALANCE_SET_POSITION_HOLD = 65
```

Linux client syntax uses degrees for operator convenience:

```bash
sudo ./build/rpmsg_client /dev/rpmsg0 \
    balance-position-hold <kp_deg_per_m> <kd_deg_per_m_s> <limit_deg>
```

Example conservative starting point:

```bash
sudo ./build/rpmsg_client /dev/rpmsg0 \
    balance-position-hold 1.5 2.0 0.8
```

The wire protocol and firmware use radians:

| Payload offset | Size | Meaning |
| --- | ---: | --- |
| 0 | 1 | `0=disable`, `1=enable` |
| 1 | 4 | Kp in rad/m, signed big-endian integer scaled by 1e6 |
| 5 | 4 | Kd in rad/(m/s), signed big-endian integer scaled by 1e6 |
| 9 | 4 | pitch limit in rad, signed big-endian integer scaled by 1e6 |

The disable form sends only one payload byte with value zero.

Firmware validation limits are:

| Setting | Accepted range |
| --- | --- |
| Kp | `0..0.5 rad/m` (`0..28.6 deg/m`) |
| Kd | `0..1.0 rad/(m/s)` (`0..57.3 deg/(m/s)`) |
| Pitch limit | `0.1..3.0 deg` |

## 5. Configuration ACK version 5

Balance configuration replies are now version 5 and 60 bytes. Bytes `0..43`
retain their previous meanings.

| Offset | Size | Meaning |
| --- | ---: | --- |
| 44 | 4 | position Kp, rad/m scaled by 1e6 |
| 48 | 4 | velocity Kd, rad/(m/s) scaled by 1e6 |
| 52 | 4 | pitch limit, rad scaled by 1e6 |
| 56 | 1 | position hold enabled |
| 57 | 3 | reserved, zero |

`rpmsg_client balance-config` prints these values in degrees.

## 6. Balance telemetry version 2

Balance telemetry increased from 44 to 60 bytes. The original fields remain at
their original offsets.

| Offset | Size | Meaning |
| --- | ---: | --- |
| 0 | 1 | telemetry version (`2`) |
| 1 | 1 | balance state |
| 2 | 1 | balance fault |
| 3 | 1 | position hold enabled |
| 4..43 | 40 | original balance and motor telemetry |
| 44 | 4 | pitch target, rad scaled by 1e6 |
| 48 | 4 | position target, m scaled by 1e6 |
| 52 | 4 | position error, m scaled by 1e6 |
| 56 | 4 | velocity error, m/s scaled by 1e6 |

`balance_logger` version 1.1.0 writes the new fields to CSV:

```text
position_hold_enabled
pitch_target_rad
position_target_m
position_error_m
velocity_error_m_s
```

The new logger remains able to decode the old version-1 44-byte telemetry, so
it can be used before and after firmware deployment.

## 7. Main source locations

| File | Responsibility |
| --- | --- |
| `remote_firmware/position_hold_controller.[ch]` | Pure validated outer-loop calculation and saturation |
| `remote_firmware/balance_controller.c` | Target integration, state errors, outer/inner cascade, runtime config |
| `remote_firmware/balance_controller.h` | Runtime configuration and telemetry structures |
| `remote_firmware/slave_app.c` | Command 65 handling, config ACK v5, telemetry v2 encoding |
| `src/rpmsg_protocol.h` | Command ID and telemetry version/size |
| `linux_user/rpmsg_client.c` | Operator command, validation, config display |
| `linux_user/balance_logger.c` | Backward-compatible telemetry decode and CSV logging |
| `tests/position_hold_controller_test.c` | Sign, enable/disable, finite-value, and limit tests |
| `tests/test_protocol.c` | Command frame and telemetry version tests |

## 8. Current tuning baseline

The best inner-loop/trim region found before adding the outer loop was:

```text
trim = 1.25 deg
K1 = -5.20
K2 = -0.60
K3 = -0.10
K4 = -0.16
pitch-rate filter = 25 Hz
wheel speed limit = 0.60 m/s
single-wheel torque limit = 0.22 Nm
```

K3/K4 are bypassed while the new outer loop is enabled, but retaining the
values provides a known fallback when it is disabled.

Initial outer-loop setting:

```text
Kp = 1.5 deg/m
Kd = 2.0 deg/(m/s)
pitch limit = 0.8 deg
```

## 9. First hardware test after filesystem repair

Before every run:

1. Verify the deployed ELF hash and `remoteproc0=running`.
2. Verify `/dev/rpmsg0`, `rpmsg-broker.service`, and
   `gimbal-daemon.service`.
3. Enable the gimbal and require `state=3`, `fault=0`, and
   `feedback_valid_mask=3`.
4. Put the robot at the same marked starting location with clear travel space.
5. Keep an operator ready to issue `balance-disable`.

Apply the baseline while disabled:

```bash
rp_client=./build/rpmsg_client
rp_dev=/dev/rpmsg0

sudo "$rp_client" "$rp_dev" balance-disable
sudo "$rp_client" "$rp_dev" balance-trim 1.25
sudo "$rp_client" "$rp_dev" balance-gains -5.20 -0.60 -0.10 -0.16
sudo "$rp_client" "$rp_dev" balance-filter 25
sudo "$rp_client" "$rp_dev" balance-speed-limit 0.60
sudo "$rp_client" "$rp_dev" balance-torque-limit 0.22
sudo "$rp_client" "$rp_dev" balance-position-hold 1.5 2.0 0.8
sudo "$rp_client" "$rp_dev" balance-config
```

Then log a short run:

```bash
sudo ./build/balance_logger \
    --rate 20 \
    --duration 5 \
    --enable \
    --stop-on-fault \
    --output-dir logs/position-hold-kp150-kd200-lim080-5s
```

For the very first run, lift or catch the robot before enabling and verify that
a positive displacement produces a negative `pitch_target_rad`. Stop
immediately if the target has the same sign as the displacement error.

## 10. Parameter search and acceptance criteria

Keep the pitch limit fixed at `0.8 deg` for the first grid:

```text
Kp: 1.0, 1.5, 2.0 deg/m
Kd: 1.5, 2.0, 2.5 deg/(m/s)
```

Run each candidate at least three times from the same floor mark. Randomize the
order if possible so battery and motor temperature do not favor one setting.

Do not choose a winner from a single run. Rank the median of repeated runs by:

1. No natural fall, speed, CAN, IMU, or overrun fault.
2. Lowest peak and RMS absolute `position_error_m`.
3. Lowest final absolute position error.
4. Short settling time without sustained pitch or velocity oscillation.
5. Low pitch-target saturation ratio; repeated operation at exactly the pitch
   limit means the requested outer-loop authority is insufficient or gains are
   excessive.
6. Acceptable peak torque/current with margin below the configured limits.

The operator previously used fault `0x80` by physically stopping a robot that
had travelled too far. Treat that as an operator termination, not a natural
controller fault, and exclude roughly the final `0.35 s` from score
calculation. Also annotate any run in which a wheel contacts a shoe or another
obstacle; do not compare that run as clean tuning data.

Only increase the pitch limit after a Kp/Kd region is stable. Suggested limit
progression is `0.5 -> 0.8 -> 1.0 deg`; never increase gain and limit in the
same batch.

## 11. Deployment boundary and board recovery

The board checkout contains unrelated uncommitted work, including leg presets,
configuration, and logs. Do not reset, clean, or blindly pull it. Preserve
`gimbal.conf` and board-local work.

At handoff time:

- The live `/lib/firmware/openamp_core0.elf` is the old verified firmware with
  SHA-256
  `8bc676fc749032d53b05b84c722f94795c74d8e08b68de9fcbb336abf859c4e4`.
- `/home/user/openamp_core0.elf` was corrupted during an attempted write and
  must not be installed.
- Kernel logs reported ext4 inode checksum, journal, directory checksum, block
  bitmap, and filesystem CRC failures on the board root partition.
- The SD card is being repaired offline as `/dev/sdb1` in a Linux VM using
  e2fsprogs 1.47.2.

After filesystem repair:

1. Boot the board and inspect `dmesg`/kernel journal for new ext4 errors.
2. Re-upload the ELF to a temporary filename using checksum-capable transfer.
3. Compare local and board SHA-256 before moving or installing it.
4. Rebuild/install the new Linux client and logger without overwriting the
   board's unrelated dirty source changes.
5. Only then run `reloadrproc`.
6. Re-enable the gimbal and reapply all RAM-only balance and chassis settings.
7. Verify config ACK v5 and telemetry v2 before enabling balance.

If file hashes change after a local move/copy again, stop immediately and
replace the SD card rather than loading the firmware.
