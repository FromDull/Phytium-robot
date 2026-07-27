#include "../src/rpmsg_protocol.h"
#include "../src/rpmsg_transport.h"
#include "../src/leg_joint_mapping.h"

#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#define RPMSG_CLIENT_VERSION "0.25.0-position-hold"
#define RAD_PER_DEG 0.017453292519943295f

static int wait_readable(int fd, int timeout_ms)
{
    fd_set rfds;
    struct timeval tv;

    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    return select(fd + 1, &rfds, NULL, NULL, &tv);
}

static void put_be_i32(uint8_t *p, int32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)(value & 0xff);
}

static void put_be_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)(value & 0xff);
}

static int32_t scaled_i32(float value, float scale)
{
    float scaled = value * scale;
    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static uint32_t read_be_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

static int32_t read_be_i32(const uint8_t *p)
{
    return (int32_t)read_be_u32(p);
}

static uint16_t read_be_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void print_motor_fault_bits(uint32_t code)
{
    static const struct {
        uint32_t bit;
        const char *name;
    } faults[] = {
        {0x00000001U, "power/calibration-current"},
        {0x00000002U, "phase-resistance"},
        {0x00000008U, "calibration-current-ripple"},
        {0x00000010U, "phase-inductance"},
        {0x00000020U, "encoder-bandwidth"},
        {0x00000040U, "encoder-spi"},
        {0x00000080U, "encoder-type"},
        {0x00000100U, "hall-not-calibrated"},
        {0x00000200U, "encoder-no-data"},
        {0x00000400U, "encoder-cpr"},
        {0x00000800U, "run-state"},
        {0x00008000U, "hall-signal"},
        {0x00020000U, "secondary-encoder"},
        {0x00080000U, "gate-driver"},
        {0x00100000U, "mos-overtemperature"},
        {0x00200000U, "motor-overtemperature"},
        {0x00400000U, "undervoltage"},
        {0x00800000U, "overvoltage"},
        {0x01000000U, "overcurrent"},
    };
    int found = 0;

    if (code == 0U) {
        printf("motor fault decoded: none\n");
        return;
    }
    printf("motor fault decoded:");
    for (size_t i = 0; i < sizeof(faults) / sizeof(faults[0]); ++i) {
        if ((code & faults[i].bit) != 0U) {
            printf(" %s", faults[i].name);
            found = 1;
        }
    }
    if (!found) {
        printf(" unknown-bits");
    }
    printf("\n");
}

static void usage(const char *prog)
{
    printf("Usage:\n");
    printf("  %s <rpmsg_dev> heartbeat\n", prog);
    printf("  %s <rpmsg_dev> enable <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> zero <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> setorigin <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> mode <motor_id> <mode>\n", prog);
    printf("  %s <rpmsg_dev> init <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> test\n", prog);
    printf("  %s <rpmsg_dev> torque-test <motor_id> <torque_nm> [duration_ms]\n", prog);
    printf("  %s <rpmsg_dev> motor-fault <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> motor-speed-diag <motor_id> <torque_nm> [duration_ms]\n", prog);
    printf("  %s <rpmsg_dev> servo <s0_deg> <s1_deg> <s2_deg> <s3_deg> CONFIRM\n", prog);
    printf("  %s <rpmsg_dev> servopol <0..7> CONFIRM\n", prog);
    printf("  %s <rpmsg_dev> servocenter CONFIRM  # legacy name: raw safe reference 45,135,45,135\n", prog);
    printf("  %s <rpmsg_dev> servo-move <duration_ms> <s0_deg> <s1_deg> <s2_deg> <s3_deg>\n", prog);
    printf("  %s <rpmsg_dev> leg-joints <duration_ms> <right_front_deg> <right_rear_deg> <left_front_deg> <left_rear_deg>\n", prog);
    printf("  %s <rpmsg_dev> leg-enable CONFIRM\n", prog);
    printf("  %s <rpmsg_dev> servo-test <1..4> <0..180_deg> <100..10000_ms> CONFIRM\n", prog);
    printf("  %s <rpmsg_dev> servo-status\n", prog);
    printf("  %s <rpmsg_dev> servo-stop  # cancel motion and disable all servo PWM outputs\n", prog);
    printf("  %s <rpmsg_dev> imuinit\n", prog);
    printf("  %s <rpmsg_dev> imuread\n", prog);
    printf("  %s <rpmsg_dev> imu-attitude\n", prog);
    printf("  %s <rpmsg_dev> imu-calibrate [20..500_samples]\n", prog);
    printf("  %s <rpmsg_dev> balance-enable\n", prog);
    printf("  %s <rpmsg_dev> balance-disable\n", prog);
    printf("  %s <rpmsg_dev> balance-status\n", prog);
    printf("  %s <rpmsg_dev> balance-trim <upright_pitch_deg>\n", prog);
    printf("  %s <rpmsg_dev> balance-gains <k_theta> <k_theta_rate> <k_position> <k_velocity>\n", prog);
    printf("  %s <rpmsg_dev> balance-config\n", prog);
    printf("  %s <rpmsg_dev> balance-reset-config\n", prog);
    printf("  %s <rpmsg_dev> balance-speed-limit <0.2..2.0_m_s>\n", prog);
    printf("  %s <rpmsg_dev> balance-filter <5..40_hz>\n", prog);
    printf("  %s <rpmsg_dev> balance-posture-angle <1..10_deg>\n", prog);
    printf("  %s <rpmsg_dev> balance-torque-limit <0.05..0.30_nm>\n", prog);
    printf("  %s <rpmsg_dev> balance-position-hold off\n", prog);
    printf("  %s <rpmsg_dev> balance-position-hold <0..28.6_kp_deg_m> <0..57.3_kd_deg_m_s> <0.1..3_limit_deg>\n", prog);
    printf("  %s <rpmsg_dev> chassis-track <0.08..0.50_m>\n", prog);
    printf("  %s <rpmsg_dev> chassis-velocity <-0.4..0.4_m_s> <-1..1_rad_s> [100..1000_timeout_ms]\n", prog);
    printf("  %s <rpmsg_dev> chassis-status\n", prog);
    printf("  %s <rpmsg_dev> pvt <motor_id> <pos_x100_deg> <speed_rpm> <torque_percent>\n", prog);
    printf("  %s <rpmsg_dev> stop [motor_id]\n", prog);
    printf("\nExamples:\n");
    printf("  %s /dev/rpmsg0 heartbeat\n", prog);
    printf("  %s /dev/rpmsg0 init 1\n", prog);
    printf("  %s /dev/rpmsg0 test\n", prog);
    printf("  %s /dev/rpmsg0 servo 45 135 45 135 CONFIRM\n", prog);
    printf("  %s /dev/rpmsg0 servopol 4 CONFIRM\n", prog);
    printf("  %s /dev/rpmsg0 servocenter CONFIRM\n", prog);
    printf("  %s /dev/rpmsg0 servo-move 3000 85 95 85 95\n", prog);
    printf("  %s /dev/rpmsg0 leg-enable CONFIRM\n", prog);
    printf("  %s /dev/rpmsg0 servo-test 3 45 1000 CONFIRM\n", prog);
    printf("  %s /dev/rpmsg0 leg-joints 3000 43 43 43 43\n", prog);
    printf("  %s /dev/rpmsg0 servo-status\n", prog);
    printf("  %s /dev/rpmsg0 servo-stop\n", prog);
    printf("  %s /dev/rpmsg0 imuinit\n", prog);
    printf("  %s /dev/rpmsg0 imuread\n", prog);
    printf("  watch -n 0.1 '%s /dev/rpmsg0 imu-attitude'\n", prog);
    printf("  %s /dev/rpmsg0 imu-calibrate 100\n", prog);
    printf("  watch -n 0.1 '%s /dev/rpmsg0 imuread'\n", prog);
    printf("  %s /dev/rpmsg0 balance-enable\n", prog);
    printf("  watch -n 0.1 '%s /dev/rpmsg0 balance-status'\n", prog);
    printf("  %s /dev/rpmsg0 balance-disable\n", prog);
    printf("  %s /dev/rpmsg0 balance-trim 1.0\n", prog);
    printf("  %s /dev/rpmsg0 balance-config\n", prog);
    printf("  %s /dev/rpmsg0 balance-speed-limit 1.2\n", prog);
    printf("  %s /dev/rpmsg0 balance-filter 20\n", prog);
    printf("  %s /dev/rpmsg0 balance-posture-angle 3\n", prog);
    printf("  %s /dev/rpmsg0 balance-torque-limit 0.22\n", prog);
    printf("  %s /dev/rpmsg0 balance-position-hold 1.5 2.0 0.8\n", prog);
    printf("  %s /dev/rpmsg0 chassis-track 0.18\n", prog);
    printf("  %s /dev/rpmsg0 chassis-velocity 0.10 0.0 300\n", prog);
    printf("  %s /dev/rpmsg0 chassis-status\n", prog);
    printf("  %s /dev/rpmsg0 enable 1\n", prog);
    printf("  %s /dev/rpmsg0 pvt 1 1000 100 20\n", prog);
    printf("  %s /dev/rpmsg0 stop 1\n", prog);
}

static int build_command(int argc, char **argv, uint8_t *type, uint8_t *payload, uint8_t *payload_len)
{
    const char *cmd = argc > 2 ? argv[2] : "heartbeat";

    *payload_len = 0;
    if (strcmp(cmd, "heartbeat") == 0) {
        *type = CMD_HEARTBEAT;
        payload[0] = 0;
        *payload_len = 1;
        return 0;
    }

    if (strcmp(cmd, "enable") == 0) {
        if (argc < 4) return -1;
        *type = CMD_CAN_ENABLE;
        payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
        *payload_len = 1;
        return 0;
    }

    if (strcmp(cmd, "zero") == 0) {
        if (argc < 4) return -1;
        *type = CMD_CAN_ZERO_POSITION;
        payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
        *payload_len = 1;
        return 0;
    }

    if (strcmp(cmd, "setorigin") == 0) {
        if (argc < 4) return -1;
        *type = CMD_CAN_SET_ORIGIN;
        payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
        *payload_len = 1;
        return 0;
    }

    if (strcmp(cmd, "mode") == 0) {
        if (argc < 5) return -1;
        *type = CMD_CAN_SET_MODE;
        payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
        put_be_u16(&payload[1], (uint16_t)strtoul(argv[4], NULL, 0));
        *payload_len = 3;
        return 0;
    }

    if (strcmp(cmd, "init") == 0) {
        if (argc < 4) return -1;
        *type = CMD_CAN_INIT_MOTOR;
        payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
        *payload_len = 1;
        return 0;
    }

    if (strcmp(cmd, "test") == 0) {
        *type = CMD_MOTOR_TEST;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "torque-test") == 0) {
        char *end = NULL;
        unsigned long motor_id;
        unsigned long duration_ms = 200U;
        float torque_nm;
        int16_t torque_x100;

        if (argc < 5) return -1;
        motor_id = strtoul(argv[3], &end, 0);
        if (end == argv[3] || *end != '\0' || motor_id < 1U || motor_id > 2U) {
            return -1;
        }
        end = NULL;
        torque_nm = strtof(argv[4], &end);
        if (end == argv[4] || *end != '\0' || !isfinite(torque_nm) ||
            torque_nm < -0.22f || torque_nm > 0.22f) {
            return -1;
        }
        if (argc >= 6) {
            end = NULL;
            duration_ms = strtoul(argv[5], &end, 0);
            if (end == argv[5] || *end != '\0' ||
                duration_ms < 20U || duration_ms > 2000U) {
                return -1;
            }
        }
        torque_x100 = (int16_t)(torque_nm >= 0.0f ?
            torque_nm * 100.0f + 0.5f : torque_nm * 100.0f - 0.5f);
        *type = CMD_CAN_TORQUE_TEST;
        payload[0] = (uint8_t)motor_id;
        put_be_u16(&payload[1], (uint16_t)torque_x100);
        put_be_u16(&payload[3], (uint16_t)duration_ms);
        *payload_len = 5U;
        return 0;
    }

    if (strcmp(cmd, "motor-fault") == 0) {
        char *end = NULL;
        unsigned long motor_id;

        if (argc < 4) return -1;
        motor_id = strtoul(argv[3], &end, 0);
        if (end == argv[3] || *end != '\0' || motor_id < 1U || motor_id > 4U) {
            return -1;
        }
        *type = CMD_CAN_MOTOR_FAULT;
        payload[0] = (uint8_t)motor_id;
        *payload_len = 1U;
        return 0;
    }

    if (strcmp(cmd, "motor-speed-diag") == 0) {
        char *end = NULL;
        unsigned long motor_id;
        unsigned long duration_ms = 1000U;
        float torque_nm;
        int16_t torque_x100;

        if (argc < 5) return -1;
        motor_id = strtoul(argv[3], &end, 0);
        if (end == argv[3] || *end != '\0' || motor_id < 1U || motor_id > 2U) {
            return -1;
        }
        end = NULL;
        torque_nm = strtof(argv[4], &end);
        if (end == argv[4] || *end != '\0' || !isfinite(torque_nm) ||
            torque_nm < -0.10f || torque_nm > 0.10f) {
            return -1;
        }
        if (argc >= 6) {
            end = NULL;
            duration_ms = strtoul(argv[5], &end, 0);
            if (end == argv[5] || *end != '\0' ||
                duration_ms < 200U || duration_ms > 2000U) {
                return -1;
            }
        }
        torque_x100 = (int16_t)(torque_nm >= 0.0f ?
            torque_nm * 100.0f + 0.5f : torque_nm * 100.0f - 0.5f);
        *type = CMD_CAN_SPEED_DIAG;
        payload[0] = (uint8_t)motor_id;
        put_be_u16(&payload[1], (uint16_t)torque_x100);
        put_be_u16(&payload[3], (uint16_t)duration_ms);
        *payload_len = 5U;
        return 0;
    }

    if (strcmp(cmd, "servo") == 0) {
        if (argc != 8 || strcmp(argv[7], "CONFIRM") != 0) return -1;
        *type = CMD_SERVO_SET4;
        for (int i = 0; i < 4; ++i) {
            unsigned long angle = strtoul(argv[3 + i], NULL, 0);
            if (angle > 180) angle = 180;
            put_be_u16(&payload[i * 2], (uint16_t)angle);
        }
        *payload_len = 8;
        return 0;
    }

    if (strcmp(cmd, "servocenter") == 0) {
        if (argc != 4 || strcmp(argv[3], "CONFIRM") != 0) return -1;
        *type = CMD_SERVO_CENTER;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "servopol") == 0) {
        if (argc != 5 || strcmp(argv[4], "CONFIRM") != 0) {
            return -1;
        }
        *type = CMD_SERVO_POLARITY;
        payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
        *payload_len = 1;
        return 0;
    }

    if (strcmp(cmd, "leg-enable") == 0) {
        if (argc != 4 || strcmp(argv[3], "CONFIRM") != 0) {
            return -1;
        }
        printf("WARNING: support the robot; PWM servos have no position feedback.\n");
        printf("enabling fixed safe command: effective RF,RR,LF,LR=45,45,45,45 deg; raw=45,135,45,135 deg\n");
        *type = CMD_LEG_ENABLE;
        *payload_len = 0U;
        return 0;
    }

    if (strcmp(cmd, "servo-test") == 0) {
        char *end = NULL;
        unsigned long servo_number;
        unsigned long duration_ms;
        float angle;

        if (argc != 7 || strcmp(argv[6], "CONFIRM") != 0) return -1;
        servo_number = strtoul(argv[3], &end, 0);
        if (end == argv[3] || *end != '\0' ||
            servo_number < 1U || servo_number > 4U) return -1;
        end = NULL;
        angle = strtof(argv[4], &end);
        if (end == argv[4] || *end != '\0' || !isfinite(angle) ||
            angle < 0.0f || angle > 180.0f) return -1;
        end = NULL;
        duration_ms = strtoul(argv[5], &end, 0);
        if (end == argv[5] || *end != '\0' ||
            duration_ms < 100U || duration_ms > 10000U) return -1;
        printf("WARNING: testing only raw servo %lu at %.1f deg for %lu ms; support the linkage.\n",
               servo_number, (double)angle, duration_ms);
        payload[0] = (uint8_t)(servo_number - 1U);
        put_be_u16(&payload[1], (uint16_t)(angle * 10.0f + 0.5f));
        put_be_u16(&payload[3], (uint16_t)duration_ms);
        *type = CMD_SERVO_TEST_ONE;
        *payload_len = 5U;
        return 0;
    }

    if (strcmp(cmd, "servo-move") == 0 || strcmp(cmd, "leg-joints") == 0) {
        char *end = NULL;
        unsigned long duration_ms;
        uint16_t angle_x10[LEG_JOINT_COUNT];
        uint16_t raw_x10[LEG_JOINT_COUNT];

        if (argc < 8) return -1;
        duration_ms = strtoul(argv[3], &end, 0);
        if (end == argv[3] || *end != '\0' ||
            duration_ms < 100U || duration_ms > 10000U) {
            return -1;
        }
        for (int i = 0; i < 4; ++i) {
            float angle;

            end = NULL;
            angle = strtof(argv[4 + i], &end);
            float max_angle = strcmp(cmd, "leg-joints") == 0 ? 45.0f : 180.0f;

            if (end == argv[4 + i] || *end != '\0' || !isfinite(angle) ||
                angle < 0.0f || angle > max_angle) {
                return -1;
            }
            angle_x10[i] = (uint16_t)(angle * 10.0f + 0.5f);
        }
        if (strcmp(cmd, "leg-joints") == 0) {
            if (leg_effective_to_servo_raw_x10(angle_x10, raw_x10) != 0) {
                return -1;
            }
            printf("leg target effective RF,RR,LF,LR=%.1f,%.1f,%.1f,%.1f deg -> raw servo 1,2,3,4=%.1f,%.1f,%.1f,%.1f deg\n",
                   (double)angle_x10[0] / 10.0,
                   (double)angle_x10[1] / 10.0,
                   (double)angle_x10[2] / 10.0,
                   (double)angle_x10[3] / 10.0,
                   (double)raw_x10[0] / 10.0,
                   (double)raw_x10[1] / 10.0,
                   (double)raw_x10[2] / 10.0,
                   (double)raw_x10[3] / 10.0);
        } else {
            memcpy(raw_x10, angle_x10, sizeof(raw_x10));
        }
        for (int i = 0; i < 4; ++i) {
            put_be_u16(&payload[i * 2], raw_x10[i]);
        }
        put_be_u16(&payload[8], (uint16_t)duration_ms);
        *type = strcmp(cmd, "leg-joints") == 0 ?
            CMD_LEG_MOVE4 : CMD_SERVO_MOVE4;
        *payload_len = 10U;
        return 0;
    }

    if (strcmp(cmd, "servo-status") == 0) {
        *type = CMD_SERVO_STATUS;
        *payload_len = 0U;
        return 0;
    }

    if (strcmp(cmd, "servo-stop") == 0) {
        *type = CMD_SERVO_STOP;
        *payload_len = 0U;
        return 0;
    }

    if (strcmp(cmd, "imuinit") == 0) {
        *type = CMD_IMU_INIT;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "imuread") == 0) {
        *type = CMD_IMU_READ;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "imu-attitude") == 0) {
        *type = CMD_IMU_TELEMETRY;
        *payload_len = 0U;
        return 0;
    }

    if (strcmp(cmd, "imu-calibrate") == 0) {
        char *end = NULL;
        unsigned long samples = 100U;

        if (argc >= 4) {
            samples = strtoul(argv[3], &end, 0);
            if (end == argv[3] || *end != '\0' ||
                samples < 20U || samples > 500U) {
                return -1;
            }
        }
        *type = CMD_IMU_CALIBRATE;
        put_be_u16(payload, (uint16_t)samples);
        *payload_len = 2U;
        return 0;
    }

    if (strcmp(cmd, "balance-enable") == 0) {
        *type = CMD_BALANCE_ENABLE;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "balance-disable") == 0) {
        *type = CMD_BALANCE_DISABLE;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "balance-status") == 0) {
        *type = CMD_BALANCE_STATUS;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "balance-trim") == 0) {
        char *end = NULL;
        float trim_deg;

        if (argc < 4) return -1;
        trim_deg = strtof(argv[3], &end);
        if (end == argv[3] || *end != '\0' || !isfinite(trim_deg) ||
            trim_deg < -5.0f || trim_deg > 5.0f) {
            return -1;
        }
        *type = CMD_BALANCE_SET_TRIM;
        put_be_i32(payload, scaled_i32(trim_deg * RAD_PER_DEG, 1000000.0f));
        *payload_len = 4U;
        return 0;
    }

    if (strcmp(cmd, "balance-gains") == 0) {
        float gains[4];
        static const float minimum[4] = {-10.0f, -5.0f, -2.0f, -2.0f};
        static const float maximum[4] = {-0.1f, 0.0f, 0.0f, 0.0f};

        if (argc < 7) return -1;
        for (int i = 0; i < 4; ++i) {
            char *end = NULL;
            gains[i] = strtof(argv[3 + i], &end);
            if (end == argv[3 + i] || *end != '\0' || !isfinite(gains[i]) ||
                gains[i] < minimum[i] || gains[i] > maximum[i]) {
                return -1;
            }
            put_be_i32(&payload[i * 4], scaled_i32(gains[i], 1000000.0f));
        }
        *type = CMD_BALANCE_SET_GAINS;
        *payload_len = 16U;
        return 0;
    }

    if (strcmp(cmd, "balance-config") == 0) {
        *type = CMD_BALANCE_CONFIG;
        *payload_len = 0U;
        return 0;
    }

    if (strcmp(cmd, "balance-reset-config") == 0) {
        *type = CMD_BALANCE_RESET_CONFIG;
        *payload_len = 0U;
        return 0;
    }

    if (strcmp(cmd, "balance-speed-limit") == 0) {
        char *end = NULL;
        float speed_limit;

        if (argc < 4) return -1;
        speed_limit = strtof(argv[3], &end);
        if (end == argv[3] || *end != '\0' || !isfinite(speed_limit) ||
            speed_limit < 0.2f || speed_limit > 2.0f) {
            return -1;
        }
        *type = CMD_BALANCE_SET_SPEED_LIMIT;
        put_be_i32(payload, scaled_i32(speed_limit, 1000000.0f));
        *payload_len = 4U;
        return 0;
    }

    if (strcmp(cmd, "balance-filter") == 0) {
        char *end = NULL;
        float cutoff_hz;

        if (argc < 4) return -1;
        cutoff_hz = strtof(argv[3], &end);
        if (end == argv[3] || *end != '\0' || !isfinite(cutoff_hz) ||
            cutoff_hz < 5.0f || cutoff_hz > 40.0f) {
            return -1;
        }
        *type = CMD_BALANCE_SET_FILTER;
        put_be_i32(payload, scaled_i32(cutoff_hz, 1000000.0f));
        *payload_len = 4U;
        return 0;
    }

    if (strcmp(cmd, "balance-posture-angle") == 0) {
        char *end = NULL;
        float angle_deg;

        if (argc < 4) return -1;
        angle_deg = strtof(argv[3], &end);
        if (end == argv[3] || *end != '\0' || !isfinite(angle_deg) ||
            angle_deg < 1.0f || angle_deg > 10.0f) {
            return -1;
        }
        *type = CMD_BALANCE_SET_POSTURE_PRIORITY;
        put_be_i32(payload,
                   scaled_i32(angle_deg * RAD_PER_DEG, 1000000.0f));
        *payload_len = 4U;
        return 0;
    }

    if (strcmp(cmd, "balance-torque-limit") == 0) {
        char *end = NULL;
        float torque_limit_nm;

        if (argc < 4) return -1;
        torque_limit_nm = strtof(argv[3], &end);
        if (end == argv[3] || *end != '\0' || !isfinite(torque_limit_nm) ||
            torque_limit_nm < 0.05f || torque_limit_nm > 0.30f) {
            return -1;
        }
        *type = CMD_BALANCE_SET_TORQUE_LIMIT;
        put_be_i32(payload, scaled_i32(torque_limit_nm, 1000000.0f));
        *payload_len = 4U;
        return 0;
    }

    if (strcmp(cmd, "balance-position-hold") == 0) {
        float values_deg[3];

        *type = CMD_BALANCE_SET_POSITION_HOLD;
        if (argc == 4 && strcmp(argv[3], "off") == 0) {
            payload[0] = 0U;
            *payload_len = 1U;
            return 0;
        }
        if (argc != 6) return -1;
        for (int i = 0; i < 3; ++i) {
            char *end = NULL;
            values_deg[i] = strtof(argv[3 + i], &end);
            if (end == argv[3 + i] || *end != '\0' ||
                !isfinite(values_deg[i])) return -1;
        }
        if (values_deg[0] < 0.0f || values_deg[0] > 28.6f ||
            values_deg[1] < 0.0f || values_deg[1] > 57.3f ||
            values_deg[2] < 0.1f || values_deg[2] > 3.0f) return -1;
        payload[0] = 1U;
        for (int i = 0; i < 3; ++i) {
            put_be_i32(&payload[1 + i * 4],
                       scaled_i32(values_deg[i] * RAD_PER_DEG, 1000000.0f));
        }
        *payload_len = 13U;
        return 0;
    }

    if (strcmp(cmd, "chassis-track") == 0) {
        char *end = NULL;
        float wheel_track_m;

        if (argc != 4) return -1;
        wheel_track_m = strtof(argv[3], &end);
        if (end == argv[3] || *end != '\0' || !isfinite(wheel_track_m) ||
            wheel_track_m < 0.08f || wheel_track_m > 0.50f) return -1;
        *type = CMD_CHASSIS_SET_TRACK_WIDTH;
        put_be_i32(payload, scaled_i32(wheel_track_m, 1000000.0f));
        *payload_len = 4U;
        return 0;
    }

    if (strcmp(cmd, "chassis-velocity") == 0) {
        char *end = NULL;
        float linear_m_s;
        float angular_rad_s;
        unsigned long timeout_ms = 300U;

        if (argc != 5 && argc != 6) return -1;
        linear_m_s = strtof(argv[3], &end);
        if (end == argv[3] || *end != '\0' || !isfinite(linear_m_s) ||
            linear_m_s < -0.40f || linear_m_s > 0.40f) return -1;
        end = NULL;
        angular_rad_s = strtof(argv[4], &end);
        if (end == argv[4] || *end != '\0' || !isfinite(angular_rad_s) ||
            angular_rad_s < -1.0f || angular_rad_s > 1.0f) return -1;
        if (argc == 6) {
            end = NULL;
            timeout_ms = strtoul(argv[5], &end, 0);
            if (end == argv[5] || *end != '\0' ||
                timeout_ms < 100U || timeout_ms > 1000U) return -1;
        }
        *type = CMD_CHASSIS_SET_VELOCITY;
        put_be_i32(&payload[0], scaled_i32(linear_m_s, 1000000.0f));
        put_be_i32(&payload[4], scaled_i32(angular_rad_s, 1000000.0f));
        put_be_u16(&payload[8], (uint16_t)timeout_ms);
        *payload_len = 10U;
        return 0;
    }

    if (strcmp(cmd, "chassis-status") == 0) {
        *type = CMD_CHASSIS_STATUS;
        *payload_len = 0U;
        return 0;
    }

    if (strcmp(cmd, "pvt") == 0) {
        if (argc < 7) return -1;
        *type = CMD_CAN_PVT;
        payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
        put_be_i32(&payload[1], (int32_t)strtol(argv[4], NULL, 0));
        put_be_u16(&payload[5], (uint16_t)strtoul(argv[5], NULL, 0));
        payload[7] = (uint8_t)strtoul(argv[6], NULL, 0);
        *payload_len = 8;
        return 0;
    }

    if (strcmp(cmd, "stop") == 0) {
        *type = CMD_CAN_SAFE_STOP;
        if (argc >= 4) {
            payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
            *payload_len = 1;
        } else {
            *payload_len = 0;
        }
        return 0;
    }

    return -1;
}

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "/dev/rpmsg0";
    uint8_t type;
    uint8_t payload[RPMSG_MAX_PAYLOAD];
    uint8_t payload_len;
    uint8_t tx_frame[128];
    uint8_t rx_frame[128];
    RpmsgFrame ack;

    printf("rpmsg_client version: %s\n", RPMSG_CLIENT_VERSION);

    if (build_command(argc, argv, &type, payload, &payload_len) != 0) {
        usage(argv[0]);
        return 1;
    }

    int fd = rpmsg_transport_open(dev);
    if (fd < 0) {
        perror("connect rpmsg broker (use direct:/dev/rpmsg0 only with broker stopped)");
        return 1;
    }

    size_t size = rpmsg_encode(type, 1, payload, payload_len, tx_frame, sizeof(tx_frame));
    if (size == 0) {
        fprintf(stderr, "encode command failed\n");
        close(fd);
        return 1;
    }

    if (write(fd, tx_frame, size) != (ssize_t)size) {
        perror("write");
        close(fd);
        return 1;
    }

    printf("command type=%u sent to %s, len=%zu\n", type, dev, size);

    int ready = wait_readable(fd, 5000);
    if (ready < 0) {
        perror("select");
        close(fd);
        return 1;
    }
    if (ready == 0) {
        printf("timeout: no reply from remote core\n");
        close(fd);
        return 2;
    }

    ssize_t rx_size = read(fd, rx_frame, sizeof(rx_frame));
    if (rx_size < 0) {
        perror("read");
        close(fd);
        return 1;
    }

    printf("received raw reply, len=%zd\n", rx_size);
    if (!rpmsg_decode(rx_frame, (size_t)rx_size, &ack)) {
        printf("decode reply failed\n");
        close(fd);
        return 3;
    }

    printf("ack type=%u seq=%u payload_len=%u\n", ack.type, ack.seq, ack.length);
    if (type == CMD_CAN_SPEED_DIAG) {
        static const char *const status_names[] = {
            "ok", "invalid-or-busy", "can-error", "incomplete"
        };
        int32_t periodic;
        int32_t register_speed;
        int32_t position_speed;
        uint8_t status;

        if (ack.type != CMD_CAN_SPEED_DIAG || ack.length < 20U ||
            ack.payload[0] != 1U) {
            printf("invalid motor speed diagnostic reply\n");
            close(fd);
            return 3;
        }
        periodic = read_be_i32(&ack.payload[4]);
        register_speed = read_be_i32(&ack.payload[8]);
        position_speed = read_be_i32(&ack.payload[12]);
        status = ack.payload[1];
        printf("motor speed diagnostic: status=%s(%u) motor_id=%u valid=0x%02X samples=%u\n",
               status < 4U ? status_names[status] : "unknown", status,
               ack.payload[2], ack.payload[3],
               read_be_u32(&ack.payload[16]));
        printf("speed comparison: periodic_0x2A=%.2f rpm register_0x0006=%.2f rpm position_delta=%.2f rpm\n",
               (double)periodic / 100.0,
               (double)register_speed / 100.0,
               (double)position_speed / 100.0);
        if (periodic != 0) {
            printf("speed ratios: register/periodic=%.4f position/periodic=%.4f\n",
                   (double)register_speed / (double)periodic,
                   (double)position_speed / (double)periodic);
        }
        close(fd);
        return status == 0U ? 0 : 4;
    }
    if ((type >= CMD_SERVO_MOVE4 && type <= CMD_SERVO_STOP) ||
        type == CMD_LEG_ENABLE || type == CMD_LEG_MOVE4 ||
        type == CMD_SERVO_TEST_ONE) {
        static const char *const status_names[] = {
            "ok", "invalid", "busy", "balance-active", "hardware-error",
            "not-armed"
        };
        static const char *const state_names[] = {
            "idle", "moving", "fault", "unarmed", "starting", "testing"
        };
        uint8_t status;
        uint8_t state;
        uint16_t current_raw[LEG_JOINT_COUNT];
        uint16_t target_raw[LEG_JOINT_COUNT];
        uint16_t current_effective[LEG_JOINT_COUNT];
        uint16_t target_effective[LEG_JOINT_COUNT];

        if (ack.type != type ||
            ack.length < SERVO_MOTION_TELEMETRY_PAYLOAD_SIZE ||
            ack.payload[0] != SERVO_MOTION_TELEMETRY_VERSION) {
            printf("invalid servo motion reply\n");
            close(fd);
            return 3;
        }
        status = ack.payload[1];
        state = ack.payload[2];
        printf("servo motion: status=%s(%u) state=%s(%u) last_error=%d remaining=%u ms\n",
               status < 6U ? status_names[status] : "unknown", status,
               state < 6U ? state_names[state] : "unknown", state,
               (int8_t)ack.payload[3], read_be_u32(&ack.payload[4]));
        printf("commanded angle (no position feedback): current=%.1f,%.1f,%.1f,%.1f deg target=%.1f,%.1f,%.1f,%.1f deg\n",
               (double)read_be_u16(&ack.payload[8]) / 10.0,
               (double)read_be_u16(&ack.payload[10]) / 10.0,
               (double)read_be_u16(&ack.payload[12]) / 10.0,
               (double)read_be_u16(&ack.payload[14]) / 10.0,
               (double)read_be_u16(&ack.payload[16]) / 10.0,
               (double)read_be_u16(&ack.payload[18]) / 10.0,
               (double)read_be_u16(&ack.payload[20]) / 10.0,
               (double)read_be_u16(&ack.payload[22]) / 10.0);
        for (int i = 0; i < 4; ++i) {
            current_raw[i] = read_be_u16(&ack.payload[8 + i * 2]);
            target_raw[i] = read_be_u16(&ack.payload[16 + i * 2]);
        }
        if (leg_servo_raw_to_effective_x10(current_raw, current_effective) == 0 &&
            leg_servo_raw_to_effective_x10(target_raw, target_effective) == 0) {
            printf("leg effective angle (RF,RR,LF,LR): current=%.1f,%.1f,%.1f,%.1f deg target=%.1f,%.1f,%.1f,%.1f deg\n",
                   (double)current_effective[LEG_JOINT_RIGHT_FRONT] / 10.0,
                   (double)current_effective[LEG_JOINT_RIGHT_REAR] / 10.0,
                   (double)current_effective[LEG_JOINT_LEFT_FRONT] / 10.0,
                   (double)current_effective[LEG_JOINT_LEFT_REAR] / 10.0,
                   (double)target_effective[LEG_JOINT_RIGHT_FRONT] / 10.0,
                   (double)target_effective[LEG_JOINT_RIGHT_REAR] / 10.0,
                   (double)target_effective[LEG_JOINT_LEFT_FRONT] / 10.0,
                   (double)target_effective[LEG_JOINT_LEFT_REAR] / 10.0);
        }
        printf("servo pwm: pulse_us=%u,%u,%u,%u\n",
               read_be_u16(&ack.payload[24]),
               read_be_u16(&ack.payload[26]),
               read_be_u16(&ack.payload[28]),
               read_be_u16(&ack.payload[30]));
        close(fd);
        return status == 0U ? 0 : 4;
    }
    if (type == CMD_IMU_TELEMETRY || type == CMD_IMU_CALIBRATE) {
        static const char *const status_names[] = {
            "ok", "no-sample", "busy", "calibration-failed"
        };
        uint8_t status;

        if (ack.type != type ||
            ack.length < IMU_TELEMETRY_PAYLOAD_SIZE ||
            ack.payload[0] != IMU_TELEMETRY_VERSION) {
            printf("invalid IMU telemetry reply\n");
            close(fd);
            return 3;
        }
        status = ack.payload[1];
        printf("imu attitude: status=%s(%u) valid=%u calibrated=%u age_ms=%u samples=%u\n",
               status < 4U ? status_names[status] : "unknown", status,
               ack.payload[2], ack.payload[3],
               read_be_u32(&ack.payload[48]),
               read_be_u32(&ack.payload[44]));
        printf("attitude: roll=%.3f deg roll_rate=%.4f rad/s pitch=%.3f deg pitch_rate=%.4f rad/s\n",
               (double)read_be_i32(&ack.payload[4]) / 1000000.0 /
                   RAD_PER_DEG,
               (double)read_be_i32(&ack.payload[8]) / 1000000.0,
               (double)read_be_i32(&ack.payload[12]) / 1000000.0 /
                   RAD_PER_DEG,
               (double)read_be_i32(&ack.payload[16]) / 1000000.0);
        printf("imu scaled: accel=%.4f,%.4f,%.4f m/s^2 gyro=%.4f,%.4f,%.4f rad/s\n",
               (double)read_be_i32(&ack.payload[20]) / 1000000.0,
               (double)read_be_i32(&ack.payload[24]) / 1000000.0,
               (double)read_be_i32(&ack.payload[28]) / 1000000.0,
               (double)read_be_i32(&ack.payload[32]) / 1000000.0,
               (double)read_be_i32(&ack.payload[36]) / 1000000.0,
               (double)read_be_i32(&ack.payload[40]) / 1000000.0);
        close(fd);
        return status == 0U ? 0 : 4;
    }
    if (type == CMD_CHASSIS_SET_VELOCITY ||
        type == CMD_CHASSIS_STATUS ||
        type == CMD_CHASSIS_SET_TRACK_WIDTH) {
        static const char *const status_names[] = {
            "ok", "invalid", "unavailable"
        };
        uint8_t status;

        if (ack.type != type ||
            ack.length < CHASSIS_TELEMETRY_PAYLOAD_SIZE ||
            ack.payload[0] != CHASSIS_TELEMETRY_VERSION) {
            printf("invalid chassis telemetry reply\n");
            close(fd);
            return 3;
        }
        status = ack.payload[1];
        printf("chassis: status=%s(%u) balance_state=%u fault=0x%02x command_age_ms=%u\n",
               status < 3U ? status_names[status] : "unknown", status,
               ack.payload[2], ack.payload[3], read_be_u32(&ack.payload[4]));
        printf("target: linear=%.4f m/s angular=%.4f rad/s; applied=%.4f m/s %.4f rad/s\n",
               (double)read_be_i32(&ack.payload[8]) / 1000000.0,
               (double)read_be_i32(&ack.payload[12]) / 1000000.0,
               (double)read_be_i32(&ack.payload[16]) / 1000000.0,
               (double)read_be_i32(&ack.payload[20]) / 1000000.0);
        printf("measured: linear=%.4f m/s angular=%.4f rad/s position=%.4f m yaw=%.4f rad track=%.4f m\n",
               (double)read_be_i32(&ack.payload[24]) / 1000000.0,
               (double)read_be_i32(&ack.payload[28]) / 1000000.0,
               (double)read_be_i32(&ack.payload[32]) / 1000000.0,
               (double)read_be_i32(&ack.payload[36]) / 1000000.0,
               (double)read_be_i32(&ack.payload[40]) / 1000000.0);
        close(fd);
        return status == 0U ? 0 : 4;
    }
    if ((type >= CMD_BALANCE_SET_TRIM &&
         type <= CMD_BALANCE_SET_SPEED_LIMIT) ||
        (type >= CMD_BALANCE_SET_FILTER &&
         type <= CMD_BALANCE_SET_POSITION_HOLD)) {
        static const char *const status_names[] = {
            "ok", "invalid", "busy"
        };
        uint8_t status;
        const char *status_name;

        if (ack.length < 32U || ack.payload[0] < 2U) {
            printf("invalid balance config reply\n");
            close(fd);
            return 3;
        }
        status = ack.payload[1];
        status_name = status < 3U ? status_names[status] : "unknown";
        printf("balance config: status=%s(%u) state=%u\n",
               status_name, status, ack.payload[2]);
        printf("balance trim: %.6f deg (%.6f rad)\n",
               (double)read_be_i32(&ack.payload[4]) / 1000000.0 /
                   RAD_PER_DEG,
               (double)read_be_i32(&ack.payload[4]) / 1000000.0);
        printf("balance gains: K1=%.6f K2=%.6f K3=%.6f K4=%.6f\n",
               (double)read_be_i32(&ack.payload[8]) / 1000000.0,
               (double)read_be_i32(&ack.payload[12]) / 1000000.0,
               (double)read_be_i32(&ack.payload[16]) / 1000000.0,
               (double)read_be_i32(&ack.payload[20]) / 1000000.0);
        printf("posture priority angle: %.6f deg\n",
               (double)read_be_i32(&ack.payload[24]) / 1000000.0 /
                   RAD_PER_DEG);
        printf("wheel speed limit: %.6f m/s (%.1f rpm)\n",
               (double)read_be_i32(&ack.payload[28]) / 1000000.0,
               (double)read_be_i32(&ack.payload[28]) / 1000000.0 /
                   0.03225 * 60.0 / (2.0 * 3.14159265358979323846));
        if (ack.length >= 40U && ack.payload[0] >= 3U) {
            printf("motor feedback speed scale: %.6f\n",
                   (double)read_be_i32(&ack.payload[32]) / 1000000.0);
            printf("pitch-rate low-pass cutoff: %.3f Hz\n",
                   (double)read_be_i32(&ack.payload[36]) / 1000000.0);
        }
        if (ack.length >= 44U && ack.payload[0] >= 4U) {
            printf("single-wheel torque limit: %.6f Nm\n",
                   (double)read_be_i32(&ack.payload[40]) / 1000000.0);
        }
        if (ack.length >= 60U && ack.payload[0] >= 5U) {
            printf("position hold: %s kp=%.6f deg/m kd=%.6f deg/(m/s) limit=%.6f deg\n",
                   ack.payload[56] ? "enabled" : "disabled",
                   (double)read_be_i32(&ack.payload[44]) / 1000000.0 /
                       RAD_PER_DEG,
                   (double)read_be_i32(&ack.payload[48]) / 1000000.0 /
                       RAD_PER_DEG,
                   (double)read_be_i32(&ack.payload[52]) / 1000000.0 /
                       RAD_PER_DEG);
        }
        close(fd);
        return status == 0U ? 0 : 4;
    }
    if (ack.length >= 4) {
        printf("remote state: heartbeat_ok=%u last_can_ret=%d can_rx=%u feedback=%u\n",
               ack.payload[2], (int8_t)ack.payload[3],
               ack.payload[0], ack.payload[1]);
    }

    if (ack.length >= 26) {
        uint32_t baudrate = read_be_u32(&ack.payload[7]);
        uint32_t send_count = read_be_u32(&ack.payload[11]);
        uint16_t frame_id = (uint16_t)(((uint16_t)ack.payload[15] << 8) | ack.payload[16]);

        printf("can debug: init_ret=%d send_ret=%d can_id=%u baudrate=%u send_count=%u\n",
               (int8_t)ack.payload[4],
               (int8_t)ack.payload[5],
               ack.payload[6],
               baudrate,
               send_count);

        printf("last can frame: id=0x%03x dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
               frame_id,
               ack.payload[17],
               ack.payload[18],
               ack.payload[19],
               ack.payload[20],
               ack.payload[21],
               ack.payload[22],
               ack.payload[23],
               ack.payload[24],
               ack.payload[25]);
    }

    if (ack.length >= 50) {
        uint32_t reg_ctrl = read_be_u32(&ack.payload[26]);
        uint32_t reg_intr = read_be_u32(&ack.payload[30]);
        uint32_t reg_xfer_sts = read_be_u32(&ack.payload[34]);
        uint32_t reg_err_cnt = read_be_u32(&ack.payload[38]);
        uint32_t reg_fifo_cnt = read_be_u32(&ack.payload[42]);
        uint32_t reg_xfer_en = read_be_u32(&ack.payload[46]);
        uint32_t tx_err = (reg_err_cnt >> 16) & 0x1ff;
        uint32_t rx_err = reg_err_cnt & 0x1ff;
        uint32_t tx_fifo = (reg_fifo_cnt >> 16) & 0x7f;
        uint32_t rx_fifo = reg_fifo_cnt & 0x7f;
        uint32_t ctrl_enable = reg_ctrl & 0x1;
        uint32_t xfer_en_bit = reg_xfer_en & 0x1;

        printf("can regs: CTRL=0x%08X INTR=0x%08X XFER_STS=0x%08X ERR_CNT=0x%08X FIFO_CNT=0x%08X XFER_EN=0x%08X\n",
               reg_ctrl, reg_intr, reg_xfer_sts, reg_err_cnt, reg_fifo_cnt, reg_xfer_en);
        printf("can decoded: tx_err=%u rx_err=%u tx_fifo=%u rx_fifo=%u ctrl_enable=%u xfer_en_bit=%u\n",
               tx_err, rx_err, tx_fifo, rx_fifo, ctrl_enable, xfer_en_bit);
    }

    if (ack.length >= 60) {
        uint16_t servo_angle[4];
        for (int i = 0; i < 4; ++i) {
            servo_angle[i] = (uint16_t)(((uint16_t)ack.payload[52 + i * 2] << 8) |
                                        ack.payload[53 + i * 2]);
        }

        printf("servo debug: init_ret=%d last_ret=%d angles=%u,%u,%u,%u\n",
               (int8_t)ack.payload[50],
               (int8_t)ack.payload[51],
               servo_angle[0],
               servo_angle[1],
               servo_angle[2],
               servo_angle[3]);
        if (ack.length >= 86 && type == CMD_CAN_MOTOR_FAULT) {
            uint32_t fault_code = read_be_u32(&ack.payload[80]);
            uint8_t motor_id = ack.payload[84];
            int8_t read_ret = (int8_t)ack.payload[85];

            printf("motor fault: id=%u read_ret=%d code=0x%08X\n",
                   motor_id, read_ret, fault_code);
            if (read_ret == 0) {
                print_motor_fault_bits(fault_code);
            }
        } else if (ack.length >= 88 &&
            (type == CMD_CAN_TORQUE_TEST ||
             (type >= CMD_BALANCE_ENABLE && type <= CMD_BALANCE_STATUS))) {
            int16_t left_current_x100 = (int16_t)read_be_u16(&ack.payload[80]);
            int16_t right_current_x100 = (int16_t)read_be_u16(&ack.payload[82]);
            int16_t left_speed_rpm = (int16_t)read_be_u16(&ack.payload[84]);
            int16_t right_speed_rpm = (int16_t)read_be_u16(&ack.payload[86]);

            printf("motor feedback%s: current=%.2f,%.2f A speed=%d,%d rpm\n",
                   type == CMD_CAN_TORQUE_TEST ? " peak" : "",
                   (double)left_current_x100 / 100.0,
                   (double)right_current_x100 / 100.0,
                   left_speed_rpm, right_speed_rpm);
        } else if (ack.length >= 88) {
            uint16_t servo_pulse[4];
            for (int i = 0; i < 4; ++i) {
                servo_pulse[i] = read_be_u16(&ack.payload[80 + i * 2]);
            }
            printf("servo pwm: pulse_us=%u,%u,%u,%u\n",
                   servo_pulse[0],
                   servo_pulse[1],
                   servo_pulse[2],
                   servo_pulse[3]);
        }
    }

    if (ack.length >= 80) {
        int16_t acc[3];
        int16_t gyro[3];
        for (int i = 0; i < 3; ++i) {
            acc[i] = (int16_t)(((uint16_t)ack.payload[64 + i * 2] << 8) |
                               ack.payload[65 + i * 2]);
            gyro[i] = (int16_t)(((uint16_t)ack.payload[70 + i * 2] << 8) |
                                ack.payload[71 + i * 2]);
        }

        printf("imu debug: init_ret=%d last_ret=%d accel_id=0x%02X gyro_id=0x%02X read_count=%u\n",
               (int8_t)ack.payload[60],
               (int8_t)ack.payload[61],
               ack.payload[62],
               ack.payload[63],
               read_be_u32(&ack.payload[76]));
        printf("imu raw: acc=%d,%d,%d gyro=%d,%d,%d\n",
               acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
    }

    if (ack.length >= 120) {
        static const char *const state_names[] = {
            "disabled", "arming", "active", "fault"
        };
        uint8_t state = ack.payload[88];
        uint8_t fault = ack.payload[89];
        const char *state_name = state < 4U ? state_names[state] : "unknown";

        printf("balance: state=%s(%u) fault=0x%02X control_hz=%u loop_count=%u\n",
               state_name, state, fault, read_be_u16(&ack.payload[90]),
               read_be_u32(&ack.payload[116]));
        printf("balance state: pitch=%.6f rad pitch_rate=%.6f rad/s position=%.6f m velocity=%.6f m/s\n",
               (double)read_be_i32(&ack.payload[92]) / 1000000.0,
               (double)read_be_i32(&ack.payload[96]) / 1000000.0,
               (double)read_be_i32(&ack.payload[100]) / 1000000.0,
               (double)read_be_i32(&ack.payload[104]) / 1000000.0);
        printf("balance output: left=%.6f Nm right=%.6f Nm\n",
               (double)read_be_i32(&ack.payload[108]) / 1000000.0,
               (double)read_be_i32(&ack.payload[112]) / 1000000.0);
        if (fault != 0U) {
            printf("balance fault bits: imu=%u left_motor=%u right_motor=%u can=%u fall=%u overrun=%u arm_timeout=%u arm_speed_or_config=%u\n",
                   !!(fault & 0x01U), !!(fault & 0x02U),
                   !!(fault & 0x04U), !!(fault & 0x08U),
                   !!(fault & 0x10U), !!(fault & 0x20U),
                   !!(fault & 0x40U), !!(fault & 0x80U));
        }
    }

    close(fd);
    return 0;
}
