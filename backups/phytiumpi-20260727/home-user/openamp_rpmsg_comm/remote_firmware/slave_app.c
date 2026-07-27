/*
 * From-core baremetal firmware skeleton.
 *
 * Integration position:
 *   phytium-pi-os/output/build/phytium-standalone-openamp-v1.0/
 *   example/system/amp/openamp_for_linux/src/slaver_00_example.c
 *
 * This file is not a full Phytium SDK project by itself. It keeps the command
 * parsing and safety logic independent, so it can be copied into the real
 * OpenAMP rpmsg callback and compiled into openamp_core0.elf.
 */

#include "rpmsg_protocol.h"
#include "balance_controller.h"
#include "gimbal_controller.h"
#include "motor_can.h"
#include "phytium_bmi088_port.h"
#include "phytium_can_port.h"
#include "phytium_servo_port.h"
#include "servo_motion_controller.h"
#include "fgeneric_timer.h"
#include "fparameters.h"
#include "fsleep.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t heartbeat_ok;
    int8_t last_can_ret;
} SlaveControlState;

static SlaveControlState g_state;
static uint8_t g_last_command_type;
static int16_t g_torque_test_peak_current_x100[2];
static int16_t g_torque_test_peak_speed_rpm[2];
static uint32_t g_motor_fault_code;
static uint8_t g_motor_fault_id;
static int8_t g_motor_fault_read_ret;
static uint64_t g_active_gimbal_poll_interval_ticks;
static uint64_t g_next_active_gimbal_poll_tick;

#define ACTIVE_GIMBAL_POLL_HZ 20U

typedef struct {
    uint8_t status;
    uint8_t motor_id;
    uint8_t valid_flags;
    int32_t periodic_speed_x100_rpm;
    int32_t register_speed_x100_rpm;
    int32_t position_speed_x100_rpm;
    uint32_t sample_count;
} MotorSpeedDiagResult;

static MotorSpeedDiagResult g_speed_diag;

static int send_motor_frame(const MotorCanFrame *frame)
{
    int ret = phytium_can_send(frame);
    g_state.last_can_ret = (int8_t)ret;
    return ret;
}

static int32_t read_be_i32(const uint8_t *p)
{
    return ((int32_t)p[0] << 24) |
           ((int32_t)p[1] << 16) |
           ((int32_t)p[2] << 8) |
           (int32_t)p[3];
}

static uint16_t read_be_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void write_be_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)(value & 0xff);
}

static void write_be_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)(value & 0xff);
}

static int handle_servo_enable4(const uint16_t angles[PHYTIUM_SERVO_NUM])
{
    if (angles == NULL) {
        return SERVO_MOTION_INVALID;
    }
    if (balance_control_get_telemetry()->state != BALANCE_STATE_DISABLED) {
        return SERVO_MOTION_BALANCE_ACTIVE;
    }
    return servo_motion_enable_at_target(angles);
}

static void handle_servo_set4(const uint8_t *payload, uint8_t length)
{
    uint8_t payload_x10[PHYTIUM_SERVO_NUM * 2U];

    if (length < PHYTIUM_SERVO_NUM * 2U) {
        return;
    }
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        write_be_u16(&payload_x10[i * 2U],
                     (uint16_t)(read_be_u16(&payload[i * 2U]) * 10U));
    }
    uint16_t angles[PHYTIUM_SERVO_NUM];
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        angles[i] = read_be_u16(&payload_x10[i * 2U]);
    }
    (void)handle_servo_enable4(angles);
}

static void handle_servo_center(void)
{
    const uint16_t angles[PHYTIUM_SERVO_NUM] = {450, 1350, 450, 1350};

    if (balance_control_get_telemetry()->state != BALANCE_STATE_DISABLED) {
        return;
    }
    (void)servo_motion_enable_at_target(angles);
}

static void handle_servo_polarity(const uint8_t *payload, uint8_t length)
{
    if (length < 1U ||
        balance_control_get_telemetry()->state != BALANCE_STATE_DISABLED) {
        return;
    }

    phytium_servo_set_polarity(payload[0]);
    const uint16_t angles[PHYTIUM_SERVO_NUM] = {450, 1350, 450, 1350};
    (void)servo_motion_enable_at_target(angles);
}

static int handle_servo_move4(const uint8_t *payload, uint8_t length)
{
    uint16_t target[PHYTIUM_SERVO_NUM];

    if (length < 10U) {
        return SERVO_MOTION_INVALID;
    }
    if (balance_control_get_telemetry()->state != BALANCE_STATE_DISABLED) {
        return SERVO_MOTION_BALANCE_ACTIVE;
    }
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        target[i] = read_be_u16(&payload[i * 2U]);
    }
    return servo_motion_start(target, read_be_u16(&payload[8]));
}

static int leg_raw_angles_are_safe(const uint8_t *payload, uint8_t length)
{
    uint16_t raw[PHYTIUM_SERVO_NUM];

    if (payload == NULL || length < PHYTIUM_SERVO_NUM * 2U) {
        return 0;
    }
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        raw[i] = read_be_u16(&payload[i * 2U]);
    }
    return raw[0] <= 450U && raw[2] <= 450U &&
           raw[1] >= 1350U && raw[1] <= 1800U &&
           raw[3] >= 1350U && raw[3] <= 1800U;
}

static int handle_leg_move4(const uint8_t *payload, uint8_t length)
{
    if (length < 10U || !leg_raw_angles_are_safe(payload, length)) {
        return SERVO_MOTION_INVALID;
    }
    return handle_servo_move4(payload, length);
}

static void handle_imu_init(void)
{
    (void)phytium_bmi088_init();
}

static void handle_imu_read(void)
{
    (void)phytium_bmi088_read_sample();
}

static void handle_can_enable(uint8_t motor_id)
{
    MotorCanFrame can_frame;
    motor_build_enable(motor_id, &can_frame);
    send_motor_frame(&can_frame);
}

static void handle_can_set_mode(uint8_t motor_id, uint16_t mode)
{
    MotorCanFrame can_frame;
    motor_build_set_mode(motor_id, mode, &can_frame);
    send_motor_frame(&can_frame);
}

static void handle_can_temporary_origin(uint8_t motor_id)
{
    MotorCanFrame can_frame;
    motor_build_set_temporary_origin(motor_id, &can_frame);
    send_motor_frame(&can_frame);
}

static int is_gimbal_motor(uint8_t motor_id)
{
    return motor_id == GIMBAL_YAW_MOTOR_ID ||
           motor_id == GIMBAL_PITCH_MOTOR_ID;
}

static void write_be_i32(uint8_t *p, int32_t value)
{
    write_be_u32(p, (uint32_t)value);
}

static int32_t float_to_i32(float value, float scale)
{
    float scaled;

    if (!isfinite(value)) {
        return 0;
    }
    scaled = value * scale;
    if (scaled > 2147483647.0f) {
        return INT32_MAX;
    }
    if (scaled < -2147483648.0f) {
        return INT32_MIN;
    }
    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static void handle_can_set_origin(uint8_t motor_id)
{
    MotorCanFrame can_frame;
    motor_build_set_origin(motor_id, &can_frame);
    send_motor_frame(&can_frame);
}

static void handle_can_safe_stop(uint8_t motor_id)
{
    MotorCanFrame can_frame;
    motor_build_safe_stop(motor_id, &can_frame);
    send_motor_frame(&can_frame);
}

static void handle_can_pvt(const uint8_t *payload, uint8_t length)
{
    if (length < 8) {
        return;
    }

    uint8_t motor_id = payload[0];
    int32_t position_x100_deg = read_be_i32(&payload[1]);
    uint16_t speed_rpm = read_be_u16(&payload[5]);
    uint8_t torque_percent = payload[7];

    MotorCanFrame can_frame;
    if (is_gimbal_motor(motor_id)) {
        return;
    }
    motor_build_pvt(motor_id, position_x100_deg, speed_rpm, torque_percent, &can_frame);
    send_motor_frame(&can_frame);
}

static void handle_can_init_motor(uint8_t motor_id)
{
    MotorCanFrame can_frame;

    /* Keep the saved origin intact: select position mode, then enter closed loop. */
    motor_build_set_mode(motor_id, 2, &can_frame);
    if (send_motor_frame(&can_frame) != 0) {
        return;
    }
    fsleep_millisec(10);
    motor_build_enable(motor_id, &can_frame);
    send_motor_frame(&can_frame);
    fsleep_millisec(10);
}

static int handle_motor_init_all(void)
{
    MotorCanFrame can_frame;

    motor_build_set_mode(1, 2, &can_frame);
    if (send_motor_frame(&can_frame) != 0) {
        return -1;
    }
    motor_build_set_mode(2, 2, &can_frame);
    if (send_motor_frame(&can_frame) != 0) {
        return -1;
    }
    fsleep_millisec(10);

    motor_build_enable(1, &can_frame);
    if (send_motor_frame(&can_frame) != 0) {
        return -1;
    }
    motor_build_enable(2, &can_frame);
    if (send_motor_frame(&can_frame) != 0) {
        return -1;
    }
    fsleep_millisec(10);
    return 0;
}

static void handle_motor_test(void)
{
    MotorCanFrame can_frame;

    if (handle_motor_init_all() != 0) {
        return;
    }
    motor_build_pvt(1, 36000, 200, 50, &can_frame);
    send_motor_frame(&can_frame);
    motor_build_pvt(2, 36000, 200, 50, &can_frame);
    send_motor_frame(&can_frame);
}

static size_t build_ack(uint8_t seq, uint8_t *out, size_t out_size)
{
    const PhytiumCanDebugState *can_dbg = phytium_can_get_debug_state();
    const PhytiumServoDebugState *servo_dbg = phytium_servo_get_debug_state();
    const PhytiumBmi088DebugState *imu_dbg = phytium_bmi088_get_debug_state();
    const BalanceTelemetry *balance = balance_control_get_telemetry();
    MotorFeedback left_feedback;
    MotorFeedback right_feedback;
    uint8_t payload[120];
    memset(payload, 0, sizeof(payload));

    payload[0] = (uint8_t)can_dbg->receive_count;
    payload[1] = (uint8_t)can_dbg->feedback_count;
    payload[2] = g_state.heartbeat_ok;
    payload[3] = (uint8_t)g_state.last_can_ret;
    payload[4] = (uint8_t)can_dbg->init_ret;
    payload[5] = (uint8_t)can_dbg->last_send_ret;
    payload[6] = (uint8_t)can_dbg->can_id;
    payload[7] = (uint8_t)(can_dbg->baudrate >> 24);
    payload[8] = (uint8_t)(can_dbg->baudrate >> 16);
    payload[9] = (uint8_t)(can_dbg->baudrate >> 8);
    payload[10] = (uint8_t)(can_dbg->baudrate & 0xff);
    payload[11] = (uint8_t)(can_dbg->send_count >> 24);
    payload[12] = (uint8_t)(can_dbg->send_count >> 16);
    payload[13] = (uint8_t)(can_dbg->send_count >> 8);
    payload[14] = (uint8_t)(can_dbg->send_count & 0xff);
    payload[15] = (uint8_t)(can_dbg->last_frame_id >> 8);
    payload[16] = (uint8_t)(can_dbg->last_frame_id & 0xff);
    payload[17] = can_dbg->last_frame_dlc;
    for (int i = 0; i < 8; ++i) {
        payload[18 + i] = can_dbg->last_frame_data[i];
    }
    write_be_u32(&payload[26], can_dbg->reg_ctrl);
    write_be_u32(&payload[30], can_dbg->reg_intr);
    write_be_u32(&payload[34], can_dbg->reg_xfer_sts);
    write_be_u32(&payload[38], can_dbg->reg_err_cnt);
    write_be_u32(&payload[42], can_dbg->reg_fifo_cnt);
    write_be_u32(&payload[46], can_dbg->reg_xfer_en);
    payload[50] = (uint8_t)servo_dbg->init_ret;
    payload[51] = (uint8_t)servo_dbg->last_ret;
    for (int i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        write_be_u16(&payload[52 + i * 2], servo_dbg->angle_deg[i]);
    }
    if (g_last_command_type == CMD_CAN_MOTOR_FAULT) {
        write_be_u32(&payload[80], g_motor_fault_code);
        payload[84] = g_motor_fault_id;
        payload[85] = (uint8_t)g_motor_fault_read_ret;
    } else if (g_last_command_type == CMD_CAN_TORQUE_TEST) {
        write_be_u16(&payload[80],
                     (uint16_t)g_torque_test_peak_current_x100[0]);
        write_be_u16(&payload[82],
                     (uint16_t)g_torque_test_peak_current_x100[1]);
        write_be_u16(&payload[84],
                     (uint16_t)g_torque_test_peak_speed_rpm[0]);
        write_be_u16(&payload[86],
                     (uint16_t)g_torque_test_peak_speed_rpm[1]);
    } else if (g_last_command_type >= CMD_BALANCE_ENABLE &&
        g_last_command_type <= CMD_BALANCE_STATUS) {
        if (phytium_can_get_motor_feedback(1U, &left_feedback) == 0) {
            write_be_u16(&payload[80], (uint16_t)left_feedback.current_x100_a);
            write_be_u16(&payload[84], (uint16_t)left_feedback.speed_rpm);
        }
        if (phytium_can_get_motor_feedback(2U, &right_feedback) == 0) {
            write_be_u16(&payload[82], (uint16_t)right_feedback.current_x100_a);
            write_be_u16(&payload[86], (uint16_t)right_feedback.speed_rpm);
        }
    } else {
        for (int i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
            write_be_u16(&payload[80 + i * 2], servo_dbg->pulse_us[i]);
        }
    }
    payload[60] = (uint8_t)imu_dbg->init_ret;
    payload[61] = (uint8_t)imu_dbg->last_ret;
    payload[62] = imu_dbg->accel_chip_id;
    payload[63] = imu_dbg->gyro_chip_id;
    for (int i = 0; i < 3; ++i) {
        write_be_u16(&payload[64 + i * 2], (uint16_t)imu_dbg->accel_raw[i]);
        write_be_u16(&payload[70 + i * 2], (uint16_t)imu_dbg->gyro_raw[i]);
    }
    write_be_u32(&payload[76], imu_dbg->read_count);

    payload[88] = balance->state;
    payload[89] = balance->fault;
    write_be_u16(&payload[90], balance->control_hz);
    write_be_i32(&payload[92], float_to_i32(balance->pitch_rad, 1000000.0f));
    write_be_i32(&payload[96], float_to_i32(balance->pitch_rate_rad_s, 1000000.0f));
    write_be_i32(&payload[100], float_to_i32(balance->wheel_position_m, 1000000.0f));
    write_be_i32(&payload[104], float_to_i32(balance->wheel_velocity_m_s, 1000000.0f));
    write_be_i32(&payload[108], float_to_i32(balance->left_torque_nm, 1000000.0f));
    write_be_i32(&payload[112], float_to_i32(balance->right_torque_nm, 1000000.0f));
    write_be_u32(&payload[116], balance->loop_count);

    return rpmsg_encode(CMD_HEARTBEAT, seq, payload, sizeof(payload), out, out_size);
}

static size_t build_balance_config_ack(uint8_t type, uint8_t seq,
                                       uint8_t status, uint8_t *out,
                                       size_t out_size)
{
    const BalanceTelemetry *telemetry = balance_control_get_telemetry();
    BalanceRuntimeConfig config;
    uint8_t payload[60];

    memset(payload, 0, sizeof(payload));
    balance_control_get_runtime_config(&config);
    payload[0] = 5U;
    payload[1] = status;
    payload[2] = telemetry->state;
    write_be_i32(&payload[4], float_to_i32(config.pitch_trim_rad, 1000000.0f));
    write_be_i32(&payload[8], float_to_i32(config.k_theta, 1000000.0f));
    write_be_i32(&payload[12], float_to_i32(config.k_theta_rate, 1000000.0f));
    write_be_i32(&payload[16], float_to_i32(config.k_position, 1000000.0f));
    write_be_i32(&payload[20], float_to_i32(config.k_velocity, 1000000.0f));
    write_be_i32(&payload[24],
                 float_to_i32(config.posture_priority_angle_rad, 1000000.0f));
    write_be_i32(&payload[28],
                 float_to_i32(config.max_wheel_speed_m_s, 1000000.0f));
    write_be_i32(&payload[32],
                 float_to_i32(config.motor_feedback_speed_scale, 1000000.0f));
    write_be_i32(&payload[36],
                 float_to_i32(config.pitch_rate_filter_hz, 1000000.0f));
    write_be_i32(&payload[40],
                 float_to_i32(config.torque_limit_nm, 1000000.0f));
    write_be_i32(&payload[44],
                 float_to_i32(config.position_hold_kp_rad_per_m, 1000000.0f));
    write_be_i32(&payload[48],
                 float_to_i32(config.position_hold_kd_rad_per_m_s, 1000000.0f));
    write_be_i32(&payload[52],
                 float_to_i32(config.position_hold_limit_rad, 1000000.0f));
    payload[56] = config.position_hold_enabled;
    return rpmsg_encode(type, seq, payload, sizeof(payload), out, out_size);
}

static size_t build_servo_motion_ack(uint8_t type, uint8_t seq,
                                     uint8_t status, uint8_t *out,
                                     size_t out_size)
{
    const ServoMotionTelemetry *telemetry = servo_motion_get_telemetry();
    uint8_t payload[SERVO_MOTION_TELEMETRY_PAYLOAD_SIZE];

    memset(payload, 0, sizeof(payload));
    payload[0] = SERVO_MOTION_TELEMETRY_VERSION;
    payload[1] = status;
    payload[2] = telemetry->state;
    payload[3] = (uint8_t)telemetry->last_error;
    write_be_u32(&payload[4], telemetry->remaining_ms);
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        write_be_u16(&payload[8 + i * 2U],
                     telemetry->current_angle_x10_deg[i]);
        write_be_u16(&payload[16 + i * 2U],
                     telemetry->target_angle_x10_deg[i]);
        write_be_u16(&payload[24 + i * 2U], telemetry->pulse_us[i]);
    }
    return rpmsg_encode(type, seq, payload, sizeof(payload), out, out_size);
}

static size_t build_imu_telemetry_ack(uint8_t type, uint8_t seq,
                                      uint8_t command_status, uint8_t *out,
                                      size_t out_size)
{
    const PhytiumBmi088DebugState *debug = phytium_bmi088_get_debug_state();
    PhytiumBmi088Sample sample;
    uint8_t payload[IMU_TELEMETRY_PAYLOAD_SIZE];
    uint64_t frequency = GenericTimerFrequecy();
    uint8_t status = 0U;
    uint32_t age_ms = UINT32_MAX;

    memset(payload, 0, sizeof(payload));
    memset(&sample, 0, sizeof(sample));
    if (phytium_bmi088_get_sample(&sample) != 0 || !sample.valid) {
        status = 1U;
    } else if (frequency != 0U) {
        uint64_t now = GenericTimerRead(GENERIC_TIMER_ID0);
        age_ms = (uint32_t)(((now - sample.update_tick) * 1000U) / frequency);
    }
    payload[0] = IMU_TELEMETRY_VERSION;
    payload[1] = command_status == UINT8_MAX ? status : command_status;
    payload[2] = sample.valid;
    payload[3] = sample.calibrated;
    write_be_i32(&payload[4], float_to_i32(sample.roll_rad, 1000000.0f));
    write_be_i32(&payload[8],
                 float_to_i32(sample.roll_rate_rad_s, 1000000.0f));
    write_be_i32(&payload[12], float_to_i32(sample.pitch_rad, 1000000.0f));
    write_be_i32(&payload[16],
                 float_to_i32(sample.pitch_rate_rad_s, 1000000.0f));
    for (uint8_t i = 0; i < 3U; ++i) {
        write_be_i32(&payload[20 + i * 4U],
                     float_to_i32(sample.accel_m_s2[i], 1000000.0f));
        write_be_i32(&payload[32 + i * 4U],
                     float_to_i32(sample.gyro_rad_s[i], 1000000.0f));
    }
    write_be_u32(&payload[44], debug->read_count);
    write_be_u32(&payload[48], age_ms);
    return rpmsg_encode(type, seq, payload, sizeof(payload), out, out_size);
}

static size_t build_balance_telemetry_ack(uint8_t seq, uint8_t *out,
                                          size_t out_size)
{
    const BalanceTelemetry *telemetry = balance_control_get_telemetry();
    MotorFeedback left = {0};
    MotorFeedback right = {0};
    uint8_t payload[BALANCE_TELEMETRY_PAYLOAD_SIZE];

    memset(payload, 0, sizeof(payload));
    (void)phytium_can_get_motor_feedback(1U, &left);
    (void)phytium_can_get_motor_feedback(2U, &right);
    payload[0] = BALANCE_TELEMETRY_VERSION;
    payload[1] = telemetry->state;
    payload[2] = telemetry->fault;
    payload[3] = telemetry->position_hold_enabled;
    write_be_u16(&payload[4], telemetry->control_hz);
    write_be_u32(&payload[8], telemetry->loop_count);
    write_be_i32(&payload[12],
                 float_to_i32(telemetry->pitch_rad, 1000000.0f));
    write_be_i32(&payload[16],
                 float_to_i32(telemetry->pitch_rate_rad_s, 1000000.0f));
    write_be_i32(&payload[20],
                 float_to_i32(telemetry->wheel_position_m, 1000000.0f));
    write_be_i32(&payload[24],
                 float_to_i32(telemetry->wheel_velocity_m_s, 1000000.0f));
    write_be_i32(&payload[28],
                 float_to_i32(telemetry->left_torque_nm, 1000000.0f));
    write_be_i32(&payload[32],
                 float_to_i32(telemetry->right_torque_nm, 1000000.0f));
    write_be_u16(&payload[36], (uint16_t)left.current_x100_a);
    write_be_u16(&payload[38], (uint16_t)right.current_x100_a);
    write_be_u16(&payload[40], (uint16_t)left.speed_rpm);
    write_be_u16(&payload[42], (uint16_t)right.speed_rpm);
    write_be_i32(&payload[44],
                 float_to_i32(telemetry->pitch_target_rad, 1000000.0f));
    write_be_i32(&payload[48],
                 float_to_i32(telemetry->position_target_m, 1000000.0f));
    write_be_i32(&payload[52],
                 float_to_i32(telemetry->position_error_m, 1000000.0f));
    write_be_i32(&payload[56],
                 float_to_i32(telemetry->velocity_error_m_s, 1000000.0f));
    return rpmsg_encode(CMD_BALANCE_TELEMETRY, seq, payload,
                        sizeof(payload), out, out_size);
}

static size_t build_chassis_telemetry_ack(uint8_t type, uint8_t seq,
                                          uint8_t status, uint8_t *out,
                                          size_t out_size)
{
    const BalanceTelemetry *balance = balance_control_get_telemetry();
    BalanceMotionTelemetry motion;
    uint8_t payload[CHASSIS_TELEMETRY_PAYLOAD_SIZE];

    memset(payload, 0, sizeof(payload));
    balance_control_get_motion_telemetry(&motion);
    payload[0] = CHASSIS_TELEMETRY_VERSION;
    payload[1] = status;
    payload[2] = balance->state;
    payload[3] = balance->fault;
    write_be_u32(&payload[4], motion.command_age_ms);
    write_be_i32(&payload[8],
                 float_to_i32(motion.target_linear_m_s, 1000000.0f));
    write_be_i32(&payload[12],
                 float_to_i32(motion.target_angular_rad_s, 1000000.0f));
    write_be_i32(&payload[16],
                 float_to_i32(motion.applied_linear_m_s, 1000000.0f));
    write_be_i32(&payload[20],
                 float_to_i32(motion.applied_angular_rad_s, 1000000.0f));
    write_be_i32(&payload[24],
                 float_to_i32(motion.measured_linear_m_s, 1000000.0f));
    write_be_i32(&payload[28],
                 float_to_i32(motion.measured_angular_rad_s, 1000000.0f));
    write_be_i32(&payload[32],
                 float_to_i32(motion.wheel_position_m, 1000000.0f));
    write_be_i32(&payload[36],
                 float_to_i32(motion.yaw_position_rad, 1000000.0f));
    write_be_i32(&payload[40],
                 float_to_i32(motion.wheel_track_m, 1000000.0f));
    return rpmsg_encode(type, seq, payload, sizeof(payload), out, out_size);
}

static size_t build_speed_diag_ack(uint8_t seq, uint8_t *out,
                                   size_t out_size)
{
    uint8_t payload[24];

    memset(payload, 0, sizeof(payload));
    payload[0] = 1U;
    payload[1] = g_speed_diag.status;
    payload[2] = g_speed_diag.motor_id;
    payload[3] = g_speed_diag.valid_flags;
    write_be_i32(&payload[4], g_speed_diag.periodic_speed_x100_rpm);
    write_be_i32(&payload[8], g_speed_diag.register_speed_x100_rpm);
    write_be_i32(&payload[12], g_speed_diag.position_speed_x100_rpm);
    write_be_u32(&payload[16], g_speed_diag.sample_count);
    return rpmsg_encode(CMD_CAN_SPEED_DIAG, seq, payload, sizeof(payload),
                        out, out_size);
}

static size_t build_gimbal_ack(uint8_t type, uint8_t seq, uint8_t status,
                               uint8_t *out, size_t out_size)
{
    const GimbalTelemetry *telemetry = gimbal_control_get_telemetry();
    uint8_t payload[GIMBAL_TELEMETRY_PAYLOAD_SIZE];

    memset(payload, 0, sizeof(payload));
    payload[0] = GIMBAL_TELEMETRY_VERSION;
    payload[1] = status;
    payload[2] = telemetry->state;
    payload[3] = telemetry->fault;
    payload[4] = telemetry->limits_valid_mask;
    payload[5] = telemetry->feedback_valid_mask;
    payload[6] = telemetry->command_torque_percent;
    write_be_i32(&payload[8], telemetry->yaw_position_x100_deg);
    write_be_i32(&payload[12], telemetry->pitch_position_x100_deg);
    write_be_u16(&payload[16], (uint16_t)telemetry->yaw_speed_rpm);
    write_be_u16(&payload[18], (uint16_t)telemetry->pitch_speed_rpm);
    write_be_u16(&payload[20], (uint16_t)telemetry->yaw_current_x100_a);
    write_be_u16(&payload[22], (uint16_t)telemetry->pitch_current_x100_a);
    write_be_i32(&payload[24], telemetry->yaw_target_x100_deg);
    write_be_i32(&payload[28], telemetry->pitch_target_x100_deg);
    write_be_i32(&payload[32], telemetry->limits.yaw_min_x100_deg);
    write_be_i32(&payload[36], telemetry->limits.yaw_max_x100_deg);
    write_be_i32(&payload[40], telemetry->limits.pitch_min_x100_deg);
    write_be_i32(&payload[44], telemetry->limits.pitch_max_x100_deg);
    write_be_u16(&payload[48], telemetry->command_speed_rpm);
    write_be_u32(&payload[52], telemetry->yaw_feedback_age_ms);
    write_be_u32(&payload[56], telemetry->pitch_feedback_age_ms);
    write_be_u32(&payload[60], telemetry->command_timeout_remaining_ms);
    write_be_i32(&payload[64], telemetry->startup_pitch_x100_deg);
    return rpmsg_encode(type, seq, payload, sizeof(payload), out, out_size);
}

static void update_torque_test_peak(uint8_t motor_id)
{
    MotorFeedback feedback;
    int index = (int)motor_id - 1;
    int current_abs;
    int peak_current_abs;
    int speed_abs;
    int peak_speed_abs;

    if (index < 0 || index >= 2 ||
        phytium_can_get_motor_feedback(motor_id, &feedback) != 0) {
        return;
    }
    current_abs = feedback.current_x100_a < 0 ?
        -(int)feedback.current_x100_a : (int)feedback.current_x100_a;
    peak_current_abs = g_torque_test_peak_current_x100[index] < 0 ?
        -(int)g_torque_test_peak_current_x100[index] :
        (int)g_torque_test_peak_current_x100[index];
    if (current_abs > peak_current_abs) {
        g_torque_test_peak_current_x100[index] = feedback.current_x100_a;
    }
    speed_abs = feedback.speed_rpm < 0 ?
        -(int)feedback.speed_rpm : (int)feedback.speed_rpm;
    peak_speed_abs = g_torque_test_peak_speed_rpm[index] < 0 ?
        -(int)g_torque_test_peak_speed_rpm[index] :
        (int)g_torque_test_peak_speed_rpm[index];
    if (speed_abs > peak_speed_abs) {
        g_torque_test_peak_speed_rpm[index] = feedback.speed_rpm;
    }
}

static void handle_can_torque_test(const uint8_t *payload, uint8_t length)
{
    const BalanceTelemetry *balance = balance_control_get_telemetry();
    MotorCanFrame can_frame;
    uint8_t motor_id;
    int16_t torque_x100_nm;
    uint16_t duration_ms;

    if (length < 5U || balance->state == BALANCE_STATE_ACTIVE ||
        balance->state == BALANCE_STATE_ARMING) {
        return;
    }
    motor_id = payload[0];
    torque_x100_nm = (int16_t)read_be_u16(&payload[1]);
    duration_ms = read_be_u16(&payload[3]);
    if (motor_id < 1U || motor_id > 2U || torque_x100_nm < -22 ||
        torque_x100_nm > 22 || duration_ms < 20U || duration_ms > 2000U) {
        return;
    }

    balance_control_disable();
    memset(g_torque_test_peak_current_x100, 0,
           sizeof(g_torque_test_peak_current_x100));
    memset(g_torque_test_peak_speed_rpm, 0,
           sizeof(g_torque_test_peak_speed_rpm));
    motor_build_set_mode(motor_id, 0U, &can_frame);
    if (send_motor_frame(&can_frame) != 0) {
        return;
    }
    fsleep_millisec(5U);
    motor_build_enable(motor_id, &can_frame);
    if (send_motor_frame(&can_frame) != 0) {
        return;
    }
    fsleep_millisec(5U);

    for (uint16_t elapsed = 0U; elapsed < duration_ms; elapsed += 10U) {
        uint32_t feedback_count_before =
            phytium_can_get_debug_state()->feedback_count;
        motor_build_torque(motor_id, torque_x100_nm, &can_frame);
        if (send_motor_frame(&can_frame) != 0) {
            break;
        }
        fsleep_millisec(2U);
        (void)phytium_can_poll();
        if (phytium_can_get_debug_state()->feedback_count !=
            feedback_count_before) {
            update_torque_test_peak(motor_id);
        }
        fsleep_millisec(8U);
    }
    motor_build_torque(motor_id, 0, &can_frame);
    (void)send_motor_frame(&can_frame);
    fsleep_millisec(5U);
    motor_build_idle(motor_id, &can_frame);
    (void)send_motor_frame(&can_frame);
}

static void handle_can_motor_fault(const uint8_t *payload, uint8_t length)
{
    const BalanceTelemetry *balance = balance_control_get_telemetry();
    const GimbalTelemetry *gimbal = gimbal_control_get_telemetry();
    MotorCanFrame can_frame;
    MotorRegisterValue value;
    uint8_t motor_id;

    g_motor_fault_code = 0U;
    g_motor_fault_id = 0U;
    g_motor_fault_read_ret = -1;
    if (length < 1U || balance->state == BALANCE_STATE_ACTIVE ||
        balance->state == BALANCE_STATE_ARMING) {
        return;
    }
    motor_id = payload[0];
    if (motor_id < 1U || motor_id > 4U ||
        (is_gimbal_motor(motor_id) &&
         gimbal->state != GIMBAL_STATE_DISABLED &&
         gimbal->state != GIMBAL_STATE_FAULT)) {
        return;
    }

    g_motor_fault_id = motor_id;
    phytium_can_clear_register_value(motor_id);
    motor_build_read_u32(motor_id, 0x000cU, &can_frame);
    if (send_motor_frame(&can_frame) != 0) {
        g_motor_fault_read_ret = -2;
        return;
    }

    for (uint32_t elapsed = 0U; elapsed < 50U; ++elapsed) {
        fsleep_millisec(1U);
        (void)phytium_can_poll();
        if (phytium_can_get_register_value(motor_id, &value) == 0 &&
            value.address == 0x000cU) {
            g_motor_fault_code = value.value;
            g_motor_fault_read_ret = 0;
            return;
        }
    }
    g_motor_fault_read_ret = -3;
}

static int32_t position_delta_speed_x100_rpm(
    const MotorFeedback *previous, const MotorFeedback *current)
{
    int64_t delta_position;
    uint64_t delta_ticks;
    uint64_t timer_frequency = GenericTimerFrequecy();

    if (timer_frequency == 0U || current->update_tick <= previous->update_tick) {
        return 0;
    }
    delta_position = (int64_t)current->position_x100_deg -
                     (int64_t)previous->position_x100_deg;
    delta_ticks = current->update_tick - previous->update_tick;

    /* x100 deg -> x100 rpm: delta * timer_hz / (6 * delta_ticks). */
    return (int32_t)(delta_position * (int64_t)timer_frequency /
                     (6LL * (int64_t)delta_ticks));
}

static void handle_can_speed_diag(const uint8_t *payload, uint8_t length)
{
    const BalanceTelemetry *balance = balance_control_get_telemetry();
    MotorCanFrame frame;
    MotorFeedback previous = {0};
    MotorFeedback current;
    MotorRegisterValue register_value;
    uint8_t motor_id;
    int16_t torque_x100_nm;
    uint16_t duration_ms;
    int32_t latest_position_speed = 0;

    memset(&g_speed_diag, 0, sizeof(g_speed_diag));
    g_speed_diag.status = 1U;
    if (length < 5U || balance->state == BALANCE_STATE_ACTIVE ||
        balance->state == BALANCE_STATE_ARMING) {
        return;
    }
    motor_id = payload[0];
    torque_x100_nm = (int16_t)read_be_u16(&payload[1]);
    duration_ms = read_be_u16(&payload[3]);
    g_speed_diag.motor_id = motor_id;
    if (motor_id < 1U || motor_id > 2U || torque_x100_nm < -10 ||
        torque_x100_nm > 10 || duration_ms < 200U || duration_ms > 2000U) {
        return;
    }

    balance_control_disable();
    phytium_can_clear_motor_feedback(motor_id);
    motor_build_set_mode(motor_id, 0U, &frame);
    if (send_motor_frame(&frame) != 0) {
        g_speed_diag.status = 2U;
        return;
    }
    fsleep_millisec(5U);
    motor_build_enable(motor_id, &frame);
    if (send_motor_frame(&frame) != 0) {
        g_speed_diag.status = 2U;
        return;
    }
    fsleep_millisec(5U);

    for (uint16_t elapsed = 0U; elapsed < duration_ms; elapsed += 10U) {
        motor_build_torque(motor_id, torque_x100_nm, &frame);
        if (send_motor_frame(&frame) != 0) {
            g_speed_diag.status = 2U;
            break;
        }
        fsleep_millisec(2U);
        (void)phytium_can_poll();
        if (phytium_can_get_motor_feedback(motor_id, &current) == 0 &&
            (!previous.valid || current.update_tick != previous.update_tick)) {
            if (previous.valid) {
                latest_position_speed =
                    position_delta_speed_x100_rpm(&previous, &current);
                g_speed_diag.valid_flags |= 0x04U;
            }
            previous = current;
            g_speed_diag.sample_count++;
            g_speed_diag.valid_flags |= 0x01U;
        }

        if ((elapsed % 50U) == 0U) {
            phytium_can_clear_register_value(motor_id);
            motor_build_read_u32(motor_id, 0x0006U, &frame);
            if (send_motor_frame(&frame) == 0) {
                for (uint32_t wait_ms = 0U; wait_ms < 8U; ++wait_ms) {
                    fsleep_millisec(1U);
                    (void)phytium_can_poll();
                    if (phytium_can_get_register_value(motor_id,
                                                       &register_value) == 0 &&
                        register_value.address == 0x0006U) {
                        int32_t periodic = previous.valid ?
                            (int32_t)previous.speed_rpm * 100 : 0;
                        int32_t magnitude = periodic < 0 ? -periodic : periodic;
                        int32_t best = g_speed_diag.periodic_speed_x100_rpm < 0 ?
                            -g_speed_diag.periodic_speed_x100_rpm :
                            g_speed_diag.periodic_speed_x100_rpm;

                        if ((g_speed_diag.valid_flags & 0x04U) != 0U &&
                            magnitude >= best) {
                            g_speed_diag.periodic_speed_x100_rpm = periodic;
                            g_speed_diag.register_speed_x100_rpm =
                                (int32_t)register_value.value;
                            g_speed_diag.position_speed_x100_rpm =
                                latest_position_speed;
                        }
                        g_speed_diag.valid_flags |= 0x02U;
                        break;
                    }
                }
            }
        } else {
            fsleep_millisec(8U);
        }
    }

    motor_build_torque(motor_id, 0, &frame);
    (void)send_motor_frame(&frame);
    fsleep_millisec(5U);
    motor_build_idle(motor_id, &frame);
    (void)send_motor_frame(&frame);
    if (g_speed_diag.status != 2U) {
        g_speed_diag.status = g_speed_diag.valid_flags == 0x07U ? 0U : 3U;
    }
}

int slave_app_init(void)
{
    int balance_ret = balance_control_init();
    int gimbal_ret = gimbal_control_init();
    int servo_ret = servo_motion_init();
    uint64_t timer_frequency = GenericTimerFrequecy();

    if (timer_frequency != 0U) {
        g_active_gimbal_poll_interval_ticks =
            timer_frequency / ACTIVE_GIMBAL_POLL_HZ;
        g_next_active_gimbal_poll_tick =
            GenericTimerRead(GENERIC_TIMER_ID0) +
            g_active_gimbal_poll_interval_ticks;
    }

    if (balance_ret != 0) {
        return balance_ret;
    }
    return gimbal_ret != 0 ? gimbal_ret : servo_ret;
}

void slave_app_poll(void)
{
    const BalanceTelemetry *balance;
    uint64_t now;

    balance_control_poll();

    /*
     * The balance loop is time-critical (100 Hz).  Keep the servo PWM motion
     * controller paused while balancing, but continue polling the CAN gimbal
     * at 20 Hz.  The gimbal refreshes its position command every 50 ms; fully
     * skipping this poll makes motors 3/4 lose holding torque during a balance
     * run even though a wheel fault only idles motors 1/2.
     */
    balance = balance_control_get_telemetry();
    if (balance != NULL && balance->state == BALANCE_STATE_ACTIVE) {
        now = GenericTimerRead(GENERIC_TIMER_ID0);
        if (g_active_gimbal_poll_interval_ticks != 0U &&
            (int64_t)(now - g_next_active_gimbal_poll_tick) >= 0) {
            g_next_active_gimbal_poll_tick =
                now + g_active_gimbal_poll_interval_ticks;
            gimbal_control_poll();
        }
        return;
    }

    if (g_active_gimbal_poll_interval_ticks != 0U) {
        g_next_active_gimbal_poll_tick =
            GenericTimerRead(GENERIC_TIMER_ID0) +
            g_active_gimbal_poll_interval_ticks;
    }
    gimbal_control_poll();
    servo_motion_poll();
}

void slave_app_shutdown(void)
{
    gimbal_control_shutdown();
    balance_control_disable();
}

static void service_gimbal_during_balance_calibration(void *context)
{
    (void)context;
    gimbal_control_poll();
}

size_t slave_handle_frame(const uint8_t *data, unsigned int len, uint8_t *reply, size_t reply_size)
{
    RpmsgFrame frame;
    if (!rpmsg_decode(data, len, &frame)) {
        return 0;
    }
    g_last_command_type = frame.type;

    switch (frame.type) {
    case CMD_HEARTBEAT:
        g_state.heartbeat_ok = 1;
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_ENABLE:
        if (frame.length >= 1 && !is_gimbal_motor(frame.payload[0])) {
            handle_can_enable(frame.payload[0]);
        }
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_SET_MODE:
        if (frame.length >= 3 && !is_gimbal_motor(frame.payload[0])) {
            handle_can_set_mode(frame.payload[0], read_be_u16(&frame.payload[1]));
        }
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_INIT_MOTOR:
        if (frame.length >= 1 && !is_gimbal_motor(frame.payload[0])) {
            handle_can_init_motor(frame.payload[0]);
        }
        return build_ack(frame.seq, reply, reply_size);
    case CMD_MOTOR_TEST:
        handle_motor_test();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_TORQUE_TEST:
        handle_can_torque_test(frame.payload, frame.length);
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_MOTOR_FAULT:
        handle_can_motor_fault(frame.payload, frame.length);
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_SPEED_DIAG:
        handle_can_speed_diag(frame.payload, frame.length);
        return build_speed_diag_ack(frame.seq, reply, reply_size);
    case CMD_CAN_SET_ORIGIN:
        if (frame.length >= 1) {
            if (is_gimbal_motor(frame.payload[0])) {
                gimbal_control_emergency_stop();
                (void)gimbal_control_reset_limits();
            }
            handle_can_set_origin(frame.payload[0]);
        }
        return build_ack(frame.seq, reply, reply_size);
    case CMD_SERVO_SET4:
        handle_servo_set4(frame.payload, frame.length);
        return build_ack(frame.seq, reply, reply_size);
    case CMD_SERVO_CENTER:
        handle_servo_center();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_SERVO_POLARITY:
        handle_servo_polarity(frame.payload, frame.length);
        return build_ack(frame.seq, reply, reply_size);
    case CMD_SERVO_MOVE4: {
        int status = handle_servo_move4(frame.payload, frame.length);
        return build_servo_motion_ack(frame.type, frame.seq, (uint8_t)status,
                                      reply, reply_size);
    }
    case CMD_SERVO_STATUS:
        return build_servo_motion_ack(frame.type, frame.seq, SERVO_MOTION_OK,
                                      reply, reply_size);
    case CMD_SERVO_STOP:
        servo_motion_stop();
        return build_servo_motion_ack(frame.type, frame.seq, SERVO_MOTION_OK,
                                      reply, reply_size);
    case CMD_LEG_ENABLE: {
        const uint16_t safe_reference[PHYTIUM_SERVO_NUM] = {
            450U, 1350U, 450U, 1350U
        };
        int status = handle_servo_enable4(safe_reference);
        return build_servo_motion_ack(frame.type, frame.seq, (uint8_t)status,
                                      reply, reply_size);
    }
    case CMD_LEG_MOVE4: {
        int status = handle_leg_move4(frame.payload, frame.length);
        return build_servo_motion_ack(frame.type, frame.seq, (uint8_t)status,
                                      reply, reply_size);
    }
    case CMD_SERVO_TEST_ONE: {
        int status = SERVO_MOTION_INVALID;
        if (frame.length >= 5U &&
            balance_control_get_telemetry()->state == BALANCE_STATE_DISABLED) {
            status = servo_motion_test_one(frame.payload[0],
                                           read_be_u16(&frame.payload[1]),
                                           read_be_u16(&frame.payload[3]));
        } else if (balance_control_get_telemetry()->state !=
                   BALANCE_STATE_DISABLED) {
            status = SERVO_MOTION_BALANCE_ACTIVE;
        }
        return build_servo_motion_ack(frame.type, frame.seq, (uint8_t)status,
                                      reply, reply_size);
    }
    case CMD_IMU_INIT:
        handle_imu_init();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_IMU_READ:
        handle_imu_read();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_IMU_TELEMETRY:
        return build_imu_telemetry_ack(frame.type, frame.seq, UINT8_MAX,
                                       reply, reply_size);
    case CMD_IMU_CALIBRATE: {
        uint8_t status = 2U;
        uint16_t samples = frame.length >= 2U ?
            read_be_u16(frame.payload) : 100U;
        if (balance_control_get_telemetry()->state == BALANCE_STATE_DISABLED &&
            gimbal_control_get_telemetry()->state == GIMBAL_STATE_DISABLED) {
            status = 3U;
            if (samples >= 20U && samples <= 500U &&
                phytium_bmi088_calibrate_gyro(samples) == 0 &&
                phytium_bmi088_update(0.01f) == 0) {
                status = 0U;
            }
        }
        return build_imu_telemetry_ack(frame.type, frame.seq, status,
                                       reply, reply_size);
    }
    case CMD_BALANCE_ENABLE:
        servo_motion_stop();
        (void)balance_control_enable_serviced(
            service_gimbal_during_balance_calibration, NULL);
        return build_ack(frame.seq, reply, reply_size);
    case CMD_BALANCE_DISABLE:
        balance_control_disable();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_BALANCE_STATUS:
        return build_ack(frame.seq, reply, reply_size);
    case CMD_BALANCE_TELEMETRY:
        return build_balance_telemetry_ack(frame.seq, reply, reply_size);
    case CMD_BALANCE_SET_TRIM: {
        int status = BALANCE_CONFIG_INVALID;
        if (frame.length >= 4U) {
            status = balance_control_set_pitch_trim(
                (float)read_be_i32(frame.payload) / 1000000.0f);
        }
        return build_balance_config_ack(frame.type, frame.seq, (uint8_t)status,
                                        reply, reply_size);
    }
    case CMD_BALANCE_SET_GAINS: {
        int status = BALANCE_CONFIG_INVALID;
        if (frame.length >= 16U) {
            status = balance_control_set_gains(
                (float)read_be_i32(&frame.payload[0]) / 1000000.0f,
                (float)read_be_i32(&frame.payload[4]) / 1000000.0f,
                (float)read_be_i32(&frame.payload[8]) / 1000000.0f,
                (float)read_be_i32(&frame.payload[12]) / 1000000.0f);
        }
        return build_balance_config_ack(frame.type, frame.seq, (uint8_t)status,
                                        reply, reply_size);
    }
    case CMD_BALANCE_CONFIG:
        return build_balance_config_ack(frame.type, frame.seq,
                                        BALANCE_CONFIG_OK, reply, reply_size);
    case CMD_BALANCE_RESET_CONFIG:
        return build_balance_config_ack(
            frame.type, frame.seq,
            (uint8_t)balance_control_reset_runtime_config(), reply, reply_size);
    case CMD_BALANCE_SET_SPEED_LIMIT: {
        int status = BALANCE_CONFIG_INVALID;
        if (frame.length >= 4U) {
            status = balance_control_set_speed_limit(
                (float)read_be_i32(frame.payload) / 1000000.0f);
        }
        return build_balance_config_ack(frame.type, frame.seq, (uint8_t)status,
                                        reply, reply_size);
    }
    case CMD_BALANCE_SET_FILTER: {
        int status = BALANCE_CONFIG_INVALID;
        if (frame.length >= 4U) {
            status = balance_control_set_pitch_rate_filter(
                (float)read_be_i32(frame.payload) / 1000000.0f);
        }
        return build_balance_config_ack(frame.type, frame.seq, (uint8_t)status,
                                        reply, reply_size);
    }
    case CMD_BALANCE_SET_POSTURE_PRIORITY: {
        int status = BALANCE_CONFIG_INVALID;
        if (frame.length >= 4U) {
            status = balance_control_set_posture_priority(
                (float)read_be_i32(frame.payload) / 1000000.0f);
        }
        return build_balance_config_ack(frame.type, frame.seq, (uint8_t)status,
                                        reply, reply_size);
    }
    case CMD_BALANCE_SET_TORQUE_LIMIT: {
        int status = BALANCE_CONFIG_INVALID;
        if (frame.length >= 4U) {
            status = balance_control_set_torque_limit(
                (float)read_be_i32(frame.payload) / 1000000.0f);
        }
        return build_balance_config_ack(frame.type, frame.seq, (uint8_t)status,
                                        reply, reply_size);
    }
    case CMD_BALANCE_SET_POSITION_HOLD: {
        int status = BALANCE_CONFIG_INVALID;
        if (frame.length == 1U && frame.payload[0] == 0U) {
            status = balance_control_set_position_hold(0U, 0.0f, 0.0f, 0.0f);
        } else if (frame.length >= 13U && frame.payload[0] == 1U) {
            status = balance_control_set_position_hold(
                1U,
                (float)read_be_i32(&frame.payload[1]) / 1000000.0f,
                (float)read_be_i32(&frame.payload[5]) / 1000000.0f,
                (float)read_be_i32(&frame.payload[9]) / 1000000.0f);
        }
        return build_balance_config_ack(frame.type, frame.seq, (uint8_t)status,
                                        reply, reply_size);
    }
    case CMD_CHASSIS_SET_VELOCITY: {
        int status = BALANCE_MOTION_INVALID;
        if (frame.length >= 10U) {
            status = balance_control_set_motion_command(
                (float)read_be_i32(&frame.payload[0]) / 1000000.0f,
                (float)read_be_i32(&frame.payload[4]) / 1000000.0f,
                read_be_u16(&frame.payload[8]));
        }
        return build_chassis_telemetry_ack(frame.type, frame.seq,
                                           (uint8_t)status,
                                           reply, reply_size);
    }
    case CMD_CHASSIS_STATUS:
        return build_chassis_telemetry_ack(frame.type, frame.seq,
                                           BALANCE_MOTION_OK,
                                           reply, reply_size);
    case CMD_CHASSIS_SET_TRACK_WIDTH: {
        int status = BALANCE_CONFIG_INVALID;
        if (frame.length >= 4U) {
            status = balance_control_set_wheel_track(
                (float)read_be_i32(frame.payload) / 1000000.0f);
        }
        return build_chassis_telemetry_ack(frame.type, frame.seq,
                                           (uint8_t)status,
                                           reply, reply_size);
    }
    case CMD_CAN_ZERO_POSITION:
        if (frame.length >= 1 && !is_gimbal_motor(frame.payload[0])) {
            handle_can_temporary_origin(frame.payload[0]);
        }
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_PVT:
        handle_can_pvt(frame.payload, frame.length);
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_SAFE_STOP:
        if (frame.length >= 1) {
            if (is_gimbal_motor(frame.payload[0])) {
                gimbal_control_emergency_stop();
            } else {
                handle_can_safe_stop(frame.payload[0]);
            }
        } else {
            handle_can_safe_stop(1);
            handle_can_safe_stop(2);
        }
        return build_ack(frame.seq, reply, reply_size);
    case CMD_GIMBAL_ENABLE: {
        uint8_t home_torque_percent =
            frame.length >= 1U ? frame.payload[0] : 50U;
        uint16_t home_speed_rpm =
            frame.length >= 3U ? read_be_u16(&frame.payload[1]) : 5U;
        uint8_t return_torque_percent =
            frame.length >= 4U ? frame.payload[3] : home_torque_percent;
        uint16_t return_speed_rpm =
            frame.length >= 6U ? read_be_u16(&frame.payload[4]) : 5U;
        int status = gimbal_control_enable(home_torque_percent,
                                           home_speed_rpm,
                                           return_torque_percent,
                                           return_speed_rpm);
        return build_gimbal_ack(frame.type, frame.seq, (uint8_t)status,
                                reply, reply_size);
    }
    case CMD_GIMBAL_DISABLE: {
        int status = gimbal_control_disable();
        return build_gimbal_ack(frame.type, frame.seq, (uint8_t)status,
                                reply, reply_size);
    }
    case CMD_GIMBAL_SET_TARGET: {
        int status = GIMBAL_STATUS_INVALID;
        if (frame.length >= 13U) {
            status = gimbal_control_set_target(
                read_be_i32(&frame.payload[0]),
                read_be_i32(&frame.payload[4]),
                read_be_u16(&frame.payload[8]), frame.payload[10],
                read_be_u16(&frame.payload[11]));
        }
        return build_gimbal_ack(frame.type, frame.seq, (uint8_t)status,
                                reply, reply_size);
    }
    case CMD_GIMBAL_STATUS:
        return build_gimbal_ack(frame.type, frame.seq, GIMBAL_STATUS_OK,
                                reply, reply_size);
    case CMD_GIMBAL_CALIBRATE_LIMIT: {
        int status = GIMBAL_STATUS_INVALID;
        if (frame.length >= 2U) {
            status = gimbal_control_calibrate_limit(frame.payload[0],
                                                    frame.payload[1]);
        }
        return build_gimbal_ack(frame.type, frame.seq, (uint8_t)status,
                                reply, reply_size);
    }
    case CMD_GIMBAL_SET_LIMITS: {
        GimbalLimits limits;
        int status = GIMBAL_STATUS_INVALID;
        if (frame.length >= 16U) {
            limits.yaw_min_x100_deg = read_be_i32(&frame.payload[0]);
            limits.yaw_max_x100_deg = read_be_i32(&frame.payload[4]);
            limits.pitch_min_x100_deg = read_be_i32(&frame.payload[8]);
            limits.pitch_max_x100_deg = read_be_i32(&frame.payload[12]);
            limits.valid_mask = GIMBAL_LIMIT_ALL_VALID;
            status = gimbal_control_set_limits(&limits);
        }
        return build_gimbal_ack(frame.type, frame.seq, (uint8_t)status,
                                reply, reply_size);
    }
    case CMD_GIMBAL_RESET_LIMITS: {
        int status = gimbal_control_reset_limits();
        return build_gimbal_ack(frame.type, frame.seq, (uint8_t)status,
                                reply, reply_size);
    }
    case CMD_GIMBAL_EMERGENCY_STOP:
        gimbal_control_emergency_stop();
        return build_gimbal_ack(frame.type, frame.seq, GIMBAL_STATUS_OK,
                                reply, reply_size);
    default:
        return 0;
    }
}
