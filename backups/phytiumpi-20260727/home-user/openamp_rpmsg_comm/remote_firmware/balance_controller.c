#include "balance_controller.h"

#include "fgeneric_timer.h"
#include "fparameters.h"
#include "fsleep.h"
#include "lqr_controller.h"
#include "motor_can.h"
#include "phytium_bmi088_port.h"
#include "phytium_can_port.h"
#include "position_hold_controller.h"

#include <math.h>
#include <string.h>

#define BALANCE_CONTROL_HZ 100U
#define BALANCE_LEFT_MOTOR_ID 1U
#define BALANCE_RIGHT_MOTOR_ID 2U
#define BALANCE_IMU_CALIBRATION_SAMPLES 100U
#define BALANCE_FEEDBACK_TIMEOUT_MS 100U
#define BALANCE_ARM_TIMEOUT_MS 2000U
#define BALANCE_ARM_MAX_WHEEL_SPEED_M_S 0.10f
#define BALANCE_ARM_MAX_PITCH_RAD (5.0f * BALANCE_PI / 180.0f)
#define BALANCE_ARM_MAX_PITCH_RATE_RAD_S 0.15f
#define BALANCE_DEFAULT_MAX_WHEEL_SPEED_M_S 1.0f
#define BALANCE_MIN_CONFIG_WHEEL_SPEED_M_S 0.2f
#define BALANCE_MAX_CONFIG_WHEEL_SPEED_M_S 2.0f
#define BALANCE_PI 3.14159265358979323846f
#define BALANCE_MOTOR_FEEDBACK_SPEED_SCALE 1.0f
#define BALANCE_DEFAULT_PITCH_RATE_FILTER_HZ 20.0f
#define BALANCE_MIN_PITCH_RATE_FILTER_HZ 5.0f
#define BALANCE_MAX_PITCH_RATE_FILTER_HZ 40.0f
#define BALANCE_IMU_MOUNT_PITCH_RAD 0.0f
#define BALANCE_POSTURE_PRIORITY_ANGLE_RAD (3.0f * BALANCE_PI / 180.0f)
#define BALANCE_MIN_POSTURE_PRIORITY_ANGLE_RAD (1.0f * BALANCE_PI / 180.0f)
#define BALANCE_MAX_POSTURE_PRIORITY_ANGLE_RAD (10.0f * BALANCE_PI / 180.0f)
#define BALANCE_MIN_TORQUE_LIMIT_NM 0.05f
#define BALANCE_MAX_TORQUE_LIMIT_NM 0.30f
#define BALANCE_MAX_PITCH_TRIM_RAD (5.0f * BALANCE_PI / 180.0f)
#define BALANCE_MAX_COMMAND_LINEAR_M_S 0.40f
#define BALANCE_MAX_COMMAND_ANGULAR_RAD_S 1.0f
#define BALANCE_LINEAR_ACCEL_LIMIT_M_S2 0.50f
#define BALANCE_ANGULAR_ACCEL_LIMIT_RAD_S2 2.0f
#define BALANCE_MIN_COMMAND_TIMEOUT_MS 100U
#define BALANCE_MAX_COMMAND_TIMEOUT_MS 1000U
#define BALANCE_MIN_WHEEL_TRACK_M 0.08f
#define BALANCE_MAX_WHEEL_TRACK_M 0.50f
#define BALANCE_YAW_RATE_KP_NM_PER_RAD_S 0.04f
#define BALANCE_MAX_YAW_TORQUE_NM 0.05f
#define BALANCE_DEFAULT_POSITION_HOLD_KP_RAD_PER_M (1.5f * BALANCE_PI / 180.0f)
#define BALANCE_DEFAULT_POSITION_HOLD_KD_RAD_PER_M_S (2.0f * BALANCE_PI / 180.0f)
#define BALANCE_DEFAULT_POSITION_HOLD_LIMIT_RAD (0.8f * BALANCE_PI / 180.0f)
#define BALANCE_MAX_POSITION_HOLD_KP_RAD_PER_M 0.5f
#define BALANCE_MAX_POSITION_HOLD_KD_RAD_PER_M_S 1.0f
#define BALANCE_MIN_POSITION_HOLD_LIMIT_RAD (0.1f * BALANCE_PI / 180.0f)
#define BALANCE_MAX_POSITION_HOLD_LIMIT_RAD (3.0f * BALANCE_PI / 180.0f)

static LqrController g_lqr;
static BalanceTelemetry g_telemetry;
static uint64_t g_timer_frequency;
static uint64_t g_period_ticks;
static uint64_t g_next_tick;
static uint64_t g_arm_tick;
static float g_max_wheel_speed_m_s = BALANCE_DEFAULT_MAX_WHEEL_SPEED_M_S;
static float g_pitch_rate_filter_hz = BALANCE_DEFAULT_PITCH_RATE_FILTER_HZ;
static float g_filtered_pitch_rate_rad_s;
static uint8_t g_pitch_rate_filter_valid;
static BalanceMotionTelemetry g_motion;
static uint64_t g_motion_command_tick;
static uint64_t g_motion_timeout_ticks;
static uint8_t g_motion_command_valid;
static float g_position_target_m;
static PositionHoldConfig g_position_hold = {
    .kp_rad_per_m = BALANCE_DEFAULT_POSITION_HOLD_KP_RAD_PER_M,
    .kd_rad_per_m_s = BALANCE_DEFAULT_POSITION_HOLD_KD_RAD_PER_M_S,
    .pitch_limit_rad = BALANCE_DEFAULT_POSITION_HOLD_LIMIT_RAD,
    .enabled = 0U,
};

static const LqrConfig g_default_lqr_config = {
    /* 100 Hz discrete LQR; input is tau_left + tau_right in N*m. */
    .k_theta = -3.759673794f,
    .k_theta_rate = -0.486784559f,
    .k_position = -0.062456846f,
    .k_velocity = -0.247058408f,
    .wheel_radius_m = 0.03225f,
    .torque_limit_nm = 0.22f,
    .fall_angle_rad = 15.0f * BALANCE_PI / 180.0f,
    .pitch_offset_rad = BALANCE_IMU_MOUNT_PITCH_RAD,
    .posture_priority_angle_rad = BALANCE_POSTURE_PRIORITY_ANGLE_RAD,
    .left_motor_direction = 1.0f,
    .right_motor_direction = -1.0f,
};

static int config_change_allowed(void)
{
    return g_telemetry.state != BALANCE_STATE_ACTIVE &&
           g_telemetry.state != BALANCE_STATE_ARMING;
}
static uint8_t g_initialized;

static uint64_t ms_to_ticks(uint32_t milliseconds)
{
    return g_timer_frequency * milliseconds / 1000U;
}

static float clamp_value(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float approach(float current, float target, float maximum_step)
{
    float difference = target - current;

    if (difference > maximum_step) {
        return current + maximum_step;
    }
    if (difference < -maximum_step) {
        return current - maximum_step;
    }
    return target;
}

static void clear_motion_command(void)
{
    g_motion.target_linear_m_s = 0.0f;
    g_motion.target_angular_rad_s = 0.0f;
    g_motion.applied_linear_m_s = 0.0f;
    g_motion.applied_angular_rad_s = 0.0f;
    g_motion.command_age_ms = UINT32_MAX;
    g_motion_command_tick = 0U;
    g_motion_timeout_ticks = 0U;
    g_motion_command_valid = 0U;
}

static int send_frame(const MotorCanFrame *frame)
{
    return phytium_can_send(frame);
}

static int send_torque(uint8_t motor_id, float torque_nm)
{
    MotorCanFrame frame;
    float scaled = torque_nm * 100.0f;
    int16_t torque_x100;

    if (scaled > 32767.0f) {
        scaled = 32767.0f;
    } else if (scaled < -32768.0f) {
        scaled = -32768.0f;
    }
    torque_x100 = (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
    motor_build_torque(motor_id, torque_x100, &frame);
    return send_frame(&frame);
}

static void stop_motors(uint8_t enter_idle)
{
    MotorCanFrame frame;

    (void)send_torque(BALANCE_LEFT_MOTOR_ID, 0.0f);
    (void)send_torque(BALANCE_RIGHT_MOTOR_ID, 0.0f);
    if (enter_idle) {
        motor_build_idle(BALANCE_LEFT_MOTOR_ID, &frame);
        (void)send_frame(&frame);
        motor_build_idle(BALANCE_RIGHT_MOTOR_ID, &frame);
        (void)send_frame(&frame);
    }
}

static void enter_fault(uint8_t fault)
{
    lqr_disable(&g_lqr);
    g_telemetry.fault |= fault;
    g_telemetry.state = BALANCE_STATE_FAULT;
    g_telemetry.left_torque_nm = 0.0f;
    g_telemetry.right_torque_nm = 0.0f;
    g_telemetry.pitch_target_rad = 0.0f;
    g_telemetry.position_target_m = 0.0f;
    g_telemetry.position_error_m = 0.0f;
    g_telemetry.velocity_error_m_s = 0.0f;
    g_telemetry.position_hold_enabled = g_position_hold.enabled;
    clear_motion_command();
    stop_motors(1U);
}

static int feedback_is_fresh(const MotorFeedback *feedback, uint64_t now)
{
    return feedback->valid &&
           now - feedback->update_tick <= ms_to_ticks(BALANCE_FEEDBACK_TIMEOUT_MS);
}

static int read_lqr_sensor(uint64_t now, LqrSensorData *sensor,
                           uint8_t *fault)
{
    PhytiumBmi088Sample imu;
    MotorFeedback left;
    MotorFeedback right;

    memset(sensor, 0, sizeof(*sensor));
    *fault = BALANCE_FAULT_NONE;
    if (phytium_bmi088_get_sample(&imu) != 0 || !imu.valid ||
        now - imu.update_tick > ms_to_ticks(30U)) {
        *fault |= BALANCE_FAULT_IMU;
    }
    if (phytium_can_get_motor_feedback(BALANCE_LEFT_MOTOR_ID, &left) != 0 ||
        !feedback_is_fresh(&left, now)) {
        *fault |= BALANCE_FAULT_LEFT_MOTOR;
    }
    if (phytium_can_get_motor_feedback(BALANCE_RIGHT_MOTOR_ID, &right) != 0 ||
        !feedback_is_fresh(&right, now)) {
        *fault |= BALANCE_FAULT_RIGHT_MOTOR;
    }
    if (*fault != BALANCE_FAULT_NONE) {
        return -1;
    }

    sensor->pitch_rad = imu.pitch_rad;
    sensor->pitch_rate_rad_s = imu.pitch_rate_rad_s;
    sensor->left_position_rad = (float)left.position_x100_deg *
                                BALANCE_PI / 18000.0f;
    sensor->right_position_rad = (float)right.position_x100_deg *
                                 BALANCE_PI / 18000.0f;
    /* The 0x2A field matches register 0x0006 and position-derived speed. */
    sensor->left_velocity_rad_s = (float)left.speed_rpm *
                                  BALANCE_MOTOR_FEEDBACK_SPEED_SCALE *
                                  2.0f * BALANCE_PI / 60.0f;
    sensor->right_velocity_rad_s = (float)right.speed_rpm *
                                   BALANCE_MOTOR_FEEDBACK_SPEED_SCALE *
                                   2.0f * BALANCE_PI / 60.0f;
    sensor->valid = 1U;
    return 0;
}

static float filter_pitch_rate(float pitch_rate_rad_s)
{
    const float dt = 1.0f / (float)BALANCE_CONTROL_HZ;
    const float rc = 1.0f / (2.0f * BALANCE_PI * g_pitch_rate_filter_hz);
    const float alpha = dt / (rc + dt);

    if (!g_pitch_rate_filter_valid) {
        g_filtered_pitch_rate_rad_s = pitch_rate_rad_s;
        g_pitch_rate_filter_valid = 1U;
    } else {
        g_filtered_pitch_rate_rad_s +=
            alpha * (pitch_rate_rad_s - g_filtered_pitch_rate_rad_s);
    }
    return g_filtered_pitch_rate_rad_s;
}

int balance_control_init(void)
{
    memset(&g_telemetry, 0, sizeof(g_telemetry));
    g_telemetry.state = BALANCE_STATE_DISABLED;
    g_telemetry.control_hz = BALANCE_CONTROL_HZ;
    memset(&g_motion, 0, sizeof(g_motion));
    g_motion.command_age_ms = UINT32_MAX;
    g_timer_frequency = GenericTimerFrequecy();
    if (g_timer_frequency == 0U ||
        lqr_init(&g_lqr, &g_default_lqr_config) != 0) {
        g_telemetry.fault = BALANCE_FAULT_CONFIG;
        g_telemetry.state = BALANCE_STATE_FAULT;
        return -1;
    }
    g_period_ticks = g_timer_frequency / BALANCE_CONTROL_HZ;
    g_next_tick = GenericTimerRead(GENERIC_TIMER_ID0) + g_period_ticks;

    if (phytium_can_init() != 0 || phytium_bmi088_init() != 0) {
        g_telemetry.fault = BALANCE_FAULT_CAN | BALANCE_FAULT_IMU;
        g_telemetry.state = BALANCE_STATE_FAULT;
        return -1;
    }

    g_initialized = 1U;
    return 0;
}

int balance_control_enable_serviced(BalanceCalibrationService service,
                                    void *context)
{
    MotorCanFrame frame;
    int ret = 0;

    if (!g_initialized || g_telemetry.state == BALANCE_STATE_ACTIVE ||
        g_telemetry.state == BALANCE_STATE_ARMING) {
        return -1;
    }

    g_telemetry.fault = BALANCE_FAULT_NONE;
    lqr_disable(&g_lqr);
    if (phytium_bmi088_calibrate_gyro_serviced(
            BALANCE_IMU_CALIBRATION_SAMPLES,
            service,
            context) != 0) {
        enter_fault(BALANCE_FAULT_IMU);
        return -1;
    }
    /* Drain old replies, then require feedback generated by this arming run. */
    (void)phytium_can_poll();
    phytium_can_clear_motor_feedback(BALANCE_LEFT_MOTOR_ID);
    phytium_can_clear_motor_feedback(BALANCE_RIGHT_MOTOR_ID);
    g_telemetry.wheel_position_m = 0.0f;
    g_telemetry.wheel_velocity_m_s = 0.0f;
    g_telemetry.left_torque_nm = 0.0f;
    g_telemetry.right_torque_nm = 0.0f;
    g_telemetry.pitch_target_rad = 0.0f;
    g_telemetry.position_target_m = 0.0f;
    g_telemetry.position_error_m = 0.0f;
    g_telemetry.velocity_error_m_s = 0.0f;
    g_telemetry.position_hold_enabled = g_position_hold.enabled;
    g_pitch_rate_filter_valid = 0U;
    clear_motion_command();
    g_motion.wheel_position_m = 0.0f;
    g_motion.yaw_position_rad = 0.0f;
    g_position_target_m = 0.0f;
    motor_build_set_mode(BALANCE_LEFT_MOTOR_ID, 0U, &frame);
    ret |= send_frame(&frame);
    motor_build_set_mode(BALANCE_RIGHT_MOTOR_ID, 0U, &frame);
    ret |= send_frame(&frame);
    fsleep_millisec(5U);
    motor_build_enable(BALANCE_LEFT_MOTOR_ID, &frame);
    ret |= send_frame(&frame);
    motor_build_enable(BALANCE_RIGHT_MOTOR_ID, &frame);
    ret |= send_frame(&frame);
    fsleep_millisec(5U);
    ret |= send_torque(BALANCE_LEFT_MOTOR_ID, 0.0f);
    ret |= send_torque(BALANCE_RIGHT_MOTOR_ID, 0.0f);
    if (ret != 0) {
        enter_fault(BALANCE_FAULT_CAN);
        return -1;
    }

    g_arm_tick = GenericTimerRead(GENERIC_TIMER_ID0);
    g_telemetry.state = BALANCE_STATE_ARMING;
    return 0;
}

int balance_control_enable(void)
{
    return balance_control_enable_serviced(NULL, NULL);
}

void balance_control_disable(void)
{
    if (!g_initialized) {
        return;
    }
    lqr_disable(&g_lqr);
    stop_motors(1U);
    g_telemetry.state = BALANCE_STATE_DISABLED;
    g_telemetry.fault = BALANCE_FAULT_NONE;
    g_telemetry.left_torque_nm = 0.0f;
    g_telemetry.right_torque_nm = 0.0f;
    g_telemetry.pitch_target_rad = 0.0f;
    g_telemetry.position_target_m = 0.0f;
    g_telemetry.position_error_m = 0.0f;
    g_telemetry.velocity_error_m_s = 0.0f;
    g_telemetry.position_hold_enabled = g_position_hold.enabled;
    clear_motion_command();
}

void balance_control_poll(void)
{
    uint64_t now;
    LqrSensorData sensor;
    LqrOutput output;
    uint8_t fault;
    float left_velocity_m_s;
    float right_velocity_m_s;
    float yaw_torque_nm;
    float yaw_error_rad_s;
    float wheel_position_m;
    float wheel_velocity_m_s;
    float pitch_target_rad;

    if (!g_initialized) {
        return;
    }
    (void)phytium_can_poll();
    now = GenericTimerRead(GENERIC_TIMER_ID0);
    if ((int64_t)(now - g_next_tick) < 0) {
        return;
    }
    if (g_telemetry.state == BALANCE_STATE_ACTIVE &&
        now - g_next_tick > 2U * g_period_ticks) {
        g_next_tick = now + g_period_ticks;
        enter_fault(BALANCE_FAULT_OVERRUN);
        return;
    }
    g_next_tick += g_period_ticks;
    if (now - g_next_tick > g_period_ticks) {
        g_next_tick = now + g_period_ticks;
    }

    if (phytium_bmi088_update(1.0f / (float)BALANCE_CONTROL_HZ) != 0) {
        if (g_telemetry.state == BALANCE_STATE_ACTIVE) {
            enter_fault(BALANCE_FAULT_IMU);
        }
        return;
    }
    now = GenericTimerRead(GENERIC_TIMER_ID0);
    if (g_telemetry.state == BALANCE_STATE_DISABLED) {
        PhytiumBmi088Sample imu;
        if (phytium_bmi088_get_sample(&imu) == 0 && imu.valid) {
            g_telemetry.pitch_rad =
                imu.pitch_rad - g_lqr.config.pitch_offset_rad;
            g_telemetry.pitch_rate_rad_s = imu.pitch_rate_rad_s;
        }
        return;
    }
    if (g_telemetry.state == BALANCE_STATE_FAULT) {
        return;
    }
    if (!phytium_can_bus_ok()) {
        enter_fault(BALANCE_FAULT_CAN);
        return;
    }

    if (g_telemetry.state == BALANCE_STATE_ARMING) {
        (void)send_torque(BALANCE_LEFT_MOTOR_ID, 0.0f);
        (void)send_torque(BALANCE_RIGHT_MOTOR_ID, 0.0f);
        if (read_lqr_sensor(now, &sensor, &fault) == 0) {
            if (fabsf(sensor.pitch_rad - g_lqr.config.pitch_offset_rad) >
                    BALANCE_ARM_MAX_PITCH_RAD ||
                fabsf(sensor.pitch_rate_rad_s) >
                    BALANCE_ARM_MAX_PITCH_RATE_RAD_S ||
                fabsf(sensor.left_velocity_rad_s * g_lqr.config.wheel_radius_m) >
                    BALANCE_ARM_MAX_WHEEL_SPEED_M_S ||
                fabsf(sensor.right_velocity_rad_s * g_lqr.config.wheel_radius_m) >
                    BALANCE_ARM_MAX_WHEEL_SPEED_M_S) {
                enter_fault(BALANCE_FAULT_ARM_CONDITION);
                return;
            }
            if (lqr_enable(&g_lqr, &sensor) == 0) {
                g_filtered_pitch_rate_rad_s = sensor.pitch_rate_rad_s;
                g_pitch_rate_filter_valid = 1U;
                g_telemetry.state = BALANCE_STATE_ACTIVE;
                return;
            }
            enter_fault(BALANCE_FAULT_CONFIG);
        } else if (now - g_arm_tick > ms_to_ticks(BALANCE_ARM_TIMEOUT_MS)) {
            enter_fault((uint8_t)(fault | BALANCE_FAULT_ARM_TIMEOUT));
        }
        return;
    }

    if (read_lqr_sensor(now, &sensor, &fault) != 0) {
        enter_fault(fault);
        return;
    }
    sensor.pitch_rate_rad_s = filter_pitch_rate(sensor.pitch_rate_rad_s);
    if (g_motion_command_valid) {
        uint64_t age_ticks = now - g_motion_command_tick;
        g_motion.command_age_ms = (uint32_t)
            ((age_ticks * 1000U) / g_timer_frequency);
        if (age_ticks > g_motion_timeout_ticks) {
            g_motion.target_linear_m_s = 0.0f;
            g_motion.target_angular_rad_s = 0.0f;
            g_motion_command_valid = 0U;
        }
    } else {
        g_motion.command_age_ms = UINT32_MAX;
    }
    g_motion.applied_linear_m_s = approach(
        g_motion.applied_linear_m_s, g_motion.target_linear_m_s,
        BALANCE_LINEAR_ACCEL_LIMIT_M_S2 / (float)BALANCE_CONTROL_HZ);
    g_motion.applied_angular_rad_s = approach(
        g_motion.applied_angular_rad_s, g_motion.target_angular_rad_s,
        BALANCE_ANGULAR_ACCEL_LIMIT_RAD_S2 / (float)BALANCE_CONTROL_HZ);
    g_position_target_m +=
        g_motion.applied_linear_m_s / (float)BALANCE_CONTROL_HZ;
    wheel_position_m = 0.5f * g_lqr.config.wheel_radius_m *
        (g_lqr.config.left_motor_direction * sensor.left_position_rad +
         g_lqr.config.right_motor_direction * sensor.right_position_rad) -
        g_lqr.wheel_zero_position_m;
    wheel_velocity_m_s = 0.5f * g_lqr.config.wheel_radius_m *
        (g_lqr.config.left_motor_direction * sensor.left_velocity_rad_s +
         g_lqr.config.right_motor_direction * sensor.right_velocity_rad_s);
    g_telemetry.position_target_m = g_position_target_m;
    g_telemetry.position_error_m = wheel_position_m - g_position_target_m;
    g_telemetry.velocity_error_m_s =
        wheel_velocity_m_s - g_motion.applied_linear_m_s;
    g_telemetry.position_hold_enabled = g_position_hold.enabled;
    if (g_position_hold.enabled) {
        pitch_target_rad = position_hold_pitch_target(
            &g_position_hold,
            g_telemetry.position_error_m,
            g_telemetry.velocity_error_m_s);
        (void)lqr_set_targets(&g_lqr, pitch_target_rad, wheel_position_m,
                              wheel_velocity_m_s);
    } else {
        pitch_target_rad = 0.0f;
        (void)lqr_set_targets(&g_lqr, 0.0f, g_position_target_m,
                              g_motion.applied_linear_m_s);
    }
    g_telemetry.pitch_target_rad = pitch_target_rad;
    output = lqr_update(&g_lqr, &sensor);
    if (output.fault || !output.enabled) {
        enter_fault(BALANCE_FAULT_FALL);
        return;
    }
    if (fabsf(sensor.left_velocity_rad_s * g_lqr.config.wheel_radius_m) >
            g_max_wheel_speed_m_s ||
        fabsf(sensor.right_velocity_rad_s * g_lqr.config.wheel_radius_m) >
            g_max_wheel_speed_m_s) {
        enter_fault(BALANCE_FAULT_SPEED);
        return;
    }
    left_velocity_m_s = g_lqr.config.left_motor_direction *
        sensor.left_velocity_rad_s * g_lqr.config.wheel_radius_m;
    right_velocity_m_s = g_lqr.config.right_motor_direction *
        sensor.right_velocity_rad_s * g_lqr.config.wheel_radius_m;
    g_motion.measured_linear_m_s =
        0.5f * (left_velocity_m_s + right_velocity_m_s);
    if (g_motion.wheel_track_m >= BALANCE_MIN_WHEEL_TRACK_M) {
        g_motion.measured_angular_rad_s =
            (right_velocity_m_s - left_velocity_m_s) /
            g_motion.wheel_track_m;
    } else {
        g_motion.measured_angular_rad_s = 0.0f;
    }
    g_motion.wheel_position_m = output.wheel_position_m;
    g_motion.yaw_position_rad +=
        g_motion.measured_angular_rad_s / (float)BALANCE_CONTROL_HZ;
    yaw_error_rad_s = g_motion.applied_angular_rad_s -
                      g_motion.measured_angular_rad_s;
    yaw_torque_nm = clamp_value(
        BALANCE_YAW_RATE_KP_NM_PER_RAD_S * yaw_error_rad_s,
        BALANCE_MAX_YAW_TORQUE_NM);
    if (fabsf(sensor.pitch_rad - g_lqr.config.pitch_offset_rad) >=
        g_lqr.config.posture_priority_angle_rad) {
        yaw_torque_nm = 0.0f;
    }
    output.left_torque_nm = clamp_value(
        output.left_torque_nm -
            g_lqr.config.left_motor_direction * yaw_torque_nm,
        g_lqr.config.torque_limit_nm);
    output.right_torque_nm = clamp_value(
        output.right_torque_nm +
            g_lqr.config.right_motor_direction * yaw_torque_nm,
        g_lqr.config.torque_limit_nm);
    if (send_torque(BALANCE_LEFT_MOTOR_ID, output.left_torque_nm) != 0 ||
        send_torque(BALANCE_RIGHT_MOTOR_ID, output.right_torque_nm) != 0) {
        enter_fault(BALANCE_FAULT_CAN);
        return;
    }

    g_telemetry.pitch_rad = sensor.pitch_rad - g_lqr.config.pitch_offset_rad;
    g_telemetry.pitch_rate_rad_s = sensor.pitch_rate_rad_s;
    g_telemetry.wheel_position_m = output.wheel_position_m;
    g_telemetry.wheel_velocity_m_s = output.wheel_velocity_m_s;
    g_telemetry.left_torque_nm = output.left_torque_nm;
    g_telemetry.right_torque_nm = output.right_torque_nm;
    g_telemetry.loop_count++;
}

const BalanceTelemetry *balance_control_get_telemetry(void)
{
    return &g_telemetry;
}

void balance_control_get_runtime_config(BalanceRuntimeConfig *config)
{
    if (config == NULL) {
        return;
    }
    config->pitch_trim_rad = g_lqr.config.pitch_offset_rad;
    config->k_theta = g_lqr.config.k_theta;
    config->k_theta_rate = g_lqr.config.k_theta_rate;
    config->k_position = g_lqr.config.k_position;
    config->k_velocity = g_lqr.config.k_velocity;
    config->posture_priority_angle_rad =
        g_lqr.config.posture_priority_angle_rad;
    config->max_wheel_speed_m_s = g_max_wheel_speed_m_s;
    config->motor_feedback_speed_scale = BALANCE_MOTOR_FEEDBACK_SPEED_SCALE;
    config->pitch_rate_filter_hz = g_pitch_rate_filter_hz;
    config->torque_limit_nm = g_lqr.config.torque_limit_nm;
    config->position_hold_kp_rad_per_m = g_position_hold.kp_rad_per_m;
    config->position_hold_kd_rad_per_m_s = g_position_hold.kd_rad_per_m_s;
    config->position_hold_limit_rad = g_position_hold.pitch_limit_rad;
    config->position_hold_enabled = g_position_hold.enabled;
}

int balance_control_set_pitch_trim(float pitch_trim_rad)
{
    if (!config_change_allowed()) {
        return BALANCE_CONFIG_BUSY;
    }
    if (!isfinite(pitch_trim_rad) ||
        fabsf(pitch_trim_rad) > BALANCE_MAX_PITCH_TRIM_RAD) {
        return BALANCE_CONFIG_INVALID;
    }
    g_lqr.config.pitch_offset_rad = pitch_trim_rad;
    return BALANCE_CONFIG_OK;
}

int balance_control_set_gains(float k_theta, float k_theta_rate,
                              float k_position, float k_velocity)
{
    if (!config_change_allowed()) {
        return BALANCE_CONFIG_BUSY;
    }
    if (!isfinite(k_theta) || !isfinite(k_theta_rate) ||
        !isfinite(k_position) || !isfinite(k_velocity) ||
        k_theta < -10.0f || k_theta > -0.1f ||
        k_theta_rate < -5.0f || k_theta_rate > 0.0f ||
        k_position < -2.0f || k_position > 0.0f ||
        k_velocity < -2.0f || k_velocity > 0.0f) {
        return BALANCE_CONFIG_INVALID;
    }
    g_lqr.config.k_theta = k_theta;
    g_lqr.config.k_theta_rate = k_theta_rate;
    g_lqr.config.k_position = k_position;
    g_lqr.config.k_velocity = k_velocity;
    return BALANCE_CONFIG_OK;
}

int balance_control_reset_runtime_config(void)
{
    if (!config_change_allowed()) {
        return BALANCE_CONFIG_BUSY;
    }
    g_lqr.config = g_default_lqr_config;
    g_max_wheel_speed_m_s = BALANCE_DEFAULT_MAX_WHEEL_SPEED_M_S;
    g_pitch_rate_filter_hz = BALANCE_DEFAULT_PITCH_RATE_FILTER_HZ;
    g_pitch_rate_filter_valid = 0U;
    g_position_hold.kp_rad_per_m =
        BALANCE_DEFAULT_POSITION_HOLD_KP_RAD_PER_M;
    g_position_hold.kd_rad_per_m_s =
        BALANCE_DEFAULT_POSITION_HOLD_KD_RAD_PER_M_S;
    g_position_hold.pitch_limit_rad =
        BALANCE_DEFAULT_POSITION_HOLD_LIMIT_RAD;
    g_position_hold.enabled = 0U;
    g_telemetry.position_hold_enabled = 0U;
    return BALANCE_CONFIG_OK;
}

int balance_control_set_speed_limit(float max_wheel_speed_m_s)
{
    if (!config_change_allowed()) {
        return BALANCE_CONFIG_BUSY;
    }
    if (!isfinite(max_wheel_speed_m_s) ||
        max_wheel_speed_m_s < BALANCE_MIN_CONFIG_WHEEL_SPEED_M_S ||
        max_wheel_speed_m_s > BALANCE_MAX_CONFIG_WHEEL_SPEED_M_S) {
        return BALANCE_CONFIG_INVALID;
    }
    g_max_wheel_speed_m_s = max_wheel_speed_m_s;
    return BALANCE_CONFIG_OK;
}

int balance_control_set_pitch_rate_filter(float cutoff_hz)
{
    if (!config_change_allowed()) {
        return BALANCE_CONFIG_BUSY;
    }
    if (!isfinite(cutoff_hz) ||
        cutoff_hz < BALANCE_MIN_PITCH_RATE_FILTER_HZ ||
        cutoff_hz > BALANCE_MAX_PITCH_RATE_FILTER_HZ) {
        return BALANCE_CONFIG_INVALID;
    }
    g_pitch_rate_filter_hz = cutoff_hz;
    g_pitch_rate_filter_valid = 0U;
    return BALANCE_CONFIG_OK;
}

int balance_control_set_posture_priority(float angle_rad)
{
    if (!config_change_allowed()) {
        return BALANCE_CONFIG_BUSY;
    }
    if (!isfinite(angle_rad) ||
        angle_rad < BALANCE_MIN_POSTURE_PRIORITY_ANGLE_RAD ||
        angle_rad > BALANCE_MAX_POSTURE_PRIORITY_ANGLE_RAD) {
        return BALANCE_CONFIG_INVALID;
    }
    g_lqr.config.posture_priority_angle_rad = angle_rad;
    return BALANCE_CONFIG_OK;
}

int balance_control_set_torque_limit(float torque_limit_nm)
{
    if (!config_change_allowed()) {
        return BALANCE_CONFIG_BUSY;
    }
    if (!isfinite(torque_limit_nm) ||
        torque_limit_nm < BALANCE_MIN_TORQUE_LIMIT_NM ||
        torque_limit_nm > BALANCE_MAX_TORQUE_LIMIT_NM) {
        return BALANCE_CONFIG_INVALID;
    }
    g_lqr.config.torque_limit_nm = torque_limit_nm;
    return BALANCE_CONFIG_OK;
}

int balance_control_set_position_hold(uint8_t enabled, float kp_rad_per_m,
                                      float kd_rad_per_m_s,
                                      float pitch_limit_rad)
{
    PositionHoldConfig config;

    if (!config_change_allowed()) {
        return BALANCE_CONFIG_BUSY;
    }
    if (enabled == 0U) {
        g_position_hold.enabled = 0U;
        g_telemetry.position_hold_enabled = 0U;
        return BALANCE_CONFIG_OK;
    }
    config.kp_rad_per_m = kp_rad_per_m;
    config.kd_rad_per_m_s = kd_rad_per_m_s;
    config.pitch_limit_rad = pitch_limit_rad;
    config.enabled = 1U;
    if (!position_hold_config_is_valid(&config) ||
        kp_rad_per_m > BALANCE_MAX_POSITION_HOLD_KP_RAD_PER_M ||
        kd_rad_per_m_s > BALANCE_MAX_POSITION_HOLD_KD_RAD_PER_M_S ||
        pitch_limit_rad < BALANCE_MIN_POSITION_HOLD_LIMIT_RAD ||
        pitch_limit_rad > BALANCE_MAX_POSITION_HOLD_LIMIT_RAD) {
        return BALANCE_CONFIG_INVALID;
    }
    g_position_hold = config;
    g_telemetry.position_hold_enabled = 1U;
    return BALANCE_CONFIG_OK;
}

int balance_control_set_motion_command(float linear_m_s, float angular_rad_s,
                                       uint16_t timeout_ms)
{
    if (g_telemetry.state != BALANCE_STATE_ACTIVE) {
        return BALANCE_MOTION_NOT_ACTIVE;
    }
    if (!isfinite(linear_m_s) || !isfinite(angular_rad_s) ||
        fabsf(linear_m_s) > BALANCE_MAX_COMMAND_LINEAR_M_S ||
        fabsf(angular_rad_s) > BALANCE_MAX_COMMAND_ANGULAR_RAD_S ||
        timeout_ms < BALANCE_MIN_COMMAND_TIMEOUT_MS ||
        timeout_ms > BALANCE_MAX_COMMAND_TIMEOUT_MS ||
        (fabsf(angular_rad_s) > 0.0f &&
         g_motion.wheel_track_m < BALANCE_MIN_WHEEL_TRACK_M)) {
        return BALANCE_MOTION_INVALID;
    }
    g_motion.target_linear_m_s = linear_m_s;
    g_motion.target_angular_rad_s = angular_rad_s;
    g_motion_command_tick = GenericTimerRead(GENERIC_TIMER_ID0);
    g_motion_timeout_ticks = ms_to_ticks(timeout_ms);
    g_motion_command_valid = 1U;
    g_motion.command_age_ms = 0U;
    return BALANCE_MOTION_OK;
}

int balance_control_set_wheel_track(float wheel_track_m)
{
    if (!config_change_allowed()) {
        return BALANCE_CONFIG_BUSY;
    }
    if (!isfinite(wheel_track_m) ||
        wheel_track_m < BALANCE_MIN_WHEEL_TRACK_M ||
        wheel_track_m > BALANCE_MAX_WHEEL_TRACK_M) {
        return BALANCE_CONFIG_INVALID;
    }
    g_motion.wheel_track_m = wheel_track_m;
    return BALANCE_CONFIG_OK;
}

void balance_control_get_motion_telemetry(BalanceMotionTelemetry *telemetry)
{
    if (telemetry != NULL) {
        *telemetry = g_motion;
    }
}
