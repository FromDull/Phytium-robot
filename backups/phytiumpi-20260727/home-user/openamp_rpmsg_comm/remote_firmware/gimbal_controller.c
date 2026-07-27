#include "gimbal_controller.h"

#include "fgeneric_timer.h"
#include "fparameters.h"
#include "motor_can.h"
#include "phytium_can_port.h"

#include <limits.h>
#include <string.h>

#define GIMBAL_HOME_SPEED_RPM 5U
#define GIMBAL_DEFAULT_SPEED_RPM 20U
#define GIMBAL_DEFAULT_TORQUE_PERCENT 50U
#define GIMBAL_MIN_HOME_TORQUE_PERCENT 5U
#define GIMBAL_MAX_TORQUE_PERCENT 80U
#define GIMBAL_MIN_CONFIG_SPEED_RPM 1U
#define GIMBAL_MAX_CONFIG_SPEED_RPM 60U
/*
 * balance-enable performs a synchronous one-second gyro calibration.  CAN
 * feedback polling pauses during that window, so retain the last valid gimbal
 * sample long enough to resume position keepalives afterwards.  The Linux
 * daemon still rejects new motion commands once feedback is 500 ms old.
 */
#define GIMBAL_FEEDBACK_TIMEOUT_MS 2000U
#define GIMBAL_HOME_TIMEOUT_MS 12000U
#define GIMBAL_START_DELAY_MS 20U
#define GIMBAL_HOME_TOLERANCE_X100_DEG 100
#define GIMBAL_HOME_SPEED_TOLERANCE_RPM 5
#define GIMBAL_HOME_SETTLE_SAMPLES 10U
#define GIMBAL_LIMIT_OVERSHOOT_X100_DEG 100
#define GIMBAL_TARGET_TOLERANCE_X100_DEG 100
#define GIMBAL_MOTION_SETTLE_MARGIN_MS 2000U
#define GIMBAL_MOTION_TIMEOUT_MIN_MS 5000U
#define GIMBAL_MOTION_TIMEOUT_MAX_MS 15000U
#define GIMBAL_STOP_STEP_MS 100U
#define GIMBAL_STOP_TORQUE_STEP_PERCENT 2U
#define GIMBAL_MAX_ABS_LIMIT_X100_DEG 36000
#define GIMBAL_CALIBRATION_START_DELAY_MS 20U
#define GIMBAL_CALIBRATION_KEEPALIVE_MS 100U
#define GIMBAL_CALIBRATION_TIMEOUT_MS 30000U
#define GIMBAL_POSITION_REFRESH_MS 50U

static GimbalTelemetry g_telemetry;
static uint64_t g_timer_frequency;
static uint64_t g_state_tick;
static uint64_t g_target_tick;
static uint64_t g_stop_step_tick;
static uint64_t g_motion_deadline_tick;
static uint16_t g_target_timeout_ms;
static uint8_t g_start_phase;
static uint8_t g_home_settle_samples;
static uint8_t g_return_settle_samples;
static uint8_t g_stop_torque_percent;
static uint8_t g_home_torque_percent;
static uint8_t g_return_torque_percent;
static uint16_t g_home_speed_rpm;
static uint16_t g_return_speed_rpm;
static uint8_t g_startup_pitch_valid;
static uint8_t g_calibration_feedback_active;
static uint8_t g_calibration_phase;
static uint64_t g_calibration_step_tick;
static uint64_t g_calibration_activity_tick;
static uint64_t g_position_refresh_tick;

static uint64_t ms_to_ticks(uint32_t milliseconds)
{
    return g_timer_frequency * (uint64_t)milliseconds / 1000U;
}

static uint32_t ticks_to_ms(uint64_t ticks)
{
    if (g_timer_frequency == 0U) {
        return UINT32_MAX;
    }
    ticks = ticks * 1000U / g_timer_frequency;
    return ticks > UINT32_MAX ? UINT32_MAX : (uint32_t)ticks;
}

static int send_frame(const MotorCanFrame *frame)
{
    return phytium_can_send(frame) == 0 ? 0 : -1;
}

static int send_idle_both(void)
{
    MotorCanFrame frame;
    int ret = 0;

    motor_build_idle(GIMBAL_YAW_MOTOR_ID, &frame);
    ret |= send_frame(&frame);
    motor_build_idle(GIMBAL_PITCH_MOTOR_ID, &frame);
    ret |= send_frame(&frame);
    return ret;
}

static int send_zero_torque_both(void)
{
    MotorCanFrame frame;

    motor_build_torque(GIMBAL_YAW_MOTOR_ID, 0, &frame);
    if (send_frame(&frame) != 0) {
        return -1;
    }
    motor_build_torque(GIMBAL_PITCH_MOTOR_ID, 0, &frame);
    return send_frame(&frame);
}

static int stop_calibration_feedback(void)
{
    int ret = 0;

    if (g_calibration_feedback_active) {
        ret = send_idle_both();
    }
    g_calibration_feedback_active = 0U;
    g_calibration_phase = 0U;
    return ret;
}

static int start_calibration_feedback(uint64_t now)
{
    MotorCanFrame frame;

    g_calibration_activity_tick = now;
    if (g_calibration_feedback_active) {
        return 0;
    }

    /* Torque mode with a zero command provides feedback without motion. */
    motor_build_set_mode(GIMBAL_YAW_MOTOR_ID, 0U, &frame);
    if (send_frame(&frame) != 0) {
        return -1;
    }
    motor_build_set_mode(GIMBAL_PITCH_MOTOR_ID, 0U, &frame);
    if (send_frame(&frame) != 0 || send_zero_torque_both() != 0) {
        return -1;
    }
    g_calibration_feedback_active = 1U;
    g_calibration_phase = 0U;
    g_calibration_step_tick = now;
    g_telemetry.command_speed_rpm = 0U;
    g_telemetry.command_torque_percent = 0U;
    return 0;
}

static int send_position_both(int32_t yaw_x100_deg,
                              int32_t pitch_x100_deg,
                              uint16_t speed_rpm,
                              uint8_t torque_percent)
{
    MotorCanFrame frame;

    motor_build_pvt(GIMBAL_YAW_MOTOR_ID, yaw_x100_deg, speed_rpm,
                    torque_percent, &frame);
    if (send_frame(&frame) != 0) {
        return -1;
    }
    motor_build_pvt(GIMBAL_PITCH_MOTOR_ID, pitch_x100_deg, speed_rpm,
                    torque_percent, &frame);
    return send_frame(&frame);
}

static int limits_are_valid(const GimbalLimits *limits)
{
    return limits != 0 && limits->valid_mask == GIMBAL_LIMIT_ALL_VALID &&
           limits->yaw_min_x100_deg < 0 && limits->yaw_max_x100_deg > 0 &&
           limits->pitch_min_x100_deg < 0 && limits->pitch_max_x100_deg > 0 &&
           limits->yaw_min_x100_deg < limits->yaw_max_x100_deg &&
           limits->pitch_min_x100_deg < limits->pitch_max_x100_deg &&
           limits->yaw_min_x100_deg >= -GIMBAL_MAX_ABS_LIMIT_X100_DEG &&
           limits->yaw_max_x100_deg <= GIMBAL_MAX_ABS_LIMIT_X100_DEG &&
           limits->pitch_min_x100_deg >= -GIMBAL_MAX_ABS_LIMIT_X100_DEG &&
           limits->pitch_max_x100_deg <= GIMBAL_MAX_ABS_LIMIT_X100_DEG;
}

static int target_is_valid(int32_t yaw_x100_deg, int32_t pitch_x100_deg)
{
    return limits_are_valid(&g_telemetry.limits) &&
           yaw_x100_deg >= g_telemetry.limits.yaw_min_x100_deg &&
           yaw_x100_deg <= g_telemetry.limits.yaw_max_x100_deg &&
           pitch_x100_deg >= g_telemetry.limits.pitch_min_x100_deg &&
           pitch_x100_deg <= g_telemetry.limits.pitch_max_x100_deg;
}

static int32_t abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static void set_motion_deadline(uint64_t now, int32_t yaw_target_x100_deg,
                                int32_t pitch_target_x100_deg,
                                uint16_t speed_rpm)
{
    uint32_t max_error = (uint32_t)abs_i32(
        yaw_target_x100_deg - g_telemetry.yaw_position_x100_deg);
    uint32_t pitch_error = (uint32_t)abs_i32(
        pitch_target_x100_deg - g_telemetry.pitch_position_x100_deg);
    uint32_t travel_ms;

    if (pitch_error > max_error) {
        max_error = pitch_error;
    }
    travel_ms = max_error * 1000U / ((uint32_t)speed_rpm * 600U);
    travel_ms += GIMBAL_MOTION_SETTLE_MARGIN_MS;
    if (travel_ms < GIMBAL_MOTION_TIMEOUT_MIN_MS) {
        travel_ms = GIMBAL_MOTION_TIMEOUT_MIN_MS;
    }
    if (travel_ms > GIMBAL_MOTION_TIMEOUT_MAX_MS) {
        travel_ms = GIMBAL_MOTION_TIMEOUT_MAX_MS;
    }
    g_motion_deadline_tick = now + ms_to_ticks(travel_ms);
}

static int feedback_is_fresh(const MotorFeedback *feedback, uint64_t now)
{
    return feedback->valid && now >= feedback->update_tick &&
           now - feedback->update_tick <=
               ms_to_ticks(GIMBAL_FEEDBACK_TIMEOUT_MS);
}

static void enter_fault(uint8_t fault)
{
    g_telemetry.fault |= fault;
    g_telemetry.state = GIMBAL_STATE_FAULT;
    g_telemetry.command_torque_percent = 0U;
    g_calibration_feedback_active = 0U;
    g_calibration_phase = 0U;
    (void)send_idle_both();
}

static void update_feedback(uint64_t now)
{
    MotorFeedback yaw = {0};
    MotorFeedback pitch = {0};

    g_telemetry.feedback_valid_mask = 0U;
    g_telemetry.yaw_feedback_age_ms = UINT32_MAX;
    g_telemetry.pitch_feedback_age_ms = UINT32_MAX;
    if (phytium_can_get_motor_feedback(GIMBAL_YAW_MOTOR_ID, &yaw) == 0) {
        g_telemetry.yaw_position_x100_deg = yaw.position_x100_deg;
        g_telemetry.yaw_speed_rpm = yaw.speed_rpm;
        g_telemetry.yaw_current_x100_a = yaw.current_x100_a;
        g_telemetry.yaw_feedback_age_ms =
            now >= yaw.update_tick ? ticks_to_ms(now - yaw.update_tick) : 0U;
        if (feedback_is_fresh(&yaw, now)) {
            g_telemetry.feedback_valid_mask |= 0x01U;
        }
    }
    if (phytium_can_get_motor_feedback(GIMBAL_PITCH_MOTOR_ID, &pitch) == 0) {
        g_telemetry.pitch_position_x100_deg = pitch.position_x100_deg;
        g_telemetry.pitch_speed_rpm = pitch.speed_rpm;
        g_telemetry.pitch_current_x100_a = pitch.current_x100_a;
        g_telemetry.pitch_feedback_age_ms =
            now >= pitch.update_tick ? ticks_to_ms(now - pitch.update_tick) : 0U;
        if (feedback_is_fresh(&pitch, now)) {
            g_telemetry.feedback_valid_mask |= 0x02U;
        }
    }
}

static int actual_position_exceeded_limits(void)
{
    if (!limits_are_valid(&g_telemetry.limits)) {
        return 0;
    }
    return g_telemetry.yaw_position_x100_deg <
               g_telemetry.limits.yaw_min_x100_deg -
                   GIMBAL_LIMIT_OVERSHOOT_X100_DEG ||
           g_telemetry.yaw_position_x100_deg >
               g_telemetry.limits.yaw_max_x100_deg +
                   GIMBAL_LIMIT_OVERSHOOT_X100_DEG ||
           g_telemetry.pitch_position_x100_deg <
               g_telemetry.limits.pitch_min_x100_deg -
                   GIMBAL_LIMIT_OVERSHOOT_X100_DEG ||
           g_telemetry.pitch_position_x100_deg >
               g_telemetry.limits.pitch_max_x100_deg +
                   GIMBAL_LIMIT_OVERSHOOT_X100_DEG;
}

int gimbal_control_init(void)
{
    memset(&g_telemetry, 0, sizeof(g_telemetry));
    g_state_tick = 0U;
    g_target_tick = 0U;
    g_stop_step_tick = 0U;
    g_motion_deadline_tick = 0U;
    g_target_timeout_ms = 0U;
    g_start_phase = 0U;
    g_home_settle_samples = 0U;
    g_return_settle_samples = 0U;
    g_stop_torque_percent = 0U;
    g_home_torque_percent = GIMBAL_DEFAULT_TORQUE_PERCENT;
    g_return_torque_percent = GIMBAL_DEFAULT_TORQUE_PERCENT;
    g_home_speed_rpm = GIMBAL_HOME_SPEED_RPM;
    g_return_speed_rpm = GIMBAL_HOME_SPEED_RPM;
    g_startup_pitch_valid = 0U;
    g_calibration_feedback_active = 0U;
    g_calibration_phase = 0U;
    g_calibration_step_tick = 0U;
    g_calibration_activity_tick = 0U;
    g_position_refresh_tick = 0U;
    g_timer_frequency = GenericTimerFrequecy();
    g_telemetry.state = GIMBAL_STATE_DISABLED;
    g_telemetry.command_speed_rpm = GIMBAL_DEFAULT_SPEED_RPM;
    g_telemetry.command_torque_percent = GIMBAL_DEFAULT_TORQUE_PERCENT;
    return g_timer_frequency == 0U ? -1 : 0;
}

int gimbal_control_enable(uint8_t home_torque_percent,
                          uint16_t home_speed_rpm,
                          uint8_t return_torque_percent,
                          uint16_t return_speed_rpm)
{
    MotorCanFrame frame;
    uint64_t now;

    if (g_telemetry.state != GIMBAL_STATE_DISABLED &&
        g_telemetry.state != GIMBAL_STATE_FAULT) {
        return GIMBAL_STATUS_BUSY;
    }
    if (home_torque_percent < GIMBAL_MIN_HOME_TORQUE_PERCENT ||
        home_torque_percent > GIMBAL_MAX_TORQUE_PERCENT ||
        return_torque_percent < GIMBAL_MIN_HOME_TORQUE_PERCENT ||
        return_torque_percent > GIMBAL_MAX_TORQUE_PERCENT ||
        home_speed_rpm < GIMBAL_MIN_CONFIG_SPEED_RPM ||
        home_speed_rpm > GIMBAL_MAX_CONFIG_SPEED_RPM ||
        return_speed_rpm < GIMBAL_MIN_CONFIG_SPEED_RPM ||
        return_speed_rpm > GIMBAL_MAX_CONFIG_SPEED_RPM) {
        return GIMBAL_STATUS_INVALID;
    }
    if (!limits_are_valid(&g_telemetry.limits)) {
        g_telemetry.fault = GIMBAL_FAULT_LIMIT_CONFIG;
        return GIMBAL_STATUS_LIMITS_NOT_READY;
    }
    now = GenericTimerRead(GENERIC_TIMER_ID0);
    update_feedback(now);
    if (g_telemetry.feedback_valid_mask != 0x03U) {
        if (start_calibration_feedback(now) != 0) {
            enter_fault(GIMBAL_FAULT_CAN);
            return GIMBAL_STATUS_CAN_ERROR;
        }
        return GIMBAL_STATUS_NO_FEEDBACK;
    }
    if (stop_calibration_feedback() != 0) {
        enter_fault(GIMBAL_FAULT_CAN);
        return GIMBAL_STATUS_CAN_ERROR;
    }

    g_telemetry.fault = GIMBAL_FAULT_NONE;
    g_telemetry.startup_pitch_x100_deg =
        g_telemetry.pitch_position_x100_deg;
    g_startup_pitch_valid = 1U;
    g_home_torque_percent = home_torque_percent;
    g_return_torque_percent = return_torque_percent;
    g_home_speed_rpm = home_speed_rpm;
    g_return_speed_rpm = return_speed_rpm;
    g_telemetry.yaw_target_x100_deg = 0;
    g_telemetry.pitch_target_x100_deg = 0;
    g_telemetry.command_speed_rpm = g_home_speed_rpm;
    g_telemetry.command_torque_percent = home_torque_percent;
    g_target_timeout_ms = 0U;
    g_motion_deadline_tick = 0U;
    g_home_settle_samples = 0U;
    g_start_phase = 0U;
    motor_build_set_mode(GIMBAL_YAW_MOTOR_ID, 2U, &frame);
    if (send_frame(&frame) != 0) {
        enter_fault(GIMBAL_FAULT_CAN);
        return GIMBAL_STATUS_CAN_ERROR;
    }
    motor_build_set_mode(GIMBAL_PITCH_MOTOR_ID, 2U, &frame);
    if (send_frame(&frame) != 0) {
        enter_fault(GIMBAL_FAULT_CAN);
        return GIMBAL_STATUS_CAN_ERROR;
    }
    g_state_tick = now;
    g_telemetry.state = GIMBAL_STATE_STARTING;
    return GIMBAL_STATUS_OK;
}

int gimbal_control_disable(void)
{
    uint64_t now;

    if (g_telemetry.state == GIMBAL_STATE_DISABLED) {
        return stop_calibration_feedback() == 0 ?
            GIMBAL_STATUS_OK : GIMBAL_STATUS_CAN_ERROR;
    }
    if (g_telemetry.state == GIMBAL_STATE_RETURNING ||
        g_telemetry.state == GIMBAL_STATE_STOPPING) {
        return GIMBAL_STATUS_OK;
    }
    if (g_telemetry.state == GIMBAL_STATE_FAULT) {
        gimbal_control_emergency_stop();
        return GIMBAL_STATUS_OK;
    }
    now = GenericTimerRead(GENERIC_TIMER_ID0);
    update_feedback(now);
    if (g_telemetry.feedback_valid_mask != 0x03U ||
        !g_startup_pitch_valid ||
        !target_is_valid(g_telemetry.yaw_position_x100_deg,
                         g_telemetry.startup_pitch_x100_deg)) {
        gimbal_control_emergency_stop();
        return GIMBAL_STATUS_NO_FEEDBACK;
    }
    g_telemetry.yaw_target_x100_deg = g_telemetry.yaw_position_x100_deg;
    g_telemetry.pitch_target_x100_deg =
        g_telemetry.startup_pitch_x100_deg;
    g_telemetry.command_speed_rpm = g_return_speed_rpm;
    g_telemetry.command_torque_percent = g_return_torque_percent;
    g_return_settle_samples = 0U;
    g_position_refresh_tick = now;
    set_motion_deadline(now, g_telemetry.yaw_target_x100_deg,
                        g_telemetry.pitch_target_x100_deg,
                        g_return_speed_rpm);
    g_telemetry.state = GIMBAL_STATE_RETURNING;
    if (send_position_both(g_telemetry.yaw_target_x100_deg,
                           g_telemetry.pitch_target_x100_deg,
                           g_return_speed_rpm,
                           g_return_torque_percent) != 0) {
        enter_fault(GIMBAL_FAULT_CAN);
        return GIMBAL_STATUS_CAN_ERROR;
    }
    return GIMBAL_STATUS_OK;
}

void gimbal_control_emergency_stop(void)
{
    (void)send_idle_both();
    g_calibration_feedback_active = 0U;
    g_calibration_phase = 0U;
    g_telemetry.state = GIMBAL_STATE_DISABLED;
    g_telemetry.command_timeout_remaining_ms = 0U;
}

int gimbal_control_set_target(int32_t yaw_x100_deg,
                              int32_t pitch_x100_deg,
                              uint16_t speed_rpm,
                              uint8_t torque_percent,
                              uint16_t timeout_ms)
{
    if (g_telemetry.state != GIMBAL_STATE_ACTIVE) {
        return GIMBAL_STATUS_BUSY;
    }
    if (!target_is_valid(yaw_x100_deg, pitch_x100_deg) ||
        speed_rpm == 0U || speed_rpm > 1000U ||
        torque_percent == 0U ||
        torque_percent > GIMBAL_MAX_TORQUE_PERCENT ||
        (timeout_ms != 0U && timeout_ms < 100U)) {
        return GIMBAL_STATUS_INVALID;
    }
    if (send_position_both(yaw_x100_deg, pitch_x100_deg, speed_rpm,
                           torque_percent) != 0) {
        enter_fault(GIMBAL_FAULT_CAN);
        return GIMBAL_STATUS_CAN_ERROR;
    }
    g_telemetry.yaw_target_x100_deg = yaw_x100_deg;
    g_telemetry.pitch_target_x100_deg = pitch_x100_deg;
    g_telemetry.command_speed_rpm = speed_rpm;
    g_telemetry.command_torque_percent = torque_percent;
    g_target_timeout_ms = timeout_ms;
    g_target_tick = GenericTimerRead(GENERIC_TIMER_ID0);
    g_position_refresh_tick = g_target_tick;
    set_motion_deadline(g_target_tick, yaw_x100_deg, pitch_x100_deg,
                        speed_rpm);
    return GIMBAL_STATUS_OK;
}

int gimbal_control_calibrate_limit(uint8_t axis, uint8_t side)
{
    GimbalLimits candidate = g_telemetry.limits;
    int32_t position;
    uint8_t mask;

    if (g_telemetry.state != GIMBAL_STATE_DISABLED &&
        g_telemetry.state != GIMBAL_STATE_FAULT) {
        return GIMBAL_STATUS_BUSY;
    }
    uint64_t now = GenericTimerRead(GENERIC_TIMER_ID0);

    update_feedback(now);
    if ((axis == GIMBAL_AXIS_YAW &&
         (g_telemetry.feedback_valid_mask & 0x01U) == 0U) ||
        (axis == GIMBAL_AXIS_PITCH &&
         (g_telemetry.feedback_valid_mask & 0x02U) == 0U)) {
        if (start_calibration_feedback(now) != 0) {
            enter_fault(GIMBAL_FAULT_CAN);
            return GIMBAL_STATUS_CAN_ERROR;
        }
        return GIMBAL_STATUS_NO_FEEDBACK;
    }
    if (axis > GIMBAL_AXIS_PITCH || side > GIMBAL_LIMIT_MAX) {
        return GIMBAL_STATUS_INVALID;
    }

    position = axis == GIMBAL_AXIS_YAW ?
        g_telemetry.yaw_position_x100_deg :
        g_telemetry.pitch_position_x100_deg;
    mask = axis == GIMBAL_AXIS_YAW ?
        (side == GIMBAL_LIMIT_MIN ? GIMBAL_LIMIT_YAW_MIN_VALID :
                                    GIMBAL_LIMIT_YAW_MAX_VALID) :
        (side == GIMBAL_LIMIT_MIN ? GIMBAL_LIMIT_PITCH_MIN_VALID :
                                    GIMBAL_LIMIT_PITCH_MAX_VALID);
    if ((side == GIMBAL_LIMIT_MIN && position >= 0) ||
        (side == GIMBAL_LIMIT_MAX && position <= 0) ||
        position < -GIMBAL_MAX_ABS_LIMIT_X100_DEG ||
        position > GIMBAL_MAX_ABS_LIMIT_X100_DEG) {
        return GIMBAL_STATUS_INVALID;
    }

    if (axis == GIMBAL_AXIS_YAW) {
        if (side == GIMBAL_LIMIT_MIN) {
            candidate.yaw_min_x100_deg = position;
        } else {
            candidate.yaw_max_x100_deg = position;
        }
    } else if (side == GIMBAL_LIMIT_MIN) {
        candidate.pitch_min_x100_deg = position;
    } else {
        candidate.pitch_max_x100_deg = position;
    }
    candidate.valid_mask |= mask;
    if (candidate.valid_mask == GIMBAL_LIMIT_ALL_VALID &&
        !limits_are_valid(&candidate)) {
        return GIMBAL_STATUS_INVALID;
    }
    g_telemetry.limits = candidate;
    g_telemetry.limits_valid_mask = candidate.valid_mask;
    g_telemetry.fault &= (uint8_t)~GIMBAL_FAULT_LIMIT_CONFIG;
    g_calibration_activity_tick = now;
    if (candidate.valid_mask == GIMBAL_LIMIT_ALL_VALID &&
        stop_calibration_feedback() != 0) {
        enter_fault(GIMBAL_FAULT_CAN);
        return GIMBAL_STATUS_CAN_ERROR;
    }
    return GIMBAL_STATUS_OK;
}

int gimbal_control_set_limits(const GimbalLimits *limits)
{
    if (g_telemetry.state != GIMBAL_STATE_DISABLED &&
        g_telemetry.state != GIMBAL_STATE_FAULT) {
        return GIMBAL_STATUS_BUSY;
    }
    if (!limits_are_valid(limits)) {
        return GIMBAL_STATUS_INVALID;
    }
    if (stop_calibration_feedback() != 0) {
        enter_fault(GIMBAL_FAULT_CAN);
        return GIMBAL_STATUS_CAN_ERROR;
    }
    g_telemetry.limits = *limits;
    g_telemetry.limits_valid_mask = limits->valid_mask;
    g_telemetry.fault &= (uint8_t)~GIMBAL_FAULT_LIMIT_CONFIG;
    return GIMBAL_STATUS_OK;
}

int gimbal_control_reset_limits(void)
{
    if (g_telemetry.state != GIMBAL_STATE_DISABLED &&
        g_telemetry.state != GIMBAL_STATE_FAULT) {
        return GIMBAL_STATUS_BUSY;
    }
    memset(&g_telemetry.limits, 0, sizeof(g_telemetry.limits));
    g_telemetry.limits_valid_mask = 0U;
    g_telemetry.fault = GIMBAL_FAULT_LIMIT_CONFIG;
    return GIMBAL_STATUS_OK;
}

void gimbal_control_poll(void)
{
    MotorCanFrame frame;
    uint64_t now = GenericTimerRead(GENERIC_TIMER_ID0);

    update_feedback(now);
    g_telemetry.limits_valid_mask = g_telemetry.limits.valid_mask;

    if (g_calibration_feedback_active) {
        if (now - g_calibration_activity_tick >=
            ms_to_ticks(GIMBAL_CALIBRATION_TIMEOUT_MS)) {
            (void)stop_calibration_feedback();
        } else if (g_calibration_phase == 0U &&
                   now - g_calibration_step_tick >=
                       ms_to_ticks(GIMBAL_CALIBRATION_START_DELAY_MS)) {
            motor_build_enable(GIMBAL_YAW_MOTOR_ID, &frame);
            if (send_frame(&frame) != 0) {
                enter_fault(GIMBAL_FAULT_CAN);
                return;
            }
            motor_build_enable(GIMBAL_PITCH_MOTOR_ID, &frame);
            if (send_frame(&frame) != 0 || send_zero_torque_both() != 0) {
                enter_fault(GIMBAL_FAULT_CAN);
                return;
            }
            g_calibration_phase = 1U;
            g_calibration_step_tick = now;
        } else if (g_calibration_phase == 1U &&
                   now - g_calibration_step_tick >=
                       ms_to_ticks(GIMBAL_CALIBRATION_KEEPALIVE_MS)) {
            if (send_zero_torque_both() != 0) {
                enter_fault(GIMBAL_FAULT_CAN);
                return;
            }
            g_calibration_step_tick = now;
        }
    }

    if (g_telemetry.state == GIMBAL_STATE_DISABLED ||
        g_telemetry.state == GIMBAL_STATE_FAULT) {
        return;
    }
    if (g_telemetry.feedback_valid_mask != 0x03U) {
        uint8_t fault = GIMBAL_FAULT_NONE;
        if ((g_telemetry.feedback_valid_mask & 0x01U) == 0U) {
            fault |= GIMBAL_FAULT_YAW_FEEDBACK;
        }
        if ((g_telemetry.feedback_valid_mask & 0x02U) == 0U) {
            fault |= GIMBAL_FAULT_PITCH_FEEDBACK;
        }
        enter_fault(fault);
        return;
    }
    if (actual_position_exceeded_limits()) {
        enter_fault(GIMBAL_FAULT_LIMIT_EXCEEDED);
        return;
    }

    if (g_telemetry.state == GIMBAL_STATE_STARTING) {
        if (now - g_state_tick < ms_to_ticks(GIMBAL_START_DELAY_MS)) {
            return;
        }
        if (g_start_phase == 0U) {
            motor_build_enable(GIMBAL_YAW_MOTOR_ID, &frame);
            if (send_frame(&frame) != 0) {
                enter_fault(GIMBAL_FAULT_CAN);
                return;
            }
            motor_build_enable(GIMBAL_PITCH_MOTOR_ID, &frame);
            if (send_frame(&frame) != 0) {
                enter_fault(GIMBAL_FAULT_CAN);
                return;
            }
            g_start_phase = 1U;
            g_state_tick = now;
            return;
        }
        if (send_position_both(0, 0, g_home_speed_rpm,
                               g_home_torque_percent) != 0) {
            enter_fault(GIMBAL_FAULT_CAN);
            return;
        }
        g_telemetry.state = GIMBAL_STATE_HOMING;
        g_state_tick = now;
        g_position_refresh_tick = now;
        return;
    }

    if (g_telemetry.state == GIMBAL_STATE_HOMING) {
        int yaw_error = g_telemetry.yaw_position_x100_deg < 0 ?
            -g_telemetry.yaw_position_x100_deg :
             g_telemetry.yaw_position_x100_deg;
        int pitch_error = g_telemetry.pitch_position_x100_deg < 0 ?
            -g_telemetry.pitch_position_x100_deg :
             g_telemetry.pitch_position_x100_deg;
        int yaw_speed = g_telemetry.yaw_speed_rpm < 0 ?
            -(int)g_telemetry.yaw_speed_rpm : (int)g_telemetry.yaw_speed_rpm;
        int pitch_speed = g_telemetry.pitch_speed_rpm < 0 ?
            -(int)g_telemetry.pitch_speed_rpm :
             (int)g_telemetry.pitch_speed_rpm;

        if (now - g_position_refresh_tick >=
            ms_to_ticks(GIMBAL_POSITION_REFRESH_MS)) {
            if (send_position_both(0, 0, g_home_speed_rpm,
                                   g_home_torque_percent) != 0) {
                enter_fault(GIMBAL_FAULT_CAN);
                return;
            }
            g_position_refresh_tick = now;
        }

        if (yaw_error <= GIMBAL_HOME_TOLERANCE_X100_DEG &&
            pitch_error <= GIMBAL_HOME_TOLERANCE_X100_DEG &&
            yaw_speed <= GIMBAL_HOME_SPEED_TOLERANCE_RPM &&
            pitch_speed <= GIMBAL_HOME_SPEED_TOLERANCE_RPM) {
            if (++g_home_settle_samples >= GIMBAL_HOME_SETTLE_SAMPLES) {
                g_telemetry.state = GIMBAL_STATE_ACTIVE;
                g_telemetry.command_speed_rpm = GIMBAL_DEFAULT_SPEED_RPM;
                g_target_tick = now;
                g_motion_deadline_tick = 0U;
            }
        } else {
            g_home_settle_samples = 0U;
        }
        if (now - g_state_tick > ms_to_ticks(GIMBAL_HOME_TIMEOUT_MS)) {
            enter_fault(GIMBAL_FAULT_HOME_TIMEOUT);
        }
        return;
    }

    if (g_telemetry.state == GIMBAL_STATE_RETURNING) {
        int32_t pitch_error = abs_i32(
            g_telemetry.startup_pitch_x100_deg -
            g_telemetry.pitch_position_x100_deg);
        int yaw_speed = g_telemetry.yaw_speed_rpm < 0 ?
            -(int)g_telemetry.yaw_speed_rpm :
             (int)g_telemetry.yaw_speed_rpm;
        int pitch_speed = g_telemetry.pitch_speed_rpm < 0 ?
            -(int)g_telemetry.pitch_speed_rpm :
             (int)g_telemetry.pitch_speed_rpm;

        if (now - g_position_refresh_tick >=
            ms_to_ticks(GIMBAL_POSITION_REFRESH_MS)) {
            if (send_position_both(g_telemetry.yaw_target_x100_deg,
                                   g_telemetry.pitch_target_x100_deg,
                                   g_return_speed_rpm,
                                   g_return_torque_percent) != 0) {
                enter_fault(GIMBAL_FAULT_CAN);
                return;
            }
            g_position_refresh_tick = now;
        }
        if (pitch_error <= GIMBAL_HOME_TOLERANCE_X100_DEG &&
            yaw_speed <= GIMBAL_HOME_SPEED_TOLERANCE_RPM &&
            pitch_speed <= GIMBAL_HOME_SPEED_TOLERANCE_RPM) {
            if (++g_return_settle_samples >= GIMBAL_HOME_SETTLE_SAMPLES) {
                g_telemetry.yaw_target_x100_deg =
                    g_telemetry.yaw_position_x100_deg;
                g_telemetry.pitch_target_x100_deg =
                    g_telemetry.pitch_position_x100_deg;
                g_stop_torque_percent = g_return_torque_percent;
                g_stop_step_tick = now;
                g_motion_deadline_tick = 0U;
                g_telemetry.state = GIMBAL_STATE_STOPPING;
                if (send_position_both(
                        g_telemetry.yaw_target_x100_deg,
                        g_telemetry.pitch_target_x100_deg,
                        g_return_speed_rpm,
                        g_stop_torque_percent) != 0) {
                    enter_fault(GIMBAL_FAULT_CAN);
                }
            }
        } else {
            g_return_settle_samples = 0U;
        }
        if (g_telemetry.state == GIMBAL_STATE_RETURNING &&
            now >= g_motion_deadline_tick) {
            enter_fault(GIMBAL_FAULT_MOTION_TIMEOUT);
        }
        return;
    }

    if (g_telemetry.state == GIMBAL_STATE_STOPPING) {
        if (now - g_stop_step_tick < ms_to_ticks(GIMBAL_STOP_STEP_MS)) {
            return;
        }
        g_stop_step_tick = now;
        if (g_stop_torque_percent <= GIMBAL_STOP_TORQUE_STEP_PERCENT) {
            g_stop_torque_percent = 0U;
            if (send_idle_both() != 0) {
                enter_fault(GIMBAL_FAULT_CAN);
                return;
            }
            g_telemetry.state = GIMBAL_STATE_DISABLED;
            g_telemetry.command_torque_percent = 0U;
            return;
        }
        g_stop_torque_percent -= GIMBAL_STOP_TORQUE_STEP_PERCENT;
        if (send_position_both(g_telemetry.yaw_target_x100_deg,
                               g_telemetry.pitch_target_x100_deg,
                               g_return_speed_rpm,
                               g_stop_torque_percent) != 0) {
            enter_fault(GIMBAL_FAULT_CAN);
        }
        return;
    }

    if (g_telemetry.state == GIMBAL_STATE_ACTIVE &&
        now - g_position_refresh_tick >=
            ms_to_ticks(GIMBAL_POSITION_REFRESH_MS)) {
        if (send_position_both(g_telemetry.yaw_target_x100_deg,
                               g_telemetry.pitch_target_x100_deg,
                               g_telemetry.command_speed_rpm,
                               g_telemetry.command_torque_percent) != 0) {
            enter_fault(GIMBAL_FAULT_CAN);
            return;
        }
        g_position_refresh_tick = now;
    }

    if (g_telemetry.state == GIMBAL_STATE_ACTIVE &&
        g_motion_deadline_tick != 0U) {
        int32_t yaw_error = abs_i32(g_telemetry.yaw_target_x100_deg -
                                    g_telemetry.yaw_position_x100_deg);
        int32_t pitch_error = abs_i32(g_telemetry.pitch_target_x100_deg -
                                      g_telemetry.pitch_position_x100_deg);
        if (yaw_error <= GIMBAL_TARGET_TOLERANCE_X100_DEG &&
            pitch_error <= GIMBAL_TARGET_TOLERANCE_X100_DEG) {
            g_motion_deadline_tick = 0U;
        } else if (now >= g_motion_deadline_tick) {
            enter_fault(GIMBAL_FAULT_MOTION_TIMEOUT);
            return;
        }
    }

    if (g_telemetry.state == GIMBAL_STATE_ACTIVE &&
        g_target_timeout_ms != 0U) {
        uint32_t elapsed_ms = ticks_to_ms(now - g_target_tick);
        if (elapsed_ms >= g_target_timeout_ms) {
            g_telemetry.command_timeout_remaining_ms = 0U;
            g_telemetry.fault |= GIMBAL_FAULT_COMMAND_TIMEOUT;
            (void)gimbal_control_disable();
        } else {
            g_telemetry.command_timeout_remaining_ms =
                g_target_timeout_ms - elapsed_ms;
        }
    } else {
        g_telemetry.command_timeout_remaining_ms = 0U;
    }
}

void gimbal_control_shutdown(void)
{
    gimbal_control_emergency_stop();
}

const GimbalTelemetry *gimbal_control_get_telemetry(void)
{
    return &g_telemetry;
}
