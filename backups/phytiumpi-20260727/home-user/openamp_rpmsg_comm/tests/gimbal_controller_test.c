#include "gimbal_controller.h"
#include "motor_can.h"
#include "phytium_can_port.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define TEST_TIMER_HZ 1000000U

static uint64_t g_now;
static MotorFeedback g_feedback[5];
static MotorCanFrame g_sent[128];
static unsigned int g_sent_count;

uint64_t GenericTimerRead(uint32_t timer_id)
{
    (void)timer_id;
    return g_now;
}

uint64_t GenericTimerFrequecy(void)
{
    return TEST_TIMER_HZ;
}

int phytium_can_send(const MotorCanFrame *frame)
{
    assert(g_sent_count < sizeof(g_sent) / sizeof(g_sent[0]));
    g_sent[g_sent_count++] = *frame;
    return 0;
}

int phytium_can_get_motor_feedback(uint8_t motor_id, MotorFeedback *feedback)
{
    if (motor_id >= 5U || !g_feedback[motor_id].valid) {
        return -1;
    }
    *feedback = g_feedback[motor_id];
    return 0;
}

static void advance_ms(uint32_t milliseconds)
{
    g_now += (uint64_t)milliseconds * TEST_TIMER_HZ / 1000U;
    g_feedback[GIMBAL_YAW_MOTOR_ID].update_tick = g_now;
    g_feedback[GIMBAL_PITCH_MOTOR_ID].update_tick = g_now;
}

static void set_feedback(int32_t yaw_x100_deg, int32_t pitch_x100_deg,
                         int16_t yaw_speed_rpm, int16_t pitch_speed_rpm)
{
    memset(g_feedback, 0, sizeof(g_feedback));
    g_feedback[GIMBAL_YAW_MOTOR_ID].motor_id = GIMBAL_YAW_MOTOR_ID;
    g_feedback[GIMBAL_YAW_MOTOR_ID].position_x100_deg = yaw_x100_deg;
    g_feedback[GIMBAL_YAW_MOTOR_ID].speed_rpm = yaw_speed_rpm;
    g_feedback[GIMBAL_YAW_MOTOR_ID].update_tick = g_now;
    g_feedback[GIMBAL_YAW_MOTOR_ID].valid = 1U;
    g_feedback[GIMBAL_PITCH_MOTOR_ID].motor_id = GIMBAL_PITCH_MOTOR_ID;
    g_feedback[GIMBAL_PITCH_MOTOR_ID].position_x100_deg = pitch_x100_deg;
    g_feedback[GIMBAL_PITCH_MOTOR_ID].speed_rpm = pitch_speed_rpm;
    g_feedback[GIMBAL_PITCH_MOTOR_ID].update_tick = g_now;
    g_feedback[GIMBAL_PITCH_MOTOR_ID].valid = 1U;
}

static GimbalLimits test_limits(void)
{
    GimbalLimits limits = {
        .yaw_min_x100_deg = -6000,
        .yaw_max_x100_deg = 6000,
        .pitch_min_x100_deg = -3000,
        .pitch_max_x100_deg = 2000,
        .valid_mask = GIMBAL_LIMIT_ALL_VALID,
    };
    return limits;
}

static void start_and_finish_homing(uint8_t home_torque_percent)
{
    assert(gimbal_control_enable(home_torque_percent, 5U,
                                 home_torque_percent, 5U) ==
           GIMBAL_STATUS_OK);
    assert(gimbal_control_get_telemetry()->state == GIMBAL_STATE_STARTING);
    assert(gimbal_control_get_telemetry()->command_torque_percent ==
           home_torque_percent);
    advance_ms(20U);
    gimbal_control_poll();
    advance_ms(20U);
    gimbal_control_poll();
    assert(gimbal_control_get_telemetry()->state == GIMBAL_STATE_HOMING);
    for (unsigned int i = 0; i < 10U; ++i) {
        advance_ms(10U);
        gimbal_control_poll();
    }
    assert(gimbal_control_get_telemetry()->state == GIMBAL_STATE_ACTIVE);
}

static void test_limits_and_homing(void)
{
    GimbalLimits limits = test_limits();

    g_now = 1000U;
    g_sent_count = 0U;
    set_feedback(0, 0, 0, 0);
    assert(gimbal_control_init() == 0);
    assert(gimbal_control_enable(10U, 5U, 10U, 5U) ==
           GIMBAL_STATUS_LIMITS_NOT_READY);
    assert(gimbal_control_set_limits(&limits) == GIMBAL_STATUS_OK);
    assert(gimbal_control_enable(4U, 5U, 10U, 5U) ==
           GIMBAL_STATUS_INVALID);
    assert(gimbal_control_enable(81U, 5U, 10U, 5U) ==
           GIMBAL_STATUS_INVALID);
    assert(gimbal_control_enable(10U, 5U, 81U, 5U) ==
           GIMBAL_STATUS_INVALID);
    assert(gimbal_control_enable(10U, 0U, 10U, 5U) ==
           GIMBAL_STATUS_INVALID);
    start_and_finish_homing(15U);
    assert(g_sent[0].id == 0x603U && g_sent[0].data[2] == 0x60U);
    assert(g_sent[1].id == 0x604U && g_sent[1].data[2] == 0x60U);
    assert(g_sent[4].data[0] == 0x25U && g_sent[5].data[0] == 0x25U);
    assert(g_sent[4].data[7] == 15U && g_sent[5].data[7] == 15U);
}

static void test_target_and_limit_fault(void)
{
    unsigned int before = g_sent_count;

    assert(gimbal_control_set_target(1500, -500, 20U, 10U, 0U) ==
           GIMBAL_STATUS_OK);
    assert(g_sent_count == before + 2U);
    before = g_sent_count;
    assert(gimbal_control_set_target(1500, -500, 20U, 80U, 0U) ==
           GIMBAL_STATUS_OK);
    assert(g_sent_count == before + 2U);
    before = g_sent_count;
    assert(gimbal_control_set_target(1500, -500, 20U, 81U, 0U) ==
           GIMBAL_STATUS_INVALID);
    assert(g_sent_count == before);
    before = g_sent_count;
    assert(gimbal_control_set_target(6100, 0, 20U, 10U, 0U) ==
           GIMBAL_STATUS_INVALID);
    assert(g_sent_count == before);

    set_feedback(7101, 0, 0, 0);
    advance_ms(10U);
    gimbal_control_poll();
    assert(gimbal_control_get_telemetry()->state == GIMBAL_STATE_FAULT);
    assert((gimbal_control_get_telemetry()->fault &
            GIMBAL_FAULT_LIMIT_EXCEEDED) != 0U);
    assert(g_sent[g_sent_count - 2U].data[0] == 0x2bU);
    assert(g_sent[g_sent_count - 2U].data[2] == 0xa0U);
}

static void test_calibration(void)
{
    const GimbalTelemetry *telemetry;

    g_now = 1000U;
    g_sent_count = 0U;
    assert(gimbal_control_init() == 0);
    set_feedback(-5000, -2500, 0, 0);
    assert(gimbal_control_calibrate_limit(GIMBAL_AXIS_YAW,
                                          GIMBAL_LIMIT_MIN) ==
           GIMBAL_STATUS_OK);
    assert(gimbal_control_calibrate_limit(GIMBAL_AXIS_PITCH,
                                          GIMBAL_LIMIT_MIN) ==
           GIMBAL_STATUS_OK);
    set_feedback(5500, 1800, 0, 0);
    assert(gimbal_control_calibrate_limit(GIMBAL_AXIS_YAW,
                                          GIMBAL_LIMIT_MAX) ==
           GIMBAL_STATUS_OK);
    assert(gimbal_control_calibrate_limit(GIMBAL_AXIS_PITCH,
                                          GIMBAL_LIMIT_MAX) ==
           GIMBAL_STATUS_OK);
    telemetry = gimbal_control_get_telemetry();
    assert(telemetry->limits_valid_mask == GIMBAL_LIMIT_ALL_VALID);
    assert(telemetry->limits.yaw_min_x100_deg == -5000);
    assert(telemetry->limits.pitch_max_x100_deg == 1800);
}

static void test_calibration_feedback_wakeup(void)
{
    g_now = 1000U;
    g_sent_count = 0U;
    memset(g_feedback, 0, sizeof(g_feedback));
    assert(gimbal_control_init() == 0);
    assert(gimbal_control_calibrate_limit(GIMBAL_AXIS_YAW,
                                          GIMBAL_LIMIT_MIN) ==
           GIMBAL_STATUS_NO_FEEDBACK);
    assert(g_sent_count == 4U);
    assert(g_sent[0].data[0] == 0x2bU && g_sent[0].data[2] == 0x60U);
    assert(g_sent[2].data[0] == 0x2bU && g_sent[2].data[2] == 0x20U);

    advance_ms(20U);
    gimbal_control_poll();
    assert(g_sent_count == 8U);
    assert(g_sent[4].data[0] == 0x2bU && g_sent[4].data[2] == 0xa2U);

    set_feedback(-5000, -2500, 0, 0);
    assert(gimbal_control_calibrate_limit(GIMBAL_AXIS_YAW,
                                          GIMBAL_LIMIT_MIN) ==
           GIMBAL_STATUS_OK);
    assert(gimbal_control_get_telemetry()->limits.yaw_min_x100_deg == -5000);

    advance_ms(30001U);
    gimbal_control_poll();
    assert(g_sent[g_sent_count - 2U].data[2] == 0xa0U);
    assert(g_sent[g_sent_count - 1U].data[2] == 0xa0U);
}

static void test_enable_feedback_wakeup(void)
{
    GimbalLimits limits = test_limits();

    g_now = 1000U;
    g_sent_count = 0U;
    memset(g_feedback, 0, sizeof(g_feedback));
    assert(gimbal_control_init() == 0);
    assert(gimbal_control_set_limits(&limits) == GIMBAL_STATUS_OK);
    assert(gimbal_control_enable(10U, 5U, 10U, 5U) ==
           GIMBAL_STATUS_NO_FEEDBACK);
    assert(g_sent_count == 4U);

    advance_ms(20U);
    gimbal_control_poll();
    assert(g_sent_count == 8U);
    set_feedback(0, 0, 0, 0);
    assert(gimbal_control_enable(10U, 5U, 10U, 5U) == GIMBAL_STATUS_OK);
    assert(gimbal_control_get_telemetry()->state == GIMBAL_STATE_STARTING);
    assert(g_sent_count == 12U);
    assert(g_sent[8].data[2] == 0xa0U);
    assert(g_sent[10].data[2] == 0x60U);
}

static void test_position_command_refresh(void)
{
    GimbalLimits limits = test_limits();
    unsigned int before;

    g_now = 1000U;
    g_sent_count = 0U;
    assert(gimbal_control_init() == 0);
    set_feedback(0, 0, 0, 0);
    assert(gimbal_control_set_limits(&limits) == GIMBAL_STATUS_OK);
    start_and_finish_homing(10U);
    before = g_sent_count;
    advance_ms(50U);
    gimbal_control_poll();
    assert(g_sent_count == before + 2U);
    assert(g_sent[before].data[0] == 0x25U);
    assert(g_sent[before + 1U].data[0] == 0x25U);
}

static void test_controlled_disable(void)
{
    GimbalLimits limits = test_limits();
    unsigned int before;

    g_now = 1000U;
    g_sent_count = 0U;
    assert(gimbal_control_init() == 0);
    set_feedback(0, 1500, 0, 0);
    assert(gimbal_control_set_limits(&limits) == GIMBAL_STATUS_OK);
    assert(gimbal_control_enable(15U, 5U, 20U, 4U) == GIMBAL_STATUS_OK);
    assert(gimbal_control_get_telemetry()->startup_pitch_x100_deg == 1500);
    advance_ms(20U);
    gimbal_control_poll();
    advance_ms(20U);
    set_feedback(0, 0, 0, 0);
    gimbal_control_poll();
    for (unsigned int i = 0; i < 10U; ++i) {
        advance_ms(10U);
        gimbal_control_poll();
    }
    assert(gimbal_control_get_telemetry()->state == GIMBAL_STATE_ACTIVE);
    set_feedback(500, -500, 0, 0);
    advance_ms(10U);
    before = g_sent_count;
    assert(gimbal_control_disable() == GIMBAL_STATUS_OK);
    assert(gimbal_control_get_telemetry()->state == GIMBAL_STATE_RETURNING);
    assert(gimbal_control_get_telemetry()->yaw_target_x100_deg == 500);
    assert(gimbal_control_get_telemetry()->pitch_target_x100_deg == 1500);
    assert(g_sent_count == before + 2U);
    assert(g_sent[before].data[6] == 4U && g_sent[before].data[7] == 20U);

    set_feedback(500, 1500, 0, 0);
    for (unsigned int i = 0; i < 10U; ++i) {
        advance_ms(10U);
        gimbal_control_poll();
    }
    assert(gimbal_control_get_telemetry()->state == GIMBAL_STATE_STOPPING);
    for (unsigned int i = 0; i < 10U; ++i) {
        advance_ms(100U);
        gimbal_control_poll();
    }
    assert(gimbal_control_get_telemetry()->state == GIMBAL_STATE_DISABLED);
    assert(g_sent[g_sent_count - 1U].data[0] == 0x2bU);
    assert(g_sent[g_sent_count - 1U].data[2] == 0xa0U);
}

static void test_motion_timeout(void)
{
    GimbalLimits limits = test_limits();

    g_now = 1000U;
    g_sent_count = 0U;
    assert(gimbal_control_init() == 0);
    set_feedback(0, 0, 0, 0);
    assert(gimbal_control_set_limits(&limits) == GIMBAL_STATUS_OK);
    start_and_finish_homing(10U);
    assert(gimbal_control_set_target(1500, 0, 20U, 10U, 0U) ==
           GIMBAL_STATUS_OK);
    advance_ms(2500U);
    gimbal_control_poll();
    assert(gimbal_control_get_telemetry()->state == GIMBAL_STATE_ACTIVE);
    advance_ms(2501U);
    gimbal_control_poll();
    assert(gimbal_control_get_telemetry()->state == GIMBAL_STATE_FAULT);
    assert((gimbal_control_get_telemetry()->fault &
            GIMBAL_FAULT_MOTION_TIMEOUT) != 0U);
}

static void test_dual_axis_near_target_has_time_to_settle(void)
{
    GimbalLimits limits = test_limits();

    g_now = 1000U;
    g_sent_count = 0U;
    assert(gimbal_control_init() == 0);
    set_feedback(0, 0, 0, 0);
    assert(gimbal_control_set_limits(&limits) == GIMBAL_STATUS_OK);
    start_and_finish_homing(10U);
    assert(gimbal_control_set_target(1000, 1000, 20U, 10U, 0U) ==
           GIMBAL_STATUS_OK);

    /* The old ~2.08 s deadline faulted here even though both axes were close. */
    advance_ms(2100U);
    set_feedback(853, 936, 0, 0);
    gimbal_control_poll();
    assert(gimbal_control_get_telemetry()->state == GIMBAL_STATE_ACTIVE);
    assert(gimbal_control_get_telemetry()->fault == GIMBAL_FAULT_NONE);

    advance_ms(1000U);
    set_feedback(950, 960, 0, 0);
    gimbal_control_poll();
    assert(gimbal_control_get_telemetry()->state == GIMBAL_STATE_ACTIVE);
    assert(gimbal_control_get_telemetry()->fault == GIMBAL_FAULT_NONE);
}

int main(void)
{
    test_limits_and_homing();
    test_target_and_limit_fault();
    test_calibration();
    test_calibration_feedback_wakeup();
    test_enable_feedback_wakeup();
    test_position_command_refresh();
    test_controlled_disable();
    test_motion_timeout();
    test_dual_axis_near_target_has_time_to_settle();
    puts("gimbal_controller_test: PASS");
    return 0;
}
