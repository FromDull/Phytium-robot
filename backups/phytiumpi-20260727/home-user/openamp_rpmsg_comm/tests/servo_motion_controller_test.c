#include "servo_motion_controller.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define TEST_TIMER_HZ 1000000U

static uint64_t g_now;
static unsigned int g_write_count;
static unsigned int g_disable_count;
static unsigned int g_single_write_count;
static uint8_t g_single_servo_id;
static PhytiumServoDebugState g_debug;

uint64_t GenericTimerRead(uint32_t timer_id)
{
    (void)timer_id;
    return g_now;
}

uint64_t GenericTimerFrequecy(void)
{
    return TEST_TIMER_HZ;
}

const PhytiumServoDebugState *phytium_servo_get_debug_state(void)
{
    return &g_debug;
}

int phytium_servo_set_all_x10(
    const uint16_t angle_x10_deg[PHYTIUM_SERVO_NUM])
{
    ++g_write_count;
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        g_debug.angle_x10_deg[i] = angle_x10_deg[i];
        g_debug.angle_deg[i] = angle_x10_deg[i] / 10U;
        g_debug.pulse_us[i] = (uint16_t)(500U +
            (2000U * angle_x10_deg[i]) / 1800U);
    }
    return 0;
}

int phytium_servo_enable_single_x10(uint8_t servo_id,
                                    uint16_t angle_x10_deg)
{
    ++g_single_write_count;
    g_single_servo_id = servo_id;
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        g_debug.pulse_us[i] = 0U;
    }
    g_debug.angle_x10_deg[servo_id] = angle_x10_deg;
    g_debug.angle_deg[servo_id] = angle_x10_deg / 10U;
    g_debug.pulse_us[servo_id] = (uint16_t)(500U +
        (2000U * angle_x10_deg) / 1800U);
    return 0;
}

void phytium_servo_disable_outputs(void)
{
    ++g_disable_count;
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        g_debug.pulse_us[i] = 0U;
    }
}

static void advance_ms(uint32_t milliseconds)
{
    g_now += (uint64_t)milliseconds * TEST_TIMER_HZ / 1000U;
}

static void reset_fixture(void)
{
    memset(&g_debug, 0, sizeof(g_debug));
    g_now = 1000U;
    g_write_count = 0U;
    g_disable_count = 0U;
    g_single_write_count = 0U;
    g_single_servo_id = 0xffU;
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        static const uint16_t safe_raw[PHYTIUM_SERVO_NUM] = {
            450U, 1350U, 450U, 1350U
        };
        g_debug.angle_deg[i] = safe_raw[i] / 10U;
        g_debug.angle_x10_deg[i] = safe_raw[i];
        g_debug.pulse_us[i] = 0U;
    }
    assert(servo_motion_init() == 0);
}

static void test_synchronized_interpolation(void)
{
    const uint16_t adopted[PHYTIUM_SERVO_NUM] = {
        450U, 1350U, 450U, 1350U
    };
    const uint16_t target[PHYTIUM_SERVO_NUM] = {1000U, 800U, 1100U, 700U};
    const ServoMotionTelemetry *telemetry;

    reset_fixture();
    assert(servo_motion_get_telemetry()->state == SERVO_MOTION_UNARMED);
    assert(servo_motion_start(target, 1000U) == SERVO_MOTION_NOT_ARMED);
    assert(g_write_count == 0U);
    assert(servo_motion_enable_at_target(adopted) == SERVO_MOTION_OK);
    assert(g_write_count == 1U);
    assert(servo_motion_get_telemetry()->state == SERVO_MOTION_STARTING);
    assert(servo_motion_get_telemetry()->remaining_ms == 3000U);
    assert(servo_motion_start(target, 1000U) == SERVO_MOTION_BUSY);
    advance_ms(3000U);
    servo_motion_poll();
    assert(servo_motion_get_telemetry()->state == SERVO_MOTION_IDLE);
    assert(servo_motion_start(target, 1000U) == SERVO_MOTION_OK);
    assert(servo_motion_start(target, 1000U) == SERVO_MOTION_BUSY);
    servo_motion_poll();
    assert(g_write_count == 2U);

    advance_ms(500U);
    servo_motion_poll();
    telemetry = servo_motion_get_telemetry();
    assert(telemetry->current_angle_x10_deg[0] == 725U);
    assert(telemetry->current_angle_x10_deg[1] == 1075U);
    assert(telemetry->current_angle_x10_deg[2] == 775U);
    assert(telemetry->current_angle_x10_deg[3] == 1025U);
    assert(telemetry->remaining_ms == 500U);

    advance_ms(500U);
    servo_motion_poll();
    telemetry = servo_motion_get_telemetry();
    assert(telemetry->state == SERVO_MOTION_IDLE);
    assert(telemetry->remaining_ms == 0U);
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        assert(telemetry->current_angle_x10_deg[i] == target[i]);
    }
}

static void test_validation_and_stop(void)
{
    const uint16_t valid[PHYTIUM_SERVO_NUM] = {
        450U, 1350U, 450U, 1350U
    };
    const uint16_t invalid[PHYTIUM_SERVO_NUM] = {
        450U, 1801U, 450U, 1350U
    };

    reset_fixture();
    assert(servo_motion_start(valid, 99U) == SERVO_MOTION_INVALID);
    assert(servo_motion_start(invalid, 1000U) == SERVO_MOTION_INVALID);
    assert(servo_motion_start(valid, 1000U) == SERVO_MOTION_NOT_ARMED);
    assert(servo_motion_enable_at_target(valid) == SERVO_MOTION_OK);
    advance_ms(3000U);
    servo_motion_poll();
    assert(servo_motion_start(valid, 1000U) == SERVO_MOTION_OK);
    servo_motion_stop();
    assert(servo_motion_get_telemetry()->state == SERVO_MOTION_UNARMED);
    assert(servo_motion_get_telemetry()->remaining_ms == 0U);
    assert(servo_motion_start(valid, 1000U) == SERVO_MOTION_NOT_ARMED);
    assert(g_disable_count == 1U);
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        assert(servo_motion_get_telemetry()->pulse_us[i] == 0U);
    }
}

static void test_single_servo_timeout(void)
{
    const ServoMotionTelemetry *telemetry;

    reset_fixture();
    assert(servo_motion_test_one(4U, 450U, 1000U) == SERVO_MOTION_INVALID);
    assert(servo_motion_test_one(2U, 1801U, 1000U) == SERVO_MOTION_INVALID);
    assert(servo_motion_test_one(2U, 450U, 99U) == SERVO_MOTION_INVALID);
    assert(servo_motion_test_one(2U, 450U, 1000U) == SERVO_MOTION_OK);
    assert(g_single_write_count == 1U);
    assert(g_single_servo_id == 2U);
    telemetry = servo_motion_get_telemetry();
    assert(telemetry->state == SERVO_MOTION_TESTING);
    assert(telemetry->pulse_us[0] == 0U);
    assert(telemetry->pulse_us[1] == 0U);
    assert(telemetry->pulse_us[2] == 1000U);
    assert(telemetry->pulse_us[3] == 0U);
    advance_ms(999U);
    servo_motion_poll();
    assert(servo_motion_get_telemetry()->state == SERVO_MOTION_TESTING);
    assert(servo_motion_get_telemetry()->remaining_ms == 1U);
    advance_ms(1U);
    servo_motion_poll();
    assert(servo_motion_get_telemetry()->state == SERVO_MOTION_UNARMED);
    assert(g_disable_count == 1U);
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        assert(servo_motion_get_telemetry()->pulse_us[i] == 0U);
    }
}

int main(void)
{
    test_synchronized_interpolation();
    test_validation_and_stop();
    test_single_servo_timeout();
    puts("servo_motion_controller_test: PASS");
    return 0;
}
